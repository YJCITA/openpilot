#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import json
import os
import sys
import threading
from collections import OrderedDict
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from queue import Empty, Queue

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

# tools/lib/vidindex.py expects DEBUG to be an integer-like string.
if not str(os.environ.get("DEBUG", "0")).lstrip("-").isdigit():
  os.environ["DEBUG"] = "0"

import cv2
import numpy as np

from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
from openpilot.common.transformations.orientation import euler_from_rot, rot_from_euler
from openpilot.tools.lib.framereader import FrameReader
import cereal.messaging as messaging
from cereal import log
from openpilot.tools.lib.logreader import LogReader

WINDOW_TITLE = "wide overlay debugger"
STATE_FILE = REPO_ROOT / ".ai" / "tools" / "wide_overlay_debugger_state.json"
INF_POINT = np.array([1000.0, 0.0, 0.0], dtype=np.float32)
BG = (20, 20, 20)
PANEL_BG = (0, 0, 0)
TEXT = (230, 230, 230)
SUBTEXT = (180, 180, 180)
NARROW_PATH = (0, 0, 255)
NARROW_LANE = (255, 255, 255)
WIDE_LIVE_PATH = (64, 255, 64)
WIDE_LIVE_LANE = (32, 220, 32)
WIDE_PROBE_PATH = (255, 180, 0)
WIDE_PROBE_LANE = (255, 220, 64)
COMPARE_TILE_W, COMPARE_TILE_H = 640, 360


class ProbeMode(Enum):
  NO_WIDE_ROT = "no_wide_rot"
  INVERT_WIDE_ROT = "invert_wide_rot"
  FCAM_INTRINSICS = "fcam_intrinsics"
  TUNED = "tuned"

  @property
  def label(self) -> str:
    return {
      ProbeMode.NO_WIDE_ROT: "Probe: ecam + no wide rot",
      ProbeMode.INVERT_WIDE_ROT: "Probe: ecam + inverse wide rot",
      ProbeMode.FCAM_INTRINSICS: "Probe: fcam intrinsics + live wide rot",
      ProbeMode.TUNED: "Probe: adjusted live wide",
    }[self]


@dataclass(frozen=True)
class CalibrationState:
  log_mono_time: int
  rpy_calib: np.ndarray
  wide_from_device_euler: np.ndarray
  height: float
  status: str
  valid: bool
  valid_blocks: int
  cal_perc: int
  rpy_calib_spread: np.ndarray


@dataclass(frozen=True)
class SpeedState:
  log_mono_time: int
  v_ego: float


@dataclass(frozen=True)
class Sample:
  sample_idx: int
  log_mono_time: int
  road_frame_id: int
  wide_frame_id: int
  road_video_idx: int
  wide_video_idx: int
  model_msg: object
  calibration: CalibrationState
  v_ego: float


@dataclass(frozen=True)
class CameraContext:
  device_type: str
  sensor: str
  fcam_intrinsics: np.ndarray
  ecam_intrinsics: np.ndarray


@dataclass(frozen=True)
class ProjectionBundle:
  title: str
  video_transform: np.ndarray
  projection_transform: np.ndarray


class BgrFrameCache:
  def __init__(self, video_path: str, cache_size: int = 180):
    self.reader = FrameReader(video_path, pix_fmt="rgb24", cache_size=max(cache_size, 60))
    self.cache_size = cache_size
    self._closed = False
    self._cache: OrderedDict[int, np.ndarray] = OrderedDict()
    self._pending: set[int] = set()
    self._queue: Queue[int] = Queue()
    self._lock = threading.Lock()
    self._stop = threading.Event()
    self._thread = threading.Thread(target=self._worker, daemon=True)
    self._thread.start()

  def _evict_if_needed(self) -> None:
    while len(self._cache) > self.cache_size:
      self._cache.popitem(last=False)

  def _decode_locked(self, frame_idx: int) -> np.ndarray:
    if frame_idx in self._cache:
      frame = self._cache.pop(frame_idx)
      self._cache[frame_idx] = frame
      return frame

    rgb = self.reader.get(frame_idx)
    frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    self._cache[frame_idx] = frame
    self._evict_if_needed()
    return frame

  def get(self, frame_idx: int) -> np.ndarray:
    with self._lock:
      frame = self._decode_locked(frame_idx)
      self._pending.discard(frame_idx)
      return frame.copy()

  def request(self, frame_idx: int) -> None:
    with self._lock:
      if frame_idx in self._cache or frame_idx in self._pending:
        return
      self._pending.add(frame_idx)
    self._queue.put(frame_idx)

  def prefetch(self, frame_indices: list[int]) -> None:
    for frame_idx in frame_indices:
      self.request(frame_idx)

  def _worker(self) -> None:
    while not self._stop.is_set():
      try:
        frame_idx = self._queue.get(timeout=0.1)
      except Empty:
        continue

      try:
        with self._lock:
          self._decode_locked(frame_idx)
          self._pending.discard(frame_idx)
      except Exception:
        with self._lock:
          self._pending.discard(frame_idx)

  def close(self) -> None:
    if self._closed:
      return
    self._closed = True
    self._stop.set()
    self._thread.join(timeout=1.0)


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Local segment viewer for debugging wide-camera overlay angle bias.")
  parser.add_argument(
    "segment_dir",
    nargs="?",
    default="/home/yj/bak/data/comma_data/tesla/20251118_C3L_master_lcc_side_1/2025-11-17--14-36-20--0",
    help="Path to a local segment directory containing rlog.zst, fcamera.hevc, and ecamera.hevc.",
  )
  parser.add_argument("--panel-width", type=int, default=640, help="Width of each image panel.")
  parser.add_argument("--start-index", type=int, default=0, help="Start from this valid model sample index.")
  parser.add_argument("--probe-mode", choices=[m.value for m in ProbeMode], default=ProbeMode.NO_WIDE_ROT.value)
  parser.add_argument("--window-scale", type=float, default=0.9, help="Initial display scale for the rendered canvas in the Qt viewer.")
  parser.add_argument("--playback-fps", type=int, default=20, help="Initial playback rate for the Qt viewer.")
  parser.add_argument("--save", type=str, help="Save the currently rendered frame to this path.")
  parser.add_argument("--export-calibration", type=str, help="Export adjusted calibration in device CalibrationParams byte format to this path.")
  parser.add_argument("--initial-calibration", type=str, help="Load a saved CalibrationParams raw file or exported JSON summary and use it as the initial calibration values.")
  parser.add_argument("--no-window", action="store_true", help="Render once and exit. Requires --save or --export-calibration for useful output.")
  parser.add_argument("--window-backend", choices=("auto", "qt"), default="auto")
  return parser.parse_args()


def _as_np3(vals, default=0.0) -> np.ndarray:
  arr = np.array(list(vals), dtype=np.float32)
  if arr.shape != (3,):
    arr = np.full(3, default, dtype=np.float32)
  return arr


def _latest_before(times: list[int], target: int) -> int:
  idx = bisect.bisect_right(times, target) - 1
  return max(idx, 0)


def _pick_camera_config(device_type: str, sensor: str):
  candidates = [
    (device_type, sensor),
    (device_type, "unknown"),
    ("tici", sensor),
    ("tici", "unknown"),
    ("unknown", sensor),
    ("unknown", "unknown"),
  ]
  for key in candidates:
    if key in DEVICE_CAMERAS:
      return DEVICE_CAMERAS[key]
  raise KeyError(f"Unable to find DEVICE_CAMERAS entry for device={device_type!r}, sensor={sensor!r}")


def load_segment(segment_dir: Path) -> tuple[CameraContext, list[Sample], int, int]:
  rlog_path = segment_dir / "rlog.zst"
  fcamera_path = segment_dir / "fcamera.hevc"
  ecamera_path = segment_dir / "ecamera.hevc"
  for p in (rlog_path, fcamera_path, ecamera_path):
    if not p.exists():
      raise FileNotFoundError(f"Missing required file: {p}")

  road_min_frame = None
  wide_min_frame = None
  device_type = None
  sensor = None
  model_msgs: list[tuple[int, object]] = []
  calibs: list[CalibrationState] = []
  speeds: list[SpeedState] = []

  for msg in LogReader(str(rlog_path), sort_by_time=True):
    which = msg.which()
    if which == "deviceState" and device_type is None:
      device_type = str(msg.deviceState.deviceType)
    elif which == "roadCameraState":
      if sensor is None:
        sensor = str(msg.roadCameraState.sensor)
      if road_min_frame is None:
        road_min_frame = int(msg.roadCameraState.frameId)
    elif which == "wideRoadCameraState" and wide_min_frame is None:
      wide_min_frame = int(msg.wideRoadCameraState.frameId)
    elif which == "liveCalibration":
      calibs.append(CalibrationState(
        log_mono_time=int(msg.logMonoTime),
        rpy_calib=_as_np3(msg.liveCalibration.rpyCalib),
        wide_from_device_euler=_as_np3(msg.liveCalibration.wideFromDeviceEuler),
        height=float(msg.liveCalibration.height[0]) if len(msg.liveCalibration.height) else 1.22,
        status=str(msg.liveCalibration.calStatus),
        valid=bool(msg.valid),
        valid_blocks=int(msg.liveCalibration.validBlocks),
        cal_perc=int(msg.liveCalibration.calPerc),
        rpy_calib_spread=_as_np3(msg.liveCalibration.rpyCalibSpread),
      ))
    elif which == "carState":
      speeds.append(SpeedState(log_mono_time=int(msg.logMonoTime), v_ego=float(msg.carState.vEgo)))
    elif which == "modelV2":
      model_msgs.append((int(msg.logMonoTime), msg))

  device_type = device_type or "tici"
  sensor = sensor or "unknown"
  road_min_frame = 0 if road_min_frame is None else road_min_frame
  wide_min_frame = 0 if wide_min_frame is None else wide_min_frame

  fr_road = FrameReader(str(fcamera_path), pix_fmt="rgb24")
  fr_wide = FrameReader(str(ecamera_path), pix_fmt="rgb24")
  road_frame_count = fr_road.frame_count
  wide_frame_count = fr_wide.frame_count

  if not calibs:
    calibs.append(CalibrationState(0, np.zeros(3, dtype=np.float32), np.zeros(3, dtype=np.float32), 1.22, "unknown", False, 0, 0, np.zeros(3, dtype=np.float32)))
  if not speeds:
    speeds.append(SpeedState(0, 0.0))

  calib_times = [c.log_mono_time for c in calibs]
  speed_times = [s.log_mono_time for s in speeds]

  samples: list[Sample] = []
  for log_mono_time, model_msg in model_msgs:
    road_video_idx = int(model_msg.modelV2.frameId) - road_min_frame
    wide_video_idx = int(model_msg.modelV2.frameIdExtra) - wide_min_frame
    if road_video_idx < 0 or wide_video_idx < 0:
      continue
    if road_video_idx >= road_frame_count or wide_video_idx >= wide_frame_count:
      continue

    calib = calibs[_latest_before(calib_times, log_mono_time)]
    speed = speeds[_latest_before(speed_times, log_mono_time)]
    samples.append(Sample(
      sample_idx=len(samples),
      log_mono_time=log_mono_time,
      road_frame_id=int(model_msg.modelV2.frameId),
      wide_frame_id=int(model_msg.modelV2.frameIdExtra),
      road_video_idx=road_video_idx,
      wide_video_idx=wide_video_idx,
      model_msg=model_msg.modelV2,
      calibration=calib,
      v_ego=speed.v_ego,
    ))

  cam = _pick_camera_config(device_type, sensor)
  camera_context = CameraContext(
    device_type=device_type,
    sensor=sensor,
    fcam_intrinsics=cam.fcam.intrinsics.astype(np.float32),
    ecam_intrinsics=cam.ecam.intrinsics.astype(np.float32),
  )
  return camera_context, samples, road_frame_count, wide_frame_count


def compute_view_from_calib(rpy_calib: np.ndarray, calib_offset_euler: np.ndarray | None = None) -> np.ndarray:
  offset = np.zeros(3, dtype=np.float32) if calib_offset_euler is None else calib_offset_euler
  device_from_calib = rot_from_euler(offset) @ rot_from_euler(rpy_calib)
  return (view_frame_from_device_frame @ device_from_calib).astype(np.float32)


def compute_view_from_wide_calib(rpy_calib: np.ndarray, wide_from_device_euler: np.ndarray,
                                 calib_offset_euler: np.ndarray | None = None,
                                 wide_offset_euler: np.ndarray | None = None) -> np.ndarray:
  calib_offset = np.zeros(3, dtype=np.float32) if calib_offset_euler is None else calib_offset_euler
  wide_offset = np.zeros(3, dtype=np.float32) if wide_offset_euler is None else wide_offset_euler
  device_from_calib = rot_from_euler(calib_offset) @ rot_from_euler(rpy_calib)
  wide_from_device = rot_from_euler(wide_offset) @ rot_from_euler(wide_from_device_euler)
  return (view_frame_from_device_frame @ wide_from_device @ device_from_calib).astype(np.float32)


def ui_zoom(device_type: str, is_wide: bool, v_ego: float) -> float:
  if device_type == "mici":
    if is_wide:
      return 0.7 * 1.5
    return float(np.interp(v_ego, [10.0, 30.0], [0.8, 1.0]))
  return 2.0 if is_wide else 1.1


def compute_ui_transforms(panel_size: tuple[int, int], intrinsics: np.ndarray, calibration: np.ndarray,
                          is_wide: bool, device_type: str, v_ego: float) -> tuple[np.ndarray, np.ndarray]:
  panel_w, panel_h = panel_size
  zoom = ui_zoom(device_type, is_wide, v_ego)
  calib_transform = intrinsics @ calibration
  kep = calib_transform @ INF_POINT
  cx, cy = intrinsics[0, 2], intrinsics[1, 2]
  margin = 5.0
  max_x_offset = cx * zoom - panel_w / 2.0 - margin
  max_y_offset = cy * zoom - panel_h / 2.0 - margin
  mici_y_offset = 20.0 if device_type == "mici" and is_wide else 0.0

  try:
    if abs(kep[2]) > 1e-6:
      x_offset = np.clip((kep[0] / kep[2] - cx) * zoom, -max_x_offset, max_x_offset)
      y_offset = np.clip((kep[1] / kep[2] - cy) * zoom + mici_y_offset, -max_y_offset, max_y_offset)
    else:
      x_offset = 0.0
      y_offset = 0.0
  except (ZeroDivisionError, OverflowError, FloatingPointError):
    x_offset = 0.0
    y_offset = 0.0

  video_transform = np.array([
    [zoom, 0.0, (panel_w / 2.0 - x_offset) - (cx * zoom)],
    [0.0, zoom, (panel_h / 2.0 - y_offset) - (cy * zoom)],
    [0.0, 0.0, 1.0],
  ], dtype=np.float32)
  projection_transform = (video_transform @ calib_transform).astype(np.float32)
  return video_transform, projection_transform


def render_panel(frame_bgr: np.ndarray, panel_size: tuple[int, int], video_transform: np.ndarray) -> np.ndarray:
  panel_w, panel_h = panel_size
  return cv2.warpAffine(
    frame_bgr,
    video_transform[:2],
    (panel_w, panel_h),
    flags=cv2.INTER_LINEAR,
    borderMode=cv2.BORDER_CONSTANT,
    borderValue=PANEL_BG,
  )


def _project_line(points: np.ndarray, projection_transform: np.ndarray, panel_size: tuple[int, int]) -> np.ndarray:
  if points.size == 0:
    return np.empty((0, 2), dtype=np.int32)
  pts = points[points[:, 0] >= 0]
  if pts.size == 0:
    return np.empty((0, 2), dtype=np.int32)

  proj = projection_transform @ pts.T
  valid = np.abs(proj[2]) > 1e-6
  if not np.any(valid):
    return np.empty((0, 2), dtype=np.int32)

  screen = (proj[:2, valid] / proj[2, valid]).T
  panel_w, panel_h = panel_size
  inside = (
    (screen[:, 0] >= -50) & (screen[:, 0] <= panel_w + 50) &
    (screen[:, 1] >= -50) & (screen[:, 1] <= panel_h + 50)
  )
  screen = screen[inside]
  if screen.shape[0] < 2:
    return np.empty((0, 2), dtype=np.int32)
  return np.round(screen).astype(np.int32)


def _draw_line(image: np.ndarray, points: np.ndarray, color: tuple[int, int, int], thickness: int) -> None:
  if points.shape[0] < 2:
    return
  cv2.polylines(image, [points.reshape(-1, 1, 2)], False, color, thickness, lineType=cv2.LINE_AA)


def _model_line(msg) -> np.ndarray:
  return np.array([msg.x, msg.y, msg.z], dtype=np.float32).T


def draw_model_overlay(image: np.ndarray, model_msg, projection_transform: np.ndarray,
                       panel_size: tuple[int, int], lane_color: tuple[int, int, int],
                       path_color: tuple[int, int, int]) -> None:
  path = _project_line(_model_line(model_msg.position), projection_transform, panel_size)
  _draw_line(image, path, path_color, 3)
  lane_left = _project_line(_model_line(model_msg.laneLines[1]), projection_transform, panel_size)
  lane_right = _project_line(_model_line(model_msg.laneLines[2]), projection_transform, panel_size)
  _draw_line(image, lane_left, lane_color, 2)
  _draw_line(image, lane_right, lane_color, 2)


def _put_text(image: np.ndarray, text: str, org: tuple[int, int], color=TEXT, scale=0.55, thickness=1) -> None:
  cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def deg_str(arr: np.ndarray) -> str:
  deg = np.degrees(arr)
  return f"[{deg[0]:+.2f}, {deg[1]:+.2f}, {deg[2]:+.2f}]"


def compose_euler(base_euler: np.ndarray, offset_euler: np.ndarray) -> np.ndarray:
  return euler_from_rot(rot_from_euler(offset_euler) @ rot_from_euler(base_euler)).astype(np.float32)


def build_calibration_params_export(sample: Sample, calib_offset_euler: np.ndarray, wide_offset_euler: np.ndarray) -> tuple[bytes, dict]:
  adjusted_rpy = compose_euler(sample.calibration.rpy_calib, calib_offset_euler)
  adjusted_wide = compose_euler(sample.calibration.wide_from_device_euler, wide_offset_euler)
  valid_blocks = max(sample.calibration.valid_blocks, 5)
  msg = messaging.new_message('liveCalibration')
  msg.valid = bool(sample.calibration.valid or valid_blocks >= 5)
  live_cal = msg.liveCalibration
  live_cal.validBlocks = valid_blocks
  live_cal.calStatus = getattr(log.LiveCalibrationData.Status, sample.calibration.status, log.LiveCalibrationData.Status.calibrated)
  live_cal.calPerc = max(sample.calibration.cal_perc, 100 if valid_blocks >= 5 else sample.calibration.cal_perc)
  live_cal.rpyCalib = adjusted_rpy.tolist()
  live_cal.rpyCalibSpread = sample.calibration.rpy_calib_spread.tolist()
  live_cal.wideFromDeviceEuler = adjusted_wide.tolist()
  live_cal.height = [float(sample.calibration.height)]

  summary = {
    'param_key': 'CalibrationParams',
    'param_type': 'BYTES',
    'device_param_path': '/data/params/d/CalibrationParams',
    'source_sample_index': sample.sample_idx,
    'source_log_mono_time': sample.log_mono_time,
    'valid': bool(msg.valid),
    'validBlocks': int(live_cal.validBlocks),
    'calStatus': str(live_cal.calStatus),
    'calPerc': int(live_cal.calPerc),
    'rpyCalib': adjusted_rpy.tolist(),
    'rpyCalibDeg': np.degrees(adjusted_rpy).tolist(),
    'rpyCalibSpread': sample.calibration.rpy_calib_spread.tolist(),
    'wideFromDeviceEuler': adjusted_wide.tolist(),
    'wideFromDeviceEulerDeg': np.degrees(adjusted_wide).tolist(),
    'height': [float(sample.calibration.height)],
    'appliedCalibOffsetDeg': np.degrees(calib_offset_euler).tolist(),
    'appliedWideOffsetDeg': np.degrees(wide_offset_euler).tolist(),
  }
  return msg.to_bytes(), summary


def export_calibration_params_files(path_str: str, sample: Sample, calib_offset_euler: np.ndarray, wide_offset_euler: np.ndarray) -> tuple[Path, Path]:
  raw_path = Path(path_str).expanduser()
  raw_path.parent.mkdir(parents=True, exist_ok=True)
  payload, summary = build_calibration_params_export(sample, calib_offset_euler, wide_offset_euler)
  raw_path.write_bytes(payload)
  if raw_path.suffix:
    json_path = raw_path.with_suffix(raw_path.suffix + '.json')
  else:
    json_path = raw_path.with_name(raw_path.name + '.json')
  json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=True) + '\n')
  return raw_path, json_path


def load_calibration_source(path_str: str) -> CalibrationState:
  src_path = Path(path_str).expanduser().resolve()
  if not src_path.exists():
    raise FileNotFoundError(src_path)

  if src_path.suffix.lower() == '.json':
    data = json.loads(src_path.read_text())
    return CalibrationState(
      log_mono_time=int(data.get('source_log_mono_time', 0)),
      rpy_calib=_as_np3(data.get('rpyCalib', [0.0, 0.0, 0.0])),
      wide_from_device_euler=_as_np3(data.get('wideFromDeviceEuler', [0.0, 0.0, 0.0])),
      height=float((data.get('height') or [1.22])[0]),
      status=str(data.get('calStatus', 'calibrated')),
      valid=bool(data.get('valid', True)),
      valid_blocks=int(data.get('validBlocks', 50)),
      cal_perc=int(data.get('calPerc', 100)),
      rpy_calib_spread=_as_np3(data.get('rpyCalibSpread', [0.0, 0.0, 0.0])),
    )

  with log.Event.from_bytes(src_path.read_bytes()) as msg:
    if msg.which() != 'liveCalibration':
      raise ValueError(f'Expected liveCalibration event, got {msg.which()}')
    lc = msg.liveCalibration
    return CalibrationState(
      log_mono_time=int(getattr(msg, 'logMonoTime', 0)),
      rpy_calib=_as_np3(lc.rpyCalib),
      wide_from_device_euler=_as_np3(lc.wideFromDeviceEuler),
      height=float(lc.height[0]) if len(lc.height) else 1.22,
      status=str(lc.calStatus),
      valid=bool(msg.valid),
      valid_blocks=int(lc.validBlocks),
      cal_perc=int(lc.calPerc),
      rpy_calib_spread=_as_np3(lc.rpyCalibSpread),
    )


def get_warp_matrix_with_camera_rotation(device_from_calib_euler: np.ndarray, intrinsics: np.ndarray,
                                         bigmodel_frame: bool, camera_from_device_euler: np.ndarray | None = None) -> np.ndarray:
  from openpilot.common.transformations.model import calib_from_medmodel, calib_from_sbigmodel

  calib_from_model = calib_from_sbigmodel if bigmodel_frame else calib_from_medmodel
  device_from_calib = rot_from_euler(device_from_calib_euler)
  camera_from_device = np.eye(3, dtype=np.float32) if camera_from_device_euler is None else rot_from_euler(camera_from_device_euler)
  camera_from_calib = intrinsics @ view_frame_from_device_frame @ camera_from_device @ device_from_calib
  return (camera_from_calib @ calib_from_model).astype(np.float32)


def warp_bgr_for_compare(image_bgr: np.ndarray, warp_matrix: np.ndarray) -> np.ndarray:
  from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
  model_w, model_h = MEDMODEL_INPUT_SIZE
  return cv2.warpPerspective(
    image_bgr,
    warp_matrix,
    (model_w, model_h),
    flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR,
    borderMode=cv2.BORDER_CONSTANT,
    borderValue=(0, 0, 0),
  )


def compare_resize_letterbox(img: np.ndarray, title: str, subtitle: str | None = None) -> np.ndarray:
  canvas = np.full((COMPARE_TILE_H, COMPARE_TILE_W, 3), BG, dtype=np.uint8)
  h, w = img.shape[:2]
  scale = min((COMPARE_TILE_W - 20) / w, (COMPARE_TILE_H - 50) / h)
  new_w = max(1, round(w * scale))
  new_h = max(1, round(h * scale))
  resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
  x0 = (COMPARE_TILE_W - new_w) // 2
  y0 = 36 + (COMPARE_TILE_H - 36 - new_h) // 2
  canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
  cv2.putText(canvas, title, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, TEXT, 2, cv2.LINE_AA)
  if subtitle:
    cv2.putText(canvas, subtitle, (12, COMPARE_TILE_H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, SUBTEXT, 1, cv2.LINE_AA)
  return canvas


def build_warp_diff_heatmap(current_warp: np.ndarray, proposed_warp: np.ndarray) -> tuple[np.ndarray, dict]:
  abs_diff = cv2.absdiff(current_warp, proposed_warp)
  gray = cv2.cvtColor(abs_diff, cv2.COLOR_BGR2GRAY)
  heat = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
  metrics = {
    'mean_abs_diff_rgb': abs_diff.mean(axis=(0, 1)).tolist(),
    'mean_abs_diff_gray': float(gray.mean()),
    'max_abs_diff_gray': int(gray.max()),
    'pixels_gt_8': float((gray > 8).mean()),
    'pixels_gt_16': float((gray > 16).mean()),
    'pixels_gt_32': float((gray > 32).mean()),
  }
  return heat, metrics


def render_warp_compare_canvas(sample: Sample, camera: CameraContext, road_bgr: np.ndarray, wide_bgr: np.ndarray,
                               calib_offset_euler: np.ndarray, wide_offset_euler: np.ndarray) -> tuple[np.ndarray, dict]:
  adjusted_rpy = compose_euler(sample.calibration.rpy_calib, calib_offset_euler)
  adjusted_wide = compose_euler(sample.calibration.wide_from_device_euler, wide_offset_euler)

  narrow_warp_matrix = get_warp_matrix_with_camera_rotation(adjusted_rpy, camera.fcam_intrinsics, False, None)
  wide_current_warp_matrix = get_warp_matrix_with_camera_rotation(adjusted_rpy, camera.ecam_intrinsics, True, None)
  wide_proposed_warp_matrix = get_warp_matrix_with_camera_rotation(adjusted_rpy, camera.ecam_intrinsics, True, adjusted_wide)

  narrow_warp = warp_bgr_for_compare(road_bgr, narrow_warp_matrix)
  wide_current_warp = warp_bgr_for_compare(wide_bgr, wide_current_warp_matrix)
  wide_proposed_warp = warp_bgr_for_compare(wide_bgr, wide_proposed_warp_matrix)
  diff_heatmap, diff_metrics = build_warp_diff_heatmap(wide_current_warp, wide_proposed_warp)

  top = cv2.hconcat([
    compare_resize_letterbox(road_bgr, 'Narrow raw', f'frameId/videoIdx: {sample.road_frame_id}/{sample.road_video_idx}'),
    compare_resize_letterbox(wide_bgr, 'Wide raw', f'frameId/videoIdx: {sample.wide_frame_id}/{sample.wide_video_idx}'),
    compare_resize_letterbox(narrow_warp, 'Narrow warp (current)', f'rpyCalib deg: {deg_str(adjusted_rpy)}'),
  ])
  bottom = cv2.hconcat([
    compare_resize_letterbox(wide_current_warp, 'Wide warp (current)', 'uses rpyCalib only'),
    compare_resize_letterbox(wide_proposed_warp, 'Wide warp (proposed)', f'uses wideFromDevice deg: {deg_str(adjusted_wide)}'),
    compare_resize_letterbox(diff_heatmap, 'Abs diff heatmap', f"mean={diff_metrics['mean_abs_diff_gray']:.2f}, gt16={diff_metrics['pixels_gt_16']*100:.1f}%"),
  ])
  canvas = cv2.vconcat([top, bottom])
  metrics = {
    'sample_index': sample.sample_idx,
    'sample_log_mono_time': sample.log_mono_time,
    'sample_road_frame_id': sample.road_frame_id,
    'sample_wide_frame_id': sample.wide_frame_id,
    'adjusted_rpy_deg': np.degrees(adjusted_rpy).tolist(),
    'adjusted_wide_from_device_deg': np.degrees(adjusted_wide).tolist(),
    'diff_metrics': diff_metrics,
  }
  return canvas, metrics


def build_live_bundles(sample: Sample, camera: CameraContext, panel_size: tuple[int, int],
                       calib_offset_euler: np.ndarray, wide_offset_euler: np.ndarray) -> tuple[ProjectionBundle, ProjectionBundle]:
  narrow_calibration = compute_view_from_calib(sample.calibration.rpy_calib, calib_offset_euler)
  wide_calibration = compute_view_from_wide_calib(
    sample.calibration.rpy_calib,
    sample.calibration.wide_from_device_euler,
    calib_offset_euler,
    wide_offset_euler,
  )

  narrow_video, narrow_projection = compute_ui_transforms(
    panel_size, camera.fcam_intrinsics, narrow_calibration, False, camera.device_type, sample.v_ego,
  )
  wide_video, wide_projection = compute_ui_transforms(
    panel_size, camera.ecam_intrinsics, wide_calibration, True, camera.device_type, sample.v_ego,
  )
  return (
    ProjectionBundle("Narrow reference", narrow_video, narrow_projection),
    ProjectionBundle("Wide live", wide_video, wide_projection),
  )


def build_probe_bundle(mode: ProbeMode, sample: Sample, camera: CameraContext, panel_size: tuple[int, int],
                       calib_offset_euler: np.ndarray, wide_offset_euler: np.ndarray) -> ProjectionBundle:
  device_from_calib = rot_from_euler(calib_offset_euler) @ rot_from_euler(sample.calibration.rpy_calib)
  wide_from_device = rot_from_euler(wide_offset_euler) @ rot_from_euler(sample.calibration.wide_from_device_euler)
  intrinsics = camera.ecam_intrinsics
  calibration = view_frame_from_device_frame @ wide_from_device @ device_from_calib
  title = mode.label

  if mode == ProbeMode.NO_WIDE_ROT:
    calibration = view_frame_from_device_frame @ device_from_calib
  elif mode == ProbeMode.INVERT_WIDE_ROT:
    calibration = view_frame_from_device_frame @ wide_from_device.T @ device_from_calib
  elif mode == ProbeMode.FCAM_INTRINSICS:
    intrinsics = camera.fcam_intrinsics
  elif mode == ProbeMode.TUNED:
    calibration = view_frame_from_device_frame @ wide_from_device @ device_from_calib

  video_transform, projection_transform = compute_ui_transforms(
    panel_size, intrinsics, calibration.astype(np.float32), True, camera.device_type, sample.v_ego,
  )
  return ProjectionBundle(title, video_transform, projection_transform)


def compose_canvas(road_bgr: np.ndarray, wide_bgr: np.ndarray, sample: Sample, camera: CameraContext,
                   panel_size: tuple[int, int], probe_mode: ProbeMode,
                   calib_offset_euler: np.ndarray, wide_offset_euler: np.ndarray,
                   total_samples: int) -> np.ndarray:
  panel_w, panel_h = panel_size
  footer_h = 120
  canvas = np.zeros((panel_h + footer_h, panel_w * 3, 3), dtype=np.uint8)
  canvas[:, :] = BG

  narrow_bundle, live_wide_bundle = build_live_bundles(sample, camera, panel_size, calib_offset_euler, wide_offset_euler)
  probe_bundle = build_probe_bundle(probe_mode, sample, camera, panel_size, calib_offset_euler, wide_offset_euler)

  narrow_panel = render_panel(road_bgr, panel_size, narrow_bundle.video_transform)
  live_wide_panel = render_panel(wide_bgr, panel_size, live_wide_bundle.video_transform)
  probe_panel = render_panel(wide_bgr, panel_size, probe_bundle.video_transform)

  draw_model_overlay(narrow_panel, sample.model_msg, narrow_bundle.projection_transform, panel_size, NARROW_LANE, NARROW_PATH)
  draw_model_overlay(live_wide_panel, sample.model_msg, live_wide_bundle.projection_transform, panel_size, WIDE_LIVE_LANE, WIDE_LIVE_PATH)
  draw_model_overlay(probe_panel, sample.model_msg, probe_bundle.projection_transform, panel_size, WIDE_PROBE_LANE, WIDE_PROBE_PATH)

  panels = [narrow_panel, live_wide_panel, probe_panel]
  titles = [narrow_bundle.title, live_wide_bundle.title, probe_bundle.title]
  for i, (panel, title) in enumerate(zip(panels, titles, strict=True)):
    x0 = i * panel_w
    canvas[:panel_h, x0:x0 + panel_w] = panel
    cv2.rectangle(canvas, (x0, 0), (x0 + panel_w - 1, panel_h - 1), (60, 60, 60), 1)
    _put_text(canvas, title, (x0 + 12, 24), scale=0.65, thickness=2)

  y0 = panel_h + 24
  _put_text(canvas, f"sample {sample.sample_idx + 1}/{total_samples}", (16, y0))
  _put_text(canvas, f"road frame/video: {sample.road_frame_id}/{sample.road_video_idx}", (16, y0 + 24), color=SUBTEXT)
  _put_text(canvas, f"wide frame/video: {sample.wide_frame_id}/{sample.wide_video_idx}", (16, y0 + 48), color=SUBTEXT)
  _put_text(canvas, f"speed: {sample.v_ego:.2f} m/s", (16, y0 + 72), color=SUBTEXT)

  x1 = panel_w + 16
  _put_text(canvas, f"rpyCalib deg: {deg_str(sample.calibration.rpy_calib)}", (x1, y0))
  _put_text(canvas, f"wideFromDevice deg: {deg_str(sample.calibration.wide_from_device_euler)}", (x1, y0 + 24), color=SUBTEXT)
  _put_text(canvas, f"calib offset deg: {deg_str(calib_offset_euler)}", (x1, y0 + 48), color=SUBTEXT)
  _put_text(canvas, f"wide offset deg: {deg_str(wide_offset_euler)}", (x1, y0 + 72), color=SUBTEXT)

  x2 = panel_w * 2 + 16
  _put_text(canvas, f"device={camera.device_type} sensor={camera.sensor}", (x2, y0))
  _put_text(canvas, f"probe mode: {probe_mode.value}", (x2, y0 + 24), color=SUBTEXT)
  _put_text(canvas, f"height: {sample.calibration.height:.3f} m", (x2, y0 + 48), color=SUBTEXT)
  _put_text(canvas, "colors: narrow=white/red, live=green, probe=orange", (x2, y0 + 72), color=SUBTEXT)

  return canvas


def save_image(path: str, image: np.ndarray) -> None:
  out_path = Path(path).expanduser()
  out_path.parent.mkdir(parents=True, exist_ok=True)
  if not cv2.imwrite(str(out_path), image):
    raise OSError(f"Failed to save image to {out_path}")
  print(f"saved {out_path}", flush=True)


def load_tool_state() -> dict:
  if not STATE_FILE.exists():
    return {}
  try:
    return json.loads(STATE_FILE.read_text())
  except Exception:
    return {}


def save_tool_state(state: dict) -> None:
  STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
  STATE_FILE.write_text(json.dumps(state, indent=2, ensure_ascii=True) + '\n')


class WideOverlayDebuggerModel:
  def __init__(self, args: argparse.Namespace):
    self.args = args
    self._closed = False
    self.segment_dir = Path(args.segment_dir).expanduser().resolve()
    self.camera, self.samples, self.road_frame_count, self.wide_frame_count = load_segment(self.segment_dir)
    if not self.samples:
      raise RuntimeError("No valid model samples found with both narrow and wide frames available.")

    self.panel_w = int(args.panel_width)
    probe_reader = FrameReader(str(self.segment_dir / "fcamera.hevc"), pix_fmt="rgb24")
    self.panel_h = round(self.panel_w * probe_reader.h / probe_reader.w)
    self.panel_size = (self.panel_w, self.panel_h)
    self.current = int(np.clip(args.start_index, 0, len(self.samples) - 1))
    self.probe_mode = ProbeMode(args.probe_mode)
    self.calib_offset_euler = np.zeros(3, dtype=np.float32)
    self.wide_offset_euler = np.zeros(3, dtype=np.float32)
    self.initial_calibration_override: CalibrationState | None = None
    self.tool_state = load_tool_state()

    if args.initial_calibration:
      self.initial_calibration_override = load_calibration_source(args.initial_calibration)

    self.road_cache = BgrFrameCache(str(self.segment_dir / "fcamera.hevc"), cache_size=220)
    self.wide_cache = BgrFrameCache(str(self.segment_dir / "ecamera.hevc"), cache_size=220)
    self.schedule_prefetch(self.current)

  def close(self) -> None:
    if self._closed:
      return
    self._closed = True
    self.road_cache.close()
    self.wide_cache.close()

  def current_sample(self) -> Sample:
    sample = self.samples[self.current]
    if self.initial_calibration_override is None:
      return sample
    return replace(sample, calibration=self.initial_calibration_override)

  def set_current(self, index: int) -> None:
    self.current = int(np.clip(index, 0, len(self.samples) - 1))
    self.schedule_prefetch(self.current)

  def step(self, delta: int) -> None:
    self.set_current(self.current + delta)

  def set_probe_mode(self, mode: ProbeMode) -> None:
    self.probe_mode = mode

  def set_calib_offset_deg(self, axis: int, value_deg: float) -> None:
    self.calib_offset_euler[axis] = np.radians(value_deg)

  def set_wide_offset_deg(self, axis: int, value_deg: float) -> None:
    self.wide_offset_euler[axis] = np.radians(value_deg)

  def get_calib_offset_deg(self) -> np.ndarray:
    return np.degrees(self.calib_offset_euler)

  def get_wide_offset_deg(self) -> np.ndarray:
    return np.degrees(self.wide_offset_euler)

  def reset_offsets(self) -> None:
    self.calib_offset_euler[:] = 0.0
    self.wide_offset_euler[:] = 0.0

  def load_initial_calibration(self, path_str: str) -> CalibrationState:
    self.initial_calibration_override = load_calibration_source(path_str)
    self.reset_offsets()
    return self.initial_calibration_override

  def clear_initial_calibration(self) -> None:
    self.initial_calibration_override = None
    self.reset_offsets()

  def schedule_prefetch(self, center_index: int) -> None:
    start = max(0, center_index - 3)
    end = min(len(self.samples), center_index + 90)
    road_idxs = [self.samples[i].road_video_idx for i in range(start, end)]
    wide_idxs = [self.samples[i].wide_video_idx for i in range(start, end)]
    self.road_cache.prefetch(road_idxs)
    self.wide_cache.prefetch(wide_idxs)

  def render_current(self) -> np.ndarray:
    sample = self.current_sample()
    road_bgr = self.road_cache.get(sample.road_video_idx)
    wide_bgr = self.wide_cache.get(sample.wide_video_idx)
    return compose_canvas(
      road_bgr,
      wide_bgr,
      sample,
      self.camera,
      self.panel_size,
      self.probe_mode,
      self.calib_offset_euler,
      self.wide_offset_euler,
      len(self.samples),
    )

  def export_current_calibration(self, path_str: str) -> tuple[Path, Path]:
    raw_path, json_path = export_calibration_params_files(path_str, self.current_sample(), self.calib_offset_euler, self.wide_offset_euler)
    self.set_last_path('last_export_path', str(raw_path))
    return raw_path, json_path

  def render_current_warp_compare(self) -> tuple[np.ndarray, dict]:
    sample = self.current_sample()
    road_bgr = self.road_cache.get(sample.road_video_idx)
    wide_bgr = self.wide_cache.get(sample.wide_video_idx)
    return render_warp_compare_canvas(sample, self.camera, road_bgr, wide_bgr, self.calib_offset_euler, self.wide_offset_euler)

  def get_last_path(self, key: str, fallback: str) -> str:
    val = self.tool_state.get(key)
    if isinstance(val, str) and val:
      return val
    return fallback

  def set_last_path(self, key: str, value: str) -> None:
    self.tool_state[key] = value
    save_tool_state(self.tool_state)


def run_qt_viewer(model: WideOverlayDebuggerModel, args: argparse.Namespace) -> int:
  from PyQt5 import QtCore, QtGui, QtWidgets

  class AngleControl(QtWidgets.QWidget):
    valueChanged = QtCore.pyqtSignal(float)

    def __init__(self, label: str, minimum: float = -10.0, maximum: float = 10.0, step: float = 0.05, button_step: float = 0.25):
      super().__init__()
      self._scale = 100
      self._button_step = button_step
      self._syncing = False
      self._minimum = minimum
      self._maximum = maximum

      row = QtWidgets.QHBoxLayout(self)
      row.setContentsMargins(0, 0, 0, 0)
      row.setSpacing(6)

      self.name_label = QtWidgets.QLabel(label)
      self.name_label.setFixedWidth(48)
      row.addWidget(self.name_label)

      self.minus_btn = QtWidgets.QToolButton(text="-")
      self.plus_btn = QtWidgets.QToolButton(text="+")
      self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
      self.slider.setRange(round(minimum * self._scale), round(maximum * self._scale))
      self.slider.setSingleStep(max(1, round(step * self._scale)))
      self.slider.setPageStep(max(1, round(button_step * self._scale)))
      self.spin = QtWidgets.QDoubleSpinBox()
      self.spin.setRange(minimum, maximum)
      self.spin.setSingleStep(step)
      self.spin.setDecimals(2)
      self.spin.setFixedWidth(80)

      row.addWidget(self.minus_btn)
      row.addWidget(self.slider, 1)
      row.addWidget(self.plus_btn)
      row.addWidget(self.spin)

      self.slider.valueChanged.connect(self._slider_changed)
      self.spin.valueChanged.connect(self._spin_changed)
      self.minus_btn.clicked.connect(lambda: self.set_value(self.value() - self._button_step))
      self.plus_btn.clicked.connect(lambda: self.set_value(self.value() + self._button_step))

    def _slider_changed(self, raw: int) -> None:
      if self._syncing:
        return
      self._syncing = True
      val = raw / self._scale
      self.spin.setValue(val)
      self._syncing = False
      self.valueChanged.emit(val)

    def _spin_changed(self, val: float) -> None:
      if self._syncing:
        return
      self._syncing = True
      self.slider.setValue(round(val * self._scale))
      self._syncing = False
      self.valueChanged.emit(val)

    def set_value(self, val: float) -> None:
      val = min(self._maximum, max(self._minimum, val))
      self.spin.setValue(val)

    def value(self) -> float:
      return float(self.spin.value())

  class EulerGroup(QtWidgets.QGroupBox):
    valueChanged = QtCore.pyqtSignal()

    def __init__(self, title: str):
      super().__init__(title)
      layout = QtWidgets.QVBoxLayout(self)
      layout.setContentsMargins(8, 8, 8, 8)
      self.controls = [
        AngleControl("roll"),
        AngleControl("pitch"),
        AngleControl("yaw"),
      ]
      for control in self.controls:
        control.valueChanged.connect(self.valueChanged.emit)
        layout.addWidget(control)

    def values_deg(self) -> np.ndarray:
      return np.array([c.value() for c in self.controls], dtype=np.float32)

    def set_values_deg(self, vals: np.ndarray) -> None:
      for control, val in zip(self.controls, vals, strict=True):
        control.set_value(float(val))

  class WarpCompareDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
      super().__init__(parent)
      self.setWindowTitle('Wide Warp Compare')
      self.resize(1500, 900)
      layout = QtWidgets.QVBoxLayout(self)
      self.image_label = QtWidgets.QLabel(alignment=QtCore.Qt.AlignCenter)
      self.image_label.setBackgroundRole(QtGui.QPalette.Base)
      self.image_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
      self.image_label.setScaledContents(False)
      self.scroll = QtWidgets.QScrollArea()
      self.scroll.setWidget(self.image_label)
      self.scroll.setWidgetResizable(True)
      layout.addWidget(self.scroll, 1)
      self.metrics_text = QtWidgets.QPlainTextEdit()
      self.metrics_text.setReadOnly(True)
      self.metrics_text.setMaximumBlockCount(2000)
      self.metrics_text.setMinimumHeight(180)
      layout.addWidget(self.metrics_text)
      self._base_pixmap: QtGui.QPixmap | None = None

    def _update_pixmap_fit(self) -> None:
      if self._base_pixmap is None:
        return
      viewport = self.scroll.viewport().size()
      if viewport.width() <= 0 or viewport.height() <= 0:
        return
      fitted = self._base_pixmap.scaled(
        viewport,
        QtCore.Qt.KeepAspectRatio,
        QtCore.Qt.SmoothTransformation,
      )
      self.image_label.setPixmap(fitted)
      self.image_label.resize(fitted.size())

    def resizeEvent(self, event) -> None:
      super().resizeEvent(event)
      self._update_pixmap_fit()

    def set_content(self, canvas_bgr: np.ndarray, metrics: dict) -> None:
      canvas_rgb = cv2.cvtColor(canvas_bgr, cv2.COLOR_BGR2RGB)
      h, w, c = canvas_rgb.shape
      qimage = QtGui.QImage(canvas_rgb.data, w, h, c * w, QtGui.QImage.Format_RGB888).copy()
      self._base_pixmap = QtGui.QPixmap.fromImage(qimage)
      self._update_pixmap_fit()
      self.metrics_text.setPlainText(json.dumps(metrics, indent=2, ensure_ascii=True))


  class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
      super().__init__()
      self.setWindowTitle(WINDOW_TITLE)
      self.resize(max(1600, round(model.panel_size[0] * 3 * args.window_scale) + 420), 980)
      self._playing = False
      self._rendering = False
      self._closed = False
      self._last_saved = args.save or model.get_last_path('last_snapshot_path', str(Path.home() / 'wide_overlay_debugger.png'))
      self._canvas_rgb: np.ndarray | None = None
      self._base_pixmap: QtGui.QPixmap | None = None
      self._compare_dialog = None

      central = QtWidgets.QWidget()
      self.setCentralWidget(central)
      root = QtWidgets.QHBoxLayout(central)
      root.setContentsMargins(8, 8, 8, 8)
      root.setSpacing(8)

      self.image_label = QtWidgets.QLabel(alignment=QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
      self.image_label.setBackgroundRole(QtGui.QPalette.Base)
      self.image_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
      self.image_label.setScaledContents(False)
      self.scroll = QtWidgets.QScrollArea()
      self.scroll.setWidget(self.image_label)
      self.scroll.setWidgetResizable(True)
      root.addWidget(self.scroll, 1)

      control_panel = QtWidgets.QWidget()
      control_panel.setMinimumWidth(400)
      controls = QtWidgets.QVBoxLayout(control_panel)
      controls.setContentsMargins(8, 8, 8, 8)
      controls.setSpacing(10)
      root.addWidget(control_panel, 0)

      playback_group = QtWidgets.QGroupBox("Playback")
      playback_layout = QtWidgets.QVBoxLayout(playback_group)
      btn_row = QtWidgets.QHBoxLayout()
      self.play_btn = QtWidgets.QPushButton("Play")
      self.prev10_btn = QtWidgets.QPushButton("-10")
      self.prev_btn = QtWidgets.QPushButton("Prev")
      self.next_btn = QtWidgets.QPushButton("Next")
      self.next10_btn = QtWidgets.QPushButton("+10")
      for btn in [self.play_btn, self.prev10_btn, self.prev_btn, self.next_btn, self.next10_btn]:
        btn_row.addWidget(btn)
      playback_layout.addLayout(btn_row)

      self.sample_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
      self.sample_slider.setRange(0, len(model.samples) - 1)
      self.sample_slider.setValue(model.current)
      playback_layout.addWidget(self.sample_slider)
      self.sample_label = QtWidgets.QLabel()
      playback_layout.addWidget(self.sample_label)

      fps_row = QtWidgets.QHBoxLayout()
      fps_row.addWidget(QtWidgets.QLabel("Playback FPS"))
      self.fps_spin = QtWidgets.QSpinBox()
      self.fps_spin.setRange(1, 60)
      self.fps_spin.setValue(max(1, args.playback_fps))
      fps_row.addWidget(self.fps_spin)
      playback_layout.addLayout(fps_row)
      controls.addWidget(playback_group)

      compare_group = QtWidgets.QGroupBox("Compare")
      compare_layout = QtWidgets.QVBoxLayout(compare_group)
      probe_row = QtWidgets.QHBoxLayout()
      probe_row.addWidget(QtWidgets.QLabel("Probe mode"))
      self.probe_combo = QtWidgets.QComboBox()
      for mode in ProbeMode:
        self.probe_combo.addItem(mode.label, mode.value)
      self.probe_combo.setCurrentIndex(list(ProbeMode).index(model.probe_mode))
      probe_row.addWidget(self.probe_combo, 1)
      compare_layout.addLayout(probe_row)

      zoom_row = QtWidgets.QHBoxLayout()
      zoom_row.addWidget(QtWidgets.QLabel("Display scale"))
      self.scale_spin = QtWidgets.QDoubleSpinBox()
      self.scale_spin.setRange(0.25, 2.0)
      self.scale_spin.setSingleStep(0.05)
      self.scale_spin.setDecimals(2)
      self.scale_spin.setValue(args.window_scale)
      zoom_row.addWidget(self.scale_spin)
      compare_layout.addLayout(zoom_row)
      controls.addWidget(compare_group)

      self.calib_group = EulerGroup("Calibration Offset (deg)")
      self.wide_group = EulerGroup("Wide Offset (deg)")
      controls.addWidget(self.calib_group)
      controls.addWidget(self.wide_group)

      action_row = QtWidgets.QHBoxLayout()
      self.reset_btn = QtWidgets.QPushButton("Reset Offsets")
      self.save_btn = QtWidgets.QPushButton("Save Snapshot")
      self.export_btn = QtWidgets.QPushButton("Export CalibrationParams")
      self.compare_btn = QtWidgets.QPushButton("Show Wide Warp Compare")
      action_row.addWidget(self.reset_btn)
      action_row.addWidget(self.save_btn)
      controls.addLayout(action_row)
      controls.addWidget(self.export_btn)
      controls.addWidget(self.compare_btn)

      load_row = QtWidgets.QHBoxLayout()
      self.load_btn = QtWidgets.QPushButton("Load CalibrationParams")
      self.clear_loaded_btn = QtWidgets.QPushButton("Clear Loaded Calibration")
      load_row.addWidget(self.load_btn)
      load_row.addWidget(self.clear_loaded_btn)
      controls.addLayout(load_row)

      info_group = QtWidgets.QGroupBox("Current Sample")
      info_layout = QtWidgets.QFormLayout(info_group)
      self.info_sample = QtWidgets.QLabel()
      self.info_frames = QtWidgets.QLabel()
      self.info_speed = QtWidgets.QLabel()
      self.info_rpy = QtWidgets.QLabel()
      self.info_wide = QtWidgets.QLabel()
      self.info_height = QtWidgets.QLabel()
      self.info_status = QtWidgets.QLabel()
      info_layout.addRow("Sample", self.info_sample)
      info_layout.addRow("Frames", self.info_frames)
      info_layout.addRow("Speed", self.info_speed)
      info_layout.addRow("rpyCalib", self.info_rpy)
      info_layout.addRow("wideFromDevice", self.info_wide)
      info_layout.addRow("Height", self.info_height)
      info_layout.addRow("Status", self.info_status)
      self.info_override = QtWidgets.QLabel()
      info_layout.addRow("Initial override", self.info_override)
      controls.addWidget(info_group)
      controls.addStretch(1)

      self.timer = QtCore.QTimer(self)
      self.timer.timeout.connect(self._advance_frame)
      self._apply_fps()

      self.play_btn.clicked.connect(self._toggle_play)
      self.prev_btn.clicked.connect(lambda: self._jump(-1))
      self.next_btn.clicked.connect(lambda: self._jump(1))
      self.prev10_btn.clicked.connect(lambda: self._jump(-10))
      self.next10_btn.clicked.connect(lambda: self._jump(10))
      self.sample_slider.valueChanged.connect(self._sample_slider_changed)
      self.probe_combo.currentIndexChanged.connect(self._probe_changed)
      self.fps_spin.valueChanged.connect(self._apply_fps)
      self.scale_spin.valueChanged.connect(self._update_pixmap_scale)
      self.calib_group.valueChanged.connect(self._calib_changed)
      self.wide_group.valueChanged.connect(self._wide_changed)
      self.reset_btn.clicked.connect(self._reset_offsets)
      self.save_btn.clicked.connect(self._save_snapshot)
      self.export_btn.clicked.connect(self._export_calibration)
      self.compare_btn.clicked.connect(self._show_warp_compare)
      self.load_btn.clicked.connect(self._load_calibration)
      self.clear_loaded_btn.clicked.connect(self._clear_loaded_calibration)

      self._refresh_info()
      self._render()

    def _apply_fps(self) -> None:
      fps = max(1, int(self.fps_spin.value()))
      self.timer.setInterval(round(1000 / fps))

    def _toggle_play(self) -> None:
      self._playing = not self._playing
      self.play_btn.setText("Pause" if self._playing else "Play")
      if self._playing:
        self.timer.start()
      else:
        self.timer.stop()

    def _jump(self, delta: int) -> None:
      model.step(delta)
      self.sample_slider.blockSignals(True)
      self.sample_slider.setValue(model.current)
      self.sample_slider.blockSignals(False)
      self._refresh_info()
      self._render()

    def _advance_frame(self) -> None:
      if not self._playing:
        return
      if model.current >= len(model.samples) - 1:
        self._toggle_play()
        return
      self._jump(1)

    def _sample_slider_changed(self, value: int) -> None:
      model.set_current(value)
      self._refresh_info()
      self._render()

    def _probe_changed(self, _index: int) -> None:
      mode_value = self.probe_combo.currentData()
      model.set_probe_mode(ProbeMode(mode_value))
      self._render()

    def _calib_changed(self) -> None:
      vals = self.calib_group.values_deg()
      for axis in range(3):
        model.set_calib_offset_deg(axis, float(vals[axis]))
      self._render()

    def _wide_changed(self) -> None:
      vals = self.wide_group.values_deg()
      for axis in range(3):
        model.set_wide_offset_deg(axis, float(vals[axis]))
      self._render()

    def _reset_offsets(self) -> None:
      model.reset_offsets()
      self.calib_group.set_values_deg(model.get_calib_offset_deg())
      self.wide_group.set_values_deg(model.get_wide_offset_deg())
      self._render()

    def _render(self) -> None:
      if self._rendering:
        return
      self._rendering = True
      try:
        canvas = model.render_current()
        self._canvas_rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        height, width, channels = self._canvas_rgb.shape
        qimage = QtGui.QImage(self._canvas_rgb.data, width, height, channels * width, QtGui.QImage.Format_RGB888).copy()
        self._base_pixmap = QtGui.QPixmap.fromImage(qimage)
        self._update_pixmap_scale()
        self._refresh_info()
        self._update_warp_compare_if_visible()
      finally:
        self._rendering = False

    def _update_pixmap_scale(self) -> None:
      if self._base_pixmap is None:
        return
      scale = max(0.25, float(self.scale_spin.value()))
      pixmap = self._base_pixmap.scaled(
        round(self._base_pixmap.width() * scale),
        round(self._base_pixmap.height() * scale),
        QtCore.Qt.KeepAspectRatio,
        QtCore.Qt.SmoothTransformation,
      )
      self.image_label.setPixmap(pixmap)
      self.image_label.resize(pixmap.size())

    def _save_snapshot(self) -> None:
      if self._base_pixmap is None:
        return
      target = self._last_saved
      if not target:
        start_path = model.get_last_path('last_snapshot_path', str(Path.home() / 'wide_overlay_debugger.png'))
        target, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Snapshot", start_path, "PNG Files (*.png)")
      if not target:
        return
      save_image(target, cv2.cvtColor(self._canvas_rgb, cv2.COLOR_RGB2BGR))
      self._last_saved = target
      model.set_last_path('last_snapshot_path', target)

    def _export_calibration(self) -> None:
      default_path = model.get_last_path('last_export_path', str(Path.home() / 'CalibrationParams'))
      target, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Export CalibrationParams', default_path, 'All Files (*)')
      if not target:
        return
      raw_path, json_path = model.export_current_calibration(target)
      QtWidgets.QMessageBox.information(
        self,
        'Calibration Exported',
        f'Raw CalibrationParams written to:\n{raw_path}\n\nReadable summary written to:\n{json_path}\n\nOn device, the param key is CalibrationParams and the backing file is typically:\n/data/params/d/CalibrationParams',
      )

    def _show_warp_compare(self) -> None:
      if self._compare_dialog is None:
        self._compare_dialog = WarpCompareDialog(self)
      canvas, metrics = model.render_current_warp_compare()
      self._compare_dialog.set_content(canvas, metrics)
      self._compare_dialog.show()
      self._compare_dialog.raise_()
      self._compare_dialog.activateWindow()

    def _update_warp_compare_if_visible(self) -> None:
      if self._compare_dialog is None or not self._compare_dialog.isVisible():
        return
      canvas, metrics = model.render_current_warp_compare()
      self._compare_dialog.set_content(canvas, metrics)

    def _load_calibration(self) -> None:
      start_path = model.get_last_path('last_load_path', args.initial_calibration or str(Path.home() / 'CalibrationParams'))
      target, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Load CalibrationParams', start_path, 'All Files (*)')
      if not target:
        return
      loaded = model.load_initial_calibration(target)
      model.set_last_path('last_load_path', target)
      self.calib_group.set_values_deg(model.get_calib_offset_deg())
      self.wide_group.set_values_deg(model.get_wide_offset_deg())
      self._refresh_info()
      self._render()
      QtWidgets.QMessageBox.information(
        self,
        'Calibration Loaded',
        f'Loaded initial calibration from:\n{target}\n\nrpyCalib deg: {deg_str(loaded.rpy_calib)}\nwideFromDevice deg: {deg_str(loaded.wide_from_device_euler)}\nheight: {loaded.height:.3f} m',
      )

    def _clear_loaded_calibration(self) -> None:
      model.clear_initial_calibration()
      self.calib_group.set_values_deg(model.get_calib_offset_deg())
      self.wide_group.set_values_deg(model.get_wide_offset_deg())
      self._refresh_info()
      self._render()

    def _refresh_info(self) -> None:
      sample = model.current_sample()
      self.info_sample.setText(f"{sample.sample_idx + 1} / {len(model.samples)}")
      self.info_frames.setText(f"road {sample.road_frame_id}/{sample.road_video_idx}, wide {sample.wide_frame_id}/{sample.wide_video_idx}")
      self.info_speed.setText(f"{sample.v_ego:.2f} m/s")
      self.info_rpy.setText(deg_str(sample.calibration.rpy_calib))
      self.info_wide.setText(deg_str(sample.calibration.wide_from_device_euler))
      self.info_height.setText(f"{sample.calibration.height:.3f} m")
      self.info_status.setText(sample.calibration.status)
      if model.initial_calibration_override is None:
        self.info_override.setText('route liveCalibration')
      else:
        self.info_override.setText('loaded CalibrationParams')

    def closeEvent(self, event) -> None:
      if self._closed:
        event.accept()
        return
      self._closed = True
      self.timer.stop()
      if self._compare_dialog is not None:
        self._compare_dialog.close()
      model.close()
      event.accept()
      super().closeEvent(event)

  app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
  window = MainWindow()
  app.aboutToQuit.connect(window.close)
  window.show()
  return app.exec_()


def main() -> int:
  args = parse_args()
  model = WideOverlayDebuggerModel(args)
  if args.no_window:
    try:
      rendered = model.render_current()
      if args.save:
        save_image(args.save, rendered)
      if args.export_calibration:
        raw_path, json_path = model.export_current_calibration(args.export_calibration)
        print(f'exported calibration params to {raw_path}')
        print(f'exported calibration summary to {json_path}')
      return 0
    finally:
      model.close()

  try:
    if args.window_backend in ("auto", "qt"):
      return run_qt_viewer(model, args)
    raise RuntimeError(f"Unsupported window backend: {args.window_backend}")
  finally:
    try:
      model.close()
    except Exception:
      pass


if __name__ == "__main__":
  raise SystemExit(main())
