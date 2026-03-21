#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

import cv2
import numpy as np

from openpilot.common.transformations.camera import view_frame_from_device_frame
from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE, calib_from_medmodel, calib_from_sbigmodel
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.tools.lib.framereader import FrameReader
from openpilot.tools.lib.logreader import LogReader
from tools.dev.wide_overlay_debugger import load_calibration_source, load_segment, deg_str

MODEL_W, MODEL_H = MEDMODEL_INPUT_SIZE
TILE_W, TILE_H = 640, 360
BG = (18, 18, 18)
TEXT = (235, 235, 235)
SUBTEXT = (180, 180, 180)


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description='Visualize current vs proposed wide-camera warp differences.')
  parser.add_argument(
    '--segment-dir',
    default='/home/yj/bak/data/comma_data/tesla/20260318_c3l_data_noenage/2026-03-19--08-58-36--0',
    help='Local segment directory containing rlog.zst/fcamera.hevc/ecamera.hevc',
  )
  parser.add_argument(
    '--calibration',
    default='/home/yj/op/tool/c3l_相机参数/CalibrationParams',
    help='CalibrationParams raw bytes or exported JSON summary',
  )
  parser.add_argument('--sample-index', type=int, default=-1, help='Model sample index to visualize. Default: use calibration source sample index if available, else middle sample.')
  parser.add_argument('--output-image', default=str(REPO_ROOT / '.ai' / 'docs' / 'codex' / 'wide_warp_compare_20260319.png'))
  parser.add_argument('--output-json', default=str(REPO_ROOT / '.ai' / 'docs' / 'codex' / 'wide_warp_compare_20260319.json'))
  return parser.parse_args()


def get_source_sample_index(calibration_path: Path) -> int | None:
  json_path = calibration_path if calibration_path.suffix.lower() == '.json' else calibration_path.with_name(calibration_path.name + '.json')
  if not json_path.exists():
    return None
  try:
    data = json.loads(json_path.read_text())
  except Exception:
    return None
  idx = data.get('source_sample_index')
  return int(idx) if isinstance(idx, int) else None


def get_warp_matrix_with_camera_rotation(device_from_calib_euler: np.ndarray, intrinsics: np.ndarray,
                                         bigmodel_frame: bool, camera_from_device_euler: np.ndarray | None = None) -> np.ndarray:
  calib_from_model = calib_from_sbigmodel if bigmodel_frame else calib_from_medmodel
  device_from_calib = rot_from_euler(device_from_calib_euler)
  camera_from_device = np.eye(3, dtype=np.float32) if camera_from_device_euler is None else rot_from_euler(camera_from_device_euler)
  camera_from_calib = intrinsics @ view_frame_from_device_frame @ camera_from_device @ device_from_calib
  return (camera_from_calib @ calib_from_model).astype(np.float32)


def warp_bgr(image_bgr: np.ndarray, warp_matrix: np.ndarray) -> np.ndarray:
  return cv2.warpPerspective(
    image_bgr,
    warp_matrix,
    (MODEL_W, MODEL_H),
    flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR,
    borderMode=cv2.BORDER_CONSTANT,
    borderValue=(0, 0, 0),
  )


def resize_letterbox(img: np.ndarray, title: str, subtitle: str | None = None) -> np.ndarray:
  canvas = np.full((TILE_H, TILE_W, 3), BG, dtype=np.uint8)
  h, w = img.shape[:2]
  scale = min((TILE_W - 20) / w, (TILE_H - 50) / h)
  new_w = max(1, round(w * scale))
  new_h = max(1, round(h * scale))
  resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
  x0 = (TILE_W - new_w) // 2
  y0 = 36 + (TILE_H - 36 - new_h) // 2
  canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
  cv2.putText(canvas, title, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, TEXT, 2, cv2.LINE_AA)
  if subtitle:
    cv2.putText(canvas, subtitle, (12, TILE_H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, SUBTEXT, 1, cv2.LINE_AA)
  return canvas


def build_diff_heatmap(current_warp: np.ndarray, proposed_warp: np.ndarray) -> tuple[np.ndarray, dict]:
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


def save_outputs(output_image: Path, output_json: Path, image: np.ndarray, metrics: dict) -> None:
  output_image.parent.mkdir(parents=True, exist_ok=True)
  output_json.parent.mkdir(parents=True, exist_ok=True)
  if not cv2.imwrite(str(output_image), image):
    raise OSError(f'Failed to save {output_image}')
  output_json.write_text(json.dumps(metrics, indent=2, ensure_ascii=True) + '\\n')


def main() -> int:
  args = parse_args()
  seg_path = Path(args.segment_dir).expanduser().resolve()
  cal_path = Path(args.calibration).expanduser().resolve()

  camera, samples, _, _ = load_segment(seg_path)
  if not samples:
    raise RuntimeError('No valid model samples found.')

  source_idx = get_source_sample_index(cal_path)
  if args.sample_index >= 0:
    sample_idx = min(max(args.sample_index, 0), len(samples) - 1)
  elif source_idx is not None and 0 <= source_idx < len(samples):
    sample_idx = source_idx
  else:
    sample_idx = len(samples) // 2

  sample = samples[sample_idx]
  calib = load_calibration_source(str(cal_path))

  road_reader = FrameReader(str(seg_path / 'fcamera.hevc'), pix_fmt='rgb24')
  wide_reader = FrameReader(str(seg_path / 'ecamera.hevc'), pix_fmt='rgb24')
  road_bgr = cv2.cvtColor(road_reader.get(sample.road_video_idx), cv2.COLOR_RGB2BGR)
  wide_bgr = cv2.cvtColor(wide_reader.get(sample.wide_video_idx), cv2.COLOR_RGB2BGR)

  narrow_warp_matrix = get_warp_matrix_with_camera_rotation(calib.rpy_calib, camera.fcam_intrinsics, False, None)
  wide_current_warp_matrix = get_warp_matrix_with_camera_rotation(calib.rpy_calib, camera.ecam_intrinsics, True, None)
  wide_proposed_warp_matrix = get_warp_matrix_with_camera_rotation(calib.rpy_calib, camera.ecam_intrinsics, True, calib.wide_from_device_euler)

  narrow_warp = warp_bgr(road_bgr, narrow_warp_matrix)
  wide_current_warp = warp_bgr(wide_bgr, wide_current_warp_matrix)
  wide_proposed_warp = warp_bgr(wide_bgr, wide_proposed_warp_matrix)
  diff_heatmap, diff_metrics = build_diff_heatmap(wide_current_warp, wide_proposed_warp)

  top = cv2.hconcat([
    resize_letterbox(road_bgr, 'Narrow raw', f'frameId/videoIdx: {sample.road_frame_id}/{sample.road_video_idx}'),
    resize_letterbox(wide_bgr, 'Wide raw', f'frameId/videoIdx: {sample.wide_frame_id}/{sample.wide_video_idx}'),
    resize_letterbox(narrow_warp, 'Narrow warp (current)', f'rpyCalib deg: {deg_str(calib.rpy_calib)}'),
  ])
  bottom = cv2.hconcat([
    resize_letterbox(wide_current_warp, 'Wide warp (current)', 'uses rpyCalib only'),
    resize_letterbox(wide_proposed_warp, 'Wide warp (proposed)', f'uses wideFromDevice deg: {deg_str(calib.wide_from_device_euler)}'),
    resize_letterbox(diff_heatmap, 'Abs diff heatmap', f"mean={diff_metrics['mean_abs_diff_gray']:.2f}, gt16={diff_metrics['pixels_gt_16']*100:.1f}%"),
  ])
  canvas = cv2.vconcat([top, bottom])

  metrics = {
    'segment_dir': str(seg_path),
    'calibration_path': str(cal_path),
    'sample_index': sample_idx,
    'sample_log_mono_time': sample.log_mono_time,
    'sample_road_frame_id': sample.road_frame_id,
    'sample_wide_frame_id': sample.wide_frame_id,
    'calibration_rpy_deg': np.degrees(calib.rpy_calib).tolist(),
    'calibration_wide_from_device_deg': np.degrees(calib.wide_from_device_euler).tolist(),
    'diff_metrics': diff_metrics,
  }

  output_image = Path(args.output_image).expanduser().resolve()
  output_json = Path(args.output_json).expanduser().resolve()
  save_outputs(output_image, output_json, canvas, metrics)
  print(f'saved image: {output_image}')
  print(f'saved metrics: {output_json}')
  print(json.dumps(metrics, indent=2, ensure_ascii=True))
  return 0


if __name__ == '__main__':
  raise SystemExit(main())