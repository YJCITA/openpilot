"""Read-only eGPU status model and the small left-side HUD panel."""
from __future__ import annotations

from dataclasses import dataclass

import pyray as rl

from openpilot.common.hardware.usb import CHESTNUT_USB_IDS


@dataclass(frozen=True)
class EgpuStatus:
  visible: bool
  healthy: bool
  headline: str
  details: tuple[str, ...]


@dataclass(frozen=True)
class CompactEgpuStatus:
  visible: bool
  healthy: bool
  text: str


@dataclass(frozen=True)
class EgpuPanelStyle:
  font_size: int
  line_height: int
  panel_width: int
  bottom_gap: int
  background_alpha: int


@dataclass(frozen=True)
class EgpuSidebarStatus:
  value: str
  severity: str
  detail: str


def build_egpu_sidebar_status(*, present: bool, compiled: bool, link_state: str | None,
                              usb_speed_mbps: int, pcie_ltssm: int | None,
                              eject_status: str | None, loading: bool,
                              active: bool | None, loading_progress: int = 0,
                              model_failed: bool = False) -> EgpuSidebarStatus:
  if eject_status == "ejecting":
    return EgpuSidebarStatus("REMOVE...", "progress", "正在安全卸载 eGPU")
  if eject_status == "safe":
    return EgpuSidebarStatus("REMOVE", "good", "已安全卸载，可以断开 eGPU")
  if eject_status == "error":
    return EgpuSidebarStatus("EJECT ERR", "danger", "eGPU 安全卸载失败")
  if not present:
    return EgpuSidebarStatus("OFFLINE", "disabled", "未检测到 eGPU USB 设备")
  if link_state == "usb_degraded" or 0 < usb_speed_mbps < 5000:
    return EgpuSidebarStatus(f"USB {usb_speed_mbps}", "danger", f"USB 链路低于 5000 Mbps（当前 {usb_speed_mbps} Mbps）")
  if link_state == "pcie_down":
    ltssm = f"0x{pcie_ltssm:02X}" if pcie_ltssm is not None else "未知"
    return EgpuSidebarStatus("PCIE ERR", "danger", f"USB 正常，但 PCIe 未进入 L0（LTSSM {ltssm}）")
  if link_state == "check_error":
    return EgpuSidebarStatus("LINK ERR", "danger", "无法被动读取 PCIe 链路状态")
  # SuperSpeed already confirmed by USB; missing chestnutState is the publisher
  # restart window, not a PCIe fault. Keep the idle/active path instead of CHECKING/ERR.
  if link_state != "ready" and not (link_state == "unchecked" and usb_speed_mbps >= 5000):
    return EgpuSidebarStatus("CHECKING", "warning", f"USB {usb_speed_mbps or '?'} Mbps，正在确认 PCIe 状态")
  if not compiled:
    return EgpuSidebarStatus("NO MODEL", "warning", "USB/PCIe 正常，但默认大模型尚未编译")
  if loading:
    return EgpuSidebarStatus(f"LOAD {loading_progress}%", "progress", f"USB/PCIe 正常，大模型加载 {loading_progress}%")
  if model_failed or active is False:
    return EgpuSidebarStatus("MODEL ERR", "danger", "链路正常，但默认大模型加载或运行失败")
  if active is True:
    return EgpuSidebarStatus("ACTIVE", "good", f"eGPU 大模型运行中 · USB {usb_speed_mbps} Mbps · PCIe L0")
  return EgpuSidebarStatus("READY", "good", f"eGPU 已就绪 · USB {usb_speed_mbps} Mbps · PCIe L0")


def classify_egpu_link_state(*, present: bool, usb_speed_mbps: int, telemetry_alive: bool,
                             telemetry_valid: bool, pcie_ltssm: int) -> str:
  if not present:
    return "disconnected"
  if usb_speed_mbps < 5000:
    return "usb_degraded"
  # modeld -> offroad statusd handoff leaves chestnutState briefly dead/invalid.
  # That is not a link failure; wait for the next valid LTSSM instead of LINK ERR.
  if not telemetry_alive or not telemetry_valid:
    return "unchecked"
  if pcie_ltssm != 0x78:
    return "pcie_down"
  return "ready"


def egpu_icon_visible(*, connected: bool) -> bool:
  """The onroad source icon is persistent for exactly the USB-connected state."""
  return connected


def resolve_egpu_connection(device_state) -> bool:
  """Return the current physical Chestnut USB presence without trip latching."""
  return bool(device_state.chestnutPresent)


def egpu_panel_style(*, compact: bool) -> EgpuPanelStyle:
  return EgpuPanelStyle(
    font_size=48,
    line_height=58,
    panel_width=1200,
    bottom_gap=73 if compact else 293,
    background_alpha=0,
  )


def build_egpu_status(*, connected: bool, compiled: bool, loading: bool, active: bool | None,
                      model_alive: bool, model_big: bool, telemetry_valid: bool,
                      model_name: str = "",
                      loading_progress: int = 0,
                      usb_speed_mbps: int = 0, model_fps: float = 0.0,
                      power_w: float = 0.0, temp_c: float = 0.0, memory_temp_c: float = 0.0,
                      memory_used_mb: int = 0, memory_total_mb: int = 0,
                      gpu_usage_percent: int = 0, gpu_clock_mhz: int = 0,
                      fan_speed_rpm: int = 0,
                      model_started: bool = False) -> EgpuStatus:
  if not connected:
    return EgpuStatus(False, False, "", ())

  model_label = model_name.strip() or "大模型"
  if not compiled:
    return EgpuStatus(True, False, f"{model_label} · 大模型未编译", ())
  starting = bool(active is True and not model_alive and not model_started)
  if loading or starting:
    return EgpuStatus(True, False, f"{model_label} · 正在加载大模型 · {loading_progress}%", ())
  if active is False:
    return EgpuStatus(True, False, f"{model_label} · 大模型失败 · 已回退小模型", ())
  if active is not True:
    return EgpuStatus(True, False, f"{model_label} · 等待模型启动", ())
  if not model_alive or not model_big:
    return EgpuStatus(True, False, f"{model_label} · 模型流中断 · 已回退/等待", ())
  if not telemetry_valid:
    return EgpuStatus(True, False, f"{model_label} · 大模型运行中 · 遥测暂不可用", ())

  used_gb = memory_used_mb / 1024.0
  total_gb = memory_total_mb / 1024.0
  degraded_link = 0 < usb_speed_mbps < 5000
  headline = (f"{model_label} · 大模型运行中 · USB 链路降速 · {model_fps:.1f} FPS" if degraded_link else
              f"{model_label} · 大模型运行中 · {model_fps:.1f} FPS")
  return EgpuStatus(True, not degraded_link, headline, (
    f"功耗 {power_w:.0f} W",
    f"GPU {temp_c:.0f}°C · 显存 {memory_temp_c:.0f}°C · {used_gb:.1f}/{total_gb:.1f} GB",
    f"负载 {gpu_usage_percent}% · {gpu_clock_mhz} MHz · 风扇 {fan_speed_rpm} RPM",
  ))


def build_compact_egpu_status(*, connected: bool, compiled: bool, loading: bool, active: bool | None,
                              model_alive: bool, model_big: bool, telemetry_valid: bool,
                              model_name: str = "", loading_progress: int = 0,
                              model_fps: float = 0.0, power_w: float = 0.0,
                              temp_c: float = 0.0, memory_temp_c: float = 0.0,
                              memory_used_mb: int = 0, memory_total_mb: int = 0,
                              gpu_usage_percent: int = 0,
                              model_started: bool = False) -> CompactEgpuStatus:
  """Compact model/GPU status for the left side of the bottom onroad strip."""
  if not connected:
    return CompactEgpuStatus(False, False, "")

  model_label = model_name.strip() or "MODEL"
  if not compiled:
    return CompactEgpuStatus(True, False, f"{model_label}: NO MODEL")
  starting = bool(active is True and not model_alive and not model_started)
  if loading or starting:
    return CompactEgpuStatus(True, False, f"{model_label}: LOAD {loading_progress}%")
  if active is False:
    return CompactEgpuStatus(True, False, f"{model_label}: ERR")
  if active is not True:
    return CompactEgpuStatus(True, False, f"{model_label}: WAIT")
  if not model_alive or not model_big:
    return CompactEgpuStatus(True, False, f"{model_label}: STREAM ERR")
  if not telemetry_valid:
    return CompactEgpuStatus(True, False, f"{model_label}: RUN")

  used_gb = memory_used_mb / 1024.0
  total_gb = memory_total_mb / 1024.0
  text = "".join((
    f"{model_label}: {model_fps:.1f}FPS  GPU {power_w:.0f}W ",
    f"{temp_c:.0f}°/{memory_temp_c:.0f}° {used_gb:.1f}/{total_gb:.1f}G {gpu_usage_percent}%",
  ))
  return CompactEgpuStatus(True, True, text)


def chestnut_usb_speed_mbps(device_state) -> int:
  speeds = [int(device.speedMbps) for device in device_state.usbState.devices
            if (int(device.vendorId), int(device.productId)) in CHESTNUT_USB_IDS]
  return max(speeds, default=0)


def draw_egpu_status_panel(rect: rl.Rectangle, font: rl.Font, *, compact: bool) -> None:
  # Avoid constructing the global UIState when importing the pure status model
  # in tests and diagnostics.
  from openpilot.selfdrive.ui.ui_state import ui_state

  sm = ui_state.sm
  connected = resolve_egpu_connection(sm["deviceState"])
  model_seen = sm.recv_frame["modelV2"] > ui_state.started_frame
  model_alive = bool(model_seen and sm.alive["modelV2"])
  model_big = bool(model_alive and sm["modelV2"].big)
  telemetry_valid = bool(sm.alive["chestnutState"] and sm.valid["chestnutState"])
  telemetry = sm["chestnutState"]
  active_bundle = ui_state.active_bundle
  model_name = ""
  if isinstance(active_bundle, dict):
    model_name = str(active_bundle.get("internalName") or active_bundle.get("displayName") or "")
  elif active_bundle is not None:
    model_name = str(getattr(active_bundle, "internalName", "") or getattr(active_bundle, "displayName", ""))
  status = build_egpu_status(
    connected=connected, compiled=ui_state.usbgpu_compiled, loading=ui_state.usbgpu_loading,
    active=ui_state.usbgpu_active, model_alive=model_alive, model_big=model_big,
    model_name=model_name,
    loading_progress=ui_state.usbgpu_loading_progress,
    model_started=model_seen,
    telemetry_valid=telemetry_valid, usb_speed_mbps=chestnut_usb_speed_mbps(sm["deviceState"]),
    model_fps=float(telemetry.modelFps), power_w=float(telemetry.powerDrawW),
    temp_c=float(telemetry.tempC), memory_temp_c=float(telemetry.memoryTempC),
    memory_used_mb=int(telemetry.memoryUsedMb), memory_total_mb=int(telemetry.memoryTotalMb),
    gpu_usage_percent=int(telemetry.gpuUsagePercent), gpu_clock_mhz=int(telemetry.gpuClockMhz),
    fan_speed_rpm=int(telemetry.fanSpeedRpm),
  )
  if not status.visible:
    return

  style = egpu_panel_style(compact=compact)
  lines = (status.headline, *status.details)
  panel_height = len(lines) * style.line_height
  panel = rl.Rectangle(
    rect.x + 12,
    rect.y + rect.height - style.bottom_gap - panel_height,
    style.panel_width,
    panel_height,
  )
  if style.background_alpha:
    rl.draw_rectangle_rounded(panel, 0.16, 8, rl.Color(0, 0, 0, style.background_alpha))
  for index, line in enumerate(lines):
    color = rl.Color(80, 220, 120, 245) if status.healthy and index == 0 else (
      rl.Color(255, 180, 60, 245) if index == 0 else rl.Color(255, 255, 255, 225)
    )
    rl.draw_text_ex(font, line, rl.Vector2(panel.x + 12, panel.y + index * style.line_height), style.font_size, 0, color)
