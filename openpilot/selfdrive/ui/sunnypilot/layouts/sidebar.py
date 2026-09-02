"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from dataclasses import dataclass
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.selfdrive.ui.egpu_status import (
  build_egpu_sidebar_status, chestnut_usb_speed_mbps, classify_egpu_link_state, resolve_egpu_connection,
)


METRIC_HEIGHT = 126
METRIC_WIDTH = 240
METRIC_MARGIN = 30
METRIC_START_Y = 300
HOME_BTN = rl.Rectangle(60, 860, 180, 180)


# Color scheme
class Colors:
  WHITE = rl.WHITE
  WHITE_DIM = rl.Color(255, 255, 255, 85)
  GRAY = rl.Color(84, 84, 84, 255)

  # Status colors
  GOOD = rl.WHITE
  WARNING = rl.Color(218, 202, 37, 255)
  DANGER = rl.Color(201, 34, 49, 255)
  PROGRESS = rl.Color(0, 134, 233, 255)
  DISABLED = rl.Color(128, 128, 128, 255)

  # UI elements
  METRIC_BORDER = rl.Color(255, 255, 255, 85)
  BUTTON_NORMAL = rl.WHITE
  BUTTON_PRESSED = rl.Color(255, 255, 255, 166)


@dataclass(slots=True)
class MetricData:
  label: str
  value: str
  color: rl.Color
  icon: object | None = None

  def update(self, label: str, value: str, color: rl.Color, icon: object | None = None):
    self.label = label
    self.value = value
    self.color = color
    self.icon = icon


class SidebarSP:
  def __init__(self):
    self._egpu_icon = gui_app.texture("icons_mici/egpu_green.png", 60, 44)
    self._egpu_icon_gray = gui_app.texture("icons_mici/egpu_gray.png", 60, 44)
    self._egpu_icon_orange = gui_app.texture("icons_mici/egpu_orange.png", 60, 44)
    self._egpu_icon_crossed = gui_app.texture("icons_mici/egpu_crossed.png", 60, 52)
    self._egpu_status = MetricData("eGPU", "OFFLINE", Colors.DISABLED, self._egpu_icon_gray)
    self._egpu_metric_rect = rl.Rectangle(0, 0, 0, 0)
    self._egpu_detail = "未检测到 eGPU USB 设备"

  def _update_egpu_status(self):
    present = resolve_egpu_connection(ui_state.sm["deviceState"])
    eject_status = ui_state.params.get("UsbGpuEjectStatus")
    speed_mbps = chestnut_usb_speed_mbps(ui_state.sm["deviceState"])
    telemetry = ui_state.sm["chestnutState"]
    link_state = classify_egpu_link_state(
      present=present,
      usb_speed_mbps=speed_mbps,
      telemetry_alive=bool(ui_state.sm.alive["chestnutState"]),
      telemetry_valid=bool(ui_state.sm.valid["chestnutState"]),
      pcie_ltssm=int(telemetry.pcieLtssm),
    )
    started = ui_state.started
    status = build_egpu_sidebar_status(
      present=present,
      compiled=ui_state.usbgpu_compiled,
      link_state=link_state,
      usb_speed_mbps=speed_mbps,
      pcie_ltssm=int(telemetry.pcieLtssm) if ui_state.sm.valid["chestnutState"] else None,
      eject_status=eject_status,
      loading=ui_state.usbgpu_loading if started else False,
      active=ui_state.usbgpu_active if started else None,
      loading_progress=ui_state.usbgpu_loading_progress,
      model_failed=ui_state.big_model_failed if started else False,
    )
    color = {
      "good": Colors.GOOD,
      "warning": Colors.WARNING,
      "danger": Colors.DANGER,
      "progress": Colors.PROGRESS,
      "disabled": Colors.DISABLED,
    }[status.severity]
    icon = {
      "good": self._egpu_icon,
      "warning": self._egpu_icon_orange,
      "danger": self._egpu_icon_crossed,
      "progress": self._egpu_icon_gray,
      "disabled": self._egpu_icon_gray,
    }[status.severity]
    if status.value == "REMOVE":
      icon = self._egpu_icon_gray
    self._egpu_detail = status.detail
    self._egpu_status.update("eGPU", status.value, color, icon)

  def _handle_egpu_click(self, mouse_pos) -> bool:
    if not rl.check_collision_point_rec(mouse_pos, self._egpu_metric_rect):
      return False

    if ui_state.started:
      gui_app.push_widget(ConfirmDialog(f"{self._egpu_detail}\neGPU · OFFROAD · {tr('REMOVE')}", tr("OK"), cancel_text=""))
      return True

    present = resolve_egpu_connection(ui_state.sm["deviceState"])
    status = ui_state.params.get("UsbGpuEjectStatus")
    if not present:
      gui_app.push_widget(ConfirmDialog(self._egpu_detail, tr("OK"), cancel_text=""))
      return True
    if status in ("ejecting", "safe"):
      return True

    def confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        ui_state.params.put_bool("UsbGpuEjectRequest", True)

    error = ui_state.params.get("UsbGpuEjectError")
    if status == "error" and error:
      message = f"{self._egpu_detail}\neGPU · {tr('ERROR')}: {error}\n{tr('REMOVE')}?"
    else:
      message = f"{self._egpu_detail}\neGPU · {tr('REMOVE')}?\n{tr('REMOVE')}...  >  {tr('REMOVE')}"
    gui_app.push_widget(ConfirmDialog(message, tr("REMOVE"), callback=confirm))
    return True

  def _draw_metrics_sp(self, rect: rl.Rectangle, _temp, _panda, _connect):
    metrics = [_temp, _panda, _connect, self._egpu_status]
    start_y = int(rect.y) + METRIC_START_Y
    available_height = max(0, int(HOME_BTN.y) - METRIC_MARGIN - METRIC_HEIGHT - start_y)
    spacing = available_height / max(1, len(metrics) - 1)
    self._egpu_metric_rect = rl.Rectangle(rect.x + METRIC_MARGIN, start_y + (len(metrics) - 1) * spacing, METRIC_WIDTH, METRIC_HEIGHT)

    return metrics, start_y, spacing
