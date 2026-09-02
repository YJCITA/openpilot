from openpilot.selfdrive.ui.egpu_status import (
  build_egpu_sidebar_status,
  build_egpu_status,
  build_compact_egpu_status,
  classify_egpu_link_state,
  egpu_icon_visible,
  egpu_panel_style,
  resolve_egpu_connection,
)


class FakeDeviceState:
  def __init__(self, present: bool):
    self.chestnutPresent = present


def test_physical_disconnect_immediately_clears_egpu_connection():
  device_state = FakeDeviceState(True)
  assert resolve_egpu_connection(device_state)

  device_state.chestnutPresent = False
  assert not resolve_egpu_connection(device_state)


def test_connected_egpu_icon_stays_visible_onroad():
  assert egpu_icon_visible(connected=True)
  assert not egpu_icon_visible(connected=False)


def test_egpu_panel_is_at_least_twice_as_large_with_transparent_background():
  style = egpu_panel_style(compact=True)
  assert style.font_size >= 48
  assert style.line_height >= 58
  assert style.background_alpha == 0


def test_egpu_status_explains_fallback_reason():
  status = build_egpu_status(
    connected=True, compiled=True, loading=False, active=False,
    model_alive=True, model_big=False, telemetry_valid=False, usb_speed_mbps=5000,
    model_name="IDM",
  )
  assert status.visible
  assert not status.healthy
  assert status.headline.startswith("IDM · ")
  assert "回退小模型" in status.headline
  assert "USB" not in " ".join((status.headline, *status.details))


def test_egpu_status_reports_live_model_and_gpu_metrics():
  status = build_egpu_status(
    connected=True, compiled=True, loading=False, active=True,
    model_alive=True, model_big=True, telemetry_valid=True,
    model_name="IDM",
    usb_speed_mbps=5000, model_fps=19.8, power_w=72.0,
    temp_c=61.0, memory_temp_c=70.0, memory_used_mb=6144,
    memory_total_mb=8192, gpu_usage_percent=88, gpu_clock_mhz=2200,
    fan_speed_rpm=1450,
  )
  assert status.healthy
  assert status.headline.startswith("IDM · ")
  assert "19.8 FPS" in status.headline
  assert status.details[0] == "功耗 72 W"
  assert "6.0/8.0 GB" in status.details[1]
  assert "88%" in status.details[2]
  assert "2200 MHz" in status.details[2]
  assert "AMD" not in " ".join((status.headline, *status.details))
  assert "USB 5000" not in " ".join((status.headline, *status.details))


def test_compact_bottom_status_contains_only_requested_gpu_metrics():
  status = build_compact_egpu_status(
    connected=True, compiled=True, loading=False, active=True,
    model_alive=True, model_big=True, telemetry_valid=True, model_name="IDM",
    model_fps=19.8, power_w=72.0, temp_c=61.0, memory_temp_c=70.0,
    memory_used_mb=6144, memory_total_mb=8192, gpu_usage_percent=88,
  )

  assert status.visible
  assert status.healthy
  assert status.text == "IDM: 19.8FPS  GPU 72W 61°/70° 6.0/8.0G 88%"
  assert "USB" not in status.text
  assert "AMD" not in status.text


def test_compact_bottom_status_replaces_metrics_with_loading_progress():
  status = build_compact_egpu_status(
    connected=True, compiled=True, loading=True, active=None,
    model_alive=False, model_big=False, telemetry_valid=False,
    model_name="IDM", loading_progress=57,
  )

  assert status.text == "IDM: LOAD 57%"


def test_absent_egpu_has_no_panel():
  status = build_egpu_status(
    connected=False, compiled=False, loading=False, active=None,
    model_alive=False, model_big=False, telemetry_valid=False,
  )
  assert not status.visible


def test_loading_status_exposes_real_progress_percentage():
  panel = build_egpu_status(
    connected=True, compiled=True, loading=True, active=None,
    model_alive=False, model_big=False, telemetry_valid=False, loading_progress=64,
    model_name="IDM",
  )
  sidebar = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="ready", usb_speed_mbps=5000,
    pcie_ltssm=0x78, eject_status=None, loading=True, active=None, loading_progress=64,
  )

  assert panel.headline.startswith("IDM · ")
  assert "64%" in panel.headline
  assert sidebar.value == "LOAD 64%"
  assert "64%" in sidebar.detail


def test_optional_egpu_absence_stays_quiet_even_when_big_model_is_compiled():
  status = build_egpu_status(
    connected=False, compiled=True, loading=False, active=None,
    model_alive=False, model_big=False, telemetry_valid=False,
  )
  assert not status.visible
  assert not status.headline


def test_sidebar_makes_usb_degradation_visible_at_a_glance():
  status = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="usb_degraded", usb_speed_mbps=480,
    pcie_ltssm=None, eject_status=None, loading=False, active=None,
  )

  assert status.value == "USB 480"
  assert status.severity == "danger"
  assert "低于 5000" in status.detail


def test_sidebar_makes_pcie_failure_distinct_from_usb_failure():
  status = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="pcie_down", usb_speed_mbps=5000,
    pcie_ltssm=0, eject_status=None, loading=False, active=None,
  )

  assert status.value == "PCIE ERR"
  assert status.severity == "danger"
  assert "LTSSM 0x00" in status.detail


def test_sidebar_reports_ready_link_before_model_starts():
  status = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="ready", usb_speed_mbps=5000,
    pcie_ltssm=0x78, eject_status=None, loading=False, active=None,
  )

  assert status.value == "READY"
  assert status.severity == "good"


def test_sidebar_uses_full_model_failure_instead_of_stale_active_flag():
  status = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="ready", usb_speed_mbps=5000,
    pcie_ltssm=0x78, eject_status=None, loading=False, active=True,
    model_failed=True,
  )

  assert status.value == "MODEL ERR"
  assert status.severity == "danger"


def test_sidebar_link_classification_uses_existing_chestnut_telemetry():
  assert classify_egpu_link_state(
    present=True, usb_speed_mbps=480, telemetry_alive=True, telemetry_valid=True, pcie_ltssm=0x78,
  ) == "usb_degraded"
  assert classify_egpu_link_state(
    present=True, usb_speed_mbps=5000, telemetry_alive=True, telemetry_valid=True, pcie_ltssm=0,
  ) == "pcie_down"
  assert classify_egpu_link_state(
    present=True, usb_speed_mbps=5000, telemetry_alive=True, telemetry_valid=True, pcie_ltssm=0x78,
  ) == "ready"


def test_compact_bottom_status_keeps_loading_until_first_model_frame():
  status = build_compact_egpu_status(
    connected=True, compiled=True, loading=False, active=True,
    model_alive=False, model_big=False, telemetry_valid=False,
    model_name="TT", loading_progress=100,
  )
  assert status.visible
  assert status.text == "TT: LOAD 100%"
  assert "ERR" not in status.text


def test_compact_bottom_status_reports_stream_error_only_after_model_started():
  status = build_compact_egpu_status(
    connected=True, compiled=True, loading=False, active=True,
    model_alive=False, model_big=False, telemetry_valid=False,
    model_name="TT", loading_progress=100, model_started=True,
  )
  assert status.text == "TT: STREAM ERR"

  status = build_compact_egpu_status(
    connected=True, compiled=True, loading=False, active=True,
    model_alive=True, model_big=False, telemetry_valid=False,
    model_name="TT",
  )
  assert status.text == "TT: STREAM ERR"


def test_sidebar_home_does_not_treat_telemetry_handoff_as_error():
  assert classify_egpu_link_state(
    present=True, usb_speed_mbps=5000, telemetry_alive=False, telemetry_valid=False, pcie_ltssm=0,
  ) == "unchecked"
  assert classify_egpu_link_state(
    present=True, usb_speed_mbps=5000, telemetry_alive=True, telemetry_valid=False, pcie_ltssm=0,
  ) == "unchecked"

  status = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="unchecked", usb_speed_mbps=5000,
    pcie_ltssm=None, eject_status=None, loading=False, active=None,
  )
  assert status.value == "READY"
  assert status.severity == "good"
  assert "ERR" not in status.value
