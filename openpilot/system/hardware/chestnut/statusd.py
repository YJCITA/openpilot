"""Publish passive Chestnut PCIe link status while offroad."""
from __future__ import annotations

import time

import openpilot.cereal.messaging as messaging
from openpilot.common.hardware.usb import is_chestnut_runtime_device
from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware.chestnut.status import read_pcie_ltssm


POLL_INTERVAL = 2.0


class PcieLinkProbe:
  def __init__(self, *, read_ltssm=read_pcie_ltssm, monotonic=time.monotonic):
    self._read_ltssm = read_ltssm
    self._monotonic = monotonic
    self._last_poll = float("-inf")
    self.ltssm = 0
    self.valid = False

  def update(self, *, connected: bool, usb_speed_mbps: int) -> tuple[bool, int]:
    if not connected or usb_speed_mbps < 5000:
      self._last_poll = float("-inf")
      self.ltssm = 0
      self.valid = connected
      return self.valid, self.ltssm

    now = self._monotonic()
    if now - self._last_poll < POLL_INTERVAL:
      return self.valid, self.ltssm
    self._last_poll = now
    try:
      self.ltssm = self._read_ltssm()
      self.valid = True
    except (OSError, RuntimeError):
      self.ltssm = 0
      self.valid = False
    return self.valid, self.ltssm


def _chestnut_usb(device_state) -> tuple[bool, int]:
  speeds = [int(device.speedMbps) for device in device_state.usbState.devices
            if is_chestnut_runtime_device({
              "vendorId": int(device.vendorId),
              "productId": int(device.productId),
              "manufacturer": str(device.manufacturer),
              "product": str(device.product),
            })]
  return bool(speeds), max(speeds, default=0)


def main() -> None:
  pm = messaging.PubMaster(["chestnutState"])
  sm = messaging.SubMaster(["deviceState"])
  probe = PcieLinkProbe()
  rk = Ratekeeper(10, print_delay_threshold=None)

  while True:
    sm.update(0)
    # First ticks after onroad modeld exits have no deviceState yet. Publishing
    # valid=False/ltssm=0 here makes the home sidebar flash LINK/PCIE ERR.
    if not sm.alive["deviceState"] and not sm.valid["deviceState"]:
      rk.keep_time()
      continue
    connected, speed = _chestnut_usb(sm["deviceState"])
    valid, ltssm = probe.update(connected=connected, usb_speed_mbps=speed)
    msg = messaging.new_message("chestnutState", valid=valid)
    msg.chestnutState.pcieLtssm = ltssm
    pm.send("chestnutState", msg)
    rk.keep_time()


if __name__ == "__main__":
  main()
