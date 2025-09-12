import os
from typing import cast

from openpilot.system.hardware.base import HardwareBase
from openpilot.system.hardware.tici.hardware import Tici
from openpilot.system.hardware.pc.hardware import Pc

TICI = os.path.isfile('/TICI')
AGNOS = os.path.isfile('/AGNOS')
# -YJ-
C3L = os.path.isfile("/data/C3L")
PC = not TICI


if TICI:
  HARDWARE = cast(HardwareBase, Tici())
else:
  HARDWARE = cast(HardwareBase, Pc())
