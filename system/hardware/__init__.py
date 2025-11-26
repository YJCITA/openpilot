import os
from typing import cast

from openpilot.system.hardware.base import HardwareBase
from openpilot.system.hardware.tici.hardware import Tici
from openpilot.system.hardware.pc.hardware import Pc

TICI = os.path.isfile('/TICI')
AGNOS = os.path.isfile('/AGNOS')
# -YJ- 没有驾驶员监控摄像头
C3XL = os.path.isfile("/data/C3XL")
# comma原厂c3, 带ssd硬盘，会开启log
C3 = os.path.isfile("/data/C3")
PC = not TICI


if TICI:
  HARDWARE = cast(HardwareBase, Tici())
else:
  HARDWARE = cast(HardwareBase, Pc())
