#!/usr/bin/env python3
import time
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_DMON
from openpilot.selfdrive.monitoring.helpers import DriverMonitoring
from openpilot.selfdrive.selfdrived.events import Events


def get_dummy_state_packet():
  """驾驶员监控关闭时发布的全通过状态"""
  dat = messaging.new_message('driverMonitoringState', valid=True)
  dat.driverMonitoringState = {
    "events": Events().to_msg(),
    "faceDetected": True,
    "isDistracted": False,
    "distractedType": 0,
    "awarenessStatus": 1.0,
    "posePitchOffset": 0.0,
    "posePitchValidCount": 0,
    "poseYawOffset": 0.0,
    "poseYawValidCount": 0,
    "stepChange": 0.0,
    "awarenessActive": 1.0,
    "awarenessPassive": 1.0,
    "isLowStd": True,
    "hiStdCount": 0,
    "isActiveMode": True,
    "isRHD": False,
    "uncertainCount": 0,
  }
  return dat


def dmonitoringd_thread():
  config_realtime_process([0, 1, 2, 3], 5)

  params = Params()
  pm = messaging.PubMaster(['driverMonitoringState'])

  # 驾驶员监控关闭时仅发布 dummy 状态，不运行模型
  while True:
    if not params.get_bool("DriverMonitoringEnabled"):
      pm.send('driverMonitoringState', get_dummy_state_packet())
      time.sleep(DT_DMON)
      continue

    break

  sm = messaging.SubMaster(['driverStateV2', 'liveCalibration', 'carState', 'selfdriveState', 'modelV2',
                            'carControl'], poll='driverStateV2')

  # 注意力衰减时间倍数：DriverMonitoringTimeMultiplierIndex 0=1x 1=2x 2=3x(默认)
  idx = params.get("DriverMonitoringTimeMultiplierIndex", return_default=True)
  time_multiplier = float(int(idx) + 1) if idx is not None else 3.0
  DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"),
                        time_multiplier=time_multiplier)
  demo_mode = False

  # 20Hz <- dmonitoringmodeld
  while True:
    # 运行时检查是否关闭
    if not params.get_bool("DriverMonitoringEnabled"):
      pm.send('driverMonitoringState', get_dummy_state_packet())
      time.sleep(DT_DMON)
      continue

    sm.update()
    if not sm.updated['driverStateV2']:
      continue

    valid = sm.all_checks()
    if demo_mode and sm.valid['driverStateV2']:
      DM.run_step(sm, demo=demo_mode)
    elif valid:
      DM.run_step(sm, demo=demo_mode)

    dat = DM.get_state_packet(valid=valid)
    pm.send('driverMonitoringState', dat)

    if sm['driverStateV2'].frameId % 40 == 1:
      DM.always_on = params.get_bool("AlwaysOnDM")
      demo_mode = params.get_bool("IsDriverViewEnabled")
      idx = params.get("DriverMonitoringTimeMultiplierIndex", return_default=True)
      new_mult = float(int(idx) + 1) if idx is not None else 3.0
      if new_mult != time_multiplier:
        time_multiplier = new_mult
        DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), always_on=DM.always_on,
                              time_multiplier=time_multiplier)

    if (sm['driverStateV2'].frameId % 6000 == 0 and not demo_mode and
        DM.wheelpos.prob_offseter.filtered_stat.n > DM.settings._WHEELPOS_FILTER_MIN_COUNT and
        DM.wheel_on_right == (DM.wheelpos.prob_offseter.filtered_stat.M > DM.settings._WHEELPOS_THRESHOLD)):
      params.put_bool_nonblocking("IsRhdDetected", DM.wheel_on_right)

def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
