"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import cereal.messaging as messaging
from cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

# 状态定义
ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)  # 激活状态:正在执行弯道控制
ENABLED_STATES = (VisionState.enabled, VisionState.overriding, *ACTIVE_STATES)  # 启用状态:功能已启用

# 状态转换阈值参数
_ENTERING_PRED_LAT_ACC_TH = 1.3  # 预测横向加速度阈值,触发进入弯道状态(m/s²)
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1  # 预测横向加速度阈值,中止进入状态(m/s²)

_TURNING_LAT_ACC_TH = 1.6  # 当前横向加速度阈值,触发转向状态(m/s²)

_LEAVING_LAT_ACC_TH = 1.3  # 当前横向加速度阈值,触发离开弯道状态(m/s²)
_FINISH_LAT_ACC_TH = 1.1  # 当前横向加速度阈值,结束转弯周期(m/s²)

_A_LAT_REG_MAX = 1.25  #  原始是2 太大最大允许横向加速度(m/s²),用于计算目标速度

_NO_OVERSHOOT_TIME_HORIZON = 4.  # 时间窗口(秒),用于基于目标加速度计算期望速度

# 进入状态(ENTERING)的平滑减速度查找表
# 根据前方预测的最大横向加速度,插值计算平滑减速度
_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.]  # 减速度值(m/s²),范围从-0.2到-1.0
_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.]  # 对应的预测横向加速度绝对值(m/s²)

# 转向状态(TURNING)的加速度查找表
# 根据当前横向加速度,插值计算舒适的目标加速度
_TURNING_ACC_V = [0.5, 0., -0.4]  # 加速度值(m/s²),范围从0.5到-0.4
_TURNING_ACC_BP = [1.5, 2.3, 3.]  # 对应的当前横向加速度绝对值(m/s²)

_LEAVING_ACC = 0.5  # 离开弯道时的舒适加速度(m/s²),用于恢复速度


class SmartCruiseControlVision:
  """
  基于视觉预测的智能巡航控制类

  核心功能:通过预测前方弯道的横向加速度,提前调整车辆速度,实现:
  1. 进入弯道前平滑减速(entering状态)
  2. 转弯时保持合适速度(turning状态)
  3. 离开弯道后恢复速度(leaving状态)

  状态机包含6个状态:
  - disabled: 禁用状态
  - enabled: 启用状态(等待进入弯道)
  - entering: 进入弯道(减速准备)
  - turning: 正在转弯(保持速度)
  - leaving: 离开弯道(加速恢复)
  - overriding: 被覆盖(用户手动控制)
  """
  v_target: float = 0  # 目标速度(m/s),基于最大曲率计算
  a_target: float = 0.  # 目标加速度(m/s²),根据状态计算
  v_ego: float = 0.  # 当前车辆速度(m/s)
  a_ego: float = 0.  # 当前车辆加速度(m/s²)
  output_v_target: float = V_CRUISE_UNSET  # 输出的目标速度
  output_a_target: float = 0.  # 输出的目标加速度

  # 基于曲率的直接控制算法输出(用于对比)
  v_target_curvature_based: float = 0.  # 基于曲率的目标速度
  a_target_curvature_based: float = 0.  # 基于曲率的平滑加速度
  output_v_target_curvature_based: float = V_CRUISE_UNSET  # 基于曲率的输出目标速度
  output_a_target_curvature_based: float = 0.  # 基于曲率的输出目标加速度

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.enabled = self.params.get_bool("SmartCruiseControlVision")
    self.v_cruise_setpoint = 0.

    self.state = VisionState.disabled
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.
    # -YJ-
    self.max_curve = 0.
    self.set_long_enabled = True # 为了方便调试

    # 基于曲率的算法状态
    self.v_target_curvature_based_raw = 0.
    self.prev_v_target_curvature = 0.  # 上一帧的目标速度(用于平滑)
    self.prev_a_target_curvature = 0.  # 上一帧的目标加速度(用于平滑)
    self.max_curvature_ahead = 0.  # 前方最大曲率

  def get_a_target_from_control(self) -> float:
    """获取目标加速度"""
    return self.a_target

  def get_v_target_from_control(self) -> float:
    """
    获取目标速度

    仅在激活状态(entering/turning/leaving)时输出目标速度
    计算公式:max(v_target, MIN_V) + a_target * 时间窗口(4秒)
    这样可以避免超调,平滑地达到目标速度
    """
    if self.is_active:
      return max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON

    return V_CRUISE_UNSET

  def _update_params(self) -> None:
    """定期更新配置参数(从参数存储中读取)"""
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    """
    计算当前和预测的横向加速度,以及目标速度

    计算步骤:
    1. 当前横向加速度 = v_ego² * |curvature|(基于当前速度和曲率)
    2. 预测横向加速度 = 从模型预测路径中提取最大值(rate_plan * vel_plan)
    3. 目标速度 = 基于最大曲率计算,确保横向加速度不超过_A_LAT_REG_MAX(2.0 m/s²)
    """
    # if not self.long_enabled and not self.set_long_enabled:
    #   return
    # else:
    # 从模型获取预测的角速度(z轴旋转率)和速度
    rate_plan = np.array(np.abs(sm['modelV2'].orientationRate.z))
    vel_plan = np.array(sm['modelV2'].velocity.x)

    # 计算当前横向加速度:a_lat = v² * κ(曲率)
    self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

    # 计算预测路径上的横向加速度,取最大值
    predicted_lat_accels = rate_plan * vel_plan
    self.max_pred_lat_acc = np.amax(predicted_lat_accels)

    # 基于当前速度计算最大曲率
    v_ego = max(self.v_ego, 0.1)  # 确保值大于0,避免除零错误
    max_curve = self.max_pred_lat_acc / (v_ego**2)
    self.max_curve = max_curve

    # 计算目标速度:基于最大曲率和最大允许横向加速度
    # v_target = sqrt(a_lat_max / κ_max)
    self.v_target = (_A_LAT_REG_MAX / max_curve) ** 0.5
    self.v_target = min(self.v_target, 150/3.6)

  # --------计算基于曲率的算法结果-------
  #   new
  def _update_calculations_curvature_based(self, sm: messaging.SubMaster) -> None:
    """
    基于曲率的直接速度控制算法

    设计初衷:直接基于前方弯道的曲率和最大横向过载来控制车速
    计算步骤:
    1. 从模型预测路径中提取曲率信息
    2. 找到前方最大曲率
    3. 基于最大曲率和最大横向加速度计算目标速度
    4. 考虑平滑性,计算平滑的加速度

    与现有算法的区别:
    - 直接使用曲率,而非通过横向加速度反推
    - 不使用状态机,直接基于曲率计算
    - 输出平滑的加速度,而非通过状态插值
    """
    if not self.long_enabled:
      return

    # 从模型获取预测路径信息
    velocity = sm['modelV2'].velocity
    orientation_rate = sm['modelV2'].orientationRate

    if len(velocity.x) == 0:
      return

    # 获取预测路径的速度和角速度
    vel_plan = np.array(velocity.x)  # 预测速度序列(x方向,纵向速度)
    orientation_rate_z = np.array(orientation_rate.z)  # z轴角速度(曲率相关)

    # 计算当前横向加速度
    self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

    # 方法1:从角速度和速度计算曲率
    # 曲率 κ = yaw_rate / velocity
    v_plan = np.maximum(vel_plan, 0.1)  # 避免除零
    curvatures = np.abs(orientation_rate_z / v_plan)

    # 方法2:从controlsState获取当前曲率作为参考
    # 注意:这里主要使用预测路径的曲率,当前曲率仅作参考

    # 找到前方最大曲率(考虑预测时间窗口,例如前5秒)
    lookahead_time = 5.0  # 向前看5秒
    time_idxs = np.linspace(0, lookahead_time, len(curvatures))
    # 只考虑前方一定时间内的曲率
    valid_mask = time_idxs <= lookahead_time
    if np.any(valid_mask):
      self.max_curvature_ahead = np.amax(curvatures[valid_mask])
    else:
      self.max_curvature_ahead = np.amax(curvatures) if len(curvatures) > 0 else 0.0

    # 基于最大曲率和最大允许横向加速度计算目标速度
    # v_target = sqrt(a_lat_max / κ_max)
    # 确保曲率不为零
    max_curvature = max(self.max_curvature_ahead, 1e-6)
    self.v_target_curvature_based_raw = (_A_LAT_REG_MAX / max_curvature) ** 0.5

    # 确保目标速度在合理范围内
    self.v_target_curvature_based = max(self.v_target_curvature_based_raw, MIN_V)
    self.v_target_curvature_based = min(self.v_target_curvature_based_raw, self.v_cruise_setpoint if self.v_cruise_setpoint > 0 else 100.0)

    # 计算平滑的加速度,考虑控制的平滑性
    # 使用一阶低通滤波器平滑速度变化
    smooth_tau = 2.0  # 平滑时间常数(秒)
    dt = DT_MDL

    # 计算达到目标速度所需的加速度
    v_error = self.v_target_curvature_based - self.v_ego
    # 使用平滑过渡,避免突变
    alpha = 1 - np.exp(-dt / smooth_tau) if smooth_tau > 0 else 1
    smooth_v_target = alpha * self.v_target_curvature_based + (1 - alpha) * self.prev_v_target_curvature

    # 计算加速度:基于速度误差和平滑后的目标速度
    # a = (v_target_smooth - v_ego) / time_horizon
    time_horizon = 3.0  # 3秒时间窗口
    a_raw = (smooth_v_target - self.v_ego) / time_horizon

    # 限制加速度变化率,保证平滑性
    max_accel_rate = 0.2  # 最大加速度变化率 m/s^3
    a_delta = np.clip(a_raw - self.prev_a_target_curvature, -max_accel_rate * dt, max_accel_rate * dt)
    self.a_target_curvature_based = self.prev_a_target_curvature + a_delta

    # 限制加速度范围
    max_accel = 2.0  # 最大加速度 m/s^2
    min_accel = -3.0  # 最大减速度 m/s^2
    self.a_target_curvature_based = np.clip(self.a_target_curvature_based, min_accel, max_accel)

    # 更新历史值
    self.prev_v_target_curvature = smooth_v_target
    self.prev_a_target_curvature = self.a_target_curvature_based

  def get_v_target_curvature_based(self) -> float:
    """
    获取基于曲率的目标速度

    仅在检测到前方有弯道时输出(曲率大于阈值)
    """
    # if self.is_active:
    #   return max(self.v_target_curvature_based, MIN_V)
    return self.v_target_curvature_based

  def get_a_target_curvature_based(self) -> float:
    """获取基于曲率的平滑加速度"""
    return self.a_target_curvature_based

  def _update_state_machine(self) -> tuple[bool, bool]:
    """
    更新状态机

    状态转换规则:
    - DISABLED → ENABLED/OVERRIDING: 当long_enabled且enabled时
    - ENABLED → ENTERING: 当预测横向加速度 ≥ 1.3 且速度 > MIN_V
    - ENTERING → TURNING: 当当前横向加速度 ≥ 1.6
    - ENTERING → ENABLED: 当预测横向加速度 < 1.1(中止进入)
    - TURNING → LEAVING: 当当前横向加速度 ≤ 1.3
    - LEAVING → TURNING: 当当前横向加速度 ≥ 1.6(重新进入转弯)
    - LEAVING → ENABLED: 当当前横向加速度 < 1.1(完成转弯)

    返回:
    - enabled: 功能是否启用
    - active: 是否处于激活状态(entering/turning/leaving)
    """
    if self.state != VisionState.disabled:
      # 在非禁用状态下,纵向控制和功能禁用始终优先
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        # ENABLED状态:等待进入弯道
        if self.state == VisionState.enabled:
          # 速度过低时不进入弯道控制循环
          if self.v_ego <= MIN_V:
            pass
          # 如果预测到前方有显著横向加速度,则进入entering状态
          elif self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.entering

        # OVERRIDING状态:用户手动控制
        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        # ENTERING状态:进入弯道,准备减速
        elif self.state == VisionState.entering:
          # 如果当前横向加速度超过阈值,转换到turning状态
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # 如果预测横向加速度下降,中止进入状态
          elif self.max_pred_lat_acc < _ABORT_ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.enabled

        # TURNING状态:正在转弯
        elif self.state == VisionState.turning:
          # 如果当前横向加速度下降到阈值以下,转换到leaving状态
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self.state = VisionState.leaving

        # LEAVING状态:离开弯道,恢复速度
        elif self.state == VisionState.leaving:
          # 如果当前横向加速度重新超过阈值,回到turning状态
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # 如果当前横向加速度下降到阈值以下,完成转弯周期
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH:
            self.state = VisionState.enabled

    # DISABLED状态:功能禁用
    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def _update_solution(self) -> float:
    """
    根据当前状态计算目标加速度

    各状态的目标加速度策略:
    - DISABLED/ENABLED/OVERRIDING: 使用当前加速度a_ego(不干预)
    - ENTERING: 根据预测横向加速度插值计算平滑的纵向减速度(-0.2到-1.0 m/s²)
    - TURNING: 根据当前横向加速度插值计算舒适的纵向加速度(0.5到-0.4 m/s²)
    - LEAVING: 固定加速度0.5 m/s²(恢复速度)
    """
    # DISABLED, ENABLED, OVERRIDING状态:不干预,使用当前加速度
    if self.state not in ACTIVE_STATES:
      a_target = self.a_ego
    # ENTERING状态:平滑减速准备进入弯道
    elif self.state == VisionState.entering:
      # 根据预测横向加速度插值计算减速度
      a_target = np.interp(self.max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V)
    # TURNING状态:根据当前横向加速度提供舒适的加速度
    elif self.state == VisionState.turning:
      # 横向加速度越大,加速度越小(甚至为负),保证舒适性
      a_target = np.interp(self.current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V)
    # LEAVING状态:提供舒适的加速度恢复速度
    elif self.state == VisionState.leaving:
      a_target = _LEAVING_ACC
    else:
      raise NotImplementedError(f"SCC-V state not supported: {self.state}")

    return a_target

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    """
    主更新方法,每帧调用

    执行流程:
    1. 更新输入参数(速度、加速度等)
    2. 更新配置参数(定期从参数存储读取)
    3. 计算横向加速度(当前和预测)
    4. 更新状态机(根据条件转换状态)
    5. 计算目标加速度(根据状态)
    6. 输出目标速度和加速度

    参数:
    - sm: 消息订阅器,包含模型预测和车辆状态信息
    - long_enabled: 纵向控制是否启用
    - long_override: 是否被用户手动覆盖
    - v_ego: 当前车辆速度(m/s)
    - a_ego: 当前车辆加速度(m/s²)
    - v_cruise_setpoint: 巡航速度设定值(m/s)
    """
    # 更新输入参数
    self.long_enabled = long_enabled or self.set_long_enabled

    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    # 更新配置参数
    self._update_params()
    # 计算横向加速度和目标速度
    self._update_calculations(sm)

    # 更新状态机
    self.is_enabled, self.is_active = self._update_state_machine()
    # 根据状态计算目标加速度
    self.a_target = self._update_solution()

    # 输出目标速度和加速度
    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    # --------计算基于曲率的算法结果-------
    self._update_calculations_curvature_based(sm)
    # 输出基于曲率的算法结果(用于对比)
    self.output_v_target_curvature_based = self.get_v_target_curvature_based()
    self.output_a_target_curvature_based = self.get_a_target_curvature_based()

    # 更新帧计数
    self.frame += 1
