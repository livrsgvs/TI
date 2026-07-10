"""
PID 控制模块 (pid_controller.py)
===============================
通用 PID 控制器，适用于循线转向、激光跟踪云台、电机调速等场景。

作者：E-Competition Team
日期：2026-07-10
版本：v1.0

依赖：无（纯 Python 实现）
"""

import time


# ============================================================
# 预置参数 — 按任务场景选择
# ============================================================
PID_PRESETS = {
    # 循线转向：offset→差速转向量
    "line_follow": {
        "kp": 0.80, "ki": 0.02, "kd": 0.15,
        "output_limit": (-100, 100),
        "integral_limit": (-50, 50),
    },
    # 激光跟踪—水平（舵机 pan）
    "laser_pan": {
        "kp": 0.30, "ki": 0.00, "kd": 0.05,
        "output_limit": (-90, 90),
        "integral_limit": (-20, 20),
    },
    # 激光跟踪—垂直（舵机 tilt）
    "laser_tilt": {
        "kp": 0.30, "ki": 0.00, "kd": 0.05,
        "output_limit": (-45, 45),
        "integral_limit": (-20, 20),
    },
    # 电机调速
    "motor_speed": {
        "kp": 1.00, "ki": 0.05, "kd": 0.00,
        "output_limit": (-100, 100),
        "integral_limit": (-30, 30),
    },
}


# ============================================================
# 通用 PID 控制器
# ============================================================
class PIDController:
    """
    通用 PID 控制器。

    特性：
        - 位置式 PID，微分先行（derivative-on-measurement）
        - 积分限幅防饱和（anti-windup）
        - 输出限幅
        - 支持预设参数快速配置

    :使用示例:
        # 方式1：手动参数
        pid = PIDController(kp=0.8, ki=0.02, kd=0.15, output_limit=(-100, 100))
        turn = pid.compute(error=12.5)

        # 方式2：预置参数
        pid = PIDController.from_preset("line_follow")
        turn = pid.compute(error=12.5)
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 setpoint=0.0,
                 output_limit=(-100, 100),
                 integral_limit=(-50, 50),
                 deadband=0.0,
                 name="PID"):
        """
        :param kp:             比例系数
        :param ki:             积分系数
        :param kd:             微分系数
        :param setpoint:       目标值（通常为 0，即期望误差为零）
        :param output_limit:   输出限幅 (min, max)
        :param integral_limit: 积分限幅 (min, max)
        :param deadband:       死区（|error| < deadband 时不积分）
        :param name:           控制器名称（调试用）
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_min, self.output_max = output_limit
        self.integral_min, self.integral_max = integral_limit
        self.deadband = deadband
        self.name = name

        # 内部状态
        self._integral = 0.0
        self._prev_measurement = 0.0
        self._prev_error = 0.0
        self._prev_output = 0.0
        self._last_time = None
        self._initialized = False

    @classmethod
    def from_preset(cls, preset_name, **overrides):
        """
        从预置参数创建 PID 控制器

        :param preset_name: 预置名称（"line_follow"/"laser_pan"/"laser_tilt"/"motor_speed"）
        :param overrides:   覆盖参数，如 kp=1.0
        :return:            PIDController 实例
        """
        preset = PID_PRESETS.get(preset_name)
        if preset is None:
            raise ValueError("未知预设: {}，可选: {}".format(
                preset_name, list(PID_PRESETS.keys())))
        params = preset.copy()
        params["name"] = preset_name
        params.update(overrides)
        return cls(**params)

    # ---- 核心计算 ----

    def compute(self, measurement, dt=None):
        """
        计算 PID 输出。

        :param measurement: 当前测量值（如 line_result.offset）
        :param dt:          时间间隔（秒），None 则自动计算
        :return:            控制输出（已限幅）
        """
        error = self.setpoint - measurement
        self._prev_error = error

        # 自动计算 dt
        now = time.ticks_ms()
        if dt is None:
            if self._last_time is None:
                dt = 0.01  # 首帧默认 10ms
            else:
                dt = time.ticks_diff(now, self._last_time) / 1000.0
                dt = max(0.001, min(dt, 0.5))  # 限幅 1ms~500ms

        # --- P 项 ---
        p_out = self.kp * error

        # --- I 项（死区 + 限幅） ---
        if abs(error) > self.deadband:
            self._integral += error * dt
            # 积分限幅
            self._integral = max(self.integral_min,
                                 min(self.integral_max, self._integral))
        i_out = self.ki * self._integral

        # --- D 项（微分先行，仅对测量值微分） ---
        if self._initialized and dt > 0.001:
            d_out = -self.kd * (measurement - self._prev_measurement) / dt
        else:
            d_out = 0.0

        # --- 合成 + 输出限幅 ---
        output = p_out + i_out + d_out
        output = max(self.output_min, min(self.output_max, output))

        # 积分分离：输出已饱和且误差同向时停止积分
        if (output == self.output_max and error > 0) or \
           (output == self.output_min and error < 0):
            self._integral -= error * dt  # 回退积分

        # 更新状态
        self._prev_measurement = measurement
        self._prev_output = output
        self._last_time = now
        self._initialized = True

        return output

    # ---- 参数设置 ----

    def set_gains(self, kp=None, ki=None, kd=None):
        """在线修改增益（传入 None 则保持不变）"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    def set_setpoint(self, setpoint):
        """设置目标值"""
        self.setpoint = setpoint

    def set_output_limit(self, min_val, max_val):
        """设置输出限幅"""
        self.output_min = min_val
        self.output_max = max_val

    # ---- 状态管理 ----

    def reset(self):
        """重置所有内部状态（丢线/丢目标时调用）"""
        self._integral = 0.0
        self._prev_measurement = 0.0
        self._prev_error = 0.0
        self._prev_output = 0.0
        self._last_time = None
        self._initialized = False

    def is_initialized(self):
        """是否已完成首帧计算"""
        return self._initialized

    # ---- 属性 ----

    @property
    def error(self):
        """上一次计算的误差"""
        return self._prev_error

    @property
    def output(self):
        """上一次计算的输出"""
        return self._prev_output

    @property
    def integral(self):
        """当前积分累计值"""
        return self._integral

    def __repr__(self):
        return ("PID({}, kp={:.3f}, ki={:.3f}, kd={:.3f}, "
                "err={:.2f}, out={:.2f})".format(
                    self.name, self.kp, self.ki, self.kd,
                    self._prev_error, self._prev_output))
