"""
无刷电机控制模块 (motor.py)
===========================
功能：通过 PWM 信号控制无刷电机（ESC电调）。
支持多通道独立控制、急停和安全保护。

作者：E-Competition Team
日期：2026-07-09
版本：v2.0 — 适配 K230 FPIOA + PWM API

依赖：machine.PWM
"""

from machine import PWM
import time


# ============================================================
# 默认引脚配置 — 按你的实际接线修改！
# 引脚号 = IO 编号
# ============================================================
DEFAULT_MOTOR_PINS = {
    1: 42,   # 电机1 → IO42
    2: 52,   # 电机2 → IO52
    3: 53,   # 电机3 → IO53
    4: 43,   # 电机4 → IO43
}


# ============================================================
# 单个电机控制器
# ============================================================
class Motor:
    """
    单个无刷电机控制器（通过 ESC 电调）。

    ESC 电调校准流程（首次使用）：
        1. 上电前 → motor.set_throttle(100)
        2. 电调上电 → 听到 "嘀-嘀" 两声
        3. motor.set_throttle(0) → 听到 "嘀——" 长鸣 → 校准完成

    :使用示例:
        m1 = Motor(channel=1, pin=9)
        m1.init()
        m1.arm()
        m1.set_throttle(50)     # 50% 油门
    """

    FREQ = 50               # ESC 标准：50Hz
    DUTY_MIN_PCT = 5.0      # 1ms → 5%   (零油门)
    DUTY_MAX_PCT = 10.0     # 2ms → 10%  (满油门)

    def __init__(self, channel: int, pin: int = None):
        """
        :param channel: 电机编号（1~4）
        :param pin:     IO 编号
        """
        self.channel = channel
        self.pin_num = pin if pin is not None else DEFAULT_MOTOR_PINS.get(channel, 42)
        self._pwm = None
        self._current_throttle = 0
        self._armed = False
        self._initialized = False

    def init(self) -> bool:
        """初始化 PWM，成功返回 True"""
        try:
            # K230 v3p0: PWM(pin, freq, duty)
            self._pwm = PWM(self.pin_num, self.FREQ, duty=0)
            self._current_throttle = 0
            self._initialized = True
            print("[Motor] 电机{} 初始化成功 (IO{})".format(
                self.channel, self.pin_num))
            return True
        except Exception as e:
            print("[Motor] 电机{} 初始化失败: {}".format(self.channel, e))
            return False

    def deinit(self):
        """关闭 PWM"""
        self.stop()
        if self._pwm:
            self._pwm.deinit()
            self._pwm = None
            self._initialized = False

    def set_throttle(self, percent: float):
        """
        设置油门百分比（0~100）
        """
        percent = max(0.0, min(100.0, percent))
        self._current_throttle = percent

        if self._pwm:
            # 线性映射：0%→5%, 100%→10% 占空比
            duty = self.DUTY_MIN_PCT + (self.DUTY_MAX_PCT - self.DUTY_MIN_PCT) * percent / 100.0
            self._pwm.duty(duty)

    def arm(self):
        """电调解锁：输出零油门"""
        self.set_throttle(0)
        time.sleep_ms(200)
        self._armed = True
        print("[Motor] 电机{} 已解锁".format(self.channel))

    def stop(self):
        """急停（油门归零）"""
        self.set_throttle(0)
        self._armed = False

    def emergency_stop(self):
        """紧急停止：切断 PWM"""
        if self._pwm:
            self._pwm.duty(0)
        self._armed = False
        self._current_throttle = 0

    @property
    def throttle(self) -> float:
        return self._current_throttle

    @property
    def is_armed(self) -> bool:
        return self._armed

    @property
    def is_initialized(self) -> bool:
        return self._initialized


# ============================================================
# 多电机管理器
# ============================================================
class MotorController:
    """
    多路电机统一控制器。

    :使用示例:
        ctrl = MotorController(num_motors=4)
        ctrl.init_all()
        ctrl.arm_all()
        ctrl.set_all_throttle(30)
    """

    def __init__(self, num_motors: int = 4):
        self.num_motors = num_motors
        self.motors = {}
        for i in range(1, num_motors + 1):
            self.motors[i] = Motor(channel=i)

    def init_all(self) -> bool:
        ok = True
        for m in self.motors.values():
            if not m.init():
                ok = False
        return ok

    def deinit_all(self):
        for m in self.motors.values():
            m.deinit()

    def set_motor(self, channel: int, throttle: float):
        if channel in self.motors:
            self.motors[channel].set_throttle(throttle)

    def set_all_throttle(self, throttle: float):
        for m in self.motors.values():
            m.set_throttle(throttle)

    def arm_all(self):
        for m in self.motors.values():
            m.arm()

    def stop_all(self):
        for m in self.motors.values():
            m.stop()

    def emergency_stop_all(self):
        for m in self.motors.values():
            m.emergency_stop()

    def __getitem__(self, channel: int):
        return self.motors[channel]
