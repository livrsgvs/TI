"""
舵机控制模块 (servo.py)
======================
功能：通过 PWM 信号控制舵机角度（云台、机械臂等）。
支持多通道独立控制、角度限幅、平滑移动。

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
DEFAULT_SERVO_PINS = {
    1: 20,   # 舵机1 → IO20
    2: 27,   # 舵机2 → IO27
    3: 30,   # 舵机3 → IO30
    4: 28,   # 舵机4 → IO28
    5: 31,   # 舵机5 → IO31
    6: 29,   # 舵机6 → IO29
}


# ============================================================
# 单个舵机控制器
# ============================================================
class Servo:
    """
    单个舵机控制器。

    :使用示例:
        s = Servo(channel=1, pin=5)
        s.init()
        s.set_angle(90)
        s.center()
    """

    FREQ = 50               # 标准舵机 50Hz
    DUTY_MIN_PCT = 2.5      # 0°   → 2.5%
    DUTY_MAX_PCT = 12.5     # 180° → 12.5%

    def __init__(self, channel: int, pin: int = None,
                 angle_min: int = 0, angle_max: int = 180):
        """
        :param channel:   舵机编号
        :param pin:       IO 编号
        :param angle_min: 最小角度（默认 0°）
        :param angle_max: 最大角度（默认 180°）
        """
        self.channel = channel
        self.pin_num = pin if pin is not None else DEFAULT_SERVO_PINS.get(channel, 20)
        self.angle_min = angle_min
        self.angle_max = angle_max

        self._pwm = None
        self._current_angle = 0.0
        self._initialized = False

    def init(self, start_angle: float = 90.0) -> bool:
        """初始化 PWM，成功返回 True"""
        try:
            # K230 v3p0: PWM(pin, freq, duty)
            self._pwm = PWM(self.pin_num, self.FREQ, duty=0)
            self._initialized = True
            self.set_angle(start_angle)
            print("[Servo] 舵机{} 初始化成功 (IO{})".format(
                self.channel, self.pin_num))
            return True
        except Exception as e:
            print("[Servo] 舵机{} 初始化失败: {}".format(self.channel, e))
            return False

    def deinit(self):
        """关闭 PWM"""
        self.release()
        if self._pwm:
            self._pwm.deinit()
            self._pwm = None
            self._initialized = False

    def set_angle(self, angle: float):
        """设置角度（限幅到 [angle_min, angle_max]）"""
        angle = max(self.angle_min, min(self.angle_max, angle))
        self._current_angle = angle

        if self._pwm:
            # 线性映射：0°→2.5%, 180°→12.5%
            duty = self.DUTY_MIN_PCT + (self.DUTY_MAX_PCT - self.DUTY_MIN_PCT) * (angle / 180.0)
            self._pwm.duty(duty)

    def get_angle(self) -> float:
        return self._current_angle

    def center(self):
        """回中（90°）"""
        self.set_angle(90.0)

    def smooth_move(self, from_angle: float, to_angle: float,
                    duration_ms: int = 500, steps: int = 50):
        """平滑移动"""
        step_delay = duration_ms // steps
        delta = (to_angle - from_angle) / steps
        for i in range(steps + 1):
            self.set_angle(from_angle + delta * i)
            time.sleep_ms(step_delay)

    def release(self):
        """释放舵机（切断 PWM）"""
        if self._pwm:
            self._pwm.duty(0)

    @property
    def is_initialized(self) -> bool:
        return self._initialized


# ============================================================
# 多舵机管理器
# ============================================================
class ServoController:
    """
    多路舵机统一控制器。

    :使用示例:
        sc = ServoController(num_servos=2)
        sc.init_all()
        sc.set_angle(1, 90)
    """

    def __init__(self, num_servos: int = 2):
        self.num_servos = num_servos
        self.servos = {}
        for i in range(1, num_servos + 1):
            self.servos[i] = Servo(channel=i)

    def init_all(self) -> bool:
        ok = True
        for s in self.servos.values():
            if not s.init():
                ok = False
        return ok

    def deinit_all(self):
        for s in self.servos.values():
            s.deinit()

    def set_angle(self, channel: int, angle: float):
        if channel in self.servos:
            self.servos[channel].set_angle(angle)

    def center_all(self):
        for s in self.servos.values():
            s.center()

    def release_all(self):
        for s in self.servos.values():
            s.release()

    def get_angles(self) -> dict:
        return {ch: s.get_angle() for ch, s in self.servos.items()}

    def __getitem__(self, channel: int):
        return self.servos[channel]
