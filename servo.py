"""
舵机控制模块 (servo.py)
======================
功能：通过 PWM 信号控制舵机角度（用于云台、机械臂等）。
支持多通道独立控制、角度范围限定和速度控制。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：machine.PWM, machine.Pin

注意：
- 标准舵机：PWM 50Hz，脉宽 0.5ms~2.5ms → 0°~180°
- 部分数字舵机支持更大的角度范围或更高的频率
"""

from machine import PWM, Pin
import time


# ============================================================
# 默认引脚配置
# ============================================================
DEFAULT_SERVO_PINS = {
    1: 10,   # 舵机1（如云台 Yaw）
    2: 11,   # 舵机2（如云台 Pitch）
    3: 12,   # 舵机3（如机械臂底座）
    4: 13,   # 舵机4
    5: 14,   # 舵机5
    6: 15,   # 舵机6
}


# ============================================================
# 单个舵机控制器
# ============================================================
class Servo:
    """
    单个舵机控制器。

    :使用示例:
        s = Servo(channel=1, pin=10)
        s.init()
        s.set_angle(90)       # 转到 90°
        s.smooth_move(0, 180, duration_ms=1000)  # 1秒内从0°平滑转到180°
    """

    # 默认 PWM 参数
    FREQ = 50                # 50Hz（标准舵机）
    PERIOD_US = 20000        # 20ms 周期
    MIN_US = 500             # 0° 对应脉宽
    MAX_US = 2500            # 180° 对应脉宽
    DEFAULT_ANGLE_MIN = 0    # 默认最小角度
    DEFAULT_ANGLE_MAX = 180  # 默认最大角度

    def __init__(self, channel: int, pin: int = None,
                 angle_min: int = None, angle_max: int = None,
                 min_us: int = None, max_us: int = None):
        """
        :param channel:   舵机编号
        :param pin:       PWM 引脚号
        :param angle_min: 最小角度限制（默认 0°）
        :param angle_max: 最大角度限制（默认 180°）
        :param min_us:    最小脉宽 μs（默认 500）
        :param max_us:    最大脉宽 μs（默认 2500）
        """
        self.channel = channel
        self.pin_num = pin if pin is not None else DEFAULT_SERVO_PINS.get(channel, 10)
        self.angle_min = angle_min if angle_min is not None else self.DEFAULT_ANGLE_MIN
        self.angle_max = angle_max if angle_max is not None else self.DEFAULT_ANGLE_MAX
        self.min_us = min_us if min_us is not None else self.MIN_US
        self.max_us = max_us if max_us is not None else self.MAX_US

        self._pwm = None
        self._current_angle = 0.0
        self._initialized = False

    def init(self, start_angle: float = 90.0) -> bool:
        """
        初始化舵机 PWM
        :param start_angle: 初始角度（默认 90°）
        :return: 成功返回 True
        """
        try:
            self._pwm = PWM(Pin(self.pin_num), freq=self.FREQ, duty=0)
            self._initialized = True
            self.set_angle(start_angle)
            print(f"[Servo] 舵机{self.channel} 初始化成功 (引脚{self.pin_num})")
            return True
        except Exception as e:
            print(f"[Servo] 舵机{self.channel} 初始化失败: {e}")
            return False

    def deinit(self):
        """关闭 PWM"""
        if self._pwm:
            self._pwm.deinit()
            self._pwm = None
            self._initialized = False

    def set_angle(self, angle: float):
        """
        设置舵机角度
        :param angle: 目标角度（会被限幅到 [angle_min, angle_max]）
        """
        angle = max(self.angle_min, min(self.angle_max, angle))
        self._current_angle = angle

        if self._pwm:
            # 线性插值计算脉宽
            pulse_us = int(self.min_us +
                           (self.max_us - self.min_us) *
                           (angle - self.DEFAULT_ANGLE_MIN) /
                           (self.DEFAULT_ANGLE_MAX - self.DEFAULT_ANGLE_MIN))
            self._pwm.duty(pulse_us)

    def get_angle(self) -> float:
        """获取当前角度"""
        return self._current_angle

    def smooth_move(self, from_angle: float, to_angle: float,
                    duration_ms: int = 500, steps: int = 50):
        """
        平滑移动到目标角度
        :param from_angle:   起始角度
        :param to_angle:     目标角度
        :param duration_ms:  总时长（毫秒）
        :param steps:        步数（越大越平滑）
        """
        step_delay = duration_ms // steps
        delta = (to_angle - from_angle) / steps

        for i in range(steps + 1):
            self.set_angle(from_angle + delta * i)
            time.sleep_ms(step_delay)

    def center(self):
        """回中（90°）"""
        self.set_angle(90.0)

    def stop(self):
        """停转（保持当前位置，不切断 PWM）"""
        pass

    def release(self):
        """释放舵机（切断 PWM 信号，舵机卸力）"""
        if self._pwm:
            self._pwm.duty(0)

    def set_angle_range(self, min_angle: int, max_angle: int):
        """设置角度限幅"""
        self.angle_min = min_angle
        self.angle_max = max_angle

    @property
    def is_initialized(self) -> bool:
        return self._initialized


# ============================================================
# 多舵机管理器
# ============================================================
class ServoController:
    """
    多路舵机统一控制器。
    适用于云台（2轴）、机械臂（多轴）等。

    :使用示例:
        sc = ServoController(num_servos=2)
        sc.init_all()
        sc.set_angle(1, 90)    # 云台 Yaw 到 90°
        sc.set_angle(2, 45)    # 云台 Pitch 到 45°
    """

    def __init__(self, num_servos: int = 2):
        """
        :param num_servos: 舵机数量
        """
        self.num_servos = num_servos
        self.servos = {}
        for i in range(1, num_servos + 1):
            self.servos[i] = Servo(channel=i)

    def init_all(self) -> bool:
        """初始化所有舵机"""
        ok = True
        for s in self.servos.values():
            if not s.init():
                ok = False
        return ok

    def deinit_all(self):
        """关闭所有舵机"""
        for s in self.servos.values():
            s.deinit()

    def set_angle(self, channel: int, angle: float):
        """设置指定舵机角度"""
        if channel in self.servos:
            self.servos[channel].set_angle(angle)

    def set_all_angles(self, angles: dict):
        """批量设置角度 {channel: angle}"""
        for ch, ang in angles.items():
            self.set_angle(ch, ang)

    def center_all(self):
        """所有舵机回中"""
        for s in self.servos.values():
            s.center()

    def release_all(self):
        """释放所有舵机"""
        for s in self.servos.values():
            s.release()

    def get_angles(self) -> dict:
        """获取所有舵机角度"""
        return {ch: s.get_angle() for ch, s in self.servos.items()}

    def __getitem__(self, channel: int) -> Servo:
        return self.servos[channel]
