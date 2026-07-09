"""
无刷电机控制模块 (motor.py)
===========================
功能：通过 PWM 信号控制无刷电机（如无人机电调）的转速。
支持多通道独立控制、急停和安全保护。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：machine.PWM, machine.Pin

注意：
- 无刷电机通常由电调(ESC)驱动，PWM 50Hz，脉宽 1ms~2ms
  对应油门 0%~100%
- 上电时需先输出最小油门（1ms）进行电调校准
"""

from machine import PWM, Pin
import time


# ============================================================
# 默认引脚配置（K230 开发板）
# 用户可根据实际接线修改
# ============================================================
DEFAULT_MOTOR_PINS = {
    1: 42,   # 电机1 PWM 引脚
    2: 43,   # 电机2
    3: 44,   # 电机3
    4: 45,   # 电机4
}


# ============================================================
# 单个电机控制器
# ============================================================
class Motor:
    """
    单个无刷电机控制器（通过电调 ESC）。

    电调校准流程（首次使用或更换电调后）：
        1. 上电前，motor.set_throttle(100)  # 最大油门
        2. 电调上电，听到 "嘀-嘀" 两声
        3. motor.set_throttle(0)            # 最小油门
        4. 听到 "嘀——" 长鸣，校准完成

    :使用示例:
        m1 = Motor(channel=1, pin=42)
        m1.calibrate()          # 首次校准
        m1.set_throttle(50)     # 50% 油门
        m1.stop()               # 停止
    """

    # PWM 参数（电调标准）
    FREQ = 50           # 50Hz
    PWM_MIN = 1000      # 1ms 脉宽（对应占空比）
    PWM_MAX = 2000      # 2ms 脉宽
    PWM_PERIOD = 20000  # 20ms 周期（微秒）

    def __init__(self, channel: int, pin: int = None):
        """
        :param channel: 电机编号（1~4）
        :param pin:     PWM 引脚号
        """
        self.channel = channel
        self.pin_num = pin if pin is not None else DEFAULT_MOTOR_PINS.get(channel, 42)
        self._pwm = None
        self._current_throttle = 0
        self._armed = False
        self._initialized = False

    def init(self) -> bool:
        """
        初始化 PWM 输出（输出最小油门以确保安全）
        :return: 成功返回 True
        """
        try:
            self._pwm = PWM(Pin(self.pin_num), freq=self.FREQ, duty=self.PWM_MIN)
            self._current_throttle = 0
            self._initialized = True
            print(f"[Motor] 电机{self.channel} 初始化成功 (引脚{self.pin_num})")
            return True
        except Exception as e:
            print(f"[Motor] 电机{self.channel} 初始化失败: {e}")
            return False

    def deinit(self):
        """关闭 PWM 输出"""
        self.stop()
        if self._pwm:
            self._pwm.deinit()
            self._pwm = None
            self._initialized = False

    def set_throttle(self, percent: float):
        """
        设置油门百分比
        :param percent: 油门 0~100（自动限幅）
        """
        percent = max(0.0, min(100.0, percent))
        self._current_throttle = percent

        if self._pwm:
            # 线性映射：0% → 1000μs, 100% → 2000μs
            pulse_us = int(self.PWM_MIN + (self.PWM_MAX - self.PWM_MIN) * percent / 100.0)
            self._pwm.duty(pulse_us)

    def arm(self):
        """
        电调解锁：输出零油门并等待
        """
        self.set_throttle(0)
        time.sleep_ms(200)
        self._armed = True
        print(f"[Motor] 电机{self.channel} 已解锁")

    def stop(self):
        """急停（油门归零）"""
        self.set_throttle(0)
        self._armed = False

    def emergency_stop(self):
        """
        紧急停止：立即切断 PWM 输出
        （某些电调需要归零油门而非切断信号）
        """
        if self._pwm:
            self._pwm.duty(0)  # 完全关闭 PWM 信号
        self._armed = False
        self._current_throttle = 0
        print(f"[Motor] 电机{self.channel} 紧急停止！")

    def calibrate(self):
        """
        电调校准流程（仅首次使用）
        注意：此函数会阻塞，请确保电机已断电！
        """
        print(f"[Motor] 电机{self.channel} 开始电调校准...")
        print("  → 请断开电机电源，然后按 Enter 继续...")
        input()

        self.set_throttle(100)
        print("  → 请接通电机电源，等待 '嘀-嘀' 音...")
        time.sleep(3)

        self.set_throttle(0)
        print("  → 等待 '嘀——' 长鸣确认音...")
        time.sleep(2)

        print(f"[Motor] 电机{self.channel} 电调校准完成！")

    @property
    def throttle(self) -> float:
        """当前油门百分比"""
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
    适用于无人机（四轴/六轴）或小车多轮驱动。

    :使用示例:
        ctrl = MotorController(num_motors=4)
        ctrl.init_all()
        ctrl.set_all_throttle(30)   # 所有电机 30% 油门
        ctrl.set_motor(1, 50)       # 电机1 单独 50%
        ctrl.emergency_stop_all()   # 急停
    """

    def __init__(self, num_motors: int = 4):
        """
        :param num_motors: 电机数量
        """
        self.num_motors = num_motors
        self.motors = {}
        for i in range(1, num_motors + 1):
            self.motors[i] = Motor(channel=i)

    def init_all(self) -> bool:
        """初始化所有电机"""
        ok = True
        for m in self.motors.values():
            if not m.init():
                ok = False
        return ok

    def deinit_all(self):
        """关闭所有电机"""
        for m in self.motors.values():
            m.deinit()

    def set_motor(self, channel: int, throttle: float):
        """单独设置某通道油门"""
        if channel in self.motors:
            self.motors[channel].set_throttle(throttle)

    def set_all_throttle(self, throttle: float):
        """统一设置所有电机油门"""
        for m in self.motors.values():
            m.set_throttle(throttle)

    def arm_all(self):
        """解锁所有电机"""
        for m in self.motors.values():
            m.arm()

    def stop_all(self):
        """停止所有电机（油门归零）"""
        for m in self.motors.values():
            m.stop()

    def emergency_stop_all(self):
        """紧急停止所有电机"""
        for m in self.motors.values():
            m.emergency_stop()

    def get_throttles(self) -> dict:
        """获取所有电机当前油门"""
        return {ch: m.throttle for ch, m in self.motors.items()}

    def __getitem__(self, channel: int) -> Motor:
        return self.motors[channel]
