"""
激光头驱动模块 (laser.py)
=========================
功能：通过 GPIO 控制激光头开关。
高电平 → 激光亮，低电平 → 激光灭。

作者：E-Competition Team
日期：2026-07-12
版本：v1.0

依赖：machine.Pin

K230v3p0 可用引脚参考（避开 UART2 IO5/IO6）：
  IO20 (Pin 5), IO27 (Pin15), IO42 (Pin 9), IO43 (Pin35),
  IO52 (Pin29), IO53 (Pin30)
"""

from machine import Pin


class Laser:
    """
    激光头驱动器。

    用法:
        laser = Laser(pin=20)   # IO20
        laser.on()              # 打开
        laser.off()             # 关闭
        laser.toggle()          # 翻转
        print(laser.is_on)      # 查询状态
    """

    def __init__(self, pin: int = 20):
        """
        :param pin: GPIO 引脚编号（IO 号，如 20 = IO20 = 物理 Pin5）
        """
        self._pin_num = pin
        self._pin = Pin(pin, Pin.OUT)
        self._pin.value(1)         # 默认关闭
        self._state = True         # False=灭, True=亮

    # ---- 开关控制 ----

    def on(self):
        """打开激光头（高电平）"""
        self._pin.value(1)
        self._state = True

    def off(self):
        """关闭激光头（低电平）"""
        self._pin.value(0)
        self._state = False

    def toggle(self):
        """翻转激光头状态"""
        if self._state:
            self.off()
        else:
            self.on()

    def set(self, enable: bool):
        """
        :param enable: True=打开, False=关闭
        """
        if enable:
            self.on()
        else:
            self.off()

    # ---- 状态查询 ----

    @property
    def is_on(self) -> bool:
        """激光头是否亮着"""
        return self._state

    @property
    def is_off(self) -> bool:
        """激光头是否灭着"""
        return not self._state

    @property
    def pin(self) -> int:
        """返回所用 GPIO 编号"""
        return self._pin_num

    # ---- 资源释放 ----

    def deinit(self):
        """释放 GPIO 资源（关闭激光 + 复位引脚）"""
        self._pin.value(0)
        self._state = False
        # Pin 对象没有显式 deinit，设为输入模式释放
        try:
            self._pin.init(Pin.IN)
        except Exception:
            pass

    def __repr__(self):
        return "Laser(pin=IO{}, state={})".format(
            self._pin_num, "ON" if self._state else "OFF")
