"""
串口通信模块 (uart_com.py)
==========================
功能：与下位机双向串口通信，K230 发命令、收状态。

包格式（12B，小端）：
  ┌────────┬─────────┬──────┬──────┬──────┬──────┬───────┬──────────┐
  │ HEADER │ command │  x   │  y   │ data │flags │ param │ checksum │
  │  2B    │   1B    │  2B  │  2B  │  2B  │  1B  │  1B   │   1B     │
  └────────┴─────────┴──────┴──────┴──────┴──────┴───────┴──────────┘
  HEADER = 0xA5 0x5A

作者：E-Competition Team
日期：2026-07-12
版本：v2.0 — 双包结构解耦 + 工厂函数 + 回调机制

依赖：machine.UART, struct
"""

import struct
from machine import UART
import debug_config


# ============================================================
# 协议常量
# ============================================================
HEADER_H = 0xA5          # 包头高字节
HEADER_L = 0x5A          # 包头低字节
PACKET_SIZE = 12         # 总包长（字节）

# ---- 命令码（K230 → 下位机） ----
CMD_STOP      = 0x00     # 停车
CMD_FORWARD   = 0x01     # 前进（y = 速度 0~100）
CMD_BACKWARD  = 0x02     # 后退（y = 速度 0~100）
CMD_TURN_LEFT = 0x03     # 左转
CMD_TURN_RIGHT= 0x04     # 右转
CMD_STEER     = 0x05     # PID 差速转向（x = 转向量 -100~100, y = 速度 0~100）
CMD_SERVO     = 0x06     # 舵机控制（x = 舵机ID, y = 角度 0~180）
CMD_EMERGENCY = 0xFF     # 急停

# ---- 状态码（下位机 → K230） ----
STATUS_CMD    = 0x10     # 状态包固定命令码

# ---- 状态标志位 ----
FLAG_ARRIVED  = 0x01     # bit0: 到达目标
FLAG_RUNNING  = 0x02     # bit1: 运行中
FLAG_ERROR    = 0x04     # bit2: 故障


# ============================================================
# 命令包：K230 → 下位机
# ============================================================
class CommandPacket:
    """
    K230 发给下位机的命令包。

    字段含义随 command 变化：
      STOP / FORWARD / BACKWARD:
        y = 速度 (0~100)
      STEER:
        x = 转向量 (-100=左满舵, 0=直行, +100=右满舵)
        y = 速度 (0~100)
      SERVO:
        x = 舵机编号 (1 / 2)
        y = 角度 (0~180)
    """

    def __init__(self, command: int = 0, x: int = 0, y: int = 0,
                 data: int = 0, flags: int = 0, param: int = 0):
        self.command = command
        self.x = x                # 有符号 16 位
        self.y = y                # 有符号 16 位
        self.data = data          # 有符号 16 位（预留）
        self.flags = flags        # 标志位
        self.param = param        # 参数（预留）

    # ---- 序列化 ----

    def pack(self) -> bytes:
        """
        将命令包打包为 12 字节（小端）：
          [H L cmd xL xH yL yH dL dH flags param chk]
        """
        raw = struct.pack('<BBBhhhBB',
                          HEADER_H, HEADER_L,
                          self.command,
                          self.x, self.y, self.data,
                          self.flags, self.param)
        # 计算校验和：前 11 字节 XOR
        chk = 0
        for b in raw:
            chk ^= b
        return raw + bytes([chk & 0xFF])

    @classmethod
    def unpack(cls, data: bytes) -> 'CommandPacket':
        """
        从 12 字节解包（通常用于调试/回显）。
        """
        if len(data) < PACKET_SIZE:
            raise ValueError("数据长度不足 12 字节")
        # 校验
        chk_calc = 0
        for b in data[:11]:
            chk_calc ^= b
        if (chk_calc & 0xFF) != data[11]:
            raise ValueError("校验和不匹配")
        hdr_h, hdr_l, cmd, x, y, d, flags, param = struct.unpack('<BBBhhhBB', data[:11])
        return cls(command=cmd, x=x, y=y, data=d, flags=flags, param=param)

    def __repr__(self):
        return "Cmd(cmd=0x{:02X}, x={}, y={}, data={}, flags=0x{:02X}, param={})".format(
            self.command, self.x, self.y, self.data, self.flags, self.param)


# ============================================================
# 状态包：下位机 → K230
# ============================================================
class StatusPacket:
    """
    下位机发给 K230 的状态包（command 固定为 0x10）。

    字段：
      speed    — 当前速度 (cm/s × 100)，如 15000 = 150.00 cm/s
      distance — 累计距离 (cm × 100)
      voltage  — 电池电压 (V × 100)，如 740 = 7.40V
      flags    — 状态位 (bit0=到达, bit1=运行中, bit2=故障)
      error_code — 错误码（仅 bit2=1 时有效）
    """

    def __init__(self):
        self.speed: int = 0        # cm/s × 100
        self.distance: int = 0     # cm × 100
        self.voltage: int = 0      # V × 100
        self.flags: int = 0        # 状态位
        self.error_code: int = 0   # 错误码

    # ---- 属性 ----

    @property
    def is_arrived(self) -> bool:
        return bool(self.flags & FLAG_ARRIVED)

    @property
    def is_running(self) -> bool:
        return bool(self.flags & FLAG_RUNNING)

    @property
    def has_error(self) -> bool:
        return bool(self.flags & FLAG_ERROR)

    @property
    def speed_cms(self) -> float:
        """速度（cm/s）"""
        return self.speed / 100.0

    @property
    def distance_cm(self) -> float:
        """距离（cm）"""
        return self.distance / 100.0

    @property
    def voltage_v(self) -> float:
        """电压（V）"""
        return self.voltage / 100.0

    # ---- 序列化 ----

    def pack(self) -> bytes:
        """
        打包（通常 K230 不发状态包，保留接口）。
        """
        raw = struct.pack('<BBBhhhBB',
                          HEADER_H, HEADER_L,
                          STATUS_CMD,
                          self.speed, self.distance, self.voltage,
                          self.flags, self.error_code)
        chk = 0
        for b in raw:
            chk ^= b
        return raw + bytes([chk & 0xFF])

    @classmethod
    def unpack(cls, data: bytes) -> 'StatusPacket':
        """
        从 12 字节解包下位机状态。
        校验失败返回 None。
        """
        if len(data) < PACKET_SIZE:
            return None
        # XOR 校验
        chk_calc = 0
        for b in data[:11]:
            chk_calc ^= b
        if (chk_calc & 0xFF) != data[11]:
            return None
        hdr_h, hdr_l, cmd, speed, dist, volt, flags, err = struct.unpack(
            '<BBBhhhBB', data[:11])
        if cmd != STATUS_CMD:
            return None  # 不是状态包
        pkt = cls()
        pkt.speed = speed
        pkt.distance = dist
        pkt.voltage = volt
        pkt.flags = flags
        pkt.error_code = err
        return pkt

    def __repr__(self):
        return ("Status(spd={:.2f}cm/s, dist={:.2f}cm, v={:.2f}V, "
                "arr={}, run={}, err={}, code={})".format(
                    self.speed_cms, self.distance_cm, self.voltage_v,
                    self.is_arrived, self.is_running, self.has_error,
                    self.error_code))


# ============================================================
# 工厂函数 — 快捷创建常用 CommandPacket
# ============================================================

def make_stop_command() -> CommandPacket:
    """急停"""
    return CommandPacket(command=CMD_STOP)

def make_emergency_command() -> CommandPacket:
    """紧急停止"""
    return CommandPacket(command=CMD_EMERGENCY)

def make_forward_command(speed: int) -> CommandPacket:
    """
    前进
    :param speed: 速度 0~100
    """
    return CommandPacket(command=CMD_FORWARD, y=speed)

def make_backward_command(speed: int) -> CommandPacket:
    """
    后退
    :param speed: 速度 0~100
    """
    return CommandPacket(command=CMD_BACKWARD, y=speed)

def make_turn_left_command(speed_left: int = 30, speed_right: int = 60) -> CommandPacket:
    """
    左转（差速）
    :param speed_left:  左轮速度
    :param speed_right: 右轮速度
    """
    return CommandPacket(command=CMD_TURN_LEFT, x=speed_left, y=speed_right)

def make_turn_right_command(speed_left: int = 60, speed_right: int = 30) -> CommandPacket:
    """
    右转（差速）
    :param speed_left:  左轮速度
    :param speed_right: 右轮速度
    """
    return CommandPacket(command=CMD_TURN_RIGHT, x=speed_left, y=speed_right)

def make_steer_command(turn: int, speed: int, flags: int = 0) -> CommandPacket:
    """
    PID 差速转向（循线/激光跟踪）
    :param turn:  转向量 -100~100（负=左转, 正=右转）
    :param speed: 基准速度 0~100
    :param flags: 附加标志位
    """
    return CommandPacket(command=CMD_STEER, x=turn, y=speed, flags=flags)

def make_servo_command(servo_id: int, angle: int) -> CommandPacket:
    """
    舵机控制
    :param servo_id: 舵机编号 1=pan, 2=tilt
    :param angle:    角度 0~180
    """
    return CommandPacket(command=CMD_SERVO, x=servo_id, y=angle)


# ============================================================
# UART 管理器
# ============================================================
class UARTManager:
    """
    串口通信管理器。
    只负责收发字节流，不关心包内容。

    用法:
        uart = UARTManager(uart_id=2, baudrate=115200, tx_pin=5, rx_pin=6)
        uart.init()
        uart.register_rx_callback(lambda s: print(f"状态: {s}"))

        # 发送命令
        cmd = make_steer_command(turn=15, speed=40)
        uart.send_command(cmd)

        # 主循环中处理接收
        uart.process_rx()
    """

    def __init__(self, uart_id: int, baudrate: int = 115200,
                 tx_pin: int = 5, rx_pin: int = 6):
        """
        :param uart_id:  UART 编号（K230 可用 UART2=2）
        :param baudrate: 波特率
        :param tx_pin:   TX 引脚
        :param rx_pin:   RX 引脚
        """
        self.uart_id = uart_id
        self.baudrate = baudrate
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin
        self._uart = None
        self._rx_callback = None     # 接收回调: callback(StatusPacket)
        self._rx_buffer = bytearray()

    # ---- 生命周期 ----

    def init(self) -> bool:
        """初始化 UART 硬件。成功返回 True。"""
        try:
            self._uart = UART(self.uart_id, self.baudrate,
                              tx=self.tx_pin, rx=self.rx_pin)
            if debug_config.DEBUG:
                print("[UART] UART{} 初始化成功 @{}bps (TX:{}, RX:{})".format(
                    self.uart_id, self.baudrate, self.tx_pin, self.rx_pin))
            return True
        except Exception as e:
            print("[UART] 初始化失败: {}".format(e))
            return False

    def deinit(self):
        """关闭 UART"""
        if self._uart is not None:
            self._uart.deinit()
        self._uart = None

    # ---- 发送 ----

    def send_command(self, cmd: CommandPacket):
        """
        发送命令包到下位机。
        :param cmd: CommandPacket 实例
        """
        if self._uart is None:
            return
        data = cmd.pack()
        self._uart.write(data)

    def send_raw(self, data: bytes):
        """发送原始字节（用于调试或自定义包）"""
        if self._uart is not None:
            self._uart.write(data)

    # ---- 接收 ----

    def register_rx_callback(self, callback):
        """
        注册接收回调。
        :param callback: callable(StatusPacket)，接收完整状态包时自动调用
        """
        self._rx_callback = callback

    def receive_status(self) -> StatusPacket:
        """
        尝试接收一个状态包（非阻塞）。
        :return: StatusPacket 或 None（无完整数据）
        """
        if self._uart is None:
            return None
        try:
            available = self._uart.any()
            if available <= 0:
                return None
            # 读入缓冲区
            chunk = self._uart.read(available)
            if chunk:
                self._rx_buffer.extend(chunk)
            # 查找完整包
            while len(self._rx_buffer) >= PACKET_SIZE:
                # 找包头
                idx = 0
                while idx <= len(self._rx_buffer) - 2:
                    if self._rx_buffer[idx] == HEADER_H and self._rx_buffer[idx + 1] == HEADER_L:
                        break
                    idx += 1
                if idx > 0:
                    # 丢弃包头前的垃圾字节
                    self._rx_buffer = self._rx_buffer[idx:]
                if len(self._rx_buffer) < PACKET_SIZE:
                    break
                # 尝试解包
                candidate = bytes(self._rx_buffer[:PACKET_SIZE])
                pkt = StatusPacket.unpack(candidate)
                if pkt is not None:
                    self._rx_buffer = self._rx_buffer[PACKET_SIZE:]
                    return pkt
                else:
                    # 校验失败 → 丢掉第一个字节，重新找包头
                    self._rx_buffer = self._rx_buffer[1:]
        except Exception as e:
            if debug_config.DEBUG:
                print(f"[UART] 接收异常: {e}")
        return None

    def process_rx(self) -> StatusPacket:
        """
        处理接收缓冲：取一个完整状态包并触发回调。
        在主循环中每次迭代调用一次。
        :return: 最新状态包 或 None
        """
        pkt = self.receive_status()
        if pkt is not None and self._rx_callback is not None:
            try:
                self._rx_callback(pkt)
            except Exception:
                pass
        return pkt

    # ---- 属性 ----

    @property
    def is_ready(self) -> bool:
        return self._uart is not None
