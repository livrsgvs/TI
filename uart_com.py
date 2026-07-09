"""
串口通信模块 (uart_com.py)
==========================
功能：与下位机（如MSP430/STM32）进行双向串口通信。
支持自定义数据包结构，提供打包/解包、发送/接收功能。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：machine.UART
"""

from machine import UART
import struct
import time

# K230 默认 UART 引脚映射
UART_PIN_MAP = {
    1: {'tx': 3,  'rx': 4},
    2: {'tx': 5,  'rx': 6},    # UART2: IO5/IO6
    3: {'tx': 33, 'rx': 34},
}


# ============================================================
# 数据包结构体定义
# ============================================================
class DataPacket:
    """
    串口通信数据包结构体

    数据包格式（可按需扩展）：
    +--------+----------+----------+-----------+---------+-------+----------+
    | HEADER | target_x | target_y | target_type | command | flags | checksum |
    +--------+----------+----------+-----------+---------+-------+----------+
    | 2B     | 2B       | 2B       | 1B        | 1B      | 1B    | 1B       |
    +--------+----------+----------+-----------+---------+-------+----------+

    默认包大小：9 字节
    """

    # 数据包头标识
    HEADER = b'\xA5\x5A'

    # 包格式（little-endian）：h=int16, B=uint8
    FORMAT = '<hhBBB'

    def __init__(self):
        """初始化数据包，所有字段置零"""
        self.target_x = 0       # 目标X坐标（像素或实际坐标*100）
        self.target_y = 0       # 目标Y坐标
        self.target_type = 0    # 目标类型（0:无 1:红 2:绿 3:蓝 4:二维码 5:条形码）
        self.command = 0        # 命令字（0:停止 1:前进 2:后退 3:左转 4:右转）
        self.flags = 0          # 标志位（bit0:检测到目标 bit1:跟踪中 bit2:到达）

    def pack(self) -> bytes:
        """
        打包为字节流，自动添加包头与校验和
        :return: 打包后的字节流
        :rtype: bytes
        """
        payload = struct.pack(self.FORMAT,
                              self.target_x, self.target_y,
                              self.target_type, self.command, self.flags)
        checksum = self._calc_checksum(payload)
        return self.HEADER + payload + bytes([checksum])

    def unpack(self, data: bytes) -> bool:
        """
        从字节流解包，自动验证包头与校验和
        :param data: 接收到的字节流
        :return: 解包成功返回 True
        :rtype: bool
        """
        fmt_size = struct.calcsize(self.FORMAT)
        min_len = len(self.HEADER) + fmt_size + 1
        if len(data) < min_len:
            return False
        if data[:2] != self.HEADER:
            return False

        payload = data[2:2 + fmt_size]
        rx_checksum = data[2 + fmt_size]

        if self._calc_checksum(payload) != rx_checksum:
            return False

        (self.target_x, self.target_y, self.target_type,
         self.command, self.flags) = struct.unpack(self.FORMAT, payload)
        return True

    @staticmethod
    def _calc_checksum(data: bytes) -> int:
        """异或校验和"""
        c = 0
        for b in data:
            c ^= b
        return c & 0xFF

    def __repr__(self):
        return "DataPacket(x={}, y={}, type={}, cmd={}, flags={:08b})".format(
            self.target_x, self.target_y, self.target_type, self.command, self.flags)


# ============================================================
# 串口通信管理类
# ============================================================
class UARTManager:
    """
    串口通信管理器
    封装 UART 初始化、发送、接收及回调机制。

    :使用示例:
        uart = UARTManager(uart_id=1, baudrate=115200, tx_pin=3, rx_pin=4)
        uart.init()
        pkt = DataPacket(); pkt.command = 1
        uart.send(pkt)
    """

    def __init__(self, uart_id: int = 1, **kwargs):
        """
        :param uart_id: UART 编号（K230 通常为 1 或 3）
        :param kwargs:
            baudrate    - 波特率（默认 115200）
            bits        - 数据位（默认 8）
            parity      - 校验位（默认 None）
            stop        - 停止位（默认 1）
            tx_pin      - 发送引脚号（默认查 UART_PIN_MAP）
            rx_pin      - 接收引脚号（默认查 UART_PIN_MAP）
        """
        self.uart_id = uart_id
        self.baudrate = kwargs.get('baudrate', 115200)
        self.bits = kwargs.get('bits', 8)
        self.parity = kwargs.get('parity', None)
        self.stop = kwargs.get('stop', 1)
        self.timeout = kwargs.get('timeout', 10)
        self.rx_buf_size = kwargs.get('rx_buf_size', 256)

        # 引脚配置（优先用传入的，否则查 UART_PIN_MAP）
        pin_defaults = UART_PIN_MAP.get(uart_id, {'tx': 3, 'rx': 4})
        self.tx_pin = kwargs.get('tx_pin', pin_defaults['tx'])
        self.rx_pin = kwargs.get('rx_pin', pin_defaults['rx'])

        self._uart = None
        self._rx_callback = None

    def init(self) -> bool:
        """初始化 UART 硬件，成功返回 True"""
        try:
            self._uart = UART(self.uart_id, self.baudrate,
                              tx=self.tx_pin, rx=self.rx_pin)
            print("[UART] UART{} 初始化成功 @{}bps (TX:{}, RX:{})".format(
                self.uart_id, self.baudrate, self.tx_pin, self.rx_pin))
            return True
        except Exception as e:
            print("[UART] 初始化失败: {}".format(e))
            return False

    def deinit(self):
        """关闭 UART"""
        if self._uart:
            self._uart.deinit()
            self._uart = None

    def send(self, packet: DataPacket) -> int:
        """发送 DataPacket，返回发送字节数"""
        if not self._uart:
            return 0
        return self._uart.write(packet.pack())

    def send_raw(self, data: bytes) -> int:
        """发送原始字节"""
        if not self._uart:
            return 0
        return self._uart.write(data)

    def receive(self):
        """
        非阻塞接收一个数据包
        :return: DataPacket 或 None
        """
        if not self._uart:
            return None
        packet = DataPacket()
        pkt_size = len(DataPacket.HEADER) + struct.calcsize(DataPacket.FORMAT) + 1
        if self._uart.any() >= pkt_size:
            data = self._uart.read(pkt_size)
            if packet.unpack(data):
                return packet
        return None

    def available(self) -> int:
        """接收缓冲区可用字节数"""
        return self._uart.any() if self._uart else 0

    def register_rx_callback(self, callback):
        """
        注册接收回调，签名为 callback(DataPacket)
        """
        self._rx_callback = callback

    def process_rx(self):
        """在主循环中周期调用，自动触发回调"""
        pkt = self.receive()
        if pkt and self._rx_callback:
            self._rx_callback(pkt)
