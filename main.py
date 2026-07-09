"""
主程序 (main.py)
================
全国大学生电子设计竞赛 —— CanMV K230 视觉处理与执行控制主程序。

功能：
  1. 初始化摄像头、串口、PWM 等外设
  2. 进入主循环：获取图像 → 视觉处理 → 串口发送结果
  3. 实时计算并打印帧率（FPS）
  4. 提供异常捕获和资源释放

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

使用方法：
  将本文件及所有依赖模块复制到 K230 的 /sd/ 或内置 Flash，
  重启开发板即可自动运行 main.py。
"""

import time
import gc
import math
from machine import UART, PWM, Pin

# ---- 导入各功能模块 ----
from uart_com        import UARTManager, DataPacket
from camera          import Camera
from color_detection import ColorDetector, COLOR_THRESHOLDS
from shape_detection import ShapeDetector, ShapeType
from barcode         import BarcodeDetector
from qrcode          import QRCodeDetector
from ocr             import OCR
from object_detection import ObjectDetector
from tracking        import Tracker, TrackState
from line_follow     import LineFollower
from coordinate_mapping import CoordinateMapper
from laser_detection import LaserDetector
from motor           import MotorController
from servo           import ServoController


# ============================================================
# 全局配置
# ============================================================
class Config:
    """
    全局配置类。
    所有关键参数集中管理，便于调试和修改。
    """
    # ---- 摄像头 ----
    CAM_WIDTH  = 320
    CAM_HEIGHT = 240
    CAM_FPS    = 60
    CAM_PIXFMT = "RGB565"  # "RGB565" / "GRAYSCALE"

    # ---- 串口 ----
    UART_ID     = 1
    UART_BAUD   = 115200

    # ---- 功能开关（按需开启/关闭以节省算力） ----
    ENABLE_COLOR_DETECTION  = True
    ENABLE_SHAPE_DETECTION  = False
    ENABLE_BARCODE          = False
    ENABLE_QRCODE           = False
    ENABLE_OCR              = False
    ENABLE_OBJECT_DETECTION = False
    ENABLE_TRACKING         = False
    ENABLE_LINE_FOLLOW      = False
    ENABLE_LASER_DETECTION  = False

    # ---- 调试 ----
    DEBUG_MODE = True           # 开启后在图像上绘制中间结果
    FPS_INTERVAL = 1.0          # FPS 打印间隔（秒）

    # ---- 电机 / 舵机 ----
    MOTOR_COUNT = 4
    SERVO_COUNT = 2

    # ---- 默认检测目标 ----
    DEFAULT_COLOR_TARGET = "red"


# ============================================================
# 系统初始化
# ============================================================
def init_all():
    """
    初始化所有外设和模块
    :return: (camera, uart, detectors_dict, actuators_dict)
    """
    print("=" * 40)
    print("  全国大学生电子设计竞赛 - K230 视觉框架")
    print("  Version 1.0")
    print("=" * 40)

    # 1. 摄像头
    print("\n[Init] 初始化摄像头...")
    cam = Camera()
    if not cam.init(width=Config.CAM_WIDTH,
                    height=Config.CAM_HEIGHT,
                    framerate=Config.CAM_FPS):
        print("[错误] 摄像头初始化失败，请检查连接！")
        raise RuntimeError("Camera init failed")

    # 2. 串口
    print("[Init] 初始化串口...")
    uart = UARTManager(uart_id=Config.UART_ID, baudrate=Config.UART_BAUD)
    if not uart.init():
        print("[警告] 串口初始化失败，将跳过串口通信")
        # 不 raise，允许纯视觉调试

    # 3. 视觉检测器（按需初始化）
    detectors = {}

    if Config.ENABLE_COLOR_DETECTION:
        detectors['color'] = ColorDetector(debug=Config.DEBUG_MODE)

    if Config.ENABLE_SHAPE_DETECTION:
        detectors['shape'] = ShapeDetector(debug=Config.DEBUG_MODE)

    if Config.ENABLE_BARCODE:
        detectors['barcode'] = BarcodeDetector(debug=Config.DEBUG_MODE)

    if Config.ENABLE_QRCODE:
        detectors['qrcode'] = QRCodeDetector(debug=Config.DEBUG_MODE)

    if Config.ENABLE_OCR:
        detectors['ocr'] = OCR(mode="template")  # 或 mode="cnn"

    if Config.ENABLE_OBJECT_DETECTION:
        detectors['object'] = ObjectDetector(mode="yolo", debug=Config.DEBUG_MODE)

    if Config.ENABLE_TRACKING:
        detectors['tracker'] = Tracker(mode="camshift", debug=Config.DEBUG_MODE)

    if Config.ENABLE_LINE_FOLLOW:
        detectors['line'] = LineFollower(line_color="black", debug=Config.DEBUG_MODE)

    if Config.ENABLE_LASER_DETECTION:
        detectors['laser'] = LaserDetector(strategy="brightness", debug=Config.DEBUG_MODE)

    # 坐标映射器（独立初始化，需预先标定）
    mapper = CoordinateMapper(mode="linear")
    # mapper.load_calibration("/sd/calib.json")  # 若有保存的标定文件

    # 4. 执行器
    actuators = {}
    if Config.MOTOR_COUNT > 0:
        motors = MotorController(num_motors=Config.MOTOR_COUNT)
        actuators['motors'] = motors
        print(f"[Init] 电机控制器: {Config.MOTOR_COUNT}路")

    if Config.SERVO_COUNT > 0:
        servos = ServoController(num_servos=Config.SERVO_COUNT)
        actuators['servos'] = servos
        print(f"[Init] 舵机控制器: {Config.SERVO_COUNT}路")

    print("[Init] 所有模块初始化完成！\n")
    return cam, uart, detectors, mapper, actuators


# ============================================================
# 视觉任务处理
# ============================================================
def process_vision(img, detectors: dict, mapper: CoordinateMapper) -> DataPacket:
    """
    执行所有已启用的视觉检测任务
    :param img:        图像帧
    :param detectors:  检测器字典
    :param mapper:     坐标映射器
    :return:           DataPacket（填充检测结果）
    """
    packet = DataPacket()

    # --- 颜色检测 ---
    if 'color' in detectors:
        blobs = detectors['color'].detect(img,
                                          color_name=Config.DEFAULT_COLOR_TARGET)
        if blobs:
            largest = blobs[0]
            # 坐标映射（若已标定）
            wx, wy = mapper.pixel_to_world(largest.cx, largest.cy)
            packet.target_x = int(wx * 100)   # 放大100倍保留精度
            packet.target_y = int(wy * 100)
            packet.target_type = 1
            packet.flags |= 0x01              # bit0: 检测到目标

            if Config.DEBUG_MODE:
                detectors['color'].draw_biggest(img, largest)

    # --- 形状检测 ---
    if 'shape' in detectors:
        shapes = detectors['shape'].detect(img)
        if shapes:
            # 找最大形状
            best = max(shapes, key=lambda s: s.area)
            if best.shape_type == ShapeType.CIRCLE:
                packet.target_type = 2   # 圆形
            elif best.shape_type == ShapeType.RECTANGLE:
                packet.target_type = 3   # 矩形
            elif best.shape_type == ShapeType.TRIANGLE:
                packet.target_type = 4   # 三角形
            else:
                packet.target_type = 5   # 其他

            if Config.DEBUG_MODE:
                detectors['shape'].draw_shapes(img, shapes)

    # --- 条形码 ---
    if 'barcode' in detectors:
        results = detectors['barcode'].detect(img)
        if results:
            packet.target_type = 6
            if Config.DEBUG_MODE:
                detectors['barcode'].draw_results(img, results)

    # --- 二维码 ---
    if 'qrcode' in detectors:
        results = detectors['qrcode'].detect(img)
        if results:
            packet.target_type = 7
            if Config.DEBUG_MODE:
                detectors['qrcode'].draw_results(img, results)

    # --- 目标检测 ---
    if 'object' in detectors:
        boxes = detectors['object'].detect(img)
        if boxes:
            best = detectors['object'].get_largest(boxes)
            if best:
                packet.target_x = best.centroid[0]
                packet.target_y = best.centroid[1]
                packet.target_type = 8 + best.class_id
            if Config.DEBUG_MODE:
                detectors['object'].draw_boxes(img, boxes)

    # --- 循迹 ---
    if 'line' in detectors:
        line_result = detectors['line'].detect(img)
        if line_result.found:
            packet.target_x = int(line_result.offset * 100)
            packet.target_y = int(line_result.angle * 100)
            packet.flags |= 0x04   # bit2: 线检测到
            if Config.DEBUG_MODE:
                detectors['line'].draw(img, line_result)

    # --- 激光 ---
    if 'laser' in detectors:
        spot = detectors['laser'].detect(img)
        if spot.found:
            packet.target_x = int(spot.cx * 100)
            packet.target_y = int(spot.cy * 100)
            packet.target_type = 20
            packet.flags |= 0x01
            if Config.DEBUG_MODE:
                detectors['laser'].draw(img, spot)

    return packet


# ============================================================
# 指令处理（接收下位机命令）
# ============================================================
def process_command(packet: DataPacket, actuators: dict):
    """
    处理从下位机接收的命令
    :param packet:    接收到的 DataPacket
    :param actuators: 执行器字典 {"motors": ..., "servos": ...}
    """
    if packet is None:
        return

    cmd = packet.command

    # 电机命令
    if 'motors' in actuators:
        mc = actuators['motors']
        if cmd == 0x01:   # 前进
            mc.set_all_throttle(40)
        elif cmd == 0x02:  # 后退
            mc.set_all_throttle(-40)
        elif cmd == 0x03:  # 左转
            mc.set_motor(1, 30)
            mc.set_motor(2, 60)
        elif cmd == 0x04:  # 右转
            mc.set_motor(1, 60)
            mc.set_motor(2, 30)
        elif cmd == 0x00:  # 停止
            mc.stop_all()
        elif cmd == 0xFF:  # 急停
            mc.emergency_stop_all()

    # 舵机命令
    if 'servos' in actuators:
        sc = actuators['servos']
        if 0x10 <= cmd <= 0x1F:  # 云台控制
            angle = packet.target_x  # 复用字段传角度
            sc.set_angle(1, angle)


# ============================================================
# 主循环
# ============================================================
def main():
    """
    主程序入口
    """
    cam = None
    uart = None

    try:
        # ---- 初始化 ----
        cam, uart, detectors, mapper, actuators = init_all()

        # ---- 可选：解锁电机 ----
        # if 'motors' in actuators:
        #     actuators['motors'].arm_all()

        # ---- FPS 计时变量 ----
        fps = 0.0
        frame_count = 0
        t_start = time.ticks_ms()
        t_last_fps_print = time.ticks_ms()

        print("[Main] 进入主循环...\n")

        # ==================== 主循环 ====================
        while True:
            t_frame_start = time.ticks_ms()

            # 1. 获取图像帧
            img = cam.get_frame()
            if img is None:
                print("[警告] 获取帧失败")
                continue

            # 2. 执行视觉处理
            packet = process_vision(img, detectors, mapper)

            # 3. 接收下位机指令
            if uart is not None:
                rx_packet = uart.receive()
                if rx_packet is not None:
                    process_command(rx_packet, actuators)

                # 4. 发送检测结果
                uart.send(packet)

            # 5. 帧率计算
            frame_count += 1
            t_now = time.ticks_ms()

            # 每隔 FPS_INTERVAL 秒打印一次
            if time.ticks_diff(t_now, t_last_fps_print) >= Config.FPS_INTERVAL * 1000:
                elapsed = time.ticks_diff(t_now, t_last_fps_print) / 1000.0
                fps = frame_count / elapsed
                print(f"FPS: {fps:.1f}")

                # 也打印帧耗时
                frame_time = time.ticks_diff(t_now, t_frame_start)
                if Config.DEBUG_MODE:
                    print(f"  Frame time: {frame_time}ms, Free mem: {gc.mem_free()}")

                frame_count = 0
                t_last_fps_print = t_now

            # 6. 垃圾回收（周期性，防止内存碎片）
            if frame_count % 50 == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("\n[Main] 用户中断")

    except Exception as e:
        print(f"\n[Main] 运行时异常: {e}")
        import sys
        sys.print_exception(e)

    finally:
        # ---- 资源释放 ----
        print("[Main] 正在释放资源...")

        if cam is not None:
            cam.deinit()

        if uart is not None:
            uart.deinit()

        if 'motors' in actuators:
            actuators['motors'].emergency_stop_all()
            actuators['motors'].deinit_all()

        if 'servos' in actuators:
            actuators['servos'].release_all()
            actuators['servos'].deinit_all()

        print("[Main] 系统已安全关闭")


# ============================================================
# 启动入口
# ============================================================
if __name__ == "__main__":
    main()
