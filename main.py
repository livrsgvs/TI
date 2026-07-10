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
from media.display import *   # 显示输出（IDE预览）
from media.sensor import Sensor  # 摄像头像素格式常量
import debug_config                # 全局调试开关

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
from pid_controller  import PIDController, PID_PRESETS

# ---- 命令码常量（K230→MCU） ----
CMD_STOP      = 0x00   # 停车
CMD_FORWARD   = 0x01   # 前进
CMD_BACKWARD  = 0x02   # 后退
CMD_TURN_LEFT = 0x03   # 左转
CMD_TURN_RIGHT= 0x04   # 右转
CMD_STEER     = 0x05   # PID转向（target_x=转向量）
CMD_LASER_SERVO = 0x06  # 激光跟踪舵机（target_x=pan, target_y=tilt）
CMD_EMERGENCY = 0xFF   # 急停

# ---- 控制模式 ----
MODE_LINE_FOLLOW = "line_follow"
MODE_LASER_TRACK = "laser_track"
MODE_COLOR_TRACK = "color_track"
CONTROL_MODE = MODE_LINE_FOLLOW   # ← 切换控制模式只需改这里！


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
    CAM_PIXFMT = Sensor.RGB565  # Sensor.RGB565 / Sensor.GRAYSCALE

    # ---- 串口 ----
    UART_ID     = 2          # UART2
    UART_BAUD   = 115200
    UART_TX_PIN = 5          # UART2_TXD = IO5 (物理Pin17)
    UART_RX_PIN = 6          # UART2_RXD = IO6 (物理Pin20)

    # ---- 功能开关（按需开启/关闭以节省算力） ----
    ENABLE_COLOR_DETECTION  = False #颜色识别
    ENABLE_SHAPE_DETECTION  = False#形状识别
    ENABLE_BARCODE          = False#条码识别
    ENABLE_QRCODE           = False#二维码识别
    ENABLE_OCR              = False#光学字符识别
    ENABLE_OBJECT_DETECTION = False#目标检测
    ENABLE_TRACKING         = False#目标跟踪
    ENABLE_LINE_FOLLOW      = True#循线行驶
    ENABLE_LASER_DETECTION  = False#激光检测

    # ---- 调试 ----
    DEBUG_MODE = True           # 开启后在图像上绘制中间结果
    FPS_INTERVAL = 1.0          # FPS 打印间隔（秒）

    # ---- 电机 / 舵机 ----#接了几个对应的设备
    MOTOR_COUNT = 0
    SERVO_COUNT = 0

    # ---- 默认检测目标 ----
    DEFAULT_COLOR_TARGET = "red"

    # ---- PID 控制 ----
    ENABLE_PID = True            # 启用 PID 控制输出

    # 循线 PID 参数 — 直接改这里调参
    LINE_KP = 0.80
    LINE_KI = 0.02
    LINE_KD = 0.15
    LINE_OUTPUT_LIMIT = (-100, 100)
    LINE_INTEGRAL_LIMIT = (-50, 50)

    # 激光跟踪 PID 参数（水平 pan）
    LASER_PAN_KP = 0.30
    LASER_PAN_KI = 0.00
    LASER_PAN_KD = 0.05
    LASER_OUTPUT_LIMIT = (-90, 90)

    # 激光跟踪 PID 参数（垂直 tilt）
    LASER_TILT_KP = 0.30
    LASER_TILT_KI = 0.00
    LASER_TILT_KD = 0.05
    LASER_TILT_LIMIT = (-45, 45)

    # ---- 基准速度 ----
    BASE_SPEED = 40              # 小车基础速度（%）


# ============================================================
# 系统初始化
# ============================================================
def init_all():
    """
    初始化所有外设和模块
    :return: (camera, uart, detectors_dict, actuators_dict)
    """
    # print("=" * 40)
    # print("  全国大学生电子设计竞赛 - K230 视觉框架")
    # print("  Version 1.0")
    # print("=" * 40)

    # 1. 显示器（必须在 sensor.run() 之前初始化！）
    if Config.DEBUG_MODE:
        try:
            Display.init(Display.VIRT, width=Config.CAM_WIDTH,
                         height=Config.CAM_HEIGHT, fps=Config.CAM_FPS, to_ide=True)
            print("[Init] 显示器初始化成功 (IDE预览)")
        except Exception as e:
            print(f"[Init] 显示器初始化失败: {e}")

    # 2. 摄像头
    if Config.DEBUG_MODE:
        print("\n[Init] 初始化摄像头...")
    cam = Camera()
    if not cam.init(width=Config.CAM_WIDTH,
                    height=Config.CAM_HEIGHT,
                    framerate=Config.CAM_FPS,
                    pixformat=Config.CAM_PIXFMT):
        print("[错误] 摄像头初始化失败，请检查连接！")
        raise RuntimeError("Camera init failed")

    # 3. 串口
    if Config.DEBUG_MODE:
        print("[Init] 初始化串口...")
    uart = UARTManager(uart_id=Config.UART_ID, baudrate=Config.UART_BAUD,
                       tx_pin=Config.UART_TX_PIN, rx_pin=Config.UART_RX_PIN)
    if not uart.init():
        print("[警告] 串口初始化失败，将跳过串口通信")

    # 4. 视觉检测器（按需初始化）
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
        detectors['ocr'] = OCR(mode="template")

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

    # 5. 执行器
    actuators = {}
    if Config.MOTOR_COUNT > 0:
        motors = MotorController(num_motors=Config.MOTOR_COUNT)
        actuators['motors'] = motors
        if Config.DEBUG_MODE:
            print(f"[Init] 电机控制器: {Config.MOTOR_COUNT}路")

    if Config.SERVO_COUNT > 0:
        servos = ServoController(num_servos=Config.SERVO_COUNT)
        actuators['servos'] = servos
        if Config.DEBUG_MODE:
            print(f"[Init] 舵机控制器: {Config.SERVO_COUNT}路")

    if Config.DEBUG_MODE:
        print("[Init] 所有模块初始化完成！\n")
    return cam, uart, detectors, mapper, actuators


# ============================================================
# 视觉任务处理
# ============================================================
def process_vision(img, detectors: dict, mapper: CoordinateMapper):
    """
    执行所有已启用的视觉检测任务

    :param img:        图像帧
    :param detectors:  检测器字典
    :param mapper:     坐标映射器
    :return:           (DataPacket, vis_results)
                       vis_results = {"line": LineResult|None, "laser": LaserSpot|None, ...}
    """
    packet = DataPacket()
    vis_results = {"line": None, "laser": None}

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
        vis_results['line'] = line_result
        if line_result.found:
            packet.target_x = int(line_result.offset * 100)
            packet.target_y = int(line_result.angle * 100)
            packet.flags |= 0x04   # bit2: 线检测到
        # 始终画调试信息（含未检测到时的 ROI + "NO LINE"）
        if Config.DEBUG_MODE:
            detectors['line'].draw(img, line_result)

    # --- 激光 ---
    if 'laser' in detectors:
        spot = detectors['laser'].detect(img)
        vis_results['laser'] = spot
        if spot.found:
            packet.target_x = int(spot.cx * 100)
            packet.target_y = int(spot.cy * 100)
            packet.target_type = 20
            packet.flags |= 0x01
        if Config.DEBUG_MODE:
            detectors['laser'].draw(img, spot)

    return packet, vis_results


# ============================================================
# PID 控制处理
# ============================================================
def process_control(packet, vis_results, pid_ctrls, line_lost, laser_lost):
    """
    根据视觉检测结果运行 PID 控制，修改 packet 中的 command/target 字段。

    :param packet:      DataPacket（已由 process_vision 填充检测数据）
    :param vis_results: 视觉检测原始结果 {"line": LineResult, "laser": LaserSpot}
    :param pid_ctrls:   PID 控制器字典
    :param line_lost:   丢线标志（单元素列表，用于跨帧保持状态）
    :param laser_lost:  丢激光标志
    """
    # --- 循线 PID ---
    if CONTROL_MODE == MODE_LINE_FOLLOW and 'line_pid' in pid_ctrls:
        line_result = vis_results.get('line')

        if line_result is not None and line_result.found:
            if line_lost[0]:  # 刚从丢线恢复，重置 PID
                pid_ctrls['line_pid'].reset()
                line_lost[0] = False

            turn = pid_ctrls['line_pid'].compute(line_result.offset)
            packet.command = CMD_STEER
            packet.target_x = int(turn)
            packet.target_y = Config.BASE_SPEED

            if Config.DEBUG_MODE:
                print("[PID-Line] err={:.1f} turn={:.0f}".format(
                    pid_ctrls['line_pid'].error, turn))
        else:
            if not line_lost[0]:
                if Config.DEBUG_MODE:
                    print("[PID-Line] 丢线 → 停车 + 重置PID")
                line_lost[0] = True
            pid_ctrls['line_pid'].reset()
            packet.command = CMD_STOP
            packet.target_x = 0
            packet.target_y = 0

    # --- 激光跟踪 PID ---
    elif CONTROL_MODE == MODE_LASER_TRACK and 'laser_pan_pid' in pid_ctrls:
        spot = vis_results.get('laser')

        if spot is not None and spot.found:
            if laser_lost[0]:
                pid_ctrls['laser_pan_pid'].reset()
                pid_ctrls['laser_tilt_pid'].reset()
                laser_lost[0] = False

            # 以图像中心 (160, 120) 为 setpoint
            pan = pid_ctrls['laser_pan_pid'].compute(spot.cx)
            tilt = pid_ctrls['laser_tilt_pid'].compute(spot.cy)

            packet.command = CMD_LASER_SERVO
            packet.target_x = int(pan)
            packet.target_y = int(tilt)

            if Config.DEBUG_MODE:
                print("[PID-Laser] cx={:.1f} cy={:.1f} → pan={:.0f} tilt={:.0f}".format(
                    spot.cx, spot.cy, pan, tilt))
        else:
            if not laser_lost[0]:
                if Config.DEBUG_MODE:
                    print("[PID-Laser] 丢失目标 → 重置PID")
                laser_lost[0] = True
            pid_ctrls['laser_pan_pid'].reset()
            pid_ctrls['laser_tilt_pid'].reset()
            packet.command = CMD_STOP
            packet.target_x = 0
            packet.target_y = 0

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
        if cmd == CMD_FORWARD:
            mc.set_all_throttle(Config.BASE_SPEED)
        elif cmd == CMD_BACKWARD:
            mc.set_all_throttle(-Config.BASE_SPEED)
        elif cmd == CMD_TURN_LEFT:
            mc.set_motor(1, 30)
            mc.set_motor(2, 60)
        elif cmd == CMD_TURN_RIGHT:
            mc.set_motor(1, 60)
            mc.set_motor(2, 30)
        elif cmd == CMD_STEER:
            # MCU 发来的差速转向: target_x=转向, target_y=速度
            turn = packet.target_x
            speed = packet.target_y if packet.target_y != 0 else Config.BASE_SPEED
            left = speed - turn / 2
            right = speed + turn / 2
            mc.set_motor(1, max(0, min(100, right)))
            mc.set_motor(2, max(0, min(100, left)))
        elif cmd == CMD_STOP:
            mc.stop_all()
        elif cmd == CMD_EMERGENCY:
            mc.emergency_stop_all()

    # 舵机命令
    if 'servos' in actuators:
        sc = actuators['servos']
        if cmd == CMD_LASER_SERVO:
            # 激光跟踪云台: target_x=pan角度, target_y=tilt角度
            sc.set_angle(1, packet.target_x)
            sc.set_angle(2, packet.target_y)
        elif 0x10 <= cmd <= 0x1F:
            angle = packet.target_x
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
    actuators = {}
    detectors = {}

    try:
        # ---- 设置全局调试开关 ----
        debug_config.DEBUG = Config.DEBUG_MODE

        # ---- 初始化 ----
        cam, uart, detectors, mapper, actuators = init_all()

        # ---- PID 控制器初始化 ----
        pid_ctrls = {}
        line_lost = [True]    # 用 list 实现跨帧可变引用
        laser_lost = [True]

        if Config.ENABLE_PID:
            if CONTROL_MODE == MODE_LINE_FOLLOW and Config.ENABLE_LINE_FOLLOW:
                pid_ctrls['line_pid'] = PIDController(
                    kp=Config.LINE_KP, ki=Config.LINE_KI, kd=Config.LINE_KD,
                    output_limit=Config.LINE_OUTPUT_LIMIT,
                    integral_limit=Config.LINE_INTEGRAL_LIMIT,
                    name="line")
                if Config.DEBUG_MODE:
                    print("[PID] 循线PID已创建 (kp={:.2f}, ki={:.2f}, kd={:.2f})".format(
                        Config.LINE_KP, Config.LINE_KI, Config.LINE_KD))

            elif CONTROL_MODE == MODE_LASER_TRACK and Config.ENABLE_LASER_DETECTION:
                # 水平 Pan：setpoint = CAM_WIDTH/2
                pid_ctrls['laser_pan_pid'] = PIDController(
                    kp=Config.LASER_PAN_KP, ki=Config.LASER_PAN_KI, kd=Config.LASER_PAN_KD,
                    setpoint=Config.CAM_WIDTH / 2,
                    output_limit=Config.LASER_OUTPUT_LIMIT,
                    integral_limit=(-20, 20), name="laser_pan")
                # 垂直 Tilt：setpoint = CAM_HEIGHT/2
                pid_ctrls['laser_tilt_pid'] = PIDController(
                    kp=Config.LASER_TILT_KP, ki=Config.LASER_TILT_KI, kd=Config.LASER_TILT_KD,
                    setpoint=Config.CAM_HEIGHT / 2,
                    output_limit=Config.LASER_TILT_LIMIT,
                    integral_limit=(-20, 20), name="laser_tilt")
                if Config.DEBUG_MODE:
                    print("[PID] 激光跟踪PID已创建")

        # ---- FPS 计时变量 ----
        fps = 0.0
        frame_count = 0
        t_start = time.ticks_ms()
        t_last_fps_print = time.ticks_ms()

        if Config.DEBUG_MODE:
            print("[Main] 进入主循环...\n")

        # ==================== 主循环 ====================
        while True:
            t_frame_start = time.ticks_ms()

            # 1. 获取图像帧
            img = cam.get_frame()
            if img is None:
                print("[警告] 获取帧失败")
                continue

            # 2. 视觉处理 → packet + 原始检测结果
            packet, vis_results = process_vision(img, detectors, mapper)

            # 3. PID 控制 → 修改 packet 的 command/target
            if Config.ENABLE_PID:
                packet = process_control(packet, vis_results, pid_ctrls,
                                         line_lost, laser_lost)

            # 4. IDE显示预览
            if Config.DEBUG_MODE:
                try:
                    Display.show_image(img)
                except Exception:
                    pass

            # 5. 串口收发
            if uart is not None:
                rx_packet = uart.receive()
                if rx_packet is not None:
                    process_command(rx_packet, actuators)

                uart.send(packet)

            # 5. 帧率计算
            frame_count += 1
            t_now = time.ticks_ms()

            # 每隔 FPS_INTERVAL 秒打印一次
            if time.ticks_diff(t_now, t_last_fps_print) >= Config.FPS_INTERVAL * 1000:
                elapsed = time.ticks_diff(t_now, t_last_fps_print) / 1000.0
                fps = frame_count / elapsed
                if Config.DEBUG_MODE:
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
        if Config.DEBUG_MODE:
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

        try:
            Display.deinit()
        except Exception:
            pass

        if Config.DEBUG_MODE:
            print("[Main] 系统已安全关闭")


# ============================================================
# 启动入口
# ============================================================
if __name__ == "__main__":
    main()
