"""
主程序 (main.py)
================
全国大学生电子设计竞赛 —— CanMV K230 视觉处理与执行控制主程序。

功能：
  1. 初始化摄像头、串口、激光、PWM 等外设
  2. 进入主循环：获取图像 → 视觉处理 → PID → 串口发命令 / 收状态
  3. 实时计算并打印帧率（FPS）
  4. 提供异常捕获和资源释放

通信协议 (v2.0)：
  CommandPacket (K230 → MCU): 12B, HEADER + cmd + x + y + data + flags + param + chk
  StatusPacket  (MCU → K230): 12B, command=0x10, 速度/距离/电压/状态/错误码

作者：E-Competition Team
日期：2026-07-12
版本：v2.0 — 双包协议 + 激光驱动

使用方法：
  将本文件及所有依赖模块复制到 K230 的 /sd/ 或内置 Flash，
  重启开发板即可自动运行 main.py。
"""

import time
import gc
import math
from machine import UART, PWM, Pin
from media.display import *          # 显示输出（IDE预览）
from media.sensor import Sensor     # 摄像头像素格式常量
import debug_config                   # 全局调试开关

# ---- 导入各功能模块 ----
from uart_com        import (UARTManager, CommandPacket, StatusPacket)
from uart_com        import (CMD_STOP, CMD_FORWARD, CMD_BACKWARD,
                             CMD_TURN_LEFT, CMD_TURN_RIGHT,
                             CMD_STEER, CMD_SERVO, CMD_EMERGENCY,
                             FLAG_ARRIVED, FLAG_RUNNING, FLAG_ERROR)
from uart_com        import (make_stop_command, make_steer_command,
                             make_servo_command)
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
from laser           import Laser
from motor           import MotorController
from servo           import ServoController
from pid_controller  import PIDController, PID_PRESETS


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
    ENABLE_COLOR_DETECTION  = False  # 颜色识别
    ENABLE_SHAPE_DETECTION  = True   # 形状识别（激光跟随矩形中心）
    ENABLE_BARCODE          = False  # 条码识别
    ENABLE_QRCODE           = False  # 二维码识别
    ENABLE_OCR              = False  # OCR
    ENABLE_OBJECT_DETECTION = False  # 目标检测
    ENABLE_TRACKING         = False  # 目标跟踪
    ENABLE_LINE_FOLLOW      = False   # 循线行驶
    ENABLE_LASER_DETECTION  = True   # 激光检测

    # ---- 激光头 ----
    LASER_ENABLE = True         # 是否启用激光头
    LASER_PIN    = 20            # GPIO 引脚（IO20 = 物理Pin5）

    # ---- 调试 ----
    DEBUG_MODE = True            # 开启后在图像上绘制中间结果 + print
    FPS_INTERVAL = 1.0           # FPS 打印间隔（秒）

    # ---- 电机 / 舵机 ----
    MOTOR_COUNT = 0              # 本地电机数（0=由下位机控制）
    SERVO_COUNT = 0              # 本地舵机数（0=由下位机控制）

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


# ---- 控制模式 ----
MODE_LINE_FOLLOW = "line_follow"
MODE_LASER_TRACK = "laser_track"
MODE_COLOR_TRACK = "color_track"
CONTROL_MODE = MODE_LASER_TRACK   # ← 切换控制模式只需改这里！


# ============================================================
# 系统初始化
# ============================================================
def init_all():
    """
    初始化所有外设和模块
    :return: (camera, uart, detectors_dict, actuators_dict, laser)
    """
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

    # 注：K230 v3p0 固件不支持 set_auto_whitebal / set_brightness 等，
    #     也不支持 set_auto_exposure 手动模式（会导致 snapshot 通道报错），
    #     画质优化需在摄像头物理调焦和光线环境上解决。

    # 3. 串口
    if Config.DEBUG_MODE:
        print("[Init] 初始化串口...")
    uart = UARTManager(uart_id=Config.UART_ID, baudrate=Config.UART_BAUD,
                       tx_pin=Config.UART_TX_PIN, rx_pin=Config.UART_RX_PIN)
    if not uart.init():
        print("[警告] 串口初始化失败，将跳过串口通信")

    # 4. 激光头
    laser = None
    if Config.LASER_ENABLE:
        try:
            laser = Laser(pin=Config.LASER_PIN)
            if Config.DEBUG_MODE:
                print(f"[Init] 激光头初始化成功 (IO{Config.LASER_PIN})")
        except Exception as e:
            print(f"[Init] 激光头初始化失败: {e}")

    # 5. 视觉检测器（按需初始化）
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
        detectors['laser'] = LaserDetector(strategy="color", debug=Config.DEBUG_MODE)

    # 坐标映射器（独立初始化，需预先标定）
    mapper = CoordinateMapper(mode="linear")

    # 6. 执行器（本地 — 通常 MOTOR_COUNT/SERVO_COUNT = 0）
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
    return cam, uart, detectors, mapper, actuators, laser


# ============================================================
# 视觉任务处理
# ============================================================
def process_vision(img, detectors: dict, mapper: CoordinateMapper):
    """
    执行所有已启用的视觉检测任务。
    检测结果填入 CommandPacket（后续由 process_control 设定 command）。

    :param img:        图像帧
    :param detectors:  检测器字典
    :param mapper:     坐标映射器
    :return:           (CommandPacket, vis_results)
                       vis_results = {"line": LineResult|None, "laser": LaserSpot|None, ...}
    """
    cmd = CommandPacket()          # 新建命令包
    vis_results = {"line": None, "laser": None}

    # --- 颜色检测 ---
    if 'color' in detectors:
        blobs = detectors['color'].detect(img,
                                          color_name=Config.DEFAULT_COLOR_TARGET)
        if blobs:
            largest = blobs[0]
            wx, wy = mapper.pixel_to_world(largest.cx, largest.cy)
            cmd.x = int(wx * 100)       # 放大100倍保留精度
            cmd.y = int(wy * 100)
            cmd.data = 1                # target_type = 1（颜色）
            cmd.flags |= 0x01           # bit0: 检测到目标
            if Config.DEBUG_MODE:
                detectors['color'].draw_biggest(img, largest)

    # --- 形状检测 ---
    if 'shape' in detectors:
        shapes = detectors['shape'].detect(img)
        if shapes:
            best = max(shapes, key=lambda s: s.area)
            if best.shape_type == ShapeType.CIRCLE:
                cmd.data = 2
                vis_results['rect_center'] = (best.centroid[0], best.centroid[1])
            elif best.shape_type == ShapeType.RECTANGLE:
                cmd.data = 3
                vis_results['rect_center'] = (best.centroid[0], best.centroid[1])
            elif best.shape_type == ShapeType.TRIANGLE:
                cmd.data = 4
                vis_results['rect_center'] = (best.centroid[0], best.centroid[1])
            else:
                cmd.data = 5
            if Config.DEBUG_MODE:
                detectors['shape'].draw_shapes(img, shapes)

    # --- 条形码 ---
    if 'barcode' in detectors:
        results = detectors['barcode'].detect(img)
        if results:
            cmd.data = 6
            if Config.DEBUG_MODE:
                detectors['barcode'].draw_results(img, results)

    # --- 二维码 ---
    if 'qrcode' in detectors:
        results = detectors['qrcode'].detect(img)
        if results:
            cmd.data = 7
            if Config.DEBUG_MODE:
                detectors['qrcode'].draw_results(img, results)

    # --- 目标检测 ---
    if 'object' in detectors:
        boxes = detectors['object'].detect(img)
        if boxes:
            best = detectors['object'].get_largest(boxes)
            if best:
                cmd.x = best.centroid[0]
                cmd.y = best.centroid[1]
                cmd.data = 8 + best.class_id
            if Config.DEBUG_MODE:
                detectors['object'].draw_boxes(img, boxes)

    # --- 循迹 ---
    if 'line' in detectors:
        line_result = detectors['line'].detect(img)
        vis_results['line'] = line_result
        if line_result.found:
            cmd.x = int(line_result.offset * 100)       # x = 偏移 ×100
            cmd.y = int(line_result.angle * 100)        # y = 角度 ×100
            cmd.flags |= 0x04                           # bit2: 线检测到
        if Config.DEBUG_MODE:
            detectors['line'].draw(img, line_result)

    # --- 激光 ---
    if 'laser' in detectors:
        spot = detectors['laser'].detect(img)
        vis_results['laser'] = spot
        if spot.found:
            cmd.x = int(spot.cx * 100)
            cmd.y = int(spot.cy * 100)
            cmd.data = 20
            cmd.flags |= 0x01
        if Config.DEBUG_MODE:
            detectors['laser'].draw(img, spot)

    return cmd, vis_results


# ============================================================
# PID 控制处理
# ============================================================
def process_control(cmd: CommandPacket, vis_results, pid_ctrls,
                    line_lost, laser_lost) -> CommandPacket:
    """
    根据视觉检测结果运行 PID，将最终指令填入 CommandPacket。

    :param cmd:         CommandPacket（已由 process_vision 填充检测数据）
    :param vis_results: 视觉检测原始结果
    :param pid_ctrls:   PID 控制器字典
    :param line_lost:   丢线标志（list，跨帧可变）
    :param laser_lost:  丢激光标志
    :return:            修改后的 CommandPacket
    """
    # --- 循线 PID ---
    if CONTROL_MODE == MODE_LINE_FOLLOW and 'line_pid' in pid_ctrls:
        line_result = vis_results.get('line')

        if line_result is not None and line_result.found:
            if line_lost[0]:
                pid_ctrls['line_pid'].reset()
                line_lost[0] = False

            turn = pid_ctrls['line_pid'].compute(line_result.offset)
            # 用工厂函数构建 STEER 命令（x=转向, y=速度）
            cmd.command = CMD_STEER
            cmd.x = int(turn)           # 转向量 -100~100，x>0 期望右转
            cmd.y = Config.BASE_SPEED   # 基准速度
            cmd.flags |= 0x04

            if Config.DEBUG_MODE:
                print("[PID-Line] err={:.1f} turn={:.0f}".format(
                    pid_ctrls['line_pid'].error, turn))
        else:
            if not line_lost[0]:
                if Config.DEBUG_MODE:
                    print("[PID-Line] 丢线 → 停车 + 重置PID")
                line_lost[0] = True
            pid_ctrls['line_pid'].reset()
            cmd.command = CMD_STOP
            cmd.x = 0
            cmd.y = 0

    # --- 激光跟踪 PID（目标 = 检测到的矩形中心） ---
    elif CONTROL_MODE == MODE_LASER_TRACK and 'laser_pan_pid' in pid_ctrls:
        spot = vis_results.get('laser')
        rect_center = vis_results.get('rect_center')

        # 每帧动态更新 PID setpoint 为矩形中心
        if rect_center is not None:
            pid_ctrls['laser_pan_pid'].set_setpoint(rect_center[0])
            pid_ctrls['laser_tilt_pid'].set_setpoint(rect_center[1])

        if spot is not None and spot.found:
            if laser_lost[0]:
                pid_ctrls['laser_pan_pid'].reset()
                pid_ctrls['laser_tilt_pid'].reset()
                laser_lost[0] = False

            pan = pid_ctrls['laser_pan_pid'].compute(spot.cx)
            tilt = pid_ctrls['laser_tilt_pid'].compute(spot.cy)

            cmd.command = CMD_SERVO
            cmd.x = int(pan)            # 舵机 pan 角度水平
            cmd.y = int(tilt)           # 舵机 tilt 角度竖直

            if Config.DEBUG_MODE:
                sp_x = pid_ctrls['laser_pan_pid'].setpoint
                sp_y = pid_ctrls['laser_tilt_pid'].setpoint
                print("[PID-Laser] spot=({:.0f},{:.0f}) tgt=({:.0f},{:.0f}) "
                      "err=({:.1f},{:.1f}) → pan={:.0f} tilt={:.0f}".format(
                          spot.cx, spot.cy, sp_x, sp_y,
                          pid_ctrls['laser_pan_pid'].error,
                          pid_ctrls['laser_tilt_pid'].error,
                          pan, tilt))
        else:
            if not laser_lost[0]:
                if Config.DEBUG_MODE:
                    print("[PID-Laser] 丢失光斑 → 重置PID")
                laser_lost[0] = True
            pid_ctrls['laser_pan_pid'].reset()
            pid_ctrls['laser_tilt_pid'].reset()
            cmd.command = CMD_STOP
            cmd.x = 0
            cmd.y = 0

    return cmd


# ============================================================
# 下位机状态处理
# ============================================================
def process_status(pkt: StatusPacket, actuators: dict):
    """
    处理从下位机收到的状态包（command=0x10）。
    在调试模式下打印状态信息。
    """
    if pkt is None:
        return

    if Config.DEBUG_MODE:
        print("[Status] spd={:.1f}cm/s dist={:.1f}cm v={:.2f}V "
              "arr={} run={} err={}".format(
                  pkt.speed_cms, pkt.distance_cm, pkt.voltage_v,
                  pkt.is_arrived, pkt.is_running, pkt.has_error))
        if pkt.has_error:
            print("[Status] ⚠ 下位机报错 code={}".format(pkt.error_code))

    # 若下位机报错 → 本地执行器也急停
    if pkt.has_error and 'motors' in actuators:
        try:
            actuators['motors'].emergency_stop_all()
        except Exception:
            pass


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
    laser = None

    try:
        # ---- 设置全局调试开关 ----
        debug_config.DEBUG = Config.DEBUG_MODE

        # ---- 初始化 ----
        cam, uart, detectors, mapper, actuators, laser = init_all()

        # ---- 注册串口接收回调 ----
        if uart is not None and uart.is_ready:
            uart.register_rx_callback(
                lambda pkt: process_status(pkt, actuators))

        # ---- PID 控制器初始化 ----
        pid_ctrls = {}
        line_lost = [True]
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
                # 默认 setpoint=画面中心，运行时每帧被矩形中心覆盖
                pid_ctrls['laser_pan_pid'] = PIDController(
                    kp=Config.LASER_PAN_KP, ki=Config.LASER_PAN_KI, kd=Config.LASER_PAN_KD,
                    setpoint=Config.CAM_WIDTH / 2,
                    output_limit=Config.LASER_OUTPUT_LIMIT,
                    integral_limit=(-20, 20), name="laser_pan")
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

            # 2. 视觉处理 → CommandPacket（检测数据） + vis_results
            cmd, vis_results = process_vision(img, detectors, mapper)

            # 3. PID 控制 → 设定最终 command/x/y
            if Config.ENABLE_PID:
                cmd = process_control(cmd, vis_results, pid_ctrls,
                                      line_lost, laser_lost)

            # 4. IDE 显示预览
            if Config.DEBUG_MODE:
                try:
                    Display.show_image(img)
                except Exception:
                    pass

            # 5. 串口通信 — 发命令包 + 收状态包
            if uart is not None and uart.is_ready:
                uart.send_command(cmd)           # 发送 12B 命令包
                uart.process_rx()                # 处理接收（触发回调）

            # 6. 帧率计算
            frame_count += 1
            t_now = time.ticks_ms()

            if time.ticks_diff(t_now, t_last_fps_print) >= Config.FPS_INTERVAL * 1000:
                elapsed = time.ticks_diff(t_now, t_last_fps_print) / 1000.0
                fps = frame_count / elapsed
                if Config.DEBUG_MODE:
                    print(f"FPS: {fps:.1f}")

                frame_time = time.ticks_diff(t_now, t_frame_start)
                if Config.DEBUG_MODE:
                    print(f"  Frame time: {frame_time}ms, Free mem: {gc.mem_free()}")

                frame_count = 0
                t_last_fps_print = t_now

            # 7. 垃圾回收（周期性，防止内存碎片）
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

        if laser is not None:
            laser.off()
            laser.deinit()

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
