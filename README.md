# CanMV K230 电赛视觉框架

基于 **CanMV K230 (MicroPython)** 的全国大学生电子设计竞赛视觉处理与执行控制框架。

适用方向：控制类 / 无人机类 / 小车类

---

## 目录结构

```
K230/
├── main.py                  # 主程序入口（摄像头→视觉处理→串口发送→FPS显示）
├── uart_com.py              # 串口通信（数据包打包/解包/收发）
├── camera.py                # 图像采集（初始化/ROI/曝光/白平衡）
├── color_detection.py       # 颜色识别（LAB阈值/多颜色/可视化）
├── shape_detection.py       # 形状/轮廓检测（圆/矩形/三角形）
├── barcode.py               # 条形码识别（EAN-13/CODE128）
├── qrcode.py                # 二维码识别
├── ocr.py                   # 数字/字符识别（模板匹配/CNN）
├── object_detection.py      # 目标检测与分类（YOLO kmodel）
├── tracking.py              # 目标跟踪（CamShift/KCF）
├── line_follow.py           # 循迹算法（回归/线段/偏差计算）
├── coordinate_mapping.py    # 坐标映射（透视变换/线性映射）
├── laser_detection.py       # 激光笔光斑检测
├── motor.py                 # 无刷电机控制（电调/PWM）
├── servo.py                 # 舵机控制（云台/机械臂）
└── README.md                # 本文件
```

---

## 快速开始

### 1. 部署文件

将所有 `.py` 文件复制到 CanMV K230 的 MicroSD 卡或内置 Flash 中：

```
/sd/main.py
/sd/uart_com.py
/sd/camera.py
...
```

### 2. 配置参数

打开 `main.py`，修改 `Config` 类中的参数：

```python
class Config:
    CAM_WIDTH  = 320        # 图像分辨率
    CAM_HEIGHT = 240
    UART_BAUD  = 115200     # 串口波特率

    # 功能开关（按需开启）
    ENABLE_COLOR_DETECTION  = True
    ENABLE_LINE_FOLLOW      = False
    ENABLE_OBJECT_DETECTION = False
    ...
```

### 3. 运行

重启 K230 开发板，`main.py` 会自动执行。

或通过 CanMV IDE 手动运行 `main.py`。

---

## 模块使用示例

### 颜色识别

```python
from camera import Camera
from color_detection import ColorDetector, COLOR_THRESHOLDS

cam = Camera()
cam.init()
detector = ColorDetector()

img = cam.get_frame()
blobs = detector.detect(img, color_name="red")
if blobs:
    print(f"最大色块: {blobs[0]}")
    detector.draw_biggest(img, blobs[0])
```

### 循迹

```python
from line_follow import LineFollower

lf = LineFollower(line_color="black")
result = lf.detect(img)
print(f"偏移: {result.offset:.1f}px, 角度: {result.angle:.1f}°")
```

### 二维码识别

```python
from qrcode import QRCodeDetector

qr = QRCodeDetector()
results = qr.detect(img)
for r in results:
    print(f"内容: {r.payload}")
```

### 串口通信

```python
from uart_com import UARTManager, DataPacket

uart = UARTManager(uart_id=1, baudrate=115200)
uart.init()

# 发送
pkt = DataPacket()
pkt.command = 0x01
uart.send(pkt)

# 接收
rx = uart.receive()
if rx:
    print(f"收到: {rx}")
```

### 电机控制

```python
from motor import MotorController

mc = MotorController(num_motors=4)
mc.init_all()
mc.set_all_throttle(30)   # 30%油门
mc.emergency_stop_all()   # 急停
```

### 舵机控制

```python
from servo import ServoController

sc = ServoController(num_servos=2)
sc.init_all()
sc.set_angle(1, 90)   # 云台Yaw → 90°
sc.set_angle(2, 45)   # 云台Pitch → 45°
```

---

## 串口通信协议

### 数据包格式

| 字段 | 字节 | 说明 |
|------|------|------|
| HEADER | 2B | 包头 `0xA5 0x5A` |
| target_x | 2B | 目标X坐标 (×100) |
| target_y | 2B | 目标Y坐标 (×100) |
| target_type | 1B | 目标类型 (1:红 2:绿 3:蓝 4:QR 5:条码 ...) |
| command | 1B | 命令字 (0:停止 1:前进 2:后退 ...) |
| flags | 1B | 标志位 (bit0:检测到目标 bit1:跟踪中 ...) |
| checksum | 1B | 异或校验和 |

### 目标类型编码

| 编码 | 含义 |
|------|------|
| 0 | 无目标 |
| 1 | 红色 |
| 2 | 圆形 |
| 3 | 矩形 |
| 4 | 三角形 |
| 6 | 条形码 |
| 7 | 二维码 |
| 20 | 激光光斑 |

---

## 配置与调试

### 颜色阈值标定

每个场景的光照条件不同，建议现场标定颜色阈值。

可使用 CanMV IDE 的 **阈值编辑器** 工具，或通过串口动态修改：

```python
detector = ColorDetector()
detector.set_threshold("red", [(Lmin, Lmax, Amin, Amax, Bmin, Bmax)])
```

### 坐标标定

1. 在场地四角放置标记点
2. 记录图像坐标和实际世界坐标
3. 调用标定：

```python
mapper = CoordinateMapper(mode="perspective")
mapper.calibrate_perspective(
    pixel_points=[(10, 200), (310, 200), (310, 40), (10, 40)],
    world_points=[(0, 50), (50, 50), (50, 0), (0, 0)]
)
mapper.save_calibration("/sd/calib.json")
```

### Debug 模式

开启后会在图像上绘制检测框、中心点等中间结果：

```python
Config.DEBUG_MODE = True
```

---

## 性能优化建议

1. **分辨率**：控制类场景建议 320×240，可保证 50+ FPS
2. **功能开关**：只开启需要的检测模块
3. **ROI 裁剪**：循迹时只处理图像下方 1/3
4. **帧率目标**：控制类 ≥30fps，识别类 ≥15fps
5. **定期 GC**：主循环中每 50 帧执行一次 `gc.collect()`

---

## 扩展指南

### 添加新模块

1. 创建 `new_module.py`，遵循统一接口：`class NewDetector` + `detect(img)` 方法
2. 在 `main.py` 的 `Config` 中添加开关 `ENABLE_NEW_MODULE`
3. 在 `init_all()` 中初始化
4. 在 `process_vision()` 中调用

### 替换为实际模型

`object_detection.py` 和 `ocr.py` 中的 YOLO/CNN 推理为占位代码。
替换步骤：
1. 将 `.kmodel` 模型文件放入 `/sd/`
2. 在对应模块中启用 `nncase.KPU()` 相关代码
3. 配置正确的 `anchors`、`input_size` 和 `labels`

---

## 电赛常见场景适配

| 赛题类型 | 推荐开启模块 | 说明 |
|----------|-------------|------|
| 小车循迹 | line_follow, color_detection | 黑线跟踪+颜色识别 |
| 无人机巡线 | line_follow, object_detection, motor | 俯拍巡线+目标检测 |
| 小球分拣 | color_detection, servo | 颜色识别+机械臂 |
| 二维码导航 | qrcode, line_follow, motor | QR路径点+巡线 |
| 激光打靶 | laser_detection, servo | 光斑定位+云台 |
| 病房巡检 | ocr, qrcode, line_follow | 数字识别+导航 |

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-07-09 | 初始版本，14个功能模块 + main.py |

---

## 许可证

MIT License — 仅供学习与竞赛使用。
