"""
图像采集模块 (camera.py)
=======================
功能：初始化摄像头，提供图像帧获取、ROI设置、自动曝光/白平衡控制。

作者：E-Competition Team
日期：2026-07-09
版本：v2.0 — 适配 CanMV K230 v3p0 (media.sensor API)

依赖：media.sensor、image 模块
"""

from media.sensor import *
import image
import time
import debug_config


# ============================================================
# 默认配置常量
# ============================================================
DEFAULT_WIDTH   = 320     # 默认分辨率宽
DEFAULT_HEIGHT  = 240     # 默认分辨率高
DEFAULT_FPS     = 60      # 默认帧率
DEFAULT_PIXFMT  = Sensor.RGB565  # 默认像素格式


# ============================================================
# 摄像头管理类
# ============================================================
class Camera:
    """
    摄像头管理类
    封装初始化、帧获取、ROI、曝光/白平衡控制。

    :使用示例:
        cam = Camera()
        cam.init()
        while True:
            img = cam.get_frame()
            # 处理图像...
    """

    def __init__(self):
        """初始化内部状态，不操作硬件"""
        self._width = DEFAULT_WIDTH
        self._height = DEFAULT_HEIGHT
        self._framerate = DEFAULT_FPS
        self._pixformat = DEFAULT_PIXFMT
        self._initialized = False
        self._roi = None            # (x, y, w, h)
        self._flip = False
        self._mirror = False
        self._sensor = None         # Sensor 实例

    # ---- 初始化 / 反初始化 ----

    def init(self,
             width: int = DEFAULT_WIDTH,
             height: int = DEFAULT_HEIGHT,
             framerate: int = DEFAULT_FPS,
             pixformat=DEFAULT_PIXFMT) -> bool:
        """
        初始化摄像头传感器
        :param width:     图像宽度
        :param height:    图像高度
        :param framerate: 帧率
        :param pixformat: 像素格式（Sensor.RGB565 / Sensor.GRAYSCALE 等）
        :return: 是否成功
        """
        try:
            # 创建 Sensor 实例
            self._sensor = Sensor(width=width, height=height)
            # 复位传感器
            self._sensor.reset()
            # 设置输出分辨率 — 320x240 用 Sensor.QVGA 常量更安全
            if width == 320 and height == 240:
                self._sensor.set_framesize(Sensor.QVGA)
            else:
                self._sensor.set_framesize(width=width, height=height)
            # 设置像素格式
            self._sensor.set_pixformat(pixformat)
            # 启动传感器
            self._sensor.run()

            self._width = width
            self._height = height
            self._framerate = framerate
            self._pixformat = pixformat
            self._initialized = True
            if debug_config.DEBUG:
                print(f"[Camera] 初始化成功 {width}x{height} @{framerate}fps")
            return True
        except Exception as e:
            print(f"[Camera] 初始化失败: {e}")
            return False

    def deinit(self):
        """关闭摄像头"""
        try:
            if self._sensor is not None:
                self._sensor.stop()
        except Exception:
            pass
        self._initialized = False
        self._sensor = None
        if debug_config.DEBUG:
            print("[Camera] 已关闭")

    # ---- 帧获取 ----

    def get_frame(self, copy_to_fb: bool = True):
        """
        获取当前帧图像
        :param copy_to_fb: 是否拷贝到帧缓冲（通常 True）
        :return: image 对象，失败返回 None
        """
        if not self._initialized or self._sensor is None:
            return None
        try:
            img = self._sensor.snapshot()
            return img
        except Exception as e:
            print(f"[Camera] 获取帧失败: {e}")
            return None

    # ---- ROI 设置 ----

    def set_roi(self, x: int, y: int, w: int, h: int):
        """
        设置感兴趣区域（软件 ROI，不改变硬件窗口）
        :param x, y: 左上角坐标
        :param w, h: 宽高
        """
        self._roi = (x, y, w, h)

    def get_roi(self) -> tuple:
        """返回当前 ROI (x, y, w, h)，未设置返回全图"""
        if self._roi is not None:
            return self._roi
        return (0, 0, self._width, self._height)

    def clear_roi(self):
        """清除 ROI，恢复全图"""
        self._roi = None

    # ---- 曝光 / 白平衡 ----
    # 注：以下方法依赖 Sensor 底层 API，若固件不支持请使用 kd_mpi_sensor_* 系列函数

    def set_auto_exposure(self, enable: bool, exposure_us: int = 0):
        """
        设置自动曝光
        :param enable:      是否开启
        :param exposure_us: 手动曝光值（微秒），仅 enable=False 时有效
        """
        try:
            self._sensor.set_auto_exposure(enable, exposure_us=exposure_us)
        except AttributeError:
            pass  # 固件不支持则忽略

    def set_auto_whitebal(self, enable: bool,
                          r_gain: float = 1.0,
                          g_gain: float = 1.0,
                          b_gain: float = 1.0):
        """
        设置自动白平衡
        """
        try:
            self._sensor.set_auto_whitebal(enable, r_gain=r_gain, g_gain=g_gain, b_gain=b_gain)
        except AttributeError:
            pass

    def set_brightness(self, value: int):
        """设置亮度"""
        try:
            self._sensor.set_brightness(value)
        except AttributeError:
            pass

    def set_contrast(self, value: int):
        """设置对比度"""
        try:
            self._sensor.set_contrast(value)
        except AttributeError:
            pass

    def set_saturation(self, value: int):
        """设置饱和度"""
        try:
            self._sensor.set_saturation(value)
        except AttributeError:
            pass

    # ---- 镜像 / 翻转 ----

    def set_vflip(self, enable: bool):
        """垂直翻转"""
        try:
            self._sensor.set_vflip(enable)
            self._flip = enable
        except AttributeError:
            pass

    def set_hmirror(self, enable: bool):
        """水平镜像"""
        try:
            self._sensor.set_hmirror(enable)
            self._mirror = enable
        except AttributeError:
            pass

    # ---- 属性 ----

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def framerate(self) -> int:
        return self._framerate

    @property
    def is_initialized(self) -> bool:
        return self._initialized
