"""
图像采集模块 (camera.py)
=======================
功能：初始化摄像头，提供图像帧获取、ROI设置、自动曝光/白平衡控制。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：CanMV K230 的 sensor 与 image 模块
"""

import sensor
import image
import time


# ============================================================
# 默认配置常量
# ============================================================
DEFAULT_WIDTH   = 320     # 默认分辨率宽
DEFAULT_HEIGHT  = 240     # 默认分辨率高
DEFAULT_FPS     = 60      # 默认帧率
DEFAULT_PIXFMT  = sensor.RGB565  # 默认像素格式


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
        :param pixformat: 像素格式（sensor.RGB565 / sensor.GRAYSCALE 等）
        :return: 是否成功
        """
        try:
            sensor.reset()
            sensor.set_pixformat(pixformat)
            sensor.set_framesize(self._resolve_framesize(width, height))
            sensor.set_windowing((0, 0, width, height))
            sensor.set_framerate(framerate)
            sensor.skip_frames(time=200)  # 跳过不稳定帧

            self._width = width
            self._height = height
            self._framerate = framerate
            self._pixformat = pixformat
            self._initialized = True
            print(f"[Camera] 初始化成功 {width}x{height} @{framerate}fps")
            return True
        except Exception as e:
            print(f"[Camera] 初始化失败: {e}")
            return False

    def deinit(self):
        """关闭摄像头"""
        try:
            sensor.shutdown()
        except Exception:
            pass
        self._initialized = False
        print("[Camera] 已关闭")

    # ---- 帧获取 ----

    def get_frame(self, copy_to_fb: bool = True):
        """
        获取当前帧图像
        :param copy_to_fb: 是否拷贝到帧缓冲（通常 True）
        :return: image 对象，失败返回 None
        """
        if not self._initialized:
            return None
        try:
            img = sensor.snapshot()
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

    def set_auto_exposure(self, enable: bool, exposure_us: int = 0):
        """
        设置自动曝光
        :param enable:      是否开启
        :param exposure_us: 手动曝光值（微秒），仅 enable=False 时有效
        """
        sensor.set_auto_exposure(enable, exposure_us=exposure_us)

    def set_auto_whitebal(self, enable: bool,
                          r_gain: float = 1.0,
                          g_gain: float = 1.0,
                          b_gain: float = 1.0):
        """
        设置自动白平衡
        :param enable: 是否开启
        :param r_gain / g_gain / b_gain: 手动增益
        """
        sensor.set_auto_whitebal(enable, r_gain=r_gain, g_gain=g_gain, b_gain=b_gain)

    def set_brightness(self, value: int):
        """设置亮度 (-3 ~ +3)"""
        sensor.set_brightness(value)

    def set_contrast(self, value: int):
        """设置对比度 (-3 ~ +3)"""
        sensor.set_contrast(value)

    def set_saturation(self, value: int):
        """设置饱和度 (-3 ~ +3)"""
        sensor.set_saturation(value)

    # ---- 镜像 / 翻转 ----

    def set_vflip(self, enable: bool):
        """垂直翻转"""
        sensor.set_vflip(enable)
        self._flip = enable

    def set_hmirror(self, enable: bool):
        """水平镜像"""
        sensor.set_hmirror(enable)
        self._mirror = enable

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

    # ---- 内部工具 ----

    @staticmethod
    def _resolve_framesize(w: int, h: int):
        """
        根据分辨率选择最接近的 sensor 帧尺寸常量
        """
        if w <= 320 and h <= 240:
            return sensor.QVGA
        elif w <= 640 and h <= 480:
            return sensor.VGA
        elif w <= 800 and h <= 600:
            return sensor.SVGA
        elif w <= 1280 and h <= 720:
            return sensor.HD
        elif w <= 1920 and h <= 1080:
            return sensor.FHD
        else:
            return sensor.QVGA  # 默认
