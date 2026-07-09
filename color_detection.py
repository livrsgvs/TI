"""
颜色识别模块 (color_detection.py)
===============================
功能：基于 LAB 颜色空间分割，识别特定颜色的目标。
输出颜色块列表（坐标、面积、中心点），支持阈值在线调节与可视化标注。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：image 模块
"""


# ============================================================
# 默认颜色阈值（LAB 色彩空间）
# 格式：[(L_min, L_max, A_min, A_max, B_min, B_max), ...]
# 用户需根据现场光照标定
# ============================================================
COLOR_THRESHOLDS = {
    "red":    [(30, 100,  15, 127,  15, 127)],   # 红色
    "green":  [(30, 100, -64,  -8,  -8,  64)],   # 绿色
    "blue":   [( 0,  60, -20,  40, -80, -20)],   # 蓝色
    "yellow": [(50, 100, -20,  20,  20, 127)],   # 黄色
    "white":  [(80, 100, -20,  20, -20,  20)],   # 白色
    "black":  [( 0,  30, -20,  20, -20,  20)],   # 黑色
    "orange": [(30, 100,  20, 127,  20, 127)],   # 橙色
    "purple": [(20,  80,  20, 127, -80, -20)],   # 紫色
}


# ============================================================
# 颜色检测结果结构
# ============================================================
class ColorBlob:
    """
    颜色块数据类

    属性:
        x, y, w, h   - 外接矩形
        area          - 面积（像素数）
        cx, cy        - 重心坐标
        density       - 饱满度（面积/外接矩形面积）
        code          - 颜色块编号
    """
    def __init__(self, x=0, y=0, w=0, h=0, area=0, cx=0, cy=0, density=0.0, code=0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.area = area
        self.cx = cx
        self.cy = cy
        self.density = density
        self.code = code

    def __repr__(self):
        return (f"Blob(x={self.x}, y={self.y}, w={self.w}, h={self.h}, "
                f"area={self.area}, cx={self.cx}, cy={self.cy})")

    @staticmethod
    def from_image_blob(blob) -> 'ColorBlob':
        """从 image.find_blobs 返回的原生 blob 转换"""
        return ColorBlob(
            x=blob.x(), y=blob.y(),
            w=blob.w(), h=blob.h(),
            area=blob.area(),
            cx=blob.cx(), cy=blob.cy(),
            density=blob.density() if hasattr(blob, 'density') else 0.0,
            code=blob.code() if hasattr(blob, 'code') else 0
        )


# ============================================================
# 颜色检测器
# ============================================================
class ColorDetector:
    """
    颜色检测器。

    :使用示例:
        detector = ColorDetector()
        blobs = detector.detect(img, COLOR_THRESHOLDS["red"], roi=(0,0,160,120))
        detector.draw_blobs(img, blobs)
    """

    def __init__(self, debug: bool = False):
        """
        :param debug: 是否开启调试绘制
        """
        self.debug = debug
        self._thresholds = COLOR_THRESHOLDS.copy()  # 运行时阈值（可在线修改）
        self._pixels_threshold = 100     # 最小面积过滤
        self._area_threshold = 100
        self._merge = True               # 是否合并相邻色块
        self._margin = 10                # 合并边距

    # ---- 阈值管理 ----

    def set_threshold(self, name: str, thresholds: list):
        """
        动态设置某颜色的阈值
        :param name:       颜色名（"red"/"green"/...）
        :param thresholds: 阈值列表 [(L_min, L_max, A_min, A_max, B_min, B_max), ...]
        """
        self._thresholds[name] = thresholds

    def get_threshold(self, name: str):
        """获取某颜色的阈值"""
        return self._thresholds.get(name, [])

    def set_min_area(self, area: int):
        """设置最小色块面积过滤"""
        self._pixels_threshold = area
        self._area_threshold = area

    def set_merge(self, enable: bool, margin: int = 10):
        """设置色块合并"""
        self._merge = enable
        self._margin = margin

    # ---- 核心检测 ----

    def detect(self, img, color_name: str = None,
               thresholds: list = None, roi: tuple = None,
               invert: bool = False) -> list:
        """
        检测指定颜色的色块
        :param img:         图像对象（sensor.snapshot() 返回）
        :param color_name:  预设颜色名（"red"/"green"/...），与 thresholds 二选一
        :param thresholds:  手动传入阈值列表
        :param roi:         感兴趣区域 (x, y, w, h)，None 为全图
        :param invert:      是否反转阈值（查找非该颜色区域）
        :return:             ColorBlob 列表，按面积降序排列
        :rtype: list[ColorBlob]
        """
        # 解析阈值
        if thresholds is not None:
            th = thresholds
        elif color_name is not None:
            th = self._thresholds.get(color_name)
            if th is None:
                print(f"[ColorDetector] 未知颜色名: {color_name}")
                return []
        else:
            print("[ColorDetector] 必须指定 color_name 或 thresholds")
            return []

        try:
            raw_blobs = img.find_blobs(th,
                                       roi=roi,
                                       x_stride=1, y_stride=1,
                                       pixels_threshold=self._pixels_threshold,
                                       area_threshold=self._area_threshold,
                                       merge=self._merge,
                                       margin=self._margin,
                                       invert=invert)

            # 转换为统一结构并按面积排序
            blobs = [ColorBlob.from_image_blob(b) for b in raw_blobs]
            blobs.sort(key=lambda b: b.area, reverse=True)
            return blobs
        except Exception as e:
            print(f"[ColorDetector] 检测异常: {e}")
            return []

    def detect_multi(self, img, color_names: list, roi: tuple = None) -> dict:
        """
        同时检测多种颜色
        :param img:         图像对象
        :param color_names: 颜色名列表 ["red", "blue"]
        :param roi:         ROI
        :return:            { "red": [ColorBlob, ...], "blue": [...] }
        """
        results = {}
        for name in color_names:
            results[name] = self.detect(img, color_name=name, roi=roi)
        return results

    def get_largest(self, img, color_name: str, roi: tuple = None):
        """
        获取指定颜色的最大色块
        :return: ColorBlob 或 None
        """
        blobs = self.detect(img, color_name=color_name, roi=roi)
        return blobs[0] if blobs else None

    # ---- 可视化 ----

    def draw_blobs(self, img, blobs: list,
                   draw_rect: bool = True, draw_cross: bool = True,
                   draw_label: bool = True, color: tuple = (255, 0, 0)):
        """
        在图像上绘制色块标注
        :param img:        图像对象
        :param blobs:      ColorBlob 列表
        :param draw_rect:  是否绘制外接矩形
        :param draw_cross: 是否绘制十字准心
        :param draw_label: 是否绘制标签（面积）
        :param color:      绘制颜色 (R, G, B)
        """
        for blob in blobs:
            if draw_rect:
                img.draw_rectangle(blob.x, blob.y, blob.w, blob.h, color=color)
            if draw_cross:
                img.draw_cross(blob.cx, blob.cy, color=color, size=10, thickness=2)
            if draw_label:
                img.draw_string(blob.x, blob.y - 10,
                                f"A{blob.area}", color=color, scale=1)

    def draw_biggest(self, img, blob: ColorBlob, color: tuple = (0, 255, 0)):
        """
        高亮绘制最大色块（绿色十字+矩形）
        """
        if blob is None:
            return
        img.draw_rectangle(blob.x, blob.y, blob.w, blob.h,
                           color=color, thickness=3)
        img.draw_cross(blob.cx, blob.cy, color=color, size=15, thickness=3)
        img.draw_string(blob.x + blob.w + 2, blob.y,
                        f"({blob.cx},{blob.cy})", color=color, scale=1)
