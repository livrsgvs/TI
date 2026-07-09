"""
形状/轮廓检测模块 (shape_detection.py)
===================================
功能：检测图像中的几何形状（圆、矩形、三角形等），计算面积、周长、圆度等特征。
适用于区分正方体、乒乓球、交通标志等。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：image 模块
"""

import math


# ============================================================
# 形状类型枚举
# ============================================================
class ShapeType:
    CIRCLE    = "circle"      # 圆
    RECTANGLE = "rectangle"   # 矩形
    SQUARE    = "square"      # 正方形
    TRIANGLE  = "triangle"    # 三角形
    POLYGON   = "polygon"     # 多边形
    UNKNOWN   = "unknown"     # 未知


# ============================================================
# 形状检测结果
# ============================================================
class ShapeResult:
    """
    形状检测结果

    属性:
        shape_type   - 形状类型（ShapeType 常量）
        rect         - 外接矩形 (x, y, w, h)
        centroid     - 重心 (cx, cy)
        area         - 面积
        perimeter    - 周长
        roundness    - 圆度 (4π·面积/周长²)，越接近1越圆
        corners      - 角点列表 [(x,y), ...]
        aspect_ratio - 宽高比 w/h
        confidence   - 置信度 0~1
    """
    def __init__(self):
        self.shape_type = ShapeType.UNKNOWN
        self.rect = (0, 0, 0, 0)
        self.centroid = (0, 0)
        self.area = 0
        self.perimeter = 0
        self.roundness = 0.0
        self.corners = []
        self.aspect_ratio = 1.0
        self.confidence = 0.0

    def __repr__(self):
        return (f"Shape({self.shape_type}, area={self.area}, "
                f"roundness={self.roundness:.3f}, conf={self.confidence:.2f})")


# ============================================================
# 形状检测器
# ============================================================
class ShapeDetector:
    """
    形状检测器。
    支持圆形、矩形、三角形检测，可扩展。

    :使用示例:
        sd = ShapeDetector()
        shapes = sd.detect(img, roi=(0,0,320,240))
        sd.draw_shapes(img, shapes)
    """

    # 圆度判断阈值
    CIRCLE_ROUNDNESS_MIN   = 0.75   # 圆度 > 此值 → 圆
    SQUARE_ROUNDNESS_MAX   = 0.40   # 圆度 < 此值 + 角点数4 → 正方形
    RECT_ROUNDNESS_MAX     = 0.40   # 圆度 < 此值 + 角点数4 → 矩形
    TRIANGLE_CORNERS_COUNT = 3      # 三角形角点数

    def __init__(self, debug: bool = False):
        """
        :param debug: 开启调试绘制
        """
        self.debug = debug
        self._min_area = 200       # 最小面积过滤
        self._edge_threshold = 50  # Canny 边缘检测阈值

    def set_min_area(self, area: int):
        """设置最小面积过滤"""
        self._min_area = area

    def set_edge_threshold(self, thr: int):
        """设置边缘检测阈值"""
        self._edge_threshold = thr

    # ---- 核心检测 ----

    def detect(self, img, roi: tuple = None, binary_threshold: int = None) -> list:
        """
        检测图像中的形状
        :param img:              图像对象
        :param roi:              感兴趣区域 (x, y, w, h)
        :param binary_threshold: 二值化阈值（用于边缘提取），None 则自动
        :return:                 ShapeResult 列表
        """
        shapes = []
        try:
            # 若传入二值化阈值，先进行二值化
            if binary_threshold is not None:
                img_bin = img.binary([(0, binary_threshold)], invert=False)
            else:
                img_bin = img

            # 查找矩形色块 → 用于矩形/正方形检测
            rects = img_bin.find_rects(threshold=10000, roi=roi) if hasattr(img_bin, 'find_rects') else []
            for r in rects:
                shape = self._analyze_rect(img_bin, r)
                if shape.area >= self._min_area:
                    shapes.append(shape)

            # 查找圆形
            circles = img_bin.find_circles(threshold=3500,
                                           x_margin=10, y_margin=10,
                                           r_margin=10, roi=roi) if hasattr(img_bin, 'find_circles') else []
            for c in circles:
                shape = self._analyze_circle(c)
                if shape.area >= self._min_area:
                    shapes.append(shape)

            # 查找线段 → 组合为三角形/多边形
            lines = img_bin.find_line_segments(merge_distance=10,
                                               max_theta_diff=15, roi=roi) if hasattr(img_bin, 'find_line_segments') else []
            if lines and len(lines) >= 3:
                poly_shapes = self._assemble_polygons(lines)
                for s in poly_shapes:
                    if s.area >= self._min_area:
                        shapes.append(s)

            shapes.sort(key=lambda s: s.area, reverse=True)
        except Exception as e:
            print(f"[ShapeDetector] 检测异常: {e}")

        return shapes

    # ---- 内部分析 ----

    def _analyze_rect(self, img, rect) -> ShapeResult:
        """分析矩形"""
        s = ShapeResult()
        w = rect.w()
        h = rect.h()
        ratio = w / h if h > 0 else 1.0
        area = rect.magnitude()

        # 正方形 vs 矩形
        if 0.8 <= ratio <= 1.25:
            s.shape_type = ShapeType.SQUARE
            s.confidence = min(1.0, 1.0 - abs(1.0 - ratio))
        else:
            s.shape_type = ShapeType.RECTANGLE
            s.confidence = 0.85

        s.rect = (rect.x(), rect.y(), w, h)
        s.centroid = (rect.x() + w // 2, rect.y() + h // 2)
        s.area = area
        s.perimeter = 2 * (w + h)
        s.aspect_ratio = ratio
        s.corners = rect.corners()
        return s

    def _analyze_circle(self, circle) -> ShapeResult:
        """分析圆形"""
        s = ShapeResult()
        s.shape_type = ShapeType.CIRCLE
        x, y, r = circle.x(), circle.y(), circle.r()
        s.rect = (x - r, y - r, 2 * r, 2 * r)
        s.centroid = (x, y)
        s.area = math.pi * r * r
        s.perimeter = 2 * math.pi * r
        s.roundness = 1.0
        s.confidence = circle.magnitude() / (r * r) if r > 0 else 0
        return s

    def _assemble_polygons(self, lines) -> list:
        """从线段组装多边形（简化版：按角点聚类）"""
        # 此处为框架占位，完整实现可对各线段端点进行聚类得到多边形
        return []

    # ---- 特征提取（通用） ----

    @staticmethod
    def calc_roundness(area: float, perimeter: float) -> float:
        """
        计算圆度：4π·面积 / 周长²
        :return: 0~1，正圆≈1
        """
        if perimeter <= 0:
            return 0.0
        return (4 * math.pi * area) / (perimeter * perimeter)

    @staticmethod
    def classify_by_roundness(roundness: float, corners_count: int) -> str:
        """
        根据圆度和角点数分类形状
        """
        if roundness > 0.75:
            return ShapeType.CIRCLE
        if corners_count == 4:
            return ShapeType.RECTANGLE
        if corners_count == 3:
            return ShapeType.TRIANGLE
        return ShapeType.POLYGON

    # ---- 可视化 ----

    def draw_shapes(self, img, shapes: list):
        """
        在图像上绘制形状检测结果
        """
        color_map = {
            ShapeType.CIRCLE:    (0, 255, 0),
            ShapeType.RECTANGLE: (255, 0, 0),
            ShapeType.SQUARE:    (255, 255, 0),
            ShapeType.TRIANGLE:  (0, 0, 255),
            ShapeType.POLYGON:   (255, 0, 255),
            ShapeType.UNKNOWN:   (128, 128, 128),
        }

        for s in shapes:
            c = color_map.get(s.shape_type, (128, 128, 128))
            x, y, w, h = s.rect
            img.draw_rectangle(x, y, w, h, color=c, thickness=2)
            img.draw_cross(s.centroid[0], s.centroid[1],
                           color=c, size=8, thickness=2)
            label = f"{s.shape_type[:4]}"
            img.draw_string(x, y - 12, label, color=c, scale=1)

            if self.debug:
                # 额外绘制圆度信息
                cx, cy = s.centroid
                img.draw_string(cx - 20, cy + 10,
                                f"R:{s.roundness:.2f}", color=(255, 255, 255), scale=1)
