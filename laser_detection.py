"""
激光笔光斑检测模块 (laser_detection.py)
======================================
功能：检测图像中激光笔照射的高亮光斑（通常为红色/白色亮点）。
通过亮度阈值或颜色阈值提取光斑，计算中心坐标。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：image 模块
"""


# ============================================================
# 光斑检测结果
# ============================================================
class LaserSpot:
    """
    激光光斑检测结果

    属性:
        found       - 是否检测到光斑
        cx, cy      - 光斑中心坐标（亚像素精度）
        radius      - 光斑半径估计（像素）
        area        - 光斑面积（像素数）
        brightness  - 光斑最大亮度
        confidence  - 置信度 0~1
    """
    def __init__(self):
        self.found = False
        self.cx = 0.0
        self.cy = 0.0
        self.radius = 0.0
        self.area = 0
        self.brightness = 0
        self.confidence = 0.0

    def __repr__(self):
        return (f"LaserSpot(found={self.found}, "
                f"center=({self.cx:.1f},{self.cy:.1f}), r={self.radius:.1f})")


# ============================================================
# 激光光斑检测器
# ============================================================
class LaserDetector:
    """
    激光笔光斑检测器。
    支持两种检测策略：
    - "brightness" : 基于亮度阈值（适用于暗环境）
    - "color"      : 基于颜色阈值（适用于亮环境下的红色激光）

    :使用示例:
        ld = LaserDetector(strategy="brightness")
        spot = ld.detect(img)
        if spot.found:
            print(f"光斑位置: ({spot.cx:.1f}, {spot.cy:.1f})")
            ld.draw(img, spot)
    """

    STRATEGY_BRIGHTNESS = "brightness"
    STRATEGY_COLOR      = "color"

    # 红色激光笔 LAB 阈值参考
    RED_LASER_THRESHOLD = [(90, 100, 40, 127, 30, 127)]
    # 亮度阈值（灰度值 0~255）
    BRIGHTNESS_THRESHOLD = 220
    # 最小面积
    MIN_AREA = 5
    # 最大面积（防止户外光线误检）
    MAX_AREA = 500

    def __init__(self, strategy: str = "brightness", debug: bool = False):
        """
        :param strategy: 检测策略 "brightness" / "color"
        :param debug:    是否调试
        """
        self.strategy = strategy
        self.debug = debug
        self._brightness_thr = self.BRIGHTNESS_THRESHOLD
        self._min_area = self.MIN_AREA
        self._max_area = self.MAX_AREA

    # ---- 参数设置 ----

    def set_brightness_threshold(self, thr: int):
        """设置亮度阈值（0~255）"""
        self._brightness_thr = thr

    def set_area_range(self, min_a: int, max_a: int):
        """设置光斑面积范围"""
        self._min_area = min_a
        self._max_area = max_a

    # ---- 核心检测 ----

    def detect(self, img, roi: tuple = None) -> LaserSpot:
        """
        检测图像中的激光光斑
        :param img: 图像对象
        :param roi: 感兴趣区域 (x, y, w, h)
        :return:    LaserSpot
        """
        if self.strategy == self.STRATEGY_BRIGHTNESS:
            return self._detect_by_brightness(img, roi)
        elif self.strategy == self.STRATEGY_COLOR:
            return self._detect_by_color(img, roi)
        return LaserSpot()

    def _detect_by_brightness(self, img, roi: tuple) -> LaserSpot:
        """
        亮度阈值法：
        1. 转灰度图
        2. 二值化（高亮为白）
        3. 找最大色块 → 光斑中心
        """
        spot = LaserSpot()
        try:
            # 灰度 + 二值化
            img_gray = img.copy().to_grayscale()
            img_bin = img_gray.binary([(self._brightness_thr, 255)], invert=False)

            # 查找高亮区域
            blobs = img_bin.find_blobs([(255, 255)], roi=roi,
                                       pixels_threshold=self._min_area,
                                       merge=True)

            if not blobs:
                return spot

            # 取面积最大的色块
            blob = max(blobs, key=lambda b: b.area())

            if blob.area() < self._min_area or blob.area() > self._max_area:
                return spot

            spot.found = True
            spot.cx = blob.cx()
            spot.cy = blob.cy()
            spot.area = blob.area()
            spot.radius = math.sqrt(blob.area() / math.pi) if blob.area() > 0 else 0

            # 原图中该区域的最大亮度
            stats = img.get_statistics(roi=(blob.x(), blob.y(), blob.w(), blob.h()))
            spot.brightness = stats.l_max()

            # 置信度：面积适中 + 亮度高 → 置信度高
            area_score = min(1.0, blob.area() / 50.0)  # 面积 50+ = 满分
            bright_score = min(1.0, (spot.brightness - self._brightness_thr) / (255 - self._brightness_thr))
            spot.confidence = (area_score + bright_score) / 2.0

        except Exception as e:
            print(f"[LaserDetector-brightness] 异常: {e}")

        return spot

    def _detect_by_color(self, img, roi: tuple) -> LaserSpot:
        """
        颜色阈值法（红色激光）：
        1. LAB 颜色空间阈值
        2. 取最大色块
        """
        spot = LaserSpot()
        try:
            blobs = img.find_blobs(self.RED_LASER_THRESHOLD, roi=roi,
                                   pixels_threshold=self._min_area,
                                   merge=True)

            if not blobs:
                return spot

            blob = max(blobs, key=lambda b: b.area())

            if blob.area() < self._min_area or blob.area() > self._max_area:
                return spot

            spot.found = True
            spot.cx = blob.cx()
            spot.cy = blob.cy()
            spot.area = blob.area()
            spot.radius = math.sqrt(blob.area() / math.pi) if blob.area() > 0 else 0
            spot.brightness = blob.density() * 255 if hasattr(blob, 'density') else 200
            spot.confidence = min(1.0, blob.area() / 50.0)

        except Exception as e:
            print(f"[LaserDetector-color] 异常: {e}")

        return spot

    # ---- 时序滤波（消抖） ----

    def filter_temporal(self, spots: list, window_size: int = 3) -> LaserSpot:
        """
        对连续多帧的光斑位置做中值滤波
        :param spots:       历史 LaserSpot 列表
        :param window_size: 滑动窗口大小
        :return:            滤波后的 LaserSpot
        """
        valid = [s for s in spots if s.found]
        if not valid:
            return LaserSpot()

        # 取最近的 window_size 帧
        recent = valid[-window_size:]

        result = LaserSpot()
        result.found = True
        result.cx = sorted(s.cx for s in recent)[len(recent) // 2]
        result.cy = sorted(s.cy for s in recent)[len(recent) // 2]
        result.radius = sum(s.radius for s in recent) / len(recent)
        result.confidence = sum(s.confidence for s in recent) / len(recent)
        return result

    # ---- 可视化 ----

    def draw(self, img, spot: LaserSpot,
             color_found: tuple = (0, 255, 0),
             color_not: tuple = (255, 0, 0)):
        """
        绘制光斑检测结果
        """
        color = color_found if spot.found else color_not

        if spot.found:
            cx, cy = int(spot.cx), int(spot.cy)
            # 十字准心
            img.draw_cross(cx, cy, color=color, size=20, thickness=2)
            # 半径圆
            r = int(spot.radius)
            img.draw_circle(cx, cy, r, color=color, thickness=1)
            # 信息文字
            info = f"L({cx},{cy}) r={spot.radius:.1f}"
            img.draw_string(cx + 15, cy - 8, info, color=color, scale=1)

        else:
            # 未检测到：显示状态
            img.draw_string(10, 10, "Laser: NOT FOUND", color=color, scale=1)


# 延迟导入（避免循环引用）
import math
