"""
循迹算法模块 (line_follow.py)
============================
功能：提取赛道线（黑线/红线），计算偏离角度或距离。
支持二值化、ROI加权回归、线段检测，输出偏差值供下位机PID使用。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：image 模块
"""

import math


# ============================================================
# 循迹结果结构
# ============================================================
class LineResult:
    """
    循迹检测结果

    属性:
        found          - 是否检测到线
        angle          - 线的角度（度），相对于垂直方向
        offset         - 横向偏移（像素），正值=右偏，负值=左偏
        line_center    - 线在图像中的中心点 (cx, cy)
        regression     - 回归线参数 (rho, theta)
        line_type      - 赛道类型: "straight"/"curve"/"cross"/"dashed"/"none"
        confidence     - 置信度 0~1
    """
    def __init__(self):
        self.found = False
        self.angle = 0.0
        self.offset = 0.0
        self.line_center = (0, 0)
        self.regression = (0.0, 0.0)   # (rho, theta)
        self.line_type = "none"
        self.confidence = 0.0

    def __repr__(self):
        return (f"Line(found={self.found}, angle={self.angle:.1f}°, "
                f"offset={self.offset:.1f}px, type={self.line_type})")


# ============================================================
# 循迹检测器
# ============================================================
class LineFollower:
    """
    循迹检测器。
    支持黑线/红线检测，可处理直线、弯道、十字、虚线等。

    :使用示例:
        lf = LineFollower(line_color="black")
        result = lf.detect(img)
        print(f"偏移: {result.offset}px, 角度: {result.angle}°")
        lf.draw(img, result)
    """

    def __init__(self, line_color: str = "black", debug: bool = False):
        """
        :param line_color: 赛道线颜色 "black" / "red"
        :param debug:      是否绘制调试信息
        """
        self.line_color = line_color
        self.debug = debug

        # ---- 可调参数 ----
        self._binary_threshold = 80      # 二值化阈值（0~255 灰度）
        self._roi_y_start = 0.3          # ROI 起始位置（图像高度的百分比）
        self._roi_y_end = 1.0            # ROI 结束位置
        self._num_segments = 5           # 水平分区数
        self._pixels_threshold = 100     # 最小线像素数

    # ---- 参数配置 ----

    def set_binary_threshold(self, thr: int):
        """设置二值化阈值"""
        self._binary_threshold = thr

    def set_roi_y(self, start_ratio: float, end_ratio: float = 1.0):
        """
        设置垂直 ROI
        :param start_ratio: 起始比例 (0~1)，如 0.3 = 从图像30%处开始
        :param end_ratio:   结束比例
        """
        self._roi_y_start = start_ratio
        self._roi_y_end = end_ratio

    # ---- 核心检测 ----

    def detect(self, img) -> LineResult:
        """
        检测赛道线并计算偏差
        :param img: 图像对象
        :return:    LineResult
        """
        result = LineResult()
        h, w = img.height(), img.width()

        # 1. 二值化
        try:
            if self.line_color == "black":
                # 黑线：灰度图 + 阈值
                img_gray = img.copy().to_grayscale()
                img_bin = img_gray.binary([(0, self._binary_threshold)], invert=False)
            elif self.line_color == "red":
                # 红线：LAB 阈值
                img_bin = img.binary([(30, 100, 15, 127, 15, 127)], invert=False)
            else:
                img_gray = img.copy().to_grayscale()
                img_bin = img_gray.binary([(0, self._binary_threshold)])
        except Exception as e:
            print(f"[LineFollower] 二值化异常: {e}")
            return result

        # 2. 设置 ROI（截取图像下方）
        y_start = int(h * self._roi_y_start)
        y_end = int(h * self._roi_y_end)
        roi = (0, y_start, w, y_end - y_start)

        # 3. 加权线性回归（OpenMV / CanMV 内置方法）
        try:
            line = img_bin.get_regression(thresholds=[(255, 255)],
                                          roi=roi,
                                          robust=True)
            if line and line.magnitude() > self._pixels_threshold:
                result.found = True
                result.regression = (line.rho(), line.theta())

                # 角度转换为度
                result.angle = math.degrees(line.theta()) - 90

                # 计算水平偏移（图像底部中心为参考）
                rho, theta = line.rho(), line.theta()
                y_ref = h - 1  # 图像底边
                if abs(math.sin(theta)) > 1e-6:
                    x_ref = (rho - y_ref * math.sin(theta)) / math.cos(theta)
                else:
                    x_ref = w / 2

                result.offset = x_ref - w / 2
                result.line_center = (int(x_ref), int(y_ref))

                # 赛道类型判断
                result.line_type = self._classify_line(result.angle, abs(result.offset), w)

                # 置信度 = 回归强度 / 最大可能值
                result.confidence = min(1.0, line.magnitude() / (w * h * 0.01))

        except Exception as e:
            print(f"[LineFollower] 回归异常: {e}")

        # 4. 备用方案：线段检测
        if not result.found:
            result = self._detect_by_segments(img_bin, roi, w, h) or result

        return result

    def _detect_by_segments(self, img_bin, roi: tuple, w: int, h: int) -> LineResult | None:
        """
        备用检测：通过 find_line_segments 组装赛道线
        """
        try:
            segments = img_bin.find_line_segments(merge_distance=10,
                                                  max_theta_diff=15, roi=roi)
            if not segments or len(segments) < 2:
                return None

            # 取最长的几条线段简单平均
            segments.sort(key=lambda s: s.length(), reverse=True)
            top_segs = segments[:min(5, len(segments))]

            avg_theta = sum(s.theta() for s in top_segs) / len(top_segs)
            avg_cx = sum((s.x1() + s.x2()) / 2 for s in top_segs) / len(top_segs)
            avg_cy = sum((s.y1() + s.y2()) / 2 for s in top_segs) / len(top_segs)

            result = LineResult()
            result.found = True
            result.angle = math.degrees(avg_theta) - 90
            result.offset = avg_cx - w / 2
            result.line_center = (int(avg_cx), int(avg_cy))
            result.confidence = 0.6
            result.line_type = "curve"
            return result
        except Exception:
            return None

    @staticmethod
    def _classify_line(angle: float, abs_offset: float, img_w: int) -> str:
        """
        根据角度和偏移量分类赛道类型
        """
        if abs(angle) < 8 and abs(abs_offset) < img_w * 0.1:
            return "straight"
        elif abs(angle) < 25:
            return "curve"
        elif abs(angle) >= 25 and abs(abs_offset) > img_w * 0.3:
            return "cross"
        else:
            return "curve"

    # ---- 十字路口 / 虚线检测 ----

    def detect_cross(self, img) -> bool:
        """
        检测是否到达十字路口
        策略：将图像分为多个水平条带，统计各条带中线的连续性
        """
        h, w = img.height(), img.width()
        strip_h = h // self._num_segments
        line_count = 0

        for i in range(self._num_segments):
            y0 = i * strip_h
            y1 = (i + 1) * strip_h
            roi = (0, y0, w, strip_h)
            try:
                blobs = img.find_blobs([(0, self._binary_threshold)],
                                       roi=roi, pixels_threshold=50, merge=True)
                if len(blobs) >= 2:  # 多个色块 → 横向线
                    line_count += 1
            except Exception:
                pass

        # 超过一半的条带出现多线 → 十字路口
        return line_count >= self._num_segments // 2

    # ---- 可视化 ----

    def draw(self, img, result: LineResult):
        """
        绘制循迹结果
        """
        if not result.found:
            return

        # 偏差线
        w, h = img.width(), img.height()
        cx_img = w // 2
        cy_bottom = h - 1
        cx_line = int(cx_img + result.offset)

        # 图像中心垂直线（参考）
        img.draw_line(cx_img, 0, cx_img, h, color=(128, 128, 128), thickness=1)
        # 检测到的线偏差点
        img.draw_line(cx_img, cy_bottom, cx_line, cy_bottom,
                      color=(0, 255, 0), thickness=3)
        img.draw_cross(cx_line, cy_bottom, color=(0, 255, 0), size=15, thickness=2)

        # 角度弧线
        angle_rad = math.radians(result.angle + 90)
        arc_r = 40
        arc_x = cx_line + int(arc_r * math.cos(angle_rad))
        arc_y = cy_bottom - int(arc_r * math.sin(angle_rad))
        img.draw_line(cx_line, cy_bottom, arc_x, arc_y,
                      color=(255, 255, 0), thickness=2)

        # 文字信息
        info = f"off:{result.offset:.0f} ang:{result.angle:.1f} {result.line_type}"
        img.draw_string(5, 5, info, color=(0, 255, 0), scale=1)

        # 回归线绘制
        if result.regression != (0.0, 0.0):
            img.draw_line_radians(result.regression[0], result.regression[1],
                                  color=(255, 0, 0), thickness=1)
