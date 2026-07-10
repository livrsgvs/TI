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
        _blobs         - 调试用：检测到的色块列表
        _roi           - 调试用：ROI 区域
    """
    def __init__(self):
        self.found = False
        self.angle = 0.0
        self.offset = 0.0
        self.line_center = (0, 0)
        self.regression = (0.0, 0.0)   # (rho, theta)
        self.line_type = "none"
        self.confidence = 0.0
        self._blobs = []
        self._roi = (0, 0, 0, 0)

    def __repr__(self):
        return "Line(found={}, angle={:.1f}deg, offset={:.1f}px, type={})".format(
            self.found, self.angle, self.offset, self.line_type)


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
        # 灰度双阈值：像素值在 [low, high] 范围内视为"线"
        # 黑线白底 → (0, 80)；白线黑底 → (180, 255)
        self._binary_threshold_low = 0      # 灰度下限
        self._binary_threshold_high = 80    # 灰度上限
        self._roi_y_start = 0.3             # ROI 起始位置（图像高度的百分比）
        self._roi_y_end = 1.0               # ROI 结束位置
        self._num_segments = 5              # 水平分区数
        self._pixels_threshold = 100        # 最小线像素数

        # ---- 平滑滤波 ----
        self._smooth_factor = 0.65          # EMA 平滑系数（0=不过滤, 1=完全锁死）
        self._prev_offset = 0.0
        self._prev_angle = 0.0
        self._smooth_ready = False

    # ---- 参数配置 ----

    def set_binary_threshold(self, low: int, high: int):
        """设置二值化双阈值（灰度下限, 灰度上限）"""
        self._binary_threshold_low = low
        self._binary_threshold_high = high

    def set_smooth_factor(self, alpha: float):
        """设置 EMA 平滑系数（0~1，越大越平滑但响应越慢）"""
        self._smooth_factor = max(0.0, min(1.0, alpha))

    def set_roi_y(self, start_ratio: float, end_ratio: float = 1.0):
        """
        设置垂直 ROI
        :param start_ratio: 起始比例 (0~1)，如 0.3 = 从图像30%处开始
        :param end_ratio:   结束比例
        """
        self._roi_y_start = start_ratio
        self._roi_y_end = end_ratio

    def reset_smoothing(self):
        """丢线时重置平滑历史"""
        self._smooth_ready = False
        self._prev_offset = 0.0
        self._prev_angle = 0.0

    # ---- 核心检测 ----

    def detect(self, img) -> LineResult:
        """
        检测赛道线并计算偏差。
        策略：二值化 → 回归线 → blob质心（回退）
        :param img: 图像对象
        :return:    LineResult
        """
        result = LineResult()
        h, w = img.height(), img.width()

        # 0. 计算 ROI
        y_start = int(h * self._roi_y_start)
        y_end = int(h * self._roi_y_end)
        roi = (0, y_start, w, y_end - y_start)
        result._roi = roi

        # 1. 二值化
        try:
            if self.line_color == "black":
                # 黑线：灰度图 + 双阈值 → 提取暗像素
                img_gray = img.copy(roi=roi).to_grayscale()
                img_bin = img_gray.binary(
                    [(self._binary_threshold_low, self._binary_threshold_high)],
                    invert=False)
            elif self.line_color == "red":
                # 红线：LAB 阈值
                img_roi = img.copy(roi=roi)
                img_bin = img_roi.binary(
                    [(30, 100, 15, 127, 15, 127)], invert=False)
            else:
                img_gray = img.copy(roi=roi).to_grayscale()
                img_bin = img_gray.binary(
                    [(self._binary_threshold_low, self._binary_threshold_high)],
                    invert=False)
        except Exception as e:
            if self.debug:
                print(f"[LineFollower] 二值化异常: {e}")
            return result

        # 2. 线性回归（首选）
        try:
            line = img_bin.get_regression([(255, 255)])
            if line and line.magnitude() > self._pixels_threshold:
                result.found = True
                result.regression = (line.rho(), line.theta())
                result.angle = math.degrees(line.theta()) - 90

                # 计算水平偏移
                rho, theta = line.rho(), line.theta()
                roi_h = y_end - y_start
                y_ref = roi_h - 1
                if abs(math.sin(theta)) > 1e-6:
                    x_ref = (rho - y_ref * math.sin(theta)) / math.cos(theta)
                else:
                    x_ref = w / 2
                result.offset = x_ref - w / 2
                result.line_center = (int(x_ref), int(y_ref + y_start))
                result.line_type = self._classify_line(result.angle, abs(result.offset), w)
                result.confidence = min(1.0, line.magnitude() / (w * roi_h * 0.01))
        except Exception as e:
            if self.debug:
                print(f"[LineFollower] 回归异常: {e}")

        # 3. 回退方案1：线段检测
        if not result.found:
            result = self._detect_by_segments(img_bin, roi, w, h) or result

        # 4. 回退方案2：所有色块面积加权质心
        if not result.found:
            result = self._detect_by_blobs(img_bin, roi, w, h) or result

        # 5. EMA 平滑（抑制帧间抖动）
        if result.found:
            if self._smooth_ready and self._smooth_factor > 0.0:
                a = self._smooth_factor
                result.offset = a * self._prev_offset + (1.0 - a) * result.offset
                result.angle = a * self._prev_angle + (1.0 - a) * result.angle
            self._prev_offset = result.offset
            self._prev_angle = result.angle
            self._smooth_ready = True
        else:
            self._smooth_ready = False

        return result

    def _detect_by_segments(self, img_bin, roi: tuple, w: int, h: int):
        """
        回退方案1：通过 find_line_segments 组装赛道线。
        注意：img_bin 已是 ROI 裁剪后的图像，不要再裁一次。
        """
        try:
            segments = img_bin.find_line_segments(merge_distance=10,
                                                  max_theta_diff=15)
            if not segments or len(segments) < 2:
                return None

            # 取最长的几条线段简单平均
            segments.sort(key=lambda s: s.length(), reverse=True)
            top_segs = segments[:min(5, len(segments))]

            avg_theta = sum(s.theta() for s in top_segs) / len(top_segs)
            avg_cx = sum((s.x1() + s.x2()) / 2 for s in top_segs) / len(top_segs)
            avg_cy = sum((s.y1() + s.y2()) / 2 for s in top_segs) / len(top_segs)

            # 坐标还原到原图（+ROI 偏移）
            x_start, y_start = roi[0], roi[1]

            result = LineResult()
            result.found = True
            result.angle = math.degrees(avg_theta) - 90
            result.offset = (avg_cx + x_start) - w / 2
            result.line_center = (int(avg_cx + x_start), int(avg_cy + y_start))
            result.confidence = 0.6
            result.line_type = "curve"
            return result
        except Exception:
            return None

    def _detect_by_blobs(self, img_bin, roi: tuple, w: int, h: int):
        """
        回退方案2：所有色块面积加权质心。
        即使线被拆成多个相邻色块，加权质心也落在它们之间，不会跳动。
        """
        try:
            blobs = img_bin.find_blobs(
                [(255, 255)],
                pixels_threshold=max(30, self._pixels_threshold // 3),
                area_threshold=20,
                merge=True)
            if not blobs:
                return None

            x_start, y_start_roi = roi[0], roi[1]

            # 所有色块面积加权质心（不是只取最大那个！）
            total_area = sum(b.area() for b in blobs)
            if total_area < self._pixels_threshold:
                return None

            weighted_cx = sum(b.cx() * b.area() for b in blobs) / total_area
            weighted_cy = sum(b.cy() * b.area() for b in blobs) / total_area

            cx = weighted_cx + x_start
            cy = weighted_cy + y_start_roi

            # 角度：取前两大色块，用它们质心连线估算
            sorted_blobs = sorted(blobs, key=lambda b: b.area(), reverse=True)
            if len(sorted_blobs) >= 2:
                b0, b1 = sorted_blobs[0], sorted_blobs[1]
                dx = b1.cx() - b0.cx()
                dy = b1.cy() - b0.cy()
                blob_angle = math.degrees(math.atan2(dy, dx)) if abs(dx) > 0.5 else 0.0
            elif hasattr(sorted_blobs[0], 'rotation_deg'):
                blob_angle = math.degrees(sorted_blobs[0].rotation_deg())
            else:
                blob_angle = 0.0

            result = LineResult()
            result.found = True
            result._blobs = blobs
            result._roi = roi
            result.line_center = (int(cx), int(cy))
            result.offset = cx - w / 2
            result.angle = blob_angle
            result.line_type = self._classify_line(result.angle, abs(result.offset), w)
            result.confidence = min(1.0, total_area / (w * roi[3] * 0.05))
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
                blobs = img.find_blobs([(self._binary_threshold_low, self._binary_threshold_high)],
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
        绘制循迹结果到 IDE 显示器。
        始终画 ROI 边界框；检测成功时额外画线位置、blob 合并包络。
        """
        w, h = img.width(), img.height()
        cx_img = w // 2
        GREEN = (0, 255, 0)
        RED = (255, 0, 0)
        YELLOW = (255, 255, 0)
        GRAY = (128, 128, 128)
        ORANGE = (255, 165, 0)

        # 1. 图像中心参考线（始终画）
        img.draw_line(cx_img, 0, cx_img, h, color=GRAY, thickness=1)

        # 2. ROI 绿色边框（始终画）
        rx, ry, rw, rh = result._roi
        if rw > 0 and rh > 0:
            img.draw_rectangle(rx, ry, rw, rh, color=GREEN, thickness=1)

        # 3. 阈值 + 平滑参数（左上角）
        info_top = "thr:[{}-{}] ROI:{:.0f}% sm:{:.2f}".format(
            self._binary_threshold_low, self._binary_threshold_high,
            self._roi_y_start * 100, self._smooth_factor)
        img.draw_string(5, 5, info_top, color=GREEN, scale=1)

        if not result.found:
            img.draw_string(5, 20, "NO LINE", color=RED, scale=1)
            return

        # ====== 检测到线 ======
        cx_line = int(cx_img + result.offset)

        # 4. 所有色块的合并外包络（橘色虚线）+ 各色块（绿色细框）
        if result._blobs:
            # 计算合并外包络
            all_x, all_y, all_x2, all_y2 = [], [], [], []
            for blob in result._blobs:
                bx, by, bw, bh = blob.rect()
                all_x.append(bx + rx)
                all_y.append(by + ry)
                all_x2.append(bx + rx + bw)
                all_y2.append(by + ry + bh)
                # 各色块画绿色细框
                img.draw_rectangle(bx + rx, by + ry, bw, bh,
                                   color=GREEN, thickness=1)
            if all_x:
                env_x = min(all_x)
                env_y = min(all_y)
                env_w = max(all_x2) - env_x
                env_h = max(all_y2) - env_y
                img.draw_rectangle(env_x, env_y, env_w, env_h,
                                   color=ORANGE, thickness=2)
            # 色块计数
            img.draw_string(5, 35, "blobs:{} area:{}".format(
                len(result._blobs),
                sum(b.area() for b in result._blobs)),
                color=GREEN, scale=1)

        # 5. 加权质心十字（黄色，更大更明显）
        cx, cy = result.line_center
        img.draw_cross(cx, cy, color=YELLOW, size=14, thickness=2)

        # 6. 偏差指示线（底部绿线）
        cy_bottom = h - 1
        img.draw_line(cx_img, cy_bottom, cx_line, cy_bottom,
                      color=GREEN, thickness=3)
        img.draw_cross(cx_line, cy_bottom, color=GREEN, size=15, thickness=2)

        # 7. 角度弧线
        angle_rad = math.radians(result.angle + 90)
        arc_r = 40
        arc_x = cx_line + int(arc_r * math.cos(angle_rad))
        arc_y = cy_bottom - int(arc_r * math.sin(angle_rad))
        img.draw_line(cx_line, cy_bottom, arc_x, arc_y,
                      color=YELLOW, thickness=2)

        # 8. 回归线
        if result.regression != (0.0, 0.0):
            img.draw_line_radians(result.regression[0], result.regression[1],
                                  color=RED, thickness=1)

        # 9. 状态文字
        info = "off:{:.0f} ang:{:.1f} {} {:.0f}%".format(
            result.offset, result.angle, result.line_type,
            result.confidence * 100)
        img.draw_string(5, 20, info, color=GREEN, scale=1)
