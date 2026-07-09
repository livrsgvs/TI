"""
目标跟踪模块 (tracking.py)
=========================
功能：对检测到的目标进行跟踪（基于颜色直方图的 CamShift 或 KCF 等）。
输入初始目标位置/颜色，持续输出当前帧目标位置。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：image 模块
"""

import time


# ============================================================
# 跟踪状态
# ============================================================
class TrackState:
    IDLE      = "idle"       # 未初始化
    TRACKING  = "tracking"   # 跟踪中
    LOST      = "lost"       # 目标丢失
    RECOVERED = "recovered"  # 重新找回


# ============================================================
# 跟踪结果
# ============================================================
class TrackResult:
    """
    目标跟踪结果

    属性:
        state      - 跟踪状态（TrackState 枚举）
        rect       - 当前目标框 (x, y, w, h)
        centroid   - 当前目标中心 (cx, cy)
        confidence - 跟踪置信度 0~1
        frame_id   - 帧序号
        velocity   - 速度估计 (vx, vy) 像素/帧
    """
    def __init__(self):
        self.state = TrackState.IDLE
        self.rect = (0, 0, 0, 0)
        self.centroid = (0, 0)
        self.confidence = 0.0
        self.frame_id = 0
        self.velocity = (0.0, 0.0)

    def __repr__(self):
        return (f"Track(state={self.state}, rect={self.rect}, "
                f"conf={self.confidence:.2f})")


# ============================================================
# 目标跟踪器
# ============================================================
class Tracker:
    """
    目标跟踪器（统一入口）。

    支持两种模式：
    - "camshift"：基于颜色直方图（Meanshift / CamShift）
    - "kcf"     ：基于相关滤波器（需扩展或使用 OpenMV 内置）

    使用流程：
        1. tracker.init(...) 初始化
        2. 每帧调用 tracker.update(img) 获取结果
        3. 必要时 tracker.reset() 重置

    :使用示例:
        trk = Tracker(mode="camshift")
        trk.init_by_roi(img, roi=(100, 80, 40, 40))
        while True:
            img = cam.get_frame()
            result = trk.update(img)
            if result.state == TrackState.TRACKING:
                trk.draw(img, result)
    """

    MODE_CAMSHIFT = "camshift"
    MODE_KCF      = "kcf"

    def __init__(self, mode: str = "camshift", debug: bool = False):
        """
        :param mode:  跟踪模式
        :param debug: 是否开启调试
        """
        self.mode = mode
        self.debug = debug
        self._tracker = None         # 底层跟踪器句柄
        self._roi = (0, 0, 0, 0)
        self._state = TrackState.IDLE
        self._frame_id = 0
        self._lost_frames = 0
        self._max_lost_frames = 30   # 丢失多少帧后标记为 LOST
        self._prev_centroid = (0, 0)

    # ---- 初始化 ----

    def init_by_roi(self, img, roi: tuple) -> bool:
        """
        通过 ROI 初始化跟踪器（手动框选目标）
        :param img: 图像对象
        :param roi: (x, y, w, h)
        :return: 成功返回 True
        """
        self._roi = roi
        self._state = TrackState.TRACKING
        self._frame_id = 0
        self._lost_frames = 0

        try:
            if self.mode == self.MODE_CAMSHIFT:
                # 提取目标区域颜色直方图
                self._tracker = self._extract_histogram(img, roi)
            elif self.mode == self.MODE_KCF:
                # KCF 初始化（依赖底层实现）
                self._tracker = roi  # 占位
            print(f"[Tracker] 初始化成功, roi={roi}")
            return True
        except Exception as e:
            print(f"[Tracker] 初始化失败: {e}")
            self._state = TrackState.IDLE
            return False

    def init_by_color(self, img, color_thresholds: list,
                      roi: tuple = None) -> bool:
        """
        通过颜色阈值自动初始化跟踪器（找最大色块）
        :param img:               图像对象
        :param color_thresholds:  LAB 阈值
        :param roi:               搜索区域
        :return:                  成功返回 True
        """
        blobs = img.find_blobs(color_thresholds, roi=roi,
                               pixels_threshold=100, merge=True)
        if not blobs:
            print("[Tracker] 未找到目标色块")
            return False

        blob = max(blobs, key=lambda b: b.area())
        target_roi = (blob.x(), blob.y(), blob.w(), blob.h())
        return self.init_by_roi(img, target_roi)

    # ---- 更新 ----

    def update(self, img) -> TrackResult:
        """
        更新跟踪，处理下一帧
        :param img: 当前帧图像
        :return:    TrackResult
        """
        self._frame_id += 1
        result = TrackResult()
        result.frame_id = self._frame_id

        if self._state == TrackState.IDLE:
            result.state = TrackState.IDLE
            return result

        try:
            if self.mode == self.MODE_CAMSHIFT:
                new_rect = self._update_camshift(img)
            elif self.mode == self.MODE_KCF:
                new_rect = self._update_kcf(img)
            else:
                new_rect = None

            if new_rect is not None:
                x, y, w, h = new_rect
                result.rect = new_rect
                result.centroid = (x + w // 2, y + h // 2)
                result.confidence = 0.85 if self._lost_frames == 0 else 0.5

                # 速度估计
                cx, cy = result.centroid
                if self._prev_centroid != (0, 0):
                    result.velocity = (cx - self._prev_centroid[0],
                                       cy - self._prev_centroid[1])
                self._prev_centroid = result.centroid

                if self._state == TrackState.LOST:
                    self._state = TrackState.RECOVERED
                else:
                    self._state = TrackState.TRACKING
                self._lost_frames = 0
            else:
                self._lost_frames += 1
                result.state = (TrackState.LOST
                                if self._lost_frames > self._max_lost_frames
                                else TrackState.TRACKING)
                # 使用上一次位置
                result.rect = self._roi
                result.centroid = self._prev_centroid
                result.confidence = max(0.0, 1.0 - self._lost_frames / self._max_lost_frames)

        except Exception as e:
            print(f"[Tracker] 更新异常: {e}")
            result.state = TrackState.LOST
            self._lost_frames += 1

        result.state = self._state
        return result

    def _update_camshift(self, img):
        """
        CamShift 更新：在当前帧中搜索与目标直方图最匹配的区域
        """
        if self._tracker is None:
            return None
        # --- 占位：实际 CamShift 流程 ---
        # 1. 根据上一帧位置 + 搜索窗口计算 back projection
        # 2. mean shift 迭代收敛
        # 3. 返回新位置
        # --- 简化版：使用颜色直方图匹配 ---
        try:
            x, y, w, h = self._roi
            search_roi = (max(0, x - w//2), max(0, y - h//2),
                          min(img.width() - x, w * 2), min(img.height() - y, h * 2))
            # 用 find_blobs 查找匹配色块
            blobs = img.find_blobs(self._tracker, roi=search_roi,
                                   pixels_threshold=w * h // 4, merge=True)
            if blobs:
                blob = max(blobs, key=lambda b: b.area())
                new_rect = (blob.x(), blob.y(), blob.w(), blob.h())
                self._roi = new_rect  # 更新 ROI 用于下一帧
                return new_rect
        except Exception:
            pass
        return None

    def _update_kcf(self, img):
        """KCF 更新（占位）"""
        return None

    @staticmethod
    def _extract_histogram(img, roi: tuple):
        """
        提取 ROI 区域的 LAB 颜色直方图（简化版：返回 LAB 统计信息作为阈值）
        """
        x, y, w, h = roi
        stats = img.get_statistics(roi=roi)
        # 构造阈值范围：以统计均值为中心 ± 偏移
        margin = 30
        l_mean = stats.l_mean()
        a_mean = stats.a_mean()
        b_mean = stats.b_mean()
        return [(max(0, l_mean - margin), min(100, l_mean + margin),
                 max(-128, a_mean - margin), min(127, a_mean + margin),
                 max(-128, b_mean - margin), min(127, b_mean + margin))]

    # ---- 重置 ----

    def reset(self):
        """重置跟踪器"""
        self._tracker = None
        self._state = TrackState.IDLE
        self._frame_id = 0
        self._lost_frames = 0
        self._prev_centroid = (0, 0)
        print("[Tracker] 已重置")

    @property
    def is_tracking(self) -> bool:
        return self._state in (TrackState.TRACKING, TrackState.RECOVERED)

    @property
    def is_lost(self) -> bool:
        return self._state == TrackState.LOST

    # ---- 可视化 ----

    def draw(self, img, result: TrackResult,
             color_tracking: tuple = (0, 255, 0),
             color_lost: tuple = (255, 0, 0)):
        """绘制跟踪结果"""
        color = color_tracking if self.is_tracking else color_lost
        x, y, w, h = result.rect
        img.draw_rectangle(x, y, w, h, color=color, thickness=2)
        cx, cy = result.centroid
        img.draw_cross(cx, cy, color=color, size=10, thickness=2)

        # 速度箭头
        vx, vy = result.velocity
        if vx != 0 or vy != 0:
            arrow_len = max(5, min(50, int((vx**2 + vy**2) ** 0.5 * 5)))
            ex = cx + int(vx * 5)
            ey = cy + int(vy * 5)
            img.draw_arrow(cx, cy, ex, ey, color=(0, 255, 255), thickness=2)

        # 状态文字
        state_str = f"{result.state} F{result.frame_id}"
        img.draw_string(x, y - 14, state_str, color=color, scale=1)
