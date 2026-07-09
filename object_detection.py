"""
目标检测与分类模块 (object_detection.py)
======================================
功能：基于 YOLO 等模型（kmodel）进行目标检测与分类。
输入图像，输出检测框、类别标签和置信度。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：K230 nncase/KPU 推理框架，或 image.find_features (Haar Cascade)
"""

import image

# ============================================================
# 检测框结构
# ============================================================
class DetectionBox:
    """
    目标检测结果框

    属性:
        label      - 类别名称（str）
        class_id   - 类别 ID
        confidence - 置信度 0~1
        rect       - 矩形框 (x, y, w, h)
        centroid   - 框中心 (cx, cy)
    """
    def __init__(self):
        self.label = ""
        self.class_id = -1
        self.confidence = 0.0
        self.rect = (0, 0, 0, 0)
        self.centroid = (0, 0)

    def __repr__(self):
        return "Detection(label='{}', id={}, conf={:.2f}, rect={})".format(
            self.label, self.class_id, self.confidence, self.rect)

    @staticmethod
    def from_yolo_result(obj, labels: list) -> 'DetectionBox':
        """
        从 YOLO 推理结果转换
        :param obj:    YOLO 输出的单条结果 (x, y, w, h, class_id, conf)
        :param labels: 类别名称列表
        """
        d = DetectionBox()
        d.class_id = obj[4]
        d.confidence = obj[5]
        d.rect = (obj[0], obj[1], obj[2], obj[3])
        d.centroid = (obj[0] + obj[2] // 2, obj[1] + obj[3] // 2)
        d.label = labels[d.class_id] if 0 <= d.class_id < len(labels) else f"cls_{d.class_id}"
        return d


# ============================================================
# 默认类别标签（电赛常见目标）
# ============================================================
DEFAULT_LABELS_COCO = [
    "person", "bicycle", "car", "motorcycle", "airplane",
    "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird",
    "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut",
    "cake", "chair", "couch", "potted plant", "bed",
    "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

# 电赛自定义标签（可按需扩展）
DEFAULT_LABELS_CUSTOM = [
    "cube",        # 正方体
    "ball",        # 球
    "animal",      # 动物
    "fire",        # 火源
    "car_target",  # 小车类目标
    "human",       # 人
    "arrow",       # 箭头/指示标志
    "obstacle",    # 障碍物
]


# ============================================================
# 目标检测器
# ============================================================
class ObjectDetector:
    """
    目标检测与分类器

    支持两种模式：
    - "yolo" : K230 KPU YOLO 加速推理
    - "haar" : image.find_features Haar Cascade（轻量级）

    :使用示例:
        det = ObjectDetector(mode="yolo", model_path="/sd/model.kmodel", labels=labels)
        boxes = det.detect(img)
        det.draw_boxes(img, boxes)
    """

    MODE_YOLO = "yolo"
    MODE_HAAR = "haar"

    def __init__(self, mode: str = "yolo", **kwargs):
        """
        :param mode: 检测模式 "yolo" / "haar"
        :param kwargs:
            model_path   - .kmodel 模型文件路径
            labels       - 类别标签列表
            anchors      - YOLO anchors
            input_size   - 模型输入 (w, h)
            conf_thresh  - 置信度阈值 (默认 0.5)
            nms_thresh   - NMS 阈值 (默认 0.4)
            haar_cascade - Haar Cascade 文件路径（mode="haar" 时）
        """
        self.mode = mode
        self.labels = kwargs.get('labels', DEFAULT_LABELS_CUSTOM)
        self._conf_thresh = kwargs.get('conf_thresh', 0.5)
        self._nms_thresh = kwargs.get('nms_thresh', 0.4)
        self._anchors = kwargs.get('anchors', None)
        self._input_size = kwargs.get('input_size', (320, 256))
        self._kpu = None
        self._loaded = False
        self.debug = kwargs.get('debug', False)

        if mode == self.MODE_YOLO:
            self._model_path = kwargs.get('model_path', '')
        elif mode == self.MODE_HAAR:
            self._haar_cascade = kwargs.get('haar_cascade', '')
        else:
            raise ValueError(f"不支持的模式: {mode}")

    # ---- 模型加载 ----

    def load_model(self, model_path: str = None, labels: list = None) -> bool:
        """
        加载 YOLO 模型
        :return: 成功返回 True
        """
        if model_path:
            self._model_path = model_path
        if labels:
            self.labels = labels

        try:
            # --- K230 KPU 加载（按实际环境启用）---
            # import nncase
            # self._kpu = nncase.KPU()
            # self._kpu.load_kmodel(self._model_path)
            self._loaded = True
            print(f"[ObjectDetector] YOLO 模型加载完成: {self._model_path}")
            print(f"[ObjectDetector] 类别数: {len(self.labels)}")
            return True
        except Exception as e:
            print(f"[ObjectDetector] 模型加载失败: {e}")
            return False

    # ---- 核心检测 ----

    def detect(self, img, roi: tuple = None) -> list:
        """
        执行目标检测
        :param img: 图像对象
        :param roi: ROI (x, y, w, h)
        :return:    DetectionBox 列表
        :rtype: list[DetectionBox]
        """
        if self.mode == self.MODE_YOLO:
            return self._detect_yolo(img, roi)
        elif self.mode == self.MODE_HAAR:
            return self._detect_haar(img, roi)
        return []

    def _detect_yolo(self, img, roi: tuple = None) -> list:
        """
        YOLO KPU 推理（占位 - 需加载 kmodel 后实现）
        """
        results = []
        if not self._loaded:
            return results

        # --- 占位：实际推理流程 ---
        # 1. img.copy(roi).resize(self._input_size)
        # 2. 转 RGB888 → 归一化
        # 3. kpu.run() 获取输出张量
        # 4. 解码 + NMS
        # 5. 转换为 DetectionBox 列表
        # for obj in decoded:
        #     d = DetectionBox.from_yolo_result(obj, self.labels)
        #     if d.confidence >= self._conf_thresh:
        #         results.append(d)
        # ----------------------------
        return results

    def _detect_haar(self, img, roi: tuple = None) -> list:
        """
        Haar Cascade 检测（可用于人脸等）
        """
        results = []
        try:
            features = img.find_features(image.HaarCascade(self._haar_cascade),
                                         roi=roi, threshold=self._conf_thresh)
            for f in features:
                d = DetectionBox()
                d.rect = f
                d.centroid = (f[0] + f[2] // 2, f[1] + f[3] // 2)
                d.confidence = 1.0
                d.label = "haar_object"
                results.append(d)
        except Exception as e:
            print(f"[ObjectDetector-Haar] 检测异常: {e}")
        return results

    # ---- 后处理 ----

    def filter_by_confidence(self, boxes: list, min_conf: float) -> list:
        """按置信度过滤"""
        return [b for b in boxes if b.confidence >= min_conf]

    def filter_by_label(self, boxes: list, labels: list) -> list:
        """按类别标签过滤"""
        return [b for b in boxes if b.label in labels]

    def get_largest(self, boxes: list):
        """返回面积最大的检测框"""
        if not boxes:
            return None
        return max(boxes, key=lambda b: b.rect[2] * b.rect[3])

    def get_centermost(self, boxes: list, img_w: int, img_h: int):
        """返回最靠近图像中心的检测框"""
        if not boxes:
            return None
        cx, cy = img_w // 2, img_h // 2
        return min(boxes, key=lambda b: ((b.centroid[0] - cx) ** 2 +
                                         (b.centroid[1] - cy) ** 2))

    # ---- 可视化 ----

    def draw_boxes(self, img, boxes: list,
                   color_by_label: bool = True,
                   draw_label: bool = True,
                   draw_conf: bool = True):
        """
        绘制检测框
        """
        if not boxes:
            return

        # 给每种标签分配颜色
        unique_labels = list(set(b.label for b in boxes))
        colors = self._gen_colors(len(unique_labels))
        label_color = dict(zip(unique_labels, colors))

        for box in boxes:
            color = label_color.get(box.label, (0, 255, 0)) if color_by_label else (0, 255, 0)
            x, y, w, h = box.rect
            img.draw_rectangle(x, y, w, h, color=color, thickness=2)

            if draw_label:
                lbl = box.label
                if draw_conf:
                    lbl += f" {box.confidence:.2f}"
                img.draw_string(x, y - 14, lbl, color=color, scale=1)

    @staticmethod
    def _gen_colors(n: int) -> list:
        """生成 n 种可区分的颜色"""
        palette = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255),
            (255, 255, 0), (255, 0, 255), (0, 255, 255),
            (128, 0, 0), (0, 128, 0), (0, 0, 128),
            (128, 128, 0), (128, 0, 128), (0, 128, 128),
        ]
        return [palette[i % len(palette)] for i in range(n)]
