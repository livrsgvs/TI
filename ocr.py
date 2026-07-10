"""
数字/字符识别模块 (ocr.py)
=========================
功能：识别打印的数字（如病房号 1~8）或简单字符。
支持模板匹配或轻量级 CNN（kmodel），返回识别结果及置信度。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：image 模块，可选 AI 加速 (nncase/kpu)
"""

import image

# ============================================================
# OCR 识别结果
# ============================================================
class OCRResult:
    """
    OCR 识别结果

    属性:
        text       - 识别文本
        confidence - 置信度 0~1
        rect       - 字符区域 (x, y, w, h)
        char_class - 字符类别（若为分类模型）
    """
    def __init__(self):
        self.text = ""
        self.confidence = 0.0
        self.rect = (0, 0, 0, 0)
        self.char_class = -1

    def __repr__(self):
        return f"OCR(text='{self.text}', conf={self.confidence:.2f})"


# ============================================================
# 模板匹配（传统方法）
# ============================================================
class TemplateMatcher:
    """
    基于模板匹配的数字识别。
    适用于固定字体、固定大小的数字（如数码管、印刷体）。

    使用前需调用 add_template() 添加模板。
    """

    def __init__(self):
        self._templates = {}   # { "0": image, "1": image, ... }
        self._threshold = 0.7  # 匹配相似度阈值

    def add_template(self, char: str, template_img):
        """
        添加字符模板
        :param char:         字符（"0"~"9", "A"~"Z"）
        :param template_img: 模板图像（需已二值化/灰度化）

        :使用示例:
            tm = TemplateMatcher()
            tm.add_template("1", img_bin_1)
            tm.add_template("2", img_bin_2)
        """
        self._templates[char] = template_img

    def remove_template(self, char: str):
        """删除模板"""
        self._templates.pop(char, None)

    def clear_templates(self):
        """清空所有模板"""
        self._templates.clear()

    def set_threshold(self, thr: float):
        """设置匹配阈值 (0~1)"""
        self._threshold = thr

    def match(self, img, char: str = None) -> OCRResult:
        """
        模板匹配
        :param img:  待识别图像（二值化后的单字符区域）
        :param char: 指定要匹配的字符，None 则匹配所有模板
        :return:     OCRResult
        """
        result = OCRResult()
        best_score = 0.0
        best_char = ""

        templates = {char: self._templates[char]} if char else self._templates

        for ch, tmpl in templates.items():
            try:
                # K230 v3p0: find_template(template, threshold) 仅2个参数
                # 模板必须为灰度图
                tmpl_gray = tmpl.copy().to_grayscale()
                score = img.find_template(tmpl_gray, 0.5)
                # find_template 返回 (x, y, score) 或 None
                if score and score[2] > best_score:
                    best_score = score[2]
                    best_char = ch
            except Exception:
                pass

        if best_score >= self._threshold:
            result.text = best_char
            result.confidence = best_score
        return result


# ============================================================
# OCR 识别器
# ============================================================
class OCR:
    """
    OCR 识别器（统一入口）
    支持模板匹配模式和 CNN 模式。

    :使用示例:
        ocr = OCR(mode="template")
        ocr.add_template("1", img_1)
        result = ocr.recognize(img_digit)
    """

    MODE_TEMPLATE = "template"
    MODE_CNN      = "cnn"

    def __init__(self, mode: str = "template", **kwargs):
        """
        :param mode:       识别模式 "template" 或 "cnn"
        :param kwargs:
            model_path  - CNN 模型路径（.kmodel）
            labels      - 类别标签列表 ["0","1",...,"9"]
            input_size  - 模型输入尺寸 (w, h)
        """
        self.mode = mode
        self.debug = kwargs.get('debug', False)

        if mode == self.MODE_TEMPLATE:
            self._matcher = TemplateMatcher()
            self._cnn = None
        elif mode == self.MODE_CNN:
            self._matcher = None
            self._cnn = CNNRecognizer(
                model_path=kwargs.get('model_path', ''),
                labels=kwargs.get('labels', []),
                input_size=kwargs.get('input_size', (28, 28))
            )
        else:
            raise ValueError(f"不支持的 OCR 模式: {mode}")

    # ---- 统一接口 ----

    def recognize(self, img, roi: tuple = None) -> OCRResult:
        """
        识别图像中的字符
        :param img: 图像对象
        :param roi: ROI (x, y, w, h)
        :return:    OCRResult
        """
        if roi:
            img = img.copy(roi=roi)

        if self.mode == self.MODE_TEMPLATE:
            return self._matcher.match(img)
        elif self.mode == self.MODE_CNN:
            return self._cnn.predict(img) if self._cnn else OCRResult()
        else:
            return OCRResult()

    def recognize_multi(self, img, rois: list) -> list:
        """
        识别多个区域（如多个数字）
        :param img:  图像对象
        :param rois: ROI 列表 [(x, y, w, h), ...]
        :return:     OCRResult 列表
        """
        return [self.recognize(img, roi=r) for r in rois]

    # ---- 模板模式专用 ----

    def add_template(self, char: str, template_img):
        """添加模板（仅 template 模式）"""
        if self._matcher:
            self._matcher.add_template(char, template_img)

    def set_threshold(self, thr: float):
        """设置匹配阈值"""
        if self._matcher:
            self._matcher.set_threshold(thr)

    # ---- CNN 模式专用 ----

    def load_model(self, model_path: str, labels: list, input_size: tuple = (28, 28)):
        """加载 CNN 模型（仅 cnn 模式）"""
        if self._cnn:
            self._cnn.load(model_path, labels, input_size)

    # ---- 可视化 ----

    def draw_result(self, img, result: OCRResult,
                    color: tuple = (0, 255, 0)):
        """绘制识别结果"""
        x, y, w, h = result.rect
        if w > 0 and h > 0:
            img.draw_rectangle(x, y, w, h, color=color, thickness=1)
        label = f"{result.text} ({result.confidence:.2f})"
        img.draw_string(x, y - 14, label, color=color, scale=1)


# ============================================================
# CNN 识别器（占位 - 需加载 kmodel）
# ============================================================
class CNNRecognizer:
    """
    基于 K230 NNCase / KPU 的轻量级 CNN 识别器。
    需要提供 .kmodel 模型文件。
    """
    def __init__(self, model_path: str = "", labels: list = None,
                 input_size: tuple = (28, 28)):
        self.model_path = model_path
        self.labels = labels or []
        self.input_size = input_size
        self._kpu = None
        self._loaded = False

    def load(self, model_path: str = None, labels: list = None,
             input_size: tuple = None) -> bool:
        """
        加载 Kmodel 到 KPU
        :return: 是否成功
        """
        if model_path:
            self.model_path = model_path
        if labels:
            self.labels = labels
        if input_size:
            self.input_size = input_size

        try:
            # import nncase  # K230 AI 推理库（按实际环境启用）
            # self._kpu = nncase.KPU()
            # self._kpu.load_kmodel(self.model_path)
            self._loaded = True
            print(f"[OCR-CNN] 模型加载成功: {self.model_path}")
            return True
        except Exception as e:
            print(f"[OCR-CNN] 模型加载失败: {e}")
            return False

    def predict(self, img) -> OCRResult:
        """
        CNN 推理
        :param img: 图像对象（会自动 resize 到 input_size）
        :return:    OCRResult
        """
        result = OCRResult()
        if not self._loaded:
            result.text = "N/A"
            return result

        # --- 占位：实际推理流程 ---
        # 1. img.copy().resize(self.input_size)
        # 2. 转为灰度/归一化
        # 3. self._kpu.run()
        # 4. softmax → argmax
        # ----------------------------
        result.text = "?"
        result.confidence = 0.5
        return result

    def __del__(self):
        """释放 KPU 资源"""
        if self._kpu:
            pass  # self._kpu.deinit()
