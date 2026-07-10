"""
条形码识别模块 (barcode.py)
===========================
功能：识别图像中的条形码（支持 EAN-13, CODE128, CODE39 等）。
使用 img.find_barcodes()，框出条形码区域并标注解码内容。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：image 模块
"""


# ============================================================
# 条形码识别结果
# ============================================================
class BarcodeResult:
    """
    条形码识别结果

    属性:
        payload   - 解码内容（字符串）
        type      - 条形码类型（"EAN13"/"CODE128"/...）
        rect      - 外接矩形 (x, y, w, h)
        rotation  - 旋转角度（弧度）
        quality   - 识别质量/置信度 0~1
    """
    def __init__(self):
        self.payload = ""
        self.type = ""
        self.rect = (0, 0, 0, 0)
        self.rotation = 0.0
        self.quality = 0.0

    def __repr__(self):
        return f"Barcode(payload='{self.payload}', type={self.type})"

    @staticmethod
    def from_image_barcode(bc) -> 'BarcodeResult':
        """从 image.find_barcodes 返回的原生对象转换"""
        r = BarcodeResult()
        r.payload = bc.payload()
        r.type = bc.type()
        r.rect = (bc.x(), bc.y(), bc.w(), bc.h())
        r.rotation = bc.rotation()
        r.quality = bc.quality()
        return r


# ============================================================
# 条形码识别器
# ============================================================
class BarcodeDetector:
    """
    条形码识别器。

    :使用示例:
        detector = BarcodeDetector()
        results = detector.detect(img)
        detector.draw_results(img, results)
    """

    def __init__(self, debug: bool = False):
        """
        :param debug: 是否开启调试输出
        """
        self.debug = debug

    # ---- 核心识别 ----

    def detect(self, img, roi: tuple = None) -> list:
        """
        检测并解码图像中的条形码
        :param img: 图像对象
        :param roi: 感兴趣区域 (x, y, w, h)
        :return:    BarcodeResult 列表
        :rtype: list[BarcodeResult]
        """
        results = []
        try:
            barcodes = img.find_barcodes(roi=roi)
            for bc in barcodes:
                r = BarcodeResult.from_image_barcode(bc)
                if self.debug:
                    print(f"[Barcode] 检测到: {r}")
                results.append(r)
        except Exception as e:
            print(f"[Barcode] 识别异常: {e}")
        return results

    def detect_first(self, img, roi: tuple = None):
        """
        只返回第一个识别到的条形码
        :return: BarcodeResult 或 None
        """
        results = self.detect(img, roi=roi)
        return results[0] if results else None

    def detect_payload(self, img, roi: tuple = None) -> list:
        """
        只返回解码内容字符串列表（快捷方式）
        :return: list[str]
        """
        return [r.payload for r in self.detect(img, roi)]

    # ---- 可视化 ----

    def draw_results(self, img, results: list,
                     color: tuple = (0, 255, 0), draw_payload: bool = True):
        """
        在图像上绘制条形码检测结果
        :param img:          图像对象
        :param results:      BarcodeResult 列表
        :param color:        绘制颜色 (R, G, B)
        :param draw_payload: 是否绘制解码内容文字
        """
        for r in results:
            x, y, w, h = r.rect
            # 外接矩形
            img.draw_rectangle(x, y, w, h, color=color, thickness=2)
            # 旋转角度线
            cx, cy = x + w // 2, y + h // 2
            import math
            end_x = cx + int(h * 0.5 * math.sin(r.rotation))
            end_y = cy - int(h * 0.5 * math.cos(r.rotation))
            img.draw_line(cx, cy, end_x, end_y, color=color, thickness=1)
            # 文字
            if draw_payload:
                label = f"{r.payload}"
                img.draw_string(x, y - 14, label, color=color, scale=1)
                img.draw_string(x, y + h + 2, r.type, color=(128, 128, 128), scale=1)
