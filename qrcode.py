"""
二维码识别模块 (qrcode.py)
==========================
功能：识别图像中的二维码（内容为数字或字符串）。
使用 img.find_qrcodes()，返回解码内容及位置信息。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：image 模块
"""


# ============================================================
# 二维码识别结果
# ============================================================
class QRCodeResult:
    """
    二维码识别结果

    属性:
        payload   - 解码内容（字符串/数字）
        rect      - 外接矩形 (x, y, w, h)
        corners   - 四个角点 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
        version   - 二维码版本号
        ecc_level - 纠错级别
        mask      - 掩码模式
        data_type - 数据类型（NUMERIC/ALPHANUMERIC/BYTE/KANJI）
        is_valid  - 是否有效解码
    """
    def __init__(self):
        self.payload = ""
        self.rect = (0, 0, 0, 0)
        self.corners = []
        self.version = 0
        self.ecc_level = 0
        self.mask = 0
        self.data_type = ""
        self.is_valid = False

    def __repr__(self):
        return f"QRCode(payload='{self.payload}', ver={self.version})"

    @staticmethod
    def from_image_qrcode(qr) -> 'QRCodeResult':
        """从 image.find_qrcodes 返回的原生对象转换"""
        r = QRCodeResult()
        r.payload = qr.payload()
        r.rect = (qr.x(), qr.y(), qr.w(), qr.h())
        r.corners = qr.corners() if hasattr(qr, 'corners') else []
        r.version = qr.version() if hasattr(qr, 'version') else 0
        r.ecc_level = qr.ecc_level() if hasattr(qr, 'ecc_level') else 0
        r.mask = qr.mask() if hasattr(qr, 'mask') else 0
        r.data_type = qr.data_type() if hasattr(qr, 'data_type') else ""
        r.is_valid = qr.is_valid() if hasattr(qr, 'is_valid') else (len(r.payload) > 0)
        return r


# ============================================================
# 二维码识别器
# ============================================================
class QRCodeDetector:
    """
    二维码识别器。

    :使用示例:
        detector = QRCodeDetector()
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
        检测并解码图像中的二维码
        :param img: 图像对象
        :param roi: 感兴趣区域 (x, y, w, h)
        :return:    QRCodeResult 列表
        :rtype: list[QRCodeResult]
        """
        results = []
        try:
            qrcodes = img.find_qrcodes(roi=roi)
            for qr in qrcodes:
                r = QRCodeResult.from_image_qrcode(qr)
                if self.debug:
                    print(f"[QRCode] 检测到: {r}")
                results.append(r)
        except Exception as e:
            print(f"[QRCode] 识别异常: {e}")
        return results

    def detect_first(self, img, roi: tuple = None):
        """
        只返回第一个识别到的二维码
        :return: QRCodeResult 或 None
        """
        results = self.detect(img, roi=roi)
        return results[0] if results else None

    def detect_payload(self, img, roi: tuple = None) -> list:
        """
        只返回解码内容字符串列表
        :return: list[str]
        """
        return [r.payload for r in self.detect(img, roi)]

    # ---- 可视化 ----

    def draw_results(self, img, results: list,
                     color_found: tuple = (0, 255, 0),
                     color_not_found: tuple = (255, 0, 0)):
        """
        在图像上绘制二维码检测结果
        :param img:              图像对象
        :param results:          QRCodeResult 列表
        :param color_found:      有效解码的绘制颜色
        :param color_not_found:  未解码的颜色
        """
        for r in results:
            color = color_found if r.is_valid else color_not_found
            x, y, w, h = r.rect

            # 外接矩形
            img.draw_rectangle(x, y, w, h, color=color, thickness=2)

            # 轮廓连线（四个角点）
            if r.corners and len(r.corners) == 4:
                pts = r.corners
                for i in range(4):
                    img.draw_line(pts[i][0], pts[i][1],
                                  pts[(i + 1) % 4][0], pts[(i + 1) % 4][1],
                                  color=color, thickness=2)

            # 文字标注
            if r.is_valid:
                label = f"{r.payload}"
                img.draw_string(x, y - 14, label, color=color, scale=1)

            # 版本号（调试）
            if self.debug and r.version > 0:
                img.draw_string(x, y + h + 2,
                                f"v{r.version}", color=(128, 128, 128), scale=1)
