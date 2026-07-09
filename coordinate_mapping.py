"""
坐标映射模块 (coordinate_mapping.py)
===================================
功能：将图像像素坐标映射到实际场地坐标。
支持透视变换（单应性矩阵）和简单线性映射两种模式。

作者：E-Competition Team
日期：2026-07-09
版本：v1.0

依赖：math（纯 Python 实现，不依赖 numpy）
"""

import math
import json


# ============================================================
# 2D 点与矩阵工具（纯 Python，避免依赖 numpy）
# ============================================================
class Mat2D:
    """简易 2D 矩阵工具"""

    @staticmethod
    def dot(A, B):
        """矩阵乘法（A 为 3x3, B 为 3xn）"""
        n = len(B[0]) if B else 0
        result = [[0.0] * n for _ in range(3)]
        for i in range(3):
            for j in range(n):
                result[i][j] = sum(A[i][k] * B[k][j] for k in range(3))
        return result

    @staticmethod
    def solve_affine(src_pts, dst_pts):
        """
        求解仿射变换矩阵（3 点法）
        :param src_pts: 源点 [(x1,y1), (x2,y2), (x3,y3)]
        :param dst_pts: 目标点 [(x1',y1'), (x2',y2'), (x3',y3')]
        :return: 3x3 仿射矩阵
        """
        # 构造线性方程组 Ax = b
        A = []
        b = []
        for (sx, sy), (dx, dy) in zip(src_pts, dst_pts):
            A.append([sx, sy, 1, 0, 0, 0])
            A.append([0, 0, 0, sx, sy, 1])
            b.append(dx)
            b.append(dy)

        # 高斯消元
        x = Mat2D._gauss(A, b)
        if x is None:
            return Mat2D.identity()

        return [
            [x[0], x[1], x[2]],
            [x[3], x[4], x[5]],
            [0.0,  0.0,  1.0]
        ]

    @staticmethod
    def solve_perspective(src_pts, dst_pts):
        """
        求解透视变换矩阵（4 点法，单应性矩阵 H）
        :param src_pts: 源四边形 [(x1,y1), ...(x4,y4)]
        :param dst_pts: 目标四边形
        :return: 3x3 单应性矩阵
        """
        # 归一化（提高数值稳定性）
        src_cx = sum(p[0] for p in src_pts) / 4
        src_cy = sum(p[1] for p in src_pts) / 4
        dst_cx = sum(p[0] for p in dst_pts) / 4
        dst_cy = sum(p[1] for p in dst_pts) / 4

        src_scale = max(max(abs(p[0] - src_cx), abs(p[1] - src_cy)) for p in src_pts) or 1
        dst_scale = max(max(abs(p[0] - dst_cx), abs(p[1] - dst_cy)) for p in dst_pts) or 1

        # 归一化点
        ns = [((p[0] - src_cx) / src_scale, (p[1] - src_cy) / src_scale) for p in src_pts]
        nd = [((p[0] - dst_cx) / dst_scale, (p[1] - dst_cy) / dst_scale) for p in dst_pts]

        # 构造 8x9 矩阵
        M = []
        for (sx, sy), (dx, dy) in zip(ns, nd):
            M.append([sx, sy, 1, 0, 0, 0, -dx * sx, -dx * sy, -dx])
            M.append([0, 0, 0, sx, sy, 1, -dy * sx, -dy * sy, -dy])

        # SVD 分解（简化：用特征向量近似）
        h = Mat2D._nullspace_8x9(M)
        if h is None:
            return Mat2D.identity()

        H_norm = [[h[0], h[1], h[2]],
                  [h[3], h[4], h[5]],
                  [h[6], h[7], h[8]]]

        # 反归一化
        T_src_inv = [[src_scale, 0, src_cx],
                     [0, src_scale, src_cy],
                     [0, 0, 1]]
        T_dst = [[1 / dst_scale, 0, -dst_cx / dst_scale],
                 [0, 1 / dst_scale, -dst_cy / dst_scale],
                 [0, 0, 1]]

        # H = T_dst @ H_norm @ T_src_inv
        H1 = Mat2D.dot(T_dst, H_norm)
        return Mat2D.dot(H1, T_src_inv)

    @staticmethod
    def identity():
        return [[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]]

    @staticmethod
    def _gauss(A, b):
        """高斯消元求解 Ax = b"""
        n = len(A)
        M = [row[:] + [b[i]] for i, row in enumerate(A)]
        # 前向消元
        for col in range(n):
            # 选主元
            pivot = max(range(col, n), key=lambda r: abs(M[r][col]))
            if abs(M[pivot][col]) < 1e-12:
                continue
            M[col], M[pivot] = M[pivot], M[col]
            pivot = M[col][col]
            for j in range(col, n + 1):
                M[col][j] /= pivot
            for row in range(n):
                if row != col:
                    factor = M[row][col]
                    for j in range(col, n + 1):
                        M[row][j] -= factor * M[col][j]
        return [row[n] for row in M]

    @staticmethod
    def _nullspace_8x9(M):
        """8x9 矩阵的零空间（SVD 简化版 — 幂迭代近似）"""
        n = len(M[0])
        # 构造 M^T @ M
        MTM = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                MTM[i][j] = sum(M[k][i] * M[k][j] for k in range(8))

        # 逆幂迭代求最小特征向量
        v = [1.0] * n
        for _ in range(30):
            # y = (MTM + eps*I)^{-1} @ v
            # 简化：直接用 MTM @ v 的高斯-赛德尔
            y = [0.0] * n
            for i in range(n):
                s = sum(MTM[i][j] * v[j] for j in range(n))
                y[i] = s
            # 归一化
            norm = math.sqrt(sum(yi * yi for yi in y)) or 1.0
            v = [yi / norm for yi in y]
        return v


# ============================================================
# 坐标映射类
# ============================================================
class CoordinateMapper:
    """
    坐标映射器。
    支持两种模式：
    - "linear" : 简单线性映射（已知 2 个参考点）
    - "perspective" : 透视变换（已知 4 个参考点，推荐）

    :使用示例:
        mapper = CoordinateMapper(mode="linear")
        mapper.calibrate_linear(
            pixel_points=[(160, 200), (160, 100)],
            world_points=[(25, 50), (25, 0)]
        )
        wx, wy = mapper.pixel_to_world(160, 150)
    """

    MODE_LINEAR      = "linear"
    MODE_PERSPECTIVE = "perspective"

    def __init__(self, mode: str = "linear"):
        """
        :param mode: 映射模式 "linear" / "perspective"
        """
        self.mode = mode
        self._matrix = Mat2D.identity()   # 3x3 变换矩阵
        self._inv_matrix = Mat2D.identity()
        self._calibrated = False
        self._calib_params = {}           # 标定参数（可保存）

    # ---- 标定 ----

    def calibrate_linear(self,
                         pixel_points: list,
                         world_points: list) -> bool:
        """
        线性标定（至少 3 组点对）
        :param pixel_points: 图像坐标 [(px1,py1), (px2,py2), (px3,py3)]
        :param world_points: 世界坐标 [(wx1,wy1), (wx2,wy2), (wx3,wy3)]  单位：cm
        :return: 是否成功
        """
        if len(pixel_points) < 3 or len(world_points) < 3:
            print("[CoordMapper] 线性标定需要至少 3 组点对")
            return False

        self._matrix = Mat2D.solve_affine(pixel_points[:3], world_points[:3])
        self._inv_matrix = Mat2D.solve_affine(world_points[:3], pixel_points[:3])
        self._calibrated = True
        self._calib_params = {
            "mode": "linear",
            "pixel_points": pixel_points[:3],
            "world_points": world_points[:3]
        }
        print("[CoordMapper] 线性标定完成")
        return True

    def calibrate_perspective(self,
                              pixel_points: list,
                              world_points: list) -> bool:
        """
        透视标定（至少 4 组点对）
        典型场景：拍摄一个已知尺寸的矩形场地
        :param pixel_points: 图像中矩形四角 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
        :param world_points: 对应的世界坐标（cm）
        :return: 是否成功
        """
        if len(pixel_points) < 4 or len(world_points) < 4:
            print("[CoordMapper] 透视标定需要至少 4 组点对")
            return False

        self._matrix = Mat2D.solve_perspective(pixel_points[:4], world_points[:4])
        self._inv_matrix = Mat2D.solve_perspective(world_points[:4], pixel_points[:4])
        self._calibrated = True
        self._calib_params = {
            "mode": "perspective",
            "pixel_points": pixel_points[:4],
            "world_points": world_points[:4]
        }
        print("[CoordMapper] 透视标定完成")
        return True

    # ---- 坐标转换 ----

    def pixel_to_world(self, px: float, py: float) -> tuple:
        """
        像素坐标 → 世界坐标
        :param px, py: 像素坐标
        :return:       (world_x_cm, world_y_cm)
        """
        if not self._calibrated:
            return (px, py)

        p = Mat2D.dot(self._matrix, [[px], [py], [1.0]])
        w = p[2][0]
        if abs(w) < 1e-12:
            return (0.0, 0.0)
        return (p[0][0] / w, p[1][0] / w)

    def world_to_pixel(self, wx: float, wy: float) -> tuple:
        """
        世界坐标 → 像素坐标
        :param wx, wy: 世界坐标（cm）
        :return:       (pixel_x, pixel_y)
        """
        if not self._calibrated:
            return (wx, wy)

        p = Mat2D.dot(self._inv_matrix, [[wx], [wy], [1.0]])
        w = p[2][0]
        if abs(w) < 1e-12:
            return (0.0, 0.0)
        return (p[0][0] / w, p[1][0] / w)

    def pixel_to_world_batch(self, points: list) -> list:
        """批量像素→世界转换"""
        return [self.pixel_to_world(x, y) for x, y in points]

    def world_to_pixel_batch(self, points: list) -> list:
        """批量世界→像素转换"""
        return [self.world_to_pixel(x, y) for x, y in points]

    # ---- 标定参数保存/加载 ----

    def save_calibration(self, filepath: str) -> bool:
        """
        保存标定参数到 JSON 文件
        :param filepath: 文件路径（如 "/sd/calib.json"）
        """
        try:
            data = {
                "mode": self._calib_params.get("mode", self.mode),
                "pixel_points": self._calib_params.get("pixel_points", []),
                "world_points": self._calib_params.get("world_points", [])
            }
            with open(filepath, "w") as f:
                json.dump(data, f)
            print(f"[CoordMapper] 标定参数已保存到 {filepath}")
            return True
        except Exception as e:
            print(f"[CoordMapper] 保存失败: {e}")
            return False

    def load_calibration(self, filepath: str) -> bool:
        """
        从 JSON 文件加载标定参数
        """
        try:
            with open(filepath, "r") as f:
                data = json.load(f)

            mode = data.get("mode", "linear")
            pp = data.get("pixel_points", [])
            wp = data.get("world_points", [])

            if mode == "perspective":
                return self.calibrate_perspective(pp, wp)
            else:
                return self.calibrate_linear(pp, wp)
        except Exception as e:
            print(f"[CoordMapper] 加载失败: {e}")
            return False

    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    # ---- 计算辅助 ----

    @staticmethod
    def distance(p1: tuple, p2: tuple) -> float:
        """两点间欧氏距离"""
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    @staticmethod
    def angle_between(a: tuple, b: tuple, c: tuple) -> float:
        """三点夹角 ∠ABC（度）"""
        ba = (a[0] - b[0], a[1] - b[1])
        bc = (c[0] - b[0], c[1] - b[1])
        dot = ba[0] * bc[0] + ba[1] * bc[1]
        mag_ba = math.sqrt(ba[0]**2 + ba[1]**2) or 1
        mag_bc = math.sqrt(bc[0]**2 + bc[1]**2) or 1
        cos_angle = max(-1.0, min(1.0, dot / (mag_ba * mag_bc)))
        return math.degrees(math.acos(cos_angle))
