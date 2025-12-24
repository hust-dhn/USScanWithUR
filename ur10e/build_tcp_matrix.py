import argparse
from pathlib import Path

import yaml
import numpy as np

DEFAULT_TCP_YAML = "ur10e/cfg/tcp.yaml"
DEFAULT_OUT_YAML = "ur10e/cfg/T_end2tcp.yaml"

DEFAULT_CAM2TCP_YAML = "camera/cfg/T_cam2tcp.yaml"
DEFAULT_OUT_END2CAM_YAML = "ur10e/cfg/T_end2cam.yaml"


def _as_vec3(name: str, v) -> np.ndarray:
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ValueError(f"'{name}' 必须是长度为3的数组，例如 [0.0, 0.0, 1.0]")
    return np.array(v, dtype=float)


def _normalize(v: np.ndarray, name: str, eps: float = 1e-12) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        raise ValueError(f"'{name}' 的范数太小（接近0），无法归一化：{v.tolist()}")
    return v / n


def build_T_end2tcp(offset, x_label, z_label):
    """
    输入：在 end(法兰)坐标系表达的 TCP 参数
      offset: TCP原点相对end的平移 [tx, ty, tz]
      x_label: TCP x轴方向（在end坐标系表达）
      z_label: TCP z轴方向（在end坐标系表达）

    计算：
      y = z × x  (与你之前写的叉乘公式一致)
      T_end2tcp: 4x4 齐次变换矩阵（row-major写入yaml）
    """
    t = _as_vec3("offset", offset)

    x = _normalize(_as_vec3("x_label", x_label), "x_label")
    z = _normalize(_as_vec3("z_label", z_label), "z_label")

    # 让 x 与 z 正交（避免输入略有误差）
    x = x - np.dot(x, z) * z
    x = _normalize(x, "x_label(orthogonalized)")

    # 右手系：y = z × x
    y = np.cross(z, x)
    y = _normalize(y, "y_label")

    # 再反算一次 x，保证严格右手正交
    x = np.cross(y, z)
    x = _normalize(x, "x_label(recomputed)")

    T = np.eye(4, dtype=float)
    # 这里保持与你之前脚本一致：把x/y/z当作旋转矩阵的列向量
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    T[:3, 3] = t
    return T


def flatten_row_major(T: np.ndarray):
    """按 row-major 展平为 16 个数，对齐 transform: [16] 的写法。"""
    flat = T.reshape(-1).tolist()  # 默认 C-order 即 row-major
    return [float(f"{v:.16g}") for v in flat]


def load_T_from_yaml(path: str) -> np.ndarray:
    """读取形如 transform: - ... (16个数) 的4x4矩阵（row-major）。"""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or "transform" not in data:
        raise ValueError(f"YAML中未找到 'transform' 字段: {path}")

    tf = data["transform"]
    if not isinstance(tf, list) or len(tf) != 16:
        raise ValueError(f"'transform' 必须是长度16的list: {path}")

    T = np.array(tf, dtype=float).reshape(4, 4)  # row-major
    return T


def save_T_to_yaml(T: np.ndarray, path: str):
    out = {"transform": flatten_row_major(T)}
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True)


def main():
    ap = argparse.ArgumentParser()

    # 1) 生成 end2tcp
    ap.add_argument("--tcp_yaml", default=DEFAULT_TCP_YAML, help="输入：cfg/tcp.yaml 路径")
    ap.add_argument("--out_yaml", default=DEFAULT_OUT_YAML, help="输出：cfg/T_end2tcp.yaml 路径")

    # 2) 读取 cam2tcp，计算 end2cam
    ap.add_argument("--cam2tcp_yaml", default=DEFAULT_CAM2TCP_YAML, help="输入：cfg/T_cam2tcp.yaml 路径")
    ap.add_argument("--out_end2cam_yaml", default=DEFAULT_OUT_END2CAM_YAML, help="输出：cfg/T_end2cam.yaml 路径")

    args = ap.parse_args()

    # ---- 生成 T_end2tcp ----
    with open(args.tcp_yaml, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    T_end2tcp = build_T_end2tcp(
        offset=cfg["offset"],
        x_label=cfg["x_label"],
        z_label=cfg["z_label"],
    )

    save_T_to_yaml(T_end2tcp, args.out_yaml)

    # ---- 读取 T_cam2tcp ----
    T_cam2tcp = load_T_from_yaml(args.cam2tcp_yaml)

    # ---- 计算 T_end2cam ----
    # 约定：p_tcp = T_end2tcp * p_end  且  p_tcp = T_cam2tcp * p_cam
    # => p_cam = inv(T_cam2tcp) * T_end2tcp * p_end
    T_end2cam = np.linalg.inv(T_cam2tcp) @ T_end2tcp

    save_T_to_yaml(T_end2cam, args.out_end2cam_yaml)

    # ---- 打印 ----
    np.set_printoptions(precision=6, suppress=True)
    print(f"[OK] 写出: {args.out_yaml}")
    print("T_end2tcp =\n", T_end2tcp)

    print(f"\n[OK] 读取: {args.cam2tcp_yaml}")
    print("T_cam2tcp =\n", T_cam2tcp)

    print(f"\n[OK] 写出: {args.out_end2cam_yaml}")
    print("T_end2cam =\n", T_end2cam)


if __name__ == "__main__":
    main()
