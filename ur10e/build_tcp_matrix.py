import argparse
from pathlib import Path

import yaml
import numpy as np

# 输入/输出路径（按你项目习惯）
DEFAULT_TCP_YAML = "ur10e/cfg/tcp.yaml"
DEFAULT_OUT_END2TCP_YAML = "ur10e/cfg/T_end2tcp.yaml"

DEFAULT_CAM2TCP_YAML = "camera/cfg/T_cam2tcp.yaml"  # 你说的 RGB-sensor2 在 tcp 下
DEFAULT_LC_YAML = "camera/cfg/config_lc.yaml"      # sensor0 为原点的 RGB(s2) 外参

DEFAULT_OUT_S0_TO_S2_YAML = "camera/cfg/T_sensor0_to_sensor2.yaml"
DEFAULT_OUT_END2S2_YAML = "camera/cfg/T_end2sensor2.yaml"
DEFAULT_OUT_END2S0_YAML = "camera/cfg/T_end2sensor0.yaml"


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
    由 tcp.yaml 里的 offset/x_label/z_label 生成 T_end2tcp
    约定：p_tcp = T_end2tcp @ p_end
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
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    T[:3, 3] = t
    return T


def flatten_row_major(T: np.ndarray):
    flat = T.reshape(-1).tolist()  # row-major
    return [float(f"{v:.16g}") for v in flat]


def load_T_from_yaml(path: str) -> np.ndarray:
    """读取形如 transform: [16] 的 4x4（row-major）"""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or "transform" not in data:
        raise ValueError(f"YAML中未找到 'transform' 字段: {path}")

    tf = data["transform"]
    if not isinstance(tf, list) or len(tf) != 16:
        raise ValueError(f"'transform' 必须是长度16的list: {path}")

    return np.array(tf, dtype=float).reshape(4, 4)


def save_T_to_yaml(T: np.ndarray, path: str):
    out = {"transform": flatten_row_major(T)}
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True)


# ====== config_lc: Rotation/Translation -> 4x4 ======
def rotvec_to_matrix(r: np.ndarray) -> np.ndarray:
    """Rodrigues 旋转向量(弧度) -> 3x3"""
    r = np.asarray(r, dtype=float).reshape(3)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3, dtype=float)

    k = r / theta
    kx, ky, kz = k.tolist()
    K = np.array([[0.0, -kz,  ky],
                  [kz,  0.0, -kx],
                  [-ky, kx,  0.0]], dtype=float)

    I = np.eye(3, dtype=float)
    return I + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def euler_xyz_to_matrix(e: np.ndarray, degrees: bool = False) -> np.ndarray:
    """欧拉角 xyz（固定轴 x->y->z）-> 3x3"""
    e = np.asarray(e, dtype=float).reshape(3)
    if degrees:
        e = np.deg2rad(e)
    rx, ry, rz = e.tolist()

    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    Rx = np.array([[1, 0, 0],
                   [0, cx, -sx],
                   [0, sx, cx]], dtype=float)
    Ry = np.array([[cy, 0, sy],
                   [0, 1, 0],
                   [-sy, 0, cy]], dtype=float)
    Rz = np.array([[cz, -sz, 0],
                   [sz, cz, 0],
                   [0, 0, 1]], dtype=float)

    return Rz @ Ry @ Rx


def build_T_from_translation_rotation_mm(translation_mm, rotation_3, rotation_type: str) -> np.ndarray:
    """
    config_lc.yaml 里 Translation 单位是 mm，这里转成 m。
    rotation_type:
      - 'rotvec'       : Rodrigues 旋转向量（弧度）
      - 'euler_xyz_rad': 欧拉角 xyz（弧度）
      - 'euler_xyz_deg': 欧拉角 xyz（度）
    """
    t_mm = _as_vec3("Translation", translation_mm)
    t_m = 1 * t_mm / 1000.0  # mm -> m

    r = _as_vec3("Rotation", rotation_3)

    if rotation_type == "rotvec":
        R = rotvec_to_matrix(r)
    elif rotation_type == "euler_xyz_rad":
        R = euler_xyz_to_matrix(r, degrees=False)
    elif rotation_type == "euler_xyz_deg":
        R = euler_xyz_to_matrix(r, degrees=True)
    else:
        raise ValueError(f"未知 rotation_type: {rotation_type}")

    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = t_m
    return T


def load_sensor_T_from_config_lc(lc_cfg: dict, sensor_key: str, rotation_type: str) -> np.ndarray:
    """
    从 config_lc.yaml 读取 camera.sensorX.Translation/Rotation，构建 4x4。
    """
    cam = lc_cfg.get("camera", {})
    sensor = cam.get(sensor_key)
    if sensor is None:
        raise KeyError(f"config_lc.yaml 中找不到 camera.{sensor_key}")

    trans = sensor.get("Translation", sensor.get("translation"))
    rot = sensor.get("Rotation", sensor.get("rotation"))
    if trans is None or rot is None:
        raise KeyError(f"camera.{sensor_key} 缺少 Translation/Rotation 字段")

    return build_T_from_translation_rotation_mm(trans, rot, rotation_type)


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("--tcp_yaml", default=DEFAULT_TCP_YAML)
    ap.add_argument("--out_end2tcp_yaml", default=DEFAULT_OUT_END2TCP_YAML)

    ap.add_argument("--cam2tcp_yaml", default=DEFAULT_CAM2TCP_YAML)

    ap.add_argument("--lc_yaml", default=DEFAULT_LC_YAML)
    ap.add_argument("--lc_rotation_type", default="rotvec",
                    choices=["rotvec", "euler_xyz_rad", "euler_xyz_deg"])

    ap.add_argument("--out_s0_to_s2_yaml", default=DEFAULT_OUT_S0_TO_S2_YAML)
    ap.add_argument("--out_end2s2_yaml", default=DEFAULT_OUT_END2S2_YAML)
    ap.add_argument("--out_end2s0_yaml", default=DEFAULT_OUT_END2S0_YAML)

    args = ap.parse_args()

    # 1) T_end2tcp
    with open(args.tcp_yaml, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    T_end2tcp = build_T_end2tcp(
        offset=cfg["offset"],
        x_label=cfg["x_label"],
        z_label=cfg["z_label"],
    )
    save_T_to_yaml(T_end2tcp, args.out_end2tcp_yaml)

    # 2) 读 T_sensor2(cam)2tcp（你给的 cam2tcp.yaml）
    T_s2_2_tcp = load_T_from_yaml(args.cam2tcp_yaml)

    # 3) 从 config_lc 得到 T_sensor0_to_sensor2
    with open(args.lc_yaml, "r", encoding="utf-8") as f:
        lc_cfg = yaml.safe_load(f)

    # config_lc 里 sensor0/sensor2 都给了一个“在同一参考系下”的位姿（sensor0 常为零）
    T_ref_s0 = load_sensor_T_from_config_lc(lc_cfg, "sensor0", args.lc_rotation_type)
    T_ref_s2 = load_sensor_T_from_config_lc(lc_cfg, "sensor2", args.lc_rotation_type)

    # 统一转成：T_sensor0_to_sensor2
    # p_s2 = T_s0_2_s2 @ p_s0
    T_s0_2_s2 = np.linalg.inv(T_ref_s0) @ T_ref_s2
    save_T_to_yaml(T_s0_2_s2, args.out_s0_to_s2_yaml)

    # 4) 先算 T_end2sensor2（end 在 RGB(s2) 下）
    # p_tcp = T_end2tcp @ p_end
    # p_s2  = inv(T_s2_2_tcp) @ p_tcp
    T_end2s2 = np.linalg.inv(T_s2_2_tcp) @ T_end2tcp
    save_T_to_yaml(T_end2s2, args.out_end2s2_yaml)

    # 5) 目标：T_end2sensor0（end 在 Depth(s0) 下）
    # p_s0 = inv(T_s0_2_s2) @ p_s2
    T_end2s0 = np.linalg.inv(T_s0_2_s2) @ T_end2s2
    save_T_to_yaml(T_end2s0, args.out_end2s0_yaml)

    # 打印
    np.set_printoptions(precision=6, suppress=True)
    print(f"[OK] 写出: {args.out_end2tcp_yaml}\nT_end2tcp =\n{T_end2tcp}\n")
    print(f"[OK] 读取: {args.cam2tcp_yaml}\nT_sensor2_to_tcp =\n{T_s2_2_tcp}\n")
    print(f"[OK] 写出: {args.out_s0_to_s2_yaml}\nT_sensor0_to_sensor2 =\n{T_s0_2_s2}\n")
    print(f"[OK] 写出: {args.out_end2s2_yaml}\nT_end2sensor2 =\n{T_end2s2}\n")
    print(f"[OK] 写出: {args.out_end2s0_yaml}\nT_end2sensor0 =\n{T_end2s0}\n")


if __name__ == "__main__":
    main()
