#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import sys
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

# ===========================
# 配置区
# ===========================

PLY_PATH = r"/home/zzz/ros2_ws/src/my_ur10e_control/camera/pc_datas/0002.ply"

PLY_DIR = r"/home/zzz/ros2_ws/src/my_ur10e_control/camera/examples"
AUTO_PICK_LATEST_PLY = False

# 不下采样：显示全部点
VOXEL_SIZE = 0.0
MAX_POINTS = 0

WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 800
DEFAULT_POINT_SIZE = 2.0
DEFAULT_DARK_BG = True
DEFAULT_SHOW_AXES = True
DEFAULT_SHOW_BBOX = False  # 是否显示包围盒（可选）

DEFAULT_COLORIZE_BY_HEIGHT_IF_NO_COLOR = True

# 坐标轴大小：看不到就调大（比如 10 / 50 / 100）
AXES_SIZE = 10

# 数值显示保留的小数位
COORD_DECIMALS = 3

# ===========================
# 配置区结束
# ===========================


def pick_ply_file() -> Path:
    p = Path(PLY_PATH).expanduser()
    if p.exists() and p.is_file():
        return p.resolve()

    if AUTO_PICK_LATEST_PLY:
        d = Path(PLY_DIR).expanduser()
        if not d.exists():
            raise FileNotFoundError(f"PLY_DIR not found: {d}")
        candidates = sorted(d.glob("*.ply"), key=lambda x: x.stat().st_mtime, reverse=True)
        if not candidates:
            raise FileNotFoundError(f"No .ply found in: {d}")
        return candidates[0].resolve()

    raise FileNotFoundError(f"PLY_PATH not found: {p}")


def colorize_by_height(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    pts = np.asarray(pcd.points)
    if pts.size == 0:
        return pcd

    z = pts[:, 2]
    zmin, zmax = float(np.min(z)), float(np.max(z))
    if abs(zmax - zmin) < 1e-9:
        t = np.zeros_like(z)
    else:
        t = (z - zmin) / (zmax - zmin)

    r = np.clip(1.5 * (t - 0.33), 0, 1)
    g = np.clip(1.5 * (0.66 - np.abs(t - 0.5)), 0, 1)
    b = np.clip(1.5 * (0.66 - t), 0, 1)
    colors = np.stack([r, g, b], axis=1).astype(np.float32)

    pcd2 = o3d.geometry.PointCloud(pcd)
    pcd2.colors = o3d.utility.Vector3dVector(colors)
    return pcd2


def downsample(pcd: o3d.geometry.PointCloud, voxel: float, max_points: int) -> o3d.geometry.PointCloud:
    p = pcd
    if voxel and voxel > 0:
        p = p.voxel_down_sample(voxel)

    if max_points and max_points > 0:
        n = np.asarray(p.points).shape[0]
        if n > max_points:
            idx = np.random.choice(n, size=max_points, replace=False)
            p = p.select_by_index(idx)

    return p


def fmt3(v):
    d = COORD_DECIMALS
    return f"({v[0]:.{d}f}, {v[1]:.{d}f}, {v[2]:.{d}f})"


class PlyViewerApp:
    def __init__(self, ply_path: Path):
        self.ply_path = ply_path

        self.show_axes = DEFAULT_SHOW_AXES
        self.show_bbox = DEFAULT_SHOW_BBOX
        self.dark_bg = DEFAULT_DARK_BG
        self.point_size = float(DEFAULT_POINT_SIZE)

        self.pcd_raw = o3d.io.read_point_cloud(str(self.ply_path))
        if self.pcd_raw.is_empty():
            raise RuntimeError(f"Empty/invalid PLY: {self.ply_path}")

        self.pcd = downsample(self.pcd_raw, VOXEL_SIZE, MAX_POINTS)

        self.use_height_colors = False
        if (not self.pcd.has_colors()) and DEFAULT_COLORIZE_BY_HEIGHT_IF_NO_COLOR:
            self.use_height_colors = True

        # 坐标轴
        self.axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=AXES_SIZE)

        # 包围盒（可选）
        self.bbox = self.pcd.get_axis_aligned_bounding_box()
        self.bbox.color = (1.0, 1.0, 1.0)

        gui.Application.instance.initialize()
        self.window = gui.Application.instance.create_window(
            f"PLY Viewer - {self.ply_path.name}", WINDOW_WIDTH, WINDOW_HEIGHT
        )

        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)

        self._apply_background()
        self.scene_widget.scene.set_lighting(rendering.Open3DScene.LightingProfile.SOFT_SHADOWS, (0, 0, 0))

        self.panel = gui.Vert(0, gui.Margins(12, 12, 12, 12))
        self.panel.preferred_width = 340
        self._build_panel()

        self.window.add_child(self.scene_widget)
        self.window.add_child(self.panel)
        self.window.set_on_layout(self._on_layout)
        self.window.set_on_key(self._on_key)

        self._refresh_geometry()
        self._reset_camera()
        self._update_xyz_labels()

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.panel.frame = gui.Rect(r.x, r.y, self.panel.preferred_width, r.height)
        self.scene_widget.frame = gui.Rect(
            r.x + self.panel.preferred_width, r.y, r.width - self.panel.preferred_width, r.height
        )

    def _apply_background(self):
        if self.dark_bg:
            self.scene_widget.scene.set_background([0.06, 0.06, 0.075, 1.0])
        else:
            self.scene_widget.scene.set_background([0.95, 0.95, 0.97, 1.0])

    def _make_material(self):
        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        mat.point_size = float(self.point_size)
        return mat

    def _refresh_geometry(self):
        self.scene_widget.scene.clear_geometry()

        p = self.pcd
        if self.use_height_colors:
            p = colorize_by_height(p)

        self.scene_widget.scene.add_geometry("pcd", p, self._make_material())

        if self.show_axes:
            self.scene_widget.scene.add_geometry("axes", self.axes, rendering.MaterialRecord())

        if self.show_bbox:
            self.scene_widget.scene.add_geometry("bbox", self.bbox, rendering.MaterialRecord())

    def _reset_camera(self):
        bbox = self.pcd.get_axis_aligned_bounding_box()
        self.scene_widget.setup_camera(60.0, bbox, bbox.get_center())

    def _update_xyz_labels(self):
        pts = np.asarray(self.pcd.points)
        if pts.size == 0:
            self.xyz_min_label.text = "XYZ min: (n/a)"
            self.xyz_max_label.text = "XYZ max: (n/a)"
            self.xyz_ctr_label.text = "XYZ center: (n/a)"
            return

        mn = pts.min(axis=0)
        mx = pts.max(axis=0)
        ctr = (mn + mx) / 2.0

        self.xyz_min_label.text = f"XYZ min: {fmt3(mn)}"
        self.xyz_max_label.text = f"XYZ max: {fmt3(mx)}"
        self.xyz_ctr_label.text = f"XYZ center: {fmt3(ctr)}"

    def _build_panel(self):
        self.panel.add_child(gui.Label("Controls"))
        self.panel.add_child(gui.Label(f"File: {self.ply_path.name}"))

        n_raw = np.asarray(self.pcd_raw.points).shape[0]
        n_show = np.asarray(self.pcd.points).shape[0]
        self.info_label = gui.Label(f"Points: raw={n_raw:,}  showing={n_show:,}")
        self.panel.add_child(self.info_label)

        self.panel.add_fixed(10)

        # 显示 XYZ 数值（min/max/center）
        self.panel.add_child(gui.Label("XYZ info"))
        self.xyz_min_label = gui.Label("XYZ min: ...")
        self.xyz_max_label = gui.Label("XYZ max: ...")
        self.xyz_ctr_label = gui.Label("XYZ center: ...")
        self.panel.add_child(self.xyz_min_label)
        self.panel.add_child(self.xyz_max_label)
        self.panel.add_child(self.xyz_ctr_label)

        self.panel.add_fixed(10)

        self.panel.add_child(gui.Label("Point size"))
        self.size_slider = gui.Slider(gui.Slider.DOUBLE)
        self.size_slider.set_limits(1.0, 10.0)
        self.size_slider.double_value = self.point_size
        self.size_slider.set_on_value_changed(self._on_point_size)
        self.panel.add_child(self.size_slider)

        self.panel.add_fixed(8)

        self.color_checkbox = gui.Checkbox("Colorize by height (Z)  (C)")
        self.color_checkbox.checked = self.use_height_colors
        self.color_checkbox.set_on_checked(self._on_color_mode)
        self.panel.add_child(self.color_checkbox)

        self.axes_checkbox = gui.Checkbox("Show axes  (A)")
        self.axes_checkbox.checked = self.show_axes
        self.axes_checkbox.set_on_checked(self._on_axes_toggle)
        self.panel.add_child(self.axes_checkbox)

        self.bbox_checkbox = gui.Checkbox("Show bounding box  (V)")
        self.bbox_checkbox.checked = self.show_bbox
        self.bbox_checkbox.set_on_checked(self._on_bbox_toggle)
        self.panel.add_child(self.bbox_checkbox)

        self.bg_checkbox = gui.Checkbox("Dark background  (B)")
        self.bg_checkbox.checked = self.dark_bg
        self.bg_checkbox.set_on_checked(self._on_bg_toggle)
        self.panel.add_child(self.bg_checkbox)

        self.panel.add_fixed(10)

        btn_row = gui.Horiz(8)
        reset_btn = gui.Button("Reset view (R)")
        reset_btn.set_on_clicked(self._reset_camera)
        btn_row.add_child(reset_btn)

        reload_btn = gui.Button("Reload (L)")
        reload_btn.set_on_clicked(self._on_reload)
        btn_row.add_child(reload_btn)

        self.panel.add_child(btn_row)
        self.panel.add_fixed(8)
        self.panel.add_child(gui.Label("Hotkeys: R reset | A axes | V bbox | C color | B bg | L reload"))

    def _on_point_size(self, v):
        self.point_size = float(v)
        self._refresh_geometry()

    def _on_color_mode(self, checked):
        self.use_height_colors = bool(checked)
        self._refresh_geometry()

    def _on_axes_toggle(self, checked):
        self.show_axes = bool(checked)
        self._refresh_geometry()

    def _on_bbox_toggle(self, checked):
        self.show_bbox = bool(checked)
        self._refresh_geometry()

    def _on_bg_toggle(self, checked):
        self.dark_bg = bool(checked)
        self._apply_background()
        self._refresh_geometry()

    def _on_reload(self):
        self.pcd_raw = o3d.io.read_point_cloud(str(self.ply_path))
        self.pcd = downsample(self.pcd_raw, VOXEL_SIZE, MAX_POINTS)

        if (not self.pcd.has_colors()) and DEFAULT_COLORIZE_BY_HEIGHT_IF_NO_COLOR:
            self.use_height_colors = True
            self.color_checkbox.checked = True

        self.bbox = self.pcd.get_axis_aligned_bounding_box()
        self.bbox.color = (1.0, 1.0, 1.0)

        n_raw = np.asarray(self.pcd_raw.points).shape[0]
        n_show = np.asarray(self.pcd.points).shape[0]
        self.info_label.text = f"Points: raw={n_raw:,}  showing={n_show:,}"

        self._refresh_geometry()
        self._reset_camera()
        self._update_xyz_labels()

    def _on_key(self, event):
        if event.type != gui.KeyEvent.Type.DOWN:
            return gui.Widget.EventCallbackResult.IGNORED

        if event.key == ord("R"):
            self._reset_camera()
            return gui.Widget.EventCallbackResult.HANDLED
        if event.key == ord("A"):
            self.show_axes = not self.show_axes
            self.axes_checkbox.checked = self.show_axes
            self._refresh_geometry()
            return gui.Widget.EventCallbackResult.HANDLED
        if event.key == ord("V"):
            self.show_bbox = not self.show_bbox
            self.bbox_checkbox.checked = self.show_bbox
            self._refresh_geometry()
            return gui.Widget.EventCallbackResult.HANDLED
        if event.key == ord("C"):
            self.use_height_colors = not self.use_height_colors
            self.color_checkbox.checked = self.use_height_colors
            self._refresh_geometry()
            return gui.Widget.EventCallbackResult.HANDLED
        if event.key == ord("B"):
            self.dark_bg = not self.dark_bg
            self.bg_checkbox.checked = self.dark_bg
            self._apply_background()
            self._refresh_geometry()
            return gui.Widget.EventCallbackResult.HANDLED
        if event.key == ord("L"):
            self._on_reload()
            return gui.Widget.EventCallbackResult.HANDLED

        return gui.Widget.EventCallbackResult.IGNORED

    def run(self):
        gui.Application.instance.run()


def main():
    ply_path = pick_ply_file()
    app = PlyViewerApp(ply_path)
    app.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)
