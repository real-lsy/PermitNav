#!/usr/bin/env python3

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from PIL import Image
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from jie_map_msgs.srv import SaveNavigationMapPackage
from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
    QDoubleSpinBox,
    QCheckBox,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker
import vtk
from vtk.util import numpy_support
try:
    from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
except ModuleNotFoundError:
    from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class RosMapImportNode(Node):
    def __init__(self) -> None:
        super().__init__(f"ros_map_import_gui_node_{time.time_ns()}")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grid_pub = self.create_publisher(OccupancyGrid, "/import_occupancy_grid", qos)
        self.occupied_sub = self.create_subscription(
            Marker, "/octomap_occupied_markers", self._on_occupied, qos
        )
        self.preblocked_sub = self.create_subscription(
            Marker, "/preblocked_cells_markers", self._on_preblocked, qos
        )
        self.traversable_sub = self.create_subscription(
            Marker, "/traversable_cells_markers", self._on_traversable, qos
        )
        self.save_client = self.create_client(
            SaveNavigationMapPackage, "/map_package_manager/save_package"
        )
        self._latest_occupied: tuple[np.ndarray, np.ndarray] | None = None
        self._latest_preblocked: tuple[np.ndarray, np.ndarray] | None = None
        self._latest_traversable: tuple[np.ndarray, np.ndarray] | None = None
        self._layer_dirty = False

    def publish_grid(self, msg: OccupancyGrid) -> None:
        self.grid_pub.publish(msg)

    def _on_occupied(self, msg: Marker) -> None:
        self._store_marker("occupied", msg)

    def _on_preblocked(self, msg: Marker) -> None:
        self._store_marker("preblocked", msg)

    def _on_traversable(self, msg: Marker) -> None:
        self._store_marker("traversable", msg)

    def _store_marker(self, layer_name: str, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return
        points = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
        scale = np.array([msg.scale.x, msg.scale.y, msg.scale.z], dtype=np.float32)
        setattr(self, f"_latest_{layer_name}", (points, scale))
        self._layer_dirty = True

    def consume_layers(self):
        if not self._layer_dirty:
            return None
        self._layer_dirty = False
        return {
            "occupied": self._latest_occupied,
            "preblocked": self._latest_preblocked,
            "traversable": self._latest_traversable,
        }

    def save_package(self, package_path: str, overwrite: bool, timeout_sec: float = 10.0):
        if not self.save_client.wait_for_service(timeout_sec=2.0):
            return False, "保存服务 /map_package_manager/save_package 不可用。"

        request = SaveNavigationMapPackage.Request()
        request.package_path = package_path
        request.overwrite = overwrite
        future = self.save_client.call_async(request)

        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        finally:
            executor.remove_node(self)
            executor.shutdown()

        if not future.done():
            return False, "保存地图超时。"
        result = future.result()
        if result is None:
            return False, "保存地图失败，服务没有返回结果。"
        return bool(result.success), str(result.message)


class SaveWorker(QObject):
    finished = pyqtSignal(bool, str)

    def __init__(self, package_path: str, overwrite: bool) -> None:
        super().__init__()
        self.package_path = package_path
        self.overwrite = overwrite

    def run(self) -> None:
        node = RosMapImportNode()
        try:
            ok, message = node.save_package(self.package_path, self.overwrite)
        finally:
            node.destroy_node()
        self.finished.emit(ok, message)


class RosMapImportWindow(QWidget):
    _LAYER_STYLE = {
        "occupied": ((0.95, 0.45, 0.15), 1.0),
        "preblocked": ((0.15, 0.35, 1.0), 1.0),
        "traversable": ((0.20, 0.95, 0.55), 0.30),
    }

    def __init__(self) -> None:
        super().__init__()
        self._default_root = Path("/home/robot/maps")
        self._selected_yaml: Path | None = None
        self._last_grid: OccupancyGrid | None = None
        self._worker_thread: threading.Thread | None = None
        self._ros_node = RosMapImportNode()
        self._renderer = vtk.vtkRenderer()
        self._layer_actors: dict[str, tuple[vtk.vtkActor, vtk.vtkActor]] = {}
        self._layer_data: dict[str, tuple[np.ndarray, np.ndarray, tuple[float, float, float], float]] = {}
        self._init_ui()
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._spin_ros_once)
        self._spin_timer.start(100)

    def _init_ui(self) -> None:
        self.setWindowTitle("二维地图导入")
        self.resize(1100, 760)

        root = QVBoxLayout()
        top_row = QHBoxLayout()

        import_group = QGroupBox("ROS 地图读取")
        import_form = QFormLayout()
        yaml_row = QHBoxLayout()
        self.yaml_edit = QLineEdit()
        self.yaml_edit.setPlaceholderText("选择 ROS 地图 yaml 文件")
        yaml_btn = QPushButton("选择 YAML")
        yaml_btn.clicked.connect(self._choose_yaml)
        yaml_row.addWidget(self.yaml_edit, 1)
        yaml_row.addWidget(yaml_btn)
        import_form.addRow("地图 YAML", yaml_row)

        self.wall_height_spin = QDoubleSpinBox()
        self.wall_height_spin.setDecimals(2)
        self.wall_height_spin.setRange(0.1, 10.0)
        self.wall_height_spin.setSingleStep(0.1)
        self.wall_height_spin.setValue(1.0)
        import_form.addRow("障碍高度(m)", self.wall_height_spin)

        self.occupied_threshold_spin = QSpinBox()
        self.occupied_threshold_spin.setRange(1, 100)
        self.occupied_threshold_spin.setValue(50)
        import_form.addRow("占据阈值", self.occupied_threshold_spin)
        import_group.setLayout(import_form)

        save_group = QGroupBox("Octomap地图保存")
        save_form = QFormLayout()
        root_row = QHBoxLayout()
        self.root_edit = QLineEdit(str(self._default_root))
        choose_root_btn = QPushButton("选择目录")
        choose_root_btn.clicked.connect(self._choose_root)
        root_row.addWidget(self.root_edit, 1)
        root_row.addWidget(choose_root_btn)
        save_form.addRow("根目录", root_row)

        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("请输入保存后的地图名")
        save_form.addRow("地图名", self.name_edit)

        self.overwrite_checkbox = QCheckBox("允许覆盖")
        self.overwrite_checkbox.setChecked(True)
        save_form.addRow("", self.overwrite_checkbox)

        save_btn = QPushButton("保存Octomap地图")
        save_btn.clicked.connect(self._save_package)
        save_form.addRow("", save_btn)
        save_group.setLayout(save_form)

        import_btn = QPushButton("导入二维地图")
        import_btn.clicked.connect(self._import_map)
        import_form.addRow("", import_btn)

        self.status_label = QLabel("等待操作。")
        self.status_label.setWordWrap(True)
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        left_panel = QVBoxLayout()
        left_panel.addWidget(import_group)
        left_panel.addWidget(save_group)
        left_panel.addWidget(self.status_label)
        left_panel_widget = QWidget()
        left_panel_widget.setLayout(left_panel)
        left_panel_widget.setFixedWidth(360)

        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.vtk_widget.GetRenderWindow().AddRenderer(self._renderer)
        self._renderer.SetBackground(0.04, 0.07, 0.09)
        self._renderer.GradientBackgroundOn()
        self._renderer.SetBackground2(0.12, 0.16, 0.19)
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.5, 1.5, 1.5)
        axes.SetXAxisLabelText("")
        axes.SetYAxisLabelText("")
        axes.SetZAxisLabelText("")
        self._renderer.AddActor(axes)
        self._renderer.AddActor(self._make_ground_grid(24.0, 1.0))
        interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        interactor.Initialize()

        top_row.addWidget(left_panel_widget, 0)
        top_row.addWidget(self.vtk_widget, 1)
        root.addLayout(top_row)
        self.setLayout(root)

    def closeEvent(self, event) -> None:
        self._ros_node.destroy_node()
        super().closeEvent(event)

    def _spin_ros_once(self) -> None:
        rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        layers = self._ros_node.consume_layers()
        if layers is None:
            return
        for layer_name, payload in layers.items():
            if payload is None:
                continue
            points, scale = payload
            color, opacity = self._LAYER_STYLE[layer_name]
            self._layer_data[layer_name] = (points, scale, color, opacity)
        self._refresh_layers()

    def _choose_yaml(self) -> None:
        selected, _ = QFileDialog.getOpenFileName(
            self,
            "选择 ROS 地图 YAML",
            str(Path.home()),
            "YAML Files (*.yaml *.yml)",
        )
        if selected:
            self._selected_yaml = Path(selected)
            self.yaml_edit.setText(selected)
            if not self.name_edit.text().strip():
                self.name_edit.setText(self._selected_yaml.stem)

    def _choose_root(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图保存根目录",
            self.root_edit.text().strip() or str(self._default_root),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if selected:
            self.root_edit.setText(selected)

    def _load_ros_map(self, yaml_path: Path) -> OccupancyGrid:
        with yaml_path.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        image_path = Path(meta["image"])
        if not image_path.is_absolute():
            image_path = (yaml_path.parent / image_path).resolve()

        image = Image.open(image_path).convert("L")
        image_array = np.array(image, dtype=np.uint8)

        negate = int(meta.get("negate", 0))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.196))

        if negate == 0:
            occupancy_prob = (255.0 - image_array.astype(np.float32)) / 255.0
        else:
            occupancy_prob = image_array.astype(np.float32) / 255.0

        data = np.full(image_array.shape, -1, dtype=np.int8)
        data[occupancy_prob >= occupied_thresh] = 100
        data[occupancy_prob <= free_thresh] = 0

        # OccupancyGrid expects row-major data starting from bottom-left.
        flipped = np.flipud(data)

        origin = meta.get("origin", [0.0, 0.0, 0.0])
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self._ros_node.get_clock().now().to_msg()
        msg.info.resolution = float(meta["resolution"])
        msg.info.width = int(flipped.shape[1])
        msg.info.height = int(flipped.shape[0])
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation = yaw_to_quaternion(float(origin[2]))
        msg.data = flipped.reshape(-1).astype(np.int8).tolist()
        return msg

    def _import_map(self) -> None:
        yaml_text = self.yaml_edit.text().strip()
        if not yaml_text:
            QMessageBox.warning(self, "二维地图导入", "请先选择 YAML 文件。")
            return

        yaml_path = Path(yaml_text)
        if not yaml_path.exists():
            QMessageBox.warning(self, "二维地图导入", f"YAML 文件不存在：{yaml_path}")
            return

        try:
            grid = self._load_ros_map(yaml_path)
        except Exception as exc:
            QMessageBox.critical(self, "二维地图导入", f"读取 ROS 地图失败：{exc}")
            return

        self._last_grid = grid
        self._ros_node.publish_grid(grid)
        self.status_label.setText(
            f"已发布二维地图：{yaml_path.name}。分辨率 {grid.info.resolution:.3f} m，尺寸 {grid.info.width}x{grid.info.height}。"
            " 后端正在转换为 3D OctoMap，请稍后保存地图包。"
        )

    def _build_package_path(self) -> Path | None:
        root_dir = self.root_edit.text().strip()
        map_name = self.name_edit.text().strip()
        if not root_dir:
            QMessageBox.warning(self, "地图目录", "请先选择保存根目录。")
            return None
        if not map_name:
            QMessageBox.warning(self, "地图目录", "请输入地图名。")
            return None
        return Path(root_dir).expanduser() / map_name

    def _save_package(self) -> None:
        if self._last_grid is None:
            QMessageBox.warning(self, "地图包保存", "请先导入二维地图。")
            return

        package_path = self._build_package_path()
        if package_path is None:
            return

        self.status_label.setText(f"正在保存地图包到 {package_path} ，请稍候。")
        worker = SaveWorker(str(package_path), self.overwrite_checkbox.isChecked())
        worker.finished.connect(self._on_save_finished)
        thread = threading.Thread(target=worker.run, daemon=True)
        self._worker_thread = thread
        self._worker = worker
        thread.start()

    def _on_save_finished(self, success: bool, message: str) -> None:
        self.status_label.setText(message)
        if success:
            QMessageBox.information(self, "地图包保存", "地图包保存成功。")
        else:
            QMessageBox.critical(self, "地图包保存", message)

    def _make_ground_grid(self, size: float, step: float) -> vtk.vtkActor:
        append = vtk.vtkAppendPolyData()
        half = int(size / step)
        for i in range(-half, half + 1):
            line_x = vtk.vtkLineSource()
            line_x.SetPoint1(-size, i * step, 0.0)
            line_x.SetPoint2(size, i * step, 0.0)
            append.AddInputConnection(line_x.GetOutputPort())
            line_y = vtk.vtkLineSource()
            line_y.SetPoint1(i * step, -size, 0.0)
            line_y.SetPoint2(i * step, size, 0.0)
            append.AddInputConnection(line_y.GetOutputPort())
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(append.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.24, 0.30, 0.34)
        actor.GetProperty().SetLineWidth(1.0)
        actor.GetProperty().SetOpacity(0.65)
        return actor

    def _build_voxel_actors(
        self, points: np.ndarray, scale: np.ndarray, color: tuple[float, float, float], opacity: float
    ) -> tuple[vtk.vtkActor, vtk.vtkActor]:
        vtk_points = vtk.vtkPoints()
        vtk_points.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)

        cube = vtk.vtkCubeSource()
        cube.SetXLength(float(scale[0]))
        cube.SetYLength(float(scale[1]))
        cube.SetZLength(float(scale[2]))
        glyph = vtk.vtkGlyph3DMapper()
        glyph.SetInputData(polydata)
        glyph.SetSourceConnection(cube.GetOutputPort())
        glyph.ScalingOff()

        actor = vtk.vtkActor()
        actor.SetMapper(glyph)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetOpacity(opacity)
        actor.GetProperty().SetInterpolationToFlat()

        edge_cube = vtk.vtkCubeSource()
        edge_cube.SetXLength(float(scale[0]))
        edge_cube.SetYLength(float(scale[1]))
        edge_cube.SetZLength(float(scale[2]))
        edge_extract = vtk.vtkExtractEdges()
        edge_extract.SetInputConnection(edge_cube.GetOutputPort())
        edge_glyph = vtk.vtkGlyph3DMapper()
        edge_glyph.SetInputData(polydata)
        edge_glyph.SetSourceConnection(edge_extract.GetOutputPort())
        edge_glyph.ScalingOff()

        edge_actor = vtk.vtkActor()
        edge_actor.SetMapper(edge_glyph)
        edge_actor.GetProperty().SetColor(0.0, 0.0, 0.0)
        edge_actor.GetProperty().SetLineWidth(1.0)
        edge_actor.GetProperty().SetOpacity(1.0)
        return actor, edge_actor

    def _refresh_layers(self) -> None:
        for actor, edge_actor in self._layer_actors.values():
            self._renderer.RemoveActor(actor)
            self._renderer.RemoveActor(edge_actor)
        self._layer_actors.clear()

        for layer_name in ("occupied", "preblocked", "traversable"):
            if layer_name not in self._layer_data:
                continue
            points, scale, color, opacity = self._layer_data[layer_name]
            if points.size == 0:
                continue
            actor, edge_actor = self._build_voxel_actors(points, scale, color, opacity)
            self._renderer.AddActor(actor)
            self._renderer.AddActor(edge_actor)
            self._layer_actors[layer_name] = (actor, edge_actor)

        self._renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()


def main() -> None:
    rclpy.init()
    app = QApplication([])
    window = RosMapImportWindow()
    window.show()
    app.exec_()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
