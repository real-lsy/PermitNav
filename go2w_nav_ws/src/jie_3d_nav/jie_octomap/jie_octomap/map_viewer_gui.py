#!/usr/bin/env python3

from __future__ import annotations

import os
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from jie_map_msgs.srv import LoadNavigationMapPackage, SaveNavigationMapPackage
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from PyQt5.QtCore import QEvent, QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
import vtk
from vtk.util import numpy_support
try:
    from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
except ModuleNotFoundError:
    from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


class SaveMapClient(Node):
    def __init__(self) -> None:
        super().__init__(f"map_viewer_save_client_{time.time_ns()}")
        self.client = self.create_client(
            SaveNavigationMapPackage, "/map_package_manager/save_package"
        )

    def save_package(self, package_path: str, overwrite: bool, timeout_sec: float = 10.0):
        if not self.client.wait_for_service(timeout_sec=2.0):
            return False, "保存服务 /map_package_manager/save_package 不可用。"

        request = SaveNavigationMapPackage.Request()
        request.package_path = package_path
        request.overwrite = overwrite
        future = self.client.call_async(request)
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


class LoadMapClient(Node):
    def __init__(self) -> None:
        super().__init__(f"map_viewer_load_client_{time.time_ns()}")
        self.client = self.create_client(
            LoadNavigationMapPackage, "/map_package_manager/load_package"
        )

    def load_package(self, package_path: str, timeout_sec: float = 10.0):
        if not self.client.wait_for_service(timeout_sec=2.0):
            return False, "读取服务 /map_package_manager/load_package 不可用。"

        request = LoadNavigationMapPackage.Request()
        request.package_path = package_path
        future = self.client.call_async(request)
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        finally:
            executor.remove_node(self)
            executor.shutdown()
        if not future.done():
            return False, "读取地图超时。"
        result = future.result()
        if result is None:
            return False, "读取地图失败，服务没有返回结果。"
        return bool(result.success), str(result.message)


class SaveWorker(QObject):
    finished = pyqtSignal(bool, str)

    def __init__(self, package_path: str, overwrite: bool) -> None:
        super().__init__()
        self.package_path = package_path
        self.overwrite = overwrite

    def run(self) -> None:
        node = SaveMapClient()
        try:
            ok, message = node.save_package(self.package_path, self.overwrite)
        finally:
            node.destroy_node()
        self.finished.emit(ok, message)


class LoadWorker(QObject):
    finished = pyqtSignal(bool, str, str)

    def __init__(self, package_path: str) -> None:
        super().__init__()
        self.package_path = package_path

    def run(self) -> None:
        node = LoadMapClient()
        try:
            ok, message = node.load_package(self.package_path)
        finally:
            node.destroy_node()
        self.finished.emit(ok, message, self.package_path)


class MapViewerRosNode(Node):
    def __init__(self) -> None:
        super().__init__("map_viewer_gui_node")
        self.declare_parameter("tf_parent_frame", "map")
        self.declare_parameter("tf_child_frame", "base_footprint")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.start_pub = self.create_publisher(PointStamped, "/start_point", latched_qos)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", latched_qos)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", latched_qos)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.external_preblocked_pub = self.create_publisher(
            Marker, "/edited_preblocked_cells_markers", latched_qos
        )
        self.edited_occupied_pub = self.create_publisher(
            Marker, "/edited_occupied_markers", latched_qos
        )
        self.occupied_pub = self.create_publisher(
            Marker, "/octomap_occupied_markers", latched_qos
        )
        self.preblocked_pub = self.create_publisher(
            Marker, "/preblocked_cells_markers", latched_qos
        )
        self.traversable_pub = self.create_publisher(
            Marker, "/traversable_cells_markers", latched_qos
        )
        self.risk_pub = self.create_publisher(PointCloud2, "/risk_cost_cells", latched_qos)
        self.path_sub = self.create_subscription(
            PathMsg, "/planned_path", self._on_path, latched_qos
        )
        self.occupied_sub = self.create_subscription(
            Marker, "/octomap_occupied_markers", self._on_occupied, latched_qos
        )
        self.preblocked_sub = self.create_subscription(
            Marker, "/preblocked_cells_markers", self._on_preblocked, latched_qos
        )
        self.traversable_sub = self.create_subscription(
            Marker, "/traversable_cells_markers", self._on_traversable, latched_qos
        )
        self.risk_sub = self.create_subscription(
            PointCloud2, "/risk_cost_cells", self._on_risk, latched_qos
        )
        self._latest_path_points: list[tuple[float, float, float]] = []
        self._path_dirty = False
        self._latest_occupied: tuple[np.ndarray, np.ndarray] | None = None
        self._occupied_dirty = False
        self._latest_preblocked: tuple[np.ndarray, np.ndarray] | None = None
        self._preblocked_dirty = False
        self._latest_traversable: tuple[np.ndarray, np.ndarray] | None = None
        self._traversable_dirty = False
        self._latest_risk: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        self._risk_dirty = False

    def publish_point(self, topic: str, frame_id: str, xyz: tuple[float, float, float]) -> None:
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        if topic == "start":
            self.start_pub.publish(msg)
        else:
            self.goal_pub.publish(msg)

    def publish_goal_pose(
        self, frame_id: str, xyz: tuple[float, float, float], yaw: float
    ) -> None:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        half_yaw = float(yaw) * 0.5
        msg.pose.orientation.z = float(np.sin(half_yaw))
        msg.pose.orientation.w = float(np.cos(half_yaw))
        self.goal_pose_pub.publish(msg)

    def publish_initial_pose(
        self, frame_id: str, xyz: tuple[float, float, float], yaw: float
    ) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(xyz[0])
        msg.pose.pose.position.y = float(xyz[1])
        msg.pose.pose.position.z = float(xyz[2])
        half_yaw = float(yaw) * 0.5
        msg.pose.pose.orientation.z = float(np.sin(half_yaw))
        msg.pose.pose.orientation.w = float(np.cos(half_yaw))
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891909122467
        self.initial_pose_pub.publish(msg)

    def publish_external_preblocked(
        self, frame_id: str, points: np.ndarray, scale: np.ndarray
    ) -> None:
        msg = Marker()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "external_preblocked_cells"
        msg.id = 0
        msg.type = Marker.CUBE_LIST
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = float(scale[0])
        msg.scale.y = float(scale[1])
        msg.scale.z = float(scale[2])
        msg.color.r = 0.95
        msg.color.g = 0.10
        msg.color.b = 0.10
        msg.color.a = 0.95
        for point in points:
            cell = Point()
            cell.x = float(point[0])
            cell.y = float(point[1])
            cell.z = float(point[2])
            msg.points.append(cell)
        self.external_preblocked_pub.publish(msg)

    def publish_edited_occupied(
        self, frame_id: str, points: np.ndarray, scale: np.ndarray
    ) -> None:
        msg = Marker()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "edited_occupied_cells"
        msg.id = 0
        msg.type = Marker.CUBE_LIST
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = float(scale[0])
        msg.scale.y = float(scale[1])
        msg.scale.z = float(scale[2])
        msg.color.r = 0.95
        msg.color.g = 0.45
        msg.color.b = 0.15
        msg.color.a = 1.0
        for point in points:
            cell = Point()
            cell.x = float(point[0])
            cell.y = float(point[1])
            cell.z = float(point[2])
            msg.points.append(cell)
        self.edited_occupied_pub.publish(msg)

    def publish_voxel_marker(
        self,
        layer_name: str,
        frame_id: str,
        points: np.ndarray,
        scale: np.ndarray,
        color: tuple[float, float, float],
        opacity: float,
    ) -> None:
        msg = Marker()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = f"{layer_name}_cells"
        msg.id = 0
        msg.type = Marker.CUBE_LIST
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = float(scale[0])
        msg.scale.y = float(scale[1])
        msg.scale.z = float(scale[2])
        msg.color.r = float(color[0])
        msg.color.g = float(color[1])
        msg.color.b = float(color[2])
        msg.color.a = float(opacity)
        for point in points:
            cell = Point()
            cell.x = float(point[0])
            cell.y = float(point[1])
            cell.z = float(point[2])
            msg.points.append(cell)

        if layer_name == "occupied":
            msg.ns = "occupied_voxels"
            self.occupied_pub.publish(msg)
        elif layer_name == "preblocked":
            msg.ns = "preblocked_cells"
            self.preblocked_pub.publish(msg)
        elif layer_name == "traversable":
            msg.ns = "traversable_cells"
            self.traversable_pub.publish(msg)

    def publish_risk_cloud(
        self,
        frame_id: str,
        points: np.ndarray,
        intensity: np.ndarray,
    ) -> None:
        ros_header = Header()
        ros_header.frame_id = frame_id
        ros_header.stamp = self.get_clock().now().to_msg()
        cloud = point_cloud2.create_cloud(
            ros_header,
            [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            [
                (float(p[0]), float(p[1]), float(p[2]), float(i))
                for p, i in zip(points, intensity)
            ],
        )
        self.risk_pub.publish(cloud)

    def _on_path(self, msg: PathMsg) -> None:
        self._latest_path_points = [
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in msg.poses
        ]
        self._path_dirty = True

    def _on_occupied(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return
        points = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
        scale = np.array([msg.scale.x, msg.scale.y, msg.scale.z], dtype=np.float32)
        self._latest_occupied = (points, scale)
        self._occupied_dirty = True

    def _on_preblocked(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return
        points = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
        scale = np.array([msg.scale.x, msg.scale.y, msg.scale.z], dtype=np.float32)
        self._latest_preblocked = (points, scale)
        self._preblocked_dirty = True

    def _on_traversable(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return
        points = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
        scale = np.array([msg.scale.x, msg.scale.y, msg.scale.z], dtype=np.float32)
        self._latest_traversable = (points, scale)
        self._traversable_dirty = True

    def _on_risk(self, msg: PointCloud2) -> None:
        records = list(
            point_cloud2.read_points(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
            )
        )
        if not records:
            xyz = np.empty((0, 3), dtype=np.float32)
            intensity = np.empty((0,), dtype=np.float32)
        else:
            points = np.array(
                [[row[0], row[1], row[2], row[3]] for row in records],
                dtype=np.float32,
            )
            xyz = points[:, :3]
            intensity = points[:, 3]
        scale = self._infer_voxel_scale()
        self._latest_risk = (xyz, scale, intensity)
        self._risk_dirty = True

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(np.arctan2(siny_cosp, cosy_cosp))

    def consume_path(self) -> list[tuple[float, float, float]] | None:
        if not self._path_dirty:
            return None
        self._path_dirty = False
        return list(self._latest_path_points)

    def consume_occupied(self) -> tuple[np.ndarray, np.ndarray] | None:
        if not self._occupied_dirty or self._latest_occupied is None:
            return None
        self._occupied_dirty = False
        return self._latest_occupied

    def consume_preblocked(self) -> tuple[np.ndarray, np.ndarray] | None:
        if not self._preblocked_dirty or self._latest_preblocked is None:
            return None
        self._preblocked_dirty = False
        return self._latest_preblocked

    def consume_traversable(self) -> tuple[np.ndarray, np.ndarray] | None:
        if not self._traversable_dirty or self._latest_traversable is None:
            return None
        self._traversable_dirty = False
        return self._latest_traversable

    def consume_risk(self) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        if not self._risk_dirty or self._latest_risk is None:
            return None
        self._risk_dirty = False
        return self._latest_risk

    def consume_robot_pose(self) -> tuple[tuple[float, float, float], float] | None:
        parent = str(self.get_parameter("tf_parent_frame").value)
        child = str(self.get_parameter("tf_child_frame").value)
        try:
            transform = self._tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return (
            (float(translation.x), float(translation.y), float(translation.z)),
            yaw,
        )

    def _infer_voxel_scale(self) -> np.ndarray:
        if self._latest_occupied is not None:
            return np.asarray(self._latest_occupied[1], dtype=np.float32)
        return np.array([0.2, 0.2, 0.2], dtype=np.float32)


class MapViewerWindow(QWidget):
    _ROBOT_DISPLAY_Z_OFFSET = -0.3
    _LAYER_STYLE = {
        "occupied": ((0.95, 0.45, 0.15), 1.0, "占据"),
        "preblocked": ((0.15, 0.35, 1.0), 1.0, "禁行"),
        "traversable": ((0.20, 0.95, 0.55), 0.30, "可通行"),
        "risk": ((0.15, 0.35, 1.0), 0.55, "风险代价"),
    }

    def __init__(self) -> None:
        super().__init__()
        default_map_package = Path(
            os.environ.get("MAP_VIEWER_DEFAULT_PACKAGE", "/home/robot/maps/map")
        ).expanduser()
        self._default_root = default_map_package.parent
        self._default_map_name = default_map_package.name
        self._suppress_next_load_dialog = False
        self._worker_thread: threading.Thread | None = None
        self._layer_actors: dict[str, tuple[vtk.vtkActor, vtk.vtkActor | None]] = {}
        self._layer_data: dict[str, tuple[np.ndarray, np.ndarray, tuple[float, float, float], float] | tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self._renderer = vtk.vtkRenderer()
        self._ros_node = MapViewerRosNode()
        self._frame_id = "map"
        self._pick_mode: str | None = None
        self._start_actor: vtk.vtkActor | None = None
        self._goal_actor: vtk.vtkActor | None = None
        self._goal_arrow_actor: vtk.vtkActor | None = None
        self._goal_pending_position: tuple[float, float, float] | None = None
        self._goal_yaw: float = 0.0
        self._current_pose_arrow_actor: vtk.vtkActor | None = None
        self._path_actor: vtk.vtkActor | None = None
        self._robot_actor: vtk.vtkProp3D | None = None
        self._latest_robot_pose: tuple[tuple[float, float, float], float] | None = None
        self._edit_cursor_actor: vtk.vtkActor | None = None
        self._edit_cursor_edge_actor: vtk.vtkActor | None = None
        self._edit_position = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self._edit_size_cells = 1
        self._init_ui()
        QApplication.instance().installEventFilter(self)
        self._spin_timer = QTimer(self)
        self._spin_timer.setTimerType(Qt.PreciseTimer)
        self._spin_timer.timeout.connect(self._spin_ros_once)
        self._spin_timer.start(20)
        QTimer.singleShot(0, self._autoload_default_map)

    def _init_ui(self) -> None:
        self.setWindowTitle("地图查看")
        self.resize(1220, 820)

        layout = QVBoxLayout()
        control_row = QHBoxLayout()

        map_group = QGroupBox("地图处理")
        map_layout = QVBoxLayout()
        map_root_row = QHBoxLayout()
        self.path_edit = QLineEdit(str(self._default_root))
        self.path_edit.setPlaceholderText("选择地图根目录")
        choose_root_btn = QPushButton("选择文件夹")
        choose_root_btn.clicked.connect(self._choose_root_directory)
        map_root_row.addWidget(QLabel("根目录"))
        map_root_row.addWidget(self.path_edit, 1)
        map_root_row.addWidget(choose_root_btn)

        map_name_row = QHBoxLayout()
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("请输入地图名，例如 lv2")
        self.name_edit.setText(self._default_map_name)
        self.overwrite_checkbox = QCheckBox("允许覆盖")
        self.overwrite_checkbox.setChecked(True)
        map_name_row.addWidget(QLabel("地图名"))
        map_name_row.addWidget(self.name_edit, 1)
        map_name_row.addWidget(self.overwrite_checkbox)

        map_button_row = QHBoxLayout()
        open_btn = QPushButton("打开地图")
        open_btn.clicked.connect(self._choose_and_open)
        refresh_map_btn = QPushButton("刷新地图")
        refresh_map_btn.clicked.connect(self._refresh_map_from_edited_occupied)
        save_btn = QPushButton("保存地图")
        save_btn.clicked.connect(self._start_save)
        map_button_row.addWidget(open_btn)
        map_button_row.addWidget(refresh_map_btn)
        map_button_row.addWidget(save_btn)

        map_layout.addLayout(map_root_row)
        map_layout.addLayout(map_name_row)
        map_layout.addLayout(map_button_row)
        map_group.setLayout(map_layout)

        planning_group = QGroupBox("路径规划")
        planning_layout = QHBoxLayout()
        self.start_btn = QPushButton("起始点")
        self.start_btn.clicked.connect(lambda: self._set_pick_mode("start"))
        self.goal_btn = QPushButton("目标点")
        self.goal_btn.clicked.connect(lambda: self._set_pick_mode("goal"))
        planning_layout.addWidget(self.start_btn)
        planning_layout.addWidget(self.goal_btn)
        planning_layout.addStretch(1)
        planning_group.setLayout(planning_layout)

        navigation_group = QGroupBox("导航")
        navigation_layout = QHBoxLayout()
        self.current_pose_btn = QPushButton("当前姿态")
        self.current_pose_btn.clicked.connect(lambda: self._set_pick_mode("current_pose"))
        self.navigate_btn = QPushButton("导航目标")
        self.navigate_btn.clicked.connect(lambda: self._set_pick_mode("navigate"))
        navigation_layout.addWidget(self.current_pose_btn)
        navigation_layout.addWidget(self.navigate_btn)
        navigation_layout.addStretch(1)
        navigation_group.setLayout(navigation_layout)

        display_group = QGroupBox("地图显示选项")
        layer_row = QHBoxLayout()
        self.occupied_checkbox = QCheckBox("占据")
        self.occupied_checkbox.setChecked(True)
        self.occupied_checkbox.toggled.connect(self._refresh_layers)
        self.preblocked_checkbox = QCheckBox("禁行")
        self.preblocked_checkbox.setChecked(False)
        self.preblocked_checkbox.toggled.connect(self._refresh_layers)
        self.traversable_checkbox = QCheckBox("可通行")
        self.traversable_checkbox.setChecked(False)
        self.traversable_checkbox.toggled.connect(self._refresh_layers)
        self.risk_checkbox = QCheckBox("风险代价")
        self.risk_checkbox.setChecked(False)
        self.risk_checkbox.toggled.connect(self._refresh_layers)
        layer_row.addWidget(self.occupied_checkbox)
        layer_row.addWidget(self.preblocked_checkbox)
        layer_row.addWidget(self.traversable_checkbox)
        layer_row.addWidget(self.risk_checkbox)
        layer_row.addStretch(1)
        display_group.setLayout(layer_row)

        edit_group = QGroupBox("栅格编辑")
        edit_layout = QVBoxLayout()
        edit_top_row = QHBoxLayout()
        self.edit_checkbox = QCheckBox("编辑栅格")
        self.edit_checkbox.toggled.connect(self._toggle_edit_mode)
        self.enlarge_btn = QPushButton("扩大")
        self.enlarge_btn.clicked.connect(self._increase_edit_size)
        self.shrink_btn = QPushButton("缩小")
        self.shrink_btn.clicked.connect(self._decrease_edit_size)
        edit_top_row.addWidget(self.edit_checkbox)
        edit_top_row.addWidget(self.enlarge_btn)
        edit_top_row.addWidget(self.shrink_btn)
        edit_top_row.addStretch(1)

        edit_radio_row = QHBoxLayout()
        self.edit_type_group = QButtonGroup(self)
        self.edit_type_buttons: dict[str, QRadioButton] = {}
        for layer_name in ("occupied", "preblocked", "traversable", "clear"):
            label = self._LAYER_STYLE[layer_name][2] if layer_name in self._LAYER_STYLE else "清空"
            radio = QRadioButton(label)
            if layer_name == "occupied":
                radio.setChecked(True)
            radio.toggled.connect(self._refocus_view_if_editing)
            self.edit_type_group.addButton(radio)
            self.edit_type_buttons[layer_name] = radio
            edit_radio_row.addWidget(radio)
        edit_radio_row.addStretch(1)
        edit_layout.addLayout(edit_top_row)
        edit_layout.addLayout(edit_radio_row)
        edit_group.setLayout(edit_layout)

        navigation_column = QVBoxLayout()
        navigation_column.addWidget(planning_group)
        navigation_column.addWidget(navigation_group)
        navigation_column.addStretch(1)

        control_row.addWidget(map_group, 2)
        control_row.addLayout(navigation_column, 1)
        control_row.addWidget(display_group, 2)
        control_row.addWidget(edit_group, 2)

        self.info_label = QLabel("尚未加载地图。")
        self.info_label.setWordWrap(True)
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

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

        grid = self._make_ground_grid(size=24, step=1.0)
        self._renderer.AddActor(grid)

        interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        interactor.Initialize()
        interactor.AddObserver("LeftButtonPressEvent", self._on_left_button_press, 1.0)
        interactor.AddObserver("MouseMoveEvent", self._on_mouse_move, 1.0)
        self.setFocusPolicy(Qt.StrongFocus)
        self.vtk_widget.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self)
        self.vtk_widget.installEventFilter(self)
        layout.addLayout(control_row)
        layout.addWidget(self.info_label)
        layout.addWidget(self.vtk_widget, 1)
        self.setLayout(layout)

    def _choose_root_directory(self) -> None:
        start_dir = self.path_edit.text().strip() or str(self._default_root)
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图根目录",
            start_dir,
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if selected:
            self.path_edit.setText(selected)

    def _autoload_default_map(self) -> None:
        package_path = self._default_root / self._default_map_name
        if not package_path.exists():
            self.info_label.setText(f"默认地图不存在：{package_path}")
            return
        self.path_edit.setText(str(self._default_root))
        self.name_edit.setText(self._default_map_name)
        self._suppress_next_load_dialog = True
        self._start_load_for_package(package_path)

    def _build_package_path(self) -> Path | None:
        root_dir = self.path_edit.text().strip()
        map_name = self.name_edit.text().strip()
        if not root_dir:
            QMessageBox.warning(self, "地图目录", "请先选择地图根目录。")
            return None
        if not map_name:
            QMessageBox.warning(self, "地图目录", "请输入地图名。")
            return None
        return Path(root_dir).expanduser() / map_name

    def _set_busy(self, busy: bool) -> None:
        self.path_edit.setEnabled(not busy)
        self.name_edit.setEnabled(not busy)
        self.overwrite_checkbox.setEnabled(not busy)
        self.start_btn.setEnabled(not busy)
        self.goal_btn.setEnabled(not busy)
        self.current_pose_btn.setEnabled(not busy)
        self.navigate_btn.setEnabled(not busy)
        self.edit_checkbox.setEnabled(not busy)
        self.enlarge_btn.setEnabled(not busy)
        self.shrink_btn.setEnabled(not busy)
        for radio in self.edit_type_buttons.values():
            radio.setEnabled(not busy)

    def _start_save(self) -> None:
        package_path = self._build_package_path()
        if package_path is None:
            return
        if not self._publish_edited_occupied_for_cpp_refresh():
            return
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(self._ros_node, timeout_sec=0.0)
            QApplication.processEvents()
            time.sleep(0.02)
        self._set_busy(True)
        self.info_label.setText(f"正在保存地图到 {package_path} ，请稍候。")
        worker = SaveWorker(str(package_path), self.overwrite_checkbox.isChecked())
        worker.finished.connect(self._on_save_finished)
        thread = threading.Thread(target=worker.run, daemon=True)
        self._worker_thread = thread
        self._worker = worker
        thread.start()

    def _start_load(self) -> None:
        package_path = self._build_package_path()
        if package_path is None:
            return
        if not package_path.exists():
            QMessageBox.warning(self, "加载地图", f"地图目录不存在：{package_path}")
            return
        self._start_load_for_package(package_path)

    def _start_load_for_package(self, package_path: Path) -> None:
        self._set_busy(True)
        self.info_label.setText(f"正在加载地图 {package_path} ，请稍候。")
        worker = LoadWorker(str(package_path))
        worker.finished.connect(self._on_load_finished)
        thread = threading.Thread(target=worker.run, daemon=True)
        self._worker_thread = thread
        self._worker = worker
        thread.start()

    def _on_save_finished(self, success: bool, message: str) -> None:
        self._set_busy(False)
        self.info_label.setText(message)
        if success:
            QMessageBox.information(self, "保存地图", "地图保存成功。")
        else:
            QMessageBox.critical(self, "保存地图", message)

    def _on_load_finished(self, success: bool, message: str, package_path: str) -> None:
        self._set_busy(False)
        suppress_dialog = self._suppress_next_load_dialog
        self._suppress_next_load_dialog = False
        if success:
            self._load_map_package(Path(package_path))
            if not suppress_dialog:
                QMessageBox.information(self, "加载地图", "地图加载成功。")
        else:
            self.info_label.setText(message)
            if not suppress_dialog:
                QMessageBox.critical(self, "加载地图", message)

    def _spin_ros_once(self) -> None:
        rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        occupied = self._ros_node.consume_occupied()
        if occupied is not None:
            points, scale = occupied
            color, opacity, _label = self._LAYER_STYLE["occupied"]
            self._layer_data["occupied"] = (points, scale, color, opacity)
            self._refresh_layers()
        preblocked = self._ros_node.consume_preblocked()
        if preblocked is not None:
            points, scale = preblocked
            color, opacity, _label = self._LAYER_STYLE["preblocked"]
            self._layer_data["preblocked"] = (points, scale, color, opacity)
            self._refresh_layers()
        traversable = self._ros_node.consume_traversable()
        if traversable is not None:
            points, scale = traversable
            color, opacity, _label = self._LAYER_STYLE["traversable"]
            self._layer_data["traversable"] = (points, scale, color, opacity)
            self._refresh_layers()
        risk = self._ros_node.consume_risk()
        if risk is not None:
            self._layer_data["risk"] = risk
            self._refresh_layers()
        robot_pose = self._ros_node.consume_robot_pose()
        if robot_pose is not None:
            self._latest_robot_pose = robot_pose
            self._update_robot_visual(*robot_pose)
        path_points = self._ros_node.consume_path()
        if path_points is not None:
            self._update_path(path_points)

    def _set_pick_mode(self, mode: str) -> None:
        if mode == "navigate" and self._latest_robot_pose is None:
            self.info_label.setText("未收到机器人 TF，无法设置导航目标。")
            return
        self._pick_mode = mode
        self._goal_pending_position = None
        if self.edit_checkbox.isChecked():
            self.edit_checkbox.setChecked(False)
        if mode == "start":
            self.info_label.setText("点击 3D 视图设置起始点。")
        elif mode == "current_pose":
            self.info_label.setText("点击 3D 视图设置当前姿态位置，再点击一次设置朝向。")
        elif mode == "navigate":
            self.info_label.setText("点击 3D 视图设置导航目标位置，再点击一次设置目标朝向。")
        else:
            self.info_label.setText("点击 3D 视图设置目标点位置，再点击一次设置目标朝向。")

    def _on_left_button_press(self, obj, event) -> None:
        if self._pick_mode is None:
            return

        actor_list = [actor for actor, _edge_actor in self._layer_actors.values()]
        if not actor_list:
            return

        click_x, click_y = obj.GetEventPosition()
        picker = vtk.vtkPropPicker()
        picker.PickFromListOn()
        for actor in actor_list:
            picker.AddPickList(actor)
        if picker.Pick(click_x, click_y, 0, self._renderer) == 0:
            self.info_label.setText("没有选中栅格。")
            return

        self.vtk_widget.setFocus()
        pos = picker.GetPickPosition()
        picked_xyz = self._snap_pick_to_navigation_cell(
            (float(pos[0]), float(pos[1]), float(pos[2]))
        )
        mode = self._pick_mode
        if mode == "start":
            self._pick_mode = None
            self._ros_node.publish_point(mode, self._frame_id, picked_xyz)
            self._update_point_actor("start", picked_xyz)
            self.info_label.setText(
                f"起始点已设置：[{picked_xyz[0]:.2f}, {picked_xyz[1]:.2f}, {picked_xyz[2]:.2f}]"
            )
            return

        if mode == "goal":
            self._goal_pending_position = picked_xyz
            self._pick_mode = "goal_heading"
            self._update_goal_visual(picked_xyz, self._goal_yaw)
            self.info_label.setText(
                "目标点位置已设置。移动鼠标预览朝向，再点击一次确认目标姿态。"
            )
            return

        if mode == "current_pose":
            self._goal_pending_position = picked_xyz
            self._pick_mode = "current_pose_heading"
            self._update_current_pose_visual(picked_xyz, self._goal_yaw)
            self.info_label.setText(
                "当前姿态位置已设置。移动鼠标预览朝向，再点击一次确认当前姿态。"
            )
            return

        if mode == "navigate":
            self._goal_pending_position = picked_xyz
            self._pick_mode = "navigate_heading"
            self._update_goal_visual(picked_xyz, self._goal_yaw)
            self.info_label.setText(
                "导航目标位置已设置。移动鼠标预览朝向，再点击一次确认导航目标。"
            )
            return

        if mode == "goal_heading" and self._goal_pending_position is not None:
            yaw = self._compute_goal_yaw(self._goal_pending_position, picked_xyz)
            self._goal_yaw = yaw
            self._pick_mode = None
            goal_xyz = self._goal_pending_position
            self._goal_pending_position = None
            self._ros_node.publish_point("goal", self._frame_id, goal_xyz)
            self._ros_node.publish_goal_pose(self._frame_id, goal_xyz, yaw)
            self._update_goal_visual(goal_xyz, yaw)
            self.info_label.setText(
                f"目标点已设置：[{goal_xyz[0]:.2f}, {goal_xyz[1]:.2f}, {goal_xyz[2]:.2f}]，"
                f"朝向 {np.degrees(yaw):.1f}°，正在规划路径。"
            )
            return

        if mode == "current_pose_heading" and self._goal_pending_position is not None:
            yaw = self._compute_goal_yaw(self._goal_pending_position, picked_xyz)
            self._goal_yaw = yaw
            self._pick_mode = None
            pose_xyz = self._goal_pending_position
            self._goal_pending_position = None
            self._ros_node.publish_initial_pose(self._frame_id, pose_xyz, yaw)
            self._clear_current_pose_visual()
            self._update_robot_visual(pose_xyz, yaw)
            self.info_label.setText(
                f"当前姿态已设置：[{pose_xyz[0]:.2f}, {pose_xyz[1]:.2f}, {pose_xyz[2]:.2f}]，"
                f"朝向 {np.degrees(yaw):.1f}°，已发送给 lidar_loc。"
            )
            return

        if mode == "navigate_heading" and self._goal_pending_position is not None:
            if self._latest_robot_pose is None:
                self._pick_mode = None
                self._goal_pending_position = None
                self.info_label.setText("未收到机器人 TF，导航起点不可用。")
                return
            yaw = self._compute_goal_yaw(self._goal_pending_position, picked_xyz)
            self._goal_yaw = yaw
            self._pick_mode = None
            goal_xyz = self._goal_pending_position
            self._goal_pending_position = None
            start_xyz, _robot_yaw = self._latest_robot_pose
            self._ros_node.publish_point("start", self._frame_id, start_xyz)
            self._ros_node.publish_point("goal", self._frame_id, goal_xyz)
            self._ros_node.publish_goal_pose(self._frame_id, goal_xyz, yaw)
            self._update_point_actor("start", start_xyz)
            self._update_goal_visual(goal_xyz, yaw)
            self.info_label.setText(
                f"导航目标已设置：[{goal_xyz[0]:.2f}, {goal_xyz[1]:.2f}, {goal_xyz[2]:.2f}]，"
                f"朝向 {np.degrees(yaw):.1f}°，正在从机器人当前位置规划。"
            )

    def _on_mouse_move(self, obj, event) -> None:
        if self._pick_mode not in ("goal_heading", "navigate_heading", "current_pose_heading") or self._goal_pending_position is None:
            return

        move_x, move_y = obj.GetEventPosition()
        cursor_xyz = self._pick_on_height_plane(
            move_x, move_y, float(self._goal_pending_position[2])
        )
        if cursor_xyz is None:
            return
        yaw = self._compute_goal_yaw(self._goal_pending_position, cursor_xyz)
        self._goal_yaw = yaw
        if self._pick_mode == "current_pose_heading":
            self._update_current_pose_visual(self._goal_pending_position, yaw)
        else:
            self._update_goal_visual(self._goal_pending_position, yaw)

    def _pick_on_height_plane(
        self, display_x: int, display_y: int, plane_z: float
    ) -> tuple[float, float, float] | None:
        self._renderer.SetDisplayPoint(float(display_x), float(display_y), 0.0)
        self._renderer.DisplayToWorld()
        near_world = self._renderer.GetWorldPoint()
        self._renderer.SetDisplayPoint(float(display_x), float(display_y), 1.0)
        self._renderer.DisplayToWorld()
        far_world = self._renderer.GetWorldPoint()

        if abs(near_world[3]) < 1.0e-9 or abs(far_world[3]) < 1.0e-9:
            return None

        p0 = np.array(
            [
                near_world[0] / near_world[3],
                near_world[1] / near_world[3],
                near_world[2] / near_world[3],
            ],
            dtype=np.float64,
        )
        p1 = np.array(
            [
                far_world[0] / far_world[3],
                far_world[1] / far_world[3],
                far_world[2] / far_world[3],
            ],
            dtype=np.float64,
        )
        direction = p1 - p0
        if abs(direction[2]) < 1.0e-9:
            return None
        t = (float(plane_z) - p0[2]) / direction[2]
        intersection = p0 + direction * t
        return (
            float(intersection[0]),
            float(intersection[1]),
            float(plane_z),
        )

    def _compute_goal_yaw(
        self,
        goal_xyz: tuple[float, float, float],
        facing_xyz: tuple[float, float, float],
    ) -> float:
        dx = float(facing_xyz[0] - goal_xyz[0])
        dy = float(facing_xyz[1] - goal_xyz[1])
        if abs(dx) < 1.0e-6 and abs(dy) < 1.0e-6:
            return self._goal_yaw
        return float(np.arctan2(dy, dx))

    def _snap_pick_to_navigation_cell(
        self, xyz: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        candidate_layers = ("traversable",)
        for layer_name in candidate_layers:
            layer = self._layer_data.get(layer_name)
            if layer is None:
                continue
            points = layer[0]
            if points.size == 0:
                continue
            diffs = points - np.asarray(xyz, dtype=np.float32)
            nearest_index = int(np.argmin(np.einsum("ij,ij->i", diffs, diffs)))
            snapped = points[nearest_index]
            return (float(snapped[0]), float(snapped[1]), float(snapped[2]))
        return xyz

    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and self.edit_checkbox.isChecked():
            if self._handle_edit_key(event):
                return True
        return super().eventFilter(obj, event)

    def _handle_edit_key(self, event) -> bool:
        key = event.key()
        move_map = {
            Qt.Key_W: np.array([1.0, 0.0, 0.0], dtype=np.float32),
            Qt.Key_S: np.array([-1.0, 0.0, 0.0], dtype=np.float32),
            Qt.Key_A: np.array([0.0, 1.0, 0.0], dtype=np.float32),
            Qt.Key_D: np.array([0.0, -1.0, 0.0], dtype=np.float32),
            Qt.Key_Q: np.array([0.0, 0.0, -1.0], dtype=np.float32),
            Qt.Key_E: np.array([0.0, 0.0, 1.0], dtype=np.float32),
        }
        if key in move_map:
            step = self._get_edit_scale()
            self._edit_position = self._edit_position + move_map[key] * step
            self._update_edit_cursor()
            self.info_label.setText(
                f"编辑栅格位置：[{self._edit_position[0]:.2f}, {self._edit_position[1]:.2f}, {self._edit_position[2]:.2f}]"
            )
            return True
        if key == Qt.Key_Space:
            self._place_edit_voxel()
            return True
        return False

    def _refocus_view_if_editing(self, checked: bool = False) -> None:
        del checked
        if self.edit_checkbox.isChecked():
            self.activateWindow()
            self.vtk_widget.setFocus()

    def _increase_edit_size(self) -> None:
        self._edit_size_cells += 1
        if self.edit_checkbox.isChecked():
            self._update_edit_cursor()
            self._refocus_view_if_editing()
        self.info_label.setText(f"编辑栅格尺寸：{self._edit_size_cells}x{self._edit_size_cells}x{self._edit_size_cells}")

    def _decrease_edit_size(self) -> None:
        self._edit_size_cells = max(1, self._edit_size_cells - 1)
        if self.edit_checkbox.isChecked():
            self._update_edit_cursor()
            self._refocus_view_if_editing()
        self.info_label.setText(f"编辑栅格尺寸：{self._edit_size_cells}x{self._edit_size_cells}x{self._edit_size_cells}")

    def _toggle_edit_mode(self, checked: bool) -> None:
        if checked:
            self._pick_mode = None
            self._initialize_edit_position()
            self._update_edit_cursor()
            self._refocus_view_if_editing()
            self.info_label.setText(
                f"编辑栅格已开启。当前尺寸 {self._edit_size_cells}x{self._edit_size_cells}x{self._edit_size_cells}。"
                " 使用 W/S 移动 X，A/D 移动 Y，Q/E 移动 Z，空格生成栅格。"
            )
        else:
            self._remove_edit_cursor()
            self.vtk_widget.GetRenderWindow().Render()

    def _initialize_edit_position(self) -> None:
        occupied_layer = self._layer_data.get("occupied")
        if occupied_layer is not None and occupied_layer[0].size > 0:
            occupied_points = np.asarray(occupied_layer[0], dtype=np.float32)
            scale = np.asarray(occupied_layer[1], dtype=np.float32)
            self._edit_position = np.array(
                [
                    float((np.min(occupied_points[:, 0]) + np.max(occupied_points[:, 0])) * 0.5),
                    float((np.min(occupied_points[:, 1]) + np.max(occupied_points[:, 1])) * 0.5),
                    float(np.max(occupied_points[:, 2]) + 2.0 * scale[2]),
                ],
                dtype=np.float32,
            )
            return
        for layer_name in ("traversable", "preblocked"):
            layer = self._layer_data.get(layer_name)
            if layer is not None and layer[0].size > 0:
                self._edit_position = layer[0][0].astype(np.float32).copy()
                return
        self._edit_position = np.array([0.0, 0.0, 0.0], dtype=np.float32)

    def _get_edit_scale(self) -> np.ndarray:
        for layer_name in ("occupied", "traversable", "preblocked"):
            layer = self._layer_data.get(layer_name)
            if layer is not None:
                return np.asarray(layer[1], dtype=np.float32)
        return np.array([0.2, 0.2, 0.2], dtype=np.float32)

    def _remove_edit_cursor(self) -> None:
        if self._edit_cursor_actor is not None:
            self._renderer.RemoveActor(self._edit_cursor_actor)
            self._edit_cursor_actor = None
        if self._edit_cursor_edge_actor is not None:
            self._renderer.RemoveActor(self._edit_cursor_edge_actor)
            self._edit_cursor_edge_actor = None

    def _update_edit_cursor(self) -> None:
        self._remove_edit_cursor()
        scale = self._get_edit_scale() * float(self._edit_size_cells)
        actor, edge_actor = self._build_voxel_actors(
            np.asarray([self._edit_position], dtype=np.float32),
            scale,
            (1.0, 0.15, 0.15),
            0.45,
        )
        edge_actor.GetProperty().SetColor(0.55, 0.0, 0.0)
        edge_actor.GetProperty().SetLineWidth(2.0)
        self._renderer.AddActor(actor)
        self._renderer.AddActor(edge_actor)
        self._edit_cursor_actor = actor
        self._edit_cursor_edge_actor = edge_actor
        self.vtk_widget.GetRenderWindow().Render()

    def _selected_edit_layer(self) -> str:
        for layer_name, radio in self.edit_type_buttons.items():
            if radio.isChecked():
                return layer_name
        return "occupied"

    def _place_edit_voxel(self) -> None:
        layer_name = self._selected_edit_layer()
        if layer_name == "clear":
            self._clear_edit_voxel()
            return
        scale = self._get_edit_scale()
        color, opacity, label = self._LAYER_STYLE[layer_name]
        points = self._edit_block_points(scale)

        if layer_name in self._layer_data:
            current_points, current_scale, _current_color, _current_opacity = self._layer_data[layer_name]
            merged_points = current_points
            added = 0
            for point in points:
                if merged_points.size > 0:
                    deltas = merged_points - point
                    duplicate = np.any(np.einsum("ij,ij->i", deltas, deltas) < 1.0e-8)
                    if duplicate:
                        continue
                merged_points = np.vstack([merged_points, point]).astype(np.float32)
                added += 1
            if added == 0:
                self.info_label.setText(f"{label}栅格已存在于当前位置范围内。")
                return
            points = merged_points.astype(np.float32)
            scale = np.asarray(current_scale, dtype=np.float32)
        else:
            points = points.astype(np.float32)

        self._layer_data[layer_name] = (points, np.asarray(scale, dtype=np.float32), color, opacity)
        checkbox_map = {
            "occupied": self.occupied_checkbox,
            "preblocked": self.preblocked_checkbox,
            "traversable": self.traversable_checkbox,
        }
        checkbox_map[layer_name].setChecked(True)
        self._refresh_layers()
        if self.edit_checkbox.isChecked():
            self._update_edit_cursor()
        if layer_name == "preblocked":
            self._sync_external_preblocked()
        self.info_label.setText(
            f"已生成{label}栅格块：中心[{self._edit_position[0]:.2f}, {self._edit_position[1]:.2f}, {self._edit_position[2]:.2f}] "
            f"尺寸{self._edit_size_cells}x{self._edit_size_cells}x{self._edit_size_cells}"
        )

    def _clear_edit_voxel(self) -> None:
        scale = self._get_edit_scale()
        half_extent = np.asarray(scale, dtype=np.float32) * float(self._edit_size_cells) * 0.5
        min_corner = self._edit_position - half_extent
        max_corner = self._edit_position + half_extent
        epsilon = np.maximum(np.asarray(scale, dtype=np.float32) * 1.0e-3, 1.0e-6)
        changed_layers: list[str] = []
        for layer_name, layer in list(self._layer_data.items()):
            if layer_name == "risk":
                current_points, current_scale, intensity = layer
            else:
                current_points, current_scale, color, opacity = layer
            if current_points.size == 0:
                continue
            points = np.asarray(current_points, dtype=np.float32)
            inside_mask = np.all(
                (points >= (min_corner - epsilon)) & (points <= (max_corner + epsilon)),
                axis=1,
            )
            removed_any = bool(np.any(inside_mask))
            if not removed_any:
                continue
            keep_mask = ~inside_mask
            new_points = current_points[keep_mask]
            if layer_name == "risk":
                self._layer_data[layer_name] = (
                    new_points.astype(np.float32),
                    np.asarray(current_scale, dtype=np.float32),
                    np.asarray(intensity, dtype=np.float32)[keep_mask],
                )
            else:
                self._layer_data[layer_name] = (
                    new_points.astype(np.float32),
                    np.asarray(current_scale, dtype=np.float32),
                    color,
                    opacity,
                )
            changed_layers.append(layer_name)

        if not changed_layers:
            self.info_label.setText("当前位置没有可清除的栅格。")
            return

        self._refresh_layers()
        if self.edit_checkbox.isChecked():
            self._update_edit_cursor()
        self._force_render()
        if "preblocked" in changed_layers:
            self._sync_external_preblocked()
        self.info_label.setText(
            f"已清空栅格块：中心[{self._edit_position[0]:.2f}, {self._edit_position[1]:.2f}, {self._edit_position[2]:.2f}] "
            f"尺寸{self._edit_size_cells}x{self._edit_size_cells}x{self._edit_size_cells}"
        )

    def _force_render(self) -> None:
        self.vtk_widget.GetRenderWindow().Render()
        interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        if interactor is not None:
            interactor.Render()
        QApplication.processEvents()

    def _edit_block_points(self, scale: np.ndarray) -> np.ndarray:
        half = (self._edit_size_cells - 1) / 2.0
        offsets = np.arange(self._edit_size_cells, dtype=np.float32) - half
        points: list[np.ndarray] = []
        for ox in offsets:
            for oy in offsets:
                for oz in offsets:
                    point = self._edit_position + np.array(
                        [ox * scale[0], oy * scale[1], oz * scale[2]], dtype=np.float32
                    )
                    points.append(point)
        return np.asarray(points, dtype=np.float32)

    def _sync_external_preblocked(self) -> None:
        layer = self._layer_data.get("preblocked")
        if layer is None:
            self._ros_node.publish_external_preblocked(
                self._frame_id,
                np.empty((0, 3), dtype=np.float32),
                self._get_edit_scale(),
            )
            return
        points, scale, _color, _opacity = layer
        self._ros_node.publish_external_preblocked(self._frame_id, points, scale)

    def _refresh_map_from_edited_occupied(self) -> None:
        if not self._publish_edited_occupied_for_cpp_refresh():
            return
        self._refresh_layers()
        if self.edit_checkbox.isChecked():
            self._update_edit_cursor()
        self._force_render()

    def _publish_edited_occupied_for_cpp_refresh(self) -> bool:
        occupied_layer = self._layer_data.get("occupied")
        if occupied_layer is None or occupied_layer[0].size == 0:
            self.info_label.setText("没有占据栅格，无法刷新地图。")
            return False

        occupied_points = np.asarray(occupied_layer[0], dtype=np.float32)
        scale = np.asarray(occupied_layer[1], dtype=np.float32)
        if np.any(scale <= 0.0):
            self.info_label.setText("占据栅格分辨率无效，无法刷新地图。")
            return False

        color, opacity, _label = self._LAYER_STYLE["occupied"]
        self._ros_node.publish_voxel_marker(
            "occupied", self._frame_id, occupied_points, scale, color, opacity
        )
        self._ros_node.publish_edited_occupied(self._frame_id, occupied_points, scale)
        self.info_label.setText(
            f"已发送编辑后的占据栅格给 jie_path_node，等待 C++ 节点重新生成派生栅格。"
            f"占据栅格数：{len(occupied_points)}。"
        )
        return True

    def _publish_visible_map_layers(self) -> None:
        for layer_name in ("occupied", "preblocked", "traversable"):
            layer = self._layer_data.get(layer_name)
            if layer is None:
                continue
            points, scale, color, opacity = layer
            self._ros_node.publish_voxel_marker(
                layer_name,
                self._frame_id,
                np.asarray(points, dtype=np.float32),
                np.asarray(scale, dtype=np.float32),
                color,
                opacity,
            )
        risk_layer = self._layer_data.get("risk")
        if risk_layer is not None:
            risk_points, _risk_scale, risk_intensity = risk_layer
            self._ros_node.publish_risk_cloud(
                self._frame_id,
                np.asarray(risk_points, dtype=np.float32),
                np.asarray(risk_intensity, dtype=np.float32),
            )

    def _points_to_grid_set(self, points: np.ndarray, scale: np.ndarray) -> set[tuple[int, int, int]]:
        indices = np.rint(points / scale).astype(np.int32)
        return {tuple(int(v) for v in idx) for idx in indices}

    def _grid_bounds(
        self, cells: set[tuple[int, int, int]], margin: int = 0
    ) -> tuple[np.ndarray, np.ndarray]:
        indices = np.asarray(list(cells), dtype=np.int32)
        return indices.min(axis=0) - margin, indices.max(axis=0) + margin

    def _grid_inside(self, idx: tuple[int, int, int], min_idx: np.ndarray, max_idx: np.ndarray) -> bool:
        return (
            int(min_idx[0]) <= idx[0] <= int(max_idx[0])
            and int(min_idx[1]) <= idx[1] <= int(max_idx[1])
            and int(min_idx[2]) <= idx[2] <= int(max_idx[2])
        )

    def _has_non_occupied_neighbor_same_level(
        self,
        idx: tuple[int, int, int],
        occupied_set: set[tuple[int, int, int]],
        min_idx: np.ndarray,
        max_idx: np.ndarray,
    ) -> bool:
        x, y, z = idx
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                n = (x + dx, y + dy, z)
                if self._grid_inside(n, min_idx, max_idx) and n not in occupied_set:
                    return True
        return False

    def _has_same_level_neighbor_with_occupied_above(
        self,
        idx: tuple[int, int, int],
        occupied_set: set[tuple[int, int, int]],
        min_idx: np.ndarray,
        max_idx: np.ndarray,
    ) -> bool:
        x, y, z = idx
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                n = (x + dx, y + dy, z)
                n_above = (x + dx, y + dy, z + 1)
                if (
                    self._grid_inside(n, min_idx, max_idx)
                    and n not in occupied_set
                    and self._grid_inside(n_above, min_idx, max_idx)
                    and n_above in occupied_set
                ):
                    return True
        return False

    def _recompute_preblocked_set(
        self,
        occupied_set: set[tuple[int, int, int]],
        min_idx: np.ndarray,
        max_idx: np.ndarray,
    ) -> set[tuple[int, int, int]]:
        candidates: set[tuple[int, int, int]] = set()
        for ox, oy, oz in occupied_set:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    candidates.add((ox + dx, oy + dy, oz))

        preblocked: set[tuple[int, int, int]] = set()
        for c in candidates:
            if not self._grid_inside(c, min_idx, max_idx) or c in occupied_set:
                continue
            below0 = (c[0], c[1], c[2] - 1)
            if (
                self._grid_inside(below0, min_idx, max_idx)
                and below0 in occupied_set
                and self._has_same_level_neighbor_with_occupied_above(c, occupied_set, min_idx, max_idx)
            ):
                preblocked.add(c)
                continue
            above1 = (c[0], c[1], c[2] + 1)
            if (
                self._has_non_occupied_neighbor_same_level(c, occupied_set, min_idx, max_idx)
                and not (self._grid_inside(above1, min_idx, max_idx) and above1 in occupied_set)
            ):
                below1 = (c[0], c[1], c[2] - 1)
                if self._grid_inside(below1, min_idx, max_idx) and below1 not in occupied_set:
                    preblocked.add(c)
        return preblocked

    def _has_ground_support(
        self,
        idx: tuple[int, int, int],
        occupied_set: set[tuple[int, int, int]],
        support_xy_radius: int = 1,
        support_depth: int = 1,
    ) -> bool:
        x, y, z = idx
        for dz in range(1, support_depth + 1):
            for dx in range(-support_xy_radius, support_xy_radius + 1):
                for dy in range(-support_xy_radius, support_xy_radius + 1):
                    if (x + dx, y + dy, z - dz) in occupied_set:
                        return True
        return False

    def _is_traversable_cell(
        self,
        idx: tuple[int, int, int],
        occupied_set: set[tuple[int, int, int]],
        preblocked_set: set[tuple[int, int, int]],
        min_idx: np.ndarray,
        max_idx: np.ndarray,
        scale: np.ndarray,
    ) -> bool:
        if not self._grid_inside(idx, min_idx, max_idx) or idx in occupied_set:
            return False
        if not self._has_ground_support(idx, occupied_set):
            return False
        for z in range(idx[2] - 1, int(min_idx[2]) - 1, -1):
            below_idx = (idx[0], idx[1], z)
            if below_idx in occupied_set:
                break
            if below_idx in preblocked_set:
                return False

        robot_radius = 0.25
        resolution = float(np.max(scale))
        radius_cells = max(1, int(np.ceil(robot_radius / max(resolution, 1.0e-6))))
        radius_sq = robot_radius * robot_radius
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                for dz in range(0, radius_cells + 1):
                    dist_sq = (
                        (dx * scale[0]) ** 2
                        + (dy * scale[1]) ** 2
                        + (dz * scale[2]) ** 2
                    )
                    if dist_sq > radius_sq:
                        continue
                    n = (idx[0] + dx, idx[1] + dy, idx[2] + dz)
                    if n in occupied_set or n in preblocked_set:
                        return False
        return True

    def _recompute_traversable_set(
        self,
        occupied_set: set[tuple[int, int, int]],
        preblocked_set: set[tuple[int, int, int]],
        min_idx: np.ndarray,
        max_idx: np.ndarray,
        scale: np.ndarray,
    ) -> set[tuple[int, int, int]]:
        traversable: set[tuple[int, int, int]] = set()
        for x in range(int(min_idx[0]), int(max_idx[0]) + 1):
            for y in range(int(min_idx[1]), int(max_idx[1]) + 1):
                for z in range(int(min_idx[2]), int(max_idx[2]) + 1):
                    idx = (x, y, z)
                    if self._is_traversable_cell(idx, occupied_set, preblocked_set, min_idx, max_idx, scale):
                        traversable.add(idx)
        return traversable

    def _recompute_risk_cost_set(
        self,
        preblocked_set: set[tuple[int, int, int]],
        traversable_set: set[tuple[int, int, int]],
        min_idx: np.ndarray,
        max_idx: np.ndarray,
    ) -> dict[tuple[int, int, int], float]:
        radius_cells = 3
        denom = float(radius_cells + 1)
        risk: dict[tuple[int, int, int], float] = {}
        for c in preblocked_set:
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    for dz in range(-radius_cells, radius_cells + 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        n = (c[0] + dx, c[1] + dy, c[2] + dz)
                        if (
                            not self._grid_inside(n, min_idx, max_idx)
                            or n not in traversable_set
                            or n in preblocked_set
                        ):
                            continue
                        dist = float(np.sqrt(dx * dx + dy * dy + dz * dz))
                        if dist > radius_cells:
                            continue
                        cost = max(0.0, (denom - dist) / denom)
                        if cost > risk.get(n, 0.0):
                            risk[n] = cost
        return risk

    def _grid_indices_to_points(
        self, cells: set[tuple[int, int, int]], scale: np.ndarray
    ) -> np.ndarray:
        if not cells:
            return np.empty((0, 3), dtype=np.float32)
        indices = np.asarray(sorted(cells), dtype=np.float32)
        return (indices * scale).astype(np.float32)

    def _set_grid_layer_from_indices(
        self, layer_name: str, cells: set[tuple[int, int, int]], scale: np.ndarray
    ) -> None:
        color, opacity, _label = self._LAYER_STYLE[layer_name]
        self._layer_data[layer_name] = (
            self._grid_indices_to_points(cells, scale),
            np.asarray(scale, dtype=np.float32),
            color,
            opacity,
        )

    def _set_risk_layer_from_costs(
        self, risk: dict[tuple[int, int, int], float], scale: np.ndarray
    ) -> None:
        if not risk:
            self._layer_data["risk"] = (
                np.empty((0, 3), dtype=np.float32),
                np.asarray(scale, dtype=np.float32),
                np.empty((0,), dtype=np.float32),
            )
            return
        cells = sorted(risk)
        points = (np.asarray(cells, dtype=np.float32) * scale).astype(np.float32)
        intensity = np.asarray([risk[cell] for cell in cells], dtype=np.float32)
        self._layer_data["risk"] = (points, np.asarray(scale, dtype=np.float32), intensity)

    def _update_point_actor(self, kind: str, xyz: tuple[float, float, float]) -> None:
        old_actor = self._start_actor if kind == "start" else self._goal_actor
        if old_actor is not None:
            self._renderer.RemoveActor(old_actor)

        sphere = vtk.vtkSphereSource()
        sphere.SetCenter(*xyz)
        sphere.SetRadius(0.16)
        sphere.SetThetaResolution(18)
        sphere.SetPhiResolution(18)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        if kind == "start":
            actor.GetProperty().SetColor(0.1, 0.95, 0.1)
            self._start_actor = actor
        else:
            actor.GetProperty().SetColor(0.95, 0.1, 0.1)
            self._goal_actor = actor
        actor.GetProperty().SetOpacity(1.0)
        self._renderer.AddActor(actor)
        self.vtk_widget.GetRenderWindow().Render()

    def _update_goal_visual(self, xyz: tuple[float, float, float], yaw: float) -> None:
        self._update_point_actor("goal", xyz)
        if self._goal_arrow_actor is not None:
            self._renderer.RemoveActor(self._goal_arrow_actor)
            self._goal_arrow_actor = None

        arrow = vtk.vtkArrowSource()
        arrow.SetTipResolution(24)
        arrow.SetShaftResolution(24)
        arrow.SetTipLength(0.30)
        arrow.SetTipRadius(0.18)
        arrow.SetShaftRadius(0.08)

        transform = vtk.vtkTransform()
        transform.PostMultiply()
        transform.Scale(0.90, 0.90, 0.90)
        transform.RotateZ(float(np.degrees(yaw)))
        transform.Translate(float(xyz[0]), float(xyz[1]), float(xyz[2]))

        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        transform_filter.SetInputConnection(arrow.GetOutputPort())

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transform_filter.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.95, 0.1, 0.1)
        actor.GetProperty().SetOpacity(0.95)
        self._renderer.AddActor(actor)
        self._goal_arrow_actor = actor
        self.vtk_widget.GetRenderWindow().Render()

    def _update_current_pose_visual(self, xyz: tuple[float, float, float], yaw: float) -> None:
        if self._current_pose_arrow_actor is not None:
            self._renderer.RemoveActor(self._current_pose_arrow_actor)
            self._current_pose_arrow_actor = None

        arrow = vtk.vtkArrowSource()
        arrow.SetTipResolution(24)
        arrow.SetShaftResolution(24)
        arrow.SetTipLength(0.30)
        arrow.SetTipRadius(0.18)
        arrow.SetShaftRadius(0.08)

        transform = vtk.vtkTransform()
        transform.PostMultiply()
        transform.Scale(0.90, 0.90, 0.90)
        transform.RotateZ(float(np.degrees(yaw)))
        transform.Translate(float(xyz[0]), float(xyz[1]), float(xyz[2]))

        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        transform_filter.SetInputConnection(arrow.GetOutputPort())

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transform_filter.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.10, 0.95, 0.10)
        actor.GetProperty().SetOpacity(0.95)
        self._renderer.AddActor(actor)
        self._current_pose_arrow_actor = actor
        self.vtk_widget.GetRenderWindow().Render()

    def _clear_current_pose_visual(self) -> None:
        if self._current_pose_arrow_actor is not None:
            self._renderer.RemoveActor(self._current_pose_arrow_actor)
            self._current_pose_arrow_actor = None
            self.vtk_widget.GetRenderWindow().Render()

    def _update_robot_visual(
        self, xyz: tuple[float, float, float], yaw: float
    ) -> None:
        display_xyz = (
            float(xyz[0]),
            float(xyz[1]),
            float(xyz[2]) + self._ROBOT_DISPLAY_Z_OFFSET,
        )
        if self._robot_actor is None:
            self._robot_actor = self._build_simple_dog_actor()
            if self._robot_actor is None:
                return
            self._renderer.AddActor(self._robot_actor)

        self._robot_actor.SetPosition(*display_xyz)
        self._robot_actor.SetOrientation(0.0, 0.0, float(np.degrees(yaw)))
        self.vtk_widget.GetRenderWindow().Render()

    def _build_simple_dog_actor(self) -> vtk.vtkAssembly | None:
        try:
            urdf_path = (
                Path(get_package_share_directory("d1_description")) / "urdf" / "simple_dog.urdf"
            )
            root = ET.parse(urdf_path).getroot()
        except Exception as exc:
            self.info_label.setText(f"加载 simple_dog.urdf 失败：{exc}")
            return None

        materials: dict[str, tuple[float, float, float, float]] = {}
        for material in root.findall("material"):
            name = material.get("name")
            color_tag = material.find("color")
            if not name or color_tag is None:
                continue
            rgba_text = color_tag.get("rgba", "").strip()
            if not rgba_text:
                continue
            rgba = tuple(float(v) for v in rgba_text.split())
            if len(rgba) == 4:
                materials[name] = rgba

        link_visuals: dict[str, list[dict[str, object]]] = {}
        for link in root.findall("link"):
            name = link.get("name")
            if not name:
                continue
            visuals: list[dict[str, object]] = []
            for visual in link.findall("visual"):
                geometry = visual.find("geometry")
                box = geometry.find("box") if geometry is not None else None
                if box is None:
                    continue
                size_text = box.get("size", "").strip()
                if not size_text:
                    continue
                size = tuple(float(v) for v in size_text.split())
                origin_tag = visual.find("origin")
                xyz = self._parse_xyz(origin_tag.get("xyz", "0 0 0") if origin_tag is not None else "0 0 0")
                rpy = self._parse_xyz(origin_tag.get("rpy", "0 0 0") if origin_tag is not None else "0 0 0")
                material_tag = visual.find("material")
                material_name = material_tag.get("name") if material_tag is not None else ""
                rgba = materials.get(material_name, (0.7, 0.7, 0.7, 1.0))
                visuals.append({"size": size, "xyz": xyz, "rpy": rpy, "rgba": rgba})
            link_visuals[name] = visuals

        children_by_parent: dict[str, list[tuple[str, np.ndarray]]] = {}
        for joint in root.findall("joint"):
            if joint.get("type") != "fixed":
                continue
            parent_tag = joint.find("parent")
            child_tag = joint.find("child")
            if parent_tag is None or child_tag is None:
                continue
            parent = parent_tag.get("link")
            child = child_tag.get("link")
            if not parent or not child:
                continue
            origin_tag = joint.find("origin")
            xyz = self._parse_xyz(origin_tag.get("xyz", "0 0 0") if origin_tag is not None else "0 0 0")
            rpy = self._parse_xyz(origin_tag.get("rpy", "0 0 0") if origin_tag is not None else "0 0 0")
            children_by_parent.setdefault(parent, []).append((child, self._make_transform(xyz, rpy)))

        assembly = vtk.vtkAssembly()

        def add_link_recursive(link_name: str, parent_transform: np.ndarray) -> None:
            for visual in link_visuals.get(link_name, []):
                actor = self._build_box_actor(
                    visual["size"], visual["rgba"], parent_transform @ self._make_transform(visual["xyz"], visual["rpy"])
                )
                assembly.AddPart(actor)
            for child_name, joint_transform in children_by_parent.get(link_name, []):
                add_link_recursive(child_name, parent_transform @ joint_transform)

        add_link_recursive("base_link", np.eye(4, dtype=np.float64))
        return assembly

    def _build_box_actor(
        self,
        size: tuple[float, float, float],
        rgba: tuple[float, float, float, float],
        transform_matrix: np.ndarray,
    ) -> vtk.vtkActor:
        cube = vtk.vtkCubeSource()
        cube.SetXLength(float(size[0]))
        cube.SetYLength(float(size[1]))
        cube.SetZLength(float(size[2]))

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(float(rgba[0]), float(rgba[1]), float(rgba[2]))
        actor.GetProperty().SetOpacity(float(rgba[3]))
        actor.SetUserMatrix(self._to_vtk_matrix(transform_matrix))
        return actor

    def _parse_xyz(self, text: str) -> tuple[float, float, float]:
        values = [float(v) for v in text.split()]
        if len(values) != 3:
            return (0.0, 0.0, 0.0)
        return (values[0], values[1], values[2])

    def _make_transform(
        self,
        xyz: tuple[float, float, float],
        rpy: tuple[float, float, float],
    ) -> np.ndarray:
        roll, pitch, yaw = rpy
        cx, sx = np.cos(roll), np.sin(roll)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cz, sz = np.cos(yaw), np.sin(yaw)

        rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
        ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
        rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
        rotation = rz @ ry @ rx

        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = np.asarray(xyz, dtype=np.float64)
        return transform

    def _to_vtk_matrix(self, matrix: np.ndarray) -> vtk.vtkMatrix4x4:
        vtk_matrix = vtk.vtkMatrix4x4()
        for row in range(4):
            for col in range(4):
                vtk_matrix.SetElement(row, col, float(matrix[row, col]))
        return vtk_matrix

    def _update_path(self, path_points: list[tuple[float, float, float]]) -> None:
        if self._path_actor is not None:
            self._renderer.RemoveActor(self._path_actor)
            self._path_actor = None
        if len(path_points) < 2:
            self.vtk_widget.GetRenderWindow().Render()
            return

        vtk_points = vtk.vtkPoints()
        for point in path_points:
            vtk_points.InsertNextPoint(point)

        poly_line = vtk.vtkPolyLine()
        poly_line.GetPointIds().SetNumberOfIds(len(path_points))
        for i in range(len(path_points)):
            poly_line.GetPointIds().SetId(i, i)

        cells = vtk.vtkCellArray()
        cells.InsertNextCell(poly_line)

        poly_data = vtk.vtkPolyData()
        poly_data.SetPoints(vtk_points)
        poly_data.SetLines(cells)

        tube = vtk.vtkTubeFilter()
        tube.SetInputData(poly_data)
        tube.SetRadius(0.06)
        tube.SetNumberOfSides(16)
        tube.CappingOn()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(tube.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.69, 0.40, 1.0)
        actor.GetProperty().SetOpacity(1.0)
        self._renderer.AddActor(actor)
        self._path_actor = actor
        self.vtk_widget.GetRenderWindow().Render()

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

    def _choose_and_open(self) -> None:
        start_dir = self.path_edit.text().strip() or str(self._default_root)
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图目录",
            start_dir,
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if not selected:
            return
        selected_path = Path(selected)
        self.path_edit.setText(str(selected_path.parent))
        self.name_edit.setText(selected_path.name)
        self._start_load_for_package(selected_path)

    def _load_map_package(self, package_dir: Path) -> None:
        meta_path = package_dir / "meta.yaml"
        layers_path = package_dir / "layers.npz"
        if not meta_path.exists() or not layers_path.exists():
            QMessageBox.critical(self, "打开地图", "目录中缺少 meta.yaml 或 layers.npz。")
            return

        try:
            with meta_path.open("r", encoding="utf-8") as f:
                meta = yaml.safe_load(f)
            layers = np.load(layers_path, allow_pickle=False)
        except Exception as exc:
            QMessageBox.critical(self, "打开地图", f"读取地图失败：{exc}")
            return

        occupied_layer = self._layer_data.get("occupied")
        self._layer_data.clear()
        for layer_name in ("occupied", "preblocked", "traversable"):
            points_key = f"{layer_name}_points"
            scale_key = f"{layer_name}_scale"
            if points_key in layers:
                color, opacity, _label = self._LAYER_STYLE[layer_name]
                self._layer_data[layer_name] = (
                    layers[points_key],
                    layers[scale_key],
                    color,
                    opacity,
                )

        if "occupied" not in self._layer_data and occupied_layer is not None:
            self._layer_data["occupied"] = occupied_layer

        if "risk_points" in layers and "risk_intensity" in layers:
            risk_scale = (
                np.asarray(layers["traversable_scale"], dtype=np.float32)
                if "traversable_scale" in layers
                else np.asarray(
                    self._layer_data.get("occupied", (np.empty((0, 3)), np.array([0.2, 0.2, 0.2], dtype=np.float32), None, None))[1],
                    dtype=np.float32,
                )
            )
            self._layer_data["risk"] = (
                np.asarray(layers["risk_points"], dtype=np.float32),
                risk_scale,
                np.asarray(layers["risk_intensity"], dtype=np.float32),
            )

        if not self._layer_data:
            QMessageBox.critical(self, "打开地图", "地图文件中没有可显示的体素层。")
            return

        self._refresh_layers(reset_camera=True)
        self._sync_external_preblocked()
        if self.edit_checkbox.isChecked():
            self._initialize_edit_position()
            self._update_edit_cursor()
        map_id = meta.get("map_id", "")
        resolution = meta.get("resolution", 0.0)
        self._frame_id = meta.get("frame_id", "map")
        occupied_count = len(self._layer_data.get("occupied", (np.empty((0, 3)), None, None))[0])
        self.info_label.setText(
            f"地图：{map_id}    分辨率：{resolution:.2f} 米    占据体素：{occupied_count}"
        )

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

    def _refresh_layers(self, checked: bool | None = None, reset_camera: bool = False) -> None:
        del checked
        for actor, edge_actor in self._layer_actors.values():
            self._renderer.RemoveActor(actor)
            if edge_actor is not None:
                self._renderer.RemoveActor(edge_actor)
        self._layer_actors.clear()

        layer_visibility = {
            "occupied": self.occupied_checkbox.isChecked(),
            "preblocked": self.preblocked_checkbox.isChecked(),
            "traversable": self.traversable_checkbox.isChecked(),
            "risk": self.risk_checkbox.isChecked(),
        }

        for layer_name, visible in layer_visibility.items():
            if not visible or layer_name not in self._layer_data:
                continue
            if layer_name == "risk":
                points, scale, intensity = self._layer_data[layer_name]
                if points.size == 0:
                    continue
                actor, edge_actor = self._build_risk_actors(points, scale, intensity)
            else:
                points, scale, color, opacity = self._layer_data[layer_name]
                if points.size == 0:
                    continue
                actor, edge_actor = self._build_voxel_actors(points, scale, color, opacity)
            self._renderer.AddActor(actor)
            if edge_actor is not None:
                self._renderer.AddActor(edge_actor)
            self._layer_actors[layer_name] = (actor, edge_actor)

        if reset_camera:
            self._renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()

    def _build_risk_actors(
        self, points: np.ndarray, scale: np.ndarray, intensity: np.ndarray
    ) -> tuple[vtk.vtkActor, None]:
        vtk_points = vtk.vtkPoints()
        vtk_points.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)

        alphas = np.clip(0.12 + 0.83 * intensity.astype(np.float32), 0.12, 0.95)
        colors = np.zeros((len(points), 4), dtype=np.uint8)
        colors[:, 0] = int(0.15 * 255.0)
        colors[:, 1] = int(0.35 * 255.0)
        colors[:, 2] = int(1.0 * 255.0)
        colors[:, 3] = np.round(alphas * 255.0).astype(np.uint8)
        vtk_colors = numpy_support.numpy_to_vtk(colors, deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
        vtk_colors.SetName("risk_rgba")
        polydata.GetPointData().SetScalars(vtk_colors)

        cube = vtk.vtkCubeSource()
        cube.SetXLength(float(scale[0]))
        cube.SetYLength(float(scale[1]))
        cube.SetZLength(float(scale[2]))
        glyph = vtk.vtkGlyph3DMapper()
        glyph.SetInputData(polydata)
        glyph.SetSourceConnection(cube.GetOutputPort())
        glyph.ScalingOff()
        glyph.SetScalarModeToUsePointData()
        glyph.ScalarVisibilityOn()
        glyph.SetColorModeToDirectScalars()

        actor = vtk.vtkActor()
        actor.SetMapper(glyph)
        actor.GetProperty().SetOpacity(1.0)
        actor.GetProperty().SetInterpolationToFlat()
        return actor, None


def main() -> None:
    rclpy.init()
    app = QApplication([])
    window = MapViewerWindow()
    window.show()
    app.exec_()
    window._ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
