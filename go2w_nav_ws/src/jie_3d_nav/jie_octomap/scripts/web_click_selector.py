#!/usr/bin/env python3

import math
from typing import Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


GridIndex = Tuple[int, int, int]


class WebClickSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("web_click_selector")

        self.declare_parameter("occupied_marker_topic", "/octomap_occupied_markers")
        self.declare_parameter("preblocked_marker_topic", "/preblocked_cells_markers")
        self.declare_parameter("raw_click_topic", "/web_clicked_point")
        self.declare_parameter("marker_topic", "/selection_markers")
        self.declare_parameter("start_topic", "/start_point")
        self.declare_parameter("goal_topic", "/goal_point")
        self.declare_parameter("status_topic", "/web_selection_status")
        self.declare_parameter("arrow_height", 0.6)
        self.declare_parameter("arrow_length", 0.7)
        self.declare_parameter("shaft_diameter", 0.16)
        self.declare_parameter("head_diameter", 0.32)
        self.declare_parameter("head_length", 0.44)
        self.declare_parameter("cube_size", 0.20)
        self.declare_parameter("robot_radius", 0.25)
        self.declare_parameter("snap_search_radius_cells", 12)
        self.declare_parameter("require_ground_support", True)
        self.declare_parameter("strict_direct_ground_support", False)
        self.declare_parameter("ground_support_xy_radius_cells", 1)
        self.declare_parameter("ground_support_depth_cells", 1)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        occupied_marker_topic = self.get_parameter("occupied_marker_topic").value
        preblocked_marker_topic = self.get_parameter("preblocked_marker_topic").value
        raw_click_topic = self.get_parameter("raw_click_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        start_topic = self.get_parameter("start_topic").value
        goal_topic = self.get_parameter("goal_topic").value
        status_topic = self.get_parameter("status_topic").value

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, transient_qos)
        self.start_pub = self.create_publisher(PointStamped, start_topic, transient_qos)
        self.goal_pub = self.create_publisher(PointStamped, goal_topic, transient_qos)
        self.status_pub = self.create_publisher(String, status_topic, transient_qos)

        self.create_subscription(Marker, occupied_marker_topic, self._on_occupied, transient_qos)
        self.create_subscription(Marker, preblocked_marker_topic, self._on_preblocked, transient_qos)
        self.create_subscription(PointStamped, raw_click_topic, self._on_raw_click, 10)

        self.expect_start = True
        self.has_start = False
        self.has_goal = False
        self.start_point: Optional[PointStamped] = None
        self.goal_point: Optional[PointStamped] = None

        self.map_frame = "map"
        self.resolution = 0.2
        self.occupied_cells: Set[GridIndex] = set()
        self.preblocked_cells: Set[GridIndex] = set()
        self.min_idx: Optional[GridIndex] = None
        self.max_idx: Optional[GridIndex] = None

        self._publish_status("等待占据栅格地图。")
        self.get_logger().info(
            f"web_click_selector started. raw_click_topic={raw_click_topic} occupied_marker_topic={occupied_marker_topic}"
        )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _on_occupied(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return

        self.map_frame = msg.header.frame_id or self.map_frame
        self.resolution = max(1e-6, float(msg.scale.x))
        occupied: Set[GridIndex] = set()

        min_x = min_y = min_z = math.inf
        max_x = max_y = max_z = -math.inf

        for p in msg.points:
            idx = self._world_to_grid(p.x, p.y, p.z)
            occupied.add(idx)
            min_x = min(min_x, idx[0])
            min_y = min(min_y, idx[1])
            min_z = min(min_z, idx[2])
            max_x = max(max_x, idx[0])
            max_y = max(max_y, idx[1])
            max_z = max(max_z, idx[2])

        self.occupied_cells = occupied
        if occupied:
            self.min_idx = (int(min_x), int(min_y), int(min_z))
            self.max_idx = (int(max_x), int(max_y), int(max_z))
            self._publish_status(
                f"地图已就绪：{len(self.occupied_cells)} 个占据栅格。"
            )

    def _on_preblocked(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return

        preblocked: Set[GridIndex] = set()
        for p in msg.points:
            preblocked.add(self._world_to_grid(p.x, p.y, p.z))
        self.preblocked_cells = preblocked

    def _on_raw_click(self, msg: PointStamped) -> None:
        if not self.occupied_cells or self.min_idx is None or self.max_idx is None:
            self._publish_status("地图尚未就绪。")
            return

        seed = self._world_to_grid(msg.point.x, msg.point.y, msg.point.z)
        snapped = self._find_nearest_traversable(seed)
        if snapped is None:
            self._publish_status("点击位置附近没有找到具备地面支撑的可通行栅格。")
            self.get_logger().warn(
                f"Raw click [{msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}] could not be snapped to a traversable cell."
            )
            return

        point = self._grid_to_point(snapped)
        snapped_msg = PointStamped()
        snapped_msg.header.frame_id = self.map_frame
        snapped_msg.header.stamp = self.get_clock().now().to_msg()
        snapped_msg.point = point

        if self.expect_start:
            self.start_point = snapped_msg
            self.has_start = True
            self.start_pub.publish(snapped_msg)
            self._publish_status(
                f"起点已设置为 [{point.x:.2f}, {point.y:.2f}, {point.z:.2f}]。再次点击可设置终点。"
            )
            self.get_logger().info(
                f"Set START point from web click: [{point.x:.3f}, {point.y:.3f}, {point.z:.3f}]"
            )
        else:
            self.goal_point = snapped_msg
            self.has_goal = True
            self.goal_pub.publish(snapped_msg)
            self._publish_status(
                f"终点已设置为 [{point.x:.2f}, {point.y:.2f}, {point.z:.2f}]。再次点击可设置起点。"
            )
            self.get_logger().info(
                f"Set GOAL point from web click: [{point.x:.3f}, {point.y:.3f}, {point.z:.3f}]"
            )

        self.expect_start = not self.expect_start
        self._publish_markers()

    def _world_to_grid(self, x: float, y: float, z: float) -> GridIndex:
        r = self.resolution
        return (math.floor(x / r), math.floor(y / r), math.floor(z / r))

    def _grid_to_point(self, idx: GridIndex) -> Point:
        r = self.resolution
        p = Point()
        p.x = (idx[0] + 0.5) * r
        p.y = (idx[1] + 0.5) * r
        p.z = (idx[2] + 0.5) * r
        return p

    def _is_inside_bounds(self, idx: GridIndex) -> bool:
        if self.min_idx is None or self.max_idx is None:
            return False
        return (
            self.min_idx[0] <= idx[0] <= self.max_idx[0]
            and self.min_idx[1] <= idx[1] <= self.max_idx[1]
            and self.min_idx[2] <= idx[2] <= self.max_idx[2]
        )

    def _is_occupied(self, idx: GridIndex) -> bool:
        return idx in self.occupied_cells

    def _has_ground_support(self, idx: GridIndex) -> bool:
        strict_direct = bool(self.get_parameter("strict_direct_ground_support").value)
        support_xy_radius = int(self.get_parameter("ground_support_xy_radius_cells").value)
        support_depth = max(1, int(self.get_parameter("ground_support_depth_cells").value))

        if strict_direct:
            below = (idx[0], idx[1], idx[2] - 1)
            return self._is_inside_bounds(below) and self._is_occupied(below)

        for dz in range(1, support_depth + 1):
            for dx in range(-support_xy_radius, support_xy_radius + 1):
                for dy in range(-support_xy_radius, support_xy_radius + 1):
                    below = (idx[0] + dx, idx[1] + dy, idx[2] - dz)
                    if self._is_inside_bounds(below) and self._is_occupied(below):
                        return True
        return False

    def _is_traversable(self, idx: GridIndex) -> bool:
        if not self._is_inside_bounds(idx):
            return False
        if self._is_occupied(idx):
            return False
        if idx in self.preblocked_cells:
            return False

        require_ground = bool(self.get_parameter("require_ground_support").value)
        if require_ground and not self._has_ground_support(idx):
            return False

        robot_radius = float(self.get_parameter("robot_radius").value)
        n = max(1, math.ceil(robot_radius / self.resolution))
        radius_sq = robot_radius * robot_radius

        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                for dz in range(0, n + 1):
                    dist_sq = (
                        (dx * self.resolution) * (dx * self.resolution)
                        + (dy * self.resolution) * (dy * self.resolution)
                        + (dz * self.resolution) * (dz * self.resolution)
                    )
                    if dist_sq > radius_sq:
                        continue
                    if self._is_occupied((idx[0] + dx, idx[1] + dy, idx[2] + dz)):
                        return False
        return True

    def _find_nearest_traversable(self, seed: GridIndex) -> Optional[GridIndex]:
        if self._is_traversable(seed):
            return seed

        radius_cells = int(self.get_parameter("snap_search_radius_cells").value)
        for r in range(1, radius_cells + 1):
            for dz in range(0, r + 1):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        if max(abs(dx), abs(dy), abs(dz)) != r:
                            continue

                        c1 = (seed[0] + dx, seed[1] + dy, seed[2] + dz)
                        if self._is_traversable(c1):
                            return c1

                        if dz > 0:
                            c2 = (seed[0] + dx, seed[1] + dy, seed[2] - dz)
                            if self._is_traversable(c2):
                                return c2
        return None

    def _make_arrow(self, marker_id: int, p: PointStamped, rgb: Tuple[float, float, float]) -> Marker:
        marker = Marker()
        marker.header = p.header
        marker.ns = "web_selector"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = float(self.get_parameter("shaft_diameter").value)
        marker.scale.y = float(self.get_parameter("head_diameter").value)
        marker.scale.z = float(self.get_parameter("head_length").value)
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        base = Point()
        base.x = p.point.x
        base.y = p.point.y
        base.z = p.point.z + float(self.get_parameter("arrow_height").value)
        tip = Point()
        tip.x = base.x
        tip.y = base.y
        tip.z = base.z - float(self.get_parameter("arrow_length").value)

        marker.points = [base, tip]
        return marker

    def _make_cube(self, marker_id: int, p: PointStamped, rgb: Tuple[float, float, float]) -> Marker:
        marker = Marker()
        marker.header = p.header
        marker.ns = "web_selector"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = p.point
        marker.pose.orientation.w = 1.0
        cube_size = float(self.get_parameter("cube_size").value)
        marker.scale.x = cube_size
        marker.scale.y = cube_size
        marker.scale.z = cube_size
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 0.95
        return marker

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        if self.has_start and self.start_point is not None:
            markers.markers.append(self._make_arrow(0, self.start_point, (0.1, 0.95, 0.1)))
            markers.markers.append(self._make_cube(2, self.start_point, (0.1, 0.95, 0.1)))
        if self.has_goal and self.goal_point is not None:
            markers.markers.append(self._make_arrow(1, self.goal_point, (0.95, 0.1, 0.1)))
            markers.markers.append(self._make_cube(3, self.goal_point, (0.95, 0.1, 0.1)))
        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebClickSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
