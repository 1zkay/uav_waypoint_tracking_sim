#!/usr/bin/env python3

import math
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as NavPath
from px4_msgs.msg import VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray


Color = tuple[float, float, float, float]
Waypoint = tuple[float, float, float]


class WaypointVisualizer(Node):
    """Publish RViz markers for PX4 local NED waypoint tracking."""

    def __init__(self) -> None:
        super().__init__("waypoint_visualizer")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("current_index_topic", "/waypoint_tracker/current_waypoint_index")
        self.declare_parameter("waypoint_markers_topic", "/waypoint_markers")
        self.declare_parameter("waypoint_path_topic", "/waypoint_path")
        self.declare_parameter("vehicle_path_topic", "/vehicle_path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("vehicle_path_min_distance_m", 0.15)
        self.declare_parameter("vehicle_path_max_points", 2000)

        config = self._load_config()
        self.waypoints = self._parse_waypoints(config)
        self.acceptance_radius_m = float(config.get("acceptance_radius_m", 1.0))
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.path_min_distance_m = float(
            self.get_parameter("vehicle_path_min_distance_m").get_parameter_value().double_value
        )
        self.path_max_points = int(
            self.get_parameter("vehicle_path_max_points").get_parameter_value().integer_value
        )

        self.current_index = 0
        self.vehicle_local_position: VehicleLocalPosition | None = None
        self.vehicle_path_points: list[Point] = []

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        tracker_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        vehicle_local_position_topic = (
            self.get_parameter("vehicle_local_position_topic").get_parameter_value().string_value
        )
        current_index_topic = self.get_parameter("current_index_topic").get_parameter_value().string_value
        waypoint_markers_topic = self.get_parameter("waypoint_markers_topic").get_parameter_value().string_value
        waypoint_path_topic = self.get_parameter("waypoint_path_topic").get_parameter_value().string_value
        vehicle_path_topic = self.get_parameter("vehicle_path_topic").get_parameter_value().string_value

        self.create_subscription(
            VehicleLocalPosition,
            vehicle_local_position_topic,
            self._vehicle_local_position_callback,
            px4_qos,
        )
        self.create_subscription(Int32, current_index_topic, self._current_index_callback, tracker_state_qos)

        self.marker_pub = self.create_publisher(MarkerArray, waypoint_markers_topic, 10)
        self.planned_path_pub = self.create_publisher(NavPath, waypoint_path_topic, 10)
        self.vehicle_path_pub = self.create_publisher(NavPath, vehicle_path_topic, 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").get_parameter_value().double_value)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._timer_callback)

        self.get_logger().info(
            f"Visualizing {len(self.waypoints)} waypoints in RViz frame '{self.frame_id}'. "
            "PX4 NED is converted to ROS ENU."
        )
        self.get_logger().info(
            f"Publishing visualization topics: {waypoint_markers_topic}, "
            f"{waypoint_path_topic}, {vehicle_path_topic}"
        )

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("waypoints_file").get_parameter_value().string_value

        if not config_path:
            share_dir = Path(get_package_share_directory("uav_waypoint_tracking"))
            config_path = str(share_dir / "config" / "waypoints.yaml")

        path = Path(config_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Waypoint config does not exist: {path}")

        with path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        if str(config.get("frame", "NED")).upper() != "NED":
            raise ValueError("Only PX4 local NED waypoints are supported in this package.")

        return config

    def _parse_waypoints(self, config: dict[str, Any]) -> list[Waypoint]:
        raw_waypoints = config.get("waypoints")
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise ValueError("Config must contain a non-empty 'waypoints' list.")

        waypoints: list[Waypoint] = []
        for index, waypoint in enumerate(raw_waypoints):
            if not isinstance(waypoint, list | tuple) or len(waypoint) != 3:
                raise ValueError(f"Waypoint {index} must be [x, y, z].")
            waypoints.append(tuple(float(value) for value in waypoint))

        return waypoints

    def _current_index_callback(self, msg: Int32) -> None:
        self.current_index = max(0, min(int(msg.data), len(self.waypoints)))

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

        if not msg.xy_valid or not msg.z_valid:
            return

        point = self._ned_to_enu((msg.x, msg.y, msg.z))
        if self.vehicle_path_points:
            previous = self.vehicle_path_points[-1]
            distance = math.dist((previous.x, previous.y, previous.z), (point.x, point.y, point.z))
            if distance < self.path_min_distance_m:
                return

        self.vehicle_path_points.append(point)
        if len(self.vehicle_path_points) > self.path_max_points:
            self.vehicle_path_points = self.vehicle_path_points[-self.path_max_points :]

    def _timer_callback(self) -> None:
        now = self.get_clock().now().to_msg()

        self.marker_pub.publish(self._make_markers(now))
        self.planned_path_pub.publish(self._make_path([self._ned_to_enu(wp) for wp in self.waypoints], now))
        self.vehicle_path_pub.publish(self._make_path(self.vehicle_path_points, now))

    def _make_markers(self, stamp) -> MarkerArray:
        markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = self.frame_id
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        marker_id = 1

        planned_points = [self._ned_to_enu(wp) for wp in self.waypoints]
        markers.markers.append(
            self._line_strip(
                marker_id,
                "planned_route",
                planned_points,
                (0.0, 0.45, 1.0, 0.85),
                0.08,
                stamp,
            )
        )
        marker_id += 1

        for index, waypoint in enumerate(self.waypoints):
            point = self._ned_to_enu(waypoint)
            if self.current_index >= len(self.waypoints) or index < self.current_index:
                color = (0.1, 0.75, 0.25, 1.0)
            elif index == self.current_index:
                color = (1.0, 0.55, 0.05, 1.0)
            else:
                color = (0.05, 0.35, 1.0, 0.9)

            markers.markers.append(
                self._sphere(marker_id, "waypoint_points", point, 0.58, color, stamp)
            )
            marker_id += 1
            markers.markers.append(
                self._acceptance_disk(marker_id, point, self.acceptance_radius_m, color, stamp)
            )
            marker_id += 1
            markers.markers.append(
                self._text(
                    marker_id,
                    "waypoint_labels",
                    Point(x=point.x, y=point.y, z=point.z + 0.9),
                    f"WP{index + 1}",
                    color,
                    stamp,
                )
            )
            marker_id += 1

        if self.vehicle_local_position is not None and self.vehicle_local_position.xy_valid:
            vehicle_point = self._ned_to_enu(
                (
                    self.vehicle_local_position.x,
                    self.vehicle_local_position.y,
                    self.vehicle_local_position.z,
                )
            )
            markers.markers.append(
                self._sphere(marker_id, "vehicle", vehicle_point, 0.38, (0.95, 0.95, 0.95, 1.0), stamp)
            )
            marker_id += 1

            if self.current_index < len(planned_points):
                markers.markers.append(
                    self._arrow(
                        marker_id,
                        "target_vector",
                        vehicle_point,
                        planned_points[self.current_index],
                        (1.0, 0.55, 0.0, 0.85),
                        stamp,
                    )
                )

        return markers

    def _make_path(self, points: list[Point], stamp) -> NavPath:
        path = NavPath()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp

        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def _base_marker(self, marker_id: int, namespace: str, marker_type: int, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _line_strip(
        self,
        marker_id: int,
        namespace: str,
        points: list[Point],
        color: Color,
        width: float,
        stamp,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.LINE_STRIP, stamp)
        marker.points = points
        marker.scale.x = width
        self._set_color(marker, color)
        return marker

    def _sphere(self, marker_id: int, namespace: str, point: Point, diameter: float, color: Color, stamp) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.SPHERE, stamp)
        marker.pose.position = point
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        self._set_color(marker, color)
        return marker

    def _acceptance_disk(self, marker_id: int, point: Point, radius: float, color: Color, stamp) -> Marker:
        marker = self._base_marker(marker_id, "acceptance_radius", Marker.CYLINDER, stamp)
        marker.pose.position = Point(x=point.x, y=point.y, z=point.z)
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = 0.04
        self._set_color(marker, (color[0], color[1], color[2], 0.18))
        return marker

    def _text(self, marker_id: int, namespace: str, point: Point, text: str, color: Color, stamp) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.TEXT_VIEW_FACING, stamp)
        marker.pose.position = point
        marker.scale.z = 0.6
        marker.text = text
        self._set_color(marker, color)
        return marker

    def _arrow(self, marker_id: int, namespace: str, start: Point, end: Point, color: Color, stamp) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.ARROW, stamp)
        marker.points = [start, end]
        marker.scale.x = 0.08
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        self._set_color(marker, color)
        return marker

    def _set_color(self, marker: Marker, color: Color) -> None:
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    def _ned_to_enu(self, point: Waypoint) -> Point:
        x_north, y_east, z_down = point
        return Point(x=y_east, y=x_north, z=-z_down)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaypointVisualizer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
