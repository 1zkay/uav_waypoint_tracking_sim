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

from uav_trajectory_tracking.parametric_trajectory import ParametricTrajectory, Point3


Color = tuple[float, float, float, float]


class TrajectoryVisualizer(Node):
    """Publish RViz markers for a sampled PX4 local NED parametric trajectory."""

    def __init__(self) -> None:
        super().__init__("trajectory_visualizer")

        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("trajectory_stage_topic", "/trajectory_tracker/current_stage")
        self.declare_parameter("trajectory_markers_topic", "/trajectory_markers")
        self.declare_parameter("trajectory_path_topic", "/trajectory_path")
        self.declare_parameter("vehicle_path_topic", "/vehicle_path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("vehicle_path_min_distance_m", 0.15)
        self.declare_parameter("vehicle_path_max_points", 2000)

        config = self._load_config()
        self.trajectory = ParametricTrajectory.from_config(config)
        self.planned_points = self.trajectory.sample_points(include_return=True)
        self.acceptance_radius_m = self.trajectory.acceptance_radius_m
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.path_min_distance_m = float(
            self.get_parameter("vehicle_path_min_distance_m").get_parameter_value().double_value
        )
        self.path_max_points = int(
            self.get_parameter("vehicle_path_max_points").get_parameter_value().integer_value
        )

        self.current_stage_index = 0
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
        trajectory_stage_topic = self.get_parameter("trajectory_stage_topic").get_parameter_value().string_value
        trajectory_markers_topic = self.get_parameter("trajectory_markers_topic").get_parameter_value().string_value
        trajectory_path_topic = self.get_parameter("trajectory_path_topic").get_parameter_value().string_value
        vehicle_path_topic = self.get_parameter("vehicle_path_topic").get_parameter_value().string_value

        self.create_subscription(
            VehicleLocalPosition,
            vehicle_local_position_topic,
            self._vehicle_local_position_callback,
            px4_qos,
        )
        self.create_subscription(Int32, trajectory_stage_topic, self._trajectory_stage_callback, tracker_state_qos)

        self.marker_pub = self.create_publisher(MarkerArray, trajectory_markers_topic, 10)
        self.planned_path_pub = self.create_publisher(NavPath, trajectory_path_topic, 10)
        self.vehicle_path_pub = self.create_publisher(NavPath, vehicle_path_topic, 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").get_parameter_value().double_value)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._timer_callback)

        self.get_logger().info(
            f"Visualizing {len(self.planned_points)} sampled parametric path points in RViz "
            f"frame '{self.frame_id}'. PX4 NED is converted to ROS ENU."
        )
        self.get_logger().info(
            f"Publishing visualization topics: {trajectory_markers_topic}, "
            f"{trajectory_path_topic}, {vehicle_path_topic}"
        )

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("trajectory_file").get_parameter_value().string_value

        if not config_path:
            share_dir = Path(get_package_share_directory("uav_trajectory_tracking"))
            config_path = str(share_dir / "config" / "trajectory_hold.yaml")

        path = Path(config_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Trajectory config does not exist: {path}")

        with path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        if str(config.get("frame", "NED")).upper() != "NED":
            raise ValueError("Only PX4 local NED trajectory configs are supported.")

        return config

    def _trajectory_stage_callback(self, msg: Int32) -> None:
        self.current_stage_index = max(0, int(msg.data))

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
        planned_path_points = [self._ned_to_enu(point) for point in self.planned_points]

        self.marker_pub.publish(self._make_markers(planned_path_points, now))
        self.planned_path_pub.publish(self._make_path(planned_path_points, now))
        self.vehicle_path_pub.publish(self._make_path(self.vehicle_path_points, now))

    def _make_markers(self, planned_points: list[Point], stamp) -> MarkerArray:
        markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = self.frame_id
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        marker_id = 1
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

        if planned_points:
            markers.markers.append(
                self._sphere(
                    marker_id,
                    "trajectory_start",
                    planned_points[0],
                    0.42,
                    (0.05, 0.35, 1.0, 0.95),
                    stamp,
                )
            )
            marker_id += 1
            markers.markers.append(
                self._sphere(
                    marker_id,
                    "trajectory_end",
                    planned_points[-1],
                    0.46,
                    (0.1, 0.75, 0.25, 1.0),
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

    def _set_color(self, marker: Marker, color: Color) -> None:
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    def _ned_to_enu(self, point: Point3) -> Point:
        x_north, y_east, z_down = point
        return Point(x=y_east, y=x_north, z=-z_down)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryVisualizer()

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
