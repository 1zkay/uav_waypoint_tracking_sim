#!/usr/bin/env python3

import math
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32


class WaypointTracker(Node):
    """Track local NED waypoints through PX4 offboard position setpoints."""

    def __init__(self) -> None:
        super().__init__("waypoint_tracker")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v1")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        config = self._load_config()

        self.waypoints = self._parse_waypoints(config)
        self.acceptance_radius_m = float(config.get("acceptance_radius_m", 1.0))
        self.dwell_time_s = float(config.get("dwell_time_s", 0.5))
        self.control_rate_hz = float(config.get("control_rate_hz", 10.0))
        self.takeoff_warmup_s = float(config.get("takeoff_warmup_s", 1.5))
        self.land_at_end = bool(config.get("land_at_end", True))
        self.yaw_mode = str(config.get("yaw_mode", "face_next_waypoint"))

        self.current_waypoint_index = 0
        self.reached_since_us: int | None = None
        self.setpoint_counter = 0
        self.land_command_sent = False
        self.last_land_command_us = 0
        self.last_mode_request_us = 0
        self.last_arm_request_us = 0

        self.vehicle_local_position: VehicleLocalPosition | None = None
        self.vehicle_status: VehicleStatus | None = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )
        tracker_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.current_index_pub = self.create_publisher(
            Int32, "/waypoint_tracker/current_waypoint_index", tracker_state_qos
        )

        vehicle_local_position_topic = (
            self.get_parameter("vehicle_local_position_topic").get_parameter_value().string_value
        )
        vehicle_status_topic = self.get_parameter("vehicle_status_topic").get_parameter_value().string_value

        self.create_subscription(
            VehicleLocalPosition,
            vehicle_local_position_topic,
            self._vehicle_local_position_callback,
            qos_profile,
        )
        self.create_subscription(
            VehicleStatus,
            vehicle_status_topic,
            self._vehicle_status_callback,
            qos_profile,
        )

        timer_period = 1.0 / max(self.control_rate_hz, 2.0)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} NED waypoints; "
            f"radius={self.acceptance_radius_m:.2f} m, rate={self.control_rate_hz:.1f} Hz"
        )
        self.get_logger().info(
            f"Subscribing to state topics: {vehicle_status_topic}, {vehicle_local_position_topic}"
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

    def _parse_waypoints(self, config: dict[str, Any]) -> list[tuple[float, float, float]]:
        raw_waypoints = config.get("waypoints")
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise ValueError("Config must contain a non-empty 'waypoints' list.")

        waypoints: list[tuple[float, float, float]] = []
        for index, waypoint in enumerate(raw_waypoints):
            if not isinstance(waypoint, list | tuple) or len(waypoint) != 3:
                raise ValueError(f"Waypoint {index} must be [x, y, z].")
            x, y, z = (float(value) for value in waypoint)
            if z >= 0.0:
                self.get_logger().warn(
                    f"Waypoint {index} z={z:.2f}; PX4 NED altitude above home should be negative."
                )
            waypoints.append((x, y, z))

        return waypoints

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _timer_callback(self) -> None:
        now_us = self._now_us()

        self._publish_offboard_control_mode(now_us)

        if not self.land_command_sent:
            self._publish_current_setpoint(now_us)

        warmup_cycles = max(1, int(self.takeoff_warmup_s * self.control_rate_hz))
        if self.setpoint_counter >= warmup_cycles and not self.land_command_sent:
            self._request_offboard_and_arm_if_needed(now_us)

        if self._ready_to_track() and not self.land_command_sent:
            self._advance_waypoint_if_reached(now_us)

        if self.land_command_sent and now_us - self.last_land_command_us > 1_000_000:
            self._land(now_us)

        self._publish_current_index()
        self.setpoint_counter += 1

    def _ready_to_track(self) -> bool:
        return (
            self.vehicle_local_position is not None
            and self.vehicle_local_position.xy_valid
            and self.vehicle_local_position.z_valid
            and self.vehicle_status is not None
            and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _request_offboard_and_arm_if_needed(self, now_us: int) -> None:
        request_interval_us = 1_000_000

        if (
            self.vehicle_status is None
            or self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            if now_us - self.last_mode_request_us >= request_interval_us:
                self._set_offboard_mode(now_us)
                self.last_mode_request_us = now_us

        if (
            self.vehicle_status is None
            or self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED
        ):
            if now_us - self.last_arm_request_us >= request_interval_us:
                self._arm(now_us)
                self.last_arm_request_us = now_us

    def _advance_waypoint_if_reached(self, now_us: int) -> None:
        if self.current_waypoint_index >= len(self.waypoints):
            self._finish_route(now_us)
            return

        distance = self._distance_to_current_waypoint()
        if distance > self.acceptance_radius_m:
            self.reached_since_us = None
            return

        if self.reached_since_us is None:
            self.reached_since_us = now_us
            waypoint_number = self.current_waypoint_index + 1
            self.get_logger().info(
                f"Reached waypoint {waypoint_number}/{len(self.waypoints)} "
                f"(distance={distance:.2f} m)"
            )
            return

        if now_us - self.reached_since_us < int(self.dwell_time_s * 1_000_000):
            return

        self.current_waypoint_index += 1
        self.reached_since_us = None

        if self.current_waypoint_index >= len(self.waypoints):
            self._finish_route(now_us)
        else:
            waypoint_number = self.current_waypoint_index + 1
            waypoint = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(
                f"Tracking waypoint {waypoint_number}/{len(self.waypoints)}: "
                f"x={waypoint[0]:.1f}, y={waypoint[1]:.1f}, z={waypoint[2]:.1f}"
            )

    def _finish_route(self, now_us: int) -> None:
        if self.land_at_end:
            self.get_logger().info("Route complete; sending land command.")
            self._land(now_us)
        else:
            self.current_waypoint_index = len(self.waypoints) - 1
            self.get_logger().info("Route complete; holding final waypoint.")

    def _distance_to_current_waypoint(self) -> float:
        assert self.vehicle_local_position is not None
        target = self.waypoints[self.current_waypoint_index]
        dx = self.vehicle_local_position.x - target[0]
        dy = self.vehicle_local_position.y - target[1]
        dz = self.vehicle_local_position.z - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _publish_offboard_control_mode(self, now_us: int) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = now_us
        self.offboard_mode_pub.publish(msg)

    def _publish_current_setpoint(self, now_us: int) -> None:
        target_index = min(self.current_waypoint_index, len(self.waypoints) - 1)
        target = self.waypoints[target_index]

        msg = TrajectorySetpoint()
        msg.position = [target[0], target[1], target[2]]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self._target_yaw(target_index)
        msg.yawspeed = math.nan
        msg.timestamp = now_us
        self.trajectory_pub.publish(msg)

    def _publish_current_index(self) -> None:
        msg = Int32()
        msg.data = min(self.current_waypoint_index, len(self.waypoints))
        self.current_index_pub.publish(msg)

    def _target_yaw(self, target_index: int) -> float:
        if self.yaw_mode != "face_next_waypoint":
            return 0.0

        if target_index + 1 < len(self.waypoints):
            start = self.waypoints[target_index]
            end = self.waypoints[target_index + 1]
        elif self.vehicle_local_position is not None:
            start = (
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z,
            )
            end = self.waypoints[target_index]
        else:
            return 0.0

        dx = end[0] - start[0]
        dy = end[1] - start[1]
        if abs(dx) < 1e-3 and abs(dy) < 1e-3:
            return 0.0
        return math.atan2(dy, dx)

    def _set_offboard_mode(self, now_us: int) -> None:
        self._publish_vehicle_command(
            now_us,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info("Requested Offboard mode.")

    def _arm(self, now_us: int) -> None:
        self._publish_vehicle_command(
            now_us,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info("Sent arm command.")

    def _land(self, now_us: int) -> None:
        self._publish_vehicle_command(now_us, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_command_sent = True
        self.last_land_command_us = now_us

    def _publish_vehicle_command(
        self,
        now_us: int,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = now_us
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaypointTracker()

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
