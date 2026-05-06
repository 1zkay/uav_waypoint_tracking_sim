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

from uav_trajectory_tracking.parametric_trajectory import ParametricTrajectory, Point3


STAGE_ENTRY = 0
STAGE_TRAJECTORY = 1
STAGE_RETURN = 2
STAGE_FINISHED = 3
ENTRY_HOLD_S = 0.5


class TrajectoryTracker(Node):
    """Publish PX4 offboard setpoints sampled from a YAML parametric curve."""

    def __init__(self) -> None:
        super().__init__("trajectory_tracker")

        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v4")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("trajectory_stage_topic", "/trajectory_tracker/current_stage")
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("source_system", 1)
        self.declare_parameter("source_component", 1)

        config = self._load_config()
        self.trajectory = ParametricTrajectory.from_config(config)
        self.control_rate_hz = float(config.get("control_rate_hz", 10.0))
        self.takeoff_warmup_s = float(config.get("takeoff_warmup_s", 1.5))
        self.land_at_end = bool(config.get("land_at_end", True))
        self.loop_route = bool(config.get("loop_route", False))
        self.yaw_mode = str(config.get("yaw_mode", "face_velocity"))
        self.entry_point = self.trajectory.sample(0.0)[0]
        self.final_point = self.trajectory.sample(self.trajectory.duration_s)[0]

        self.current_stage_index = STAGE_ENTRY
        self.entry_reached_since_us: int | None = None
        self.trajectory_start_us: int | None = None
        self.returning = False
        self.return_target: Point3 | None = None
        self.return_reached_since_us: int | None = None
        self.finished = False
        self.setpoint_counter = 0
        self.land_command_sent = False
        self.last_land_command_us = 0
        self.last_mode_request_us = 0
        self.last_arm_request_us = 0

        self.vehicle_local_position: VehicleLocalPosition | None = None
        self.vehicle_status: VehicleStatus | None = None

        self.target_system = int(self.get_parameter("target_system").value)
        self.target_component = int(self.get_parameter("target_component").value)
        self.source_system = int(self.get_parameter("source_system").value)
        self.source_component = int(self.get_parameter("source_component").value)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        offboard_control_mode_topic = (
            self.get_parameter("offboard_control_mode_topic").get_parameter_value().string_value
        )
        trajectory_setpoint_topic = (
            self.get_parameter("trajectory_setpoint_topic").get_parameter_value().string_value
        )
        vehicle_command_topic = self.get_parameter("vehicle_command_topic").get_parameter_value().string_value
        trajectory_stage_topic = self.get_parameter("trajectory_stage_topic").get_parameter_value().string_value

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, offboard_control_mode_topic, qos_profile
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, trajectory_setpoint_topic, qos_profile
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, vehicle_command_topic, qos_profile
        )
        tracker_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.trajectory_stage_pub = self.create_publisher(
            Int32, trajectory_stage_topic, tracker_state_qos
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
            "Loaded parametric NED trajectory: "
            f"duration={self.trajectory.duration_s:.2f} s, "
            f"rate={self.control_rate_hz:.1f} Hz, "
            f"entry=({self.entry_point[0]:.2f}, {self.entry_point[1]:.2f}, {self.entry_point[2]:.2f}) m, "
            f"acceptance_radius={self.trajectory.acceptance_radius_m:.2f} m, "
            f"velocity_feedforward={self.trajectory.has_velocity}"
        )
        self.get_logger().info(
            f"Subscribing to state topics: {vehicle_status_topic}, {vehicle_local_position_topic}"
        )
        self.get_logger().info(
            f"Publishing control topics: {offboard_control_mode_topic}, "
            f"{trajectory_setpoint_topic}, {vehicle_command_topic}; target_system={self.target_system}"
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

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _timer_callback(self) -> None:
        now_us = self._now_us()

        if self._ready_to_track() and not self.land_command_sent:
            self._advance_trajectory_if_needed(now_us)

        self._publish_offboard_control_mode(now_us)

        if not self.land_command_sent:
            self._publish_current_setpoint(now_us)

        warmup_cycles = max(1, int(self.takeoff_warmup_s * self.control_rate_hz))
        if self.setpoint_counter >= warmup_cycles and not self.land_command_sent:
            self._request_offboard_and_arm_if_needed(now_us)

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

    def _advance_trajectory_if_needed(self, now_us: int) -> None:
        if self.trajectory_start_us is None:
            self._advance_entry_if_needed(now_us)
            return

        if self.finished:
            return

        if self.returning:
            self._advance_return_if_needed(now_us)
            return

        if self.loop_route and not self.land_at_end:
            return

        if self._elapsed_s(now_us) < self.trajectory.duration_s:
            return

        self.returning = True
        self.return_target = self.trajectory.return_point or self.final_point
        self.return_reached_since_us = None
        self.current_stage_index = STAGE_RETURN
        if self.trajectory.return_point is not None:
            self.get_logger().info("Parametric trajectory complete; flying to return point.")
        else:
            self.get_logger().info("Parametric trajectory complete; settling at final point.")

    def _advance_entry_if_needed(self, now_us: int) -> None:
        distance = self._distance_to_point(self.entry_point)
        if distance > self.trajectory.acceptance_radius_m:
            self.entry_reached_since_us = None
            return

        if self.entry_reached_since_us is None:
            self.entry_reached_since_us = now_us
            self.get_logger().info(f"Reached trajectory entry point (distance={distance:.2f} m).")
            return

        if now_us - self.entry_reached_since_us < int(ENTRY_HOLD_S * 1_000_000):
            return

        self.trajectory_start_us = now_us
        self.current_stage_index = STAGE_TRAJECTORY
        self.get_logger().info("Started parametric trajectory from entry point.")

    def _advance_return_if_needed(self, now_us: int) -> None:
        assert self.return_target is not None
        distance = self._distance_to_point(self.return_target)
        if distance > self.trajectory.acceptance_radius_m:
            self.return_reached_since_us = None
            return

        if self.return_reached_since_us is None:
            self.return_reached_since_us = now_us
            self.get_logger().info(f"Reached return point (distance={distance:.2f} m).")
            return

        self._finish_trajectory(now_us)

    def _finish_trajectory(self, now_us: int) -> None:
        self.finished = True
        self.current_stage_index = STAGE_FINISHED
        if self.land_at_end:
            self.get_logger().info("Trajectory complete; sending land command.")
            self._land(now_us)
        else:
            self.get_logger().info("Trajectory complete; holding final setpoint.")

    def _distance_to_point(self, target: Point3) -> float:
        assert self.vehicle_local_position is not None
        dx = self.vehicle_local_position.x - target[0]
        dy = self.vehicle_local_position.y - target[1]
        dz = self.vehicle_local_position.z - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

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

    def _publish_offboard_control_mode(self, now_us: int) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = (
            self.trajectory.has_velocity
            and self.trajectory_start_us is not None
            and not self.returning
            and not self.finished
        )
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = now_us
        self.offboard_mode_pub.publish(msg)

    def _publish_current_setpoint(self, now_us: int) -> None:
        target, velocity, yaw = self._current_setpoint(now_us)

        msg = TrajectorySetpoint()
        msg.position = [target[0], target[1], target[2]]
        msg.velocity = self._px4_vector_or_nan(velocity)
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = yaw
        msg.yawspeed = math.nan
        msg.timestamp = now_us
        self.trajectory_pub.publish(msg)

    def _current_setpoint(self, now_us: int) -> tuple[Point3, Point3 | None, float]:
        if self.trajectory_start_us is None:
            _, _, configured_yaw = self.trajectory.sample(0.0)
            yaw = configured_yaw if configured_yaw is not None else self._yaw_to_point(self.entry_point)
            return self.entry_point, None, yaw

        if self.returning and self.return_target is not None:
            target = self.return_target
            return target, None, self._yaw_to_point(target)

        position, velocity, configured_yaw = self.trajectory.sample(self._trajectory_time_s(now_us))
        yaw = self._target_yaw(velocity, configured_yaw)
        if self.finished:
            velocity = None
        return position, velocity, yaw

    @staticmethod
    def _px4_vector_or_nan(value: Point3 | None) -> list[float]:
        if value is None:
            return [math.nan, math.nan, math.nan]
        return [value[0], value[1], value[2]]

    def _trajectory_time_s(self, now_us: int) -> float:
        if self.finished:
            return self.trajectory.duration_s
        if self.trajectory_start_us is None:
            return 0.0

        elapsed_s = self._elapsed_s(now_us)
        if self.loop_route and not self.land_at_end:
            return elapsed_s % self.trajectory.duration_s
        return min(elapsed_s, self.trajectory.duration_s)

    def _elapsed_s(self, now_us: int) -> float:
        if self.trajectory_start_us is None:
            return 0.0
        return max(0.0, (now_us - self.trajectory_start_us) * 1e-6)

    def _target_yaw(self, velocity: Point3 | None, configured_yaw: float | None) -> float:
        if configured_yaw is not None:
            return configured_yaw
        if self.yaw_mode != "face_velocity" or velocity is None:
            return 0.0
        vx, vy, _ = velocity
        if abs(vx) < 1e-6 and abs(vy) < 1e-6:
            return 0.0
        return math.atan2(vy, vx)

    def _yaw_to_point(self, target: Point3) -> float:
        if self.yaw_mode != "face_velocity" or self.vehicle_local_position is None:
            return 0.0
        dx = target[0] - self.vehicle_local_position.x
        dy = target[1] - self.vehicle_local_position.y
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0
        return math.atan2(dy, dx)

    def _publish_current_index(self) -> None:
        msg = Int32()
        msg.data = self.current_stage_index
        self.trajectory_stage_pub.publish(msg)

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
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = self.source_system
        msg.source_component = self.source_component
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryTracker()

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
