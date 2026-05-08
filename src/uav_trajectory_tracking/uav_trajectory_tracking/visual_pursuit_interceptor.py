#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class InterceptorState:
    INITIALIZING = "initializing"
    TAKEOFF = "takeoff"
    HOLD = "hold"
    PURSUIT = "pursuit"
    TARGET_LOST = "target_lost"


class VisualPursuitInterceptor(Node):
    """Drive the host UAV along the gimbal camera optical axis."""

    def __init__(self) -> None:
        super().__init__("visual_pursuit_interceptor")

        self.declare_parameter("config_file", "")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v4")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("vehicle_attitude_topic", "/fmu/out/vehicle_attitude")
        self.declare_parameter("gimbal_joint_state_topic", "/x500_0/gimbal/joint_states")
        self.declare_parameter("tracking_active_topic", "/x500_0/gimbal_target_tracker/tracking_active")
        self.declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("diagnostics_topic", "/x500_0/visual_pursuit_interceptor/diagnostics")
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("source_system", 1)
        self.declare_parameter("source_component", 1)

        config = self._load_config()
        self.control_rate_hz = positive_float(config.get("control_rate_hz", 20.0), "control_rate_hz")
        self.takeoff_warmup_s = nonnegative_float(config.get("takeoff_warmup_s", 1.5), "takeoff_warmup_s")
        self.initial_hover_position = parse_point(
            config.get("initial_hover_position_ned", [1.0, 2.0, -5.0]),
            "initial_hover_position_ned",
        )
        self.hover_acceptance_radius_m = positive_float(
            config.get("hover_acceptance_radius_m", 0.3),
            "hover_acceptance_radius_m",
        )
        self.pursuit_speed_mps = positive_float(config.get("pursuit_speed_mps", 2.0), "pursuit_speed_mps")
        self.max_vertical_speed_mps = positive_float(
            config.get("max_vertical_speed_mps", 1.0),
            "max_vertical_speed_mps",
        )
        self.tracking_active_timeout_s = positive_float(
            config.get("tracking_active_timeout_s", 0.5),
            "tracking_active_timeout_s",
        )
        self.gimbal_feedback_timeout_s = positive_float(
            config.get("gimbal_feedback_timeout_s", 0.5),
            "gimbal_feedback_timeout_s",
        )
        self.hold_position_on_loss = bool(config.get("hold_position_on_loss", True))
        self.yaw_mode = str(config.get("yaw_mode", "face_los")).strip().lower()
        self.gimbal_yaw_joint_name = str(
            config.get("gimbal_yaw_joint_name", "cgo3_vertical_arm_joint")
        )
        self.gimbal_pitch_joint_name = str(
            config.get("gimbal_pitch_joint_name", "cgo3_camera_joint")
        )
        self.gimbal_yaw_sign = float(config.get("gimbal_yaw_sign", 1.0))
        self.gimbal_pitch_sign = float(config.get("gimbal_pitch_sign", 1.0))
        self.min_forward_component = float(config.get("min_forward_component", 0.05))

        self.target_system = int(self.get_parameter("target_system").value)
        self.target_component = int(self.get_parameter("target_component").value)
        self.source_system = int(self.get_parameter("source_system").value)
        self.source_component = int(self.get_parameter("source_component").value)

        self.vehicle_status: VehicleStatus | None = None
        self.vehicle_local_position: VehicleLocalPosition | None = None
        self.vehicle_attitude: VehicleAttitude | None = None
        self.tracking_active = False
        self.last_tracking_active_time_s: float | None = None
        self.gimbal_yaw_rad: float | None = None
        self.gimbal_pitch_rad: float | None = None
        self.last_gimbal_feedback_time_s: float | None = None
        self.hold_position: tuple[float, float, float] | None = self.initial_hover_position
        self.initial_hover_reached = False
        self.state = InterceptorState.INITIALIZING
        self.previous_state = InterceptorState.INITIALIZING
        self.setpoint_counter = 0
        self.last_mode_request_us = 0
        self.last_arm_request_us = 0
        self.last_los_body = (1.0, 0.0, 0.0)
        self.last_los_ned = (1.0, 0.0, 0.0)
        self.last_velocity_ned = (0.0, 0.0, 0.0)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            str(self.get_parameter("offboard_control_mode_topic").value),
            px4_qos,
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            str(self.get_parameter("trajectory_setpoint_topic").value),
            px4_qos,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            str(self.get_parameter("vehicle_command_topic").value),
            px4_qos,
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )

        self.create_subscription(
            VehicleStatus,
            str(self.get_parameter("vehicle_status_topic").value),
            self._vehicle_status_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            str(self.get_parameter("vehicle_local_position_topic").value),
            self._vehicle_local_position_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleAttitude,
            str(self.get_parameter("vehicle_attitude_topic").value),
            self._vehicle_attitude_callback,
            px4_qos,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("gimbal_joint_state_topic").value),
            self._gimbal_joint_state_callback,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("tracking_active_topic").value),
            self._tracking_active_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0),
            self._timer_callback,
        )

        self.get_logger().info(
            "Visual pursuit interceptor ready: "
            f"rate={self.control_rate_hz:.1f} Hz, "
            f"initial_hover=({self.initial_hover_position[0]:.2f}, "
            f"{self.initial_hover_position[1]:.2f}, "
            f"{self.initial_hover_position[2]:.2f}) m NED, "
            f"speed={self.pursuit_speed_mps:.2f} m/s, "
            f"gimbal_joints=({self.gimbal_yaw_joint_name}, {self.gimbal_pitch_joint_name}), "
            f"target_system={self.target_system}"
        )

    def _load_config(self) -> dict[str, Any]:
        config_path = str(self.get_parameter("config_file").value)
        if not config_path:
            share_dir = Path(get_package_share_directory("uav_trajectory_tracking"))
            config_path = str(share_dir / "config" / "visual_interception.yaml")

        path = Path(config_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Visual interception config does not exist: {path}")

        with path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        if str(config.get("frame", "NED")).upper() != "NED":
            raise ValueError("Only PX4 local NED interception configs are supported.")

        return config

    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def _vehicle_attitude_callback(self, msg: VehicleAttitude) -> None:
        self.vehicle_attitude = msg

    def _tracking_active_callback(self, msg: Bool) -> None:
        self.tracking_active = bool(msg.data)
        self.last_tracking_active_time_s = self._now_s()

    def _gimbal_joint_state_callback(self, msg: JointState) -> None:
        positions = {
            name: position
            for name, position in zip(msg.name, msg.position, strict=False)
        }
        if (
            self.gimbal_yaw_joint_name not in positions
            or self.gimbal_pitch_joint_name not in positions
        ):
            return

        self.gimbal_yaw_rad = self.gimbal_yaw_sign * float(positions[self.gimbal_yaw_joint_name])
        self.gimbal_pitch_rad = self.gimbal_pitch_sign * float(positions[self.gimbal_pitch_joint_name])
        self.last_gimbal_feedback_time_s = self._now_s()

    def _timer_callback(self) -> None:
        now_us = self._now_us()
        self.previous_state = self.state
        pursuing = self._ready_to_pursue()

        self._publish_offboard_control_mode(now_us, pursuing)
        self._publish_setpoint(now_us, pursuing)

        warmup_cycles = max(1, int(self.takeoff_warmup_s * self.control_rate_hz))
        if (
            self.setpoint_counter >= warmup_cycles
            and self._has_setpoint_available()
            and self._can_request_offboard()
        ):
            self._request_offboard_and_arm_if_needed(now_us)

        self._publish_diagnostics(now_us, pursuing)
        self.setpoint_counter += 1

    def _ready_to_pursue(self) -> bool:
        if not self._vehicle_ready():
            self.state = InterceptorState.INITIALIZING
            return False

        if not self.initial_hover_reached:
            self.state = InterceptorState.TAKEOFF
            if self._distance_to(self.initial_hover_position) <= self.hover_acceptance_radius_m:
                self.initial_hover_reached = True
                self.state = InterceptorState.HOLD
            return False

        if not self._fresh_tracking_active():
            self._capture_loss_hold_position_if_needed()
            self.state = InterceptorState.TARGET_LOST
            return False

        if not self._fresh_gimbal_feedback():
            self._capture_loss_hold_position_if_needed()
            self.state = InterceptorState.TARGET_LOST
            return False

        self.state = InterceptorState.PURSUIT
        return True

    def _vehicle_ready(self) -> bool:
        return (
            self.vehicle_status is not None
            and self.vehicle_local_position is not None
            and self.vehicle_local_position.xy_valid
            and self.vehicle_local_position.z_valid
            and self.vehicle_attitude is not None
        )

    def _has_setpoint_available(self) -> bool:
        return self.hold_position is not None or self.state == InterceptorState.PURSUIT

    def _can_request_offboard(self) -> bool:
        return (
            self.vehicle_status is not None
            and self.vehicle_local_position is not None
            and self.vehicle_local_position.xy_valid
            and self.vehicle_local_position.z_valid
        )

    def _capture_loss_hold_position_if_needed(self) -> None:
        if not self.hold_position_on_loss:
            return
        if self.previous_state != InterceptorState.PURSUIT:
            return
        if (
            self.vehicle_local_position is None
            or not self.vehicle_local_position.xy_valid
            or not self.vehicle_local_position.z_valid
        ):
            return
        self.hold_position = (
            float(self.vehicle_local_position.x),
            float(self.vehicle_local_position.y),
            float(self.vehicle_local_position.z),
        )

    def _distance_to(self, target: tuple[float, float, float]) -> float:
        assert self.vehicle_local_position is not None
        dx = float(self.vehicle_local_position.x) - target[0]
        dy = float(self.vehicle_local_position.y) - target[1]
        dz = float(self.vehicle_local_position.z) - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _fresh_tracking_active(self) -> bool:
        if not self.tracking_active or self.last_tracking_active_time_s is None:
            return False
        return self._now_s() - self.last_tracking_active_time_s <= self.tracking_active_timeout_s

    def _fresh_gimbal_feedback(self) -> bool:
        if (
            self.gimbal_yaw_rad is None
            or self.gimbal_pitch_rad is None
            or self.last_gimbal_feedback_time_s is None
        ):
            return False
        return self._now_s() - self.last_gimbal_feedback_time_s <= self.gimbal_feedback_timeout_s

    def _publish_offboard_control_mode(self, now_us: int, pursuing: bool) -> None:
        msg = OffboardControlMode()
        msg.position = not pursuing
        msg.velocity = pursuing
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = now_us
        self.offboard_mode_pub.publish(msg)

    def _publish_setpoint(self, now_us: int, pursuing: bool) -> None:
        assert self.vehicle_local_position is not None or not pursuing

        if pursuing:
            self._publish_pursuit_setpoint(now_us)
            return

        self._publish_hold_setpoint(now_us)

    def _publish_pursuit_setpoint(self, now_us: int) -> None:
        assert self.vehicle_local_position is not None
        assert self.vehicle_attitude is not None
        assert self.gimbal_yaw_rad is not None
        assert self.gimbal_pitch_rad is not None

        los_body = gimbal_angles_to_body_los(self.gimbal_yaw_rad, self.gimbal_pitch_rad)
        los_ned = normalize(rotate_body_to_ned(tuple(float(v) for v in self.vehicle_attitude.q), los_body))
        los_ned = enforce_forward_component(los_ned, self.min_forward_component)
        velocity_ned = limit_vertical_speed(
            scale(los_ned, self.pursuit_speed_mps),
            self.max_vertical_speed_mps,
        )

        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [velocity_ned[0], velocity_ned[1], velocity_ned[2]]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self._yaw_from_los(los_ned)
        msg.yawspeed = math.nan
        msg.timestamp = now_us
        self.trajectory_pub.publish(msg)

        if not self.hold_position_on_loss:
            self.hold_position = (
                float(self.vehicle_local_position.x),
                float(self.vehicle_local_position.y),
                float(self.vehicle_local_position.z),
            )
        self.last_los_body = los_body
        self.last_los_ned = los_ned
        self.last_velocity_ned = velocity_ned

    def _publish_hold_setpoint(self, now_us: int) -> None:
        if self.vehicle_local_position is not None and self.vehicle_local_position.xy_valid and self.vehicle_local_position.z_valid:
            if not self.hold_position_on_loss or self.hold_position is None:
                self.hold_position = (
                    float(self.vehicle_local_position.x),
                    float(self.vehicle_local_position.y),
                    float(self.vehicle_local_position.z),
                )

        if self.hold_position is None:
            return

        self.state = InterceptorState.HOLD if self.state == InterceptorState.INITIALIZING else self.state
        msg = TrajectorySetpoint()
        msg.position = [self.hold_position[0], self.hold_position[1], self.hold_position[2]]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self._yaw_from_los(self.last_los_ned)
        msg.yawspeed = math.nan
        msg.timestamp = now_us
        self.trajectory_pub.publish(msg)
        self.last_velocity_ned = (0.0, 0.0, 0.0)

    def _yaw_from_los(self, los_ned: tuple[float, float, float]) -> float:
        if self.yaw_mode != "face_los":
            return 0.0
        return math.atan2(los_ned[1], los_ned[0])

    def _request_offboard_and_arm_if_needed(self, now_us: int) -> None:
        request_interval_us = 1_000_000

        if (
            self.vehicle_status is None
            or self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            if now_us - self.last_mode_request_us >= request_interval_us:
                self._publish_vehicle_command(
                    now_us,
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0,
                )
                self.last_mode_request_us = now_us
                self.get_logger().info("Requested Offboard mode.")

        if (
            self.vehicle_status is None
            or self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED
        ):
            if now_us - self.last_arm_request_us >= request_interval_us:
                self._publish_vehicle_command(
                    now_us,
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,
                )
                self.last_arm_request_us = now_us
                self.get_logger().info("Sent arm command.")

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

    def _publish_diagnostics(self, now_us: int, pursuing: bool) -> None:
        msg = DiagnosticArray()
        msg.header.stamp.sec = int(now_us // 1_000_000)
        msg.header.stamp.nanosec = int((now_us % 1_000_000) * 1000)

        status = DiagnosticStatus()
        status.name = "visual_pursuit_interceptor"
        status.hardware_id = f"target_system={self.target_system}"
        status.level = DiagnosticStatus.OK if pursuing else DiagnosticStatus.WARN
        status.message = self.state
        status.values = [
            diagnostic_value("state", self.state),
            diagnostic_value("pursuing", pursuing),
            diagnostic_value("initial_hover_reached", self.initial_hover_reached),
            diagnostic_value("tracking_active", self.tracking_active),
            diagnostic_value("gimbal_yaw_deg", degrees_or_none(self.gimbal_yaw_rad)),
            diagnostic_value("gimbal_pitch_deg", degrees_or_none(self.gimbal_pitch_rad)),
            diagnostic_value("los_body_x", self.last_los_body[0]),
            diagnostic_value("los_body_y", self.last_los_body[1]),
            diagnostic_value("los_body_z", self.last_los_body[2]),
            diagnostic_value("los_ned_x", self.last_los_ned[0]),
            diagnostic_value("los_ned_y", self.last_los_ned[1]),
            diagnostic_value("los_ned_z", self.last_los_ned[2]),
            diagnostic_value("velocity_ned_x_mps", self.last_velocity_ned[0]),
            diagnostic_value("velocity_ned_y_mps", self.last_velocity_ned[1]),
            diagnostic_value("velocity_ned_z_mps", self.last_velocity_ned[2]),
        ]
        msg.status.append(status)
        self.diagnostics_pub.publish(msg)

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def gimbal_angles_to_body_los(
    yaw_rad: float,
    pitch_rad: float,
) -> tuple[float, float, float]:
    return normalize(
        (
            math.cos(pitch_rad) * math.cos(yaw_rad),
            math.cos(pitch_rad) * math.sin(yaw_rad),
            -math.sin(pitch_rad),
        )
    )


def rotate_body_to_ned(
    q_body_to_ned: tuple[float, float, float, float],
    vector_body: tuple[float, float, float],
) -> tuple[float, float, float]:
    w, x, y, z = q_body_to_ned
    vx, vy, vz = vector_body

    # q * [0, v] * q_conjugate
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def enforce_forward_component(
    vector: tuple[float, float, float],
    min_forward_component: float,
) -> tuple[float, float, float]:
    if min_forward_component <= 0.0 or vector[0] >= min_forward_component:
        return vector
    adjusted = (min_forward_component, vector[1], vector[2])
    return normalize(adjusted)


def limit_vertical_speed(
    vector: tuple[float, float, float],
    max_vertical_speed: float,
) -> tuple[float, float, float]:
    if abs(vector[2]) <= max_vertical_speed:
        return vector
    return (vector[0], vector[1], math.copysign(max_vertical_speed, vector[2]))


def normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    if norm <= 1e-9:
        return (1.0, 0.0, 0.0)
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def scale(vector: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    return (vector[0] * scalar, vector[1] * scalar, vector[2] * scalar)


def positive_float(value: Any, name: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise ValueError(f"{name} must be positive.")
    return number


def nonnegative_float(value: Any, name: str) -> float:
    number = float(value)
    if number < 0.0:
        raise ValueError(f"{name} must be non-negative.")
    return number


def parse_point(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{name} must be [x, y, z].")
    return (float(value[0]), float(value[1]), float(value[2]))


def degrees_or_none(value: float | None) -> float | None:
    return None if value is None else math.degrees(value)


def diagnostic_value(key: str, value: bool | float | str | None) -> KeyValue:
    item = KeyValue()
    item.key = key
    if isinstance(value, bool):
        item.value = str(value).lower()
    elif value is None:
        item.value = "nan"
    elif isinstance(value, float):
        item.value = f"{value:.6g}"
    else:
        item.value = str(value)
    return item


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisualPursuitInterceptor()

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
