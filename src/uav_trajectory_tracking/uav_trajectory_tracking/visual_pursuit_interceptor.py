#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
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
    TRANSIT = "transit_to_hover"
    HOLD = "hold"
    ACQUIRING = "acquiring_target"
    COAST = "coast_on_lock_loss"
    PURSUIT = "pursuit"
    TARGET_LOST = "target_lost"


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]
VELOCITY_STATES = {InterceptorState.PURSUIT, InterceptorState.COAST}


@dataclass(frozen=True)
class GimbalCameraKinematics:
    """Body-to-camera optical axis model derived from the Gazebo SDF chain."""

    mount_rpy_rad: Vector3
    yaw_axis: Vector3
    roll_axis: Vector3
    pitch_axis: Vector3
    sensor_rpy_rad: Vector3
    optical_axis_sensor: Vector3


@dataclass(frozen=True)
class TruthState:
    position_ned: Vector3
    velocity_ned: Vector3
    stamp_s: float


@dataclass(frozen=True)
class RelativeTruth:
    position_ned: Vector3
    velocity_ned: Vector3
    target_velocity_ned: Vector3
    range_m: float
    los_ned: Vector3
    closing_speed_mps: float


class VisualPursuitInterceptor(Node):
    """Intercept using gimbal lock as seeker input and PN-style guidance."""

    def __init__(self) -> None:
        super().__init__("visual_pursuit_interceptor")

        self.declare_parameter("config_file", "")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v4")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("vehicle_attitude_topic", "/fmu/out/vehicle_attitude")
        self.declare_parameter("host_truth_odometry_topic", "/model/x500_0/odometry_with_covariance")
        self.declare_parameter("target_truth_odometry_topic", "/model/x500_1/odometry_with_covariance")
        self.declare_parameter("gimbal_joint_state_topic", "/x500_0/gimbal/joint_states")
        self.declare_parameter("tracking_active_topic", "/x500_0/gimbal_target_tracker/tracking_active")
        self.declare_parameter("lock_active_topic", "/x500_0/gimbal_target_tracker/lock_active")
        self.declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("diagnostics_topic", "/x500_0/visual_pursuit_interceptor/diagnostics")
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("source_system", 1)
        self.declare_parameter("source_component", 1)

        config = self._load_config()
        self.control_rate_hz = positive_float(
            config.get("control_rate_hz", 20.0),
            "control_rate_hz",
        )
        self.takeoff_warmup_s = nonnegative_float(
            config.get("takeoff_warmup_s", 1.5),
            "takeoff_warmup_s",
        )
        self.initial_hover_position = parse_point(
            config.get("initial_hover_position_ned", [1.0, 2.0, -5.0]),
            "initial_hover_position_ned",
        )
        self.hover_acceptance_radius_m = positive_float(
            config.get("hover_acceptance_radius_m", 0.3),
            "hover_acceptance_radius_m",
        )
        self.pursuit_speed_mps = positive_float(
            config.get("pursuit_speed_mps", 2.0),
            "pursuit_speed_mps",
        )
        self.max_vertical_speed_mps = positive_float(
            config.get("max_vertical_speed_mps", 1.0),
            "max_vertical_speed_mps",
        )
        self.max_pursuit_accel_mps2 = positive_float(
            config.get("max_pursuit_accel_mps2", 1.0),
            "max_pursuit_accel_mps2",
        )
        self.navigation_gain = positive_float(
            config.get("navigation_gain", 3.0),
            "navigation_gain",
        )
        self.max_guidance_accel_mps2 = nonnegative_float(
            config.get("max_guidance_accel_mps2", self.max_pursuit_accel_mps2),
            "max_guidance_accel_mps2",
        )
        self.max_lateral_guidance_speed_mps = nonnegative_float(
            config.get("max_lateral_guidance_speed_mps", self.pursuit_speed_mps),
            "max_lateral_guidance_speed_mps",
        )
        self.los_rate_filter_alpha = clamp(
            float(config.get("los_rate_filter_alpha", 0.35)),
            0.0,
            1.0,
        )
        self.truth_guidance_enabled = bool(config.get("truth_guidance_enabled", True))
        self.truth_guidance_required = bool(
            config.get("truth_guidance_required", self.truth_guidance_enabled)
        )
        self.truth_odometry_timeout_s = positive_float(
            config.get("truth_odometry_timeout_s", 0.5),
            "truth_odometry_timeout_s",
        )
        self.coast_velocity_decay_s = positive_float(
            config.get("coast_velocity_decay_s", 0.6),
            "coast_velocity_decay_s",
        )
        self.lock_loss_grace_s = nonnegative_float(
            config.get("lock_loss_grace_s", 0.3),
            "lock_loss_grace_s",
        )
        self.tracking_active_timeout_s = positive_float(
            config.get("tracking_active_timeout_s", 0.5),
            "tracking_active_timeout_s",
        )
        self.lock_active_timeout_s = positive_float(
            config.get("lock_active_timeout_s", self.tracking_active_timeout_s),
            "lock_active_timeout_s",
        )
        self.gimbal_feedback_timeout_s = positive_float(
            config.get("gimbal_feedback_timeout_s", 0.5),
            "gimbal_feedback_timeout_s",
        )
        self.hold_position_on_loss = bool(config.get("hold_position_on_loss", True))
        self.yaw_mode = str(config.get("yaw_mode", "face_los")).strip().lower()
        self._validate_yaw_mode()
        self.gimbal_yaw_joint_name = str(
            config.get("gimbal_yaw_joint_name", "cgo3_vertical_arm_joint")
        )
        self.gimbal_pitch_joint_name = str(
            config.get("gimbal_pitch_joint_name", "cgo3_camera_joint")
        )
        self.gimbal_roll_joint_name = str(
            config.get("gimbal_roll_joint_name", "cgo3_horizontal_arm_joint")
        )
        self.gimbal_yaw_sign = float(config.get("gimbal_yaw_sign", 1.0))
        self.gimbal_pitch_sign = float(config.get("gimbal_pitch_sign", 1.0))
        self.gimbal_roll_sign = float(config.get("gimbal_roll_sign", 1.0))
        self.gimbal_kinematics = GimbalCameraKinematics(
            mount_rpy_rad=parse_vector3(
                config.get("gimbal_mount_rpy_rad", [0.0, 0.0, math.pi]),
                "gimbal_mount_rpy_rad",
            ),
            yaw_axis=parse_unit_vector3(
                config.get("gimbal_yaw_axis", [0.0, 0.0, -1.0]),
                "gimbal_yaw_axis",
            ),
            roll_axis=parse_unit_vector3(
                config.get("gimbal_roll_axis", [-1.0, 0.0, 0.0]),
                "gimbal_roll_axis",
            ),
            pitch_axis=parse_unit_vector3(
                config.get("gimbal_pitch_axis", [0.0, 1.0, 0.0]),
                "gimbal_pitch_axis",
            ),
            sensor_rpy_rad=parse_vector3(
                config.get("camera_sensor_rpy_rad", [0.0, 0.0, math.pi]),
                "camera_sensor_rpy_rad",
            ),
            optical_axis_sensor=parse_unit_vector3(
                config.get("camera_optical_axis_sensor", [1.0, 0.0, 0.0]),
                "camera_optical_axis_sensor",
            ),
        )

        self.target_system = int(self.get_parameter("target_system").value)
        self.target_component = int(self.get_parameter("target_component").value)
        self.source_system = int(self.get_parameter("source_system").value)
        self.source_component = int(self.get_parameter("source_component").value)

        self.vehicle_status: VehicleStatus | None = None
        self.vehicle_local_position: VehicleLocalPosition | None = None
        self.vehicle_attitude: VehicleAttitude | None = None
        self.host_truth_state: TruthState | None = None
        self.target_truth_state: TruthState | None = None
        self.tracking_active = False
        self.last_tracking_active_time_s: float | None = None
        self.tracking_true_since_s: float | None = None
        self.last_tracking_true_time_s: float | None = None
        self.lock_active = False
        self.last_lock_active_time_s: float | None = None
        self.lock_true_since_s: float | None = None
        self.last_lock_true_time_s: float | None = None
        self.gimbal_yaw_rad: float | None = None
        self.gimbal_pitch_rad: float | None = None
        self.gimbal_roll_rad: float | None = None
        self.last_gimbal_feedback_time_s: float | None = None
        self.takeoff_position: tuple[float, float, float] | None = None
        self.takeoff_altitude_reached = False
        self.hold_position: tuple[float, float, float] | None = None
        self.initial_hover_reached = False
        self.state = InterceptorState.INITIALIZING
        self.previous_state = InterceptorState.INITIALIZING
        self.setpoint_counter = 0
        self.last_mode_request_us = 0
        self.last_arm_request_us = 0
        self.last_los_body = (1.0, 0.0, 0.0)
        self.last_los_ned = (1.0, 0.0, 0.0)
        self.last_gimbal_los_ned = (1.0, 0.0, 0.0)
        self.last_truth_los_ned: Vector3 | None = None
        self.last_relative_position_ned: Vector3 | None = None
        self.last_relative_velocity_ned: Vector3 | None = None
        self.last_range_m: float | None = None
        self.last_closing_speed_mps: float | None = None
        self.last_guidance_los_ned: Vector3 | None = None
        self.last_los_rate_ned = (0.0, 0.0, 0.0)
        self.last_guidance_accel_ned = (0.0, 0.0, 0.0)
        self.last_lateral_velocity_ned = (0.0, 0.0, 0.0)
        self.last_velocity_ned = (0.0, 0.0, 0.0)
        self.last_control_time_s: float | None = None

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
            Odometry,
            str(self.get_parameter("host_truth_odometry_topic").value),
            self._host_truth_odometry_callback,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("target_truth_odometry_topic").value),
            self._target_truth_odometry_callback,
            sensor_qos,
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
        self.create_subscription(
            Bool,
            str(self.get_parameter("lock_active_topic").value),
            self._lock_active_callback,
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
            f"closing_speed={self.pursuit_speed_mps:.2f} m/s, "
            f"nav_gain={self.navigation_gain:.2f}, "
            f"truth_guidance={self.truth_guidance_enabled}, "
            f"gimbal_joints=({self.gimbal_yaw_joint_name}, {self.gimbal_pitch_joint_name}), "
            f"roll_joint={self.gimbal_roll_joint_name}, "
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

    def _host_truth_odometry_callback(self, msg: Odometry) -> None:
        self.host_truth_state = truth_state_from_odometry(msg, self._now_s())

    def _target_truth_odometry_callback(self, msg: Odometry) -> None:
        self.target_truth_state = truth_state_from_odometry(msg, self._now_s())

    def _tracking_active_callback(self, msg: Bool) -> None:
        now_s = self._now_s()
        self.tracking_active = bool(msg.data)
        self.last_tracking_active_time_s = now_s
        if self.tracking_active:
            if self.tracking_true_since_s is None:
                self.tracking_true_since_s = now_s
            self.last_tracking_true_time_s = now_s
        else:
            self.tracking_true_since_s = None

    def _lock_active_callback(self, msg: Bool) -> None:
        now_s = self._now_s()
        self.lock_active = bool(msg.data)
        self.last_lock_active_time_s = now_s
        if self.lock_active:
            if self.lock_true_since_s is None:
                self.lock_true_since_s = now_s
            self.last_lock_true_time_s = now_s
        else:
            self.lock_true_since_s = None

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

        self.gimbal_yaw_rad = self.gimbal_yaw_sign * float(
            positions[self.gimbal_yaw_joint_name]
        )
        self.gimbal_pitch_rad = self.gimbal_pitch_sign * float(
            positions[self.gimbal_pitch_joint_name]
        )
        self.gimbal_roll_rad = self.gimbal_roll_sign * float(
            positions.get(self.gimbal_roll_joint_name, 0.0)
        )
        self.last_gimbal_feedback_time_s = self._now_s()

    def _timer_callback(self) -> None:
        now_s = self._now_s()
        now_us = int(now_s * 1_000_000)
        dt_s = self._control_dt_s(now_s)
        self.previous_state = self.state
        pursuing = self._ready_to_pursue(now_s)
        velocity_control = pursuing or self.state == InterceptorState.COAST

        self._publish_offboard_control_mode(now_us, velocity_control)
        self._publish_setpoint(now_us, pursuing, velocity_control, dt_s)

        warmup_cycles = max(1, int(self.takeoff_warmup_s * self.control_rate_hz))
        if (
            self.setpoint_counter >= warmup_cycles
            and self._has_setpoint_available()
            and self._can_request_offboard()
        ):
            self._request_offboard_and_arm_if_needed(now_us)

        self._publish_diagnostics(now_us, pursuing, velocity_control)
        self.setpoint_counter += 1

    def _ready_to_pursue(self, now_s: float) -> bool:
        if not self._vehicle_ready():
            self.state = InterceptorState.INITIALIZING
            return False

        if not self.takeoff_altitude_reached:
            self._capture_takeoff_position_if_needed()
            self.state = InterceptorState.TAKEOFF
            if (
                self.takeoff_position is not None
                and abs(
                    float(self.vehicle_local_position.z)
                    - self.takeoff_position[2]
                )
                <= self.hover_acceptance_radius_m
            ):
                self.takeoff_altitude_reached = True
                self.hold_position = self.initial_hover_position
                self.state = InterceptorState.TRANSIT
            return False

        if not self.initial_hover_reached:
            self.state = InterceptorState.TRANSIT
            self.hold_position = self.initial_hover_position
            if (
                self._distance_to(self.initial_hover_position)
                <= self.hover_acceptance_radius_m
            ):
                self.initial_hover_reached = True
                self.state = InterceptorState.HOLD
            return False

        if not self._fresh_gimbal_feedback(now_s):
            self._capture_loss_hold_position_if_needed()
            self.state = InterceptorState.TARGET_LOST
            return False

        if self.truth_guidance_required and not self._fresh_truth_feedback(now_s):
            if self.previous_state in VELOCITY_STATES and self._lock_recently_active(now_s):
                self.state = InterceptorState.COAST
                return False
            self._capture_loss_hold_position_if_needed()
            self.state = (
                InterceptorState.ACQUIRING
                if self._tracking_signal_active(now_s) or self._lock_signal_active(now_s)
                else InterceptorState.TARGET_LOST
            )
            return False

        if self._lock_signal_active(now_s):
            self.state = InterceptorState.PURSUIT
            return True

        if (
            self.previous_state in VELOCITY_STATES
            and self._lock_recently_active(now_s)
        ):
            self.state = InterceptorState.COAST
            return False

        if self._tracking_signal_active(now_s) or self._lock_signal_active(now_s):
            self._capture_loss_hold_position_if_needed()
            self.state = InterceptorState.ACQUIRING
            return False

        self._capture_loss_hold_position_if_needed()
        self.state = InterceptorState.TARGET_LOST
        return False

    def _vehicle_ready(self) -> bool:
        return (
            self.vehicle_status is not None
            and self.vehicle_local_position is not None
            and self.vehicle_local_position.xy_valid
            and self.vehicle_local_position.z_valid
            and self.vehicle_attitude is not None
        )

    def _has_setpoint_available(self) -> bool:
        return self.hold_position is not None or self.state in VELOCITY_STATES

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
        if self.previous_state not in VELOCITY_STATES:
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

    def _capture_takeoff_position_if_needed(self) -> None:
        if self.takeoff_position is not None:
            return
        if (
            self.vehicle_local_position is None
            or not self.vehicle_local_position.xy_valid
            or not self.vehicle_local_position.z_valid
        ):
            return
        self.takeoff_position = (
            float(self.vehicle_local_position.x),
            float(self.vehicle_local_position.y),
            self.initial_hover_position[2],
        )
        self.hold_position = self.takeoff_position

    def _distance_to(self, target: tuple[float, float, float]) -> float:
        assert self.vehicle_local_position is not None
        dx = float(self.vehicle_local_position.x) - target[0]
        dy = float(self.vehicle_local_position.y) - target[1]
        dz = float(self.vehicle_local_position.z) - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _tracking_signal_active(self, now_s: float) -> bool:
        if not self.tracking_active or self.last_tracking_active_time_s is None:
            return False
        return now_s - self.last_tracking_active_time_s <= self.tracking_active_timeout_s

    def _lock_signal_active(self, now_s: float) -> bool:
        if not self.lock_active or self.last_lock_active_time_s is None:
            return False
        return now_s - self.last_lock_active_time_s <= self.lock_active_timeout_s

    def _lock_recently_active(self, now_s: float) -> bool:
        if self.last_lock_true_time_s is None:
            return False
        return now_s - self.last_lock_true_time_s <= self.lock_loss_grace_s

    def _fresh_gimbal_feedback(self, now_s: float) -> bool:
        if (
            self.gimbal_yaw_rad is None
            or self.gimbal_pitch_rad is None
            or self.gimbal_roll_rad is None
            or self.last_gimbal_feedback_time_s is None
        ):
            return False
        return now_s - self.last_gimbal_feedback_time_s <= self.gimbal_feedback_timeout_s

    def _fresh_truth_feedback(self, now_s: float) -> bool:
        if not self.truth_guidance_enabled:
            return True
        if self.host_truth_state is None or self.target_truth_state is None:
            return False
        return (
            now_s - self.host_truth_state.stamp_s <= self.truth_odometry_timeout_s
            and now_s - self.target_truth_state.stamp_s <= self.truth_odometry_timeout_s
        )

    def _publish_offboard_control_mode(self, now_us: int, velocity_control: bool) -> None:
        msg = OffboardControlMode()
        msg.position = not velocity_control
        msg.velocity = velocity_control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = now_us
        self.offboard_mode_pub.publish(msg)

    def _publish_setpoint(
        self,
        now_us: int,
        pursuing: bool,
        velocity_control: bool,
        dt_s: float,
    ) -> None:
        assert self.vehicle_local_position is not None or not pursuing

        if pursuing:
            self._publish_pursuit_setpoint(now_us, dt_s)
            return

        if velocity_control:
            self._publish_coast_setpoint(now_us, dt_s)
            return

        self._reset_guidance_state()
        self._publish_hold_setpoint(now_us)

    def _publish_pursuit_setpoint(self, now_us: int, dt_s: float) -> None:
        assert self.vehicle_local_position is not None
        assert self.vehicle_attitude is not None
        assert self.gimbal_yaw_rad is not None
        assert self.gimbal_pitch_rad is not None
        assert self.gimbal_roll_rad is not None

        los_body = gimbal_angles_to_body_los(
            self.gimbal_yaw_rad,
            self.gimbal_roll_rad,
            self.gimbal_pitch_rad,
            self.gimbal_kinematics,
        )
        los_ned = normalize(
            rotate_body_to_ned(
                tuple(float(value) for value in self.vehicle_attitude.q),
                los_body,
            )
        )
        self.last_gimbal_los_ned = los_ned

        relative_truth = self._relative_truth(now_us * 1e-6)
        if self.truth_guidance_enabled and relative_truth is not None:
            guidance_los_ned = relative_truth.los_ned
            self.last_truth_los_ned = relative_truth.los_ned
            self.last_relative_position_ned = relative_truth.position_ned
            self.last_relative_velocity_ned = relative_truth.velocity_ned
            self.last_range_m = relative_truth.range_m
            self.last_closing_speed_mps = relative_truth.closing_speed_mps
        else:
            guidance_los_ned = los_ned
            relative_truth = None

        target_velocity_ned = self._proportional_navigation_velocity(
            guidance_los_ned,
            dt_s,
            relative_truth,
        )
        velocity_ned = limit_vector_delta(
            self.last_velocity_ned,
            target_velocity_ned,
            self.max_pursuit_accel_mps2 * max(dt_s, 0.0),
        )

        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [velocity_ned[0], velocity_ned[1], velocity_ned[2]]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self._yaw_from_los(guidance_los_ned)
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
        self.last_los_ned = guidance_los_ned
        self.last_velocity_ned = velocity_ned

    def _publish_coast_setpoint(self, now_us: int, dt_s: float) -> None:
        decay = math.exp(-max(dt_s, 0.0) / self.coast_velocity_decay_s)
        velocity_ned = scale(self.last_velocity_ned, decay)
        velocity_ned = limit_vertical_speed(velocity_ned, self.max_vertical_speed_mps)

        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [velocity_ned[0], velocity_ned[1], velocity_ned[2]]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self._yaw_from_los(self.last_los_ned)
        msg.yawspeed = math.nan
        msg.timestamp = now_us
        self.trajectory_pub.publish(msg)
        self.last_velocity_ned = velocity_ned

    def _proportional_navigation_velocity(
        self,
        los_ned: Vector3,
        dt_s: float,
        relative_truth: RelativeTruth | None,
    ) -> Vector3:
        los_rate_ned = self._update_los_rate(los_ned, dt_s, relative_truth)
        closing_speed_mps = (
            max(relative_truth.closing_speed_mps, 0.0)
            if relative_truth is not None
            else self.pursuit_speed_mps
        )
        if relative_truth is None:
            self.last_closing_speed_mps = closing_speed_mps
        guidance_accel_ned = scale(
            cross(los_rate_ned, los_ned),
            self.navigation_gain * closing_speed_mps,
        )
        guidance_accel_ned = limit_vector_norm(
            guidance_accel_ned,
            self.max_guidance_accel_mps2,
        )

        lateral_velocity_ned = add_vectors(
            self.last_lateral_velocity_ned,
            scale(guidance_accel_ned, max(dt_s, 0.0)),
        )
        lateral_velocity_ned = reject_from_axis(lateral_velocity_ned, los_ned)
        lateral_velocity_ned = limit_vector_norm(
            lateral_velocity_ned,
            self.max_lateral_guidance_speed_mps,
        )

        closing_velocity_ned = scale(los_ned, self.pursuit_speed_mps)
        target_velocity_ned = add_vectors(closing_velocity_ned, lateral_velocity_ned)
        if relative_truth is not None:
            target_velocity_ned = add_vectors(
                relative_truth.target_velocity_ned,
                target_velocity_ned,
            )
        target_velocity_ned = limit_vertical_speed(
            target_velocity_ned,
            self.max_vertical_speed_mps,
        )

        self.last_guidance_accel_ned = guidance_accel_ned
        self.last_lateral_velocity_ned = lateral_velocity_ned
        return target_velocity_ned

    def _update_los_rate(
        self,
        los_ned: Vector3,
        dt_s: float,
        relative_truth: RelativeTruth | None,
    ) -> Vector3:
        if self.last_guidance_los_ned is None or dt_s <= 1e-6:
            self.last_guidance_los_ned = los_ned
            self.last_los_rate_ned = (0.0, 0.0, 0.0)
            return self.last_los_rate_ned

        if relative_truth is not None and relative_truth.range_m > 1e-3:
            raw_los_rate_ned = scale(
                cross(los_ned, relative_truth.velocity_ned),
                1.0 / relative_truth.range_m,
            )
        else:
            los_derivative_ned = scale(
                subtract_vectors(los_ned, self.last_guidance_los_ned),
                1.0 / dt_s,
            )
            raw_los_rate_ned = cross(los_ned, los_derivative_ned)
        alpha = self.los_rate_filter_alpha
        self.last_los_rate_ned = add_vectors(
            scale(self.last_los_rate_ned, 1.0 - alpha),
            scale(raw_los_rate_ned, alpha),
        )
        self.last_guidance_los_ned = los_ned
        return self.last_los_rate_ned

    def _relative_truth(self, now_s: float) -> RelativeTruth | None:
        if not self.truth_guidance_enabled:
            return None
        if not self._fresh_truth_feedback(now_s):
            return None

        position_ned = subtract_vectors(
            self.target_truth_state.position_ned,
            self.host_truth_state.position_ned,
        )
        velocity_ned = subtract_vectors(
            self.target_truth_state.velocity_ned,
            self.host_truth_state.velocity_ned,
        )
        range_m = vector_norm(position_ned)
        los_ned = normalize(position_ned)
        closing_speed_mps = -dot(velocity_ned, los_ned)
        return RelativeTruth(
            position_ned=position_ned,
            velocity_ned=velocity_ned,
            target_velocity_ned=self.target_truth_state.velocity_ned,
            range_m=range_m,
            los_ned=los_ned,
            closing_speed_mps=closing_speed_mps,
        )

    def _reset_guidance_state(self) -> None:
        self.last_guidance_los_ned = None
        self.last_los_rate_ned = (0.0, 0.0, 0.0)
        self.last_guidance_accel_ned = (0.0, 0.0, 0.0)
        self.last_lateral_velocity_ned = (0.0, 0.0, 0.0)

    def _publish_hold_setpoint(self, now_us: int) -> None:
        if self.takeoff_position is None:
            self._capture_takeoff_position_if_needed()

        if (
            self.vehicle_local_position is not None
            and self.vehicle_local_position.xy_valid
            and self.vehicle_local_position.z_valid
        ):
            if not self.hold_position_on_loss or self.hold_position is None:
                self.hold_position = (
                    float(self.vehicle_local_position.x),
                    float(self.vehicle_local_position.y),
                    float(self.vehicle_local_position.z),
                )

        if self.hold_position is None:
            return

        self.state = (
            InterceptorState.HOLD
            if self.state == InterceptorState.INITIALIZING
            else self.state
        )
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
        if self.yaw_mode == "fixed_north":
            return 0.0
        if self.yaw_mode != "face_los":
            raise ValueError(f"Unsupported yaw_mode: {self.yaw_mode!r}")
        return math.atan2(los_ned[1], los_ned[0])

    def _gimbal_truth_los_error_deg(self) -> float | None:
        if self.last_truth_los_ned is None:
            return None
        alignment = clamp(dot(self.last_gimbal_los_ned, self.last_truth_los_ned), -1.0, 1.0)
        return math.degrees(math.acos(alignment))

    def _validate_yaw_mode(self) -> None:
        valid_modes = {"fixed_north", "face_los"}
        if self.yaw_mode not in valid_modes:
            raise ValueError(
                f"yaw_mode must be one of {sorted(valid_modes)}; "
                f"got {self.yaw_mode!r}"
            )

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

    def _publish_diagnostics(
        self,
        now_us: int,
        pursuing: bool,
        velocity_control: bool,
    ) -> None:
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
            diagnostic_value("velocity_control_active", velocity_control),
            diagnostic_value("takeoff_altitude_reached", self.takeoff_altitude_reached),
            diagnostic_value("takeoff_x_m", point_value(self.takeoff_position, 0)),
            diagnostic_value("takeoff_y_m", point_value(self.takeoff_position, 1)),
            diagnostic_value("takeoff_z_m", point_value(self.takeoff_position, 2)),
            diagnostic_value("initial_hover_reached", self.initial_hover_reached),
            diagnostic_value("hold_x_m", point_value(self.hold_position, 0)),
            diagnostic_value("hold_y_m", point_value(self.hold_position, 1)),
            diagnostic_value("hold_z_m", point_value(self.hold_position, 2)),
            diagnostic_value("detection_active", self.tracking_active),
            diagnostic_value("lock_active", self.lock_active),
            diagnostic_value(
                "detection_true_duration_s",
                self._detection_true_duration_s(now_us * 1e-6),
            ),
            diagnostic_value(
                "lock_true_duration_s",
                self._lock_true_duration_s(now_us * 1e-6),
            ),
            diagnostic_value(
                "detection_loss_age_s",
                self._detection_loss_age_s(now_us * 1e-6),
            ),
            diagnostic_value("lock_loss_grace_s", self.lock_loss_grace_s),
            diagnostic_value(
                "lock_loss_age_s",
                self._lock_loss_age_s(now_us * 1e-6),
            ),
            diagnostic_value("navigation_gain", self.navigation_gain),
            diagnostic_value("commanded_closing_speed_mps", self.pursuit_speed_mps),
            diagnostic_value("truth_guidance_enabled", self.truth_guidance_enabled),
            diagnostic_value("truth_guidance_required", self.truth_guidance_required),
            diagnostic_value(
                "truth_feedback_fresh",
                self._fresh_truth_feedback(now_us * 1e-6),
            ),
            diagnostic_value("range_m", self.last_range_m),
            diagnostic_value("closing_speed_mps", self.last_closing_speed_mps),
            diagnostic_value(
                "relative_position_ned_x_m",
                vector_value(self.last_relative_position_ned, 0),
            ),
            diagnostic_value(
                "relative_position_ned_y_m",
                vector_value(self.last_relative_position_ned, 1),
            ),
            diagnostic_value(
                "relative_position_ned_z_m",
                vector_value(self.last_relative_position_ned, 2),
            ),
            diagnostic_value(
                "relative_velocity_ned_x_mps",
                vector_value(self.last_relative_velocity_ned, 0),
            ),
            diagnostic_value(
                "relative_velocity_ned_y_mps",
                vector_value(self.last_relative_velocity_ned, 1),
            ),
            diagnostic_value(
                "relative_velocity_ned_z_mps",
                vector_value(self.last_relative_velocity_ned, 2),
            ),
            diagnostic_value("gimbal_yaw_deg", degrees_or_none(self.gimbal_yaw_rad)),
            diagnostic_value("gimbal_pitch_deg", degrees_or_none(self.gimbal_pitch_rad)),
            diagnostic_value("gimbal_roll_deg", degrees_or_none(self.gimbal_roll_rad)),
            diagnostic_value("los_body_x", self.last_los_body[0]),
            diagnostic_value("los_body_y", self.last_los_body[1]),
            diagnostic_value("los_body_z", self.last_los_body[2]),
            diagnostic_value("los_ned_x", self.last_los_ned[0]),
            diagnostic_value("los_ned_y", self.last_los_ned[1]),
            diagnostic_value("los_ned_z", self.last_los_ned[2]),
            diagnostic_value("gimbal_los_ned_x", self.last_gimbal_los_ned[0]),
            diagnostic_value("gimbal_los_ned_y", self.last_gimbal_los_ned[1]),
            diagnostic_value("gimbal_los_ned_z", self.last_gimbal_los_ned[2]),
            diagnostic_value("truth_los_ned_x", vector_value(self.last_truth_los_ned, 0)),
            diagnostic_value("truth_los_ned_y", vector_value(self.last_truth_los_ned, 1)),
            diagnostic_value("truth_los_ned_z", vector_value(self.last_truth_los_ned, 2)),
            diagnostic_value("gimbal_truth_los_error_deg", self._gimbal_truth_los_error_deg()),
            diagnostic_value("los_rate_ned_x_rad_s", self.last_los_rate_ned[0]),
            diagnostic_value("los_rate_ned_y_rad_s", self.last_los_rate_ned[1]),
            diagnostic_value("los_rate_ned_z_rad_s", self.last_los_rate_ned[2]),
            diagnostic_value(
                "guidance_accel_ned_x_mps2",
                self.last_guidance_accel_ned[0],
            ),
            diagnostic_value(
                "guidance_accel_ned_y_mps2",
                self.last_guidance_accel_ned[1],
            ),
            diagnostic_value(
                "guidance_accel_ned_z_mps2",
                self.last_guidance_accel_ned[2],
            ),
            diagnostic_value(
                "lateral_velocity_ned_x_mps",
                self.last_lateral_velocity_ned[0],
            ),
            diagnostic_value(
                "lateral_velocity_ned_y_mps",
                self.last_lateral_velocity_ned[1],
            ),
            diagnostic_value(
                "lateral_velocity_ned_z_mps",
                self.last_lateral_velocity_ned[2],
            ),
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

    def _control_dt_s(self, now_s: float) -> float:
        if self.last_control_time_s is None:
            self.last_control_time_s = now_s
            return 1.0 / max(self.control_rate_hz, 1.0)

        dt_s = max(0.0, now_s - self.last_control_time_s)
        self.last_control_time_s = now_s
        return min(dt_s, 0.25)

    def _detection_true_duration_s(self, now_s: float) -> float | None:
        if (
            not self._tracking_signal_active(now_s)
            or self.tracking_true_since_s is None
        ):
            return None
        return max(0.0, now_s - self.tracking_true_since_s)

    def _lock_true_duration_s(self, now_s: float) -> float | None:
        if not self._lock_signal_active(now_s) or self.lock_true_since_s is None:
            return None
        return max(0.0, now_s - self.lock_true_since_s)

    def _detection_loss_age_s(self, now_s: float) -> float | None:
        if self.tracking_active or self.last_tracking_true_time_s is None:
            return None
        return max(0.0, now_s - self.last_tracking_true_time_s)

    def _lock_loss_age_s(self, now_s: float) -> float | None:
        if self.lock_active or self.last_lock_true_time_s is None:
            return None
        return max(0.0, now_s - self.last_lock_true_time_s)


def gimbal_angles_to_body_los(
    yaw_rad: float,
    roll_rad: float,
    pitch_rad: float,
    kinematics: GimbalCameraKinematics,
) -> tuple[float, float, float]:
    rotation_gazebo_base_to_sensor = matmul3(
        rotation_from_rpy(*kinematics.mount_rpy_rad),
        matmul3(
            rotation_around_axis(kinematics.yaw_axis, yaw_rad),
            matmul3(
                rotation_around_axis(kinematics.roll_axis, roll_rad),
                matmul3(
                    rotation_around_axis(kinematics.pitch_axis, pitch_rad),
                    rotation_from_rpy(*kinematics.sensor_rpy_rad),
                ),
            ),
        ),
    )
    los_gazebo_base = matvec3(
        rotation_gazebo_base_to_sensor,
        kinematics.optical_axis_sensor,
    )
    return normalize(gazebo_flu_to_px4_frd(los_gazebo_base))


def gazebo_flu_to_px4_frd(vector: Vector3) -> Vector3:
    return (vector[0], -vector[1], -vector[2])


def truth_state_from_odometry(msg: Odometry, fallback_time_s: float) -> TruthState:
    pose = msg.pose.pose
    twist = msg.twist.twist
    return TruthState(
        position_ned=(
            float(pose.position.y),
            float(pose.position.x),
            -float(pose.position.z),
        ),
        velocity_ned=(
            float(twist.linear.y),
            float(twist.linear.x),
            -float(twist.linear.z),
        ),
        stamp_s=fallback_time_s,
    )


def rotation_from_rpy(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> Matrix3:
    return matmul3(
        rotation_z(yaw_rad),
        matmul3(rotation_y(pitch_rad), rotation_x(roll_rad)),
    )


def rotation_x(angle_rad: float) -> Matrix3:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rotation_y(angle_rad: float) -> Matrix3:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rotation_z(angle_rad: float) -> Matrix3:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def rotation_around_axis(axis: Vector3, angle_rad: float) -> Matrix3:
    ux, uy, uz = normalize(axis)
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    one_minus_c = 1.0 - c
    return (
        (
            c + ux * ux * one_minus_c,
            ux * uy * one_minus_c - uz * s,
            ux * uz * one_minus_c + uy * s,
        ),
        (
            uy * ux * one_minus_c + uz * s,
            c + uy * uy * one_minus_c,
            uy * uz * one_minus_c - ux * s,
        ),
        (
            uz * ux * one_minus_c - uy * s,
            uz * uy * one_minus_c + ux * s,
            c + uz * uz * one_minus_c,
        ),
    )


def matmul3(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][col] for index in range(3))
            for col in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def matvec3(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][col] * vector[col] for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


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


def limit_vertical_speed(
    vector: tuple[float, float, float],
    max_vertical_speed: float,
) -> tuple[float, float, float]:
    if abs(vector[2]) <= max_vertical_speed:
        return vector
    return (vector[0], vector[1], math.copysign(max_vertical_speed, vector[2]))


def limit_vector_norm(vector: Vector3, max_norm: float) -> Vector3:
    if max_norm <= 0.0:
        return (0.0, 0.0, 0.0)
    norm = vector_norm(vector)
    if norm <= max_norm or norm <= 1e-9:
        return vector
    return scale(vector, max_norm / norm)


def limit_vector_delta(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    max_delta: float,
) -> tuple[float, float, float]:
    if max_delta <= 0.0:
        return current

    delta = (
        target[0] - current[0],
        target[1] - current[1],
        target[2] - current[2],
    )
    delta_norm = math.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)
    if delta_norm <= max_delta or delta_norm <= 1e-9:
        return target

    ratio = max_delta / delta_norm
    return (
        current[0] + delta[0] * ratio,
        current[1] + delta[1] * ratio,
        current[2] + delta[2] * ratio,
    )


def normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = vector_norm(vector)
    if norm <= 1e-9:
        return (1.0, 0.0, 0.0)
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def vector_norm(vector: Vector3) -> float:
    return math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)


def dot(left: Vector3, right: Vector3) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def add_vectors(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def subtract_vectors(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def reject_from_axis(vector: Vector3, axis: Vector3) -> Vector3:
    unit_axis = normalize(axis)
    return subtract_vectors(vector, scale(unit_axis, dot(vector, unit_axis)))


def scale(vector: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    return (vector[0] * scalar, vector[1] * scalar, vector[2] * scalar)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


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


def parse_vector3(value: Any, name: str) -> Vector3:
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{name} must be [x, y, z].")
    return (float(value[0]), float(value[1]), float(value[2]))


def parse_unit_vector3(value: Any, name: str) -> Vector3:
    vector = parse_vector3(value, name)
    norm = vector_norm(vector)
    if norm <= 1e-9:
        raise ValueError(f"{name} must not be a zero vector.")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def degrees_or_none(value: float | None) -> float | None:
    return None if value is None else math.degrees(value)


def point_value(point: tuple[float, float, float] | None, index: int) -> float | None:
    return None if point is None else point[index]


def vector_value(vector: Vector3 | None, index: int) -> float | None:
    return None if vector is None else vector[index]


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
