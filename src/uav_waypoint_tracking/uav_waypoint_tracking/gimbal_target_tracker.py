#!/usr/bin/env python3
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import (
    GimbalDeviceAttitudeStatus,
    GimbalManagerSetAttitude,
    VehicleCommand,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray


@dataclass
class SelectedDetection:
    """Detection selected for the gimbal visual-servo loop."""

    track_id: str
    class_id: str
    score: float
    center_x: float
    center_y: float


@dataclass
class CameraIntrinsics:
    """Pinhole camera intrinsics in pixel units."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: float
    height: float
    source: str


class GimbalTargetTracker(Node):
    """Keep a detected target near the gimbal camera image center."""

    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000
    MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001
    GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16
    GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME = 32
    GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME = 64
    COMMAND_INTERFACE_GIMBAL_MANAGER_SET_ATTITUDE = "gimbal_manager_set_attitude"
    COMMAND_INTERFACE_VEHICLE_COMMAND = "vehicle_command"

    def __init__(self) -> None:
        super().__init__("gimbal_target_tracker")

        self.declare_parameter("detections_topic", "/x500_0/yolo/tracks")
        self.declare_parameter("camera_info_topic", "/x500_0/camera/camera_info")
        self.declare_parameter(
            "gimbal_attitude_topic",
            "/fmu/out/gimbal_device_attitude_status",
        )
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter(
            "gimbal_set_attitude_topic",
            "/fmu/in/gimbal_manager_set_attitude",
        )
        self.declare_parameter("error_topic", "/x500_0/gimbal_target_tracker/error")
        self.declare_parameter(
            "tracking_active_topic",
            "/x500_0/gimbal_target_tracker/tracking_active",
        )

        self.declare_parameter("target_class_id", "")
        self.declare_parameter("target_track_id", "")
        self.declare_parameter("lock_target_track", True)
        self.declare_parameter("min_score", 0.35)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("lost_timeout_s", 0.3)
        self.declare_parameter("fallback_fx_px", 410.93927419797166)
        self.declare_parameter("fallback_fy_px", 410.93927419797166)
        self.declare_parameter("fallback_cx_px", 640.0)
        self.declare_parameter("fallback_cy_px", 360.0)
        self.declare_parameter("fallback_image_width", 1280.0)
        self.declare_parameter("fallback_image_height", 720.0)
        self.declare_parameter("deadband_angle_deg", 1.5)

        self.declare_parameter("yaw_rate_gain_s_inv", 0.8)
        self.declare_parameter("pitch_rate_gain_s_inv", 0.85)
        self.declare_parameter("max_yaw_rate_deg_s", 60.0)
        self.declare_parameter("max_pitch_rate_deg_s", 45.0)
        self.declare_parameter("yaw_error_sign", 1.0)
        self.declare_parameter("pitch_error_sign", -1.0)

        self.declare_parameter("initial_yaw_deg", 0.0)
        self.declare_parameter("initial_pitch_deg", 0.0)
        self.declare_parameter("min_yaw_deg", -90.0)
        self.declare_parameter("max_yaw_deg", 90.0)
        self.declare_parameter("min_pitch_deg", -90.0)
        self.declare_parameter("max_pitch_deg", 30.0)

        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("source_system", 1)
        self.declare_parameter("source_component", 1)
        self.declare_parameter("gimbal_device_id", 0.0)
        self.declare_parameter("yaw_frame", "vehicle")
        self.declare_parameter(
            "command_interface",
            self.COMMAND_INTERFACE_GIMBAL_MANAGER_SET_ATTITUDE,
        )

        self.declare_parameter("hold_last_command_on_loss", True)
        self.declare_parameter("send_command_before_first_detection", False)
        self.declare_parameter("use_gimbal_feedback", True)
        self.declare_parameter("configure_gimbal_manager", True)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.gimbal_attitude_topic = str(
            self.get_parameter("gimbal_attitude_topic").value
        )
        self.vehicle_command_topic = str(
            self.get_parameter("vehicle_command_topic").value
        )
        self.gimbal_set_attitude_topic = str(
            self.get_parameter("gimbal_set_attitude_topic").value
        )
        self.error_topic = str(self.get_parameter("error_topic").value)
        self.tracking_active_topic = str(
            self.get_parameter("tracking_active_topic").value
        )

        self.target_class_id = str(self.get_parameter("target_class_id").value).strip()
        self.target_track_id = str(self.get_parameter("target_track_id").value).strip()
        self.lock_target_track = bool(self.get_parameter("lock_target_track").value)
        self.min_score = float(self.get_parameter("min_score").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.lost_timeout_s = float(self.get_parameter("lost_timeout_s").value)
        self.fallback_fx_px = float(self.get_parameter("fallback_fx_px").value)
        self.fallback_fy_px = float(self.get_parameter("fallback_fy_px").value)
        self.fallback_cx_px = float(self.get_parameter("fallback_cx_px").value)
        self.fallback_cy_px = float(self.get_parameter("fallback_cy_px").value)
        self.fallback_image_width = float(
            self.get_parameter("fallback_image_width").value
        )
        self.fallback_image_height = float(
            self.get_parameter("fallback_image_height").value
        )
        self.deadband_angle_deg = float(
            self.get_parameter("deadband_angle_deg").value
        )

        self.yaw_rate_gain_s_inv = float(
            self.get_parameter("yaw_rate_gain_s_inv").value
        )
        self.pitch_rate_gain_s_inv = float(
            self.get_parameter("pitch_rate_gain_s_inv").value
        )
        self.max_yaw_rate_deg_s = float(
            self.get_parameter("max_yaw_rate_deg_s").value
        )
        self.max_pitch_rate_deg_s = float(
            self.get_parameter("max_pitch_rate_deg_s").value
        )
        self.yaw_error_sign = float(self.get_parameter("yaw_error_sign").value)
        self.pitch_error_sign = float(self.get_parameter("pitch_error_sign").value)

        self.yaw_deg = float(self.get_parameter("initial_yaw_deg").value)
        self.pitch_deg = float(self.get_parameter("initial_pitch_deg").value)
        self.min_yaw_deg = float(self.get_parameter("min_yaw_deg").value)
        self.max_yaw_deg = float(self.get_parameter("max_yaw_deg").value)
        self.min_pitch_deg = float(self.get_parameter("min_pitch_deg").value)
        self.max_pitch_deg = float(self.get_parameter("max_pitch_deg").value)

        self.target_system = int(self.get_parameter("target_system").value)
        self.target_component = int(self.get_parameter("target_component").value)
        self.source_system = int(self.get_parameter("source_system").value)
        self.source_component = int(self.get_parameter("source_component").value)
        self.gimbal_device_id = float(self.get_parameter("gimbal_device_id").value)
        self.yaw_frame = str(self.get_parameter("yaw_frame").value).strip().lower()
        self.gimbal_manager_flags = float(self._gimbal_manager_flags())
        self.command_interface = (
            str(self.get_parameter("command_interface").value).strip().lower()
        )
        self._validate_command_interface()

        self.hold_last_command_on_loss = bool(
            self.get_parameter("hold_last_command_on_loss").value
        )
        self.send_command_before_first_detection = bool(
            self.get_parameter("send_command_before_first_detection").value
        )
        self.use_gimbal_feedback = bool(
            self.get_parameter("use_gimbal_feedback").value
        )
        self.configure_gimbal_manager = bool(
            self.get_parameter("configure_gimbal_manager").value
        )

        self.last_detection: SelectedDetection | None = None
        self.last_detection_time_s: float | None = None
        self.last_update_time_s: float | None = None
        self.last_gimbal_feedback_time_s: float | None = None
        self.has_sent_gimbal_command = False
        self.has_sent_gimbal_configure = False
        self.camera_intrinsics = self._fallback_intrinsics()
        self.locked_track_id = self.target_track_id or None
        self.warned_feedback_frame_mismatch = False

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            self.vehicle_command_topic,
            px4_qos,
        )
        self.gimbal_set_attitude_pub = self.create_publisher(
            GimbalManagerSetAttitude,
            self.gimbal_set_attitude_topic,
            px4_qos,
        )
        self.error_pub = self.create_publisher(Vector3Stamped, self.error_topic, 10)
        self.tracking_active_pub = self.create_publisher(
            Bool,
            self.tracking_active_topic,
            10,
        )

        self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self._detections_callback,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GimbalDeviceAttitudeStatus,
            self.gimbal_attitude_topic,
            self._gimbal_attitude_callback,
            px4_qos,
        )

        timer_period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        target_filter = self.target_class_id or "all classes"
        track_filter = self.target_track_id or "auto-lock"
        self.get_logger().info(
            "Gimbal target tracker ready: "
            f"detections={self.detections_topic}, camera_info={self.camera_info_topic}, "
            f"gimbal_attitude={self.gimbal_attitude_topic}, "
            f"command_interface={self.command_interface}, "
            f"command={self.vehicle_command_topic}, "
            f"set_attitude={self.gimbal_set_attitude_topic}, class={target_filter}, "
            f"track={track_filter}, yaw_frame={self.yaw_frame}, "
            f"rate={self.control_rate_hz:.1f} Hz"
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if len(msg.k) < 6:
            return

        fx = float(msg.k[0])
        fy = float(msg.k[4])
        if fx <= 0.0 or fy <= 0.0:
            return

        self.camera_intrinsics = CameraIntrinsics(
            fx=fx,
            fy=fy,
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
            width=float(msg.width),
            height=float(msg.height),
            source="camera_info",
        )

    def _gimbal_attitude_callback(self, msg: GimbalDeviceAttitudeStatus) -> None:
        if not self.use_gimbal_feedback:
            return
        if self.gimbal_device_id and msg.gimbal_device_id != int(self.gimbal_device_id):
            return

        feedback_yaw_frame = self._feedback_yaw_frame(int(msg.device_flags))
        if feedback_yaw_frame != self.yaw_frame:
            if not self.warned_feedback_frame_mismatch:
                self.get_logger().warn(
                    "Ignoring gimbal attitude feedback because its yaw frame "
                    f"is {feedback_yaw_frame!r}, but commands use {self.yaw_frame!r}"
                )
                self.warned_feedback_frame_mismatch = True
            return

        _, pitch_deg, yaw_deg = quaternion_to_euler_deg(msg.q)
        self.pitch_deg = clamp(pitch_deg, self.min_pitch_deg, self.max_pitch_deg)
        self.yaw_deg = clamp(yaw_deg, self.min_yaw_deg, self.max_yaw_deg)
        self.last_gimbal_feedback_time_s = self._now_s()
        self.warned_feedback_frame_mismatch = False

    def _detections_callback(self, msg: Detection2DArray) -> None:
        selected = self._select_detection(msg.detections)
        if selected is None:
            return

        self.last_detection = selected
        self.last_detection_time_s = self._now_s()
        if self.lock_target_track and not self.locked_track_id and selected.track_id:
            self.locked_track_id = selected.track_id

    def _select_detection(
        self,
        detections: list[Detection2D],
    ) -> SelectedDetection | None:
        candidates: list[SelectedDetection] = []

        for detection in detections:
            class_id, score = self._best_hypothesis(detection)
            if score < self.min_score:
                continue
            if self.target_class_id and class_id != self.target_class_id:
                continue

            candidates.append(
                SelectedDetection(
                    track_id=str(detection.id),
                    class_id=class_id,
                    score=score,
                    center_x=float(detection.bbox.center.position.x),
                    center_y=float(detection.bbox.center.position.y),
                )
            )

        if not candidates:
            return None

        required_track_id = self.target_track_id or self.locked_track_id
        if required_track_id:
            for candidate in candidates:
                if candidate.track_id == required_track_id:
                    return candidate
            return None

        return max(candidates, key=lambda candidate: candidate.score)

    def _release_auto_track_lock_if_lost(self, active: bool) -> None:
        if active or self.target_track_id or not self.locked_track_id:
            return
        self.locked_track_id = None
        self.last_detection = None
        self.last_detection_time_s = None

    def _gimbal_manager_flags(self) -> int:
        if self.yaw_frame == "vehicle":
            return self.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME
        if self.yaw_frame == "earth":
            return (
                self.GIMBAL_MANAGER_FLAGS_YAW_LOCK
                | self.GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME
            )
        raise ValueError(
            "yaw_frame must be 'vehicle' or 'earth'; "
            f"got {self.yaw_frame!r}"
        )

    def _validate_command_interface(self) -> None:
        valid_interfaces = {
            self.COMMAND_INTERFACE_GIMBAL_MANAGER_SET_ATTITUDE,
            self.COMMAND_INTERFACE_VEHICLE_COMMAND,
        }
        if self.command_interface not in valid_interfaces:
            raise ValueError(
                "command_interface must be one of "
                f"{sorted(valid_interfaces)}; got {self.command_interface!r}"
            )

    def _feedback_yaw_frame(self, device_flags: int) -> str:
        has_vehicle_frame = bool(
            device_flags & self.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME
        )
        has_earth_frame = bool(device_flags & self.GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME)
        if has_vehicle_frame and not has_earth_frame:
            return "vehicle"
        if has_earth_frame and not has_vehicle_frame:
            return "earth"

        # Backward-compatible MAVLink interpretation when neither explicit
        # yaw-frame flag is present.
        if not has_vehicle_frame and not has_earth_frame:
            if device_flags & self.GIMBAL_MANAGER_FLAGS_YAW_LOCK:
                return "earth"
            return "vehicle"

        return "invalid"

    @staticmethod
    def _best_hypothesis(detection: Detection2D) -> tuple[str, float]:
        best_class_id = ""
        best_score = 0.0

        for result in detection.results:
            score = float(result.hypothesis.score)
            if score >= best_score:
                best_score = score
                best_class_id = str(result.hypothesis.class_id)

        return best_class_id, best_score

    def _timer_callback(self) -> None:
        now_s = self._now_s()
        dt_s = self._time_delta_s(now_s)

        active = self._has_fresh_detection(now_s)
        self._publish_tracking_active(active)
        self._release_auto_track_lock_if_lost(active)

        if not active or self.last_detection is None:
            self._handle_missing_target()
            return

        yaw_error_deg, pitch_error_deg = self._camera_angle_error_deg(
            self.last_detection
        )
        yaw_error_deg = self._apply_angle_deadband(yaw_error_deg)
        pitch_error_deg = self._apply_angle_deadband(pitch_error_deg)

        yaw_rate_deg_s = clamp(
            self.yaw_error_sign * self.yaw_rate_gain_s_inv * yaw_error_deg,
            -self.max_yaw_rate_deg_s,
            self.max_yaw_rate_deg_s,
        )
        pitch_rate_deg_s = clamp(
            self.pitch_error_sign * self.pitch_rate_gain_s_inv * pitch_error_deg,
            -self.max_pitch_rate_deg_s,
            self.max_pitch_rate_deg_s,
        )

        self.yaw_deg = clamp(
            self.yaw_deg + yaw_rate_deg_s * dt_s,
            self.min_yaw_deg,
            self.max_yaw_deg,
        )
        self.pitch_deg = clamp(
            self.pitch_deg + pitch_rate_deg_s * dt_s,
            self.min_pitch_deg,
            self.max_pitch_deg,
        )

        now_us = self._now_us()
        self._publish_gimbal_configure_if_needed(now_us)
        self._publish_error(
            now_us,
            yaw_error_deg,
            pitch_error_deg,
            self.last_detection.score,
        )
        self._publish_gimbal_setpoint(now_us, self.pitch_deg, self.yaw_deg)

    def _handle_missing_target(self) -> None:
        now_us = self._now_us()
        self._publish_gimbal_configure_if_needed(now_us)

        if self.has_sent_gimbal_command and self.hold_last_command_on_loss:
            self._publish_gimbal_setpoint(now_us, self.pitch_deg, self.yaw_deg)
            return

        if (
            not self.has_sent_gimbal_command
            and self.send_command_before_first_detection
        ):
            self._publish_gimbal_setpoint(now_us, self.pitch_deg, self.yaw_deg)

    def _time_delta_s(self, now_s: float) -> float:
        if self.last_update_time_s is None:
            self.last_update_time_s = now_s
            return 1.0 / max(self.control_rate_hz, 1.0)

        dt_s = max(0.0, now_s - self.last_update_time_s)
        self.last_update_time_s = now_s
        return min(dt_s, 0.25)

    def _has_fresh_detection(self, now_s: float) -> bool:
        return (
            self.last_detection is not None
            and self.last_detection_time_s is not None
            and now_s - self.last_detection_time_s <= self.lost_timeout_s
        )

    def _camera_angle_error_deg(
        self,
        detection: SelectedDetection,
    ) -> tuple[float, float]:
        intrinsics = self.camera_intrinsics
        image_width = max(intrinsics.width, 1.0)
        image_height = max(intrinsics.height, 1.0)
        center_x = clamp(detection.center_x, 0.0, image_width)
        center_y = clamp(detection.center_y, 0.0, image_height)

        yaw_error_rad = math.atan2(center_x - intrinsics.cx, intrinsics.fx)
        pitch_error_rad = math.atan2(center_y - intrinsics.cy, intrinsics.fy)
        return math.degrees(yaw_error_rad), math.degrees(pitch_error_rad)

    def _fallback_intrinsics(self) -> CameraIntrinsics:
        return CameraIntrinsics(
            fx=max(self.fallback_fx_px, 1.0),
            fy=max(self.fallback_fy_px, 1.0),
            cx=self.fallback_cx_px,
            cy=self.fallback_cy_px,
            width=max(self.fallback_image_width, 1.0),
            height=max(self.fallback_image_height, 1.0),
            source="fallback",
        )

    def _apply_angle_deadband(self, value: float) -> float:
        return 0.0 if abs(value) < self.deadband_angle_deg else value

    def _publish_error(
        self,
        now_us: int,
        yaw_error_deg: float,
        pitch_error_deg: float,
        score: float,
    ) -> None:
        msg = Vector3Stamped()
        msg.header.stamp.sec = int(now_us // 1_000_000)
        msg.header.stamp.nanosec = int((now_us % 1_000_000) * 1000)
        msg.header.frame_id = "camera_link"
        msg.vector.x = float(yaw_error_deg)
        msg.vector.y = float(pitch_error_deg)
        msg.vector.z = float(score)
        self.error_pub.publish(msg)

    def _publish_tracking_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self.tracking_active_pub.publish(msg)

    def _publish_gimbal_configure_if_needed(self, now_us: int) -> None:
        if self.has_sent_gimbal_configure or not self.configure_gimbal_manager:
            return

        msg = VehicleCommand()
        msg.timestamp = now_us
        msg.command = int(
            getattr(
                VehicleCommand,
                "VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE",
                self.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            )
        )
        msg.param1 = float(self.source_system)
        msg.param2 = float(self.source_component)
        msg.param3 = -1.0
        msg.param4 = -1.0
        msg.param5 = math.nan
        msg.param6 = math.nan
        msg.param7 = self.gimbal_device_id
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = self.source_system
        msg.source_component = self.source_component
        msg.from_external = True
        self.command_pub.publish(msg)
        self.has_sent_gimbal_configure = True

    def _publish_gimbal_setpoint(
        self,
        now_us: int,
        pitch_deg: float,
        yaw_deg: float,
    ) -> None:
        if (
            self.command_interface
            == self.COMMAND_INTERFACE_GIMBAL_MANAGER_SET_ATTITUDE
        ):
            self._publish_gimbal_manager_set_attitude(now_us, pitch_deg, yaw_deg)
        else:
            self._publish_gimbal_vehicle_command(now_us, pitch_deg, yaw_deg)

    def _publish_gimbal_manager_set_attitude(
        self,
        now_us: int,
        pitch_deg: float,
        yaw_deg: float,
    ) -> None:
        msg = GimbalManagerSetAttitude()
        msg.timestamp = now_us
        msg.origin_sysid = self.source_system
        msg.origin_compid = self.source_component
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.flags = int(self.gimbal_manager_flags)
        msg.gimbal_device_id = int(self.gimbal_device_id)
        msg.q = euler_to_quaternion(
            0.0,
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )
        msg.angular_velocity_x = math.nan
        msg.angular_velocity_y = math.nan
        msg.angular_velocity_z = math.nan
        self.gimbal_set_attitude_pub.publish(msg)
        self.has_sent_gimbal_command = True

    def _publish_gimbal_vehicle_command(
        self,
        now_us: int,
        pitch_deg: float,
        yaw_deg: float,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = now_us
        msg.command = int(
            getattr(
                VehicleCommand,
                "VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW",
                self.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            )
        )
        msg.param1 = float(pitch_deg)
        msg.param2 = float(yaw_deg)
        msg.param3 = math.nan
        msg.param4 = math.nan
        msg.param5 = self.gimbal_manager_flags
        msg.param6 = 0.0
        msg.param7 = self.gimbal_device_id
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = self.source_system
        msg.source_component = self.source_component
        msg.from_external = True
        self.command_pub.publish(msg)
        self.has_sent_gimbal_command = True

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def quaternion_to_euler_deg(
    q: Sequence[float],
) -> tuple[float, float, float]:
    w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    roll_rad = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_sin = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch_rad = math.asin(pitch_sin)
    yaw_rad = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )

    return (
        math.degrees(roll_rad),
        math.degrees(pitch_rad),
        math.degrees(yaw_rad),
    )


def euler_to_quaternion(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> list[float]:
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GimbalTargetTracker()

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
