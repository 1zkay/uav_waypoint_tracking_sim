#!/usr/bin/env python3
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import (
    GimbalDeviceAttitudeStatus,
    GimbalManagerSetAttitude,
    VehicleCommand,
    VehicleCommandAck,
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
from sensor_msgs.msg import CameraInfo, Image
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
    GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME = 32
    GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME = 64
    COMMAND_INTERFACE_GIMBAL_MANAGER_SET_ATTITUDE = "gimbal_manager_set_attitude"
    COMMAND_INTERFACE_VEHICLE_COMMAND = "vehicle_command"

    def __init__(self) -> None:
        super().__init__("gimbal_target_tracker")

        self.declare_parameter("detections_topic", "/x500_0/yolo/tracks")
        self.declare_parameter("camera_image_topic", "/x500_0/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/x500_0/camera/camera_info")
        self.declare_parameter(
            "gimbal_attitude_topic",
            "/fmu/out/gimbal_device_attitude_status",
        )
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter(
            "vehicle_command_ack_topic",
            "/fmu/out/vehicle_command_ack",
        )
        self.declare_parameter(
            "gimbal_set_attitude_topic",
            "/fmu/in/gimbal_manager_set_attitude",
        )
        self.declare_parameter("error_topic", "/x500_0/gimbal_target_tracker/error")
        self.declare_parameter(
            "tracking_active_topic",
            "/x500_0/gimbal_target_tracker/tracking_active",
        )
        self.declare_parameter("state_topic", "/x500_0/gimbal_target_tracker/state")

        self.declare_parameter("target_class_id", "")
        self.declare_parameter("target_track_id", "")
        self.declare_parameter("lock_target_track", True)
        self.declare_parameter("min_score", 0.35)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("lost_timeout_s", 0.8)
        self.declare_parameter("camera_image_timeout_s", 1.0)
        self.declare_parameter("detections_stream_timeout_s", 2.0)
        self.declare_parameter("fallback_fx_px", 410.93927419797166)
        self.declare_parameter("fallback_fy_px", 410.93927419797166)
        self.declare_parameter("fallback_cx_px", 640.0)
        self.declare_parameter("fallback_cy_px", 360.0)
        self.declare_parameter("fallback_image_width", 1280.0)
        self.declare_parameter("fallback_image_height", 720.0)
        self.declare_parameter("deadband_angle_deg", 1.5)

        self.declare_parameter("yaw_kp_s_inv", 0.18)
        self.declare_parameter("pitch_kp_s_inv", 0.14)
        self.declare_parameter("yaw_ki_s_inv2", 0.0)
        self.declare_parameter("pitch_ki_s_inv2", 0.0)
        self.declare_parameter("yaw_feedforward_deg_s", 0.0)
        self.declare_parameter("pitch_feedforward_deg_s", 0.0)
        self.declare_parameter("max_yaw_error_integral_deg_s", 90.0)
        self.declare_parameter("max_pitch_error_integral_deg_s", 90.0)
        self.declare_parameter("max_yaw_rate_deg_s", 18.0)
        self.declare_parameter("max_pitch_rate_deg_s", 14.0)
        self.declare_parameter("yaw_error_sign", 1.0)
        self.declare_parameter("pitch_error_sign", 1.0)

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
        self.declare_parameter("search_enabled", True)
        self.declare_parameter("search_after_lost_s", 1.0)
        self.declare_parameter("local_search_duration_s", 8.0)
        self.declare_parameter("search_yaw_rate_deg_s", 12.0)
        self.declare_parameter("search_pitch_rate_deg_s", 8.0)
        self.declare_parameter("search_initial_yaw_amplitude_deg", 10.0)
        self.declare_parameter("search_max_yaw_amplitude_deg", 90.0)
        self.declare_parameter(
            "search_pitch_bands_deg",
            [0.0, -8.0, 8.0, -16.0, 16.0],
        )
        self.declare_parameter("max_search_cmd_actual_error_deg", 20.0)
        self.declare_parameter("use_gimbal_feedback", True)
        self.declare_parameter("initialize_command_from_feedback", True)
        self.declare_parameter("configure_gimbal_manager", True)
        self.declare_parameter("configure_retry_period_s", 1.0)
        self.declare_parameter("configure_max_attempts", 0)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.camera_image_topic = str(self.get_parameter("camera_image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.gimbal_attitude_topic = str(
            self.get_parameter("gimbal_attitude_topic").value
        )
        self.vehicle_command_topic = str(
            self.get_parameter("vehicle_command_topic").value
        )
        self.vehicle_command_ack_topic = str(
            self.get_parameter("vehicle_command_ack_topic").value
        )
        self.gimbal_set_attitude_topic = str(
            self.get_parameter("gimbal_set_attitude_topic").value
        )
        self.error_topic = str(self.get_parameter("error_topic").value)
        self.tracking_active_topic = str(
            self.get_parameter("tracking_active_topic").value
        )
        self.state_topic = str(self.get_parameter("state_topic").value)

        self.target_class_id = str(self.get_parameter("target_class_id").value).strip()
        self.target_track_id = str(self.get_parameter("target_track_id").value).strip()
        self.lock_target_track = bool(self.get_parameter("lock_target_track").value)
        self.min_score = float(self.get_parameter("min_score").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.lost_timeout_s = float(self.get_parameter("lost_timeout_s").value)
        self.camera_image_timeout_s = max(
            0.0,
            float(self.get_parameter("camera_image_timeout_s").value),
        )
        self.detections_stream_timeout_s = max(
            0.0,
            float(self.get_parameter("detections_stream_timeout_s").value),
        )
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

        self.yaw_kp_s_inv = float(self.get_parameter("yaw_kp_s_inv").value)
        self.pitch_kp_s_inv = float(self.get_parameter("pitch_kp_s_inv").value)
        self.yaw_ki_s_inv2 = float(self.get_parameter("yaw_ki_s_inv2").value)
        self.pitch_ki_s_inv2 = float(self.get_parameter("pitch_ki_s_inv2").value)
        self.yaw_feedforward_deg_s = float(
            self.get_parameter("yaw_feedforward_deg_s").value
        )
        self.pitch_feedforward_deg_s = float(
            self.get_parameter("pitch_feedforward_deg_s").value
        )
        self.max_yaw_error_integral_deg_s = max(
            0.0,
            float(self.get_parameter("max_yaw_error_integral_deg_s").value),
        )
        self.max_pitch_error_integral_deg_s = max(
            0.0,
            float(self.get_parameter("max_pitch_error_integral_deg_s").value),
        )
        self.max_yaw_rate_deg_s = float(
            self.get_parameter("max_yaw_rate_deg_s").value
        )
        self.max_pitch_rate_deg_s = float(
            self.get_parameter("max_pitch_rate_deg_s").value
        )
        self.yaw_error_sign = float(self.get_parameter("yaw_error_sign").value)
        self.pitch_error_sign = float(self.get_parameter("pitch_error_sign").value)

        self.cmd_yaw_deg = float(self.get_parameter("initial_yaw_deg").value)
        self.cmd_pitch_deg = float(self.get_parameter("initial_pitch_deg").value)
        self.actual_yaw_deg: float | None = None
        self.actual_pitch_deg: float | None = None
        self.yaw_error_integral_deg_s = 0.0
        self.pitch_error_integral_deg_s = 0.0
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
        self.search_enabled = bool(self.get_parameter("search_enabled").value)
        self.search_after_lost_s = max(
            0.0,
            float(self.get_parameter("search_after_lost_s").value),
        )
        self.local_search_duration_s = max(
            0.0,
            float(self.get_parameter("local_search_duration_s").value),
        )
        self.search_yaw_rate_deg_s = max(
            0.0,
            float(self.get_parameter("search_yaw_rate_deg_s").value),
        )
        self.search_pitch_rate_deg_s = max(
            0.0,
            float(self.get_parameter("search_pitch_rate_deg_s").value),
        )
        self.search_initial_yaw_amplitude_deg = max(
            0.0,
            float(self.get_parameter("search_initial_yaw_amplitude_deg").value),
        )
        self.search_max_yaw_amplitude_deg = max(
            self.search_initial_yaw_amplitude_deg,
            float(self.get_parameter("search_max_yaw_amplitude_deg").value),
        )
        self.search_pitch_bands_deg = self._float_list_parameter(
            "search_pitch_bands_deg",
            [0.0],
        )
        self.max_search_cmd_actual_error_deg = max(
            0.0,
            float(self.get_parameter("max_search_cmd_actual_error_deg").value),
        )
        self.use_gimbal_feedback = bool(
            self.get_parameter("use_gimbal_feedback").value
        )
        self.initialize_command_from_feedback = bool(
            self.get_parameter("initialize_command_from_feedback").value
        )
        self.configure_gimbal_manager = bool(
            self.get_parameter("configure_gimbal_manager").value
        )
        self.configure_retry_period_s = max(
            0.1,
            float(self.get_parameter("configure_retry_period_s").value),
        )
        self.configure_max_attempts = max(
            0,
            int(self.get_parameter("configure_max_attempts").value),
        )

        self.last_detection: SelectedDetection | None = None
        self.last_detection_time_s: float | None = None
        self.last_detections_msg_time_s: float | None = None
        self.last_camera_image_time_s: float | None = None
        self.target_missing_since_s: float | None = None
        self.last_update_time_s: float | None = None
        self.last_gimbal_feedback_time_s: float | None = None
        self.tracking_state = "initializing"
        self.search_start_time_s: float | None = None
        self.search_anchor_yaw_deg = self.cmd_yaw_deg
        self.search_anchor_pitch_deg = self.cmd_pitch_deg
        self.search_pitch_band_index = 0
        self.search_direction = 1.0
        self.search_pitch_target_deg = self.cmd_pitch_deg
        self.search_waiting_for_gimbal = False
        self.has_sent_gimbal_command = False
        self.command_initialized_from_feedback = False
        self.has_sent_gimbal_configure = False
        self.gimbal_configure_accepted = False
        self.gimbal_configure_attempts = 0
        self.last_gimbal_configure_time_s: float | None = None
        self.last_gimbal_configure_ack_result: int | None = None
        self.warned_gimbal_configure_max_attempts = False
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
        self.state_pub = self.create_publisher(DiagnosticArray, self.state_topic, 10)

        self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self._detections_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.camera_image_topic,
            self._camera_image_callback,
            qos_profile_sensor_data,
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
        self.create_subscription(
            VehicleCommandAck,
            self.vehicle_command_ack_topic,
            self._vehicle_command_ack_callback,
            px4_qos,
        )

        timer_period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        target_filter = self.target_class_id or "all classes"
        track_filter = self.target_track_id or "auto-lock"
        self.get_logger().info(
            "Gimbal target tracker ready: "
            f"detections={self.detections_topic}, image={self.camera_image_topic}, "
            f"camera_info={self.camera_info_topic}, "
            f"gimbal_attitude={self.gimbal_attitude_topic}, "
            f"command_interface={self.command_interface}, "
            f"command={self.vehicle_command_topic}, ack={self.vehicle_command_ack_topic}, "
            f"set_attitude={self.gimbal_set_attitude_topic}, class={target_filter}, "
            f"track={track_filter}, yaw_frame={self.yaw_frame}, "
            f"state={self.state_topic}, rate={self.control_rate_hz:.1f} Hz"
        )

    def _float_list_parameter(
        self,
        parameter_name: str,
        fallback: list[float],
    ) -> list[float]:
        values = self.get_parameter(parameter_name).value
        if isinstance(values, str):
            return fallback
        try:
            result = [float(value) for value in values]
        except (TypeError, ValueError):
            result = []
        return result or fallback

    def _camera_image_callback(self, _msg: Image) -> None:
        self.last_camera_image_time_s = self._now_s()

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
        self.actual_pitch_deg = clamp(
            pitch_deg,
            self.min_pitch_deg,
            self.max_pitch_deg,
        )
        self.actual_yaw_deg = clamp(yaw_deg, self.min_yaw_deg, self.max_yaw_deg)
        self.last_gimbal_feedback_time_s = self._now_s()
        self._initialize_command_from_feedback_if_needed()
        self.warned_feedback_frame_mismatch = False

    def _initialize_command_from_feedback_if_needed(self) -> None:
        if not self.initialize_command_from_feedback:
            return
        if self.command_initialized_from_feedback or self.has_sent_gimbal_command:
            return
        if self.actual_yaw_deg is None or self.actual_pitch_deg is None:
            return

        self.cmd_yaw_deg = self.actual_yaw_deg
        self.cmd_pitch_deg = self.actual_pitch_deg
        self.command_initialized_from_feedback = True
        self.get_logger().info(
            "Initialized gimbal command from feedback: "
            f"yaw={self.cmd_yaw_deg:.2f} deg, pitch={self.cmd_pitch_deg:.2f} deg"
        )

    def _vehicle_command_ack_callback(self, msg: VehicleCommandAck) -> None:
        if msg.command != self._gimbal_configure_command_id():
            return
        if self.gimbal_configure_attempts == 0:
            return
        if msg.target_system != self.source_system:
            return
        if msg.target_component != self.source_component:
            return

        result = int(msg.result)
        accepted_result = int(
            getattr(VehicleCommandAck, "VEHICLE_CMD_RESULT_ACCEPTED", 0)
        )
        if result == accepted_result:
            if not self.gimbal_configure_accepted:
                self.get_logger().info(
                    "Gimbal manager configure accepted "
                    f"after {self.gimbal_configure_attempts} attempt(s)"
                )
            self.gimbal_configure_accepted = True
            return

        if result != self.last_gimbal_configure_ack_result:
            self.get_logger().warn(
                "Gimbal manager configure ACK was not accepted: "
                f"result={result}"
            )
            self.last_gimbal_configure_ack_result = result

    def _detections_callback(self, msg: Detection2DArray) -> None:
        now_s = self._now_s()
        self.last_detections_msg_time_s = now_s
        selected = self._select_detection(msg.detections)
        if selected is None:
            if self.target_missing_since_s is None:
                self.target_missing_since_s = now_s
            return

        self.last_detection = selected
        self.last_detection_time_s = now_s
        self.target_missing_since_s = None
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

        if self.lock_target_track and self.last_detection is not None:
            return min(
                candidates,
                key=lambda candidate: squared_image_distance(
                    candidate,
                    self.last_detection,
                ),
            )

        return max(candidates, key=lambda candidate: candidate.score)

    def _release_auto_track_lock_if_lost(self, active: bool) -> None:
        if active or self.target_track_id or not self.locked_track_id:
            return
        self.locked_track_id = None

    def _gimbal_manager_flags(self) -> int:
        if self.yaw_frame == "vehicle":
            return 0
        if self.yaw_frame == "earth":
            return self.GIMBAL_MANAGER_FLAGS_YAW_LOCK
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
            device_flags & self.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME
        )
        has_earth_frame = bool(
            device_flags & self.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME
        )
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
            self._handle_missing_target(now_s, dt_s)
            return

        self.tracking_state = "tracking"
        self._reset_search()

        yaw_error_deg, pitch_error_deg = self._camera_angle_error_deg(
            self.last_detection
        )
        yaw_error_deg = self._apply_angle_deadband(yaw_error_deg)
        pitch_error_deg = self._apply_angle_deadband(pitch_error_deg)

        yaw_control_error_deg = self.yaw_error_sign * yaw_error_deg
        pitch_control_error_deg = self.pitch_error_sign * pitch_error_deg
        self._update_error_integrals(
            yaw_control_error_deg,
            pitch_control_error_deg,
            dt_s,
        )

        yaw_rate_deg_s = self._yaw_control_rate_deg_s(yaw_control_error_deg)
        pitch_rate_deg_s = self._pitch_control_rate_deg_s(pitch_control_error_deg)

        self.cmd_yaw_deg = clamp(
            self.cmd_yaw_deg + yaw_rate_deg_s * dt_s,
            self.min_yaw_deg,
            self.max_yaw_deg,
        )
        self.cmd_pitch_deg = clamp(
            self.cmd_pitch_deg + pitch_rate_deg_s * dt_s,
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
        self._publish_gimbal_setpoint(
            now_us,
            self.cmd_pitch_deg,
            self.cmd_yaw_deg,
        )
        self._publish_gimbal_state(now_us, active)

    def _handle_missing_target(self, now_s: float, dt_s: float) -> None:
        now_us = self._now_us()
        self._publish_gimbal_configure_if_needed(now_us)
        self._reset_error_integrals()
        self._mark_target_missing_if_needed(now_s)

        if not self._has_recent_camera_image(now_s):
            self.tracking_state = "camera_fault"
            self._reset_search()
            self._publish_hold_setpoint_if_needed(now_us)
        elif not self._has_recent_detections_stream(now_s):
            self.tracking_state = "vision_stream_lost"
            self._reset_search()
            self._publish_hold_setpoint_if_needed(now_us)
        elif self._should_search_for_target(now_s):
            self._update_search_command(now_s, dt_s)
            self._publish_gimbal_setpoint(
                now_us,
                self.cmd_pitch_deg,
                self.cmd_yaw_deg,
            )
        else:
            self.tracking_state = "hold"
            self._reset_search()
            self._publish_hold_setpoint_if_needed(now_us)

        self._publish_gimbal_state(now_us, False)

    def _publish_hold_setpoint_if_needed(self, now_us: int) -> None:
        if self.has_sent_gimbal_command and self.hold_last_command_on_loss:
            self._publish_gimbal_setpoint(
                now_us,
                self.cmd_pitch_deg,
                self.cmd_yaw_deg,
            )
            return

        if (
            not self.has_sent_gimbal_command
            and self.send_command_before_first_detection
        ):
            self._publish_gimbal_setpoint(
                now_us,
                self.cmd_pitch_deg,
                self.cmd_yaw_deg,
            )

    def _mark_target_missing_if_needed(self, now_s: float) -> None:
        if self.target_missing_since_s is not None:
            return
        if (
            self.last_detection_time_s is not None
            and now_s - self.last_detection_time_s > self.lost_timeout_s
        ):
            self.target_missing_since_s = self.last_detection_time_s
            return
        if self.last_detections_msg_time_s is not None:
            self.target_missing_since_s = self.last_detections_msg_time_s

    def _should_search_for_target(self, now_s: float) -> bool:
        if not self.search_enabled:
            return False
        missing_duration_s = self._target_missing_duration_s(now_s)
        if missing_duration_s is None:
            return False
        return missing_duration_s >= self.search_after_lost_s

    def _target_missing_duration_s(self, now_s: float) -> float | None:
        if self.target_missing_since_s is None:
            return None
        return max(0.0, now_s - self.target_missing_since_s)

    def _has_recent_camera_image(self, now_s: float) -> bool:
        if self.camera_image_timeout_s <= 0.0:
            return True
        return (
            self.last_camera_image_time_s is not None
            and now_s - self.last_camera_image_time_s <= self.camera_image_timeout_s
        )

    def _has_recent_detections_stream(self, now_s: float) -> bool:
        if self.detections_stream_timeout_s <= 0.0:
            return True
        return (
            self.last_detections_msg_time_s is not None
            and now_s - self.last_detections_msg_time_s
            <= self.detections_stream_timeout_s
        )

    def _reset_search(self) -> None:
        self.search_start_time_s = None
        self.search_waiting_for_gimbal = False

    def _update_search_command(self, now_s: float, dt_s: float) -> None:
        if self.search_start_time_s is None:
            self._start_search(now_s)

        search_elapsed_s = max(0.0, now_s - (self.search_start_time_s or now_s))
        local_search = search_elapsed_s <= self.local_search_duration_s
        self.tracking_state = "local_search" if local_search else "global_search"

        if local_search:
            yaw_min_deg, yaw_max_deg = self._local_search_yaw_limits(
                search_elapsed_s
            )
        else:
            yaw_min_deg = self.min_yaw_deg
            yaw_max_deg = self.max_yaw_deg

        self.cmd_yaw_deg = clamp(self.cmd_yaw_deg, yaw_min_deg, yaw_max_deg)
        self.search_waiting_for_gimbal = self._search_gimbal_lag_too_large(now_s)
        if self.search_waiting_for_gimbal:
            return

        yaw_step_deg = self.search_direction * self.search_yaw_rate_deg_s * dt_s
        next_yaw_deg = self.cmd_yaw_deg + yaw_step_deg
        if next_yaw_deg >= yaw_max_deg:
            self.cmd_yaw_deg = yaw_max_deg
            self.search_direction = -1.0
            self._advance_search_pitch_band()
        elif next_yaw_deg <= yaw_min_deg:
            self.cmd_yaw_deg = yaw_min_deg
            self.search_direction = 1.0
            self._advance_search_pitch_band()
        else:
            self.cmd_yaw_deg = next_yaw_deg

        pitch_step_deg = self.search_pitch_rate_deg_s * dt_s
        self.cmd_pitch_deg = move_toward(
            self.cmd_pitch_deg,
            self.search_pitch_target_deg,
            pitch_step_deg,
        )

    def _start_search(self, now_s: float) -> None:
        self.search_start_time_s = now_s
        self.search_waiting_for_gimbal = False
        self.search_direction = 1.0
        self.search_pitch_band_index = 0

        self.search_anchor_yaw_deg = self._search_anchor_yaw_deg(now_s)
        self.search_anchor_pitch_deg = self._search_anchor_pitch_deg(now_s)
        self.cmd_yaw_deg = clamp(
            self.search_anchor_yaw_deg,
            self.min_yaw_deg,
            self.max_yaw_deg,
        )
        self.cmd_pitch_deg = clamp(
            self.search_anchor_pitch_deg,
            self.min_pitch_deg,
            self.max_pitch_deg,
        )
        self._set_search_pitch_target()

    def _search_anchor_yaw_deg(self, now_s: float) -> float:
        if self._has_fresh_gimbal_feedback(now_s) and self.actual_yaw_deg is not None:
            return self.actual_yaw_deg
        return self.cmd_yaw_deg

    def _search_anchor_pitch_deg(self, now_s: float) -> float:
        if (
            self._has_fresh_gimbal_feedback(now_s)
            and self.actual_pitch_deg is not None
        ):
            return self.actual_pitch_deg
        return self.cmd_pitch_deg

    def _local_search_yaw_limits(
        self,
        search_elapsed_s: float,
    ) -> tuple[float, float]:
        if self.local_search_duration_s <= 0.0:
            amplitude_deg = self.search_max_yaw_amplitude_deg
        else:
            expansion = clamp(
                search_elapsed_s / self.local_search_duration_s,
                0.0,
                1.0,
            )
            amplitude_deg = self.search_initial_yaw_amplitude_deg + expansion * (
                self.search_max_yaw_amplitude_deg
                - self.search_initial_yaw_amplitude_deg
            )

        return (
            clamp(
                self.search_anchor_yaw_deg - amplitude_deg,
                self.min_yaw_deg,
                self.max_yaw_deg,
            ),
            clamp(
                self.search_anchor_yaw_deg + amplitude_deg,
                self.min_yaw_deg,
                self.max_yaw_deg,
            ),
        )

    def _advance_search_pitch_band(self) -> None:
        self.search_pitch_band_index = (
            self.search_pitch_band_index + 1
        ) % len(self.search_pitch_bands_deg)
        self._set_search_pitch_target()

    def _set_search_pitch_target(self) -> None:
        pitch_offset_deg = self.search_pitch_bands_deg[self.search_pitch_band_index]
        self.search_pitch_target_deg = clamp(
            self.search_anchor_pitch_deg + pitch_offset_deg,
            self.min_pitch_deg,
            self.max_pitch_deg,
        )

    def _search_gimbal_lag_too_large(self, now_s: float) -> bool:
        if self.max_search_cmd_actual_error_deg <= 0.0:
            return False
        if not self._has_fresh_gimbal_feedback(now_s):
            return False
        if self.actual_yaw_deg is None or self.actual_pitch_deg is None:
            return False

        yaw_error_deg = abs(self.cmd_yaw_deg - self.actual_yaw_deg)
        pitch_error_deg = abs(self.cmd_pitch_deg - self.actual_pitch_deg)
        return (
            max(yaw_error_deg, pitch_error_deg)
            > self.max_search_cmd_actual_error_deg
        )

    def _has_fresh_gimbal_feedback(self, now_s: float) -> bool:
        return (
            self.last_gimbal_feedback_time_s is not None
            and now_s - self.last_gimbal_feedback_time_s <= 1.0
        )

    def _update_error_integrals(
        self,
        yaw_control_error_deg: float,
        pitch_control_error_deg: float,
        dt_s: float,
    ) -> None:
        self.yaw_error_integral_deg_s = self._next_error_integral(
            self.yaw_error_integral_deg_s,
            yaw_control_error_deg,
            dt_s,
            self.max_yaw_error_integral_deg_s,
            self.cmd_yaw_deg,
            self.min_yaw_deg,
            self.max_yaw_deg,
        )
        self.pitch_error_integral_deg_s = self._next_error_integral(
            self.pitch_error_integral_deg_s,
            pitch_control_error_deg,
            dt_s,
            self.max_pitch_error_integral_deg_s,
            self.cmd_pitch_deg,
            self.min_pitch_deg,
            self.max_pitch_deg,
        )

    def _next_error_integral(
        self,
        current_integral_deg_s: float,
        error_deg: float,
        dt_s: float,
        max_abs_integral_deg_s: float,
        cmd_deg: float,
        min_cmd_deg: float,
        max_cmd_deg: float,
    ) -> float:
        if (cmd_deg <= min_cmd_deg and error_deg < 0.0) or (
            cmd_deg >= max_cmd_deg and error_deg > 0.0
        ):
            return current_integral_deg_s

        next_integral_deg_s = current_integral_deg_s + error_deg * dt_s
        if max_abs_integral_deg_s <= 0.0:
            return next_integral_deg_s
        return clamp(
            next_integral_deg_s,
            -max_abs_integral_deg_s,
            max_abs_integral_deg_s,
        )

    def _reset_error_integrals(self) -> None:
        self.yaw_error_integral_deg_s = 0.0
        self.pitch_error_integral_deg_s = 0.0

    def _yaw_control_rate_deg_s(self, yaw_control_error_deg: float) -> float:
        return clamp(
            self.yaw_kp_s_inv * yaw_control_error_deg
            + self.yaw_ki_s_inv2 * self.yaw_error_integral_deg_s
            + self.yaw_feedforward_deg_s,
            -self.max_yaw_rate_deg_s,
            self.max_yaw_rate_deg_s,
        )

    def _pitch_control_rate_deg_s(self, pitch_control_error_deg: float) -> float:
        return clamp(
            self.pitch_kp_s_inv * pitch_control_error_deg
            + self.pitch_ki_s_inv2 * self.pitch_error_integral_deg_s
            + self.pitch_feedforward_deg_s,
            -self.max_pitch_rate_deg_s,
            self.max_pitch_rate_deg_s,
        )

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

    def _publish_gimbal_state(self, now_us: int, active: bool) -> None:
        now_s = now_us * 1e-6
        camera_image_age_s = (
            None
            if self.last_camera_image_time_s is None
            else max(0.0, now_s - self.last_camera_image_time_s)
        )
        detections_stream_age_s = (
            None
            if self.last_detections_msg_time_s is None
            else max(0.0, now_s - self.last_detections_msg_time_s)
        )
        feedback_age_s = (
            None
            if self.last_gimbal_feedback_time_s is None
            else max(0.0, now_s - self.last_gimbal_feedback_time_s)
        )
        target_missing_duration_s = self._target_missing_duration_s(now_s)
        search_elapsed_s = (
            None
            if self.search_start_time_s is None
            else max(0.0, now_s - self.search_start_time_s)
        )
        cmd_actual_yaw_error_deg = (
            None
            if self.actual_yaw_deg is None
            else self.cmd_yaw_deg - self.actual_yaw_deg
        )
        cmd_actual_pitch_error_deg = (
            None
            if self.actual_pitch_deg is None
            else self.cmd_pitch_deg - self.actual_pitch_deg
        )

        msg = DiagnosticArray()
        msg.header.stamp.sec = int(now_us // 1_000_000)
        msg.header.stamp.nanosec = int((now_us % 1_000_000) * 1000)

        status = DiagnosticStatus()
        status.name = "gimbal_target_tracker"
        status.hardware_id = f"gimbal_device_id={int(self.gimbal_device_id)}"
        if self.tracking_state == "camera_fault":
            status.level = DiagnosticStatus.ERROR
            status.message = "camera image stream lost"
        elif self.tracking_state == "vision_stream_lost":
            status.level = DiagnosticStatus.WARN
            status.message = "detection stream lost"
        elif self.tracking_state in {"local_search", "global_search"}:
            status.level = DiagnosticStatus.WARN
            status.message = self.tracking_state
        elif not self.use_gimbal_feedback:
            status.level = DiagnosticStatus.OK
            status.message = "gimbal feedback disabled"
        elif feedback_age_s is None:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for gimbal feedback"
        elif feedback_age_s > 1.0:
            status.level = DiagnosticStatus.WARN
            status.message = "gimbal feedback stale"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "gimbal feedback active"

        status.values = [
            self._diagnostic_value("state", self.tracking_state),
            self._diagnostic_value("tracking_active", active),
            self._diagnostic_value("cmd_yaw_deg", self.cmd_yaw_deg),
            self._diagnostic_value("cmd_pitch_deg", self.cmd_pitch_deg),
            self._diagnostic_value("actual_yaw_deg", self.actual_yaw_deg),
            self._diagnostic_value("actual_pitch_deg", self.actual_pitch_deg),
            self._diagnostic_value(
                "cmd_actual_yaw_error_deg",
                cmd_actual_yaw_error_deg,
            ),
            self._diagnostic_value(
                "cmd_actual_pitch_error_deg",
                cmd_actual_pitch_error_deg,
            ),
            self._diagnostic_value(
                "yaw_error_integral_deg_s",
                self.yaw_error_integral_deg_s,
            ),
            self._diagnostic_value(
                "pitch_error_integral_deg_s",
                self.pitch_error_integral_deg_s,
            ),
            self._diagnostic_value("feedback_age_s", feedback_age_s),
            self._diagnostic_value(
                "command_initialized_from_feedback",
                self.command_initialized_from_feedback,
            ),
            self._diagnostic_value("camera_image_age_s", camera_image_age_s),
            self._diagnostic_value(
                "detections_stream_age_s",
                detections_stream_age_s,
            ),
            self._diagnostic_value(
                "target_missing_duration_s",
                target_missing_duration_s,
            ),
            self._diagnostic_value("search_elapsed_s", search_elapsed_s),
            self._diagnostic_value(
                "search_anchor_yaw_deg",
                self.search_anchor_yaw_deg,
            ),
            self._diagnostic_value(
                "search_anchor_pitch_deg",
                self.search_anchor_pitch_deg,
            ),
            self._diagnostic_value(
                "search_pitch_band_index",
                float(self.search_pitch_band_index),
            ),
            self._diagnostic_value(
                "search_pitch_target_deg",
                self.search_pitch_target_deg,
            ),
            self._diagnostic_value(
                "search_waiting_for_gimbal",
                self.search_waiting_for_gimbal,
            ),
        ]
        msg.status.append(status)
        self.state_pub.publish(msg)

    @staticmethod
    def _diagnostic_value(key: str, value: bool | float | str | None) -> KeyValue:
        item = KeyValue()
        item.key = key
        if isinstance(value, bool):
            item.value = str(value).lower()
        elif isinstance(value, str):
            item.value = value
        elif value is None:
            item.value = "nan"
        else:
            item.value = f"{value:.6f}"
        return item

    def _publish_gimbal_configure_if_needed(self, now_us: int) -> None:
        if not self.configure_gimbal_manager or self.gimbal_configure_accepted:
            return

        now_s = now_us * 1e-6
        if (
            self.last_gimbal_configure_time_s is not None
            and now_s - self.last_gimbal_configure_time_s
            < self.configure_retry_period_s
        ):
            return

        if (
            self.configure_max_attempts > 0
            and self.gimbal_configure_attempts >= self.configure_max_attempts
        ):
            if not self.warned_gimbal_configure_max_attempts:
                self.get_logger().warn(
                    "Gimbal manager configure was not accepted after "
                    f"{self.gimbal_configure_attempts} attempt(s); "
                    "setpoints may be ignored by PX4"
                )
                self.warned_gimbal_configure_max_attempts = True
            return

        msg = VehicleCommand()
        msg.timestamp = now_us
        msg.command = self._gimbal_configure_command_id()
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
        self.gimbal_configure_attempts += 1
        self.last_gimbal_configure_time_s = now_s

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
        msg.command = self._gimbal_pitchyaw_command_id()
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

    def _gimbal_configure_command_id(self) -> int:
        return int(
            getattr(
                VehicleCommand,
                "VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE",
                self.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            )
        )

    def _gimbal_pitchyaw_command_id(self) -> int:
        return int(
            getattr(
                VehicleCommand,
                "VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW",
                self.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            )
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def move_toward(value: float, target: float, max_step: float) -> float:
    if max_step <= 0.0:
        return value
    if value < target:
        return min(value + max_step, target)
    if value > target:
        return max(value - max_step, target)
    return value


def squared_image_distance(a: SelectedDetection, b: SelectedDetection) -> float:
    dx = a.center_x - b.center_x
    dy = a.center_y - b.center_y
    return dx * dx + dy * dy


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
