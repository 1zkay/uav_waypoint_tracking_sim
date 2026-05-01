#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import VehicleCommand
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray


@dataclass
class SelectedDetection:
    """Detection selected for the gimbal visual-servo loop."""

    class_id: str
    score: float
    center_x: float
    center_y: float
    width: float
    height: float


class GimbalTargetTracker(Node):
    """Keep a detected target close to the x500_0 gimbal camera image center.

    The node implements an image-plane visual-servo loop:
    1. subscribe to a Detection2DArray produced by ``yolo_detector``;
    2. compute normalized pixel error between target bbox center and image center;
    3. integrate bounded pitch/yaw rates;
    4. send PX4 gimbal manager pitch/yaw commands through VehicleCommand.
    """

    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000

    def __init__(self) -> None:
        super().__init__("gimbal_target_tracker")

        self.declare_parameter("detections_topic", "/x500_0/yolo/detections")
        self.declare_parameter("image_topic", "/x500_0/camera/image_raw")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("error_topic", "/x500_0/gimbal_target_tracker/error")
        self.declare_parameter("tracking_active_topic", "/x500_0/gimbal_target_tracker/tracking_active")

        self.declare_parameter("target_class_id", "")
        self.declare_parameter("min_score", 0.25)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("lost_timeout_s", 0.5)
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)
        self.declare_parameter("deadband_normalized", 0.03)

        self.declare_parameter("yaw_rate_gain_deg_s", 45.0)
        self.declare_parameter("pitch_rate_gain_deg_s", 35.0)
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
        self.declare_parameter("gimbal_manager_flags", 0.0)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.vehicle_command_topic = str(self.get_parameter("vehicle_command_topic").value)
        self.error_topic = str(self.get_parameter("error_topic").value)
        self.tracking_active_topic = str(self.get_parameter("tracking_active_topic").value)

        self.target_class_id = str(self.get_parameter("target_class_id").value).strip()
        self.min_score = float(self.get_parameter("min_score").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.lost_timeout_s = float(self.get_parameter("lost_timeout_s").value)
        self.image_width = float(self.get_parameter("image_width").value)
        self.image_height = float(self.get_parameter("image_height").value)
        self.deadband_normalized = float(self.get_parameter("deadband_normalized").value)

        self.yaw_rate_gain_deg_s = float(self.get_parameter("yaw_rate_gain_deg_s").value)
        self.pitch_rate_gain_deg_s = float(self.get_parameter("pitch_rate_gain_deg_s").value)
        self.max_yaw_rate_deg_s = float(self.get_parameter("max_yaw_rate_deg_s").value)
        self.max_pitch_rate_deg_s = float(self.get_parameter("max_pitch_rate_deg_s").value)
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
        self.gimbal_manager_flags = float(self.get_parameter("gimbal_manager_flags").value)

        self.last_detection: SelectedDetection | None = None
        self.last_detection_time_s: float | None = None
        self.last_update_time_s: float | None = None
        self.last_tracking_active: bool | None = None

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.command_pub = self.create_publisher(VehicleCommand, self.vehicle_command_topic, px4_qos)
        self.error_pub = self.create_publisher(Vector3Stamped, self.error_topic, 10)
        self.tracking_active_pub = self.create_publisher(Bool, self.tracking_active_topic, 10)

        self.create_subscription(Detection2DArray, self.detections_topic, self._detections_callback, 10)
        self.create_subscription(Image, self.image_topic, self._image_callback, qos_profile_sensor_data)

        timer_period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        target_filter = self.target_class_id or "highest-score detection"
        self.get_logger().info(
            "Gimbal target tracker ready: "
            f"detections={self.detections_topic}, image={self.image_topic}, "
            f"command={self.vehicle_command_topic}, target={target_filter}, rate={self.control_rate_hz:.1f} Hz"
        )

    def _image_callback(self, msg: Image) -> None:
        if msg.width > 0 and msg.height > 0:
            self.image_width = float(msg.width)
            self.image_height = float(msg.height)

    def _detections_callback(self, msg: Detection2DArray) -> None:
        selected = self._select_detection(msg.detections)
        if selected is None:
            return

        self.last_detection = selected
        self.last_detection_time_s = self._now_s()

    def _select_detection(self, detections: list[Detection2D]) -> SelectedDetection | None:
        best: SelectedDetection | None = None

        for detection in detections:
            class_id, score = self._best_hypothesis(detection)
            if score < self.min_score:
                continue
            if self.target_class_id and class_id != self.target_class_id:
                continue

            candidate = SelectedDetection(
                class_id=class_id,
                score=score,
                center_x=float(detection.bbox.center.position.x),
                center_y=float(detection.bbox.center.position.y),
                width=float(detection.bbox.size_x),
                height=float(detection.bbox.size_y),
            )
            if best is None or candidate.score > best.score:
                best = candidate

        return best

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

        if not active or self.last_detection is None:
            self._publish_gimbal_command(self._now_us(), self.pitch_deg, self.yaw_deg)
            return

        error_x, error_y = self._normalized_image_error(self.last_detection)
        error_x = self._apply_deadband(error_x)
        error_y = self._apply_deadband(error_y)

        yaw_rate_deg_s = clamp(
            self.yaw_error_sign * self.yaw_rate_gain_deg_s * error_x,
            -self.max_yaw_rate_deg_s,
            self.max_yaw_rate_deg_s,
        )
        pitch_rate_deg_s = clamp(
            self.pitch_error_sign * self.pitch_rate_gain_deg_s * error_y,
            -self.max_pitch_rate_deg_s,
            self.max_pitch_rate_deg_s,
        )

        self.yaw_deg = clamp(self.yaw_deg + yaw_rate_deg_s * dt_s, self.min_yaw_deg, self.max_yaw_deg)
        self.pitch_deg = clamp(self.pitch_deg + pitch_rate_deg_s * dt_s, self.min_pitch_deg, self.max_pitch_deg)

        now_us = self._now_us()
        self._publish_error(now_us, error_x, error_y, self.last_detection.score)
        self._publish_gimbal_command(now_us, self.pitch_deg, self.yaw_deg)

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

    def _normalized_image_error(self, detection: SelectedDetection) -> tuple[float, float]:
        half_width = max(self.image_width * 0.5, 1.0)
        half_height = max(self.image_height * 0.5, 1.0)
        error_x = (detection.center_x - half_width) / half_width
        error_y = (detection.center_y - half_height) / half_height
        return clamp(error_x, -1.0, 1.0), clamp(error_y, -1.0, 1.0)

    def _apply_deadband(self, value: float) -> float:
        return 0.0 if abs(value) < self.deadband_normalized else value

    def _publish_error(self, now_us: int, error_x: float, error_y: float, score: float) -> None:
        msg = Vector3Stamped()
        msg.header.stamp.sec = int(now_us // 1_000_000)
        msg.header.stamp.nanosec = int((now_us % 1_000_000) * 1000)
        msg.header.frame_id = "camera_image"
        msg.vector.x = float(error_x)
        msg.vector.y = float(error_y)
        msg.vector.z = float(score)
        self.error_pub.publish(msg)

    def _publish_tracking_active(self, active: bool) -> None:
        if self.last_tracking_active == active:
            return
        self.last_tracking_active = active
        msg = Bool()
        msg.data = active
        self.tracking_active_pub.publish(msg)

    def _publish_gimbal_command(self, now_us: int, pitch_deg: float, yaw_deg: float) -> None:
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

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


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
