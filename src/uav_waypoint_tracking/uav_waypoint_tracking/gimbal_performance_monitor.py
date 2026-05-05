#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray


@dataclass
class SelectedDetection:
    """Target selected for performance evaluation."""

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


class GimbalPerformanceMonitor(Node):
    """Publish core gimbal visual-servo performance metrics."""

    def __init__(self) -> None:
        super().__init__("gimbal_performance_monitor")

        self.declare_parameter("detections_topic", "/x500_0/yolo/tracks")
        self.declare_parameter(
            "tracking_active_topic",
            "/x500_0/gimbal_target_tracker/tracking_active",
        )
        self.declare_parameter("camera_info_topic", "/x500_0/camera/camera_info")
        self.declare_parameter(
            "metrics_topic",
            "/x500_0/gimbal_performance/metrics",
        )
        self.declare_parameter("target_class_id", "")
        self.declare_parameter("target_track_id", "")
        self.declare_parameter("lock_target_track", True)
        self.declare_parameter("min_score", 0.35)
        self.declare_parameter("lost_timeout_s", 0.8)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("tracking_active_timeout_s", 0.5)
        self.declare_parameter("detections_timeout_s", 0.5)
        self.declare_parameter("fallback_fx_px", 410.93927419797166)
        self.declare_parameter("fallback_fy_px", 410.93927419797166)
        self.declare_parameter("fallback_cx_px", 640.0)
        self.declare_parameter("fallback_cy_px", 360.0)
        self.declare_parameter("fallback_image_width", 1280.0)
        self.declare_parameter("fallback_image_height", 720.0)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.tracking_active_topic = str(
            self.get_parameter("tracking_active_topic").value
        )
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.metrics_topic = str(self.get_parameter("metrics_topic").value)
        self.target_class_id = str(self.get_parameter("target_class_id").value).strip()
        self.target_track_id = str(self.get_parameter("target_track_id").value).strip()
        self.lock_target_track = bool(self.get_parameter("lock_target_track").value)
        self.min_score = float(self.get_parameter("min_score").value)
        self.lost_timeout_s = max(
            0.0,
            float(self.get_parameter("lost_timeout_s").value),
        )
        self.publish_rate_hz = max(
            0.2,
            float(self.get_parameter("publish_rate_hz").value),
        )
        self.tracking_active_timeout_s = max(
            0.0,
            float(self.get_parameter("tracking_active_timeout_s").value),
        )
        self.detections_timeout_s = max(
            0.0,
            float(self.get_parameter("detections_timeout_s").value),
        )
        self.camera_intrinsics = self._fallback_intrinsics()

        self.start_time_s = self._now_s()
        self.last_integral_time_s = self.start_time_s

        self.raw_tracking_active = False
        self.effective_tracking_active = False
        self.last_tracking_msg_time_s: float | None = None
        self.has_been_tracking = False
        self.lost_start_time_s: float | None = None
        self.last_reacquire_time_s: float | None = None
        self.tracking_active_time_s = 0.0

        self.last_detection: SelectedDetection | None = None
        self.last_detection_time_s: float | None = None
        self.target_missing_since_s: float | None = None
        self.locked_track_id = self.target_track_id or None
        self.last_pixel_error_px: float | None = None
        self.pixel_error_squared_sum = 0.0
        self.pixel_error_sample_count = 0

        self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self._detections_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.tracking_active_topic,
            self._tracking_active_callback,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10,
        )
        self.metrics_pub = self.create_publisher(
            DiagnosticArray,
            self.metrics_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._timer_callback,
        )

        target_filter = self.target_class_id or "all classes"
        track_filter = self.target_track_id or "auto-lock"
        self.get_logger().info(
            "Gimbal performance monitor ready: "
            f"detections={self.detections_topic}, "
            f"active={self.tracking_active_topic}, "
            f"camera_info={self.camera_info_topic}, metrics={self.metrics_topic}, "
            f"class={target_filter}, track={track_filter}"
        )

    def _detections_callback(self, msg: Detection2DArray) -> None:
        now_s = self._now_s()
        self._set_effective_tracking_active(
            self._fresh_raw_tracking_active(now_s),
            now_s,
        )

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

        pixel_error_px = self._pixel_error(selected)
        self.last_pixel_error_px = pixel_error_px
        if self.effective_tracking_active:
            self.pixel_error_squared_sum += pixel_error_px * pixel_error_px
            self.pixel_error_sample_count += 1

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

    def _tracking_active_callback(self, msg: Bool) -> None:
        now_s = self._now_s()
        self.raw_tracking_active = bool(msg.data)
        self.last_tracking_msg_time_s = now_s
        self._set_effective_tracking_active(self.raw_tracking_active, now_s)

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

    def _timer_callback(self) -> None:
        now_s = self._now_s()
        self._set_effective_tracking_active(
            self._fresh_raw_tracking_active(now_s),
            now_s,
        )
        self._release_auto_track_lock_if_lost(now_s)
        self._publish_metrics(now_s)

    def _release_auto_track_lock_if_lost(self, now_s: float) -> None:
        if self.target_track_id or not self.locked_track_id:
            return
        if self.last_detection_time_s is None:
            return
        if now_s - self.last_detection_time_s > self.lost_timeout_s:
            self.locked_track_id = None

    def _fresh_raw_tracking_active(self, now_s: float) -> bool:
        if self.last_tracking_msg_time_s is None:
            return False
        if self.tracking_active_timeout_s <= 0.0:
            return self.raw_tracking_active
        if now_s - self.last_tracking_msg_time_s > self.tracking_active_timeout_s:
            return False
        return self.raw_tracking_active

    def _set_effective_tracking_active(self, active: bool, now_s: float) -> None:
        self._accumulate_tracking_time(now_s)
        if active == self.effective_tracking_active:
            return

        was_active = self.effective_tracking_active
        self.effective_tracking_active = active

        if was_active and not active:
            if self.has_been_tracking:
                self.lost_start_time_s = now_s
            return

        if not was_active and active:
            if self.lost_start_time_s is not None:
                self.last_reacquire_time_s = now_s - self.lost_start_time_s
                self.lost_start_time_s = None
            self.has_been_tracking = True

    def _accumulate_tracking_time(self, now_s: float) -> None:
        dt_s = max(0.0, now_s - self.last_integral_time_s)
        if self.effective_tracking_active:
            self.tracking_active_time_s += dt_s
        self.last_integral_time_s = now_s

    def _publish_metrics(self, now_s: float) -> None:
        elapsed_s = max(0.0, now_s - self.start_time_s)
        current_pixel_error_px = self._current_pixel_error(now_s)
        rms_pixel_error_px = self._rms_pixel_error()
        tracking_rate = (
            self.tracking_active_time_s / elapsed_s if elapsed_s > 0.0 else 0.0
        )
        reacquire_time_s = self.last_reacquire_time_s

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "gimbal_performance_monitor"
        status.hardware_id = "x500_0_gimbal"
        status.level = DiagnosticStatus.OK
        status.message = "metrics active"
        if self.last_tracking_msg_time_s is None:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for tracking_active"
        elif self.last_detection_time_s is None:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for selected target"

        status.values = [
            self._value("current_pixel_error_px", current_pixel_error_px),
            self._value("rms_pixel_error_px", rms_pixel_error_px),
            self._value("tracking_rate", tracking_rate),
            self._value("last_reacquire_time_s", reacquire_time_s),
        ]
        msg.status.append(status)
        self.metrics_pub.publish(msg)

    def _current_pixel_error(self, now_s: float) -> float | None:
        if self.last_pixel_error_px is None or self.last_detection_time_s is None:
            return None
        if not self.effective_tracking_active:
            return None
        if self.detections_timeout_s <= 0.0:
            return self.last_pixel_error_px
        if now_s - self.last_detection_time_s > self.detections_timeout_s:
            return None
        return self.last_pixel_error_px

    def _rms_pixel_error(self) -> float | None:
        if self.pixel_error_sample_count == 0:
            return None
        mean_square = self.pixel_error_squared_sum / self.pixel_error_sample_count
        return math.sqrt(mean_square)

    def _pixel_error(self, detection: SelectedDetection) -> float:
        intrinsics = self.camera_intrinsics
        image_width = max(intrinsics.width, 1.0)
        image_height = max(intrinsics.height, 1.0)
        center_x = clamp(detection.center_x, 0.0, image_width)
        center_y = clamp(detection.center_y, 0.0, image_height)
        return math.hypot(center_x - intrinsics.cx, center_y - intrinsics.cy)

    def _fallback_intrinsics(self) -> CameraIntrinsics:
        return CameraIntrinsics(
            fx=max(1.0, float(self.get_parameter("fallback_fx_px").value)),
            fy=max(1.0, float(self.get_parameter("fallback_fy_px").value)),
            cx=float(self.get_parameter("fallback_cx_px").value),
            cy=float(self.get_parameter("fallback_cy_px").value),
            width=max(1.0, float(self.get_parameter("fallback_image_width").value)),
            height=max(1.0, float(self.get_parameter("fallback_image_height").value)),
            source="fallback",
        )

    @staticmethod
    def _value(key: str, value: float | None) -> KeyValue:
        item = KeyValue()
        item.key = key
        item.value = "nan" if value is None else f"{value:.6f}"
        return item

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def squared_image_distance(a: SelectedDetection, b: SelectedDetection) -> float:
    dx = a.center_x - b.center_x
    dy = a.center_y - b.center_y
    return dx * dx + dy * dy


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GimbalPerformanceMonitor()
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
