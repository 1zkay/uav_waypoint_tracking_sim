from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray

try:
    import cv2
except ImportError as exc:  # pragma: no cover - dependency is provided by cv_bridge/OpenCV installs.
    raise RuntimeError(
        "OpenCV is required for yolo_annotator."
    ) from exc


@dataclass(frozen=True)
class AnnotationTrack:
    """Single detection rendered on the annotated image."""

    track_id: str
    class_id: str
    score: float
    center_x: float
    center_y: float
    width: float
    height: float


class YoloAnnotator(Node):
    """Render YOLO/BoT-SORT tracks in a separate visualization node."""

    def __init__(self) -> None:
        super().__init__("yolo_annotator")

        self.declare_parameter("image_topic", "/x500_0/camera/image_raw")
        self.declare_parameter("tracks_topic", "/x500_0/yolo/tracks")
        self.declare_parameter("annotated_image_topic", "/x500_0/yolo/tracks_image")
        self.declare_parameter("max_publish_hz", 15.0)
        self.declare_parameter("tracks_timeout_s", 0.5)
        self.declare_parameter("publish_without_tracks", False)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.tracks_topic = str(self.get_parameter("tracks_topic").value)
        self.annotated_image_topic = str(
            self.get_parameter("annotated_image_topic").value
        )
        self.max_publish_hz = max(0.0, float(self.get_parameter("max_publish_hz").value))
        self.tracks_timeout_s = max(
            0.0,
            float(self.get_parameter("tracks_timeout_s").value),
        )
        self.publish_without_tracks = bool(
            self.get_parameter("publish_without_tracks").value
        )

        self.bridge = CvBridge()
        self.latest_tracks: list[AnnotationTrack] = []
        self.latest_tracks_time_s: float | None = None
        self.last_publish_time_s = 0.0

        self.create_subscription(
            Detection2DArray,
            self.tracks_topic,
            self._tracks_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.annotated_pub = self.create_publisher(
            Image,
            self.annotated_image_topic,
            10,
        )

        publish_limit = (
            "unlimited"
            if self.max_publish_hz <= 0.0
            else f"{self.max_publish_hz:.1f} Hz"
        )
        self.get_logger().info(
            "YOLO annotator ready: "
            f"image={self.image_topic}, tracks={self.tracks_topic}, "
            f"annotated={self.annotated_image_topic}, max_publish={publish_limit}"
        )

    def _tracks_callback(self, msg: Detection2DArray) -> None:
        self.latest_tracks = [track_from_detection(detection) for detection in msg.detections]
        self.latest_tracks_time_s = self._now_s()

    def _image_callback(self, msg: Image) -> None:
        now_s = self._now_s()
        if not self._ready_to_publish(now_s):
            return

        tracks = self._fresh_tracks(now_s)
        if not tracks and not self.publish_without_tracks:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Failed to convert image: {exc}")
            return

        for track in tracks:
            draw_track(frame, track)

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Failed to convert annotated image: {exc}")
            return

        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)
        self.last_publish_time_s = time.monotonic()

    def _ready_to_publish(self, _now_s: float) -> bool:
        if self.max_publish_hz <= 0.0:
            return True
        return time.monotonic() - self.last_publish_time_s >= 1.0 / self.max_publish_hz

    def _fresh_tracks(self, now_s: float) -> list[AnnotationTrack]:
        if self.latest_tracks_time_s is None:
            return []
        if self.tracks_timeout_s <= 0.0:
            return self.latest_tracks
        if now_s - self.latest_tracks_time_s > self.tracks_timeout_s:
            return []
        return self.latest_tracks

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def track_from_detection(detection: Detection2D) -> AnnotationTrack:
    class_id, score = best_hypothesis(detection)
    return AnnotationTrack(
        track_id=str(detection.id),
        class_id=class_id,
        score=score,
        center_x=float(detection.bbox.center.position.x),
        center_y=float(detection.bbox.center.position.y),
        width=float(detection.bbox.size_x),
        height=float(detection.bbox.size_y),
    )


def best_hypothesis(detection: Detection2D) -> tuple[str, float]:
    best_class_id = ""
    best_score = 0.0
    for result in detection.results:
        score = float(result.hypothesis.score)
        if score >= best_score:
            best_score = score
            best_class_id = str(result.hypothesis.class_id)
    return best_class_id, best_score


def draw_track(frame, track: AnnotationTrack) -> None:
    height, width = frame.shape[:2]
    x1 = clamp_int(round(track.center_x - track.width / 2.0), 0, width - 1)
    y1 = clamp_int(round(track.center_y - track.height / 2.0), 0, height - 1)
    x2 = clamp_int(round(track.center_x + track.width / 2.0), 0, width - 1)
    y2 = clamp_int(round(track.center_y + track.height / 2.0), 0, height - 1)
    color = (0, 180, 255)

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    label_parts = []
    if track.class_id:
        label_parts.append(track.class_id)
    if track.track_id:
        label_parts.append(f"id:{track.track_id}")
    label_parts.append(f"{track.score:.2f}")
    label = " ".join(label_parts)
    draw_label(frame, label, x1, y1, color)


def draw_label(frame, label: str, x: int, y: int, color: tuple[int, int, int]) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 1
    text_size, baseline = cv2.getTextSize(label, font, scale, thickness)
    text_w, text_h = text_size
    top = max(0, y - text_h - baseline - 6)
    bottom = min(frame.shape[0] - 1, top + text_h + baseline + 6)
    right = min(frame.shape[1] - 1, x + text_w + 8)

    cv2.rectangle(frame, (x, top), (right, bottom), color, -1)
    cv2.putText(
        frame,
        label,
        (x + 4, bottom - baseline - 3),
        font,
        scale,
        (0, 0, 0),
        thickness,
        cv2.LINE_AA,
    )


def clamp_int(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloAnnotator()
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
