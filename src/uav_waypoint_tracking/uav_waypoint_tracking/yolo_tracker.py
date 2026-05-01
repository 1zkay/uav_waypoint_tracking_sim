from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


@dataclass(frozen=True)
class YoloTrack:
    """Single tracked object exported as a Detection2D message."""

    track_id: str
    class_id: str
    score: float
    center_x: float
    center_y: float
    width: float
    height: float


class YoloTracker(Node):
    """Run YOLO tracking with a persistent Ultralytics tracker backend.

    The default tracker backend is ByteTrack. Output remains
    vision_msgs/Detection2DArray so downstream nodes can consume either
    detector or tracker results with the same message type. For tracked objects,
    Detection2D.id is the persistent track id.
    """

    def __init__(self) -> None:
        super().__init__("yolo_tracker")

        self.declare_parameter("weights_path", "/home/zk/uav_waypoint_tracking_sim/yolov8s.pt")
        self.declare_parameter("image_topic", "/x500_0/camera/image_raw")
        self.declare_parameter("tracks_topic", "/x500_0/yolo/tracks")
        self.declare_parameter("annotated_image_topic", "/x500_0/yolo/tracks_image")
        self.declare_parameter("tracker_config", "bytetrack.yaml")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("max_detections", 100)
        self.declare_parameter("classes", "")
        self.declare_parameter("device", "")
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("publish_tracks", True)
        self.declare_parameter("publish_untracked_detections", False)

        self.weights_path = str(self.get_parameter("weights_path").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.tracks_topic = str(self.get_parameter("tracks_topic").value)
        self.annotated_image_topic = str(self.get_parameter("annotated_image_topic").value)
        self.tracker_config = str(self.get_parameter("tracker_config").value).strip()
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.classes = parse_classes(str(self.get_parameter("classes").value))
        self.device = str(self.get_parameter("device").value).strip()
        self.publish_annotated_image = bool(self.get_parameter("publish_annotated_image").value)
        self.publish_tracks = bool(self.get_parameter("publish_tracks").value)
        self.publish_untracked_detections = bool(
            self.get_parameter("publish_untracked_detections").value
        )

        if not Path(self.weights_path).is_file():
            raise RuntimeError(f"YOLO weights file does not exist: {self.weights_path}")

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required for yolo_tracker. Activate /home/zk/px4-venv "
                "or install it with: pip install ultralytics"
            ) from exc

        self.model = YOLO(self.weights_path)
        self.class_names = getattr(self.model, "names", {})
        self.bridge = CvBridge()

        self.tracks_pub = (
            self.create_publisher(Detection2DArray, self.tracks_topic, 10)
            if self.publish_tracks
            else None
        )
        self.annotated_image_pub = (
            self.create_publisher(Image, self.annotated_image_topic, 10)
            if self.publish_annotated_image
            else None
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        class_filter = "all classes" if self.classes is None else f"classes={self.classes}"
        self.get_logger().info(
            "YOLO tracker ready: "
            f"image={self.image_topic}, tracks={self.tracks_topic}, "
            f"annotated={self.annotated_image_topic}, weights={self.weights_path}, "
            f"tracker={self.tracker_config or 'ultralytics default'}, {class_filter}"
        )

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Failed to convert image: {exc}")
            return

        track_kwargs = {
            "source": frame,
            "conf": self.confidence_threshold,
            "iou": self.iou_threshold,
            "imgsz": self.image_size,
            "max_det": self.max_detections,
            "persist": True,
            "verbose": False,
        }
        if self.tracker_config:
            track_kwargs["tracker"] = self.tracker_config
        if self.classes is not None:
            track_kwargs["classes"] = self.classes
        if self.device:
            track_kwargs["device"] = self.device

        results = self.model.track(**track_kwargs)
        result = results[0]

        tracks = self._extract_tracks(result)
        if self.tracks_pub is not None:
            self.tracks_pub.publish(self._to_detection_array(msg, tracks))

        if self.annotated_image_pub is not None:
            annotated = result.plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)

    def _extract_tracks(self, result) -> list[YoloTrack]:
        tracks: list[YoloTrack] = []
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return tracks

        for index, box in enumerate(boxes):
            track_id = extract_track_id(box)
            if track_id is None and not self.publish_untracked_detections:
                continue

            xywh = box.xywh[0].detach().cpu().tolist()
            confidence = float(box.conf[0].detach().cpu())
            class_index = int(box.cls[0].detach().cpu())
            tracks.append(
                YoloTrack(
                    track_id=str(track_id if track_id is not None else f"untracked_{index}"),
                    class_id=str(self.class_names.get(class_index, class_index)),
                    score=confidence,
                    center_x=float(xywh[0]),
                    center_y=float(xywh[1]),
                    width=float(xywh[2]),
                    height=float(xywh[3]),
                )
            )
        return tracks

    def _to_detection_array(self, source_msg: Image, tracks: list[YoloTrack]) -> Detection2DArray:
        array_msg = Detection2DArray()
        array_msg.header = source_msg.header

        for track in tracks:
            detection_msg = Detection2D()
            detection_msg.header = source_msg.header
            detection_msg.id = track.track_id
            detection_msg.bbox.center.position.x = track.center_x
            detection_msg.bbox.center.position.y = track.center_y
            detection_msg.bbox.center.theta = 0.0
            detection_msg.bbox.size_x = track.width
            detection_msg.bbox.size_y = track.height

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = track.class_id
            hypothesis.hypothesis.score = track.score
            detection_msg.results.append(hypothesis)
            array_msg.detections.append(detection_msg)

        return array_msg


def extract_track_id(box) -> int | None:
    raw_id = getattr(box, "id", None)
    if raw_id is None:
        return None

    try:
        if len(raw_id) == 0:
            return None
    except TypeError:
        pass

    try:
        return int(raw_id[0].detach().cpu())
    except (AttributeError, IndexError, TypeError, ValueError):
        try:
            return int(raw_id)
        except (TypeError, ValueError):
            return None


def parse_classes(value: str) -> list[int] | None:
    value = value.strip()
    if not value:
        return None
    return [int(item.strip()) for item in value.split(",") if item.strip()]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloTracker()
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
