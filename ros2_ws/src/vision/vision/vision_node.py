from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from bridge_interfaces.msg import VisionDetection, VisionDetectionArray
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from vision.traffic_light import classify_traffic_light_color


def _clamp_box(x: int, y: int, w: int, h: int, image_width: int, image_height: int) -> tuple[int, int, int, int] | None:
    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(image_width, x + w)
    y1 = min(image_height, y + h)
    width = x1 - x0
    height = y1 - y0
    if width <= 0 or height <= 0:
        return None
    return x0, y0, width, height


@dataclass
class DetectionAttribute:
    name: str
    value: str
    score: float


@dataclass
class DetectionRecord:
    class_name: str
    confidence: float
    x: int
    y: int
    width: int
    height: int
    attributes: list[DetectionAttribute] = field(default_factory=list)

    def add_attribute(self, name: str, value: str, score: float) -> None:
        self.attributes.append(DetectionAttribute(name=name, value=value, score=float(score)))


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")

        share_dir = Path(get_package_share_directory("vision"))
        data_dir = share_dir / "data"
        source_data_dir = Path("/ros2_ws/src/vision/data")
        default_weights_path = (
            source_data_dir / "yolov4.weights"
            if source_data_dir.is_dir()
            else data_dir / "yolov4.weights"
        )

        self.declare_parameter("camera_device", "/dev/video10")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 15.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("model_cfg_path", str(data_dir / "yolov4.cfg"))
        self.declare_parameter("model_weights_path", str(default_weights_path))
        self.declare_parameter("class_names_path", str(data_dir / "obj.names"))
        self.declare_parameter("model_input_width", 416)
        self.declare_parameter("model_input_height", 416)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("nms_threshold", 0.4)
        self.declare_parameter("enable_traffic_light_color", True)
        self.declare_parameter("reconnect_delay_sec", 1.0)
        self.declare_parameter("log_interval_sec", 5.0)

        self._camera_device = str(self.get_parameter("camera_device").value)
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        self._camera_fps = float(self.get_parameter("camera_fps").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self._nms_threshold = float(self.get_parameter("nms_threshold").value)
        self._model_input_width = int(self.get_parameter("model_input_width").value)
        self._model_input_height = int(self.get_parameter("model_input_height").value)
        self._enable_traffic_light_color = bool(self.get_parameter("enable_traffic_light_color").value)
        self._reconnect_delay_sec = max(0.1, float(self.get_parameter("reconnect_delay_sec").value))
        self._log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._publisher = self.create_publisher(VisionDetectionArray, "/vision/detections", 10)
        self._capture: cv2.VideoCapture | None = None
        self._camera_connected = False
        self._last_loop_summary = 0.0

        cfg_path = Path(str(self.get_parameter("model_cfg_path").value))
        weights_path = Path(str(self.get_parameter("model_weights_path").value))
        names_path = Path(str(self.get_parameter("class_names_path").value))

        self._class_names = self._load_class_names(names_path)
        self._net = self._load_network(cfg_path, weights_path)
        self._output_layers = self._resolve_output_layers()

        self.get_logger().info(
            "Loaded YOLO model cfg=%s weights=%s classes=%d"
            % (cfg_path, weights_path, len(self._class_names))
        )

    def _load_class_names(self, path: Path) -> list[str]:
        if not path.is_file():
            raise FileNotFoundError(f"class names file not found: {path}")
        with path.open("r", encoding="utf-8") as handle:
            return [line.strip() for line in handle if line.strip()]

    def _load_network(self, cfg_path: Path, weights_path: Path):
        if not cfg_path.is_file():
            raise FileNotFoundError(f"YOLO cfg file not found: {cfg_path}")
        if not weights_path.is_file():
            raise FileNotFoundError(
                "YOLO weights file not found: %s. Copy the weights to "
                "ros2_ws/src/vision/data/yolov4.weights or pass "
                "model_weights_path:=/path/to/yolov4.weights."
                % weights_path
            )
        net = cv2.dnn.readNetFromDarknet(str(cfg_path), str(weights_path))
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return net

    def _resolve_output_layers(self) -> list[str]:
        layer_names = self._net.getLayerNames()
        raw_layers = self._net.getUnconnectedOutLayers()
        indices = np.array(raw_layers).reshape(-1)
        return [layer_names[int(index) - 1] for index in indices]

    def _release_camera(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        if self._camera_connected:
            self._camera_connected = False
            self.get_logger().warn("Vision camera disconnected; waiting for %s" % self._camera_device)

    def _ensure_camera(self) -> bool:
        if self._capture is not None and self._capture.isOpened():
            return True

        self._release_camera()
        capture = cv2.VideoCapture(self._camera_device)
        if not capture.isOpened():
            self.get_logger().warn(
                "Waiting for camera device %s"
                % self._camera_device,
                throttle_duration_sec=self._log_interval_sec,
            )
            capture.release()
            time.sleep(self._reconnect_delay_sec)
            return False

        if self._camera_width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._camera_width))
        if self._camera_height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._camera_height))
        if self._camera_fps > 0.0:
            capture.set(cv2.CAP_PROP_FPS, self._camera_fps)

        self._capture = capture
        self._camera_connected = True
        self.get_logger().info(
            "Connected vision camera %s at requested %dx%d @ %.1f fps"
            % (self._camera_device, self._camera_width, self._camera_height, self._camera_fps)
        )
        return True

    def _infer(self, frame: np.ndarray) -> tuple[list[list[int]], list[int], list[float]]:
        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=1.0 / 255.0,
            size=(self._model_input_width, self._model_input_height),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False,
        )
        self._net.setInput(blob)
        outputs = self._net.forward(self._output_layers)

        image_height, image_width = frame.shape[:2]
        boxes: list[list[int]] = []
        class_ids: list[int] = []
        confidences: list[float] = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = int(np.argmax(scores))
                confidence = float(scores[class_id])
                if confidence < self._confidence_threshold:
                    continue

                center_x = int(detection[0] * image_width)
                center_y = int(detection[1] * image_height)
                width = int(detection[2] * image_width)
                height = int(detection[3] * image_height)
                x = int(center_x - width / 2)
                y = int(center_y - height / 2)
                clamped = _clamp_box(x, y, width, height, image_width, image_height)
                if clamped is None:
                    continue

                x, y, width, height = clamped
                boxes.append([x, y, width, height])
                class_ids.append(class_id)
                confidences.append(confidence)

        return boxes, class_ids, confidences

    def _decode_detections(
        self,
        boxes: list[list[int]],
        class_ids: list[int],
        confidences: list[float],
    ) -> list[DetectionRecord]:
        records: list[DetectionRecord] = []
        if not boxes:
            return records

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self._confidence_threshold, self._nms_threshold)
        flat_indices = np.array(indices).reshape(-1) if len(indices) > 0 else np.array([], dtype=int)
        for index in flat_indices:
            class_id = class_ids[int(index)]
            class_name = self._class_names[class_id] if 0 <= class_id < len(self._class_names) else f"class_{class_id}"
            x, y, width, height = boxes[int(index)]
            records.append(
                DetectionRecord(
                    class_name=class_name,
                    confidence=float(confidences[int(index)]),
                    x=int(x),
                    y=int(y),
                    width=int(width),
                    height=int(height),
                )
            )
        return records

    def _build_detection_msg(self, record: DetectionRecord) -> VisionDetection:
        detection = VisionDetection()
        detection.class_name = record.class_name
        detection.confidence = float(record.confidence)
        detection.x = int(record.x)
        detection.y = int(record.y)
        detection.width = int(record.width)
        detection.height = int(record.height)
        for attribute in record.attributes:
            detection.attribute_names.append(attribute.name)
            detection.attribute_values.append(attribute.value)
            detection.attribute_scores.append(float(attribute.score))
        return detection

    def _build_detection_array_msg(
        self,
        capture_stamp,
        image_width: int,
        image_height: int,
        records: list[DetectionRecord],
    ) -> VisionDetectionArray:
        message = VisionDetectionArray()
        message.header.stamp = capture_stamp
        message.header.frame_id = "vision_camera"
        message.image_width = int(image_width)
        message.image_height = int(image_height)
        for record in records:
            message.detections.append(self._build_detection_msg(record))
        return message

    def run(self) -> None:
        period = 1.0 / self._publish_rate_hz if self._publish_rate_hz > 0.0 else 0.0
        next_cycle = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self._ensure_camera():
                next_cycle = time.monotonic()
                continue

            ok, frame = self._capture.read()
            if not ok or frame is None:
                self.get_logger().warn(
                    "Failed to read a frame from %s; reconnecting"
                    % self._camera_device,
                    throttle_duration_sec=self._log_interval_sec,
                )
                self._release_camera()
                time.sleep(self._reconnect_delay_sec)
                next_cycle = time.monotonic()
                continue

            capture_stamp = self.get_clock().now().to_msg()
            inference_start = time.monotonic()
            try:
                boxes, class_ids, confidences = self._infer(frame)
                detections = self._decode_detections(boxes, class_ids, confidences)

                for detection in detections:
                    object_crop = frame[
                        detection.y : detection.y + detection.height,
                        detection.x : detection.x + detection.width,
                    ]

                    if self._enable_traffic_light_color and detection.class_name == "trafficlight":
                        traffic_light_crop = object_crop
                        color_label, color_score = classify_traffic_light_color(traffic_light_crop)
                        detection.add_attribute("color", color_label, color_score)
                    elif detection.class_name == "face":
                        face_crop = object_crop
                        # TODO(student): analyze the face crop and attach your own
                        # attributes here, for example gender or customer type.
                        _ = face_crop
                        pass
                    elif detection.class_name == "my_object":
                        custom_object_crop = object_crop
                        # TODO(student): add custom object-specific checks here.
                        _ = custom_object_crop
                        pass

                message = self._build_detection_array_msg(
                    capture_stamp=capture_stamp,
                    image_width=frame.shape[1],
                    image_height=frame.shape[0],
                    records=detections,
                )
                self._publisher.publish(message)
                detection_count = len(message.detections)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Vision inference failed for one frame: {exc}")
                detection_count = 0
            inference_ms = (time.monotonic() - inference_start) * 1000.0

            now = time.monotonic()
            if now - self._last_loop_summary >= self._log_interval_sec:
                self._last_loop_summary = now
                self.get_logger().info(
                    "Vision frame %dx%d inference=%.1fms detections=%d"
                    % (frame.shape[1], frame.shape[0], inference_ms, detection_count)
                )

            if period > 0.0:
                next_cycle += period
                sleep_time = next_cycle - time.monotonic()
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                else:
                    next_cycle = time.monotonic()

        self._release_camera()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
