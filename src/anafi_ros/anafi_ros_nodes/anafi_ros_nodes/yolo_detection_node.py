#!/usr/bin/env python3
# yolo_detection_node.py
#
# ROS2 Node for YOLO-based object detection on ANAFI camera images.
#
# - Subscribes:
#     /anafi/camera/image  (sensor_msgs/Image)
#
# - Publishes:
#     /anafi/yolo/image           (sensor_msgs/Image)  - Annotated image with bboxes
#     /anafi/yolo/detections      (std_msgs/String)    - Detection results as JSON
#     /anafi/yolo/image/compressed (sensor_msgs/CompressedImage)  - Compressed annotated image
#     /anafi/yolo/ocr_image       (sensor_msgs/Image)  - Preprocessed images used for OCR
#
# Parameters:
#     model           : YOLO model path or name (default: yolov8n.pt)
#     device          : torch device hint: '', 'cpu', 'cuda' (default: '')
#     confidence      : Detection confidence threshold (default: 0.25)
#     inference_rate  : Max inference rate in Hz (default: 5.0)
#     publish_compressed : Whether to publish compressed image (default: True)
#     camera_topic    : Camera topic to subscribe (default: camera/image)
#     ocr_enabled     : Enable OCR on detected regions (default: True)
#     ocr_classes     : Classes to run OCR on (default: [] = all classes, or full image if no detections)
#     ocr_lang        : Tesseract language (default: 'eng')
#     ocr_whitelist   : Character whitelist for OCR (default: '' = all characters)

import time
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header, String

import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    from cv_bridge import CvBridge
    CVBRIDGE_AVAILABLE = True
except ImportError:
    CVBRIDGE_AVAILABLE = False

# OCR utilities
try:
    from anafi_ros_nodes.ocr_utils import OCRProcessor, OCRBufferManager, crop_from_xyxy, validate_white_screen
    OCR_AVAILABLE = True
except ImportError:
    OCR_AVAILABLE = False


def _make_qos(depth=10, reliable=True):
    return QoSProfile(
        depth=depth,
        reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
        history=HistoryPolicy.KEEP_LAST,
    )


class YoloDetectionNode(Node):
    """
    ROS2 Node that performs YOLO object detection on camera images.
    
    Subscribes to camera images, runs YOLO inference at a configurable rate,
    and publishes annotated images along with detection results.
    
    Detection JSON format:
    {
        "timestamp": {"sec": int, "nanosec": int},
        "frame_id": str,
        "detections": [
            {
                "class_id": int,
                "class_name": str,
                "confidence": float,
                "bbox": {
                    "x1": float, "y1": float,
                    "x2": float, "y2": float,
                    "center_x": float, "center_y": float,
                    "width": float, "height": float
                }
            },
            ...
        ]
    }
    """

    def __init__(self):
        super().__init__('yolo_detection_node', namespace='/anafi')

        # Check dependencies
        if not CV2_AVAILABLE:
            self.get_logger().fatal("OpenCV (cv2) is not installed. Cannot run YOLO detection node.")
            raise RuntimeError("OpenCV not available")
        
        if not YOLO_AVAILABLE:
            self.get_logger().fatal("Ultralytics YOLO is not installed. Install with: pip install ultralytics")
            raise RuntimeError("YOLO not available")
        
        if not CVBRIDGE_AVAILABLE:
            self.get_logger().fatal("cv_bridge is not installed. Cannot convert ROS images.")
            raise RuntimeError("cv_bridge not available")

        # ---------- Parameters ----------
        self.declare_parameter('model', 'yolov8n.engine')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('inference_rate', 10)  # Hz
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('camera_topic', 'camera/image')
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('max_detections', 100)
        self.declare_parameter('classes', [62, 63])  # Empty list means all classes
        
        # OCR parameters
        self.declare_parameter('ocr_enabled', True)
        self.declare_parameter('ocr_classes', [])  # Empty = all classes or full image
        self.declare_parameter('ocr_buffer_size', 10)  # Buffer size for batch processing
        self.declare_parameter('ocr_timeout_sec', 4.0)  # Timeout for batch processing
        self.declare_parameter('ocr_confidence_threshold', 90.0)  # Min confidence to accept

        self.model_path = self.get_parameter('model').value
        self.device = self.get_parameter('device').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.inference_rate = float(self.get_parameter('inference_rate').value)
        self.publish_compressed = bool(self.get_parameter('publish_compressed').value)
        self.camera_topic = self.get_parameter('camera_topic').value
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.max_detections = int(self.get_parameter('max_detections').value)
        self.filter_classes = self.get_parameter('classes').value
        
        # OCR parameters
        self.ocr_enabled = bool(self.get_parameter('ocr_enabled').value)
        self.ocr_classes = self.get_parameter('ocr_classes').value
        self.ocr_buffer_size = int(self.get_parameter('ocr_buffer_size').value)
        self.ocr_timeout_sec = float(self.get_parameter('ocr_timeout_sec').value)
        self.ocr_confidence_threshold = float(self.get_parameter('ocr_confidence_threshold').value)
        
        # Calculate minimum interval between inferences
        self.min_inference_interval = 1.0 / self.inference_rate if self.inference_rate > 0 else 0.0

        # ---------- YOLO Model ----------
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        try:
            self.yolo = YOLO(self.model_path)
            if self.device:
                try:
                    self.yolo.to(self.device)
                    self.get_logger().info(f"Model moved to device: {self.device}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to move model to '{self.device}': {e}")
            self.get_logger().info(f"YOLO model loaded successfully: {self.model_path}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load YOLO model: {e}")
            raise RuntimeError(f"YOLO model load failed: {e}")

        # Get class names from model
        self.class_names = self.yolo.names if hasattr(self.yolo, 'names') else {}

        # ---------- CV Bridge ----------
        self.bridge = CvBridge()
        
        # ---------- OCR Processor and Buffer Manager ----------
        self.ocr = None
        self.ocr_buffer = None
        if self.ocr_enabled:
            if OCR_AVAILABLE:
                self.ocr = OCRProcessor(
                    model_name="microsoft/trocr-base-printed",
                    use_gpu=True,
                    preprocess=True,
                    logger=self.get_logger()
                )
                # Create buffer manager for batch OCR processing
                self.ocr_buffer = OCRBufferManager(
                    ocr_processor=self.ocr,
                    buffer_size=self.ocr_buffer_size,
                    timeout_sec=self.ocr_timeout_sec,
                    confidence_threshold=self.ocr_confidence_threshold,
                    logger=self.get_logger()
                )
                self.get_logger().info(
                    f"[OCR] Buffer initialized: size={self.ocr_buffer_size}, "
                    f"timeout={self.ocr_timeout_sec}s, conf_threshold={self.ocr_confidence_threshold}%"
                )
            else:
                self.get_logger().warn("OCR requested but ocr_utils module not available")

        # ---------- State ----------
        self._last_inference_time = 0.0
        self._frame_count = 0
        self._inference_count = 0
        self._lock = threading.Lock()
        self._fps = 0.0
        self._fps_update_time = time.time()
        self._fps_frame_count = 0

        # ---------- QoS Profiles ----------
        qos_image = _make_qos(depth=5, reliable=False)  # Best effort for image streaming
        qos_detection = _make_qos(depth=10, reliable=True)

        # ---------- Subscribers ----------
        self.sub_camera = self.create_subscription(
            Image,
            self.camera_topic,
            self._on_camera_image,
            qos_image
        )

        # ---------- Publishers ----------
        self.pub_annotated_image = self.create_publisher(
            Image, 'yolo/image', qos_image
        )
        self.pub_detections = self.create_publisher(
            String, 'yolo/detections', qos_detection
        )
        if self.publish_compressed:
            self.pub_compressed = self.create_publisher(
                CompressedImage, 'yolo/image/compressed', qos_image
            )
        
        # OCR publisher (Image)
        self.pub_ocr_image = self.create_publisher(
            Image, 'yolo/ocr_image', qos_image
        )

        # ---------- Logging ----------
        self.get_logger().info("=" * 60)
        self.get_logger().info("YOLO Detection Node Started")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Model:            {self.model_path}")
        self.get_logger().info(f"  Device:           {self.device if self.device else 'auto'}")
        self.get_logger().info(f"  Confidence:       {self.confidence}")
        self.get_logger().info(f"  Inference Rate:   {self.inference_rate} Hz")
        self.get_logger().info(f"  IOU Threshold:    {self.iou_threshold}")
        self.get_logger().info(f"  Camera Topic:     /anafi/{self.camera_topic}")
        self.get_logger().info(f"  Annotated Topic:  /anafi/yolo/image")
        self.get_logger().info(f"  Detections Topic: /anafi/yolo/detections (JSON)")
        self.get_logger().info(f"  OCR Topic:        /anafi/yolo/ocr_image")
        self.get_logger().info(f"  OCR Enabled:      {self.ocr_enabled and self.ocr is not None}")
        if self.ocr_enabled and self.ocr:
            self.get_logger().info(f"  OCR Classes:      {self.ocr_classes if self.ocr_classes else 'all/full image'}")
            self.get_logger().info(f"  OCR Buffer Size:  {self.ocr_buffer_size}")
            self.get_logger().info(f"  OCR Timeout:      {self.ocr_timeout_sec}s")
            self.get_logger().info(f"  OCR Conf Thresh:  {self.ocr_confidence_threshold}%")
        self.get_logger().info("=" * 60)

    def _on_camera_image(self, msg: Image):
        """
        Callback for camera image messages.
        Performs YOLO inference if enough time has passed since last inference.
        """
        self._frame_count += 1
        current_time = time.time()

        # Update FPS calculation
        self._fps_frame_count += 1
        if current_time - self._fps_update_time >= 1.0:
            self._fps = self._fps_frame_count / (current_time - self._fps_update_time)
            self._fps_frame_count = 0
            self._fps_update_time = current_time

        # Check if we should run inference based on rate limiting
        with self._lock:
            time_since_last = current_time - self._last_inference_time
            if time_since_last < self.min_inference_interval:
                return  # Skip this frame
            self._last_inference_time = current_time

        # Convert ROS Image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Run YOLO inference
        try:
            t = time.time()
            results = self.yolo.predict(
                cv_image,
                conf=self.confidence,
                iou=self.iou_threshold,
                max_det=self.max_detections,
                classes=self.filter_classes if self.filter_classes else None,
                verbose=False
            )
            self._inference_count += 1
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        # Process results
        if results and len(results) > 0:
            result = results[0]
            inference_time_ms = result.speed['inference']
            preprocess_time_ms = result.speed['preprocess']
            postprocess_time_ms = result.speed['postprocess']
            self.get_logger().info(f"Inference time: {inference_time_ms:.2f} ms")
            self.get_logger().info(f"Preprocess time: {preprocess_time_ms:.2f} ms")
            self.get_logger().info(f"Postprocess time: {postprocess_time_ms:.2f} ms")
            self.get_logger().info(f"Total time: {inference_time_ms + preprocess_time_ms + postprocess_time_ms:.2f} ms")
            
            # Get annotated image with bboxes
            annotated_image = result.plot()
            
            # Draw FPS on image
            self._draw_info(annotated_image)
            
            # Publish annotated image
            self._publish_annotated_image(annotated_image, msg.header)
            
            # Extract and publish detections as JSON
            detections_json = self._extract_detections_json(result, msg.header)
            detection_msg = String()
            detection_msg.data = detections_json
            self.pub_detections.publish(detection_msg)
            
            # Run OCR on detected regions or full image

            # keep = []
            # for i in range(len(result.boxes)):
            #     cid = int(result.boxes.cls[i].cpu().numpy())
            #     if cid == 62:  # 너가 원하는 class
            #         keep.append(i)

            # result.boxes = result.boxes[keep]

            if self.ocr is not None:
                self._run_ocr(cv_image, result, msg.header)
            
            # Log periodically
            if self._inference_count % 1 == 0:
                det_data = json.loads(detections_json)
                n_dets = len(det_data.get('detections', []))
                self.get_logger().info(
                    f"Inference #{self._inference_count}: {n_dets} detections, "
                    f"FPS: {self._fps:.1f}"
                )

    def _draw_info(self, img: np.ndarray):
        """Draw FPS and other info on the image."""
        info_text = f"FPS: {self._fps:.1f} | Inference: {self._inference_count}"
        
        # Background rectangle for better readability
        (text_width, text_height), baseline = cv2.getTextSize(
            info_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
        )
        cv2.rectangle(
            img,
            (5, 5),
            (15 + text_width, 30 + baseline),
            (0, 0, 0),
            -1
        )
        cv2.putText(
            img, info_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 255), 2, cv2.LINE_AA
        )

    def _publish_annotated_image(self, cv_image: np.ndarray, header: Header):
        """Publish annotated image in both raw and compressed formats."""
        try:
            # Publish raw image
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            ros_image.header = header
            ros_image.header.frame_id = 'yolo_detection'
            self.pub_annotated_image.publish(ros_image)

            # Publish compressed image
            if self.publish_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header = header
                compressed_msg.header.frame_id = 'yolo_detection'
                compressed_msg.format = 'jpeg'
                _, encoded = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                compressed_msg.data = encoded.tobytes()
                self.pub_compressed.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

    def _extract_detections_json(self, result, header: Header) -> str:
        """
        Extract detection results from YOLO output and convert to JSON string.
        
        Returns a JSON string with bounding boxes and class information.
        """
        detection_data = {
            "timestamp": {
                "sec": header.stamp.sec,
                "nanosec": header.stamp.nanosec
            },
            "frame_id": header.frame_id,
            "detections": []
        }

        if result.boxes is None or len(result.boxes) == 0:
            return json.dumps(detection_data)

        boxes = result.boxes
        
        for i in range(len(boxes)):
            # Get bounding box coordinates (x1, y1, x2, y2)
            box = boxes.xyxy[i].cpu().numpy()
            x1, y1, x2, y2 = box
            
            # Calculate center and size
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            width = x2 - x1
            height = y2 - y1
            
            # Get class info
            class_id = int(boxes.cls[i].cpu().numpy())
            confidence = float(boxes.conf[i].cpu().numpy())
            class_name = self.class_names.get(class_id, str(class_id))
            
            detection = {
                "class_id": class_id,
                "class_name": class_name,
                "confidence": round(confidence, 4),
                "bbox": {
                    "x1": round(float(x1), 2),
                    "y1": round(float(y1), 2),
                    "x2": round(float(x2), 2),
                    "y2": round(float(y2), 2),
                    "center_x": round(float(center_x), 2),
                    "center_y": round(float(center_y), 2),
                    "width": round(float(width), 2),
                    "height": round(float(height), 2)
                }
            }
            detection_data["detections"].append(detection)

        return json.dumps(detection_data)

    def _run_ocr(self, cv_image: np.ndarray, result, header: Header):
        """
        Buffer-based OCR processing on detected regions.
        
        Logic:
        1. Skip if no YOLO detection
        2. Add cropped images to buffer
        3. Process batch when buffer is full or timeout occurs
        4. Filter by confidence >= 90%, remove non-alphabet chars
        5. Return mode (most frequent) result
        
        Args:
            cv_image: Original BGR image
            result: YOLO detection result
            header: ROS message header
        """
        has_detections = result.boxes is not None and len(result.boxes) > 0
        
        if not has_detections:
            # No detections - skip OCR, check timeout for pending buffer
            self.get_logger().info("[OCR] No YOLO detection, skipping OCR")
            # Still check if buffer needs timeout processing
            if self.ocr_buffer:
                processed, mode_result = self.ocr_buffer.check_timeout()
                if processed and mode_result:
                    self.get_logger().info(f"[OCR] ★★★ FINAL RESULT (timeout): '{mode_result}' ★★★")
            return
        
        # Process detected regions
        boxes = result.boxes
        processed_any = False
        
        for i in range(len(boxes)):
            class_id = int(boxes.cls[i].cpu().numpy())
            class_name = self.class_names.get(class_id, str(class_id))
            confidence = float(boxes.conf[i].cpu().numpy())
            
            # Check if we should process this class
            if self.ocr_classes and class_name not in self.ocr_classes:
                continue
            
            processed_any = True
            
            # Get bounding box
            box = boxes.xyxy[i].cpu().numpy()
            x1, y1, x2, y2 = map(int, box)
            
            # Crop region
            cropped = crop_from_xyxy(cv_image, x1, y1, x2, y2, padding=5)
            
            if cropped is not None and cropped.size > 0:
                # Validate white screen ratio
                is_valid, white_ratio = validate_white_screen(cropped, min_white_ratio=0.3)
                if not is_valid:
                    self.get_logger().debug(f"[OCR] Skipping detection (white ratio: {white_ratio:.2f} < 0.3)")
                    continue
                
                # Add to buffer instead of immediate OCR
                if self.ocr_buffer:
                    should_process, mode_result = self.ocr_buffer.add_image(cropped)
                    
                    if should_process and mode_result:
                        # Batch processing completed, log final result
                        self.get_logger().info(f"[OCR] ★★★ FINAL RESULT: '{mode_result}' ★★★")
                    
                    # Publish cropped image for visualization
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(cropped, encoding='bgr8')
                        ros_image.header = header
                        ros_image.header.frame_id = f'ocr_{class_name}_{i}'
                        self.pub_ocr_image.publish(ros_image)
                    except Exception as e:
                        self.get_logger().error(f"Failed to publish OCR image: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YoloDetectionNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Failed to start YOLO detection node: {e}")
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    main()
