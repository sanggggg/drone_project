import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np

def get_backend(tag):
    tag = (tag or "FFMPEG").upper()
    return cv2.CAP_FFMPEG if tag == "FFMPEG" else cv2.CAP_GSTREAMER

class RTSPImagePublisher(Node):
    """
    ANAFI AI: rtsp://192.168.53.1/live -> /anafi_ai/image_raw 퍼블리시
    synthetic:=true 면 카메라 없이도 ArUco가 움직이는 합성 영상을 퍼블리시
    """
    def __init__(self):
        super().__init__("rtsp_image_publisher")
        # 파라미터
        self.declare_parameter("uri", "rtsp://192.168.53.1/live")
        self.declare_parameter("backend", "FFMPEG")     # FFMPEG 또는 GST
        self.declare_parameter("frame_id", "anafi_ai_camera")
        self.declare_parameter("target_fps", 30.0)
        self.declare_parameter("width", 0)              # 0이면 설정 안 함
        self.declare_parameter("height", 0)
        self.declare_parameter("synthetic", False)
        self.declare_parameter("aruco_id", 0)
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("syn_width",  960)
        self.declare_parameter("syn_height", 540)

        self.uri         = self.get_parameter("uri").get_parameter_value().string_value
        self.backend     = self.get_parameter("backend").get_parameter_value().string_value
        self.frame_id    = self.get_parameter("frame_id").get_parameter_value().string_value
        self.target_fps  = float(self.get_parameter("target_fps").get_parameter_value().double_value)
        self.req_w       = int(self.get_parameter("width").get_parameter_value().integer_value)
        self.req_h       = int(self.get_parameter("height").get_parameter_value().integer_value)
        self.synthetic   = self.get_parameter("synthetic").get_parameter_value().bool_value
        self.aruco_id    = int(self.get_parameter("aruco_id").get_parameter_value().integer_value)
        self.aruco_dict  = self.get_parameter("aruco_dict").get_parameter_value().string_value
        self.syn_w       = int(self.get_parameter("syn_width").get_parameter_value().integer_value)
        self.syn_h       = int(self.get_parameter("syn_height").get_parameter_value().integer_value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Image, "/anafi_ai/image_raw", qos)
        self.bridge = CvBridge()

        self._stop = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.get_logger().info(f"RTSP publisher started. synthetic={self.synthetic}, uri={self.uri}")

    def _open_rtsp(self):
        cap = cv2.VideoCapture(self.uri, get_backend(self.backend))
        if self.req_w > 0: cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.req_w)
        if self.req_h > 0: cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.req_h)
        cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        return cap

    def _run(self):
        dt = 1.0 / max(self.target_fps, 1.0)
        if self.synthetic:
            # 합성: 움직이는 ArUco
            try:
                aruco = cv2.aruco
                aruco_dict = getattr(aruco, self.aruco_dict)
                aruco_dict = aruco.getPredefinedDictionary(aruco_dict)
            except Exception:
                aruco = None; aruco_dict = None

            t0 = time.time()
            while rclpy.ok() and not self._stop:
                img = np.zeros((self.syn_h, self.syn_w, 3), dtype=np.uint8)
                if aruco is not None and aruco_dict is not None:
                    # 마커 크기와 위치를 시간에 따라 이동
                    sz = int(80 + 40 * math.sin((time.time()-t0)*1.3))
                    marker = aruco.generateImageMarker(aruco_dict, self.aruco_id, sz)
                    y = int(self.syn_h*0.5 + (self.syn_h*0.3)*math.sin((time.time()-t0)*0.6)) - sz//2
                    x = int(self.syn_w*0.5 + (self.syn_w*0.3)*math.cos((time.time()-t0)*0.8)) - sz//2
                    y = max(0, min(self.syn_h-sz, y)); x = max(0, min(self.syn_w-sz, x))
                    img[y:y+sz, x:x+sz, 0] = marker
                    img[y:y+sz, x:x+sz, 1] = marker
                    img[y:y+sz, x:x+sz, 2] = marker
                msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                self.pub.publish(msg)
                time.sleep(dt)
            return

        # RTSP 모드
        cap = self._open_rtsp()
        if not cap.isOpened():
            self.get_logger().error("Could not open RTSP stream. Check URI/network.")
            return
        while rclpy.ok() and not self._stop:
            ok, frame = cap.read()
            if not ok:
                self.get_logger().warn("RTSP read failed, retrying...")
                cap.release()
                time.sleep(0.2)
                cap = self._open_rtsp()
                continue
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)
            time.sleep(dt)
        cap.release()

    def destroy_node(self):
        self._stop = True
        try: self.thread.join(timeout=1.0)
        except Exception: pass
        super().destroy_node()

def main():
    rclpy.init()
    node = RTSPImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
