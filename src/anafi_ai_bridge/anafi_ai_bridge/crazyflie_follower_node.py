import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# cv_bridge & OpenCV(aruco) 필요
from cv_bridge import CvBridge, CvBridgeError
import cv2


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class CrazyflieFollower(Node):
    """
    카메라에서 Crazyflie(앞쪽 마커)를 검출해 /anafi/cmd_vel 로 추적 명령 퍼블리시.
    기본: ArUco 4x4_50 ID=0 사용(aruco_size_px를 목표 픽셀 크기로 유지).
    """
    def __init__(self):
        super().__init__("crazyflie_follower")

        # ---- Parameters
        self.declare_parameter("image_topic", "/anafi_ai/image_raw")
        self.declare_parameter("cmd_topic", "/anafi/cmd_vel")
        self.declare_parameter("ctrl_rate_hz", 20.0)

        # 제어 게인/한계
        self.declare_parameter("k_yaw", 1.2)        # [rad/s] @ full screen error
        self.declare_parameter("k_vx",  1.5)        # [m/s]   size error
        self.declare_parameter("k_vz",  1.0)        # [m/s]   vertical error
        self.declare_parameter("max_vx", 1.5)
        self.declare_parameter("max_vz", 1.0)
        self.declare_parameter("max_yaw", 1.0)      # [rad/s]
        self.declare_parameter("target_px", 120.0)  # 목표 마커 크기(픽셀, 한 변 길이/대각 중 택1)
        self.declare_parameter("aruco_id", 0)
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("lost_timeout", 0.5) # [s] 검출 끊기면 정지
        self.declare_parameter("scan_when_lost", True) # 분실 시 아주 천천히 좌우 요 회전

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.cmd_topic   = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self.ctrl_rate_hz = float(self.get_parameter("ctrl_rate_hz").get_parameter_value().double_value)

        self.k_yaw = float(self.get_parameter("k_yaw").get_parameter_value().double_value)
        self.k_vx  = float(self.get_parameter("k_vx").get_parameter_value().double_value)
        self.k_vz  = float(self.get_parameter("k_vz").get_parameter_value().double_value)
        self.max_vx = float(self.get_parameter("max_vx").get_parameter_value().double_value)
        self.max_vz = float(self.get_parameter("max_vz").get_parameter_value().double_value)
        self.max_yaw = float(self.get_parameter("max_yaw").get_parameter_value().double_value)
        self.target_px = float(self.get_parameter("target_px").get_parameter_value().double_value)
        self.aruco_id = int(self.get_parameter("aruco_id").get_parameter_value().integer_value)
        self.aruco_dict_name = self.get_parameter("aruco_dict").get_parameter_value().string_value
        self.lost_timeout = float(self.get_parameter("lost_timeout").get_parameter_value().double_value)
        self.scan_when_lost = self.get_parameter("scan_when_lost").get_parameter_value().bool_value

        # QoS
        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.bridge = CvBridge()
        self.last_det_ts = 0.0
        self.last_cmd = Twist()

        # ArUco 준비
        try:
            aruco = cv2.aruco
        except AttributeError:
            self.get_logger().error("OpenCV가 ArUco 모듈을 포함하지 않습니다. opencv-contrib-python 설치 필요.")
            aruco = None
        self._aruco = aruco
        self._aruco_params = None
        self._aruco_dict = None
        if self._aruco is not None:
            try:
                self._aruco_dict = getattr(self._aruco, self.aruco_dict_name)
                self._aruco_dict = self._aruco.getPredefinedDictionary(self._aruco_dict)
                if hasattr(self._aruco, "DetectorParameters"):
                    self._aruco_params = self._aruco.DetectorParameters()
                else:
                    self._aruco_params = self._aruco.DetectorParameters_create()
            except Exception as e:
                self.get_logger().error(f"ArUco 초기화 실패: {e}")

        # 구독/퍼블리시/타이머
        self.sub = self.create_subscription(Image, self.image_topic, self._img_cb, qos_img)
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.timer = self.create_timer(1.0/max(self.ctrl_rate_hz, 1.0), self._ctrl_tick)

        self.get_logger().info(f"Follower ready. image: {self.image_topic}, cmd: {self.cmd_topic}")

    def _detect_aruco(self, frame_bgr):
        if self._aruco is None or self._aruco_dict is None or self._aruco_params is None:
            return None
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # OpenCV 버전 차이 호환
        corners, ids, _ = None, None, None
        try:
            if hasattr(self._aruco, "ArucoDetector"):
                det = self._aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
                corners, ids, _ = det.detectMarkers(gray)
            else:
                corners, ids, _ = self._aruco.detectMarkers(gray, self._aruco_dict, parameters=self._aruco_params)
        except Exception as e:
            self.get_logger().warn(f"ArUco detect 오류: {e}")
            return None

        if ids is None or len(ids) == 0:
            return None

        # 원하는 ID 우선 선택, 없으면 가장 큰 마커
        ids = ids.flatten()
        pick = -1
        for idx, mid in enumerate(ids):
            if int(mid) == self.aruco_id:
                pick = idx
                break
        if pick < 0:
            # 가장 큰 마커 선택
            areas = [cv2.contourArea(c.astype(np.float32)) for c in corners]
            pick = int(np.argmax(areas))

        c = corners[pick][0]  # 4x2
        cx = float(np.mean(c[:, 0]))
        cy = float(np.mean(c[:, 1]))
        # 크기 스칼라(가로 길이 평균)
        w = float((np.linalg.norm(c[0] - c[1]) + np.linalg.norm(c[2] - c[3])) * 0.5)
        return cx, cy, w

    def _img_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return

        h, w, _ = frame.shape
        det = self._detect_aruco(frame)
        if det is None:
            return

        cx, cy, wpx = det
        # 정규화 에러
        ex = (cx - w * 0.5) / (w * 0.5)        # [-1..1], +면 좌측(이미지 좌표 기준)
        ey = (cy - h * 0.5) / (h * 0.5)        # +면 아래쪽
        es = (self.target_px - wpx) / max(self.target_px, 1.0)

        # 제어(간단): yaw로 중심 맞추고, vx로 거리 맞추고, vz로 높이 맞춤
        cmd = Twist()
        cmd.linear.x  = clamp(self.k_vx  * es, -self.max_vx,  self.max_vx)
        cmd.linear.y  = 0.0
        cmd.linear.z  = clamp(-self.k_vz * ey, -self.max_vz,  self.max_vz)   # 이미지 아래(+ey)면 내려가므로 부호 반전
        cmd.angular.z = clamp(self.k_yaw * ex, -self.max_yaw, self.max_yaw)  # 좌(+ex)면 +yaw(반시계)

        self.last_cmd = cmd
        self.last_det_ts = time.time()

    def _ctrl_tick(self):
        # 데드맨: 최근 검출 없으면 정지(옵션: 느린 스캔)
        now = time.time()
        if (now - self.last_det_ts) > self.lost_timeout:
            cmd = Twist()
            if self.scan_when_lost:
                cmd.angular.z = 0.15  # 천천히 좌/우 스캔 (원하면 파라미터화 가능)
            self.pub.publish(cmd)
            return
        self.pub.publish(self.last_cmd)


def main():
    rclpy.init()
    node = CrazyflieFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
