#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

class TakeoffAndHoverTest(Node):
    def __init__(self,
                 z_target=0.40,
                 hover_hz=50.0,
                 hover_secs=10.0,
                 topic_hover='/cf/cmd_hover',
                 topic_tko='/cf/hl/takeoff',
                 topic_land='/cf/hl/land'):
        super().__init__('takeoff_and_hover_test')

        # QoS: 구독자(브리지)와 "반드시 동일"해야 함 — 지금 경고 메시지대로 RELIABLE 사용
        qos_ctrl = QoSProfile(depth=10)
        qos_ctrl.history = HistoryPolicy.KEEP_LAST
        qos_ctrl.reliability = ReliabilityPolicy.RELIABLE

        self.pub_hover = self.create_publisher(TwistStamped, topic_hover, qos_ctrl)
        self.pub_tko   = self.create_publisher(Float32, topic_tko, qos_ctrl)
        self.pub_land  = self.create_publisher(Float32, topic_land, qos_ctrl)

        self.z_target = float(z_target)
        self.dt = 1.0 / max(1.0, float(hover_hz))
        self.hover_secs = float(hover_secs)
        self.sent = 0
        self._phase = 'init'

        self.get_logger().info(
            f"[START] RELIABLE QoS, takeoff→hover@{hover_hz:.0f}Hz z={self.z_target:.2f}m for {self.hover_secs:.1f}s"
        )

        # 1) takeoff 한번 보내고 → 2) hover 루프 시작
        self.create_timer(0.2, self._kickoff)  # 1회

    def _kickoff(self):
        if self._phase != 'init':
            return
        self._phase = 'takeoff'
        self.get_logger().info("[TAKEOFF] z=%.2f" % self.z_target)
        self.pub_tko.publish(Float32(data=self.z_target))
        # takeoff 전송 후 약간 여유
        self.create_timer(1.0, self._start_hover)

    def _start_hover(self):
        if self._phase != 'takeoff':
            return
        self._phase = 'hover'
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.timer_hover = self.create_timer(self.dt, self._tick_hover)

    def _tick_hover(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self.t0) >= self.hover_secs:
            self.get_logger().info("[LAND] z=0.00")
            self.pub_land.publish(Float32(data=0.0))
            self.timer_hover.cancel()
            self._phase = 'done'
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 제자리 유지: x=0, y=0, z=절대고도(유지), yaw_rate=0
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = self.z_target
        msg.twist.angular.z = 0.0
        self.pub_hover.publish(msg)
        self.sent += 1
        if self.sent % int(1/self.dt) == 0:
            self.get_logger().info(f"[HOVER] sent={self.sent}")

def main():
    rclpy.init()
    rclpy.spin(TakeoffAndHoverTest(
        z_target=0.40,
        hover_hz=50.0,
        hover_secs=10.0,
    ))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
