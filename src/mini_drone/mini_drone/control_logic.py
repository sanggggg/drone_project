#!/usr/bin/env python3
import math
import time
import threading
from typing import Optional, Dict

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float32, Float32MultiArray, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_srvs.srv import Trigger


def _qos_ctrl() -> QoSProfile:
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.RELIABLE   # 제어/HL은 반드시 RELIABLE
    q.history = HistoryPolicy.KEEP_LAST
    return q


def _qos_sense() -> QoSProfile:
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.BEST_EFFORT  # 상태 브로드캐스트/센싱은 BEST_EFFORT 허용
    q.history = HistoryPolicy.KEEP_LAST
    return q


class ControlManager:
    """
    Crazyflie 제어 브리지.

    - hover latch 옵션:
      * hover_hold_forever: True면 hover_timeout_s를 넘겨도 마지막 세트포인트를 계속 재전송
      * hover_max_hold_s: 0보다 크면 그 시간 이상은 안전을 위해 notify_stop 전송
    """

    def __init__(self, node,
                 cmd_rate_hz: float = 50.0,
                 hover_timeout_s: float = 0.5,
                 hl_durations: Dict[str, float] = None):
        self.node = node

        # ---------- 파라미터 get/declare 헬퍼 (이미 선언돼 있으면 재선언하지 않음) ----------
        def get_or_declare(name: str, default):
            if hasattr(self.node, "has_parameter") and self.node.has_parameter(name):
                return self._param_value(self.node.get_parameter(name))
            # 이미 선언돼 있지 않으면 선언
            p = self.node.declare_parameter(name, default)
            return self._param_value(p)

        # ---------- Parameters ----------
        self.dry_run = bool(get_or_declare('dry_run', False))
        # 타 모듈이 미리 선언했을 가능성이 높은 항목들: get_or_declare 로 안전하게 처리
        self.hover_timeout_s = float(get_or_declare('hover_timeout_s', float(hover_timeout_s)))
        self.hover_hold_forever = bool(get_or_declare('hover_hold_forever', True))
        self.hover_max_hold_s = float(get_or_declare('hover_max_hold_s', 0.0))  # 0=무제한

        # 패턴 관련(중복 선언 방지)
        self.spin_duration_s = float(get_or_declare('spin_duration_s', 3.0))
        self.square_turn_rate_deg_s = float(get_or_declare('square_turn_rate_deg_s', 90.0))

        # 파라미터 변경 콜백
        self.node.add_on_set_parameters_callback(self._on_set_params)

        self.node.get_logger().info(f'dry_run initial={self.dry_run}')

        # 내부 상태
        self.cf = None
        self._lock = threading.Lock()
        self._last_hover: Optional[TwistStamped] = None
        self._last_hover_time = 0.0

        self.cmd_rate_hz = float(cmd_rate_hz)
        self.hl_durations = hl_durations or {
            'takeoff': 2.0,
            'land': 2.0,
            'goto': 2.5,
        }

        # ---- E-STOP latch state ----
        self.estop_latched = False

        # ---- Publishers ----
        self.pub_estop = self.node.create_publisher(Bool, '/cf/estop', _qos_sense())

        # ---- Subscriptions (저수준 hover & HL) ----
        self.node.create_subscription(TwistStamped, '/cf/cmd_hover', self._on_cmd_hover, _qos_ctrl())
        self.node.create_subscription(Float32, '/cf/hl/takeoff', self._on_hl_takeoff, _qos_ctrl())
        self.node.create_subscription(Float32, '/cf/hl/land',    self._on_hl_land,    _qos_ctrl())
        self.node.create_subscription(PoseStamped, '/cf/hl/goto', self._on_hl_goto,   _qos_ctrl())

        # ---- Pattern commands ----
        self.node.create_subscription(Float32MultiArray, '/cf/pattern/circle', self._on_pattern_circle, _qos_ctrl())
        self.node.create_subscription(Float32, '/cf/pattern/spin', self._on_pattern_spin, _qos_ctrl())
        self.node.create_subscription(Float32MultiArray, '/cf/pattern/square', self._on_pattern_square, _qos_ctrl())

        # ---- Services ----
        self.node.create_service(Trigger, '/cf/stop',         self._srv_stop_cb)
        self.node.create_service(Trigger, '/cf/estop_reset',  self._srv_estop_reset)
        self.node.create_service(Trigger, '/cf/notify_stop',  self._srv_notify_cb)
        self.node.create_service(Trigger, '/cf/pattern/stop', self._srv_pattern_stop)

        # ---- Timers ----
        self._hover_timer = self.node.create_timer(1.0 / max(1.0, self.cmd_rate_hz), self._hover_tick)

        # ---- Pattern thread ----
        self._pat_th: Optional[threading.Thread] = None
        self._pat_stop = threading.Event()

        if self.dry_run:
            self.node.get_logger().info('[DRY-RUN] 실제 전송 없이 로그만 출력합니다. (-p dry_run:=false 로 전송 허용 가능)')

    # ---------- utils ----------
    @staticmethod
    def _param_value(param_or_declared):
        # rclpy.Parameter 또는 ParameterValue 래핑 모두 안전하게 값만 뽑아내기
        if hasattr(param_or_declared, "value"):
            return param_or_declared.value
        if hasattr(param_or_declared, "get_parameter_value"):
            v = param_or_declared.get_parameter_value()
            # bool_value / double_value / integer_value / string_value 중 채워진 걸 사용
            for attr in ("bool_value", "double_value", "integer_value", "string_value"):
                if hasattr(v, attr):
                    vv = getattr(v, attr)
                    # rclpy는 미설정이면 0/False/''가 올 수 있으므로 그냥 반환
                    return vv
        return param_or_declared

    # ========== Public API ==========
    def attach_cf(self, cf):
        with self._lock:
            self.cf = cf
        self.node.get_logger().info('ControlManager attached to CF')

    def detach_cf(self):
        with self._lock:
            self.cf = None

    def stop_patterns(self):
        self._pat_stop.set()
        if self._pat_th and self._pat_th.is_alive():
            self._pat_th.join(timeout=1.0)
        self._pat_th = None
        self._pat_stop.clear()

    # ========== Param updates ==========
    def _on_set_params(self, params):
        for p in params:
            if p.name == 'dry_run':
                self.dry_run = bool(p.value)
                self.node.get_logger().warn(f'[PARAM] dry_run -> {self.dry_run}')
            elif p.name == 'hover_timeout_s':
                self.hover_timeout_s = float(p.value)
                self.node.get_logger().warn(f'[PARAM] hover_timeout_s -> {self.hover_timeout_s:.3f}s')
            elif p.name == 'hover_hold_forever':
                self.hover_hold_forever = bool(p.value)
                self.node.get_logger().warn(f'[PARAM] hover_hold_forever -> {self.hover_hold_forever}')
            elif p.name == 'hover_max_hold_s':
                self.hover_max_hold_s = float(p.value)
                self.node.get_logger().warn(f'[PARAM] hover_max_hold_s -> {self.hover_max_hold_s:.3f}s')
            elif p.name == 'spin_duration_s':
                self.spin_duration_s = float(p.value)
                self.node.get_logger().warn(f'[PARAM] spin_duration_s -> {self.spin_duration_s:.2f}s')
            elif p.name == 'square_turn_rate_deg_s':
                self.square_turn_rate_deg_s = float(p.value)
                self.node.get_logger().warn(f'[PARAM] square_turn_rate_deg_s -> {self.square_turn_rate_deg_s:.1f} deg/s')
        return SetParametersResult(successful=True)

    # ========== E-STOP helpers ==========
    def _publish_estop_state(self):
        self.pub_estop.publish(Bool(data=self.estop_latched))

    # ========== Internal send wrappers ==========
    def _send_hover_setpoint(self, vx: float, vy: float, yawrate_deg: float, z: float):
        if self.estop_latched:
            self.node.get_logger().warn("[E-STOP] hover 차단됨")
            return
        if self.dry_run or self.cf is None:
            self.node.get_logger().info(f"[SIM hover] vx={vx:.3f} m/s, vy={vy:.3f} m/s, z={z:.3f} m, yawrate={yawrate_deg:.1f} deg/s")
            return
        try:
            self.cf.commander.send_hover_setpoint(vx, vy, yawrate_deg, z)
        except Exception as e:
            self.node.get_logger().warn(f'hover send failed: {e}')

    def _send_notify_stop(self):
        if self.dry_run or self.cf is None:
            self.node.get_logger().info("[SIM notify_setpoint_stop]")
            return
        try:
            self.cf.commander.send_notify_setpoint_stop()
        except Exception:
            pass

    def _send_stop(self):
        if self.dry_run or self.cf is None:
            self.node.get_logger().info("[SIM STOP setpoint]")
            return
        try:
            self.cf.commander.send_stop_setpoint()
        except Exception:
            pass

    def _enable_hl(self) -> bool:
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] HL enable 차단됨')
            return False
        if self.dry_run or self.cf is None:
            self.node.get_logger().info("[SIM HL enable]")
            return True
        try:
            self.cf.param.set_value('commander.enHighLevel', '1')
            time.sleep(0.05)
            self.cf.commander.send_notify_setpoint_stop()
            return True
        except Exception as e:
            self.node.get_logger().warn(f'Enable HL skipped/failed: {e}')
            return False

    def _hl_takeoff(self, z: float, dur: float):
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] takeoff 차단됨')
            return
        if self.dry_run or self.cf is None:
            self.node.get_logger().info(f"[SIM HL takeoff] z={z:.2f} m, dur={dur:.2f} s")
            return
        from cflib.crazyflie.high_level_commander import HighLevelCommander
        try:
            HighLevelCommander(self.cf).takeoff(z, dur)
        except Exception as e:
            self.node.get_logger().error(f'HL takeoff failed: {e}')

    def _hl_land(self, z: float, dur: float):
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] land 차단됨')
            return
        if self.dry_run or self.cf is None:
            self.node.get_logger().info(f"[SIM HL land] z={z:.2f} m, dur={dur:.2f} s")
            return
        from cflib.crazyflie.high_level_commander import HighLevelCommander
        try:
            HighLevelCommander(self.cf).land(z, dur)
        except Exception as e:
            self.node.get_logger().error(f'HL land failed: {e}')

    def _hl_goto(self, x: float, y: float, z: float, yaw_rad: float, dur: float):
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] goto 차단됨')
            return
        if self.dry_run or self.cf is None:
            self.node.get_logger().info(f"[SIM HL goto] x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw_rad:.2f} rad, dur={dur:.2f} s")
            return
        from cflib.crazyflie.high_level_commander import HighLevelCommander
        try:
            HighLevelCommander(self.cf).go_to(x, y, z, yaw_rad, dur, relative=False, linear=False)
        except Exception as e:
            self.node.get_logger().error(f'HL goto failed: {e}')

    # ========== Hover (low-level) ==========
    def _on_cmd_hover(self, msg: TwistStamped):
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] hover 명령 무시')
            return
        with self._lock:
            self._last_hover = msg
            self._last_hover_time = self.node.get_clock().now().nanoseconds * 1e-9

    def _hover_tick(self):
        with self._lock:
            cf = self.cf
            cmd = self._last_hover
            t0 = self._last_hover_time

        # 연결 없고 드라이런도 아니면 아무것도 안 함
        if (cf is None) and not self.dry_run:
            return

        # E-STOP 래치: 계속 STOP 유지
        if self.estop_latched:
            self._send_stop()
            return

        now = self.node.get_clock().now().nanoseconds * 1e-9

        # 최근 호버 명령 자체가 없으면 정지 알림
        if cmd is None:
            self._send_notify_stop()
            return

        age = now - t0

        # 안전 최대 유지시간(옵션) 초과 시 정지
        if self.hover_max_hold_s > 0.0 and age > self.hover_max_hold_s:
            self.node.get_logger().warn(
                f"[HOVER] max hold exceeded ({age:.3f}s > {self.hover_max_hold_s:.3f}s) → notify_stop"
            )
            self._send_notify_stop()
            return

        # latch 비활성인 경우: 타임아웃 넘기면 notify_stop
        if (not self.hover_hold_forever) and (age > self.hover_timeout_s):
            self.node.get_logger().warn(
                f"[HOVER] 입력 타임아웃 → notify_stop 전송 (dt={age:.3f}s, limit={self.hover_timeout_s:.3f}s)"
            )
            self._send_notify_stop()
            return

        # 여기까지 왔으면 마지막 세트포인트를 계속 재전송(래치)
        vx = float(cmd.twist.linear.x)
        vy = float(cmd.twist.linear.y)
        z  = float(cmd.twist.linear.z)   # Crazyflie hover API: 절대 고도
        yawrate_deg = math.degrees(float(cmd.twist.angular.z))
        self._send_hover_setpoint(vx, vy, yawrate_deg, z)

    # ========== High-level (takeoff/land/goto) ==========
    def _on_hl_takeoff(self, msg: Float32):
        if not self._enable_hl():
            return
        self._hl_takeoff(float(msg.data), float(self.hl_durations['takeoff']))

    def _on_hl_land(self, msg: Float32):
        if not self._enable_hl():
            return
        self._hl_land(float(msg.data), float(self.hl_durations['land']))

    def _on_hl_goto(self, msg: PoseStamped):
        if not self._enable_hl():
            return
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)) if (qw or qx or qy or qz) else 0.0
        self._hl_goto(float(msg.pose.position.x),
                      float(msg.pose.position.y),
                      float(msg.pose.position.z),
                      float(yaw),
                      float(self.hl_durations['goto']))

    # ========== Patterns ==========
    def _start_pattern(self, target_fn, *args, **kwargs):
        self.stop_patterns()
        self._pat_stop.clear()
        self._pat_th = threading.Thread(target=target_fn, args=args, kwargs=kwargs, daemon=True)
        self._pat_th.start()

    def _srv_pattern_stop(self, req, res):
        self.stop_patterns()
        res.success = True; res.message = 'pattern stopped'
        return res

    def _on_pattern_circle(self, msg: Float32MultiArray):
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] circle 패턴 차단됨')
            return

        d = list(msg.data)
        if len(d) < 4:
            self.node.get_logger().error('circle needs [radius_m, speed_mps, z_m, duration_s]')
            return
        radius, speed, z, duration = map(float, d[:4])
        omega = speed / max(1e-6, abs(radius))  # rad/s
        yawrate_deg = math.degrees(omega) * (1.0 if speed >= 0 else -1.0)

        def _run():
            dt = 1.0 / max(1.0, self.cmd_rate_hz)
            self.node.get_logger().info(f'[PATTERN circle] r={radius:.2f}m v={speed:.2f}m/s z={z:.2f}m dur={duration:.2f}s (yaw={yawrate_deg:.1f}deg/s)')
            t0 = time.time()
            try:
                while (time.time() - t0) < duration and not self._pat_stop.is_set() and not self.estop_latched:
                    self._send_hover_setpoint(speed, 0.0, yawrate_deg, z)
                    time.sleep(dt)
            finally:
                self._send_notify_stop()
                self.node.get_logger().info('[PATTERN circle] done')

        self._start_pattern(_run)

    def _on_pattern_spin(self, msg: Float32):
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] spin 패턴 차단됨')
            return

        yawrate_deg = math.degrees(float(msg.data))
        duration = float(self.spin_duration_s)

        def _run():
            dt = 1.0 / max(1.0, self.cmd_rate_hz)
            self.node.get_logger().info(f'[PATTERN spin] yawrate={yawrate_deg:.1f} deg/s dur={duration:.2f}s')
            t0 = time.time()
            try:
                while (time.time() - t0) < duration and not self._pat_stop.is_set() and not self.estop_latched:
                    self._send_hover_setpoint(0.0, 0.0, yawrate_deg, 0.0)
                    time.sleep(dt)
            finally:
                self._send_notify_stop()
                self.node.get_logger().info('[PATTERN spin] done')

        self._start_pattern(_run)

    def _on_pattern_square(self, msg: Float32MultiArray):
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] square 패턴 차단됨')
            return

        d = list(msg.data)
        if len(d) < 3:
            self.node.get_logger().error('square needs [side_m, speed_mps, z_m]')
            return
        L, speed, z = map(float, d[:3])
        turn_rate = float(self.square_turn_rate_deg_s)
        move_time = abs(L / max(1e-6, speed))
        turn_time = 90.0 / max(1e-3, abs(turn_rate))

        def _run():
            dt = 1.0 / max(1.0, self.cmd_rate_hz)
            self.node.get_logger().info(f'[PATTERN square] L={L:.2f}m v={speed:.2f} z={z:.2f} (turn={turn_rate:.1f}deg/s)')
            try:
                for _ in range(4):
                    t0 = time.time()
                    while (time.time() - t0) < move_time and not self._pat_stop.is_set() and not self.estop_latched:
                        self._send_hover_setpoint(speed, 0.0, 0.0, z)
                        time.sleep(dt)
                    t1 = time.time()
                    yaw_deg_s = turn_rate if speed >= 0 else -turn_rate
                    while (time.time() - t1) < turn_time and not self._pat_stop.is_set() and not self.estop_latched:
                        self._send_hover_setpoint(0.0, 0.0, yaw_deg_s, z)
                        time.sleep(dt)
                self.node.get_logger().info('[PATTERN square] done')
            finally:
                self._send_notify_stop()

        self._start_pattern(_run)

    # ========== Services ==========
    def _srv_stop_cb(self, req, res):
        self.estop_latched = True
        self._publish_estop_state()
        self._send_stop()
        res.success = True
        res.message = 'E-STOP latched; motors stop'
        return res

    def _srv_estop_reset(self, req, res):
        self.estop_latched = False
        self._publish_estop_state()
        res.success = True
        res.message = 'E-STOP reset; command gate open'
        return res

    def _srv_notify_cb(self, req, res):
        self._send_notify_stop()
        res.success = True
        res.message = 'notify_setpoint_stop (or SIM) sent'
        return res
