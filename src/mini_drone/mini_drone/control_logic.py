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
from mini_drone_interfaces.srv import RunTrajectory

# ---- cflib 라이브러리 (High-Level Trajectory용) ----
try:
    from cflib.crazyflie.mem import MemoryElement, Poly4D
except ImportError:
    Poly4D = None
    MemoryElement = None

# ---- Figure 8 데이터 ----
FIGURE8_DATA = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
]

VERTICAL_A_DATA = [
    [2.42949, 0, 0, 0, 0, 0.127304, -0.0957927, 0.0269372, -0.00273502, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0.546897, -0.481704, 0.150591, -0.0165101, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.53532, 0.5, 0.358651, -0.0126382, -0.0263671, 0.0107106, -0.0131687, 0.00597471, -0.000814174, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 0.171638, -0.357494, -0.0598504, -0.0768108, 0.13584, -0.048622, 0.00548807, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.11311, 1, 0.00541951, -0.034309, 0.00576377, 0.270093, -0.349443, 0.146087, -0.0203644, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, -0.0966474, 0.300951, -0.0184669, 0.0608461, -0.113805, 0.0492638, -0.00674873, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.62923, 0.75, -0.466818, -0.0603873, 0.042239, -0.201069, 0.493527, -0.317951, 0.0635366, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0.126556, -0.161561, 0.0012387, 0.0203292, 0.0639211, -0.0540882, 0.011548, 0, 0, 0, 0, 0, 0, 0, 0]
]

# ---- Trajectory Registry: type_name -> (data, traj_id) ----
TRAJECTORY_REGISTRY = {
    'figure8': (FIGURE8_DATA, 1),
    'vertical_a': (VERTICAL_A_DATA, 2),
}

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

        self._hl_active_until = 0.0

        # 파라미터 변경 콜백
        self.node.add_on_set_parameters_callback(self._on_set_params)

        self.node.get_logger().info(f'dry_run initial={self.dry_run}')

        # 내부 상태
        self.cf = None
        self._lock = threading.Lock()

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

        # ---- Subscriptions (HL) ----
        self.node.create_subscription(Float32, '/cf/hl/takeoff', self._on_hl_takeoff, _qos_ctrl())
        self.node.create_subscription(Float32, '/cf/hl/land',    self._on_hl_land,    _qos_ctrl())
        self.node.create_subscription(PoseStamped, '/cf/hl/goto', self._on_hl_goto,   _qos_ctrl())

        # ---- Services ----
        self.node.create_service(Trigger, '/cf/stop',         self._srv_stop_cb)
        self.node.create_service(Trigger, '/cf/estop_reset',  self._srv_estop_reset)
        self.node.create_service(Trigger, '/cf/notify_stop',  self._srv_notify_cb)

        # ---- Trajectory Service (단일 서비스, type 인자로 선택) ----
        self.node.create_service(RunTrajectory, '/cf/traj/run', self._srv_traj_run)

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

    # ========== Param updates ==========
    def _on_set_params(self, params):
        for p in params:
            if p.name == 'dry_run':
                self.dry_run = bool(p.value)
                self.node.get_logger().warn(f'[PARAM] dry_run -> {self.dry_run}')
        return SetParametersResult(successful=True)

    # ========== E-STOP helpers ==========
    def _publish_estop_state(self):
        self.pub_estop.publish(Bool(data=self.estop_latched))

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

    # ========== High-level (takeoff/land/goto) ==========
    def _on_hl_takeoff(self, msg: Float32):
        if not self._enable_hl():
            return
        
        self._hl_takeoff(float(msg.data), float(self.hl_durations['takeoff']))
        self._hl_active_until = time.time() + float(self.hl_durations['takeoff']) + 0.5

    def _on_hl_land(self, msg: Float32):
        if not self._enable_hl():
            return

        self._hl_land(float(msg.data), float(self.hl_durations['land']))
        import time
        self._hl_active_until = time.time() + float(self.hl_durations['land']) + 0.5

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
        self._hl_active_until = time.time() + float(self.hl_durations['goto']) + 0.5

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

# ========== Trajectory Service Implementation ==========
    def _upload_trajectory(self, traj_data, traj_id: int) -> tuple[bool, str, float]:
        """
        궤적 데이터를 드론 메모리에 업로드하는 공통 헬퍼.
        Returns: (success, message, total_duration)
        """
        if self.cf is None:
            return False, "CF not connected", 0.0
        
        if Poly4D is None:
            return False, "cflib not installed", 0.0

        try:
            traj_mem = self.cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
            traj_mem.trajectory = []
            
            total_duration = 0.0
            for row in traj_data:
                duration = row[0]
                x = Poly4D.Poly(row[1:9])
                y = Poly4D.Poly(row[9:17])
                z = Poly4D.Poly(row[17:25])
                yaw = Poly4D.Poly(row[25:33])
                traj_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
                total_duration += duration
            
            if not traj_mem.write_data_sync():
                return False, "Upload failed", 0.0
            
            self.cf.high_level_commander.define_trajectory(traj_id, 0, len(traj_mem.trajectory))
            return True, f"Uploaded ID {traj_id} (dur={total_duration:.1f}s)", total_duration
            
        except Exception as e:
            return False, str(e), 0.0

    def _start_trajectory(self, traj_id: int, duration: float) -> tuple[bool, str]:
        """
        업로드된 궤적을 실행하는 공통 헬퍼.
        Returns: (success, message)
        """
        if not self._enable_hl():
            return False, "Enable HL failed"

        try:
            self.cf.high_level_commander.start_trajectory(traj_id, 1.0, relative=True)
            self._hl_active_until = time.time() + duration + 2.0
            return True, f"Started Trajectory ID {traj_id}"
        except Exception as e:
            return False, str(e)

    def _srv_traj_run(self, req, res):
        """
        단일 Trajectory 서비스: trajectory_type 인자로 궤적 선택
        지원 타입: 'figure8', 'vertical_a'
        """
        traj_type = req.trajectory_type.lower().strip()
        
        # Registry에서 조회
        if traj_type not in TRAJECTORY_REGISTRY:
            available = ', '.join(TRAJECTORY_REGISTRY.keys())
            res.success = False
            res.message = f"Unknown trajectory type '{traj_type}'. Available: {available}"
            self.node.get_logger().warn(res.message)
            return res
        
        traj_data, traj_id = TRAJECTORY_REGISTRY[traj_type]
        self.node.get_logger().info(f"[TRAJ] Running '{traj_type}' (upload + start)...")
        
        # 1. Upload
        ok, msg, duration = self._upload_trajectory(traj_data, traj_id=traj_id)
        if not ok:
            self.node.get_logger().error(f"[TRAJ] Upload failed: {msg}")
            res.success = False
            res.message = f"Upload failed: {msg}"
            return res
        
        self.node.get_logger().info(f"[TRAJ] {msg}")
        
        # 2. Start
        ok, msg = self._start_trajectory(traj_id=traj_id, duration=duration)
        if not ok:
            self.node.get_logger().error(f"[TRAJ] Start failed: {msg}")
            res.success = False
            res.message = f"Start failed: {msg}"
            return res
        
        res.success = True
        res.message = f"'{traj_type}' started (dur={duration:.1f}s)"
        self.node.get_logger().info(res.message)
        return res