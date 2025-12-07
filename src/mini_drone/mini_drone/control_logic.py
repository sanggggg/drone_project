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
    [1.65628, 0, 0, 0, 0, -0.121551, 0.181915, -0.0916309, 0.0157621, 0.5, 0, 0, 0, -1.56156, 2.16355, -1.05757, 0.178905, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.47227, 0, 0.060961, 0.0470486, 0.0100226, 0.335743, -0.395749, 0.168823, -0.0255419, 0, -0.209423, 0.047923, 0.0246114, 0.286757, -0.343998, 0.145604, -0.0216555, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.2633, 0.4, 0.439461, 0.0160825, -0.0159811, -0.0624916, 0.0330919, -0.0050928, 5.68715e-05, 0, 0.170319, 0.0239837, -0.0109936, 0.278611, -0.30598, 0.114905, -0.0146759, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.12818, 0.95, -0.0509326, -0.110118, -0.0152558, -0.851892, 1.88692, -1.38252, 0.344594, 0.5, 0.0216977, -0.0388086, 0.00259493, -1.35395, 2.8008, -2.02841, 0.506451, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.926147, 0.75, -0.15356, 0.0581784, -0.00880007, 0.677957, -1.38046, 1.03284, -0.274854, 0.4, -0.0797571, -0.00438273, 0.0032464, -3.50741, 9.08189, -8.16959, 2.51991, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.919465, 0.7, 0.016052, 0.021843, -0.0060338, -0.0135201, 0.233089, -0.307285, 0.114518, 0.25, -0.0841121, -0.000817808, 0.000154153, -3.66252, 9.57966, -8.68992, 2.70167, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.996671, 0.75, 0.104605, 0.0345211, 0.0022989, 1.37218, -3.11419, 2.49326, -0.691969, 0.1, -0.0760591, 0.00762747, 0.00276749, -2.01119, 4.99308, -4.24725, 1.23103, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.01176, 0.95, 0.214126, 0.000210336, 0.00877242, 0.834222, -2.25782, 1.97904, -0.579772, 0, 0.00171439, 0.0224002, -0.000442042, 2.0864, -4.84794, 3.93867, -1.10164, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.804898, 1.15, 0.103643, -0.0349274, 0.00267308, 0.802596, -2.86373, 3.26576, -1.23188, 0.1, 0.0717459, 0.00702847, -0.00125899, 7.04139, -20.9125, 21.599, -7.65463, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.818143, 1.2, -0.00828887, -0.0207993, -0.00366476, 0.35104, -1.58251, 1.92839, -0.745054, 0.25, 0.0915947, 0.00235469, -0.00297423, 6.55254, -19.3671, 19.8062, -6.93398, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.09035, 1.15, -0.148706, -0.0615678, -0.0100617, -1.23137, 2.99849, -2.37682, 0.634059, 0.4, 0.0613594, -0.0157976, 0.000575796, 1.14811, -2.52231, 1.9308, -0.506984, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.60464, 0.95, -0.0328433, 0.119865, 0.0131412, 0.6246, -0.868836, 0.420694, -0.0708131, 0.5, 0.0469146, 0.0069052, -0.00170369, 0.0974976, -0.202286, 0.11999, -0.0229527, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.72824, 1.4, 0.366076, -0.0481393, -0.0173332, -0.283321, 0.335692, -0.141209, 0.0209577, 0.5, -0.122089, -0.0479698, 0.00469299, -1.21965, 1.73795, -0.846936, 0.14077, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.86353, 1.65, 0.0143266, 0.00231867, 0.0161083, -0.169422, 0.271613, -0.135593, 0.0221505, 0, 0.00219241, 0.0749351, -0.000233534, 0.891777, -1.15181, 0.512681, -0.0782018, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.7906, 1.9, 0.393093, 0.0651583, -0.014672, 0.276992, -0.46468, 0.237605, -0.0398859, 0.5, 0.115953, -0.0490969, -0.00408145, -0.178587, 0.23364, -0.104304, 0.0160337, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.5508, 2.5, 0.00126429, -0.142131, 0.00180316, -1.20083, 1.89933, -1.02019, 0.186966, 0.5, -0.0116285, 0.0211323, 0.0004156, 0.143274, -0.251335, 0.141209, -0.0265565, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.10048, 2.1, -0.112777, 0.0840384, -0.0183685, -0.243161, 0.761374, -0.678604, 0.194359, 0.5, -0.0603716, -0.0345366, 0.00177774, -3.73488, 8.15599, -6.16809, 1.59898, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.26049, 2.1, 0.137378, 0.0621863, 0.0208631, 1.83854, -3.70231, 2.49677, -0.571333, 0.25, -0.0822586, 0.0304274, 0.00562004, 0.243312, -0.382087, 0.214821, -0.0427154, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.2897, 2.4, 0.0186071, -0.113402, -0.00620356, -1.18615, 2.0755, -1.27136, 0.270349, 0.25, 0.0497352, -0.000824597, -0.00792784, 0.0186127, -0.125739, 0.11577, -0.0305574, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.20367, 2.1, -0.28177, 0.0445899, 0.0293624, 0.74505, -1.01963, 0.523025, -0.0969746, 0.25, -0.0774628, -0.0283738, 0.00464267, -2.37118, 4.75575, -3.29761, 0.783025, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.98482, 2.1, 0.313333, 0.168633, -0.0385474, 0.205188, -0.370881, 0.180942, -0.0280797, 0, -0.0736052, 0.028504, 0.00352955, 0.0451112, -0.0512118, 0.0196289, -0.00260813, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [4.28935, 2.5, -0.410378, -0.252532, 0.023436, -0.0390855, 0.0337204, -0.00792153, 0.000586282, 0, 0.0298737, -0.00616869, -0.00287593, -0.00153611, 0.00147013, -0.000321568, 2.24768e-05, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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