#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading
from enum import Enum, auto
from typing import Optional, Tuple, Union

# ===== OpenCV / Detector =====
try:
    import cv2
except Exception:
    cv2 = None

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

# ===== Crazyflie (cflib) =====
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.high_level_commander import HighLevelCommander

# =========================
# Configuration
# =========================
CF_URI = 'radio://0/80/2M/E7E7E7E7E7'  # 필요시 수정
LOG_PERIOD_MS = 100                     # cflib 로그 주기
HOVER_RATE_HZ = 30.0                    # hover setpoint 전송 주기

# 시나리오 파라미터 (업로드 코드 기준)
TAKEOFF_HEIGHT_M     = 0.4
TAKEOFF_TIMEOUT_S    = 8.0
FORWARD_SPEED_MPS    = 0.3
FORWARD1_TIME_S      = 3.0
FORWARD2_TIME_S      = 3.0
FORWARD3_TIME_S      = 3.0
GREET_DELTA_Z_M      = 0.15
GREET_PAUSE_S        = 0.8
AVOID_LAT_SPEED_MPS  = 0.25
AVOID_FWD_SPEED_MPS  = 0.15
AVOID_TIME_S         = 2.0
SAFETY_FRONT_MIN_M   = 0.20

# 탐지 파라미터
USE_YOLO          = True    # YOLO 사용 (미설치/로드 실패시 HOG 폴백)
YOLO_MODEL        = 'yolov8n.pt'
YOLO_CONF_TH      = 0.6
HOG_WINSTRIDE     = 8
DETECT_CENTER_W   = 0.3     # 중앙 가중(0~1)
DETECT_TIMEOUT_S  = 12.0

# 카메라 소스 (기본: 노트북 웹캠 0번)
# 예) RTSP/HTTP/파일 경로 가능: "rtsp://...", "http://...", "/path/video.mp4"
CAMERA_SOURCE: Union[int, str] = 0


def deg2rad(x): return x * math.pi / 180.0
def rad2deg(x): return x * 180.0 / math.pi


class Phase(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    FORWARD1 = auto()
    DETECT = auto()
    GREET_DOWN = auto()
    GREET_UP = auto()
    AVOID = auto()
    FORWARD2 = auto()
    AVOID2 = auto()
    FORWARD3 = auto()
    LAND = auto()
    DONE = auto()
    ABORT = auto()


class CFClient:
    """Crazyflie 연결/로그/제어(hover & HL) 래퍼 (ROS 제거 버전)."""
    def __init__(self, uri: str, log_period_ms: int, hover_rate_hz: float):
        self.uri = uri
        self.log_period_ms = int(log_period_ms)
        self.hover_rate_hz = float(hover_rate_hz)

        self._lock = threading.Lock()
        self._connected = False
        self._cf: Optional[Crazyflie] = None
        self._hlc: Optional[HighLevelCommander] = None

        # 최신 상태(ROS 없이 자체 보관)
        self.roll_deg = None
        self.pitch_deg = None
        self.yaw_deg = None
        self.pos = {'x': None, 'y': None, 'z': 0.0}
        self.vel = {'x': None, 'y': None, 'z': None}
        self.ranges = {'front': None, 'back': None, 'left': None, 'right': None, 'up': None, 'down': None}
        self.vbat = None

        # hover 세트포인트(주기 전송)
        self._last_hover = {'vx': 0.0, 'vy': 0.0, 'yawrate_deg': 0.0, 'z': 0.0, 't': 0.0}
        self._hover_timeout_s = 0.5
        self._stop_evt = threading.Event()

        # 작업 스레드
        self._link_thread = threading.Thread(target=self._link_worker, daemon=True)
        self._hover_thread = threading.Thread(target=self._hover_worker, daemon=True)

    # ---------- Public ----------
    def start(self):
        self._link_thread.start()
        self._hover_thread.start()

    def stop(self):
        self._stop_evt.set()
        # 안전 정지 시도
        try:
            with self._lock:
                if self._cf:
                    self._cf.commander.send_stop_setpoint()
        except Exception:
            pass

    def is_connected(self) -> bool:
        with self._lock:
            return bool(self._connected)

    def get_altitude(self) -> float:
        with self._lock:
            return float(self.pos['z'] or 0.0)

    def get_front_range(self) -> Optional[float]:
        with self._lock:
            return self.ranges['front']

    def set_hover(self, vx: float, vy: float, z: float, yawrate_rad_s: float = 0.0):
        """hover setpoint는 계속 주기적으로 내보내야 하므로 내부 버퍼에 기록만 하고 전송은 hover thread가 담당."""
        with self._lock:
            self._last_hover.update({
                'vx': float(vx),
                'vy': float(vy),
                'yawrate_deg': rad2deg(float(yawrate_rad_s)),
                'z': float(z),
                't': time.time(),
            })

    def enable_high_level(self) -> bool:
        with self._lock:
            if not self._connected or self._cf is None:
                return False
            try:
                self._cf.param.set_value('commander.enHighLevel', '1')
                time.sleep(0.05)
                self._cf.commander.send_notify_setpoint_stop()
                self._hlc = HighLevelCommander(self._cf)
                return True
            except Exception as e:
                print(f"[HL] enable 실패: {e}")
                return False

    def hl_takeoff(self, z: float, dur_s: float):
        with self._lock:
            if self._hlc:
                try:
                    self._hlc.takeoff(float(z), float(dur_s))
                except Exception as e:
                    print(f"[HL] takeoff 실패: {e}")

    def hl_land(self, z: float, dur_s: float):
        with self._lock:
            if self._hlc:
                try:
                    self._hlc.land(float(z), float(dur_s))
                except Exception as e:
                    print(f"[HL] land 실패: {e}")

    def hl_goto_same_xy(self, z: float, dur_s: float):
        """현재 x,y 유지, z만 변경 (yaw는 0 rad 유지)"""
        with self._lock:
            if not self._hlc:
                return
            x = float(self.pos['x'] or 0.0)
            y = float(self.pos['y'] or 0.0)
            try:
                self._hlc.go_to(x, y, float(z), 0.0, float(dur_s), relative=False, linear=False)
            except Exception as e:
                print(f"[HL] goto 실패: {e}")

    # ---------- Internal workers ----------
    def _link_worker(self):
        cflib.crtp.init_drivers(enable_debug_driver=False)
        try:
            with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                cf = scf.cf
                with self._lock:
                    self._cf = cf
                    self._connected = True
                print(f"[CF] connected: {self.uri}")

                # (옵션) Arming 지원 펌웨어일 경우
                try:
                    cf.platform.send_arming_request(True)
                    print("[CF] arming request sent")
                except Exception:
                    pass

                # 로그 설정
                period = self.log_period_ms
                lgs = []

                lg_att = LogConfig(name='LG_ATT', period_in_ms=period)
                lg_att.add_variable('stabilizer.roll', 'float')
                lg_att.add_variable('stabilizer.pitch', 'float')
                lg_att.add_variable('stabilizer.yaw', 'float'); lgs.append(lg_att)

                lg_posvel = LogConfig(name='LG_POSVEL', period_in_ms=period)
                for v in ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
                          'stateEstimate.vx', 'stateEstimate.vy', 'stateEstimate.vz']:
                    lg_posvel.add_variable(v, 'float'); lgs.append(lg_posvel)

                lg_rng = LogConfig(name='LG_RNG', period_in_ms=period)
                for name, typ in [
                    ('range.front', 'uint16_t'), ('range.back', 'uint16_t'),
                    ('range.left', 'uint16_t'),  ('range.right', 'uint16_t'),
                    ('range.up', 'uint16_t'),    ('range.zrange', 'uint16_t')
                ]:
                    try:
                        lg_rng.add_variable(name, typ)
                    except KeyError:
                        pass
                if len(lg_rng.variables) > 0:
                    lgs.append(lg_rng)

                lg_bat = LogConfig(name='LG_BAT', period_in_ms=period)
                lg_bat.add_variable('pm.vbat', 'float'); lgs.append(lg_bat)

                # 콜백
                def on_att(ts, data, _):
                    with self._lock:
                        self.roll_deg  = data.get('stabilizer.roll')
                        self.pitch_deg = data.get('stabilizer.pitch')
                        self.yaw_deg   = data.get('stabilizer.yaw')

                def on_posvel(ts, data, _):
                    with self._lock:
                        self.pos['x'] = data.get('stateEstimate.x')
                        self.pos['y'] = data.get('stateEstimate.y')
                        self.pos['z'] = data.get('stateEstimate.z')
                        self.vel['x'] = data.get('stateEstimate.vx')
                        self.vel['y'] = data.get('stateEstimate.vy')
                        self.vel['z'] = data.get('stateEstimate.vz')

                def on_rng(ts, data, _):
                    with self._lock:
                        if 'range.front'  in data: self.ranges['front'] = data['range.front']   / 1000.0
                        if 'range.back'   in data: self.ranges['back']  = data['range.back']    / 1000.0
                        if 'range.left'   in data: self.ranges['left']  = data['range.left']    / 1000.0
                        if 'range.right'  in data: self.ranges['right'] = data['range.right']   / 1000.0
                        if 'range.up'     in data: self.ranges['up']    = data['range.up']      / 1000.0
                        if 'range.zrange' in data: self.ranges['down']  = data['range.zrange']  / 1000.0

                def on_bat(ts, data, _):
                    with self._lock:
                        self.vbat = data.get('pm.vbat')

                def on_err(logconf, msg):
                    print(f"[CF][{logconf.name}] error: {msg}")

                # 시작
                for lg in lgs:
                    try:
                        cf.log.add_config(lg)
                        if lg.name == 'LG_ATT':      lg.data_received_cb.add_callback(on_att)
                        elif lg.name == 'LG_POSVEL': lg.data_received_cb.add_callback(on_posvel)
                        elif lg.name == 'LG_RNG':    lg.data_received_cb.add_callback(on_rng)
                        elif lg.name == 'LG_BAT':    lg.data_received_cb.add_callback(on_bat)
                        lg.error_cb.add_callback(on_err)
                        lg.start()
                    except Exception as e:
                        print(f"[CF] Failed to start {lg.name}: {e}")

                # 유지
                while not self._stop_evt.is_set():
                    time.sleep(0.1)

                # 종료 처리
                try:
                    cf.commander.send_stop_setpoint()
                except Exception:
                    pass
        except Exception as e:
            print(f"[CF] link error: {e}")
        finally:
            with self._lock:
                self._connected = False
                self._cf = None
                self._hlc = None
            print("[CF] disconnected")

    def _hover_worker(self):
        dt = 1.0 / max(1.0, self.hover_rate_hz)
        while not self._stop_evt.is_set():
            send_stop = True
            with self._lock:
                cf = self._cf
                cmd = self._last_hover.copy()
                t_last = cmd['t']
                now = time.time()
                if (cf is not None) and ((now - t_last) <= self._hover_timeout_s):
                    send_stop = False
                    try:
                        cf.commander.send_hover_setpoint(cmd['vx'], cmd['vy'], cmd['yawrate_deg'], cmd['z'])
                    except Exception as e:
                        print(f"[CF] hover send failed: {e}")
            if send_stop:
                try:
                    with self._lock:
                        if self._cf:
                            self._cf.commander.send_notify_setpoint_stop()
                except Exception:
                    pass
            time.sleep(dt)


class Detector:
    """YOLO(HOG 폴백) 기반 사람 검출기 + 프레임 그랩."""
    def __init__(self, source=0, use_yolo=True, yolo_model='yolov8n.pt',
                 yolo_conf=0.6, hog_stride=8, center_weight=0.3):
        self.cap = None
        self.source = source
        self.use_yolo = use_yolo and (YOLO is not None)
        self.yolo = None
        self.yolo_conf = float(yolo_conf)
        self.center_weight = float(center_weight)

        self.hog = None
        self.hog_stride = int(hog_stride)

        # 모델 준비
        if self.use_yolo:
            try:
                self.yolo = YOLO(yolo_model)
                print(f"[DET] YOLO loaded: {yolo_model}")
            except Exception as e:
                print(f"[DET] YOLO load failed: {e}")
                self.yolo = None

        if (self.yolo is None) and (cv2 is not None):
            try:
                self.hog = cv2.HOGDescriptor()
                self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
                print("[DET] fallback to HOG")
            except Exception as e:
                print(f"[DET] HOG init failed: {e}")
                self.hog = None

    def open(self) -> bool:
        if cv2 is None:
            print("[DET] OpenCV not available")
            return False
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            print(f"[DET] cannot open camera/stream: {self.source}")
            self.cap = None
            return False
        return True

    def close(self):
        if self.cap:
            self.cap.release()
            self.cap = None

    def read_and_detect(self) -> Tuple[bool, Optional[float]]:
        """프레임을 읽어 사람 검출. (detected, x_offset_norm[-1..1])"""
        if (cv2 is None) or (self.cap is None):
            return False, None

        ok, frame = self.cap.read()
        if not ok:
            return False, None

        H, W = frame.shape[:2]
        if self.yolo is not None:
            try:
                res = self.yolo.predict(frame, verbose=False, conf=self.yolo_conf, imgsz=640)[0]
                found = False
                best_cx = None
                best_score = -1.0
                for b in res.boxes:
                    cls = int(b.cls[0])
                    if cls != 0:
                        continue
                    conf = float(b.conf[0])
                    x1, y1, x2, y2 = map(float, b.xyxy[0])
                    cx = 0.5 * (x1 + x2)
                    center_off = abs((cx - W / 2.0) / (W / 2.0))
                    score = (1.0 - self.center_weight) * conf + self.center_weight * (1.0 - center_off)
                    if score > best_score:
                        best_score = score
                        best_cx = cx
                    found = True
                if found and best_cx is not None and W > 0:
                    x_off_norm = (best_cx - W/2.0) / (W/2.0)
                    return True, float(x_off_norm)
                return False, None
            except Exception as e:
                print(f"[DET] YOLO infer failed: {e}")
                return False, None

        if self.hog is not None:
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                rects, _ = self.hog.detectMultiScale(gray, winStride=(self.hog_stride, self.hog_stride))
                if len(rects) > 0:
                    areas = [(w*h, (x + w/2.0)) for (x, y, w, h) in rects]
                    _, cx = max(areas, key=lambda t: t[0])
                    if W > 0:
                        x_off_norm = (cx - W/2.0) / (W/2.0)
                        return True, float(x_off_norm)
                return False, None
            except Exception as e:
                print(f"[DET] HOG infer failed: {e}")
                return False, None

        return False, None


class Behavior:
    """CreativeBehaviorNode의 상태머신을 ROS 없이 이식."""
    def __init__(self, cf: CFClient, det: Detector):
        self.cf = cf
        self.det = det
        self.phase = Phase.TAKEOFF
        self.t0 = time.time()
        self.avoid_dir = -1  # 사람 왼쪽(xoff<0)이면 오른쪽(-1)으로 회피
        self._stop = False

    def now(self) -> float:
        return time.time()

    def step(self):
        if self._stop:
            return

        now = self.now()
        alt = self.cf.get_altitude()
        front = self.cf.get_front_range()

        # 공통 안전: 전방 초근접 → 착륙/중단
        if (front is not None) and (front < SAFETY_FRONT_MIN_M) and self.phase not in (Phase.LAND, Phase.DONE, Phase.ABORT):
            print(f"[SAFE] front obstacle {front:.2f} m → EMERGENCY LAND")
            self.cf.set_hover(0.0, 0.0, alt, 0.0)
            self.cf.hl_land(0.0, 2.0)
            self.phase = Phase.ABORT
            self.t0 = now
            return

        # 상태 처리
        if self.phase == Phase.TAKEOFF:
            if (now - self.t0) < 0.2:
                if self.cf.enable_high_level():
                    self.cf.hl_takeoff(TAKEOFF_HEIGHT_M, 2.0)
            # 고도 도달?
            if alt >= (TAKEOFF_HEIGHT_M - 0.05):
                print("[PHASE] TAKEOFF → FORWARD1")
                self.phase = Phase.FORWARD1
                self.t0 = now
            elif (now - self.t0) > TAKEOFF_TIMEOUT_S:
                print("[WARN] takeoff timeout → LAND/ABORT")
                self.cf.hl_land(0.0, 2.0)
                self.phase = Phase.ABORT
                self.t0 = now

        elif self.phase == Phase.FORWARD1:
            self.cf.set_hover(FORWARD_SPEED_MPS, 0.0, TAKEOFF_HEIGHT_M, 0.0)
            if (now - self.t0) > FORWARD1_TIME_S:
                print("[PHASE] FORWARD1 → DETECT")
                self.phase = Phase.DETECT
                self.t0 = now

        elif self.phase == Phase.DETECT:
            detected, xoff = self.det.read_and_detect()
            if detected:
                if xoff is not None:
                    self.avoid_dir = -1 if (xoff < 0.0) else 1
                print(f"[DETECT] person seen (xoff={xoff}) → GREET_DOWN")
                self.cf.hl_goto_same_xy(max(0.1, TAKEOFF_HEIGHT_M - GREET_DELTA_Z_M), GREET_PAUSE_S)
                self.phase = Phase.GREET_DOWN
                self.t0 = now
            elif (now - self.t0) > DETECT_TIMEOUT_S:
                print("[WARN] no person (timeout) → LAND")
                self.cf.hl_land(0.0, 2.0)
                self.phase = Phase.LAND
                self.t0 = now
            else:
                # 제자리 유지
                self.cf.set_hover(0.0, 0.0, TAKEOFF_HEIGHT_M, 0.0)

        elif self.phase == Phase.GREET_DOWN:
            if (now - self.t0) > GREET_PAUSE_S:
                self.cf.hl_goto_same_xy(TAKEOFF_HEIGHT_M, GREET_PAUSE_S)
                self.phase = Phase.GREET_UP
                self.t0 = now

        elif self.phase == Phase.GREET_UP:
            if (now - self.t0) > GREET_PAUSE_S:
                print("[PHASE] GREET_UP → AVOID (diag)")
                self.phase = Phase.AVOID
                self.t0 = now

        elif self.phase == Phase.AVOID:
            vx = AVOID_FWD_SPEED_MPS
            vy = AVOID_LAT_SPEED_MPS * float(self.avoid_dir)  # 좌(+), 우(-)
            self.cf.set_hover(vx, vy, TAKEOFF_HEIGHT_M, 0.0)
            if (now - self.t0) > AVOID_TIME_S:
                print("[PHASE] AVOID → FORWARD2")
                self.phase = Phase.FORWARD2
                self.t0 = now

        elif self.phase == Phase.FORWARD2:
            self.cf.set_hover(FORWARD_SPEED_MPS, 0.0, TAKEOFF_HEIGHT_M, 0.0)
            if (now - self.t0) > FORWARD2_TIME_S:
                print("[PHASE] FORWARD2 → AVOID2 (opposite diag)")
                self.phase = Phase.AVOID2
                self.t0 = now

        elif self.phase == Phase.AVOID2:
            vx = AVOID_FWD_SPEED_MPS
            vy = - AVOID_LAT_SPEED_MPS * float(self.avoid_dir)
            self.cf.set_hover(vx, vy, TAKEOFF_HEIGHT_M, 0.0)
            if (now - self.t0) > AVOID_TIME_S:
                print("[PHASE] AVOID2 → FORWARD3")
                self.phase = Phase.FORWARD3
                self.t0 = now

        elif self.phase == Phase.FORWARD3:
            self.cf.set_hover(FORWARD_SPEED_MPS, 0.0, TAKEOFF_HEIGHT_M, 0.0)
            if (now - self.t0) > FORWARD3_TIME_S:
                print("[PHASE] FORWARD3 → LAND")
                self.cf.hl_land(0.0, 2.0)
                self.phase = Phase.LAND
                self.t0 = now

        elif self.phase == Phase.LAND:
            if alt <= 0.05:
                print("[DONE] landed")
                self.phase = Phase.DONE
                self.t0 = now

        elif self.phase in (Phase.DONE, Phase.ABORT):
            self._stop = True

    def run(self):
        try:
            # 카메라 open
            if not self.det.open():
                print("[WARN] detector/camera unavailable → 탐지 없이 진행")
            # 루프
            while not self._stop:
                self.step()
                time.sleep(1.0 / 50.0)  # 상태 머신 주기
        finally:
            self.det.close()


def main():
    cf = CFClient(CF_URI, LOG_PERIOD_MS, HOVER_RATE_HZ)
    det = Detector(
        source=CAMERA_SOURCE,
        use_yolo=USE_YOLO,
        yolo_model=YOLO_MODEL,
        yolo_conf=YOLO_CONF_TH,
        hog_stride=HOG_WINSTRIDE,
        center_weight=DETECT_CENTER_W
    )
    cf.start()
    time.sleep(0.5)  # 연결 안정화 대기

    try:
        beh = Behavior(cf, det)
        beh.run()
    except KeyboardInterrupt:
        print("\n[INTERRUPT] landing…")
        try:
            if cf.enable_high_level():
                cf.hl_land(0.0, 2.0)
        except Exception:
            pass
    finally:
        cf.stop()
        time.sleep(0.5)


if __name__ == '__main__':
    main()
