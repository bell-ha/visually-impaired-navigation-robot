#!/usr/bin/env python3
"""
메인 라우터 + 웹 대시보드
- 시리얼 파싱 → interface.py(버튼1/압력) / vision_assistant.py(버튼2)
- 브라우저에서 실시간 로그 확인 + 수동 조작 가능
- 실행: python3 main.py  →  http://localhost:8080 자동 오픈
"""
import collections
import json
import os
import signal
import subprocess
import sys
import threading
import time
import webbrowser
from pathlib import Path

try:
    import serial
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False
    print("[경고] pyserial 없음 – 시리얼(버튼/압력) 비활성화")
from flask import Flask, Response, jsonify, request, send_from_directory

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
THIS_DIR = Path(__file__).resolve().parent
WEB_DIR  = THIS_DIR / "web"      # 대시보드 정적 페이지 (index.html)

# ── 진단 로거 (간헐 버그 블랙박스) — 같은 폴더의 robot_diag.py 사용 ──
sys.path.insert(0, str(THIS_DIR))
try:
    import robot_diag as _diag
except Exception as _e:          # 로거 없어도 본체는 정상 동작해야 함
    _diag = None
    print(f"[경고] robot_diag 로드 실패: {_e}")
_diaglog = None
ENV_FILE = next(
    (p for p in [
        THIS_DIR / "../../.env",
        THIS_DIR / "../../../.env",
        Path.home() / "GitHub/visually-impaired-navigation-robot/src/.env",
    ] if p.exists()),
    THIS_DIR / "../../.env",
)

# ── 시리얼 설정 ───────────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0"
BAUD = 115200

# ── 스피커 설정 ───────────────────────────────────────────────────────────────
# interface.py 스피커: index 0 (HDA Intel PCH)
# vision_assistant.py 스피커: index 0 (같은 장치 사용 — index 1 없음)
VISION_SPEAKER_INDEX = 0

# ── Pull 감지 상수 ────────────────────────────────────────────────────────────
_GRIP_ARM   = 3731
_PULL_TRIG  = 4095
_QUICK_SEC  = 0.80
_GRIP_RESET = 3158
_DEBOUNCE   = 0.25

# ── ROS2 cmd_vel ──────────────────────────────────────────────────────────────
# FastDDS 공유메모리 비활성화(UDP 강제) — 어느 터미널에서 시작해도 적용되도록
# 코드에서 직접 주입 (rclpy import 전이어야 함). 자식(launch·interface·vision)도 상속.
# SHM 반복 고장(유령 참가자·데이터 채널 잠김) 방지 — 2026-07-24
os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE",
                      os.path.expanduser("~/.ros/fastdds_no_shm.xml"))
try:
    import rclpy
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String as ROSString
    from control_msgs.action import FollowJointTrajectory   # 팔 수납(트래젝토리) 트리거용
    from trajectory_msgs.msg import JointTrajectoryPoint
    from rclpy.action import ActionClient
    from std_srvs.srv import Trigger
    _ROS_OK = True
    try:
        from nav2_msgs.srv import LoadMap   # 지도(층) 전환용 — 없어도 나머지는 동작
    except ImportError:
        LoadMap = None
except ImportError:
    _ROS_OK = False
    LoadMap = None

# ── 로그 버퍼 ─────────────────────────────────────────────────────────────────
_LOG_BUF: collections.deque = collections.deque(maxlen=800)
_log_lock = threading.Lock()
_LOG_SEQ = 0   # 단조증가 id — SSE가 "보낸 개수"(len, 포화 시 고정) 대신 이걸로 커서 삼음

def _log(src: str, msg: str):
    global _LOG_SEQ
    ts = time.strftime("%H:%M:%S")
    text = msg.rstrip()
    with _log_lock:
        _LOG_SEQ += 1
        entry = {"id": _LOG_SEQ, "t": ts, "src": src, "msg": text}
        _LOG_BUF.append(entry)
    # 파일 영속(#15) — 락 밖에서 호출. 이걸로 SSE/UI 소비자는 분리되지만,
    # 자식 stdout 백프레셔는 락과 무관하게 남아 있다(디스크 정체 시).
    if _diaglog:
        try:
            _diaglog.log(src, entry["msg"], echo=False)   # 터미널 에코는 차단(파이어호스)
        except Exception:
            pass

# ── Pull 감지기 ───────────────────────────────────────────────────────────────
def _make_pull_detector():
    s = {"armed": False, "t": 0.0, "fired": False}
    def update(val):
        now = time.monotonic()
        if val < _GRIP_RESET:
            s["armed"] = s["fired"] = False
            return False
        if not s["armed"] and val >= _GRIP_ARM:
            s.update(armed=True, t=now, fired=False)
            return False
        if s["armed"] and not s["fired"]:
            if val >= _PULL_TRIG and (now - s["t"]) <= _QUICK_SEC:
                s["fired"] = True
                return True
        return False
    return update

# ── 서브프로세스 관리 ─────────────────────────────────────────────────────────
_procs: dict = {}

_NOISE_PREFIXES = ("ALSA lib", "Cannot open device", "Unknown PCM", "Invalid field", "Invalid card", "Found no matching", "Expression '")

def _capture(proc: subprocess.Popen, name: str):
    for line in proc.stdout:
        if any(line.strip().startswith(p) for p in _NOISE_PREFIXES):
            continue
        _log(name, line)

def _write(name: str, cmd: str) -> bool:
    """자식 프로세스 stdin에 한 줄 보낸다.

    반환은 "썼다"까지지 "상대가 처리했다"가 아니다. False = 프로세스가 없거나
    죽었거나 쓰다 실패 — 지금까지 이걸 조용히 삼켜서, 인터페이스가 죽어 있어도
    호출부는 보낸 줄 알았다. 기존 호출부는 반환을 안 쓰므로 동작은 그대로다."""
    p = _procs.get(name)
    if not (p and p.poll() is None):
        return False
    # 줄 단위 프로토콜이라 개행이 섞이면 뒷부분이 별개 줄로 들어간다 —
    # 통보문이 두 동강 나면서 뒤쪽이 "사용자 발화"로 오인될 수 있다.
    cmd = cmd.replace("\n", " ").replace("\r", " ")
    try:
        p.stdin.write(cmd + "\n")
        p.stdin.flush()
        return True
    except Exception:
        return False

def start_subprocesses():
    iface = subprocess.Popen(
        [sys.executable, "-u", str(THIS_DIR / "interface.py"),
         "--no-hw", "--env-file", str(ENV_FILE)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    vision = subprocess.Popen(
        [sys.executable, "-u", str(THIS_DIR / "vision_assistant.py"),
         "--out-index", str(VISION_SPEAKER_INDEX)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    # nav2 블랙박스 — 독립 프로세스로 spawn (대시보드 spin이 섬 현상으로 마비돼도
    # 얘는 자체 participant/spin이라 계속 기록 → 주행 멈춤/드롭을 확실히 파일로 남김).
    # 자동 실행이라 사용자는 따로 켤 필요 없음(interface·vision과 동일).
    diagnav = subprocess.Popen(
        [sys.executable, "-u", str(THIS_DIR / "robot_diag_nav.py")],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    _procs["iface"] = iface
    _procs["vision"] = vision
    _procs["diagnav"] = diagnav
    threading.Thread(target=_capture, args=(iface, "IFACE"), daemon=True).start()
    threading.Thread(target=_capture, args=(vision, "VISION"), daemon=True).start()
    threading.Thread(target=_capture, args=(diagnav, "NAV2"), daemon=True).start()
    _log("MAIN", f"interface.py PID={iface.pid}, vision_assistant.py PID={vision.pid}, "
                 f"nav-blackbox PID={diagnav.pid}")

# ── ROS2 cmd_vel 퍼블리셔 ─────────────────────────────────────────────────────
_cmd_node = None
_cmd_pub  = None
_arm_client = None   # 팔 수납 트래젝토리 액션 클라이언트 (init_ros에서 생성)
_lc_loc_client = None    # lifecycle_manager_localization is_active 클라이언트 (init_ros에서 생성)
_lc_nav_client = None    # lifecycle_manager_navigation is_active 클라이언트 (init_ros에서 생성)
_manual_mode = False
_manual_lock = threading.Lock()

_backup_warn_until = 0.0   # 후진 안내 디바운스

# ── 배터리 상태 ────────────────────────────────────────────────────────────────
_battery = {"pct": None, "voltage": None, "charging": None}

# ── 준비상태 신호 갱신시각 (2a-1: 뼈대) ────────────────────────────────────────
# monotonic 갱신시각만 저장 — age는 /readiness에서 요청 시점에 계산(값 저장 금지).
# 초기값 0.0 → age가 거대해져 자동으로 unknown/stale 판정됨(낙관 초기값 금지).
_ready = {"amcl": 0.0, "battery": 0.0, "handle": 0.0, "nav2": 0.0, "elev_app": 0.0, "gripper_camera": 0.0}
# 2a-2: 폴링 신호는 "답의 내용"이 age와 독립 → 값을 따로 저장(2a-1의 '값 저장 금지'는 콜백형에만 적용).
#   _ready[key]=monotonic() = "마지막으로 물어본 시각"(폴러 건강),  _ready_val[key] = 판정결과(대상 건강).
_ready_val = {"nav2": None, "elev_app": None, "gripper_camera": None}

# ── 엘베 고립 관측(A5 B2) — 표시 전용 ─────────────────────────────────────────
# 8/31의 signature: 엘베앱 프로세스는 살아서 /ping에 200을 주는데 ROS 쪽은
# 그래프에서 사라져 있었다(앱은 "켜져 있음", 실제로는 아무 명령도 안 통함).
# 함정: 정상 기동 때도 Flask(/ping)가 rclpy.init()보다 먼저 떠서 같은 창이
# 매번 생긴다 — 유예 없이 판정하면 매 기동 100% 오발(늑대소년)이 된다.
# 그래서 (1) 앱 기동 후 유예 (2) 연속 관측 두 조건을 모두 요구한다.
_elev_started_mono = None    # 엘베앱을 spawn한 시각(=/ping 최초 성공이 아니라 기동 시각)
_ELEV_ISO_GRACE    = 20.0    # 기동 유예(초) — _wait_elev_app_up(timeout=20.0)와 같은 값
_ELEV_ISO_HITS     = 3       # 연속 관측 횟수 — 폴러 3초 주기라 약 9초
_elev_iso_hits     = 0       # 연속 카운터(폴러 스레드 전용)
_diag_st = None              # robot_diag.attach()가 돌려주는 상태 캐시(HB가 채움)
# 엘베앱이 어느 모드로 떠 있나(True=사진모드/False=정상/None=모름). UI가 모드를
# 표시하려면 이 값이 필요한데, /status를 화면이 직접 부를 수는 없다(5000은 별
# 오리진). 새 폴링은 만들지 않고 _readiness_poll_loop이 이미 받아오는 st에서
# 주워 담는다 — 추가 HTTP 0건.
_elev_no_ocr = None
# 엘베앱 obs 캐시가 얼마나 오래됐으면 "모름"으로 볼지 — 엘베앱 OBS_STALE_SEC의
# 미러(별도 프로세스라 상수를 직접 못 읽는다). 엘베앱 OBS_PERIOD=1s라 그보다
# 훨씬 커야 GIL 경합에 깜빡이지 않는다. 저쪽 값을 바꾸면 여기도 같이 바꿀 것.
_OBS_AGE_MAX = 5.0

# 측위 정지게이트: amcl은 update_min_d/a라 정지 중엔 /amcl_pose가 안 나온다 —
# 이동명령 여부로 "미갱신=고장"과 "미갱신=정상(정지)"을 구분한다.
_last_move_cmd = 0.0
_STILL_GRACE = 3.0

# ── 장애물 상태 ────────────────────────────────────────────────────────────────
_obstacle_state = {"detected": False, "dist": None, "decision": None}
_obstacle_last_log_t = 0.0

def _obstacle_objects_cb(msg):
    global _obstacle_last_log_t
    try:
        objects = json.loads(msg.data).get("objects", [])
        now = time.monotonic()
        if objects:
            dist = objects[0].get("distance", 0)
            _obstacle_state.update(detected=True, dist=round(dist, 1))
            if now - _obstacle_last_log_t > 4.0:
                _log("OBSTACLE", f"의자 감지: {dist:.1f}m 앞")
                _obstacle_last_log_t = now
        else:
            if _obstacle_state["detected"]:
                _log("OBSTACLE", "의자 사라짐")
            _obstacle_state.update(detected=False, dist=None)
    except Exception:
        pass

def _obstacle_decision_cb(msg):
    try:
        data   = json.loads(msg.data)
        action = data.get("action", "")
        reason = data.get("reason", "")
        if action == "push":
            _obstacle_state["decision"] = "push"
            _log("OBSTACLE", f"밀고 통과 — {reason}")
        elif action == "detour":
            _obstacle_state["decision"] = "detour"
            _log("OBSTACLE", f"우회 — {reason}")
        elif action == "probe_start":
            _obstacle_state["decision"] = "probing"
            _log("OBSTACLE", "probe 시작 (의자 접촉 테스트)")
    except Exception:
        pass

# ── 지도(층) 전환 ─────────────────────────────────────────────────────────────
# 같은 건물 = 같은 구조 = 같은 좌표계 (floor4.pgm은 all.pgm의 동일 사본, origin 동일)
_FLOOR_MAPS = {
    "5": str((THIS_DIR / "../maps/all.yaml").resolve()),
    "4": str((THIS_DIR / "../maps/floor4.yaml").resolve()),
    "3": str((THIS_DIR / "../maps/floor3.yaml").resolve()),
    "2": str((THIS_DIR / "../maps/floor2.yaml").resolve()),
    "1": str((THIS_DIR / "../maps/floor1.yaml").resolve()),
}
_current_floor = "5"   # 런치 기본 지도 = all.yaml(5층)
_map_client = None     # /map_server/load_map 서비스 클라이언트
_init_pub   = None     # /initialpose 퍼블리셔 (AMCL 재정위치용)

def _load_exit_point():
    """location.yaml의 '엘리베이터 하차지점' — 층 전환 직후 AMCL 초기 위치로 사용."""
    try:
        import yaml as _yaml
        locs = _yaml.safe_load(open(THIS_DIR / "../config/location.yaml"))["locations"]
        return locs.get("엘리베이터 하차지점")
    except Exception:
        return None

# ── 현재 위치 (AMCL) ──────────────────────────────────────────────────────────
_robot_pose = {"x": None, "y": None, "z": None, "w": None, "yaw_deg": None}

def _amcl_pose_cb(msg):
    try:
        import math
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        _robot_pose.update(x=round(p.x, 3), y=round(p.y, 3),
                           z=round(q.z, 4), w=round(q.w, 4),
                           yaw_deg=round(math.degrees(yaw), 1))
        _ready["amcl"] = time.monotonic()
    except Exception:
        pass   # 콜백 예외가 절대 rclpy.spin 스레드를 죽이지 않게

def _battery_callback(msg):
    # ⚠️ Stretch 드라이버는 percentage를 NaN으로 발행(전압만 유효). 과거엔 round(NaN)이
    # ValueError를 내고, 이 콜백에 try/except가 없어 rclpy.spin 스레드를 통째로 죽였다
    # → 대시보드 ROS 절반(pose·battery·load_map)이 부팅 몇 초 뒤 먹통 (2026-08-10 규명).
    try:
        import math
        p = msg.percentage
        if p is None or math.isnan(p) or math.isinf(p):
            _battery["pct"] = None                          # 알 수 없음 → 전압만 사용
        else:
            _battery["pct"] = round(p * 100) if p <= 1.0 else round(p)
        v = msg.voltage
        _battery["voltage"]  = round(v, 1) if (v is not None and not math.isnan(v)) else None
        _battery["charging"] = msg.power_supply_status == 1  # CHARGING=1
        _ready["battery"] = time.monotonic()
    except Exception:
        pass

def _cmdvel_callback(msg):
    global _backup_warn_until, _last_move_cmd
    try:
        if (abs(msg.linear.x) > 1e-4 or abs(msg.linear.y) > 1e-4
                or abs(msg.angular.z) > 1e-4):
            _last_move_cmd = time.monotonic()
        if _manual_mode:
            return   # 수동 모드에서는 안내 생략
        if msg.linear.x < -0.01:
            now = time.monotonic()
            if now > _backup_warn_until:
                _backup_warn_until = now + 10.0
                _write("iface", "/backup")
                _log("MAIN", "후진 감지 → TTS 안내")
    except Exception:
        pass

# ── 2a-2: 준비상태 능동폴링 (nav2 lifecycle + 엘베앱 HTTP) ──────────────────────
def _query_lc(client):
    """lifecycle_manager의 is_active를 논블로킹으로 질의. spin_until_future_complete
    금지(공유 스핀 스레드 데드락) — add_done_callback + Event만 사용."""
    if client is None or not client.service_is_ready():
        return {"status": "unknown", "detail": "기동중"}   # 서비스 자체 없음 = 부팅 정상창
    fut = client.call_async(Trigger.Request())
    ev = threading.Event()
    fut.add_done_callback(lambda f: ev.set())   # 콜백 본문은 set()만 — 로깅·락·HTTP 금지(공유 스핀 스레드)
    if not ev.wait(timeout=0.5):
        client.remove_pending_request(fut)      # 필수 — 안 하면 pending 요청이 무한 누적
        return {"status": "unknown", "detail": "응답없음(매니저 멈춤 의심)"}
    try:
        res = fut.result()
    except Exception:
        return {"status": "unknown", "detail": "응답오류"}
    if res is not None and res.success:
        return {"status": "ok", "detail": "활성"}   # "정상"은 아님 — is_active는 내부 멈춤을 못 잡음
    return {"status": "bad", "detail": "비활성"}

def _diag_fresh() -> bool:
    """HB 캐시(_diag_st)가 방금 잰 값인지. HB가 멈췄거나 그래프 질의가 계속
    예외면 캐시가 상하는데, 상한 값으로 판정하면 탐지가 조용히 꺼지거나(옛
    blind=True) 엉뚱한 걸 범인으로 만든다. blind·miss가 같은 캐시에서 오므로
    둘 다 이 게이트 뒤에서만 읽는다 — 하나만 게이트하면 나머지가 샌다.
    (miss_ts는 robot_diag가 time.time()으로 찍으니 같은 시계로 잰다.)"""
    if not isinstance(_diag_st, dict):
        return False
    ts = _diag_st.get("miss_ts")
    hb = _diag_st.get("hb_period") or 2.0
    return ts is not None and (time.time() - ts) <= 3 * hb


def _elev_isolated(ping_ok: bool) -> bool:
    """엘베 고립 signature 판정 — "앱은 응답하는데 ROS 그래프엔 없다".

    재료는 둘 다 대시보드가 이미 갖고 있는 것이다: /ping 결과(이 폴러)와
    HB가 채워둔 없는노드 캐시(스핀 스레드). 여기선 캐시를 읽기만 한다 —
    HB 콜백에 HTTP를 넣거나 이 스레드에서 그래프를 다시 질의하지 않는다.

    ⚠ 정상 기동 때도 엘베앱은 Flask(/ping)를 먼저 띄우고 rclpy.init()을 나중에
    한다 → "200인데 elevator_tracker 없음"이 매 기동 반드시 생긴다. 유예 없이
    판정하면 100% 오발이라, 기동 후 20초 + 연속 3회를 모두 넘어야 고립이라 부른다.
    반대로 오래 돌던 앱에서 사라진 것은 진짜 런타임 고립(8/31)이라 그대로 잡힌다."""
    global _elev_iso_hits
    # ★불변식: blind와 miss를 따로 읽는다(개별 read). 연속 _ELEV_ISO_HITS회를
    # 요구하고 폴러 주기(3s)가 HB 주기(2s)보다 길어서 한 틱 섞인 조합이 판정을
    # 바꾸지 못하기 때문이다. _ELEV_ISO_HITS를 1로 줄이거나 폴러를 HB보다 짧게
    # 하면 그 전제가 깨지므로, 그때는 둘을 한 덩어리로(dict 통째 교체) 읽어야 한다.
    if not _diag_fresh():   # 캐시 없음(_diag_st None)도 여기서 걸린다
        _elev_iso_hits = 0
        return False
    if _diag_st.get("blind"):
        # 대시보드 자신조차 그래프에 안 보이면 elevator_tracker가 없는 것도
        # 당연하다 — 이걸 고립으로 부르면 멀쩡한 엘베앱을 범인으로 지목해
        # 운영자가 엉뚱한 재시작을 하게 된다. 이 경우는 판정 자체를 보류한다.
        _elev_iso_hits = 0
        return False
    miss = _diag_st.get("miss")
    started = _elev_started_mono
    if (not ping_ok) or started is None or miss is None:
        _elev_iso_hits = 0          # 재료가 없으면 판정 보류 (무소식을 정상으로 읽지 않음)
        return False
    if time.monotonic() - started < _ELEV_ISO_GRACE:
        _elev_iso_hits = 0          # 기동 창 — 발견 중일 뿐이라 판정하지 않는다
        return False
    if "elevator_tracker" in miss:
        _elev_iso_hits += 1
    else:
        _elev_iso_hits = 0
    return _elev_iso_hits >= _ELEV_ISO_HITS


def _obs_brief(obs) -> str:
    """엘베앱 /status의 고립 관측(A5 B1)을 한 줄로 옮겨 적는다 — 표시 전용.
    판정은 엘베앱이 이미 했고 여기선 문자열로 바꾸기만 한다(대시보드 재판정 금지)."""
    if not isinstance(obs, dict):
        # 신선도 게이트에 걸렸거나 아예 안 온 경우 — "정상"이 아니라 "모름"이다.
        return "관측 없음/오래됨 — 판정 불가"
    mark = {"ok": "정상", "stale": "끊김", "unknown": "미관측"}
    # 몸체캠은 표시만 하고 바를 빨갛게 만들지 않으므로, 초록 바에 "끊김"이
    # 떠 있어도 모순이 아니라는 걸 꼬리표로 알린다.
    return " · ".join(f"{lbl} {mark.get(obs.get(k), '미관측')}"
                      for k, lbl in (("driver", "팔"), ("body", "몸체캠(표시)"),
                                     ("depth", "depth")))


def _readiness_poll_loop():
    global _elev_started_mono, _elev_no_ocr
    order = {"bad": 0, "unknown": 1, "ok": 2}
    while True:
        try:
            loc = _query_lc(_lc_loc_client)
            nav = _query_lc(_lc_nav_client)
            worst = min([loc, nav], key=lambda d: order[d["status"]])
            _ready_val["nav2"] = {"status": worst["status"],
                                  "detail": f"측위 {loc['detail']} / 주행 {nav['detail']}"}

            try:
                import requests   # 로컬 import — urllib과 달리 프록시 우회(proxies=)가 명시적
                r = requests.get("http://127.0.0.1:5000/ping", timeout=(0.3, 0.5),
                                 proxies={"http": None, "https": None})
                if r.status_code == 200:
                    # 리스 만료(deadman) 표면화 — 엘베가 스스로 권한을 내린 채
                    # 대기 중이면 "정상 응답"이 아니라 재부여가 필요한 상태
                    st = _elev_status(timeout=1.0)
                    _elev_no_ocr = bool(st.get("no_ocr")) if st else None
                    if _elev_started_mono is None:
                        # 대시보드만 재시작했거나 앱을 밖에서 띄운 경우 — Popen을
                        # 우리가 안 해서 기동시각이 없다. 그대로 두면 고립 탐지가
                        # 영영 꺼진 채로 남으므로 지금을 기준으로 삼는다. 유예가
                        # 처음부터 다시 도는 fail-late일 뿐, 틀린 경보는 안 낸다.
                        _elev_started_mono = time.monotonic()
                        _log("MAIN", "엘베앱 기동시각 미상(외부 기동/대시보드 재시작) "
                                     "→ 지금부터 고립 유예 재시작")
                    # 세 신호는 서로 다른 것을 본다: 고립=ROS 그래프 단절,
                    # 리스만료=제어권, obs=엘베앱이 본 자기 의존성(팔/카메라).
                    # 어느 하나라도 나쁘면 bad로 올리고(하나가 다른 하나를 가리지
                    # 않게), 앞머리 문구는 고립 > 리스만료 순으로 하나만 고른 뒤
                    # 관측 상태는 항상 뒤에 병기한다.
                    obs = st.get("obs") if st else None
                    # 엘베앱의 obs는 그쪽 ROS 타이머가 채우는 캐시다 — 스핀이
                    # 굶으면 얼어붙는데 /status는 캐시만 읽으므로 옛 "정상"이
                    # 그대로 넘어온다(무증상 실패의 재현). 잰 지 오래됐으면
                    # 내용을 믿지 않고 통째로 모름으로 둔다.
                    # ★모름은 고장이 아니다 — 여기서 bad로 올리지 말 것.
                    # 부하 스파이크마다 빨간 바가 뜨면 아무도 바를 안 믿는다.
                    obs_age = st.get("obs_age") if st else None
                    if obs_age is None or obs_age > _OBS_AGE_MAX:
                        obs = None
                    iso  = _elev_isolated(True)
                    leas = bool(st and st.get("lease_expired"))
                    # obs에 stale이 있으면 픽토그램 승격 — 문자열로만 적어두면
                    # 팔이 "끊김"인데 바는 초록이라 아무도 안 본다.
                    # 단 승격은 driver·depth만 본다: body(D435i)가 끊겨도 누르기·
                    # 정렬·주행은 그대로 되므로 여정을 막지 않는다. body 상태는
                    # _obs_brief 문자열에 그대로 남으니 운영자는 여전히 본다
                    # (승격에서 빼는 것이지 숨기는 게 아니다).
                    obs_stale = isinstance(obs, dict) and any(
                        obs.get(k) == "stale" for k in ("driver", "depth"))
                    if iso:
                        head = "고립 — 앱은 응답하나 elevator_tracker 없음(ROS 단절)"
                    elif leas:
                        head = "엘베 리스 만료 — 제어권 재부여 필요"
                    else:
                        # 응답 = HTTP가 살아있다는 뜻까지다. 자기고립이면 고립 여부를
                        # "정상"이라 말할 수 없으니 모른다고 적는다.
                        head = ("응답(고립 판정 불가 — 대시보드도 그래프 미검출)"
                                if _diag_fresh() and _diag_st.get("blind")
                                else "응답")
                    _ready_val["elev_app"] = {
                        "status": "bad" if (iso or leas or obs_stale) else "ok",
                        "detail": head + " · " + _obs_brief(obs)}
                    # 그리퍼 카메라 — "대기"로 속아 헛걸음시킨 사고 방지(엘베앱 /status의
                    # camera_missing 재사용, 엘베앱이 이미 기동 10초 유예까지 다 처리함)
                    cam_missing = st.get("camera_missing") if st else None
                    if cam_missing is None:
                        _ready_val["gripper_camera"] = {"status": "unknown", "detail": "판정 불가"}
                    elif cam_missing:
                        _ready_val["gripper_camera"] = {"status": "bad",
                                                        "detail": "미수신 — 엘베앱 재시작 필요"}
                    else:
                        _ready_val["gripper_camera"] = {"status": "ok", "detail": "정상"}
                else:
                    _elev_isolated(False)   # 연속 카운터 리셋 — 비200은 고립 signature가 아니다
                    _ready_val["elev_app"] = {"status": "bad", "detail": f"HTTP {r.status_code}"}
                    _ready_val["gripper_camera"] = {"status": "unknown", "detail": "엘베앱 응답 없음"}
                    _elev_no_ocr = None     # 못 물어봤으면 모드도 모른다(옛 값 금지)
            except Exception:
                _elev_isolated(False)   # 연속 카운터 리셋 — 무응답은 고립 signature가 아니다
                _ready_val["elev_app"] = {"status": "unknown", "detail": "무응답(미기동/접속거부)"}
                _ready_val["gripper_camera"] = {"status": "unknown", "detail": "미기동"}
                _elev_no_ocr = None     # 위와 같은 이유 — 바로 옆 카메라 판정과 같은 규율
        except Exception:
            time.sleep(3.0)
            continue
        finally:
            # 성공/타임아웃/예외 무관 — 폴러가 살아있다는 증거로 매 사이클 갱신
            _ready["nav2"] = time.monotonic()
            _ready["elev_app"] = time.monotonic()
            _ready["gripper_camera"] = time.monotonic()
        time.sleep(3.0)

def init_ros():
    global _cmd_node, _cmd_pub, _map_client, _init_pub, _arm_client, _lc_loc_client, _lc_nav_client, _diag_st
    if not _ROS_OK:
        return
    rclpy.init()
    _cmd_node = rclpy.create_node("main_web_cmdvel")
    _cmd_pub  = _cmd_node.create_publisher(Twist, "/stretch/cmd_vel", 10)
    _cmd_node.create_subscription(Twist,        "/stretch/cmd_vel",           _cmdvel_callback,       10)
    _cmd_node.create_subscription(BatteryState, "/battery",                   _battery_callback,      10)
    _cmd_node.create_subscription(ROSString,    "/obstacle_pusher/objects",   _obstacle_objects_cb,   10)
    _cmd_node.create_subscription(ROSString,    "/obstacle_pusher/decision",  _obstacle_decision_cb,  10)
    # AMCL 위치 — QoS 이중 구독: 퍼블리셔가 transient_local(latched)이든
    # volatile이든 어느 쪽이어도 수신되게. (TL 구독은 VOL 퍼블리셔와 매칭 자체가
    # 안 됨 — 한쪽만 구독하면 Nav2 버전에 따라 영영 미수신이 될 수 있음)
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    _pose_qos_tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
    _cmd_node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                  _amcl_pose_cb, _pose_qos_tl)
    _cmd_node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                  _amcl_pose_cb, 10)
    # 지도(층) 전환: map_server load_map 클라이언트 + AMCL 재정위치 퍼블리셔
    if LoadMap is not None:
        _map_client = _cmd_node.create_client(LoadMap, "/map_server/load_map")
    _init_pub = _cmd_node.create_publisher(PoseWithCovarianceStamped,
                                           "/initialpose", 10)
    # 팔 수납(armleft 대체) — 트래젝토리 액션 클라이언트 (엘베와 동일 서버)
    _arm_client = ActionClient(_cmd_node, FollowJointTrajectory,
                               "/stretch_controller/follow_joint_trajectory")
    # 2a-2: 준비상태 폴링용 lifecycle is_active 클라이언트 — 여기서 1회만 생성(폴러 안에서 만들지 않음)
    _lc_loc_client = _cmd_node.create_client(Trigger, "/lifecycle_manager_localization/is_active")
    _lc_nav_client = _cmd_node.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
    # 진단 계측 부착 (cmd_vel 퍼블리셔 수·엘리베이터 노드 존재 추적)
    if _diag is not None and _diaglog is not None:
        # 이름은 런치 XML의 <node name=...> 문자 그대로 (추측 금지).
        # 없는노드 목록은 로그 detail 문자열 재료로만 쓴다 — 픽토그램으로 승격하면
        # 대시보드 자신이 그래프에서 고립됐을 때 전부 "없음"으로 보여 오탐이 된다.
        _diag_st = _diag.attach(
            _cmd_node, _diaglog,
            cmd_vel_topic="/stretch/cmd_vel",
            own_node_name="main_web_cmdvel",
            expected_nodes=["elevator_tracker", "rplidar", "camera",
                            "gripper_camera", "safety_zone_left_back_tf",
                            "map_server", "amcl",
                            "lifecycle_manager_localization"],
        )
    def _spin_resilient():
        # 콜백 예외로 스핀이 죽어도 되살려 ROS 절반(pose·battery·load_map·initialpose)이
        # 계속 살게 한다. (과거: 보호 없던 콜백 예외 하나에 이 스레드가 영구 사망)
        while rclpy.ok():
            try:
                rclpy.spin(_cmd_node)
            except Exception as e:
                try:
                    _log("MAIN", f"ROS 스핀 예외 → 재개: {e!r}")
                except Exception:
                    pass
                time.sleep(0.3)
    threading.Thread(target=_spin_resilient, daemon=True).start()
    threading.Thread(target=_readiness_poll_loop, daemon=True).start()
    _log("MAIN", "ROS2 cmd_vel 퍼블리셔/구독자 시작")

# ── 팔 수납 (armleft 프로세스 대체 — 대시보드가 직접 one-shot 트리거) ────────────────
# 엘베 이동-씬의 수납 로직을 그대로 복제: 그리퍼 먼저 닫고(과부하 방지: 열린 채 손목
# 돌리면 손가락이 몸통에 닿음) → 손목 안쪽 + 팔 완전 수축. 엘베 앱 없이도 동작하고,
# one-shot이라 armleft처럼 계속 재전송하며 팔을 두고 다투는 일이 없다.
def _stow_arm():
    if _arm_client is None:
        _log("MAIN", "팔 수납 실패 — 액션클라 미초기화(ROS 없음)")
        return
    def _send(joint_names, positions, sec, wait_timeout=0.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start.sec = int(sec)
        goal.trajectory.points = [pt]
        if wait_timeout > 0:                       # 완료까지 블로킹 (엘베 _move_joint_wait 복제)
            done = threading.Event()
            def _on_res(_): done.set()
            def _on_resp(fut):
                h = fut.result()
                if h and h.accepted:
                    h.get_result_async().add_done_callback(_on_res)
                else:
                    done.set()
            _arm_client.send_goal_async(goal).add_done_callback(_on_resp)
            done.wait(timeout=wait_timeout)
        else:                                      # 비블로킹 (엘베 _send_goal 복제)
            _arm_client.send_goal_async(goal)
    def _run():
        try:
            if not _arm_client.wait_for_server(timeout_sec=2.0):
                _log("MAIN", "팔 수납 실패 — 트래젝토리 서버 없음 (런치 떴나?)")
                return
            _send(["gripper_aperture"], [0.00], sec=2, wait_timeout=5.0)   # 1) 그리퍼 닫기(대기)
            # 2) 손목 안쪽 + 팔 수축 + lift 올림(0.90 = armleft와 동일한 올린 높이)
            _send(["joint_wrist_pitch", "joint_wrist_yaw", "joint_wrist_roll",
                   "wrist_extension", "joint_lift"],
                  [-0.02, 3.4, 0.0, 0.0, 0.90], sec=2)
            _log("MAIN", "팔 수납 완료 (닫기→손목 안쪽→팔 수축→lift 0.90)")
        except Exception as e:
            _log("MAIN", f"팔 수납 예외: {e!r}")
    threading.Thread(target=_run, daemon=True).start()

def publish_cmd(lx: float, az: float):
    if not _cmd_pub:
        return
    msg = Twist()
    msg.linear.x  = float(lx)
    msg.angular.z = float(az)
    _cmd_pub.publish(msg)

# ── 수동 조작 루프 (버튼 누르는 동안 연속 퍼블리시) ──────────────────────────
_manual_cmd = {"lx": 0.0, "az": 0.0}
_manual_active = False
_last_cmd_t = 0.0   # 마지막 /cmd 수신 시각(monotonic) — 서버측 데드맨(#27, 브라우저/와이파이 끊김 대비)

def _manual_loop():
    global _manual_active
    while True:
        with _manual_lock:
            active = _manual_active
            lx = _manual_cmd["lx"]
            az = _manual_cmd["az"]
            # 데드맨: 활성 상태인데 0.5초 넘게 새 /cmd가 안 오면(탭 닫힘·와이파이
            # 끊김 등) 마지막 명령이 영원히 재발행되는 걸 막고 서버가 스스로 정지
            stale = active and (time.monotonic() - _last_cmd_t > 0.5)
            if stale:
                _manual_active = False
                _manual_cmd["lx"] = 0.0
                _manual_cmd["az"] = 0.0
        if stale:
            publish_cmd(0.0, 0.0)
            _log("MAIN", "⚠ 수동 명령 0.5초 무수신 → 서버측 데드맨 정지")
        elif active:
            # 수동 활성 상태일 때만 퍼블리시 — 자동 모드에서는 Nav2가 직접 제어
            publish_cmd(lx, az)
        time.sleep(0.1)

# ── Flask ─────────────────────────────────────────────────────────────────────
app = Flask(__name__)
# 브라우저가 옛 자산을 붙들어 "고쳤는데 화면이 안 바뀜"이 되는 것 방지.
# localhost 관제라 캐시 이득이 없다.
app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0

@app.route("/")
def index():
    return send_from_directory(WEB_DIR, "index.html")

@app.route("/logs")
def logs_sse():
    def sse(entry):
        return f"data: {json.dumps(entry, ensure_ascii=False)}\n\n"

    def gen():
        # id(단조증가) 커서로 스캔 — "보낸 개수"(len)는 deque(maxlen=800) 포화 시
        # 항상 800으로 고정돼 entries[sent:]가 영원히 빈 리스트가 되는 버그가 있었음.
        last_id = 0
        last_beat = time.monotonic()
        while True:
            with _log_lock:
                new = []
                for e in reversed(_LOG_BUF):
                    if e["id"] <= last_id:
                        break
                    new.append(e)
                new.reverse()
                oldest_id = _LOG_BUF[0]["id"] if _LOG_BUF else 0
            # 커서보다 오래된 로그가 이미 버퍼에서 밀려났으면(포화) 유실 고지
            if new and last_id and oldest_id > last_id + 1:
                yield sse({"t": time.strftime("%H:%M:%S"), "src": "SYS",
                          "msg": f"⚠ 로그 {oldest_id - last_id - 1}줄 유실(버퍼 초과)"})
            for e in new:
                yield sse(e)
                last_id = e["id"]
                last_beat = time.monotonic()
            if not new and time.monotonic() - last_beat > 15:
                yield ": ping\n\n"   # SSE 주석 하트비트 — 유령 스레드 회수
                last_beat = time.monotonic()
            time.sleep(0.1)
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

@app.route("/cmd", methods=["POST"])
def cmd():
    global _manual_active, _last_cmd_t
    data = request.json or {}
    lx = float(data.get("lx", 0.0))
    az = float(data.get("az", 0.0))
    with _manual_lock:
        _manual_cmd["lx"] = lx
        _manual_cmd["az"] = az
        _manual_active = data.get("active", True)
        _last_cmd_t = time.monotonic()
    return jsonify(ok=True)

@app.route("/stop", methods=["POST"])
def stop():
    global _manual_active
    with _manual_lock:
        _manual_cmd["lx"] = 0.0
        _manual_cmd["az"] = 0.0
        _manual_active = False
    publish_cmd(0.0, 0.0)
    return jsonify(ok=True)

# ── 엘리베이터 제어권 (주도권은 대시보드가 소유, 5000에 부여/회수) ─────────────
_elev_authority = False   # 우리가 아는 엘리베이터 앱의 제어권 보유 상태
_elev_lease_held = False  # 리스(=이동권+guard_off) 보유 여부 — 한 여정에 1회 부여, 층 이동 전 반납
_lease_stop = threading.Event()   # set()이면 하트비트 중단 — 리스 보유 중에만 clear() 상태로 돎
_lease_renewer_thread = None      # 하트비트 스레드 핸들 (반납 시 join용)

def _lease_renewer():
    """리스 하트비트(2초 주기) — 엘베앱은 6초(LEASE_TTL) 내 재갱신 없으면 자체 회수(deadman).
    반드시 별도 스레드에서 돈다(_auto_run은 확인 대기로 최대 15분 블록될 수 있음)."""
    while not _lease_stop.wait(2.0):
        _set_elev_authority(True, "리스 갱신", quiet=True)

def _stop_armleft_proc():
    """armleft(팔 고정)를 확실히 종료 — 대시보드가 추적 못 하는 것(재시작 desync)도
    이름으로 kill. graceful하게 SIGTERM(pkill 기본) 사용 → armleft가 깨끗이 정리."""
    p = _procs.get("armleft")
    if p and p.poll() is None:
        try:
            p.terminate()
        except Exception:
            pass
    try:
        subprocess.run(["pkill", "-f", "tools/armleft.py"], capture_output=True)
    except Exception:
        pass
    _procs.pop("armleft", None)

def _set_elev_authority(granted: bool, reason: str = "", quiet: bool = False):
    """엘리베이터 앱에 이동 제어권 부여/회수. 회수 시 엘리베이터는 즉시 전면
    정지(바퀴 정지·타겟 해제·안무 중단)하고 이후 모든 이동을 거부한다.
    구버전 엘리베이터 앱(엔드포인트 없음)이면 /reset 폴백. 꺼져 있으면 무시.
    quiet=True면 로그를 안 남김 — 리스 하트비트(2초 주기)가 다른 로그를 덮지 않게."""
    global _elev_authority
    # 대시보드 의도를 항상 반영 — 엘리베이터 앱(5000)이 안 떠서 POST가 실패해도
    # 토글이 되돌아오지 않게. (JS 폴링이 이 값으로 토글을 동기화하므로)
    _elev_authority = granted
    if granted:
        # 엘리베이터가 팔(트래젝토리 액션)을 써야 함 → armleft를 반드시 종료(팔 넘겨줌).
        # quiet(하트비트)면 생략 — 상태변화 없는 재확인일 뿐이라 이미 첫 grant(non-quiet)
        # 에서 멈췄음. 매번 pkill을 새로 스폰하면 DDS churn(#22와 같은 패턴).
        if not quiet:
            _stop_armleft_proc()
            _log("MAIN", "엘리베이터 제어권 부여 → armleft 자동 종료")
    try:
        import urllib.request
        req = urllib.request.Request(
            "http://localhost:5000/authority",
            data=json.dumps({"granted": granted}).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        urllib.request.urlopen(req, timeout=1)
        if not quiet:
            _log("MAIN", f"엘리베이터 제어권 {'부여' if granted else '회수'}"
                         + (f" ({reason})" if reason else ""))
    except Exception:
        if not granted:
            try:   # 폴백: 구버전 앱이면 최소한 타겟 해제
                import urllib.request
                urllib.request.urlopen(urllib.request.Request(
                    "http://localhost:5000/reset", method="POST"), timeout=1)
                if not quiet:
                    _log("MAIN", "엘리베이터 /authority 없음 → /reset 폴백")
            except Exception:
                pass

@app.route("/elev_authority", methods=["GET", "POST"])
def elev_authority_route():
    if request.method == "POST":
        granted = bool((request.json or {}).get("granted", False))
        # _set_elev_authority 직접호출 금지 — 하트비트 renewer가 안 켜져서 수동 grant가
        # deadman(TTL 6s)에 회수당함. _grant_elev_lease 경유해야 renewer 시작/중단됨.
        threading.Thread(target=_grant_elev_lease,
                         args=(granted, "수동 제어권 토글"), daemon=True).start()
        return jsonify(ok=True, granted=granted)
    return jsonify(granted=_elev_authority)

@app.route("/mode", methods=["POST"])
def set_mode():
    global _manual_mode, _manual_active
    data = request.json or {}
    _manual_mode = data.get("manual", False)
    if _manual_mode:
        # 수동 전환 시: 진행 중인 목적지 취소 + LOCKED 상태로
        _write("iface", "/cancel")
        # 주도권 단일화: 수동 = 사람이 바퀴 주인 → 엘리베이터 제어권 자동 회수
        # (두 앱이 /stretch/cmd_vel을 동시에 지휘하면 수동 조작이 먹히지 않음)
        # _set_elev_authority 직접호출 금지 — renewer(2초 하트비트)가 안 꺼져서
        # 회수 직후 authority=True를 도로 쏨(수동 중 이동권·guard_off 부활).
        threading.Thread(target=_grant_elev_lease,
                         args=(False, "수동 전환"), daemon=True).start()
    else:
        # 자동 전환 시: 수동 이동 정지 명령 한 번만 발행 후 Nav2에 제어권 넘김
        with _manual_lock:
            _manual_active = False
            _manual_cmd["lx"] = 0.0
            _manual_cmd["az"] = 0.0
        publish_cmd(0.0, 0.0)
    _log("MAIN", f"모드 변경: {'수동' if _manual_mode else '자동'}")
    return jsonify(ok=True, manual=_manual_mode)

@app.route("/button", methods=["POST"])
def web_button():
    _write("iface", "/button")
    _log("WEB", "버튼1 (웹)")
    return jsonify(ok=True)

@app.route("/vision", methods=["POST"])
def web_vision():
    _write("vision", "/vision")
    _log("WEB", "버튼2 시각 분석 (웹)")
    return jsonify(ok=True)

@app.route("/switch_map", methods=["GET", "POST"])
def switch_map():
    """지도(층) 전환 — 재시작 없이 map_server에 load_map 서비스 호출.
    init_exit=true면 전환 직후 AMCL 초기 위치를 '엘리베이터 하차지점'으로 설정
    (두 층 지도가 동일 사본·동일 origin이라 좌표계가 같음 → 좌표 재사용 가능)."""
    global _current_floor
    if request.method == "GET":
        return jsonify(floor=_current_floor)
    data  = request.json or {}
    floor = str(data.get("floor", ""))
    path  = _FLOOR_MAPS.get(floor)
    if path is None:
        return jsonify(ok=False, error="알 수 없는 층"), 400
    if _map_client is None:
        return jsonify(ok=False, error="ROS/LoadMap 미초기화"), 503
    if not _map_client.service_is_ready():
        _log("MAP", "map_server 서비스 안 보임 — 런치가 떠 있는지 확인")
        return jsonify(ok=False, error="map_server 서비스 없음")
    req = LoadMap.Request()
    req.map_url = path
    fut = _map_client.call_async(req)     # 완료는 백그라운드 spin 스레드가 처리
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < 6.0:
        time.sleep(0.1)
    res = fut.result() if fut.done() else None
    if res is None or res.result != 0:    # RESULT_SUCCESS = 0
        code = getattr(res, "result", "timeout")
        _log("MAP", f"지도 전환 실패 (result={code})")
        return jsonify(ok=False, error=f"load_map 실패 ({code})")
    _current_floor = floor
    _log("MAP", f"🗺 지도 전환 완료 → {floor}층 ({os.path.basename(path)})")
    if data.get("init_exit"):
        p = _load_exit_point()
        if p and _init_pub is not None:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = _cmd_node.get_clock().now().to_msg()
            msg.pose.pose.position.x    = float(p["x"])
            msg.pose.pose.position.y    = float(p["y"])
            msg.pose.pose.orientation.z = float(p.get("z", 0.0))
            msg.pose.pose.orientation.w = float(p.get("w", 1.0))
            msg.pose.covariance[0]  = 0.25   # RViz 2D Pose Estimate와 동일 분산
            msg.pose.covariance[7]  = 0.25
            msg.pose.covariance[35] = 0.068
            _init_pub.publish(msg)
            _log("MAP", "AMCL 초기 위치 → 엘리베이터 하차지점 (RViz 클릭 불필요)")
        else:
            _log("MAP", "⚠ 하차지점 좌표를 못 읽음 — RViz 2D Pose Estimate로 지정 필요")
    return jsonify(ok=True, floor=floor)

@app.route("/robot_pose")
def robot_pose():
    """현재 로봇 위치(AMCL) 조회 — 값은 응답 + 대시보드 로그에 동시 기록.
    location.yaml에 바로 붙여넣을 수 있는 x/y/z/w 형식 포함."""
    if _robot_pose["x"] is None:
        _log("POSE", "위치 미수신 — RViz 2D Pose Estimate로 AMCL을 먼저 초기화하세요")
        return jsonify(ok=False, error="AMCL pose 미수신")
    _log("POSE", f"📍 현재 위치 x={_robot_pose['x']} y={_robot_pose['y']} "
                 f"방향 {_robot_pose['yaw_deg']:+.1f}°  |  location.yaml용 → "
                 f"x: {_robot_pose['x']}  y: {_robot_pose['y']}  "
                 f"z: {_robot_pose['z']}  w: {_robot_pose['w']}")
    return jsonify(ok=True, **_robot_pose)

@app.route("/battery_status")
def battery_status():
    return jsonify(**_battery)

# 2a-1: 실측 3신호 임계값(관대한 잠정값, 첫 주행 실측 후 정밀화 예정)
_READY_THRESH = {"amcl": 30.0, "battery": 15.0, "handle": 5.0, "nav2": 12.0, "elev_app": 12.0,
                 "gripper_camera": 12.0}

def _readiness_signal(key, label):
    updated_at = _ready[key]
    if updated_at <= 0.0:
        return {"status": "unknown", "age_sec": None, "detail": f"{label} 미수신"}
    age = time.monotonic() - updated_at
    if age > _READY_THRESH[key]:
        return {"status": "bad", "age_sec": round(age, 1), "detail": f"{label} 신호 끊김(stale)"}
    return {"status": "ok", "age_sec": round(age, 1), "detail": f"{label} 정상"}

def _readiness_amcl():
    """amcl 전용 — update_min_d/a라 정지 중엔 /amcl_pose가 안 나온다(정상).
    이동명령 여부로 '미갱신=고장'과 '미갱신=정상(정지)'을 구분한다."""
    updated_at = _ready["amcl"]
    if updated_at <= 0.0:
        return {"status": "unknown", "age_sec": None, "detail": "측위 미수신"}
    age = time.monotonic() - updated_at
    if age <= _READY_THRESH["amcl"]:
        return {"status": "ok", "age_sec": round(age, 1), "detail": "측위 정상"}
    moving = (time.monotonic() - _last_move_cmd) <= _STILL_GRACE
    if not moving:
        return {"status": "ok", "age_sec": round(age, 1), "detail": "측위 정지 중(갱신 없음이 정상)"}
    return {"status": "bad", "age_sec": round(age, 1), "detail": "측위 신호 끊김(이동 중 미갱신)"}

def _readiness_polled(key, label):
    """폴링형 표시 — age는 폴러 자체의 건강(콜백형과 동일 age 계산), 내용은 _ready_val."""
    updated_at = _ready.get(key, 0.0)
    if updated_at <= 0.0:
        return {"status": "unknown", "age_sec": None, "detail": f"{label} 폴링 대기"}
    age = time.monotonic() - updated_at
    if age > _READY_THRESH[key]:
        return {"status": "unknown", "age_sec": round(age, 1), "detail": f"{label} 폴러 정지(stale)"}
    val = _ready_val.get(key)
    if val is None:
        return {"status": "unknown", "age_sec": round(age, 1), "detail": f"{label} 판정대기"}
    return {"status": val["status"], "age_sec": round(age, 1), "detail": val["detail"]}

@app.route("/readiness")
def readiness():
    return jsonify(
        amcl=_readiness_amcl(),
        battery=_readiness_signal("battery", "배터리"),
        handle=_readiness_signal("handle", "손잡이"),
        nav2=_readiness_polled("nav2", "nav2"),
        gripper_camera=_readiness_polled("gripper_camera", "그리퍼캠"),
        elev_app=_readiness_polled("elev_app", "엘베앱"),
    )

@app.route("/robot_speed", methods=["POST"])
def set_robot_speed():
    speed = float(request.json.get("speed", 0.26))
    speed = max(0.10, min(0.50, speed))
    subprocess.run(
        ["ros2", "param", "set", "/controller_server",
         "FollowPath.max_vel_x", str(speed)],
        capture_output=True, timeout=5
    )
    subprocess.run(
        ["ros2", "param", "set", "/controller_server",
         "FollowPath.max_speed_xy", str(speed)],
        capture_output=True, timeout=5
    )
    _log("MAIN", f"로봇 속도 변경 → {speed} m/s")
    return jsonify(ok=True, speed=speed)

@app.route("/tts_speed", methods=["POST"])
def set_tts_speed():
    speed = float(request.json.get("speed", 1.5))
    speed = max(0.5, min(2.0, speed))
    Path("/tmp/tts_speed").write_text(str(speed))
    _log("MAIN", f"TTS 속도 변경 → {speed}x")
    return jsonify(ok=True, speed=speed)

@app.route("/pull", methods=["POST"])
def web_pull():
    _write("iface", "/pull")
    _log("WEB", "당김 트리거 (웹)")
    return jsonify(ok=True)

_social_nav_enabled = False   # 기본 비활성 (2026-07-24 사용자 요청)

@app.route("/toggle_social_nav", methods=["POST"])
def toggle_social_nav():
    global _social_nav_enabled
    _social_nav_enabled = not _social_nav_enabled
    Path("/tmp/social_nav_enabled").write_text("1" if _social_nav_enabled else "0")
    _log("WEB", f"사회적 내비게이션: {'ON' if _social_nav_enabled else 'OFF'}")
    return jsonify(ok=True, enabled=_social_nav_enabled)

@app.route("/obstacle_status")
def obstacle_status():
    return jsonify(_obstacle_state)

_obstacle_push_enabled = False   # 기능 제거 (2026-07-24 사용자 결정 — UI 토글도 삭제)

@app.route("/toggle_obstacle_push", methods=["POST"])
def toggle_obstacle_push():
    global _obstacle_push_enabled
    _obstacle_push_enabled = not _obstacle_push_enabled
    Path("/tmp/obstacle_push_enabled").write_text("1" if _obstacle_push_enabled else "0")
    _log("WEB", f"장애물 밀기: {'ON' if _obstacle_push_enabled else 'OFF'}")
    return jsonify(ok=True, enabled=_obstacle_push_enabled)

@app.route("/armleft_status")
def armleft_status():
    p = _procs.get("armleft")
    running = bool(p and p.poll() is None)
    return jsonify(running=running)

@app.route("/stow_arm", methods=["POST"])
def stow_arm():
    _stow_arm()
    _log("WEB", "팔 수납 (웹 트리거)")
    return jsonify(ok=True)

@app.route("/armleft", methods=["POST"])
def armleft():
    data = request.json or {}
    desired = data.get("running")  # True=켜기, False=끄기, None=토글
    p = _procs.get("armleft")
    currently_running = bool(p and p.poll() is None)
    # 추적 못 하는 armleft 도 감지 (대시보드 재시작 desync 대비)
    if not currently_running:
        try:
            currently_running = subprocess.run(
                ["pgrep", "-f", "tools/armleft.py"],
                capture_output=True).returncode == 0
        except Exception:
            pass

    # 끄기(또는 토글인데 켜져 있음) → 추적 여부 무관하게 이름으로 확실히 종료
    if desired is False or (desired is None and currently_running):
        _stop_armleft_proc()
        _log("MAIN", "armleft.py 종료 (pkill 확실 종료)")
        return jsonify(ok=True, running=False)

    # 이미 켜져 있으면 그대로
    if desired is True and currently_running:
        return jsonify(ok=True, running=True)

    proc = subprocess.Popen(
        [sys.executable, "-u", str(THIS_DIR / "tools/armleft.py")],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    _procs["armleft"] = proc
    threading.Thread(target=_capture, args=(proc, "ARM"), daemon=True).start()
    _log("MAIN", f"armleft.py 시작 PID={proc.pid}")
    return jsonify(ok=True, running=True)

# ── 엘리베이터 앱 on-demand 실행 (nav과 동시 구동 = 과부하 → 순차 운영) ──────────────
#   근거(2026-08-10 규명): nav + 엘베앱(OCR 서버·그리퍼 카메라)을 동시에 돌리면
#   컴퓨터가 과부하 → 센서/TF 타이밍 붕괴 → costmap이 센서를 버림 → 주행이 5초/30초
#   멈춤. 엘베앱을 끄고 주행하면 완전히 깨끗함(838드롭→0). 그래서 "한 번에 하나씩":
#   승차지점까지 주행(엘베앱 OFF) → 도착 후 엘베앱 ON(제어권 자동) → 버튼 → OFF → nav 복귀.
#   [시작] 엘베앱 spawn → 5000 뜨면 제어권 자동 부여(+armleft 자동 종료)
#   [종료] 제어권 회수(엘베 즉시 정지) → SIGTERM(엘베앱 finally 정리로 서보·바퀴 깨끗이)
def _elev_app_running() -> bool:
    p = _procs.get("elevator")
    if p and p.poll() is None:
        return True
    try:   # 대시보드가 추적 못 하는 것(재시작 desync)도 이름으로 감지
        return subprocess.run(["pgrep", "-f", "elevator_button_press/main.py"],
                              capture_output=True).returncode == 0
    except Exception:
        return False

def _wait_elev_app_up(timeout: float = 20.0) -> bool:
    """엘베앱 5000 서버가 응답할 때까지 대기. 막 spawn한 직후엔 5000이 안 떠서
    POST가 유실되므로, 뜬 뒤에 호출해야 확실히 닿는다. 제어권은 절대 안 줌
    (엘베앱 켜기 ≠ 제어권 주기 — 리스 부여는 _grant_elev_lease 몫)."""
    import urllib.request
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            urllib.request.urlopen("http://localhost:5000/authority", timeout=0.5)
            return True      # 응답 = 5000 떴음
        except Exception:
            time.sleep(0.4)
    return False

def _grant_elev_lease(granted: bool, reason: str = "") -> bool:
    """제어권 리스 부여/회수 — 한 여정에 딱 1회, 층 이동 전 반납.
    리스 보유 중엔 guard(라이다 충돌가드)가 꺼지므로, 반납 후 guard_off가
    실제로 복원됐는지 실측 검증한다(의도가 아니라 실측 — POST 실패해도
    _set_elev_authority는 내부 플래그를 갱신하므로 실측 없인 못 잡는다).
    반환값은 호출부가 반드시 확인해야 함 — grant는 실패해도 fail-closed(안전)라
    재시도 안 함(1회), revoke는 가드 복원이 핵심이라 2회 재시도."""
    global _elev_lease_held, _lease_renewer_thread
    if not granted:
        # 반납 진입 시 하트비트부터 멈춘다 — 안 그러면 갱신 스레드가 반납 직후
        # 다시 True로 되돌릴 수 있음(revoke가 항상 하트비트보다 우선해야 함)
        _lease_stop.set()
        if _lease_renewer_thread is not None:
            _lease_renewer_thread.join(timeout=1.0)
            _lease_renewer_thread = None
    attempts = 1 if granted else 2
    for i in range(attempts):
        _set_elev_authority(granted, reason)
        time.sleep(0.4)
        st = _elev_status(timeout=1.0)
        ok = bool(st) and (st.get("authority") is granted) and \
             (granted or st.get("guard_off") is False)
        _log("ELEVLEASE", f"리스={granted} 시도{i+1}/{attempts} → 실측 "
                          f"authority={st and st.get('authority')} "
                          f"guard_off={st and st.get('guard_off')} ({reason})")
        if ok:
            _elev_lease_held = granted
            if granted:
                _lease_stop.clear()
                _lease_renewer_thread = threading.Thread(target=_lease_renewer, daemon=True)
                _lease_renewer_thread.start()
            return True
    if not granted:
        _log("ELEVLEASE", "🚨 리스 반납 실패 — 가드 미복원 상태로 엘베앱이 살아있음")
    return False

@app.route("/elevator_app_status")
def elevator_app_status():
    _run = _elev_app_running()
    # 꺼져 있으면 모드는 '없음'이지 지난번 값이 아니다 — 화면이 옛 모드를 붙들고
    # 있으면 그게 곧 거짓말이다.
    return jsonify(running=_run, authority=_elev_authority,
                   no_ocr=(_elev_no_ocr if _run else None))

@app.route("/elevator_app", methods=["POST"])
def elevator_app():
    global _elev_lease_held, _lease_renewer_thread, _elev_started_mono, _rescue_hold, _elev_no_ocr
    data    = request.json or {}
    desired = data.get("running")     # True=시작, False=종료, None=토글
    running = _elev_app_running()

    # 종료 (또는 토글인데 켜져 있음)
    if desired is False or (desired is None and running):
        # 0) 하트비트부터 정지 — 앱 kill 전에 걸어야 종료 창 동안 재부여가 안 튐
        #    (phantom 하트비트가 재기동된 앱에 조용히 재부여하는 갭 차단)
        _lease_stop.set()
        # 1) 제어권 먼저 회수 → 엘베가 즉시 정지·타겟 해제 (프로세스 살아있을 때 받게)
        _set_elev_authority(False, "엘리베이터 앱 종료")
        time.sleep(0.3)
        # 2) 프로세스 종료 (SIGTERM → 엘베앱 finally 정리로 서보·바퀴 깨끗이)
        p = _procs.get("elevator")
        if p and p.poll() is None:
            try:
                p.terminate()
            except Exception:
                pass
        try:
            subprocess.run(["pkill", "-f", "elevator_button_press/main.py"],
                           capture_output=True)
        except Exception:
            pass
        _procs.pop("elevator", None)
        _elev_started_mono = None   # 꺼진 앱에 고립 판정을 하지 않는다
        if _lease_renewer_thread is not None:
            _lease_renewer_thread.join(timeout=1.0)
            _lease_renewer_thread = None
        # 프로세스가 죽으면 리스는 물리적으로 소멸 — abort·수동종료·8단계가 전부
        # 이 라우트를 지나므로 여기 한 곳만 리셋하면 죽은 앱에 finally가 재POST해
        # 가짜 🚨를 내는 것도 자연히 없어짐(_elev_lease_held=False라 no-op)
        _elev_lease_held = False
        _rescue_hold     = False   # 구조용 리스 유지의 유일한 해제 지점
        _elev_no_ocr     = None    # 껐으니 모드도 모름 — 다음 기동이 다시 채운다
        _log("MAIN", "엘리베이터 앱 종료 + 제어권 회수 → 주행(nav) 복귀")
        return jsonify(ok=True, running=False)

    # 이미 켜져 있음 → 앱만 켜진 상태 유지, 제어권은 건드리지 않음(엘베앱 켜기 ≠ 제어권 주기)
    if desired is True and running:
        # 요청한 모드와 실행 중인 모드가 다르면 조용히 ok를 돌려주지 않는다 —
        # 아무 일도 안 일어났는데 UI는 성공으로 읽는다. 그렇다고 여기서 앱을
        # 재시작하지도 않는다: 사용자가 제어권을 쥐고 여정 중일 수 있고, 앱을
        # 죽이는 위험은 #92에서 확인했다. 정직하게 실패하고 사람이 끄고 켜게 한다.
        want_photo = bool(data.get("photo"))
        st = _elev_status(timeout=1.0)
        if st is not None and bool(st.get("no_ocr")) != want_photo:
            return jsonify(ok=False, running=True, error=(
                "엘베앱이 사진모드로 실행 중입니다 — 정상 모드로 바꾸려면 끄고 다시 켜세요"
                if st.get("no_ocr") else
                "엘베앱이 정상 모드로 실행 중입니다 — 사진모드로 바꾸려면 끄고 다시 켜세요"
            )), 409
        # 모드를 못 읽으면 불일치를 단정하지 않는다. 여기서까지 fail-closed로 막으면
        # 앱 토글 자체가 막힌다 — 아래 /auto_goto와 방향이 다른 이유는 위험의
        # 비대칭이다: 여정 시작은 사람이 갇히지만, 앱 토글의 최악은 "다시 누르기"다.
        return jsonify(ok=True, running=True)

    # 시작 (경로는 __file__ 기준이라 cwd 무관, 부모 env(ROS·FastDDS) 상속)
    # photo=True면 사진모드 — OCR 서버 없이 카메라만. 기본은 지금 그대로(플래그 없음).
    # 여정(_auto_run)이 직접 띄우는 경로는 이 라우트를 지나지 않으므로 영향 없다.
    argv = [sys.executable, "-u", str(THIS_DIR / "elevator_button_press/main.py")]
    if data.get("photo"):
        argv.append("--no-ocr")
    proc = subprocess.Popen(
        argv,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    _procs["elevator"] = proc
    _elev_started_mono = time.monotonic()   # 고립 판정 유예 기준(=기동 시각)
    threading.Thread(target=_capture, args=(proc, "ELEV"), daemon=True).start()
    # 앱만 켠다 — 제어권은 안 줌(엘베앱 켜기 ≠ 제어권 주기). 수동 사용 시 UI의
    # "제어권 부여" 토글로, 자동 여정 중엔 _auto_run이 _grant_elev_lease로 부여.
    threading.Thread(target=_wait_elev_app_up, daemon=True).start()
    _log("MAIN", f"엘리베이터 앱 시작 PID={proc.pid} (제어권은 별도 부여 필요)"
                 + (" — 사진모드(--no-ocr): 버튼 인식 없음, 여정 불가"
                    if data.get("photo") else ""))
    return jsonify(ok=True, running=True)

# ── 반자동 엘리베이터 여정 오케스트레이터 ──────────────────────────────────────────
# 목적지(예: '504호(연구실)')를 받아 [승차지점 주행 → 엘베 호출/탑승/층선택/하차 →
# 지도전환 → 목적지 주행]을 자동 진행. 블로커 단계(호출 press, 탑승 등)에서는 멈춰
# 사용자 '다음 확인'을 기다린다(반자동). 도착 감지는 로봇위치 vs 목표좌표 거리로 판단.
import math as _math_auto

def _loc(name):
    """location.yaml에서 한 지점의 dict(x/y/floor 등) 반환 (없으면 None)."""
    try:
        import yaml as _yaml
        locs = _yaml.safe_load(open(THIS_DIR / "../config/location.yaml"))["locations"]
        return locs.get(name)
    except Exception:
        return None

def _dist_to(x, y):
    """현재 로봇 위치에서 (x,y)까지 거리(m). 위치 미수신/좌표없음이면 None."""
    if _robot_pose["x"] is None or x is None or y is None:
        return None
    return _math_auto.hypot(_robot_pose["x"] - float(x), _robot_pose["y"] - float(y))

_AUTO = {"active": False, "dest": "", "step": "", "msg": "",
         "waiting": False, "cancel": False,
         "phase": "", "dest_floor": "", "mode": ""}
_auto_lock = threading.Lock()

def _auto_set(step, msg, wait=False, phase=None):
    """단계 상태 갱신. phase=전체 흐름 트래커에서 하이라이트할 단계 id(없으면 유지)."""
    with _auto_lock:
        _AUTO["step"] = step; _AUTO["msg"] = msg; _AUTO["waiting"] = wait
        if phase is not None:
            _AUTO["phase"] = phase
    _log("AUTO", f"[{step}] {msg}" + ("  — 확인 대기" if wait else ""))

def _auto_wait_arrival(name, tol=0.10, settle=1.0, timeout=200):
    """name 지점 '정밀 도착'까지 대기. nav이 5cm로 서므로, 여기선 목표 tol(기본 10cm)
    이내에서 로봇이 settle초간 '멈춰있으면'(=nav 완료 = 정밀 도착) True.
    - 단순히 tol 이내를 지나가는 중인 것과 구분하려고 '정지'까지 확인 (jitter 여유로 10cm).
    - nav 정밀도(5cm)와 일관: 로봇은 ≤5cm에 서고, 오케스트레이터는 그 정지를 감지해 진행.
    cancel/timeout이면 False."""
    p = _loc(name)
    if not p:
        return False
    tx, ty = p.get("x"), p.get("y")
    t0 = time.monotonic()
    stable_since = None
    last = None
    while time.monotonic() - t0 < timeout:
        if _AUTO["cancel"]:
            return False
        rx, ry = _robot_pose["x"], _robot_pose["y"]
        d = _dist_to(tx, ty)
        near    = (d is not None and d <= tol)
        stopped = (last is not None and rx is not None
                   and _math_auto.hypot(rx - last[0], ry - last[1]) < 0.02)  # 폴링 간 2cm 미만 = 정지
        if near and stopped:
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= settle:
                return True      # 목표 이내 + settle초 정지 = 정밀 도착 확정
        else:
            stable_since = None
        if rx is not None:
            last = (rx, ry)
        time.sleep(0.3)
    return False

def _auto_wait_confirm(timeout=900):
    """블로커 단계 — 사용자 '다음 확인' 대기. /auto_confirm이 waiting=False로 풀어줌."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if _AUTO["cancel"]:
            return False
        with _auto_lock:
            if not _AUTO["waiting"]:
                return True
        time.sleep(0.2)
    return False

def _elev_post(path, payload=None, timeout=20):
    """엘베앱(5000) POST — 응답 JSON(dict) 또는 None."""
    try:
        import urllib.request
        r = urllib.request.urlopen(urllib.request.Request(
            f"http://localhost:5000{path}",
            data=json.dumps(payload or {}).encode(),
            headers={"Content-Type": "application/json"}, method="POST"), timeout=timeout)
        return json.loads(r.read().decode() or "{}")
    except Exception as e:
        _log("AUTO", f"엘베 {path} 실패: {e}")
        return None

@app.route("/snapshot", methods=["POST"])
def snapshot_route():
    """VLM 매핑용 스냅샷 — 저장은 전부 엘베앱(5000)이 함(원본 프레임을 이미 갖고 있음).
    대시보드는 게이트(이동 중 거부) + 엘베앱이 모르는 대시보드 쪽 컨텍스트만 실어 전달."""
    if (time.monotonic() - _last_move_cmd) <= 1.0:
        return jsonify(ok=False, error="이동 중에는 스냅샷 불가"), 409
    if not _elev_app_running():
        return jsonify(ok=False, error="엘베앱 실행 필요"), 409
    data = request.json or {}
    payload = {
        "label": (data.get("label") or "").strip(),
        "floor": _current_floor,
        "battery": dict(_battery),
        "amcl_pose": dict(_robot_pose) if _robot_pose.get("x") is not None else None,
        "dash_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    result = _elev_post("/snapshot", payload, timeout=10)
    if result is None:
        return jsonify(ok=False, error="엘베앱 응답 없음"), 502
    return jsonify(result)

def _manual_arm_gate():
    """팔 수동조작 공통 관문. 막을 이유가 있으면 (사유, HTTP코드), 없으면 None.

    셋 다여야 실제로 움직인다: 엘베앱이 떠 있고, 제어권 리스를 쥐고 있고,
    자동 여정 중이 아니다. 하나라도 아니면 눌러도 아무 일이 안 일어나는데,
    화면이 그걸 안 알려주면 "슬라이더가 고장났다"로 읽힌다 — 조용히 안 되는
    조작기를 하나 더 만드는 셈이라 사유를 그대로 돌려준다.
    자동 여정 중인지는 대시보드만 아는 사실이라 여기서 막는다(엘베앱은 모른다)."""
    if _AUTO.get("active"):
        return "자동 여정 중 — 여정을 멈춘 뒤 조작하세요", 409
    if not _elev_app_running():
        return "엘베앱 꺼짐 — 엘리베이터 앱을 먼저 켜세요", 409
    if not _elev_lease_held:
        return "제어권 없음 — 대시보드에서 제어권을 부여하세요", 403
    return None


@app.route("/manual_lift", methods=["POST"])
def manual_lift():
    """팔 높이 수동 조정 프록시 — 실제 이동·범위 제한은 엘베앱 /lift가 한다."""
    blocked = _manual_arm_gate()
    if blocked:
        return jsonify(ok=False, error=blocked[0]), blocked[1]
    result = _elev_post("/lift", {"lift": (request.json or {}).get("lift")}, timeout=5)
    if result is None:
        return jsonify(ok=False, error="엘베앱 응답 없음"), 502
    return jsonify(result)


@app.route("/manual_arm_ext", methods=["POST"])
def manual_arm_ext():
    """팔 뻗기 수동 조정 프록시 — 1회 이동 상한·안전범위는 엘베앱 /arm_ext가
    강제한다(현재값을 아는 쪽이 거기다). 여기서 다시 자르지 않는다."""
    blocked = _manual_arm_gate()
    if blocked:
        return jsonify(ok=False, error=blocked[0]), blocked[1]
    result = _elev_post("/arm_ext", {"arm_ext": (request.json or {}).get("arm_ext")}, timeout=5)
    if result is None:
        return jsonify(ok=False, error="엘베앱 응답 없음"), 502
    return jsonify(result)


@app.route("/manual_arm_state")
def manual_arm_state():
    """팔 컨트롤 활성 여부 + 이유 + 현재 관절값 — UI가 비활성 사유를 그대로
    보여주고, 슬라이더를 실제 위치에 맞추는 데 쓴다. 값은 엘베앱이 재는 것을
    그대로 옮길 뿐이고 여기서 판정하지 않는다(못 물으면 None)."""
    blocked = _manual_arm_gate()
    lift = arm_ext = None
    if _elev_app_running():
        st = _elev_status(timeout=1.0)
        if st:
            lift, arm_ext = st.get("lift"), st.get("arm_ext")
    return jsonify(ok=True, enabled=(blocked is None),
                   reason=("" if blocked is None else blocked[0]),
                   lift=lift, arm_ext=arm_ext)


def _elev_status(timeout=3):
    """엘베앱 상태(/status) 조회 — dict 또는 None. ready=정렬완료, door_open 등 포함."""
    try:
        import urllib.request
        r = urllib.request.urlopen("http://localhost:5000/status", timeout=timeout)
        return json.loads(r.read().decode() or "{}")
    except Exception:
        return None

def _elev_scene(n):
    """엘베앱 씬 n 트리거(자세 전환·자동안무). 자세 전송이 블로킹(~10s)이라 여유 타임아웃."""
    return _elev_post("/scene", {"n": int(n)}, timeout=20) is not None

def _elev_select(text):
    """버튼 자동 선택(POST /select). 호출=^(상)/s(하), 층=번호. 성공 시 True."""
    r = _elev_post("/select", {"text": str(text)}, timeout=5)
    return bool(r and r.get("ok", True))

def _elev_press():
    """누르기 실행(POST /press). (성공여부, 사유) 반환.

    사유를 버리면 "왜 거부됐는지"가 여기서 끊긴다 — 거부 대부분은 고장이 아니라
    "지금은 안 된다"라서, 사유를 봐야 기다릴 일인지 멈출 일인지 가른다.
    사유는 응답이 없을 때 None."""
    r = _elev_post("/press", {}, timeout=15)
    return bool(r and r.get("ok")), (r or {}).get("error")

def _elev_wait_ready(timeout=45):
    """정렬 완료(centered && press_ready) 대기 — 취소 존중. 성공 True / 타임아웃·취소 False."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        with _auto_lock:
            if _AUTO.get("cancel"):
                return False
        st = _elev_status()
        if st and st.get("ready"):
            return True
        time.sleep(0.3)
    return False

def _elev_wait_press_done(timeout=30):
    """누르기 완료 대기 — press 씬(0/4)의 scene_next_ok(press_ok_ts>scene_ts) True까지.
    이게 True면 '버튼 눌림 + 팔 복귀 + 그리퍼 열기'까지 끝난 상태라 이동해도 안전.
    취소 존중. 완료 True / 타임아웃·취소 False.
    ※ False면 _auto_run은 흐름을 계속하지 않고 여정을 중단한다(S1) — 팔이
    복귀했는지 확인할 다른 수단이 없어서, 모르면 베이스를 안 움직인다."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        with _auto_lock:
            if _AUTO.get("cancel"):
                return False
        st = _elev_status()
        if st and st.get("scene_next_ok"):
            return True
        time.sleep(0.3)
    return False

# ⑥ 하차(후진 186cm) 완료 판정용. 목표값은 엘베앱 SCENE_MOVES[5]와 같은 수치다.
_EXIT_TARGET_CM = -186.0
_EXIT_TOL_CM    = 1.0     # 도달 허용 오차. 넓히지 않는다 — scene_acc는 연속이 아니라
                          # 스텝 완료마다 점프하므로(0 → -51.9 → -102.9 → -153.8 →
                          # -187.2 → -184.0) tol을 넓히면 done이 오버슛 시점(-187.2)에
                          # 터진다. 앱이 +3.1cm 보정을 하러 가기 직전이다 — 대시보드가
                          # 완료를 선언하고 리스를 회수해 그 보정이 "제어권 없음"으로
                          # 거부된다. #92가 없애려던 구조를 게이트 안에 다시 심는 꼴이라,
                          # 오차는 tol이 아니라 아래 강등(_EXIT_NEAR_CM)으로 흡수한다.
# 진동 break로 스스로 멈춘 완주를 실패로 오판하지 않기 위한 강등 임계.
# 유도가 아니라 실측이다 (~/.ros/log/python3_12606_1784856324677.log:217, ⑥ 하차):
#   - 스텝 오버슛: 명령 -50cm → 실제 -51.9 / -51.0 / -50.8 = 1~2cm
#   - 진동 break 시점 잔여: 최대 2cm. 관측 2건 — ⑥ 하차 1건(잔여 2.0cm, 위 로그)과
#     ② 문앞 정렬 1건(잔여 1cm, python3_303226_1784871041433.log:97). 씬은 다르지만
#     둘 다 같은 _run_scene_moves_inner 진동 감지 경로다. ⑥ 완주 4건 중 이 경로는 1건.
#   - 5.0 = 관측 최대의 2.5배 마진
# "186cm면 문틀 마진이 얼마"라는 근거가 아니다 — 그건 아무도 잰 적이 없다.
# 근거는 "관측된 진동 잔여가 최대 2cm"뿐이다. #92가 잡으려는 진짜 실패는 잔여
# 136cm 이상이라 27배 떨어져 있어 이 창으로 새어나올 수 없다.
_EXIT_NEAR_CM   = 5.0
_EXIT_STALL_SEC = 10.0    # 첫 진행이 기록된 뒤, 이만큼 진행이 없으면 실패
# 첫 진행이 기록되기 전에만 쓰는 유예. scene_acc는 스텝이 '끝나야' 갱신되므로
# (엘베앱 _manual_trans 말미) 그 전까지는 정상 동작 중에도 진행이 0으로 보인다.
# 최악 합(엘베앱 main.py): 이전 안무 스레드 종료 대기 3.0s(L2525) + 이동자세
# _goal_done 대기 5.0s(L2531) + 첫 50cm 스텝 자체 타임아웃 8.5s
# (abs(0.5)/BOARD_SPEED 0.20 * 3.0 + 1.0, L2412) = 16.5s. 여기에 여유 3.5s.
# 정상값은 5~6s지만 스핀이 굶으면 5.0s 상한을 다 쓴다 — 오판이 가장 잘 나는
# 조건(nav+엘베 과부하)에서 나므로 최악 합으로 잡는다.
# 이건 이론값이 아니다. 과거 ⑥ 완주 4건 실측:
#   - _goal_done 5.0s 상한이 4건 중 2건에서 실제로 걸렸다
#     (python3_313116_1784103398500.log 5.07s, python3_12606_1784856324677.log 5.11s)
#   - 첫 진행 지연 최대 11.70s(python3_313116_1784103398500.log) — 구 임계 10s를
#     이미 넘겼다. 오판은 가정이 아니라 과거 로그에 이미 찍혀 있었다.
#   ※ 날짜 대신 로그 경로로 적는다 — 이 데이터셋에서 epoch→날짜 변환 실수가
#     두 번 났다(경로는 검산이 필요 없다). 날짜가 필요하면 파일명 뒤 숫자의 앞
#     10자리로 date -d @<epoch>를 직접 돌릴 것. 위 두 파일은 그렇게 검산해
#     각각 2026-07-15, 2026-07-24다. 완주 4건은 07-15·07-21·07-24·07-24이고,
#     ⑥ 하차 완주 로그는 python3_{313116,117654,12606,57478}_*.log 넷이다.
#   - 20.0 = 관측 최대 11.7s에 8.3s, 이론 최악 16.5s에 3.5s 여유
_EXIT_START_GRACE = 20.0
_EXIT_MAX_SEC   = 60.0    # 절대 상한(무한대기 방지 백스톱)
_EXIT_POLL_SEC  = 0.4
_EXIT_MISS_MAX  = 2       # /status 무응답이 이만큼 연속되면 앱이 죽은 것으로 본다

# ⑥ 하차가 끝나지 않은 채 여정이 끊긴 상태 — finally가 리스를 되돌리지 못하게 막는다.
# 해제는 /elevator_app {running:false} 한 곳에서만 (거기서 리스도 물리적으로 소멸).
_rescue_hold = False


def _elev_wait_exit_done():
    """⑥ 하차(후진 186cm)를 '진행량'으로 확인. (사유, 진행cm) 반환.

    사유: "done"=목표 도달 / "stall"=무진행 / "timeout"=절대상한 /
          "cancel"=사용자 취소 / "noapp"=엘베앱 상태 조회 불가

    /scene은 안무 스레드를 start한 뒤 즉시 ok를 돌려준다 — 접수이지 완료가 아니다.
    상수 총시간으로 대신할 수도 없다: _manual_step 한 스텝의 자체 타임아웃만
    50cm 최악 8.5s라 총합이 ~40s까지 늘어난다(25~30s 상수는 정상 완주를 실패로
    오판한다). 그래서 '얼마나 걸렸나'가 아니라 '진행이 멎었나'로 본다."""
    t0 = time.monotonic()
    last_prog = t0
    best = None          # 지금까지 진행한 최대 거리(cm, 절대값)
    cur  = None          # 마지막으로 읽은 누적 진행량 — 실패 통보에 그대로 실린다
    miss = 0             # /status 연속 무응답 횟수
    while True:
        if _AUTO["cancel"]:
            return "cancel", cur
        st = _elev_status(timeout=1.0)
        if st is None:
            # 1회 표본으로 단정하지 않는다. noapp은 stall과 달리 _rescue_hold를
            # 세우지 않아 finally가 리스를 반납하는데, 일시적 끊김에 오탐하면
            # 이 함수가 지키려던 구조용 조종 패드를 그대로 잃는다.
            # 그렇다고 60초 상한까지 헛돌지도 않는다 — 2회면 충분하다. 비용은
            # 최악 ≈2.4s(무응답이 _elev_status timeout 1.0s를 다 쓸 때 1.0+0.4+1.0),
            # 포트가 닫혀 즉시 거부되면 ≈0.4s다.
            miss += 1
            if miss >= _EXIT_MISS_MAX:
                return "noapp", cur
        else:
            miss = 0
            v = (st.get("scene_acc") or {}).get("fwd_cm")
            if isinstance(v, (int, float)):
                cur = float(v)
                if abs(cur - _EXIT_TARGET_CM) <= _EXIT_TOL_CM:
                    return "done", cur
                if best is None or abs(cur) - best > 0.5:   # 0.5cm = 진행으로 칠 최소량
                    best = abs(cur)
                    last_prog = time.monotonic()
        now = time.monotonic()
        # best > 0.5 = 실제 진행이 한 번이라도 기록됨. 첫 폴링에서 0.0을 읽은 것은
        # 진행이 아니므로 그때까지는 유예를 쓴다.
        limit = _EXIT_STALL_SEC if (best is not None and best > 0.5) else _EXIT_START_GRACE
        if now - last_prog >= limit:
            # 진동 감지로 안무가 스스로 마친 완주는 여기서 무진행으로 보인다.
            # 잔여가 충분히 작으면 성공으로 강등해 정상 경로에 합류시킨다.
            if cur is not None and abs(_EXIT_TARGET_CM - cur) <= _EXIT_NEAR_CM:
                _log("AUTO", f"⑥ 하차 완료 (잔여 {abs(_EXIT_TARGET_CM - cur):.1f}cm "
                             f"— 진동 보정으로 안무가 스스로 마침, 누적 {cur:+.1f}cm)")
                return "done", cur
            return "stall", cur
        if now - t0 >= _EXIT_MAX_SEC:
            return "timeout", cur
        time.sleep(_EXIT_POLL_SEC)


_EXIT_WHY = {"stall":   "후진이 멈췄습니다",
             "timeout": "제한 시간을 넘겼습니다",
             "cancel":  "취소되었습니다",
             "noapp":   "엘리베이터 앱 응답이 없습니다"}


def _exit_failed(reason: str, cur):
    """⑥ 하차 미완료 — 여기서 여정을 끊고, 사람이 로봇을 빼낼 수단을 남긴다.

    _auto_abort_elev()를 부르지 않는다: 그건 엘베앱을 종료시켜 조종 패드·상태·
    유일한 구조 경로를 통째로 없앤다. 하차 실패는 '정리하고 끝낼' 상황이 아니라
    '사람이 개입해 로봇을 빼내야 할' 상황이다.

    리스도 반납하지 않는다(noapp 제외). 반납하면 authority=False가 되어 엘베앱
    패드의 /step_move가 _manual_step 첫 줄에서 거부된다 — 문턱에 낀 로봇을 빼낼
    유일한 수단이 사라진다. 회수 상태에서 가드가 돌아오는 것은 안전 이득이 아니다:
    라이다 가드는 별도 브레이크가 아니라 _manual_step 안의 여유거리 검사일 뿐이라
    authority가 False면 검사할 이동 자체가 없다.

    가드는 따로 복원하지도 않는다 — 리스 유지 = 하트비트(2s) 유지인데, 엘베앱
    /authority POST가 하트비트마다 guard_off=True로 되돌린다. 하트비트를 멈추면
    deadman(LEASE_TTL 6s)이 제어권을 회수해 구조 경로가 사라진다. 가드보다
    구조 경로가 우선이다.

    noapp만 예외다: 앱이 죽었으면 리스는 이미 물리적으로 소멸했고, 여기서
    _rescue_hold를 세우면 하트비트가 살아남아 앱 재기동 시 조용히 재부여하는
    유령 갱신이 된다. 그 경우엔 finally의 정상 반납 경로로 보낸다.

    통보에는 형용사가 아니라 숫자를 싣는다 — 구조하러 오는 사람에게 필요한 건
    '완료되지 않았다'가 아니라 '186cm 중 몇 cm까지 나왔나'다."""
    global _rescue_hold
    # 무엇보다 먼저 바퀴를 멈춘다. 구코드는 3초 뒤 리스를 반납해 엘베앱
    # _revoke_authority가 _step_abort + Twist() 정지를 대신 해줬다 — 우연히
    # 멈추고 있었던 것이다. 리스를 유지하는 이 경로에는 그 회수가 없으므로
    # 명시적으로 멈추지 않으면 안무 스레드가 남은 후진을 계속한다.
    # "로봇이 걸쳐 있으니 도움을 요청하세요"라고 통보한 뒤 로봇이 계속
    # 움직이면, 구조하러 온 사람이 움직이는 로봇을 만난다 — 사람을 태운 채로.
    # noapp도 예외를 두지 않는다: noapp은 연속 2회 무응답에서 나온 '추정'이지
    # 사망 증명이 아니다. 앱이 살아 있는데 느렸던 경우가 정확히 멈춰야 할
    # 경우이고, 정말 죽었으면 POST가 실패할 뿐 해가 없다.
    stopped = _elev_post("/step_stop", {}, timeout=3) is not None
    _log("AUTO", "⑥ 하차 실패 → 베이스 정지 요청 " +
                 ("전송됨(_step_abort) — 안무·현재 스텝 중단"
                  if stopped else "🚨 전송 실패 — 로봇이 계속 움직일 수 있다"))
    moved = f"{abs(cur):.0f}cm" if isinstance(cur, float) else "확인 불가"
    why   = _EXIT_WHY.get(reason, reason)
    if reason != "noapp":
        _rescue_hold = True
    _auto_notify(f"하차가 끝나지 않았습니다 — 186cm 중 {moved}만 후진했습니다. "
                 f"{why}. 로봇이 엘리베이터에 걸쳐 있을 수 있으니 도움을 요청하세요")
    _log("AUTO", f"⑥ 하차 미완료({reason}) 진행 {moved}/186cm — "
                 + ("제어권 유지(구조용 조종 패드 보존) — 엘베앱 종료 시 해제"
                    if reason != "noapp" else "앱 응답 없음 — 리스는 이미 소멸"))
    _auto_set("오류", f"🚨 하차 미완료({reason}) — 186cm 중 {moved} · 여정 중단"
                      + (" · 제어권 유지, 엘베앱 조종 패드로 빼낼 것"
                         if reason != "noapp" else ""))


def _auto_abort_elev():
    """여정 취소 시 안전 정리: 엘베앱 종료(→제어권 회수→가드 복구)."""
    _auto_set("취소", "여정 취소됨 — 엘베앱 종료·제어권 회수")
    _http_self("/elevator_app", {"running": False})

def _http_self(path, payload):
    """대시보드 자기 자신(8080)의 라우트를 호출 (지도전환 등 재사용)."""
    try:
        import urllib.request
        urllib.request.urlopen(urllib.request.Request(
            f"http://localhost:8080{path}",
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"}, method="POST"), timeout=10)
        return True
    except Exception as e:
        _log("AUTO", f"{path} 호출 실패: {e}")
        return False

# 중단 지점은 전부 팔이 뻗어 있을 수 있는 자리인데 자동 수납은 하지 않는다
# (그리퍼 선행 닫기가 패널·문틀에 걸린다). 그 상태로 밀거나 수동주행하면
# 막으려던 충돌이 사람 손으로 다시 난다 — 운영자에게만 알린다.
_ARM_STOW_NOTE = ("팔이 뻗은 상태일 수 있음 — 대시보드 '팔 수납'(/stow_arm)으로 "
                  "넣은 뒤 밀거나 수동주행할 것")


def _auto_notify(msg: str, voice: bool = True, stow_hint: bool = False):
    """여정 거부·중단을 알린다. 조용히 멈추면 그게 또 무증상 실패다.

    운영자 로그는 언제나 남기고, voice=True면 같은 문장을 음성으로도 보낸다 —
    화면과 음성이 갈라지면 어느 쪽을 믿을지 모르게 되므로 문장을 공유한다.

    왜 이제 음성이 되나: interface에 낭독 전용 /say 명령을 뒀다. 별도
    kind='say' 이벤트라 사용자 발화(text)로 오인되지 않고, _handle_text의
    상태 게이트(NAV·LOCKED에서 즉시 return)를 타지 않는다. 낭독은 이벤트 루프
    스레드가 아니라 데몬 스레드에서 돈다 — 루프에서 동기로 재생하면 재생이
    끝날 때까지 버튼·당김·취소가 처리되지 않는다(모달이 Esc를 막던 것과 같은
    결함). 막는 것은 AudioGate가 아니라 루프 스레드를 물고 있는 동기 재생이다.
    그 스레드 안에서는 _say를 쓴다 — AudioGate가 재생 동안 마이크를 막아
    TTS 소리를 로봇이 자기 발화로 되받는 것을 방지한다.

    voice=False는 운영자 전용 통보다 — 시각장애인이 할 수 없는 조치(팔 수납)를
    음성으로 읽어주면 도움이 안 된다. stow_hint=True면 그 수납 안내를 운영자
    로그에만 덧붙인다(중단 지점은 팔이 뻗어 있을 수 있는 자리라서).

    _write 반환으로 전달 실패는 로그에 남지만, 성공해도 "stdin에 썼다"까지지
    "들렸다"는 아니다(전송성공≠완료)."""
    if not msg.strip():
        return          # 빈 통보는 화면에도 음성에도 의미가 없다
    _log("AUTO", f"🚨 여정 중단 통보: {msg}")
    if stow_hint:
        _log("AUTO", f"↳ 운영자 안내: {_ARM_STOW_NOTE}")
    if not voice:
        return
    if not _write("iface", f"/say {msg}"):
        _log("AUTO", f"음성 전달 실패(iface 없음/죽음): {msg}")


# /press 거부 사유별 처방. 문자열은 엘베앱 start_press()가 돌려주는 그대로이고
# 수치가 섞인 사유(정렬 어긋남·너무 멂·아직 접근 완료 전)가 있어 부분일치로 본다.
#   RETRY_WAIT : 시간이 지나면 저절로 풀리는 것 — 기다렸다가 한 번 더
#   RETRY_HELP : 사람이 조준을 고쳐야 풀리는 것 — 운영자 개입 후 한 번 더
# 여기 없는 사유는 전부 중단이다(분류를 못 하면 계속 갈 근거가 없다).
_PRESS_RETRY_WAIT = ("인식이 멈춰 있음", "버튼 관측이 오래됨", "아직 접근 완료 전")
_PRESS_RETRY_HELP = ("CENTERED 상태가 아님", "정렬 어긋남")
_PRESS_WAIT_SEC   = 4.0     # 재인식·접근이 한 번 더 돌 만큼만. 길게 잡으면 사용자가 방치된다


def _auto_sleep(sec: float) -> bool:
    """취소를 존중하는 대기. 취소되면 False.
    time.sleep 한 방으로 자면 그 동안 '취소'가 안 먹는다 — 사람이 멈추라고 한
    뒤에도 로봇이 다음 동작으로 넘어가는 게 제일 나쁘다."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        if _AUTO["cancel"]:
            return False
        time.sleep(0.2)
    return not _AUTO["cancel"]


def _press_or_pass() -> bool:
    """누르기 시도. 여정을 계속해도 되면 True, 중단해야 하면 False.

    /press 거부가 곧 실패는 아니다 — "이미 누르기 진행 중"처럼 거부와 동시에
    실제로는 누르기가 돌고 있는 경우가 있어서, 사유만 보고 끊으면 멀쩡히 눌리는
    중인 여정을 죽인다. 그래서 거부 뒤에는 반드시 상태를 한 번 물어보고,
    진행 중(pressing)이거나 이미 끝났으면(scene_next_ok) 그대로 완료 대기로
    넘긴다. 상태조차 못 물으면 팔이 어떤 자세인지 알 방법이 없으므로 중단한다
    — 모르면 안 움직인다.

    여기서는 아무것도 움직이지 않는다. 팔 복귀 확인(arm_safe)은 여전히
    _elev_wait_press_done()만 풀 수 있다(S1 불변식)."""
    ok, reason = _elev_press()
    if ok:
        return True
    why = reason or "사유 없음"
    st = _elev_status()
    if st is None:
        _auto_notify("누르기 명령을 보내지 못해 멈췄습니다", stow_hint=True)
        _auto_set("오류", f"누르기 거부({why}) + 엘베앱 상태 조회 실패 — 여정 중단")
        return False
    if st.get("pressing") or st.get("scene_next_ok"):
        # 거부는 됐지만 실제로는 눌리는 중이거나 이미 끝났다 — 사용자에게 알릴
        # 일이 아니라 정상 진행이므로 음성 없이 통과시킨다.
        _log("AUTO", f"누르기 거부({why})지만 엘베앱 상태는 진행 중 — 통과")
        return True

    # ── 사유별 처방 — 재시도는 통틀어 한 번뿐이다(직선 코드, 반복 없음) ──
    if any(k in why for k in _PRESS_RETRY_WAIT):
        # 재인식·자동 접근이 한 번 더 돌면 풀리는 사유 — 사람 손이 필요 없다.
        _auto_notify("잠시만 기다려 주세요")
        _auto_set("재시도", f"누르기 대기 후 재시도: {why}")
        if not _auto_sleep(_PRESS_WAIT_SEC):
            _auto_notify("여정을 멈췄습니다", stow_hint=True)
            _auto_set("오류", "취소됨 — 여정 중단")
            return False
    elif any(k in why for k in _PRESS_RETRY_HELP):
        # 조준이 어긋난 것 — 운영자가 엘베UI에서 맞춰줘야 풀린다.
        # 베이스를 움직이면 정렬이 통째로 날아가므로 그러지 말라고 못박는다.
        _auto_notify("버튼 위치를 다시 맞추고 있습니다. 잠시만 기다려 주세요")
        _auto_set("재시도",
                  f"누르기 거부: {why} — 베이스는 움직이지 말고 엘베UI 조준트림(⇧)·"
                  "lift(+/-)로 맞춘 뒤 '다음'", wait=True)
        if not _auto_wait_confirm():
            _auto_notify("여정을 멈췄습니다", stow_hint=True)
            _auto_set("오류", "취소됨 — 여정 중단")
            return False
    else:
        _auto_notify("누르기 명령을 보내지 못해 멈췄습니다", stow_hint=True)
        _auto_set("오류", f"누르기 거부: {why} — 여정 중단")
        return False

    ok2, reason2 = _elev_press()      # 재시도는 여기 한 번뿐
    if ok2:
        return True
    st2 = _elev_status()
    if st2 is not None and (st2.get("pressing") or st2.get("scene_next_ok")):
        _log("AUTO", f"재시도 거부({reason2 or '사유 없음'})지만 엘베앱 상태는 진행 중 — 통과")
        return True
    _auto_notify("누르기 명령을 보내지 못해 멈췄습니다", stow_hint=True)
    _auto_set("오류", f"재시도도 거부: {reason2 or '사유 없음'} — 여정 중단")
    return False


def _auto_run(dest):
    """반자동 여정 상태머신 (백그라운드 스레드)."""
    global _elev_started_mono, _rescue_hold
    # 지난 여정의 구조 유지 플래그를 물려받지 않는다 — 앱을 끄지 않고 새 여정을
    # 시작하면 True인 채 상속돼 이번 여정의 finally도 반납을 건너뛴다(가드가 계속
    # 꺼진 채 남는 #91 재발). 이건 해제(release)가 아니라 진입 시 상태 위생이고,
    # 실제 해제 지점은 여전히 /elevator_app {running:false} 한 곳뿐이다.
    _rescue_hold = False
    # 팔이 수납돼 있다고 볼 수 있는가. 대시보드는 /joint_states를 안 보므로 팔
    # 자세를 직접 못 잰다 — 엘베앱이 주는 "누르기+팔복귀+그리퍼열기 완료" 신호가
    # 유일한 근거다. 그래서 이 래치를 푸는 곳은 아래 단 두 곳(press_done True)뿐이고,
    # 다른 데서 True로 만들면 근거 없는 안전 주장이 된다.
    arm_safe = True     # 여정 시작 시점의 "가정" — 잰 값이 아니다. 직전에
                        # 운영자가 팔을 뻗어둔 채 여정을 시작하면 이 가정은 틀린다.
    try:
        with _auto_lock:
            _AUTO.update(active=True, dest=dest, cancel=False, phase="", mode="")
        d = _loc(dest)
        if not d:
            _auto_set("오류", f"'{dest}' 좌표를 location.yaml에서 못 찾음"); return
        dest_floor = str(d.get("floor") or _current_floor)
        with _auto_lock:
            _AUTO["dest_floor"] = dest_floor

        # 같은 층이면 엘베 없이 바로
        if dest_floor == _current_floor:
            with _auto_lock:
                _AUTO["mode"] = "same"
            _auto_set("주행", f"{dest} 바로 이동 (같은 층)", phase="drive")
            _write("iface", f"/goto {dest}")
            _auto_set("완료", f"{dest} 도착 ✅" if _auto_wait_arrival(dest) else "도착 실패",
                      phase="done")
            return

        with _auto_lock:
            _AUTO["mode"] = "elevator"
        up = int(dest_floor) > int(_current_floor)
        dir_txt = "▲ 상행" if up else "▼ 하행"
        _auto_set("시작", f"{dest}={dest_floor}층 / 현재 {_current_floor}층 → 엘베 {dir_txt}",
                  phase="board")

        # 1) 승차지점 주행 (자동)
        _auto_set("주행", "엘리베이터 탑승지점으로 이동 중...", phase="board")
        _write("iface", "/goto 엘리베이터 탑승지점")
        if not _auto_wait_arrival("엘리베이터 탑승지점"):
            _auto_set("오류", "승차지점 도착 실패(취소/시간초과)"); return
        _auto_set("도착", "승차지점 도착 ✅", phase="board")

        # 2) 엘베앱 ON + 제어권 리스 부여 (한 여정에 1회 — 유일한 자동 grant 지점)
        _auto_set("엘베시작", "엘리베이터 앱 시작 + 제어권 부여...", phase="app")
        if not _elev_app_running():
            proc = subprocess.Popen(
                [sys.executable, "-u", str(THIS_DIR / "elevator_button_press/main.py")],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            _procs["elevator"] = proc
            _elev_started_mono = time.monotonic()   # 고립 판정 유예 기준(=기동 시각)
            threading.Thread(target=_capture, args=(proc, "ELEV"), daemon=True).start()
        if not _wait_elev_app_up(20):
            _auto_set("오류", "엘베앱 기동 실패"); return
        if not _grant_elev_lease(True, "엘베 여정 시작"):
            _auto_set("오류", "제어권 부여 실패 — 여정 중단"); return

        # ── 엘리베이터 6 시나리오 ──────────────────────────────────────────
        #   버튼 선택(상/하행·층)·정렬 = 자동 / 누르기(press) = '다음'에 포함.
        #   씬: 0=①호출 1=②문앞 2=③문열림 3=④탑승 4=⑤층 5=⑥하차 (SCENES와 1:1)

        # ① 호출 press: 인식자세 → 호출버튼(상/하행) 자동선택 → 정렬 → '다음'에 누르기
        _auto_set("① 호출", f"인식 자세 + {dir_txt} 버튼 자동 선택·정렬 중...", phase="call")
        # 인식자세 자체는 lift·손목만 잡고 arm_extension은 건드리지 않는다.
        # 팔이 실제로 뻗는 것은 그 뒤 자동 접근·서보와 누르기 시퀀스다 —
        # 여기서 내려두는 건 "이 지점부터는 수납을 장담 못 한다"는 뜻이다.
        arm_safe = False
        _elev_scene(0)                          # place=hall + 인식자세(블로킹)
        _elev_select("^" if up else "s")        # 호출버튼 자동 선택 (^=상행 s=하행)
        if _elev_wait_ready():
            _auto_set("① 호출", f"정렬 완료 ✅ — '다음' 누르면 호출({dir_txt}) 누름", wait=True)
        else:
            _auto_set("① 호출", "⚠ 자동정렬 실패 — 엘베UI서 수동 정렬 후 '다음'", wait=True)
        if not _auto_wait_confirm(): _auto_abort_elev(); return
        if not _press_or_pass():                # 다음 → 호출버튼 누르기
            _auto_abort_elev(); return
        _auto_set("① 호출", "호출 버튼 누르는 중... (팔 복귀까지 대기)")
        # 눌림+팔복귀+그리퍼열기 완료까지 대기 — 이게 True여야 이동해도 안전하다.
        if not _elev_wait_press_done():
            _auto_notify("누르기를 확인하지 못해 멈췄습니다", stow_hint=True)
            _auto_set("오류", "누르기/팔복귀 미완료(타임아웃/취소) — 여정 중단")
            _auto_abort_elev(); return
        arm_safe = True                         # 유일한 release 지점 (1/2)

        # ② 문앞 정렬: 전진 56.5 + 우회전 90° (자동 안무)
        if not arm_safe:
            _auto_notify("팔이 안전한지 확인되지 않아 이동을 멈췄습니다", stow_hint=True)
            _auto_set("오류", "팔 복귀 미확인 — 베이스 이동 거부(② 문앞정렬)")
            _auto_abort_elev(); return
        _auto_set("② 문앞정렬", "문 앞으로 정렬 중(전진·회전)... 완료되면 '다음'",
                  wait=True, phase="front")
        _elev_scene(1)
        if not _auto_wait_confirm(): _auto_abort_elev(); return

        # ③ 문 열림 대기 (자동 감지)
        _auto_set("③ 문열림", "문 열림 대기 중... 문 열리면 '다음'", wait=True, phase="door")
        _elev_scene(2)
        if not _auto_wait_confirm(): _auto_abort_elev(); return

        # ④ 탑승: 전진 185 (자동 안무)
        if not arm_safe:
            _auto_notify("팔이 안전한지 확인되지 않아 이동을 멈췄습니다", stow_hint=True)
            _auto_set("오류", "팔 복귀 미확인 — 베이스 이동 거부(④ 탑승)")
            _auto_abort_elev(); return
        _auto_set("④ 탑승", "탑승(전진) 중... 다 탔으면 '다음'", wait=True, phase="ride")
        _elev_scene(3)
        if not _auto_wait_confirm(): _auto_abort_elev(); return

        # ⑤ 층 press: 인식자세 → 목적층 자동선택 → 정렬 → '다음'에 누르기
        _auto_set("⑤ 층선택", f"인식 자세 + {dest_floor}층 버튼 자동 선택·정렬 중...",
                  phase="floor")
        arm_safe = False                        # 씬0과 같은 이유(위 주석 참고)
        _elev_scene(4)                          # place=cab + 인식자세
        _elev_select(dest_floor)                # 층버튼 자동 선택
        if _elev_wait_ready():
            _auto_set("⑤ 층선택", f"정렬 완료 ✅ — '다음' 누르면 {dest_floor}층 누름", wait=True)
        else:
            _auto_set("⑤ 층선택", "⚠ 자동정렬 실패 — 엘베UI서 수동 정렬 후 '다음'", wait=True)
        if not _auto_wait_confirm(): _auto_abort_elev(); return
        if not _press_or_pass():                # 다음 → 층버튼 누르기
            _auto_abort_elev(); return
        _auto_set("⑤ 층선택", f"{dest_floor}층 버튼 누르는 중... (팔 복귀까지 대기)")
        if not _elev_wait_press_done():         # 눌림+팔복귀 완료까지 대기
            _auto_notify("누르기를 확인하지 못해 멈췄습니다", stow_hint=True)
            _auto_set("오류", "누르기/팔복귀 미완료(타임아웃/취소) — 여정 중단")
            _auto_abort_elev(); return
        arm_safe = True                         # 유일한 release 지점 (2/2)

        # 엘베 이동 대기 → ⑥ 하차: 후진 186 (자동 안무)
        _auto_set("이동중", f"{dest_floor}층 이동 중 — 도착·하차 준비되면 '다음'",
                  wait=True, phase="moving")
        if not _auto_wait_confirm(): _auto_abort_elev(); return
        if not arm_safe:
            _auto_notify("팔이 안전한지 확인되지 않아 이동을 멈췄습니다", stow_hint=True)
            _auto_set("오류", "팔 복귀 미확인 — 베이스 이동 거부(⑥ 하차)")
            _auto_abort_elev(); return
        _auto_set("⑥ 하차", "하차(후진) 중...", phase="exit")
        _elev_scene(5)
        # 상수 대기로 넘기면(#92) 후진 186cm가 10초 넘게 걸리는 동안 리스가 반납돼
        # 이동이 통째로 거부되고, 그 실패가 여기로 전파될 길이 없어 사람을 태운 채
        # 캐빈/문턱에서 /switch_map·/goto로 넘어간다. 씬 ①②③④와 ⑥직전이 전부
        # 확인 대기를 거치는데 이 한 자리만 상수였다.
        ex_reason, ex_cm = _elev_wait_exit_done()
        if ex_reason != "done":
            # 지도전환·AMCL 초기화·앱 종료·목적지 주행은 하나도 실행하지 않는다.
            _exit_failed(ex_reason, ex_cm)
            return

        # 리스 반납 — 지도전환·AMCL초기화·목적지주행은 가드(라이다 충돌가드)가
        # 켜진 상태로 시작해야 함. 앱 종료(8단계)까지 리스를 끌고 가지 않는다.
        if not _grant_elev_lease(False, "하차 완료"):
            _auto_abort_elev()   # 순서 중요: abort가 먼저(문구 "취소" 세팅) →
                                  # 아래 _auto_set("오류",...)로 덮어써야 UI에 사고원인이 남음
            _auto_set("오류", "🚨 엘베 가드 미복원 — 여정 중단(앱 강제종료)")
            return

        # 7) 지도 전환 + 하차지점 초기화 (자동)
        _auto_set("지도전환", f"{dest_floor}층 지도 전환 + 하차지점 초기화", phase="exit")
        _http_self("/switch_map", {"floor": dest_floor, "init_exit": True})
        time.sleep(2)

        # 8) 엘베앱 OFF (제어권 회수 + 종료)
        _auto_set("엘베종료", "엘리베이터 앱 종료 + 제어권 회수", phase="exit")
        _http_self("/elevator_app", {"running": False})
        time.sleep(1)

        # 9) 목적지 주행 (자동)
        _auto_set("주행", f"{dest}로 이동 중...", phase="drive")
        _write("iface", f"/goto {dest}")
        _auto_set("완료", f"🎉 {dest} 도착! 여정 완료" if _auto_wait_arrival(dest)
                  else "목적지 도착 실패", phase="done")
    except Exception as e:
        _auto_set("오류", f"여정 예외: {e!r}")
    finally:
        # 최종 보루 — 위 정상 반납(하차 직후)을 못 탄 모든 이탈 경로(예외·취소·
        # 좌표없음 등)에서 리스가 켜진 채 방치되면 가드가 계속 꺼져 있다.
        try:
            if _rescue_hold:
                # ⑥ 하차 미완료 — 반납하면 엘베앱 패드가 첫 줄에서 거부돼 문턱에 낀
                # 로봇을 빼낼 수단이 사라진다. 해제는 /elevator_app {running:false}
                # 한 곳에서만 (거기서 리스도 물리적으로 소멸).
                _log("ELEVLEASE", "구조 대기 — 제어권 유지(하차 미완료). "
                                  "엘베앱 종료 시 해제됨")
            elif _elev_lease_held and not _grant_elev_lease(False, "여정 종료(finally)"):
                _log("ELEVLEASE", "🚨 최후보루 반납 실패")
        except Exception:
            pass
        with _auto_lock:
            _AUTO["active"] = False; _AUTO["waiting"] = False

@app.route("/auto_goto", methods=["POST"])
def auto_goto():
    if _AUTO["active"]:
        return jsonify(ok=False, error="이미 여정 진행 중")
    dest = ((request.json or {}).get("dest") or "").strip()
    if not dest:
        return jsonify(ok=False, error="목적지 없음"), 400
    if _manual_mode:
        return jsonify(ok=False, error="수동 모드 — 자동 모드로 전환하세요")
    # 사진모드 엘베앱으로 여정을 돌리면 detections가 영원히 비어 _elev_wait_ready가
    # 45s 타임아웃 → /press 거부 → 재시도도 거부 → 여정 중단이다. fail-closed라
    # 물리 사고는 없지만 arm_safe가 False로 남아 이후 베이스 이동이 전부 거부된다 —
    # 시각장애인이 팔 나온 채 홀에 발이 묶인다. 시작 전에 막는다.
    if _elev_app_running():
        # 한 번 못 읽었다고 단정하지 않는다. 엘베앱은 start_infer_server()의 모델
        # 로딩(블로킹, 수초)을 마친 뒤에야 :5000을 여는데, 그 창이 최대 20초다
        # (_wait_elev_app_up의 timeout=20이 그 기대치를 말해준다). 그 사이 /status는
        # 계속 None이라, 즉시 거부하면 아무 이상 없는 정상 기동을 잘라버린다.
        # (사진모드는 모델 로딩을 건너뛰어 빠르므로 이 창에 걸리는 건 정상 모드다.)
        _st = None
        for _i in range(3):
            _st = _elev_status(timeout=1.0)
            if _st is not None:
                break
            if _i < 2:          # 마지막 회차 뒤엔 잘 이유가 없다(순수 낭비 0.4s)
                time.sleep(0.4)
        if _st is None:
            # 모르면 안 움직인다. 앱이 떠 있는데 상태를 못 읽으면 모드도 모르는
            # 것이고, 그 상태로 여정이 성공할 시나리오가 없다 — 하류가 어차피
            # 실패하는데 그 실패는 팔이 나온 채 홀에 발이 묶이는 형태다.
            # 사유는 사진모드와 구분해서 낸다(운영자가 원인을 알아야 한다).
            # 재시도 1.2초로는 20초 로딩 창을 못 덮으므로, 문구가 그 가능성을
            # 말해야 한다 — "엘베앱을 확인하세요"는 할 일을 틀리게 지시한다.
            return jsonify(ok=False, error="엘베앱 상태를 읽을 수 없어 여정을 시작하지 "
                                           "않습니다 — 기동 중일 수 있습니다(OCR 모델 "
                                           "로딩, 최대 20초). 잠시 후 다시 시도하세요"), 409
        if _st.get("no_ocr"):
            return jsonify(ok=False, error="사진모드에서는 여정을 시작할 수 없습니다 "
                                           "— 엘베앱을 정상 모드로 다시 켜세요"), 409
    threading.Thread(target=_auto_run, args=(dest,), daemon=True).start()
    return jsonify(ok=True, dest=dest)

@app.route("/auto_confirm", methods=["POST"])
def auto_confirm():
    """블로커 단계 '다음 확인' — 대기 해제."""
    with _auto_lock:
        _AUTO["waiting"] = False
    return jsonify(ok=True)

@app.route("/auto_cancel", methods=["POST"])
def auto_cancel():
    with _auto_lock:
        _AUTO["cancel"] = True; _AUTO["waiting"] = False
    _write("iface", "/cancel")
    return jsonify(ok=True)

@app.route("/auto_status")
def auto_status():
    with _auto_lock:
        return jsonify(**{k: _AUTO[k] for k in
                          ("active", "dest", "step", "msg", "waiting",
                           "phase", "dest_floor", "mode")})

_CONFIRMED_LOCATIONS = {
    "인공지능 플랫폼",
    "특별전시관",
    "ITRC 120 상담장",
    "ITRC 219 문화행사",
    "ITRC 217 지능통감융합 연구센터 (KAIST)",
    "ITRC 215 배리어프리 ICT기술 연구센터 (단국대)",
    "ITRC 114 UAM-eVTOL 융합 연구센터 (세종대)",
}

@app.route("/goto", methods=["POST"])
def goto():
    """장소 목록 클릭 → 즉시 출발 (2026-07-24 사용자 결정: 확인 절차 생략).
    interface의 /goto 직행 통로 사용 — GPT·별칭 해석·마이크 전부 건너뜀.
    (음성 흐름의 확인 절차는 별개로 유지 — 이건 클릭 전용 경로)"""
    name = ((request.json or {}).get("name") or "").strip()
    if not name:
        return jsonify(ok=False, error="이름 없음"), 400
    if _manual_mode:
        return jsonify(ok=False, error="수동 모드에서는 불가 — 자동 모드로 전환하세요")
    _write("iface", f"/goto {name}")
    _log("MAIN", f"🖱 장소 클릭 → '{name}' 즉시 출발")
    return jsonify(ok=True)

@app.route("/locations")
def get_locations():
    import yaml
    yaml_path = THIS_DIR / "../config/location.yaml"
    try:
        data = yaml.safe_load(yaml_path.read_text("utf-8")) or {}
        locs = data.get("locations") or {}
    except Exception:
        locs = {}
    return jsonify(locations=[
        {"name": n, "confirmed": n in _CONFIRMED_LOCATIONS,
         "floor": (v or {}).get("floor")}   # 층 메타데이터 (없으면 null = 공용)
        for n, v in locs.items()
    ])

# ── 시스템 프로세스 관리 ──────────────────────────────────────────────────────────
_sys_procs: dict = {}

_ROS_SOURCE = (
    "source /opt/ros/humble/setup.bash && "
    "source /home/hello-robot/ament_ws/install/setup.bash 2>/dev/null; "
)

_SYS_PROC_DEFS = {
    "launch": {
        "label": "ROS2 Launch",
        # exec: bash가 ros2 launch 프로세스로 자체 교체됨
        # → proc.pid = ros2 launch 자신 → proc.wait()가 모든 노드 종료 후까지 대기
        # exec 없이는 bash만 대기하고 ros2 launch + rplidar 등이 orphan으로 남아 모터가 계속 돔
        "cmd": ["bash", "-c",
                _ROS_SOURCE +
                "exec ros2 launch /home/hello-robot/GitHub/visually-impaired-navigation-robot/"
                "src/blind_nav_system/launch/stretch_robot_process.launch.xml"],
        "log_tag": "ROS2",
        "kill_timeout": 20,    # ros2 launch의 모든 노드 graceful shutdown 완료 대기
        "auto_free_lock": True, # 만약 SIGKILL로 강제 종료됐을 때 Stretch filelock 안전망
    },
    "rviz": {
        "label": "RViz2",
        # 사전 구성 설정(-d)으로 실행: Map(Transient Local)·RobotModel·LaserScan·
        # 2D Pose Estimate 도구 포함 — 맨 RViz의 "지도 안 뜸" 수동 설정 반복 제거.
        # 설정 파일이 없으면 맨 RViz로 폴백.
        "cmd": ["bash", "-c", _ROS_SOURCE +
                "if [ -f /home/hello-robot/robot_view.rviz ]; then "
                "exec ros2 run rviz2 rviz2 -- -d /home/hello-robot/robot_view.rviz; "
                "else exec ros2 run rviz2 rviz2; fi"],
        "log_tag": "RVIZ",
    },
    "battery": {
        "label": "배터리 확인",
        "cmd": ["bash", "-c", "stretch_robot_battery_check.py"],
        "log_tag": "BATT",
    },
    "free": {
        "label": "프로세스 정리",
        "cmd": ["bash", "-c", "stretch_free_robot_process.py"],
        "log_tag": "FREE",
    },
    "home": {
        "label": "홈 위치",
        "cmd": ["bash", "-c", "stretch_robot_home.py"],
        "log_tag": "HOME",
    },
}


def _proc_zombie(pid) -> bool:
    """좀비(Z)인가. os.kill(pid, 0)은 좀비에도 성공하므로 그것만으로는 못 가른다.
    comm에 공백·괄호가 들어갈 수 있어 마지막 ')' 뒤부터 파싱한다(상태는 그 첫 필드)."""
    try:
        with open(f"/proc/{pid}/stat", "rb") as f:
            tail = f.read().decode("utf-8", "replace").rsplit(")", 1)[-1].split()
        return bool(tail) and tail[0] == "Z"
    except Exception:
        return False


def _proc_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except (ProcessLookupError, OSError):
        return False
    return not _proc_zombie(pid)


# 외부(= 이 대시보드가 띄우지 않은) 프로세스 탐지. name → pid.
# _sys_procs와 섞지 않는다 — 그건 Popen 핸들 dict이고 poll()에 의미가 있다.
# 가짜 핸들을 섞으면 대시보드가 자기 상태에 대해 거짓말하는 구조가 하나 더 생긴다.
# (나중에 종료를 지원하게 되면 여기서 PID를 꺼내 쓰면 된다.)
_sys_ext: dict = {}

_LAUNCH_XML = ("/home/hello-robot/GitHub/visually-impaired-navigation-robot/"
               "src/blind_nav_system/launch/stretch_robot_process.launch.xml")

# 탐지 대상은 launch·rviz 둘뿐이다. battery·free·home은 몇 초짜리 단발 스크립트라
# "실행 중(외부)" 깜빡임만 만들고, 이중 기동 방지 대상도 아니다.
# 표시(_SHOW)와 시작 게이트(_GATE)를 비대칭으로 둔다 — rviz 참고.
_SYS_EXT_SHOW = {
    "launch": (_LAUNCH_XML,),
    # 표시는 자식 바이너리만 본다. 래퍼(ros2 run)만 보고 "실행 중"이라고 하면
    # 렌더러가 죽어도 켜졌다고 말하게 된다 — 사용자가 보는 건 창이지 래퍼가 아니다.
    "rviz":   ("/opt/ros/humble/lib/rviz2/rviz2",),
}
_SYS_EXT_GATE = {
    "launch": (_LAUNCH_XML,),
    # 시작 거부는 래퍼든 바이너리든 하나라도 있으면 한다(fail-closed). 이 비대칭이
    # "렌더러만 죽어 꺼짐으로 보이는데 켜기를 눌러 래퍼 고아가 남는" 경로를 막는다.
    "rviz":   ("/opt/ros/humble/lib/rviz2/rviz2", "ros2 run rviz2", "rviz2 rviz2"),
}


def _scan_sys_ext():
    """/proc를 직접 읽어 외부 launch·rviz를 찾는다. (표시용, 게이트용) 둘 다 name→pid.

    pgrep을 쓰지 않는다: 이 저장소는 pgrep -f가 자기 명령줄을 매치해 오판한 사고를
    겪었다. /proc 순회는 os.getpid()만 건너뛰면 그 오판이 원천적으로 불가능하고
    (패턴을 담은 명령줄 자체가 존재하지 않는다), 2초 폴링마다 fork하지도 않으며,
    PID·cmdline·PGID를 한 번에 얻는다.

    이 대시보드가 띄운 것은 PGID로 걸러낸다 — Popen이 start_new_session=True라
    자식 세션의 PGID가 곧 그 Popen의 pid다. rviz처럼 프로세스가 둘인 경우도
    부모·자식이 같은 PGID라 한 번에 빠진다.

    순회 자체가 실패하면 (None, None)을 돌려준다 — 호출부가 '모름'을 알아야
    시작을 막을 수 있다. 빈 dict(=아무것도 없음)와 구분되어야 한다."""
    me = os.getpid()
    mine = set()
    for pr in list(_sys_procs.values()):
        try:
            if pr.poll() is None:
                mine.add(pr.pid)          # start_new_session=True → pgid == pid
        except Exception:
            pass
    try:
        entries = [d for d in os.listdir("/proc") if d.isdigit()]
    except Exception as e:
        _log("SYS", f"/proc 순회 실패 — 외부 프로세스 판정 불가: {e}")
        return None, None
    show, gate = {}, {}
    for d in entries:
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                argv = [a for a in f.read().decode("utf-8", "replace").split("\0") if a]
        except Exception:
            continue          # 순회 도중 죽은 프로세스 — 정상이다
        if not argv:
            continue          # 커널 스레드
        # 명령줄에 문자열이 '있는' 것과 그 프로세스'인' 것은 다르다. 실측에서
        # 패턴을 인자로 담고 있던 남의 셸이 그대로 잡혔다 — pgrep -f를 못 쓰게 만든
        # 그 함정이 /proc 순회에도 똑같이 있다(다른 세션이 이 문자열을 grep하기만
        # 해도 걸린다). 그래서 argv[0]/argv[1]로 정체를 먼저 가른다.
        is_rviz_bin = argv[0] == "/opt/ros/humble/lib/rviz2/rviz2"
        is_ros2 = any(len(argv) > i + 1 and os.path.basename(argv[i]) == "ros2"
                      and argv[i + 1] in ("launch", "run") for i in (0, 1))
        if not (is_rviz_bin or is_ros2):
            continue
        cl = " ".join(argv)
        hit_show = [n for n, pats in _SYS_EXT_SHOW.items() if any(x in cl for x in pats)]
        hit_gate = [n for n, pats in _SYS_EXT_GATE.items() if any(x in cl for x in pats)]
        if not hit_show and not hit_gate:
            continue          # 흔한 경우 — 여기서 끝내 open을 한 번만 한다
        pid = int(d)
        if pid == me or _proc_zombie(pid):
            continue
        try:
            if os.getpgid(pid) in mine:
                continue      # 이 대시보드가 띄운 것 — 외부가 아니다
        except Exception:
            pass
        for n in hit_show:
            show.setdefault(n, pid)
        for n in hit_gate:
            gate.setdefault(n, pid)
    return show, gate


def _sys_start_gate(name):
    """시스템 프로세스 시작 관문. 막을 이유가 있으면 (사유, HTTP코드), 없으면 None.
    _manual_arm_gate와 같은 형식이다 — 조용히 안 되는 조작기를 하나 더 만들지 않는다.

    이중 기동은 무해한 실수가 아니다: stretch_driver 2개(바디 락 충돌) +
    rplidar 2개(같은 시리얼) + amcl/map_server 2개(TF 파손)다."""
    global _sys_ext
    pr = _sys_procs.get(name)
    if pr and pr.poll() is None:
        return f"이미 실행 중입니다 (PID {pr.pid})", 409
    if name not in _SYS_EXT_GATE:
        return None            # 단발 스크립트는 이중 기동 대상이 아니다
    show, gate = _scan_sys_ext()
    if gate is None:
        # 모르면 막는다. 비대칭이 근거다 — 거짓 "안 돌고 있음"의 대가는 이중 기동
        # (드라이버·라이다 중복, TF 파손)이고, 거짓 "돌고 있음"의 대가는 사용자가
        # 한 번 확인하는 것뿐이다.
        return "실행 여부를 확인하지 못해 시작하지 않습니다 — 잠시 후 다시 시도하세요", 409
    _sys_ext = show
    pid = gate.get(name)
    if pid:
        return (f"실행 중(외부) — PID {pid}, 이 대시보드가 띄운 게 아닙니다. "
                "끄려면 터미널에서."), 409
    return None


def _stop_lidar_motor():
    """rplidar_composition 종료 후 시리얼로 직접 모터 정지."""
    import time as _t
    try:
        import serial
        with serial.Serial('/dev/hello-lrf', 115200, timeout=1) as ser:
            ser.write(b'\xa5\x25')   # RPLIDAR_CMD_STOP (스캔 정지)
            _t.sleep(0.05)
            ser.rts = False          # RTS 내림 → RPLiDAR 모터 PWM 정지
        _log("SYS", "LiDAR 모터 정지 완료 (시리얼 RTS)")
    except ImportError:
        _log("SYS", "LiDAR 시리얼 정지 실패: pyserial 없음 (pip install pyserial)")
    except Exception as e:
        _log("SYS", f"LiDAR 시리얼 정지 실패: {e}")


def _kill_proc_group(proc: subprocess.Popen, label: str, kill_timeout: int = 6) -> bool:
    """
    프로세스 그룹 전체 종료. 정상 종료 성공 여부 반환.
    exec ros2 launch 패턴에서 proc.pid = ros2 launch 자신이므로
    proc.wait()가 모든 자식 노드 종료 후까지 대기함.
    SIGINT를 쓰는 이유(SIGTERM 금지):
    - ros2 launch는 SIGTERM을 받으면 자식 노드를 정리하지 않고 즉시 종료 (고아 발생)
    - rplidar_composition은 SIGINT에만 모터 정지 경로가 연결됨 —
      SIGTERM이면 rclcpp 컨텍스트 shutdown 후 publish 예외로 SIGABRT 크래시,
      모터 정지 명령(setMotorSpeed(0))이 시리얼로 안 나가서 LiDAR가 계속 돎
    """
    if proc is None or proc.poll() is not None:
        return True
    try:
        pgid = os.getpgid(proc.pid)
        # exec 여부 확인: proc.pid 프로세스 이름 로그
        try:
            with open(f"/proc/{proc.pid}/comm") as f:
                pname = f.read().strip()
        except Exception:
            pname = "?"
        _log("SYS", f"{label} SIGINT → PID={proc.pid}({pname}) PGID={pgid} (최대 {kill_timeout}s 대기)")
        os.killpg(pgid, signal.SIGINT)
        try:
            import time as _time
            t0 = _time.monotonic()
            proc.wait(timeout=kill_timeout)
            elapsed = _time.monotonic() - t0
            _log("SYS", f"{label} 정상 종료 ({elapsed:.1f}s)")
            return True
        except subprocess.TimeoutExpired:
            _log("SYS", f"{label} {kill_timeout}s 초과 → SIGKILL 강제 종료")
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=3)
            return False
    except (ProcessLookupError, OSError):
        try:
            proc.kill()
        except Exception:
            pass
        return True


@app.route("/sys_proc/<name>", methods=["POST"])
def sys_proc_ctrl(name):
    if name not in _SYS_PROC_DEFS:
        return jsonify(ok=False, error="unknown"), 400
    defn    = _SYS_PROC_DEFS[name]
    action  = (request.json or {}).get("action", "toggle")
    p       = _sys_procs.get(name)
    running = bool(p and p.poll() is None)

    if action == "stop" or (action == "toggle" and running):
        _sys_procs.pop(name, None)
        kill_timeout = defn.get("kill_timeout", 6)
        auto_free    = defn.get("auto_free_lock", False)

        def _do_kill():
            import time as _t
            graceful = _kill_proc_group(p, defn["label"], kill_timeout=kill_timeout)
            _log("SYS", f"{defn['label']} 종료 완료 ({'graceful' if graceful else 'SIGKILL'})")

            # rplidar_composition 처리: ros2 launch가 0s 만에 종료해도 rplidar는 고아로 남을 수 있음
            # → 소멸자(destructor)가 모터를 멈출 시간을 주고, 안 되면 강제 종료 + 시리얼 정지
            r = subprocess.run(["pgrep", "-a", "rplidar"], capture_output=True, text=True)
            rplidar_pids = []
            for line in r.stdout.strip().splitlines():
                try:
                    rplidar_pids.append(int(line.split()[0]))
                except (ValueError, IndexError):
                    pass

            if rplidar_pids:
                _log("SYS", f"rplidar 고아 프로세스 감지 {rplidar_pids} → 종료 대기 (최대 5s)")
                deadline = _t.monotonic() + 5.0
                while _t.monotonic() < deadline:
                    _t.sleep(0.3)
                    rplidar_pids = [pid for pid in rplidar_pids
                                    if _proc_alive(pid)]
                    if not rplidar_pids:
                        break

                if rplidar_pids:
                    _log("SYS", "rplidar 종료 안 됨 → SIGKILL")
                    for pid in rplidar_pids:
                        try:
                            os.kill(pid, signal.SIGKILL)
                        except Exception:
                            pass
                # 고아로 남았다는 것 자체가 비정상 경로 — 프로세스가 빨리 죽었어도
                # SIGABRT 크래시 등으로 모터 정지 명령이 안 나갔을 수 있음 (실측: Signal 6 crash 기록).
                # "빨리 죽음 = 소멸자 실행 = 모터 정지"로 추론하지 말고 무조건 시리얼 정지.
                _t.sleep(0.4)  # 포트 해제 대기
                _stop_lidar_motor()
            elif graceful:
                _log("SYS", "rplidar 프로세스 없음 (launch가 정상 정리)")
            else:
                # 그룹째 SIGKILL된 경우 rplidar도 모터 정지 없이 죽었을 수 있음
                _t.sleep(0.4)
                _stop_lidar_motor()

            if auto_free:
                try:
                    subprocess.run(["stretch_free_robot_process.py"], timeout=5,
                                   capture_output=True)
                    _log("SYS", "Stretch filelock 해제 완료")
                except Exception:
                    pass

        threading.Thread(target=_do_kill, daemon=True).start()
        return jsonify(ok=True, running=False)

    # [FastDDS 찌꺼기 청소는 여기서 하면 안 됨 — 부팅 시(main, 노드 생성 전)로 이동]
    # 여기서 청소하면 이미 살아있는 대시보드/interface 자신의 노드 세그먼트를 지워
    # DDS 유령(발행·수신 전부 침묵)을 만든다. "대시보드 자신의 세마포어는 unlink돼도
    # 계속 유효"라는 기존 가정은 틀림 — unlink된 세그먼트 파일은 이후 시작한
    # 프로세스가 열 수 없어, 옛/새 프로세스가 서로 다른 섬으로 갈라진다
    # (2026-07-21 실측: 수동 전진 버튼 무반응 + 새 CLI에서 드라이버 노드 실종 사건)

    blocked = _sys_start_gate(name)
    if blocked:
        _log("SYS", f"{defn['label']} 시작 거부 — {blocked[0]}")
        return jsonify(ok=False, error=blocked[0]), blocked[1]

    try:
        proc = subprocess.Popen(
            defn["cmd"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
            start_new_session=True,   # 자체 프로세스 그룹 → killpg로 자식까지 종료 가능
        )
        _sys_procs[name] = proc
        threading.Thread(target=_capture, args=(proc, defn["log_tag"]), daemon=True).start()
        _log("SYS", f"{defn['label']} 시작 PID={proc.pid}")
        return jsonify(ok=True, running=True)
    except Exception as e:
        _log("SYS", f"{defn['label']} 시작 실패: {e}")
        return jsonify(ok=False, error=str(e)), 500

@app.route("/sys_proc_status")
def sys_proc_status_route():
    """이름 → {state, running, pid}. state = mine(내가 띄움) / external(외부 실행 중)
    / unknown(판정 실패) / off. '외부 실행 중'은 오류가 아니라 소유권 정보다 —
    준비바에서 빨갛게 만들 것이 아니라 '내가 끌 수 없다'는 사실을 알리는 것이다."""
    global _sys_ext
    show, _gate = _scan_sys_ext()
    if show is not None:
        _sys_ext = show
    out = {}
    for name in _SYS_PROC_DEFS:
        pr = _sys_procs.get(name)
        if pr and pr.poll() is None:
            out[name] = {"state": "mine", "running": True, "pid": pr.pid}
        elif show is not None and show.get(name):
            out[name] = {"state": "external", "running": True, "pid": show[name]}
        elif show is None and name in _SYS_EXT_GATE:
            out[name] = {"state": "unknown", "running": False, "pid": None}
        else:
            out[name] = {"state": "off", "running": False, "pid": None}
    return jsonify(out)

# 시각장애인 안내 로봇 관련 알려진 프로세스 이름 (정확히 일치)
_ROBOT_PROC_NAMES = [
    "rplidar_composition",
    "realsense2_camera_node", "realsense2_camera",
    "robot_state_publisher",
    "amcl", "map_server",
    "lifecycle_manager", "nav2_lifecycle_manager",
    "controller_server", "planner_server",
    "behavior_server", "bt_navigator",
    "waypoint_follower", "velocity_smoother",
    "collision_monitor", "costmap_2d",
    "rviz2",
    "component_container",
    "static_transform_publisher",
]

_killall_running = False

@app.route("/killall_robot", methods=["POST"])
def killall_robot():
    global _killall_running
    if _killall_running:
        return jsonify(ok=False, error="이미 실행 중")
    _killall_running = True

    def _do():
        global _killall_running
        import time as _t
        try:
            _log("SYS", "=== 로봇 프로세스 전체 정리 시작 ===")

            # 1. 대시보드가 추적 중인 프로세스 먼저 종료
            for n, proc in list(_sys_procs.items()):
                if proc and proc.poll() is None:
                    defn = _SYS_PROC_DEFS.get(n, {})
                    _kill_proc_group(proc, defn.get("label", n))
            _sys_procs.clear()

            # 2. 이름으로 시스템 전체 검색 → SIGINT (Ctrl+C 동일 — ROS 노드 정상 종료 경로)
            # pkill -x는 comm(15자 잘림)과 비교하므로 이름을 15자로 잘라야 매치됨
            # (예: rplidar_composition의 comm은 "rplidar_composi")
            killed = []
            for pname in _ROBOT_PROC_NAMES:
                r = subprocess.run(["pkill", "-INT", "-x", pname[:15]], capture_output=True)
                if r.returncode == 0:
                    killed.append(pname)
            if killed:
                _log("SYS", f"SIGINT: {', '.join(killed)}")
            else:
                _log("SYS", "실행 중인 로봇 프로세스 없음")

            # 3. 3초 대기 (graceful shutdown)
            _t.sleep(3)

            # 4. 살아남은 것 SIGKILL (comm 15자 잘림 주의)
            force_killed = []
            for pname in _ROBOT_PROC_NAMES:
                r = subprocess.run(["pkill", "-KILL", "-x", pname[:15]], capture_output=True)
                if r.returncode == 0:
                    force_killed.append(pname)
            if force_killed:
                _log("SYS", f"SIGKILL: {', '.join(force_killed)}")

            # 5. rplidar 확인 후 시리얼 모터 정지
            _t.sleep(0.4)
            r = subprocess.run(["pgrep", "-x", "rplidar_composition"[:15]], capture_output=True)
            if r.returncode != 0:  # 종료됨 → 포트 사용 가능
                _stop_lidar_motor()

            # 6. Stretch filelock 해제
            try:
                subprocess.run(["stretch_free_robot_process.py"], timeout=5, capture_output=True)
                _log("SYS", "Stretch filelock 해제 완료")
            except Exception:
                pass

            _log("SYS", "=== 전체 정리 완료 ===")
        finally:
            _killall_running = False

    threading.Thread(target=_do, daemon=True).start()
    return jsonify(ok=True)

# ── 시리얼 루프 ───────────────────────────────────────────────────────────────
def serial_loop():
    if not _SERIAL_OK:
        _log("ARD", "시리얼 비활성화 (pyserial 없음)")
        return

    pull_detect = _make_pull_detector()
    last_pull_t = 0.0
    ser = None

    while ser is None:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
            _log("ARD", f"시리얼 연결됨: {SERIAL_PORT}")
        except Exception as e:
            _log("ARD", f"시리얼 연결 실패({e}), 3초 후 재시도...")
            time.sleep(3)

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            parts = text.split(",")
            if len(parts) < 3:
                continue

            tag = parts[0].strip()
            now = time.monotonic()
            _ready["handle"] = now

            if tag == "TRIG" and len(parts) >= 3:
                btn = parts[1].strip()
                if btn == "1":
                    _write("iface", "/button")
                    _log("HW", "버튼1")
                elif btn == "2":
                    _write("vision", "/vision")
                    _log("HW", "버튼2 → 시각 분석")
                # 버튼 누르는 순간 압력이 PULL_TRIG를 넘으므로 2초 잠금
                last_pull_t = now + 2.0

            elif tag == "DATA" and len(parts) >= 4:
                try:
                    pressure = int(parts[3].strip())
                except ValueError:
                    continue
                if pull_detect(pressure) and (now - last_pull_t) > _DEBOUNCE:
                    last_pull_t = now
                    _write("iface", "/pull")
                    _log("HW", "당김 감지")
    except Exception as e:
        _log("ARD", f"시리얼 오류: {e}")
    finally:
        if ser:
            ser.close()

# ── 진입점 ────────────────────────────────────────────────────────────────────
def _clean_stale_shm_at_boot():
    """FastDDS 잔재 청소 — 반드시 '자기 ROS 노드를 만들기 전'에만 호출할 것.
    ROS 프로세스가 하나도 없을 때만 안전 (잔재가 전부 주인 없는 상태).
    비정상 종료 후 잔재는 "기존 노드는 멀쩡한데 새 프로그램만 통신 실패"를 만들고
    (2026-07-15 실측), 노드 생성 후의 청소는 자기 자신을 유령으로 만든다
    (2026-07-21 실측) — 그래서 위치가 '부팅 직후, init_ros() 이전' 딱 한 곳이다."""
    try:
        # 주의: ROS 노드를 가진 "우리 앱들"도 반드시 포함 — 빠지면 그 앱이
        # 살아있는 채로 청소가 돌아 그 앱을 유령으로 만든다 (섬 분리 사고)
        r = subprocess.run(["pgrep", "-f",
                            "ros2 launch|stretch_driver|rplidar|realsense2|"
                            "component_container|nav2|amcl|map_server|"
                            "elevator_button_press|interface.py|vision_assistant.py|"
                            "rviz2|mouse_teleop|obstacle|people_tracker"],
                           capture_output=True, text=True)
        _alive = r.stdout.strip()
        if _diaglog:
            _n = len(_alive.splitlines()) if _alive else 0
            _diaglog.log("SHM", f"부팅 청소 판단: 살아있는 ROS 프로세스 {_n}개 → "
                                f"{'건너뜀(기존 프로세스 보호)' if _alive else '청소 진행'}")
        if not _alive:
            import glob as _glob
            stale = _glob.glob("/dev/shm/fastrtps_*") + \
                    _glob.glob("/dev/shm/fast_datasharing*") + \
                    _glob.glob("/dev/shm/sem.fastrtps*")
            for f in stale:
                try:
                    os.remove(f)
                except OSError:
                    pass
            if stale:
                _log("SYS", f"FastDDS 잔재 {len(stale)}개 청소 (부팅 시, 노드 생성 전)")
    except Exception:
        pass   # 청소 실패가 대시보드 시작을 막으면 안 됨


def main():
    global _diaglog
    if _diag is not None:
        try:
            _diaglog = _diag.DiagLogger("system")
            _diaglog.boot_snapshot()
        except Exception as _le:
            _diaglog = None
            _log("SYS", f"robot_diag 로거 생성 실패 — 파일 로그 비활성: {_le}")
        else:
            if not getattr(_diaglog, "ok", True):
                _log("SYS", "파일 로그 열기 실패 — 계측 비활성")
    else:
        _log("SYS", "robot_diag 임포트 실패 — 파일 로그 비활성")   # 계측이 조용히 죽는 것 방지(#15)
    if not (WEB_DIR / "index.html").is_file():
        # 없으면 '/'가 404로 조용히 죽는다 — 로그로 드러내야 원인을 안다.
        _log("SYS", f"index.html 없음 — 대시보드 UI 비활성: {WEB_DIR / 'index.html'}")
    Path("/tmp/social_nav_enabled").write_text("1" if _social_nav_enabled else "0")
    Path("/tmp/obstacle_push_enabled").write_text("1" if _obstacle_push_enabled else "0")
    _clean_stale_shm_at_boot()   # ← init_ros()보다 반드시 먼저
    init_ros()
    start_subprocesses()

    threading.Thread(target=serial_loop, daemon=True).start()
    threading.Thread(target=_manual_loop, daemon=True).start()

    threading.Timer(1.5, lambda: webbrowser.open("http://localhost:8080")).start()
    _log("MAIN", "대시보드: http://localhost:8080")

    # 종료 신호 로깅 (원래 동작은 그대로, 로그만 추가) — app.run 직전에 설치
    if _diag is not None and _diaglog is not None:
        _diag.install_signal_logging(_diaglog, reraise=False)

    app.run(host="0.0.0.0", port=8080, threaded=True)


if __name__ == "__main__":
    main()
