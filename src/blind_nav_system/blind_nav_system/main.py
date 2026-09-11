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
# ── 여정 블랙박스 — 같은 폴더의 journey_log.py (없어도 본체는 정상 동작) ──
try:
    import journey_log as _jlog
except Exception as _e:
    _jlog = None
    print(f"[경고] journey_log 로드 실패: {_e}")
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
_SERIAL_BACKOFF_MAX = 60.0    # 재시도 간격 상한(초) — 로그 홍수 방지용 백오프
_SERIAL_SUM_SEC     = 600.0   # 같은 실패가 계속될 때 요약 1줄을 남기는 주기(초)

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


# ── 여정 블랙박스(journey_log.py) 관문 — 2026-09-11 ───────────────────────────────
# 자동 여정 한 번을 ~/.ros/journey/<run>.jsonl 로 남긴다. 목적·제약은 journey_log.py 머리말.
# 훅 규칙 세 가지:
#   · 훅은 `_jr(...)` 로만 부른다 — 레코더가 없거나 무엇이 터져도 여기서 삼킨다
#     (훅 예외 = 여정 사망).
#   · 훅의 **인자 식**은 `_jr` 의 try 밖에서 평가된다. 인자에는 그 지점에서 반드시 정의된
#     이름과 예외를 못 내는 식만 둔다. dict 에서 꺼내야 하면 `_jr_pick` 을 쓴다.
#   · 느린 것(파일·/proc·pgrep·yaml)은 레코더 deferred 로 writer 스레드에 넘긴다.
_DASH_SRC = None
_JR = None
if _jlog is not None:
    try:
        _DASH_SRC = _jlog.src_fingerprint(__file__)   # import 시점 = 이 프로세스가 로드한 코드
        _JR = _jlog.JourneyRecorder(log_fn=lambda m: _log("JOURNEY", m),
                                    src_id=_DASH_SRC, repo_path=str(THIS_DIR))
    except Exception as _e:
        _JR = None
        print(f"[경고] 여정 블랙박스 생성 실패: {_e}")


def _jr(method, *args, **kwargs):
    """블랙박스 훅 단일 관문. 레코더 메서드의 반환(없거나 실패하면 None)."""
    try:
        if _JR is not None:
            return getattr(_JR, method)(*args, **kwargs)
    except Exception:
        pass
    return None


def _jr_traced(name, post=None, ev="call"):
    """호출 기록 데코레이터 — 인자·반환·지연만 적고 그대로 통과시킨다. 레코더가 없으면 원 함수."""
    if _JR is None:
        return lambda fn: fn
    try:
        return _JR.traced(name, post=post, ev=ev)
    except Exception:
        return lambda fn: fn


def _jr_caller(depth=2):
    """부른 함수 이름(분석용 태그). 실패하면 None."""
    try:
        return sys._getframe(depth).f_code.co_name
    except Exception:
        return None


def _jr_pick(d, keys):
    try:
        return {k: d.get(k) for k in keys} if isinstance(d, dict) else None
    except Exception:
        return None


def _jr_cancel_post(_ret):
    """대기 함수의 False 가 '취소'인지 '시간초과'인지 — 반환 직후의 cancel 플래그."""
    return {"cancel_flag": bool(_AUTO.get("cancel"))}


def _jr_map_post(_ret):
    """_map_loaded_floor 의 입력 R(우리 기록). P 원문은 바로 앞 map_server_yaml 게이트에 있다."""
    return {"R_path": _loaded_map_path}

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
# [지도 교체 규약] 새 지도가 나오면 이 참조들을 고치지 말고, all.pgm 자체를
# 갈아끼우고 옛것을 all_backup_YYYYMMDD.pgm으로 남긴다. 참조가 5곳(all.yaml·
# floor1~4.yaml)이라 한 곳만 고치면 조용히 어긋난다 — 파일을 바꾸면 고칠 참조가
# 0개가 되어 그 위험이 아예 없어진다. (2026-09-02 캐빈 병합을 그렇게 처리했다.)
#
# [동기화 경고] 런치(stretch_robot_process.launch.xml)의 map 인자와 아래
# _FLOOR_MAPS는 반드시 같은 지도를 가리켜야 한다. 어긋나면 증상이 "지도가 안
# 바뀐다"가 아니라 "UI는 5층이라는데 그 지도에 캐빈이 없다"로 나와 원인을 찾기
# 어렵다. 지금은 양쪽 다 all.yaml을 보므로 일치한다 — 나중에 층별로 지도를 다시
# 갈라놓게 되면 이 일치를 직접 지켜야 한다.
_FLOOR_MAPS = {
    "5": str((THIS_DIR / "../maps/all.yaml").resolve()),
    "4": str((THIS_DIR / "../maps/floor4.yaml").resolve()),
    "3": str((THIS_DIR / "../maps/floor3.yaml").resolve()),
    "2": str((THIS_DIR / "../maps/floor2.yaml").resolve()),
    "1": str((THIS_DIR / "../maps/floor1.yaml").resolve()),
}
_current_floor = "5"   # 런치 기본 지도 = all.yaml(5층)
# 🔴 위 값이 "사람이 고른 층"인지 "부팅 하드코딩"인지 구분하는 플래그.
# 둘을 같은 변수로 두면 5층이 **우연히 맞은 것**과 진짜로 확정된 것을 못 가른다.
# POST /switch_map 이 성공할 때만 True 가 된다(사람이 층을 고르는 유일한 경로).
# 미확정이면: 엘베 여정을 거부하고(▲▼ 를 거꾸로 누르게 된다), 엘베앱에는
# confirmed=False 로 알려 층별 높이표·버튼 제한을 **쓰지 않게** 한다.
_floor_confirmed = False
# 우리가 **성공시킨** 마지막 load_map 의 지도 경로. None = 이 프로세스에서 아직
# 한 번도 안 바꿨다(= 런치가 올린 것이 그대로 떠 있다).
# _current_floor 와 다른 물건이다 — 그쪽은 "사람이 고른 층"이고 이쪽은 "실제로
# 로드된 지도"다. 같은 층 확정에서 load_map 을 건너뛸지 판단할 때 이 구분이 필요하다
# (_current_floor 로 판단하면 미확정 초기값을 근거로 쓰는 순환이 된다).
_loaded_map_path = None
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
            data=json.dumps({"granted": granted, "reason": reason}).encode(),
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
        # 조회(GET)는 열어 둔다 — UI 가 상태를 계속 읽는다. 바꾸는 것만 막는다.
        _g = _journey_gate("엘베 제어권 토글")
        if _g:
            return _g
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
    _g = _journey_gate('수동/자동 모드 전환')
    if _g:
        return _g
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

def _map_server_yaml():
    """map_server 가 **런치 때 읽은** 지도 경로(문자열) 또는 None.

    _nav_param_get 과 따로 둔 이유: 그쪽은 값을 float 로 캐스팅한다(padding 전용).
    ※ 이 파라미터는 load_map 서비스로 지도를 바꿔도 갱신되지 **않을 수 있다**.
      그래서 이 값만으로 "지금 로드된 지도"를 단정하지 않는다 — _map_loaded_floor 참고.
    """
    _jt0 = time.monotonic()
    try:
        r = subprocess.run(["ros2", "param", "get", "/map_server", "yaml_filename"],
                           capture_output=True, timeout=5, text=True)
        _jr("gate", "map_server_yaml", rc=r.returncode,
            stdout_head=(r.stdout or "")[:200], stderr_head=(r.stderr or "")[:200],
            latency_ms=round((time.monotonic() - _jt0) * 1000))
        if r.returncode != 0:
            return None
        v = (r.stdout or "").rsplit(":", 1)[-1].strip()
        return v or None
    except Exception as _je:
        _jr("gate", "map_server_yaml", exc=repr(_je),
            latency_ms=round((time.monotonic() - _jt0) * 1000))
        return None


def _floor_of_map(path):
    """지도 경로 → 층 문자열 또는 None. 경로 표기 차이를 realpath 로 눌러서 비교한다."""
    if not path:
        return None
    try:
        tgt = os.path.realpath(path)
    except Exception:
        return None
    for fl, p in _FLOOR_MAPS.items():
        try:
            if os.path.realpath(p) == tgt:
                return fl
        except Exception:
            continue
    return None


@_jr_traced("map_loaded_floor", post=_jr_map_post, ev="gate")
def _map_loaded_floor():
    """지금 map_server 에 올라가 있는 지도의 층 → (층|None, 근거 문자열).

    근거가 둘 있고 **둘 다 혼자서는 못 믿는다**:
      R = 우리가 성공시킨 마지막 load_map 경로 (_loaded_map_path)
      P = map_server 의 yaml_filename 파라미터 (런치가 읽은 값)
    우리가 한 번도 안 바꿨으면 R 이 없고 P 가 진실이다.
    우리가 바꿨는데 P 가 그대로면, P 가 낡은 것인지(load_map 이 파라미터를 안 고친다)
    아니면 R 이 낡은 것인지(nav2 가 재시작돼 런치값으로 돌아갔다) **구분할 수 없다**.
    그래서 **둘이 어긋나면 모른다고 답한다** → 호출부가 안전하게 load_map 을 한다.
    이 논리는 load_map 이 파라미터를 갱신하든 안 하든 둘 다 맞다:
      갱신한다면 R == P 로 일치해 생략이 되고, 갱신 안 하면 한 번 바꾼 뒤부터는
      (조금 낭비지만) 항상 실제 load_map 을 한다.
    """
    p_floor = _floor_of_map(_map_server_yaml())
    r_floor = _floor_of_map(_loaded_map_path)
    if r_floor is None:
        return (p_floor, "map_server 파라미터") if p_floor else (None, "확인 불가")
    if p_floor is None:
        # 🔴 R 단독을 믿지 않는다 — 이 함수의 독스트링이 "둘 다 혼자서는 못 믿는다"인데
        #    예전 구현은 여기서 R 을 믿었다. **P 가 없어지는 것은 예외가 아니라 상시다**:
        #    `ros2 param get` 은 ros2 데몬이 있어야 답하고, 2026-09-10 하루에 데몬
        #    staleness 를 두 번 겪었다(`ros2 daemon stop/start` 로 고쳤다).
        #    사고 경로: 3층으로 바꿈(R=3) → nav2 재시작(실제 지도=런치 기본 5층)
        #              → 데몬 다운(P=None) → R 을 믿어 load_map 생략 + 확정
        #              → AMCL 이 **5층 지도에서 3층이라고 믿는다**.
        #    대가는 중복 load_map 한 번이고, 그건 이 함수가 이미 감수하기로 한 비용이다.
        return None, f"기록({r_floor}층)은 있으나 파라미터를 못 읽어 확인 불가"
    if p_floor == r_floor:
        return r_floor, "우리 기록·파라미터 일치"
    return None, f"기록({r_floor}층)과 파라미터({p_floor}층) 불일치"


def _amcl_init_exit():
    """AMCL 초기 위치를 '엘리베이터 하차지점'으로. switch_map 의 두 경로가 같이 쓴다."""
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


@app.route("/switch_map", methods=["GET", "POST"])
def switch_map():
    """지도(층) 전환 — 재시작 없이 map_server에 load_map 서비스 호출.
    init_exit=true면 전환 직후 AMCL 초기 위치를 '엘리베이터 하차지점'으로 설정
    (두 층 지도가 동일 사본·동일 origin이라 좌표계가 같음 → 좌표 재사용 가능)."""
    global _current_floor, _floor_confirmed, _loaded_map_path
    if request.method == "GET":
        # confirmed 를 같이 준다 — 받는 쪽이 "사람이 고른 값"인지 알아야
        # 추측할지 말지를 스스로 정할 수 있다(엘베앱 보조 폴링이 이걸 본다).
        # loaded_floor 는 **실제로 올라가 있는 지도**다(웹 버튼이 "이 층 확인" /
        # "지도 전환" 중 무엇을 할지 라벨에 미리 써 주는 데 쓴다).
        _lf, _why = _map_loaded_floor()
        return jsonify(floor=_current_floor, confirmed=_floor_confirmed,
                       loaded_floor=_lf, loaded_why=_why)
    data  = request.json or {}
    floor = str(data.get("floor", ""))
    path  = _FLOOR_MAPS.get(floor)
    if path is None:
        return jsonify(ok=False, error="알 수 없는 층"), 400
    # ── 이미 그 층 지도가 올라와 있으면 load_map 을 건너뛴다 ──────────────────
    # 런치 기본이 all.yaml(5층)이라 "5층에서 켜고 **현재 층만 확정**하고 싶다"가 가장
    # 흔하다. 그 경우 같은 지도를 6초 걸려 다시 읽는 것은 낭비이고, 그 과정에서
    # init_exit 를 켠 채 누르는 사고 위험만 생긴다.
    # 모르면(둘이 어긋나면) 건너뛰지 않는다 — 틀린 층을 '확정'으로 만드는 것이
    # 층을 모르는 것보다 나쁘다.
    _lf, _why = _map_loaded_floor()
    if _lf is not None and _lf == floor:
        _current_floor   = floor
        _floor_confirmed = True
        _log("MAP", f"🗺 {floor}층 지도가 이미 로드돼 있다({_why}) — load_map 생략, "
                    "현재 층만 확정")
        _elev_post("/floor", {"floor": floor, "confirmed": True}, timeout=2)
        # 체크박스를 일부러 켰다면 그 요청은 지도와 **별개로** 존중한다 — 조용히
        # 버리면 "켰는데 아무 일도 안 일어났다"가 된다.
        if data.get("init_exit"):
            _amcl_init_exit()
        return jsonify(ok=True, floor=floor, switched=False)
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
    _current_floor   = floor
    _floor_confirmed = True      # 사람(또는 여정)이 실제로 고른 값이 됐다
    _loaded_map_path = path      # 실제로 올라간 지도 — 다음 "같은 층 확정"의 근거
    _log("MAP", f"🗺 지도 전환 완료 → {floor}층 ({os.path.basename(path)})")
    # 엘베앱에 층을 **밀어 넣는다**(폴링 아님). 사용자 모델이 "내가 4층이라 설정하면
    # 지도도 바뀌고 엘리베이터도 바뀐다" 이므로, 고르는 순간에 흘러야 한다. 다음 씬
    # 전환을 기다릴 이유가 없다. 여정의 하차 후 층 갱신(/switch_map init_exit)도
    # 같은 경로라 공짜로 따라온다.
    # 엘베앱이 꺼져 있으면 _elev_post 가 조용히 실패한다 — 그쪽은 기동 시 1회
    # 보조 폴링(GET /switch_map)으로 메우므로 여기서 재시도하지 않는다.
    _elev_post("/floor", {"floor": floor, "confirmed": True}, timeout=2)
    if data.get("init_exit"):
        _amcl_init_exit()
    return jsonify(ok=True, floor=floor, switched=True)

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

# 🔴 "한 번도 안 왔다"를 영원히 `unknown` 으로 두면 **정상 초기 상태와 영구 고장이 같은
#    표시**가 된다. 2026-09-10 에 그 값을 치렀다 — 손잡이 시리얼이 7일 동안 한 번도
#    안 붙어 `"시리얼 연결 실패"` 가 10,307줄 쌓였는데, 화면의 손잡이 pill 은 첫날부터
#    "모름"이었고 아무도 고장으로 읽지 않았다. 당김·버튼 이벤트는 그 7일간 0건이다.
#    (실제 꽂힌 FTDI 는 AG0KRCTV/AG0KSATI = Stretch 자기 dynamixel 이고, 코드가 찾는
#     A5069RR4 는 이 로봇에 없다. 포트 고정은 별 건 — wip 의 udev 초안 참고.)
# ⇒ 기동 직후 짧은 유예 동안만 `unknown`, 그 뒤로는 `bad` 다.
_BOOT_MONO = time.monotonic()
_READY_GRACE_MULT = 2.0    # 유예 = 임계값 × 이 배수
_READY_GRACE_MIN  = 10.0   # 다만 최소 이만큼은 기다린다(기동 직후 혼잡)


def _ready_grace(key) -> float:
    return max(_READY_GRACE_MIN, _READY_THRESH[key] * _READY_GRACE_MULT)


def _readiness_signal(key, label):
    """콜백형 신호 판정. 🔴 **"연결된 적 없음"과 "연결됐다 끊김"을 갈라서 말한다** —
    사람에게 요구하는 행동이 다르다(꽂아라 vs 확인해라)."""
    updated_at = _ready[key]
    if updated_at <= 0.0:
        since = time.monotonic() - _BOOT_MONO
        grace = _ready_grace(key)
        if since <= grace:
            return {"status": "unknown", "age_sec": None,
                    "detail": f"{label} 미수신 (기동 {since:.0f}s — {grace:.0f}s 까지 대기)"}
        return {"status": "bad", "age_sec": None,
                "detail": f"{label} **한 번도 수신 안 됨** ({since / 60:.0f}분째) — "
                          "연결·포트를 확인하세요"}
    age = time.monotonic() - updated_at
    if age > _READY_THRESH[key]:
        return {"status": "bad", "age_sec": round(age, 1),
                "detail": f"{label} 신호 끊김 — 받다가 {age:.0f}s 무소식"}
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
    # 여정 중에는 잠근다 — 실행 도중 컨트롤러 속도가 바뀌면 그 실행의 기록이 해석 불가가 된다.
    # (실제 값을 기록하는 쪽은 ros2 param get 을 새로 불러야 해서 택하지 않았다)
    _g = _journey_gate('로봇 속도 변경')
    if _g:
        return _g
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
    _g = _journey_gate('팔 수납')
    if _g:
        return _g
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
_ELEV_SCRIPT_REL = "elevator_button_press/main.py"


def _elev_app_pids() -> list:
    """진짜 엘베앱 프로세스의 PID 목록. 없으면 빈 리스트.

    왜 pgrep 결과를 그대로 못 쓰나: `pgrep -f` 는 명령줄 **전체 문자열**에 매치한다.
    그래서 이 경로를 인자로 가진 아무 프로세스나 잡힌다. 2026-09-08 실제로 잡힌 것:
        /bin/bash -c source ~/.claude/shell-snapshots/snapshot-bash-….sh …
        … grep -n "elevator_button_press/main.py" …
    엘베앱이 완전히 꺼져 있는데도 '떠 있다'가 나왔고 대시보드 토글이 깜빡였다.

    그래서 pgrep 은 후보를 싸게 추리는 데만 쓰고, 판정은 /proc/<pid>/cmdline 의 argv 를
    직접 갈라서 한다. cmdline 은 NUL 구분이라 공백 섞인 인자에도 안 흔들리고,
    정규식 이스케이프에 기대지 않는다.

    참으로 보는 조건:
      argv[0] 이 python 계열이고 인자 중 하나가 그 스크립트 경로로 끝난다
      (대시보드는 [sys.executable, "-u", ".../elevator_button_press/main.py"] 로 띄운다)
      또는 argv[0] 자체가 그 스크립트다(셰방으로 직접 실행하는 경우 대비).
    """
    try:
        r = subprocess.run(["pgrep", "-f", _ELEV_SCRIPT_REL],
                           capture_output=True, text=True)
    except Exception:
        return []
    pids = []
    for tok in (r.stdout or "").split():
        try:
            pid = int(tok)
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                argv = [a.decode("utf-8", "replace") for a in f.read().split(b"\0") if a]
        except Exception:
            continue          # 그새 죽었거나 못 읽음 — 후보에서 뺀다
        if not argv:
            continue
        exe = os.path.basename(argv[0])
        if (exe.startswith("python") and any(a.endswith(_ELEV_SCRIPT_REL) for a in argv[1:])) \
                or argv[0].endswith(_ELEV_SCRIPT_REL):
            pids.append(pid)
    return pids


def _elev_app_running() -> bool:
    p = _procs.get("elevator")
    if p and p.poll() is None:
        return True
    # 대시보드가 추적 못 하는 것(재시작 desync)도 감지 — 그 폴백은 그대로 살린다.
    return bool(_elev_app_pids())

@_jr_traced("wait_elev_app_up")
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

@_jr_traced("grant_elev_lease", ev="gate")
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
        _jr("gate", "lease_attempt", granted=granted, reason=reason, attempt=i + 1,
            attempts=attempts, ok=ok, st=_jr_pick(st, ("authority", "guard_off", "lease_expired")))
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
    _g = _journey_gate('엘베앱 켜기/끄기')
    if _g:
        return _g
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
        # pkill -f 를 쓰면 안 된다 — _elev_app_pids 주석의 그 오탐이 여기서는
        # '남의 프로세스에 SIGTERM 을 보낸다'가 된다(실제로 Claude 세션의 bash 가
        # 매치됐다). 판정을 마친 PID 에만 정확히 보낸다.
        for _pid in _elev_app_pids():
            try:
                os.kill(_pid, signal.SIGTERM)
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
         # force: '강제로 넘어가기' 1회용 신호. 안무가 도는 동안 '다음'이 잠겨 있을 때
         # 사람이 그래도 진행시키려고 누르는 버튼이다. 누른 쪽(/auto_force)이 로봇을
         # '먼저' 멈추고 나서 이걸 세운다 — 순서가 바뀌면 이 버튼이 막으려던 위험이 된다.
         # 소비하는 쪽(_elev_wait_scene_done / _auto_wait_confirm)이 즉시 False 로 되돌린다.
         "force": False,
         "phase": "", "dest_floor": "", "mode": ""}
_auto_lock = threading.Lock()

def _auto_set(step, msg, wait=False, phase=None):
    """단계 상태 갱신. phase=전체 흐름 트래커에서 하이라이트할 단계 id(없으면 유지)."""
    with _auto_lock:
        _AUTO["step"] = step; _AUTO["msg"] = msg; _AUTO["waiting"] = wait
        if phase is not None:
            _AUTO["phase"] = phase
    _jr("step", step, msg, wait, phase)
    _log("AUTO", f"[{step}] {msg}" + ("  — 확인 대기" if wait else ""))

def _auto_wait_arrival(name, tol=0.10, settle=1.0, timeout=200):
    """name 지점 '정밀 도착'까지 대기. nav이 5cm로 서므로, 여기선 목표 tol(기본 10cm)
    이내에서 로봇이 settle초간 '멈춰있으면'(=nav 완료 = 정밀 도착) True.
    - 단순히 tol 이내를 지나가는 중인 것과 구분하려고 '정지'까지 확인 (jitter 여유로 10cm).
    - nav 정밀도(5cm)와 일관: 로봇은 ≤5cm에 서고, 오케스트레이터는 그 정지를 감지해 진행.
    cancel/timeout이면 False."""
    p = _loc(name)
    if not p:
        _jr_gate_arrival(name, None, None, None, None, tol, timeout, "no_loc", False)
        return False
    tx, ty = p.get("x"), p.get("y")
    _jr("set_goal", name, p)
    t0 = time.monotonic()
    stable_since = None
    last = None
    while time.monotonic() - t0 < timeout:
        if _AUTO["cancel"]:
            _jr_gate_arrival(name, tx, ty, t0, stable_since, tol, timeout, "cancel", False)
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
                _jr_gate_arrival(name, tx, ty, t0, stable_since, tol, timeout, "arrived", True)
                return True      # 목표 이내 + settle초 정지 = 정밀 도착 확정
        else:
            stable_since = None
        if rx is not None:
            last = (rx, ry)
        time.sleep(0.3)
    _jr_gate_arrival(name, tx, ty, t0, stable_since, tol, timeout, "timeout", False)
    return False

@_jr_traced("auto_wait_confirm", post=_jr_cancel_post)
def _auto_wait_confirm(timeout=900):
    """블로커 단계 — 사용자 '다음 확인' 대기. /auto_confirm이 waiting=False로 풀어줌."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if _AUTO["cancel"]:
            return False
        with _auto_lock:
            if _AUTO.get("force"):
                # 안무가 막 끝난 순간에 강제 버튼이 눌린 경우. 여기서 소비하지 않으면
                # 그 신호가 다음 씬의 대기를 건너뛴다 — 반드시 여기서 털어낸다.
                _AUTO["force"] = False
                _AUTO["waiting"] = False
                return True
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


# ── Nav2 파라미터 '평소값' 복원 — 복원을 '해제 시점'이 아니라 '사용 시점'에 보장 ──
# 엘베 구간에서 Nav2 파라미터를 바꿨다가 되돌리는 설계(B안)를 넣게 되면, try/finally 는
# 1겹도 못 된다(SIGKILL·전원·예외 중 예외). 그래서 **복도 주행이 시작되는 자리**에서
# 무조건 평소값을 확인한다. 해제 경로의 성공을 전혀 가정하지 않는다.
#   1겹: nav2 yaml 을 고치지 않는다 → Nav2 재시작이 곧 자가복구다. 이 전제가 깨지면
#        1겹이 사라지므로, 엘베값을 yaml 에 넣는 변경은 하지 말 것.
#   2겹: 여기 — /goto 직전. 복도 주행은 /goto 로만 일어난다(호출처 4곳).
#   3겹: 준비상태 폴러의 주기 확인 (아직 없음 — B안 때 추가).
# ※ 지금(2026-09-09, C안 1주차)은 파라미터를 **하나도 바꾸지 않는다.** 이 함수는 그때
#   무해한 no-op 이고, 나중에 누가 엘베값을 넣었을 때 유일한 방어선이 된다.
# ※ 복원 실패의 결과는 "로봇이 사람을 치는 것"이 아니라 "로봇이 안 가는 것"이다 —
#   footprint 가 작아지면 footprint_clearing_enabled 가 동반자를 못 지워서 동반자가
#   장애물이 되고 로봇이 멈춘다. 물리 위험이 아니라 기능 고장이라 fail-open 으로 둔다.
# ※ 값은 yaml 에서 읽는다(하드코딩하지 않는다) — _EXIT_TARGET_CM 처럼 상수로 박으면
#   나중에 yaml 을 튜닝했을 때 조용히 되돌려 버린다. 같은 함정을 또 밟지 않는다.
_NAV2_PARAMS_YAML = Path("/home/hello-robot/ament_ws/src/stretch_ros2/stretch_nav2/"
                         "config/nav2_params_human.yaml")
_NAV2_PAD_NODE  = "/local_costmap/local_costmap"
_NAV2_PAD_PARAM = "footprint_padding"


def _nav_normal_padding():
    """nav2 yaml 이 말하는 local costmap footprint_padding. 못 읽으면 None."""
    try:
        import yaml as _yaml
        d = _yaml.safe_load(_NAV2_PARAMS_YAML.read_text("utf-8")) or {}
        v = ((d.get("local_costmap") or {}).get("local_costmap") or {}) \
            .get("ros__parameters", {}).get(_NAV2_PAD_PARAM)
        return float(v) if v is not None else None
    except Exception as e:
        _log("NAV2", f"파라미터 yaml 읽기 실패 — 평소값 확인 생략 ({e!r})")
        return None


def _nav_param_get(node, name):
    """ros2 param get 한 번. 값(float) 또는 None(모른다). 예외를 던지지 않는다.

    ※ 2026-09-09 실측: nav2 실행 중이면 **데몬이 있어도 즉시 응답한다**(읽기 3/3,
      local 0.08 / global 0.01). 오히려 `ros2 param get --no-daemon` 쪽이
      /global_costmap/global_costmap 을 "Node not found" 로 놓쳤다(같은 노드를
      ros2 node list 는 보여준다). 그래서 여기서는 데몬 경로를 그대로 쓴다.
    """
    _jt0 = time.monotonic()
    try:
        r = subprocess.run(["ros2", "param", "get", node, name],
                           capture_output=True, timeout=5, text=True)
        _jr("gate", "param_get", inputs={"node": node, "name": name}, rc=r.returncode,
            stdout_head=(r.stdout or "")[:160], stderr_head=(r.stderr or "")[:200],
            latency_ms=round((time.monotonic() - _jt0) * 1000))
        if r.returncode != 0:
            return None
        return float((r.stdout or "").rsplit(":", 1)[-1].strip())
    except Exception as _je:
        _jr("gate", "param_get", inputs={"node": node, "name": name}, exc=repr(_je),
            latency_ms=round((time.monotonic() - _jt0) * 1000))
        return None


@_jr_traced("nav_params_restore_normal", ev="gate")
def _nav_params_restore_normal(why=""):
    """/goto 직전 호출. **반환: 주행해도 되는가(bool).** False 면 호출부가 거부해야 한다.

    **읽어서 다를 때만 쓴다.** 같은 값을 매번 쓰면 살아 있는 코스트맵에 불필요한
    footprint 갱신을 넣는 셈이고, 이 함수는 사람을 태운 여정의 주행 직전에 돈다.
    정상 상태(= 아무도 안 바꿨을 때)의 라이브 쓰기는 **0 회**다.

    🔴 읽기 실패를 '같다'로 넘기지 않는다. 읽기 실패는 "모른다"이지 "정상"이 아니다.
       모르면 쓴다. 쓰기까지 실패하면 **주행을 거부한다** — 반환값을 안 보고 이동으로
       진행하는 것이 이 프로젝트의 뿌리 A(실패·취소 시 안전복귀 실패)다.
    """
    want = _nav_normal_padding()
    if want is None:
        # 목표값을 모르면 쓸 수도 없다. 다만 이 빌드는 엘베 구간에서 파라미터를
        # 하나도 바꾸지 않으므로(2026-09-09 C안) 되돌릴 것도 없다 — 경고만 하고
        # 주행은 허용한다.
        # 🔴 엘베용 파라미터 전환을 넣는 날, 이 분기는 **거부**로 바뀌어야 한다.
        #    그때 이 주석을 지우지 말고 고쳐라.
        _log("NAV2", "🚨 nav2 파라미터 yaml 을 못 읽어 평소값을 확인하지 못했다 — "
                     "이 빌드는 엘베 파라미터를 바꾸지 않으므로 주행은 계속한다"
                     + (f" [{why}]" if why else ""))
        return True
    cur = _nav_param_get(_NAV2_PAD_NODE, _NAV2_PAD_PARAM)
    _jr("gate", "nav_padding_read", why=why, want=want, cur=cur)
    if cur is not None and abs(cur - want) <= 1e-6:
        return True                 # 평소값이다 — 아무것도 쓰지 않는다(정상 경로)
    unknown = (cur is None)
    _log("NAV2", (f"⚠ {_NAV2_PAD_NODE} {_NAV2_PAD_PARAM} 조회 실패 — '모른다'이므로 "
                  f"평소값 {want} 을 그대로 쓴다" if unknown else
                  f"🚨 {_NAV2_PAD_PARAM} 이 평소값과 다름 {cur} → {want} 복원 시도 "
                  "— 엘베값이 복도로 새어 나온 것이다. 원인을 찾아라")
                 + (f" [{why}]" if why else ""))
    try:
        w = subprocess.run(["ros2", "param", "set", _NAV2_PAD_NODE, _NAV2_PAD_PARAM,
                            str(want)], capture_output=True, timeout=5, text=True)
        ok = (w.returncode == 0)
        err = (w.stderr or w.stdout or "").strip()[:120]
    except Exception as e:
        ok, err = False, repr(e)
    _jr("gate", "nav_padding_write", why=why, want=want, cur=cur, ok=ok, err=err)
    if ok:
        _log("NAV2", f"{_NAV2_PAD_PARAM} = {want} 재설정 성공"
                     + (f" [{why}]" if why else ""))
        return True
    _log("NAV2", f"🚨 {_NAV2_PAD_PARAM} 재설정 실패 → **주행 거부**. {err}"
                 + (f" [{why}]" if why else ""))
    return False


@_jr_traced("auto_scene_step")
def _auto_scene_step(label, n, busy_msg, noanmu_msg, phase):
    """씬 n 을 '잠금 → 실행 → 완료 대기 → 결과와 함께 개방' 순서로 돌린다.

    왜 이 순서인가: 기존에는 _auto_set(wait=True) 가 _elev_scene() **앞**에 있었다.
    그래서 로봇이 도는 동안 '다음' 버튼이 살아 있었고, 사람이 그때 누르면 회전
    중인데도 다음 씬으로 넘어갔다(②③④ 전부). 잠그고 → 돌리고 → 끝난 뒤 결과와
    함께 연다. (2026-09-08 사용자 승인 ③안: 이동 중 잠금 + 별도 강제 버튼)

    반환: (계속할까, 사유, 결과dict)
    미달이어도 시퀀스를 중단시키지 않는다 — 사람이 화면을 보고 판단한다. 여기서
    하는 일은 '사실대로 띄우고 사람이 누를 때까지 기다리는 것'까지다.
    """
    with _auto_lock:
        _AUTO["force"] = False        # 이전 단계에서 남았을 리 없지만 확실히 턴다
    _auto_set(label, busy_msg, wait=False, phase=phase)      # ← 여기서 '다음'이 잠긴다
    sent, seq = _elev_scene(n)
    if not sent:
        _auto_set(label, f"⚠ 엘베앱에 {label} 전송 실패 — 확인 후 '다음'",
                  wait=True, phase=phase)
        return _auto_wait_confirm(), "post", None
    why, res = _elev_wait_scene_done(n, seq)
    txt = _scene_res_txt(res)
    tail = f" — {txt}" if txt else ""
    # 미달일 때는 사유까지 보여야 한다 — "미달"인지 "좌표 거부 → 하드코딩 폴백"인지가
    # 사람이 다음을 누를지 말지 가르는 정보다.
    why_txt = (res or {}).get("reason") or ""
    fail_tail = " — " + " · ".join(x for x in (why_txt, txt) if x) if (why_txt or txt) else ""
    msg = {
        "done":    f"{label} 완료 ✅{tail} · '다음'",
        "fail":    f"{label} ⚠ 미달{fail_tail} · 확인 후 '다음'",
        "timeout": f"⚠ {label} 안무가 안 끝남(시간초과) — 엘베UI 확인 후 '다음'",
        "noapp":   f"⚠ 엘베앱 응답 없음({label}) — 확인 후 '다음'",
        "noanmu":  noanmu_msg,
        "forced":  f"⏭ 강제로 넘어감 — {label} 미완료 상태로 진행",
    }.get(why, f"{label} — {why}{tail} · 확인 후 '다음'")
    if why == "cancel":
        return False, why, res
    if why == "forced":
        # 강제 = 확인 절차까지 건너뛴다. 여기서 다시 wait=True 로 열면 방금 누른
        # 사람에게 한 번 더 누르라고 요구하는 셈이라 버튼의 의미가 없어진다.
        _auto_set(label, msg, wait=False, phase=phase)
        return True, why, res
    _auto_set(label, msg, wait=True, phase=phase)             # ← 여기서 '다음'이 열린다
    return _auto_wait_confirm(), why, res


def _elev_status(timeout=3):
    """엘베앱 상태(/status) 조회 — dict 또는 None. ready=정렬완료, door_open 등 포함.

    블랙박스: 이미 받은 응답을 버리지 않고 넘길 뿐이다(새 호출 0). 여정 중이 아니면 레코더가
    캐시만 갱신하고 적지 않는다."""
    try:
        import urllib.request
        r = urllib.request.urlopen("http://localhost:5000/status", timeout=timeout)
        st = json.loads(r.read().decode() or "{}")
    except Exception as _je:
        _jr("elev_status", None, _je, _jr_caller())
        return None
    _jr("elev_status", st, None, _jr_caller())
    return st

@_jr_traced("elev_scene")
def _elev_scene(n, move=True):
    """엘베앱 씬 n 트리거(자세 전환·자동안무). **(전송성공, run_seq)** 반환.

    run_seq 는 이번 안무의 실행번호다. _elev_wait_scene_done 에 그대로 넘기면 '내가
    시킨 그 회차'의 결과만 인정하므로, 낡은 회차 결과를 자기 것으로 오독하는 일이
    없다. 자동 안무가 없는 씬(SCENE_MOVES 에 없는 ①③⑤)은 None 이다.
    ※ 반환을 bool 하나로 두면 '전송 실패'와 '안무 없음'을 못 가른다. 그래서 튜플이다.
    자세 전송이 블로킹(~10s)이라 여유 타임아웃."""
    # 현재 층을 같이 보낸다 — 엘베앱이 승강장 버튼 높이를 층별로 고르는 데 쓴다.
    # 엘베앱은 층을 스스로 알 길이 없다. 이 값의 신뢰도는 이미 여정 전체가 의존하는
    # 것과 같다(상/하행 버튼 선택 `up = int(dest_floor) > int(_current_floor)`, 지도 전환).
    # move=False → 단계 표시·누적 리셋·자세 전환만. 자동 안무는 띄우지 않는다
    # (다른 주체가 이미 그 자리로 데려다 놨을 때. 예: ② 를 Nav2 로 가는 경로).
    # floor_confirmed 를 같이 보낸다 — 층 값만 보내면 받는 쪽이 "부팅 초기값 5"와
    # "사람이 고른 5"를 구분할 수 없다.
    r = _elev_post("/scene", {"n": int(n), "floor": _current_floor,
                              "floor_confirmed": _floor_confirmed, "move": bool(move)},
                   timeout=20)
    if r is None:
        return False, None
    return True, r.get("run_seq")

@_jr_traced("elev_select")
def _elev_select(text):
    """버튼 자동 선택(POST /select). 호출=^(상)/s(하), 층=번호. 성공 시 True."""
    r = _elev_post("/select", {"text": str(text)}, timeout=5)
    return bool(r and r.get("ok", True))

@_jr_traced("elev_press")
def _elev_press():
    """누르기 실행(POST /press). (성공여부, 사유) 반환.

    사유를 버리면 "왜 거부됐는지"가 여기서 끊긴다 — 거부 대부분은 고장이 아니라
    "지금은 안 된다"라서, 사유를 봐야 기다릴 일인지 멈출 일인지 가른다.
    사유는 응답이 없을 때 None."""
    r = _elev_post("/press", {}, timeout=15)
    return bool(r and r.get("ok")), (r or {}).get("error")

@_jr_traced("elev_wait_ready", post=_jr_cancel_post)
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

@_jr_traced("elev_wait_press_done", post=_jr_cancel_post)
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

# ⑥ 하차 완료 판정은 2026-09-09 부터 엘베앱의 실행 기록(scene_result)으로 한다.
# 그래서 _EXIT_TARGET_CM(-186.0) · _EXIT_TOL_CM(1.0) · _EXIT_NEAR_CM(5.0) 을 지웠다.
#   왜: 그 셋은 "scene_acc 누적이 하드코딩 -186cm 에 닿았나"를 재는 자였다. 좌표
#   경로가 들어오자 누적이 시작점에 따라 -147~-226cm 로 변해서, 하필 좌표가 잘 될
#   때만 'stall' → 구조 절차가 도는 역전이 났다(91a3bfb. 실측 -206.9cm → stall).
#   이제 도달 여부는 엘베앱이 POSE_DONE_M(5cm)/POSE_DONE_DEG(5°)로 판정해
#   scene_result.ok 에 싣는다 — 데이터가 있는 곳에서 판정하는 것이 맞고, 오버슛·
#   진동 강등도 거기서 흡수된다(그게 _EXIT_TOL_CM/_EXIT_NEAR_CM 가 하던 일이다).
# scene_acc 는 버리지 않는다 — 아래 _EXIT_STALL_SEC 의 '진행이 멎었나'(생존 판정)에
# 계속 쓴다. 완료 판정에는 쓰지 않는다. 두 역할을 분리한 것이 이번 변경의 핵심이다.
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
# 진행(생존) 판정 임계 — scene_acc 의 두 채널 각각.
# 근거(9/8·9/9 로그 [MOVE] 전수): scene_acc 는 스텝이 **끝날 때만** 갱신되는 계단
# 함수라 연속 잡음이 없다. 그래서 임계는 '가장 작은 실제 스텝보다 작게'만 잡으면 된다.
#   회전 1회 크기 표본 30개: 최소 3° · 5퍼센타일 5° · 중앙 23° · 최대 91° → 1.0° = 3배 여유
#   병진 보정 스텝 관측 최소 1.7cm                                      → 0.5cm = 3.4배 여유
# (0.5cm 는 2026-09-08 이전 코드가 쓰던 값 그대로다 — 바꿀 이유가 없었다.)
_EXIT_PROG_CM   = 0.5
_EXIT_PROG_DEG  = 1.0
# 절대 상한(무한대기 방지 백스톱). 2026-09-09: 60.0 → 120.0.
# 근거 강도를 정확히 적는다 —
#   · **씬5(⑥) 좌표 경로 실측은 0건이다.** 씬5 좌표 목표는 오늘 처음 들어갔다.
#   · 가장 가까운 실측은 같은 좌표를 쓰는 ② 문앞(씬1)이다: 9/8·9/9 3건 중 최악 58.6s.
#     60초는 그 최악과 1.4초 차라 정상 완주를 timeout 으로 죽인다.
#   · 유사 씬 ④ 엘리베이터 안(1.9m)이 52.7s — 거리는 비슷하지만 같은 씬이 아니다.
#   · 반대로 _SCENE_MAX_SEC(240)을 쓰면 문턱에 걸친 로봇의 구조 호출이 3분 뒤다.
#   120 = ② 최악 58.6s 의 2.0배. 정상 경로가 아니라 백스톱이다.
#   ⑥ 좌표 실측이 쌓이면 이 값을 다시 보라.
_EXIT_MAX_SEC   = 120.0
_EXIT_POLL_SEC  = 0.4
_EXIT_MISS_MAX  = 2       # /status 무응답이 이만큼 연속되면 앱이 죽은 것으로 본다

# ⑥ 하차가 끝나지 않은 채 여정이 끊긴 상태 — finally가 리스를 되돌리지 못하게 막는다.
# 해제는 /elevator_app {running:false} 한 곳에서만 (거기서 리스도 물리적으로 소멸).
_rescue_hold = False


@_jr_traced("elev_wait_exit_done")
def _elev_wait_exit_done(seq=None):
    """⑥ 하차 완료를 엘베앱 실행 기록으로 확인. (사유, 결과dict, 진행cm) 반환.

    사유: "done"=목표 자세 도달 / "fallback"=좌표 거부 후 하드코딩 폴백으로 탈출
          완주(목표 자세는 아니다) / "fail"=끝났지만 미달 / "noanmu"=안무가 안 떴다 /
          "stall"=진행이 멎음 / "timeout"=절대상한 / "cancel"=사용자 취소 /
          "forced"=사용자 강제 / "noapp"=엘베앱 상태 조회 불가 /
          "noseq"=실행번호 없이 불렸다(호출부 오류 — 방어)

    ■ 두 신호를 같이 보는 이유 — 역할이 다르다
      · 성공 판정 = scene_result. 엘베앱이 POSE_DONE_M/DEG 로 이미 판정한 값이다.
        2026-09-08 까지는 여기서 scene_acc 누적을 하드코딩 -186cm 와 비교했는데,
        좌표 경로의 누적은 시작점에 따라 -147~-226cm 로 변해서 좌표가 잘 될 때만
        실패로 뜨는 역전이 났다(91a3bfb).
      · 생존 판정 = scene_acc **두 채널 모두**(fwd_cm·rot_deg). running 만 보면 엘베앱이 스텝 중간에
        굳었을 때 10초가 아니라 절대상한(120초)까지 기다린다. 사람을 태운 채 문턱에
        걸친 상황에서 그 차이가 크다. 그래서 '진행이 멎었나'는 계속 본다.
        완료 판정에는 절대 쓰지 않는다.

    ■ 강제(forced)는 ⑥에서 'done' 이 아니다 — 규칙의 귀결이다
      강제는 "이 대기를 건너뛴다"이지 "전제조건이 충족됐다고 선언한다"가 아니다.
      ②③④는 다음 행위가 가역적(다음 씬, 여전히 엘베앱 안)이라 대기만 건너뛰어도
      된다. ⑥은 다음이 비가역·자동이다 — 리스 반납 → /switch_map(지도 전환) →
      /elevator_app{running:False}(구조용 조종 패드가 사라진다) →
      /goto(사람 태운 채 Nav2 출발). 그리고 버튼을 누른 사람에게는 "로봇이 실제로
      나왔나"를 확인할 정보가 없다. 그래서 ⑥의 강제는 _exit_failed 로 보낸다:
      제어권·앱을 유지하고 여정만 끊는다 = "그만 묻고 수동 제어를 달라".

    ※ seq 는 _elev_scene(5) 가 돌려준 실행번호다. 반드시 넘겨라 — 안 넘기면 직전
      회차의 낡은 기록을 자기 것으로 오독할 수 있다(_elev_wait_scene_done 과 같은 사유).
    """
    if seq is None:
        # 방어 — 호출부가 이미 걸러야 하지만, seq 없이 들어오면 '남의 회차'를 자기
        # 것으로 읽을 수 있다. 특히 같은 앱 세션에서 ⑥을 한 번 성공시킨 뒤 다시
        # 돌면 n=5/running=False/ok=True 기록이 남아 **후진 0cm 로 done 이 난다.**
        return "noseq", None, None
    t0 = time.monotonic()
    last_prog = t0
    cur  = None          # 마지막으로 읽은 누적 전진량 — 실패 통보에 그대로 실린다
    prev_f = prev_r = None   # 직전 폴링의 누적값 — '변했나'의 기준
    started = False      # 이 씬에서 실제 이동이 한 번이라도 기록됐나
    last_res = None      # 마지막으로 본 실행 기록(실행중이어도 담는다)
    miss = 0             # /status 연속 무응답 횟수
    while True:
        with _auto_lock:
            if _AUTO.get("force"):
                _AUTO["force"] = False      # 1회용 — 다음 씬으로 새어나가면 안 된다
                return "forced", last_res, cur
        if _AUTO["cancel"]:
            return "cancel", last_res, cur
        st = _elev_status(timeout=1.0)
        if st is None:
            # 1회 표본으로 단정하지 않는다. noapp 은 stall 과 달리 _rescue_hold 를
            # 세우지 않아 finally 가 리스를 반납하는데, 일시적 끊김에 오탐하면
            # 이 함수가 지키려던 구조용 조종 패드를 그대로 잃는다.
            miss += 1
            if miss >= _EXIT_MISS_MAX:
                return "noapp", last_res, cur
        else:
            miss = 0
            # (1) 생존 신호 — 진행이 있었나
            # 🔴 2026-09-09 수정. 이전 판정은 **정상 하차를 끊었다**(실측 3/6, 최악 29.2초):
            #   (a) fwd_cm 만 읽어서 **회전 레그가 통째로 무진행으로 보였다.** ③정렬은
            #       몇 초~수십 초 동안 회전만 한다. ⑥은 후진이라 마지막 ③정렬이 통째로
            #       무갱신 구간이고, 씬5 목표는 씬1과 **같은 좌표**다 — 29.2s·17.2s 를
            #       낸 바로 그 씬이다.
            #   (b) '지금까지의 최댓값 갱신'만 진행으로 쳐서, 목표를 넘었다 되돌아오는
            #       보정 스텝(뒤로 갔다 앞으로)이 생존 신호가 되지 못했다.
            # ⇒ 두 채널 모두 읽고 **방향 무관 |Δ|** 로 본다. 9/8·9/9 좌표 경로 씬 전수
            #   6건 재생: 최대 무진행 창 29.2s → 6.0s (문턱 10s 초과 3건 → 0건).
            #   상수를 늘리지 않았다 — 늘리면 '진짜 정지'도 그만큼 기다린다(문에 끼인 채로).
            acc = st.get("scene_acc") or {}
            _f, _r = acc.get("fwd_cm"), acc.get("rot_deg")
            moved = False
            if isinstance(_f, (int, float)):
                cur = float(_f)
                if prev_f is not None and abs(cur - prev_f) > _EXIT_PROG_CM:
                    moved = True
                prev_f = cur
            if isinstance(_r, (int, float)):
                _rv = float(_r)
                if prev_r is not None and abs(_rv - prev_r) > _EXIT_PROG_DEG:
                    moved = True
                prev_r = _rv
            if moved:
                last_prog = time.monotonic()
            # started = 이 씬이 실제로 움직인 적이 있다. 누적은 단계 진입 때 0 으로
            # 리셋되므로, 누적이 0 이 아니면 그 자체가 '움직였다'는 증거다.
            if moved or abs(prev_f or 0.0) > _EXIT_PROG_CM \
                     or abs(prev_r or 0.0) > _EXIT_PROG_DEG:
                started = True
            # (2) 완료 판정 — 실행 기록
            res = st.get("scene_result") or {}
            if res:
                last_res = res
            if seq is not None and res.get("seq") != seq:
                # 내가 시킨 회차가 아니다 = 안무가 안 떴다. /scene 라우트가 응답
                # 전에 기록을 세우므로 '아직 안 생김'은 아니다. 옛 코드는 이 경우
                # scene_acc 가 0 인 채로 유예 20초를 태운 뒤 stall 로 잡았다.
                return "noanmu", res, cur
            if res.get("n") != 5:
                return "noanmu", res, cur
            if not res.get("running", True):
                if res.get("ok"):
                    return "done", res, cur
                # 좌표 거부 → 하드코딩 폴백. 폴백이 완주했으면 목표 자세는 아니어도
                # 엘리베이터에서는 나왔다. 거기서 구조 절차를 부르는 것은 오경보다.
                # 폴백이 중간에 멈춘 경우(fallback_ok=False)는 진짜 실패다.
                if res.get("fallback") and res.get("fallback_ok"):
                    return "fallback", res, cur
                return "fail", res, cur
        now = time.monotonic()
        # 첫 폴링에서 0.0 을 읽은 것은 진행이 아니므로 그때까지는 유예를 쓴다.
        limit = _EXIT_STALL_SEC if started else _EXIT_START_GRACE
        if now - last_prog >= limit:
            # 옛 코드는 여기서 _EXIT_NEAR_CM 강등("진동 보정으로 스스로 마침")을 했다.
            # 이제 그 경우는 엘베앱이 running=False 로 닫고 ok 를 실어 주므로 위
            # 완료 분기에서 먼저 잡힌다 — 강등 창이 필요 없다.
            return "stall", last_res, cur
        if now - t0 >= _EXIT_MAX_SEC:
            return "timeout", last_res, cur
        time.sleep(_EXIT_POLL_SEC)


# ②④ 자동 안무 완료 판정용 — ⑥ 하차의 _elev_wait_exit_done 과 같은 자리의 함수다.
# 절대 상한. 좌표 안무는 최대 SCENE_MAX_ITERS(4)회 반복이고 한 회가 ①정렬·②주행·
# ③정렬 3구간이다. ④ 엘리베이터 안의 185cm는 50cm 스텝 4번이고 한 스텝 타임아웃이 최악
# 8.5s라(엘베앱 _manual_trans) 주행만 34s — 1회 ≈50s, 4회 ≈200s. 여기에 여유 40s.
# 정상값은 20~40s다. 이 상한에 닿는 것 자체가 고장 신호다.
_SCENE_MAX_SEC  = 240.0
_SCENE_POLL_SEC = 0.4
_SCENE_MISS_MAX = 2       # /status 무응답이 이만큼 연속되면 앱이 죽은 것으로 본다
                          # (_EXIT_MISS_MAX 와 같은 근거 — 1회 표본으로 단정하지 않는다)

@_jr_traced("elev_wait_scene_done")
def _elev_wait_scene_done(n, seq=None, timeout=None):
    """씬 n 자동 안무가 끝날 때까지 대기. (사유, 결과dict) 반환.

    사유: "done"=목표 도달 / "fail"=끝났지만 미달·중단(결과dict의 reason에 이유)
          "timeout"=절대상한 / "cancel"=사용자 취소 / "noapp"=엘베앱 상태 조회 불가
          "noanmu"=그 씬은 자동 안무가 없다(자세 전환만 — 기다릴 것이 없음)

    /scene은 안무 스레드를 start한 뒤 즉시 ok를 돌려준다 — 접수이지 완료가 아니다
    (_elev_wait_exit_done 과 같은 이유). 그래서 판정을 응답이 아니라 엘베앱이
    /status에 남기는 실행 기록(scene_result)에서 읽는다.
    2026-09-04 실기: 씬②가 잔여 yaw -97.3°로 끝났는데 그 사실이 로그에만 남고
    대시보드까지 올라오는 길이 아예 없었다.

    ※ 반드시 _elev_scene(n) 직후에 불러라. /scene 라우트가 응답을 돌려주기 전에
      실행 기록을 '실행중'으로 세워 두므로, 그 직후라면 낡은 기록을 볼 수 없다.
    seq: /scene 응답의 run_seq(실행번호). 넘기면 '내가 시킨 그 회차'만 인정하므로
      낡은 기록을 자기 것으로 오독할 여지가 원천적으로 사라진다. 안 넘기면 씬번호로만
      맞춰 보는데, 그때 오독이 가능한 경우가 둘 있다 — /scene POST 자체가 실패해
      새 기록이 안 생겼을 때, 그리고 같은 씬 버튼을 연달아 눌러 '정지 토글'이 됐을 때
      (둘 다 직전 회차 결과를 그대로 돌려준다). 배선할 때 seq를 넘기는 쪽이 맞다.
    """
    t0 = time.monotonic()
    limit = _SCENE_MAX_SEC if timeout is None else timeout
    miss = 0
    while True:
        with _auto_lock:
            if _AUTO.get("force"):
                _AUTO["force"] = False      # 1회용 — 다음 씬으로 새어나가면 안 된다
                return "forced", None
        if _AUTO["cancel"]:
            return "cancel", None
        st = _elev_status(timeout=1.0)
        if st is None:
            miss += 1
            if miss >= _SCENE_MISS_MAX:
                return "noapp", None
        else:
            miss = 0
            res = st.get("scene_result") or {}
            if seq is not None and res.get("seq") != seq:
                return "noanmu", None      # 내가 시킨 회차가 아니다 = 안 떴다
            if res.get("n") != n:
                # 이 씬은 안무를 띄우지 않았다(SCENE_MOVES에 없는 ①③⑤ 자세 전환 씬).
                # /scene 응답 시점에 이미 기록이 세워지므로 '아직 안 생김'은 아니다.
                return "noanmu", None
            if not res.get("running", True):
                return ("done" if res.get("ok") else "fail"), res
        if time.monotonic() - t0 >= limit:
            return "timeout", None
        time.sleep(_SCENE_POLL_SEC)


_FRONT_LOC        = "엘리베이터 문앞"
# ② 는 승차지점에서 80.8cm 다. 0.26m/s 면 회전까지 넣어도 20초대다. 60초는 그 3배이고,
# 동시에 '목적지를 interface 가 모를 때' 를 빨리 드러내는 값이다 — 기본 200초로 두면
# 조용히 3분 넘게 서 있다가 실패한다.
_FRONT_ARRIVE_SEC = 60.0

# ── 바퀴 주인 넘기기 — Nav2 → 엘베앱 (2026-09-11 A1·A2, verifier 재현) ───────────────
# /cmd_vel 에 중재자가 없다(_auto_front_nav2 독스트링). Nav2 가 바퀴를 잡은 채 엘베앱에
# 제어권을 돌려주면 주인이 둘이 된다. 재현된 두 경로:
#   A1 ② 60초 미도착 → Nav2 목표가 살아 있는 채 /authority granted=true → '다음'만 누르면
#      ④ 전진 185cm 까지 Nav2 goal 이 산다.
#   A2 _auto_wait_arrival 은 xy 만 보므로 RotateToGoal 제자리 회전 중에 '도착'이 난다
#      (xy 도착 1.75초 뒤 yaw 46° 남은 채 제어권 이양). 승차지점 첫 부여도 같은 판정을 쓴다.
# '놓았다'의 근거는 pose 가 아니라 **바퀴 명령**이다. amcl_pose 는 update_min_d 0.15 /
# update_min_a 0.1 을 넘어야 갱신돼 '멈춤'과 '못 받음'을 못 가른다. 대시보드는 이미
# /stretch/cmd_vel 을 구독해 0 아닌 명령 시각을 _last_move_cmd 에 둔다(구독 추가 0).
_NAV_QUIET_SEC  = 1.0    # 0 아닌 cmd_vel 이 이만큼 없으면 '바퀴를 놓았다'
# 도착 뒤 Nav2 가 스스로 끝나기를 기다리는 상한 — 회전을 중간에 끊지 않으려고.
# max_vel_theta 0.4 rad/s(nav2_params_human.yaml)로 180° 가 7.9s 다. 넘으면 /cancel.
_NAV_SETTLE_SEC = 10.0
_NAV_CANCEL_SEC = 5.0    # /cancel 뒤 멈추기를 기다리는 상한(컨트롤러는 취소 즉시 0 을 낸다)
# cmd_vel 콜백이 살아 있다는 증거. 스핀 스레드가 굶거나 죽으면 _last_move_cmd 가 멈춰서
# '조용함'으로 **보인다**. /battery 는 드라이버 상태 타이머 30Hz(stretch_driver rate)이고
# 같은 스핀의 _battery_callback 이 받으므로, 그게 5초 넘게 조용하면 '모른다'로 본다.
_SPIN_ALIVE_SEC = 5.0


def _cmd_quiet_wait(quiet_sec, timeout):
    """0 아닌 /stretch/cmd_vel 이 quiet_sec 동안 없을 때까지 대기. (조용해졌나, 사유).
    사유: "quiet" / "moving"(상한까지 명령이 계속 나옴) / "spin_dead"(콜백 생존 증거 없음)."""
    t0 = time.monotonic()
    while True:
        now = time.monotonic()
        if now - (_ready.get("battery") or 0.0) > _SPIN_ALIVE_SEC:
            return False, "spin_dead"
        if now - _last_move_cmd >= quiet_sec:
            return True, "quiet"
        if now - t0 >= timeout:
            return False, "moving"
        time.sleep(0.1)


@_jr_traced("nav_release_wheels", ev="gate")
def _nav_release_wheels(arrived, where):
    """엘베앱에 제어권을 넘기기 **전에** Nav2 가 바퀴를 놓았는지 확인한다. 놓았으면 True.

    False 면 호출부는 제어권을 넘기지 말고 여정을 끊어야 한다 — 두 주체가 같은 바퀴를
    잡는 것보다 멈추는 편이 낫다(fail-closed).
      · 도착했으면 먼저 스스로 끝나기를 기다린다. 정상 경로는 여기서 바로 True 이고
        /cancel 을 보내지 않는다(interface 의 도착 안내·상태 전이를 건드리지 않는다).
      · 미도착이거나 도착 뒤에도 명령이 계속 나오면 interface 에 /cancel 을 보낸다.
        /auto_cancel 과 같은 경로다(_go_locked → nav.stop, 음성 없음). 그 뒤 다시 기다린다."""
    if arrived:
        quiet, why = _cmd_quiet_wait(_NAV_QUIET_SEC, _NAV_SETTLE_SEC)
        if quiet:
            return True
        if why == "spin_dead":
            _log("AUTO", f"🚨 {where}: cmd_vel 콜백 생존 증거 없음(/battery {_SPIN_ALIVE_SEC:.0f}s 넘게 "
                         "조용) — 바퀴를 놓았는지 모른다")
            return False
        _log("AUTO", f"⚠ {where}: 도착 판정 뒤 {_NAV_SETTLE_SEC:.0f}s 동안 Nav2 명령이 계속 나온다 "
                     "— /cancel 로 끊는다")
    sent = _write("iface", "/cancel")
    quiet, why = _cmd_quiet_wait(_NAV_QUIET_SEC, _NAV_CANCEL_SEC)
    _log("AUTO", f"{where}: Nav2 목표 취소 요청 "
                 + ("전송" if sent else "🚨 전송 실패(interface 없음)") + " → "
                 + ("바퀴 멈춤 확인" if quiet else f"🚨 멈춤 확인 실패({why})"))
    return quiet


@_jr_traced("auto_front_nav2")
def _auto_front_nav2():
    """② 문앞 — 3단 안무 대신 Nav2 로 간다. 계속할지(bool) 반환.

    ■ 왜 바꾸나 (2026-09-09 사용자)
      "좌표로 이동하는 게 너무 이상했어… 문 앞에 위치하는 것도 너무 멀리 떨어져있었고"
      9/9 실기 ② 기록: 잔여 dist 0.100m / yaw +2.4° / 회전 13회 · |회전|합 297°.
      Nav2 는 같은 구간을 xy_goal_tolerance 0.03 · yaw_goal_tolerance 0.05(2.9°)로 선다.
      ② 는 엘베 **밖**이고 문을 통과하지 않는다 — 지도 실측으로 현재 파라미터 그대로
      목표·경로·회전이 전부 통과한다(승차지점→문앞 80.8cm 스윕, 필요 회전 168.4°→83.3°).
      **그래서 1주차는 Nav2 파라미터를 하나도 바꾸지 않는다.**
      ④ 엘리베이터 안·⑥ 하차는 Nav2 로 넘기지 않는다 — 뒤 0.9m footprint 꼬리는 사용자가
      "동반자를 장애물로 안 보게" 직접 넣은 설계(footprint_clearing_enabled)라,
      캐빈 선회를 얻으려고 줄이면 **로봇을 잡고 있는 시각장애인이 장애물이 된다.**

    ■ 바퀴를 누가 잡나 — /cmd_vel 에 중재자가 없다
      실측 cmd_vel_pubs=8. Nav2 와 엘베앱이 같은 토픽에 동시에 쓸 수 있고 막는 것이
      없다. 지금까지는 순차 운영(엘베앱 토글)으로 우연히 피해 왔다. ② 를 Nav2 로
      바꾸면 여정 **중간에** Nav2 주행이 처음으로 끼어들므로 규칙을 명시한다:
          Nav2 가 바퀴를 잡는 구간 ⟺ 엘베앱 authority=False
      회수(_revoke_authority)가 _step_abort + Twist() 정지를 하므로 전환은 안전하다.
      대가: guard_off 가 ② 전후로 두 번 더 뒤집힌다. 그래서 엘베앱에 [GUARD] 전이
      로그를 넣었다 — 지금까지 조용히 켜져서 사고 조사 때 안 보였다.

    ■ 합격선 (못 넘으면 되돌린다)
      3단의 9/9 기록(dist 0.100m / yaw +2.4° / 회전 13회 297°)을 **거리와 회전 횟수
      둘 다** 이겨야 한다. 아래에서 도착 잔여를 기준선과 나란히 로그에 남긴다.
      회전 횟수는 Nav2 가 보고하지 않는다 — 영상·[MOVE] 로그로 사람이 센다.
    """
    with _auto_lock:
        _AUTO["force"] = False
    p = _loc(_FRONT_LOC)
    if not p:
        _auto_set("오류", f"'{_FRONT_LOC}' 좌표가 location.yaml 에 없음 — ② 중단")
        return False
    _auto_set("② 문앞정렬", "Nav2 주행 준비 — 엘베앱 제어권 회수 중...",
              wait=False, phase="front")
    # 제어권 회수가 실패하면 주행을 시작하지 않는다. 두 주체가 /cmd_vel 을 동시에
    # 쓰는 것은 사람 옆에서 절대 허용할 수 없다(fail-closed).
    if not _grant_elev_lease(False, "② 문앞 Nav2 주행"):
        _auto_set("오류", "🚨 제어권 회수 실패 — Nav2 와 엘베앱이 /cmd_vel 을 "
                          "동시에 쓸 수 있다. ② 중단")
        return False
    if not _nav_params_restore_normal("② 문앞"):
        # 여기서 거부하면 제어권은 이미 회수된 상태다 — 되돌려 놓고 나간다.
        _grant_elev_lease(True, "② 파라미터 복원 실패 — 엘베 모드 복귀")
        _auto_set("오류", "🚨 Nav2 파라미터를 평소값으로 되돌리지 못했다 — ② 주행 거부")
        return False
    _auto_set("② 문앞정렬", "문 앞으로 Nav2 주행 중... 로봇이 멈추면 '다음'이 열립니다",
              wait=False, phase="front")
    _write("iface", f"/goto {_FRONT_LOC}")
    arrived = _auto_wait_arrival(_FRONT_LOC, timeout=_FRONT_ARRIVE_SEC)
    # 🔴 제어권을 돌려주기 **전에** Nav2 가 바퀴를 놓았는지 확인한다(A1·A2 — _NAV_* 주석).
    #    못 확인하면 제어권을 돌려주지 않는다. 여기서 False 를 돌려주면 _auto_run 이 엘베앱을
    #    끈다 — 팔은 ① 누르기 뒤 수납 확인된 상태(arm_safe)라 앱 종료로 잃는 것은 패드뿐이고,
    #    Nav2 가 아직 바퀴를 잡고 있다면 패드는 어차피 쓸 수 없다.
    if not _nav_release_wheels(arrived, "② 문앞"):
        _auto_notify("로봇이 멈췄는지 확인하지 못해 여정을 멈췄습니다. 도움을 요청하세요")
        _auto_set("오류", "🚨 Nav2 가 바퀴를 놓았는지 확인 못 함 — 엘베앱 제어권을 돌려주지 "
                          "않고 ② 중단", phase="front")
        return False
    # 도착 여부와 무관하게 제어권은 되돌린다 — ③ 문대기·④ 엘리베이터 안이 엘베앱이고,
    # 실패해도 사람이 엘베UI 조종 패드로 수습해야 한다(패드는 제어권이 있어야 듣는다).
    if not _grant_elev_lease(True, "② 문앞 구간 종료 — 엘베 모드 복귀"):
        _auto_set("오류", "제어권 재부여 실패 — 여정 중단")
        return False
    if not arrived:
        # 가장 흔한 원인을 문구에 박는다: interface 는 location.yaml 을 **init 에 한 번만**
        # 읽는다(interface.py:1177). 장소를 새로 추가했으면 interface 를 재시작해야
        # _handle_goto 가 받는다 — 아니면 "목록에 없는 목적지 무시"로 조용히 버린다.
        _auto_set("② 문앞정렬",
                  f"⚠ 문앞 도착 실패(≤{_FRONT_ARRIVE_SEC:.0f}s) — interface 가 "
                  f"'{_FRONT_LOC}' 를 모를 수 있다(장소 추가 후 interface 재시작 필요). "
                  "엘베UI 로 수동 정렬 후 '다음'", wait=True, phase="front")
        return _auto_wait_confirm()
    # 단계 기록만 ② 로 넘긴다 — 안무는 띄우지 않는다(Nav2 가 이미 데려다 놨다).
    # 건너뛰면 [SCENE] 매듭·누적 리셋이 없어 다음 단계 로그가 ① 로 남는다.
    _elev_scene(1, move=False)
    d = _dist_to(p.get("x"), p.get("y"))
    try:
        tgt_yaw = math.degrees(2.0 * math.atan2(float(p.get("z") or 0.0),
                                                float(p.get("w") or 1.0)))
        dy = (_robot_pose["yaw_deg"] - tgt_yaw + 180.0) % 360.0 - 180.0
    except Exception:
        dy = None
    _log("AUTO", "② 문앞 Nav2 도착 — 잔여 위치 "
                 + ("?" if d is None else f"{d:.3f}m")
                 + " 방향 " + ("?" if dy is None else f"{dy:+.1f}°")
                 + "  (3단 기준선 2026-09-09: 0.100m / +2.4° / 회전 13회 297°)")
    _auto_set("② 문앞정렬", "문 앞 도착 ✅ (Nav2) — 잔여 "
                            + ("?" if d is None else f"{d*100:.1f}cm")
                            + ("" if dy is None else f" / {dy:+.1f}°") + " · '다음'",
              wait=True, phase="front")
    return _auto_wait_confirm()


def _scene_res_txt(res):
    """실행 기록의 **잔여값만** 사람이 읽는 한 줄로. 좌표 목표가 없는 씬은 빈 문자열.

    사유(reason)는 일부러 넣지 않는다 — 부르는 쪽이 이미 '완료 ✅' / '⚠ 미달' 같은
    말을 붙이므로 여기서도 붙이면 "완료 — 완료 — 잔여…" 처럼 겹친다. 사유가 필요한
    자리(미달·강제)는 부르는 쪽에서 직접 붙인다."""
    res = res or {}
    if res.get("dist_m") is None:
        return ""
    return (f"잔여 위치 {res['dist_m']:.3f}m 방향 {res['yaw_deg']:+.1f}° "
            f"횡 {res['lat_cm']:+.1f}cm")



_EXIT_WHY = {"stall":   "후진이 멈췄습니다",
             "timeout": "제한 시간을 넘겼습니다",
             "cancel":  "취소되었습니다",
             "noapp":   "엘리베이터 앱 응답이 없습니다",
             "fail":    "목표 자세까지 가지 못했습니다",
             "noanmu":  "하차 동작이 시작되지 않았습니다",
             "forced":  "사람이 대기를 강제로 넘겼습니다 — 하차는 확인되지 않았습니다",
             "post":    "엘리베이터 앱에 하차 명령이 전달되지 않았습니다",
             "noseq":   "하차 동작이 접수되지 않았습니다"}


def _exit_failed(reason: str, cur, res=None):
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
    '완료되지 않았다'가 아니라 '어디까지 나왔나'다. 2026-09-09 부터 목표가 하드코딩
    거리가 아니라 좌표이므로 '목표까지 몇 cm 남았나'(scene_result.dist_m)를 먼저 쓰고,
    그 값이 없을 때(stall·noanmu 등 기록이 안 닫힌 경우)만 누적 진행량으로 적는다.

    TODO(무인 운전): 실전에는 화면을 보는 조작자가 없다 — 로봇을 잡고 있는 사람이
    시각장애인이다. 지금 이 함수의 종착점은 '제어권을 유지한 채 여정 중단'이고,
    그건 조작자가 패드로 빼내 주기를 전제한다. 무인이면 여기서 **음성 안내**가
    나가야 한다(현재 위치·상황·무엇을 해야 하는지). 호출 지점은 아래 _auto_notify
    자리다 — 문구와 TTS 경로는 별건으로 정한다. 이 함수를 최종형으로 보지 마라."""
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
    _d    = (res or {}).get("dist_m")
    # cur 도 dist_m 도 없을 수 있다 — 첫 폴링 전에 강제·취소가 들어오거나, /scene
    # 전송 자체가 실패해 실행 기록이 없는 경우다. 그때 "확인 불가 후진한 상태입니다"
    # 로 문장이 깨지지 않게 갈래를 셋으로 둔다(구조하러 오는 사람이 읽는 문장이다).
    where = (f"목표까지 {_d * 100:.0f}cm 남은 상태" if isinstance(_d, (int, float))
             else f"{moved} 후진한 상태" if isinstance(cur, float)
             else "진행량을 확인할 수 없는 상태")
    why   = _EXIT_WHY.get(reason, reason)
    if reason != "noapp":
        _rescue_hold = True
    # TODO(무인 운전): 아래 _auto_notify 가 음성으로도 나가야 한다. 위 독스트링 참고.
    _auto_notify(f"하차가 끝나지 않았습니다 — {where}입니다. "
                 f"{why}. 로봇이 엘리베이터에 걸쳐 있을 수 있으니 도움을 요청하세요")
    _log("AUTO", f"⑥ 하차 미완료({reason}) {where} (누적 진행 {moved}) — "
                 + ("제어권 유지(구조용 조종 패드 보존) — 엘베앱 종료 시 해제"
                    if reason != "noapp" else "앱 응답 없음 — 리스는 이미 소멸"))
    _auto_set("오류", f"🚨 하차 미완료({reason}) — {where} · 여정 중단"
                      + (" · 제어권 유지, 엘베앱 조종 패드로 빼낼 것"
                         if reason != "noapp" else ""))


def _auto_abort_elev():
    """여정 취소 시 안전 정리: 엘베앱 종료(→제어권 회수→가드 복구)."""
    _auto_set("취소", "여정 취소됨 — 엘베앱 종료·제어권 회수")
    _http_self("/elevator_app", {"running": False})

# ── 여정 활성 중 서버측 잠금 (2026-09-11 advisor §6) ────────────────────────────────
# UI 비활성만으로는 막히지 않는다 — 키보드 단축키(숫자키 5 = 엘베앱 토글), 열어 둔 옛 탭,
# 직접 POST 가 그대로 통과한다. 캐빈 안에서 엘베앱이 꺼지면 조종 패드가 사라지고, ② Nav2 구간에
# 제어권을 켜면 바퀴 주인이 둘이 되고, 장소 클릭은 여정의 Nav2 목표를 가로챈다.
# 여정 스레드 **자신의** 호출(_http_self·_http_self_json → /elevator_app 등)은 막으면 안 된다
# (8단계 엘베앱 종료·취소 정리가 스스로 막힌다). 그래서 내부 호출에만 붙는 헤더 토큰으로
# 가른다. 토큰은 프로세스마다 새로 만든다 — 밖에서 맞출 수 없다.
_JOURNEY_INTERNAL_HDR   = "X-Journey-Internal"
_JOURNEY_INTERNAL_TOKEN = os.urandom(16).hex()


def _journey_gate(what):
    """자동 여정이 도는 동안 여정을 흔드는 조작이면 (응답, 409), 아니면 None.

    여정이 끝나면(_AUTO.active=False) 전부 열린다 — ⑥ 하차 미완료의 구조 경로
    (_rescue_hold: 엘베앱 패드·제어권 토글)는 여정 스레드가 끝난 뒤에 쓰는 것이라 막히지 않는다.
    멈추는 경로(/auto_cancel)는 여기에 걸지 않는다."""
    if not _AUTO.get("active"):
        return None
    if request.headers.get(_JOURNEY_INTERNAL_HDR) == _JOURNEY_INTERNAL_TOKEN:
        return None
    step = _AUTO.get("step", "")
    err = "자동 여정 진행 중 — 취소 후 다시"
    _log("AUTO", f"⛔ 여정 중 조작 거부 — {what} [{step}] ({request.remote_addr})")
    _jr("reject", _AUTO.get("dest"), err, what=what, path=request.path, step=step,
        remote_addr=request.remote_addr)
    return jsonify(ok=False, error=err), 409


@_jr_traced("http_self")
def _http_self(path, payload):
    """대시보드 자기 자신(8080)의 라우트를 호출 (지도전환 등 재사용)."""
    try:
        import urllib.request
        urllib.request.urlopen(urllib.request.Request(
            f"http://localhost:8080{path}",
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json",
                     _JOURNEY_INTERNAL_HDR: _JOURNEY_INTERNAL_TOKEN},   # 여정 자신의 호출
            method="POST"), timeout=10)
        return True
    except Exception as e:
        _log("AUTO", f"{path} 호출 실패: {e}")
        return False


# /switch_map 핸들러의 최악 소요 ≈ 13s = _map_loaded_floor 의 ros2 param get(timeout 5s)
# + load_map 대기(6.0s) + 엘베앱 /floor 통보(timeout 2s). _http_self 의 10s 로는 정상 전환도
# 시간초과로 끊긴다(2026-09-11 verifier A3 재현: 10.05s 'timed out' 뒤 지도 전환은 뒤늦게 성공).
# 시간초과가 '실패'가 아니라 '아직 모름'이 되지 않게 핸들러 최악의 2배 넘게 둔다.
_SWITCH_MAP_TIMEOUT = 30.0


@_jr_traced("http_self_json")
def _http_self_json(path, payload, timeout):
    """_http_self 와 같은 POST 이지만 **응답 본문**을 돌려준다. (본문 dict | None, 오류 | None).

    _http_self 는 HTTP 200 이면 True 라서 {"ok": false} 를 성공으로 넘긴다. 판정이 본문에
    실리는 라우트(/switch_map — "map_server 서비스 없음"·"load_map 실패"가 200 으로 온다)는
    이걸 써야 한다. 4xx/5xx 도 본문을 읽는다(라우트가 사유를 싣는다)."""
    try:
        import urllib.request
        import urllib.error
        try:
            r = urllib.request.urlopen(urllib.request.Request(
                f"http://localhost:8080{path}",
                data=json.dumps(payload).encode(),
                headers={"Content-Type": "application/json",
                         _JOURNEY_INTERNAL_HDR: _JOURNEY_INTERNAL_TOKEN},   # 여정 자신의 호출
                method="POST"), timeout=timeout)
            raw = r.read()
        except urllib.error.HTTPError as he:
            raw = he.read()
        body = json.loads((raw or b"").decode() or "{}")
        return (body if isinstance(body, dict) else None), (None if isinstance(body, dict)
                                                             else "본문이 dict 아님")
    except Exception as e:
        _log("AUTO", f"{path} 호출 실패: {e}")
        return None, repr(e)

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
    _jr("ev", "notify", text=msg, voice=voice, stow_hint=stow_hint)
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


@_jr_traced("press_or_pass")
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
    global _elev_started_mono, _rescue_hold, _floor_confirmed
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
    _AUTO["arm_safe"] = arm_safe   # 블랙박스 미러(run_end.state) — 아래 4곳도 같다. 판정에 안 쓴다
    # 🔴 손잡이가 죽은 채로 여정을 시작하면 **사람이 로봇을 멈출 수단이 없는 상태로**
    #    도는 것이다(당김 = 정지 요청). 지금은 막지 않는다 — 실기가 이 상태로 돌고
    #    있고 막으면 리허설이 통째로 불가능해진다. 대신 **조용히 넘어가지 않는다.**
    #    무인 운전 3단계에서는 이것을 하드 게이트로 승격해야 한다.
    _h = _readiness_signal("handle", "손잡이")
    if _h["status"] != "ok":
        _log("AUTO", f"⚠ 손잡이 신호 없음({_h['detail']}) — 여정 중 당김(정지 요청)을 "
                     "받을 수 없다. 여정은 진행한다(1단계 경고)")
        _auto_notify("손잡이 신호가 없습니다. 여정 중 손잡이를 당겨도 "
                     "정지 요청이 전달되지 않습니다")
    try:
        with _auto_lock:
            _AUTO.update(active=True, dest=dest, cancel=False, force=False,
                         phase="", mode="")
        d = _loc(dest)
        if not d:
            _auto_set("오류", f"'{dest}' 좌표를 location.yaml에서 못 찾음"); return
        dest_floor = str(d.get("floor") or _current_floor)
        with _auto_lock:
            _AUTO["dest_floor"] = dest_floor

        # 🔴 층이 미확정이면 이 아래 판정이 전부 어긋난다 — 같은 층 여부도, ▲▼도.
        #    같은 층 주행은 거부하지 않는다(여기서 막으면 평소 복도 주행이 통째로
        #    멈춘다). 엘베 여정만 거부한다 — 거기서만 틀린 층이 "반대 버튼을 누른다"가
        #    되기 때문이다. 같은 층 쪽은 사실을 로그로 남기고 진행한다.
        if not _floor_confirmed:
            _log("AUTO", f"⚠ 현재 층 미확정(부팅 초기값 {_current_floor}층 그대로) — "
                         f"'{dest}'={dest_floor}층 판정을 그 값으로 한다")
        # 같은 층이면 엘베 없이 바로
        if dest_floor == _current_floor:
            with _auto_lock:
                _AUTO["mode"] = "same"
            _auto_set("주행", f"{dest} 바로 이동 (같은 층)", phase="drive")
            if not _nav_params_restore_normal(f"같은 층 {dest}"):
                _auto_set("오류", "🚨 Nav2 파라미터를 평소값으로 되돌리지 못했다 — "
                                  "주행 거부(좁은 안전거리로 복도를 달릴 수 있다)")
                return
            _write("iface", f"/goto {dest}")
            _auto_set("완료", f"{dest} 도착 ✅" if _auto_wait_arrival(dest) else "도착 실패",
                      phase="done")
            return

        with _auto_lock:
            _AUTO["mode"] = "elevator"
        # 🔴 미확정 상태로 엘베를 타면 안 된다. 1층에서 켜고 4층으로 가면
        #    up = 4 > 5 = False 가 되어 **▼를 누른다**. 정상 여정을 한 번 돌면
        #    하차 시 /switch_map 이 추적하므로 구멍은 '부팅 직후 첫 여정'이다.
        #    5층에서만 켠다는 것은 운영 관행이고 코드 보증이 아니다.
        if not _floor_confirmed:
            _auto_notify("지금 몇 층인지 확인되지 않아 엘리베이터를 쓸 수 없습니다. "
                         "대시보드에서 현재 층을 먼저 선택해 주세요")
            _auto_set("오류", f"🚨 현재 층 미확정 — 엘베 여정 거부 (부팅 초기값 "
                              f"{_current_floor}층 그대로다). 대시보드에서 층을 "
                              "선택하면 풀린다", phase="board")
            return
        up = int(dest_floor) > int(_current_floor)
        dir_txt = "▲ 상행" if up else "▼ 하행"
        _auto_set("시작", f"{dest}={dest_floor}층 / 현재 {_current_floor}층 → 엘베 {dir_txt}",
                  phase="board")

        # 1) 승차지점 주행 (자동)
        _auto_set("주행", "엘리베이터 탑승지점으로 이동 중...", phase="board")
        if not _nav_params_restore_normal("승차지점"):
            _auto_set("오류", "🚨 Nav2 파라미터를 평소값으로 되돌리지 못했다 — "
                              "승차지점 주행 거부")
            return
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
            _jr("proc", "elev", proc.pid, str(THIS_DIR / "elevator_button_press/main.py"))
            threading.Thread(target=_capture, args=(proc, "ELEV"), daemon=True).start()
        if not _wait_elev_app_up(20):
            _auto_set("오류", "엘베앱 기동 실패"); return
        # 🔴 첫 제어권 부여도 같다 — 승차지점 '도착'은 xy 만 본 판정이라 Nav2 가 아직
        #    제자리 회전 중일 수 있다(A2). 확인 못 하면 제어권을 주지 않는다.
        if not _nav_release_wheels(True, "승차지점"):
            _auto_notify("로봇이 멈췄는지 확인하지 못해 여정을 멈췄습니다. 도움을 요청하세요")
            _auto_set("오류", "🚨 Nav2 가 바퀴를 놓았는지 확인 못 함 — 엘베앱 제어권 부여 거부")
            return
        if not _grant_elev_lease(True, "엘베 여정 시작"):
            _auto_set("오류", "제어권 부여 실패 — 여정 중단"); return

        # ── 엘리베이터 6 시나리오 ──────────────────────────────────────────
        #   버튼 선택(상/하행·층)·정렬 = 자동 / 누르기(press) = '다음'에 포함.
        #   씬: 0=①호출 1=②문앞 2=③문열림 3=④엘베안 4=⑤층 5=⑥하차 (SCENES와 1:1)
        #   🔴 ④ 는 **캐빈 안**이다. location.yaml 의 `엘리베이터 탑승지점`(복도 쪽
        #      호출 지점)과 다른 장소다 — 예전에 둘 다 "탑승"이라 불러 섞였다.

        # ① 호출 press: 인식자세 → 호출버튼(상/하행) 자동선택 → 정렬 → '다음'에 누르기
        _auto_set("① 호출", f"인식 자세 + {dir_txt} 버튼 자동 선택·정렬 중...", phase="call")
        # 인식자세 자체는 lift·손목만 잡고 arm_extension은 건드리지 않는다.
        # 팔이 실제로 뻗는 것은 그 뒤 자동 접근·서보와 누르기 시퀀스다 —
        # 여기서 내려두는 건 "이 지점부터는 수납을 장담 못 한다"는 뜻이다.
        arm_safe = False
        _AUTO["arm_safe"] = arm_safe
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
        _AUTO["arm_safe"] = arm_safe

        # ② 문앞 정렬 — 2026-09-09: 3단 안무(전진 80.8 + 우회전 90°) → Nav2 주행.
        # 팔 수납 확인은 그대로 선행한다(⑥ 직전과 같은 게이트). Nav2 든 3단이든
        # 뻗은 팔로 문틀 옆을 지나면 부딪힌다.
        if not arm_safe:
            _auto_notify("팔이 안전한지 확인되지 않아 이동을 멈췄습니다", stow_hint=True)
            _auto_set("오류", "팔 복귀 미확인 — 베이스 이동 거부(② 문앞정렬)")
            _auto_abort_elev(); return
        if not _auto_front_nav2(): _auto_abort_elev(); return

        # ③ 문 열림 대기 (자동 감지)
        # ③은 SCENE_MOVES 에 없어 안무가 없다 → noanmu 로 즉시 '다음'이 열린다.
        ok_, _why, _res = _auto_scene_step(
            "③ 문열림", 2, "문 열림 감시 준비 중...",
            "문 열림 대기 중... 문 열리면 '다음'", "door")
        if not ok_: _auto_abort_elev(); return

        # ④ 엘리베이터 안: 전진 185 (자동 안무)
        if not arm_safe:
            _auto_notify("팔이 안전한지 확인되지 않아 이동을 멈췄습니다", stow_hint=True)
            _auto_set("오류", "팔 복귀 미확인 — 베이스 이동 거부(④ 엘리베이터 안)")
            _auto_abort_elev(); return
        ok_, _why, _res = _auto_scene_step(
            "④ 엘리베이터 안", 3,
            "엘리베이터 안으로 전진 중... 로봇이 멈추면 '다음'이 열립니다",
            "④ 엘리베이터 안 — 다 탔으면 '다음'", "ride")
        if not ok_: _auto_abort_elev(); return

        # ⑤ 층 press: 인식자세 → 목적층 자동선택 → 정렬 → '다음'에 누르기
        _auto_set("⑤ 층선택", f"인식 자세 + {dest_floor}층 버튼 자동 선택·정렬 중...",
                  phase="floor")
        arm_safe = False                        # 씬0과 같은 이유(위 주석 참고)
        _AUTO["arm_safe"] = arm_safe
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
        _AUTO["arm_safe"] = arm_safe

        # 엘베 이동 대기 → ⑥ 하차: 후진 186 (자동 안무)
        _auto_set("이동중", f"{dest_floor}층 이동 중 — 도착·하차 준비되면 '다음'",
                  wait=True, phase="moving")
        if not _auto_wait_confirm(): _auto_abort_elev(); return
        if not arm_safe:
            _auto_notify("팔이 안전한지 확인되지 않아 이동을 멈췄습니다", stow_hint=True)
            _auto_set("오류", "팔 복귀 미확인 — 베이스 이동 거부(⑥ 하차)")
            _auto_abort_elev(); return
        _auto_set("⑥ 하차", "하차(후진) 중...", phase="exit")
        # seq 를 받아 넘긴다 — 내가 시킨 그 회차의 기록만 인정하기 위해서다.
        # 🔴 반환 둘을 **모두** 확인한다. _elev_scene 독스트링이 "bool 하나면 '전송
        # 실패'와 '안무 없음'을 못 가른다. 그래서 튜플이다"라고 적어 둔 그 구분을
        # 여기서 버리면 양방향으로 틀린다:
        #   · seq=None 로 폴링에 들어가면 첫 검사(seq 비교)가 통째로 건너뛰어지고
        #     n 비교만 남는다. ⑤ press 는 SCENE_MOVES 에 없어 ⑥ 직전 마지막 기록은
        #     항상 ④(n=3) → 즉시 noanmu → **후진 중인 로봇을 세운다.**
        #   · 더 나쁜 쪽: 같은 앱 세션에서 ⑥을 한 번 성공시킨 뒤 다시 돌면 기록이
        #     n=5/running=False/ok=True 로 남아 **후진 0cm 로 done 이 난다.** 그 뒤
        #     리스 반납 → switch_map → 앱 종료(구조 패드 소멸) → /goto 가
        #     사람을 태운 채 돈다. #92 그 자체다.
        # ※ POST 실패(_sent5=False)는 "안 떴다"와 "떴는데 응답만 늦다"를 **가르지
        #   못한다** — /scene 라우트가 자세 전환을 블로킹으로 끝낸 뒤에야
        #   _scene_run_begin 을 부르므로 타임아웃이 두 경우에 다 걸린다. 그래서
        #   추측하지 않고 _exit_failed 로 보낸다: 거기서 /step_stop 이 먼저 나가
        #   혹시 움직이고 있었다면 멈춘다.
        _sent5, _seq5 = _elev_scene(5)
        if not _sent5 or _seq5 is None:
            _exit_failed("post" if not _sent5 else "noseq", None, None)
            return
        # 상수 대기로 넘기면(#92) 후진 186cm가 10초 넘게 걸리는 동안 리스가 반납돼
        # 이동이 통째로 거부되고, 그 실패가 여기로 전파될 길이 없어 사람을 태운 채
        # 캐빈/문턱에서 /switch_map·/goto로 넘어간다. 씬 ①②③④와 ⑥직전이 전부
        # 확인 대기를 거치는데 이 한 자리만 상수였다.
        ex_reason, ex_res, ex_cm = _elev_wait_exit_done(_seq5)
        if ex_reason == "fallback":
            # 좌표가 거부돼 하드코딩(-186cm)으로 물러섰지만 그 후진은 완주했다 =
            # 엘리베이터에서는 나왔다. 목표 자세는 아니므로 경고는 하되 구조 절차는
            # 부르지 않는다 — 나온 로봇을 두고 "도움을 요청하세요"는 오경보다.
            # ⚠ 예전에는 바로 뒤 /switch_map{init_exit} 가 AMCL 을 하차지점으로 재초기화해
            #   폴백을 유발한 측위 오류를 그 단계에서 교정했다. 2026-09-11 그 초기화를 껐으므로
            #   (아래 7단계 주석) 이제 교정되지 않는다 — 목적지 주행이 그 측위를 이어받는다.
            # ex_cm 은 None 일 수 있다(첫 폴링 전에 끝난 경우). 포맷에서 터지면
            # except 로 떨어져 _rescue_hold=False 로 리스가 반납된다 — 도달 가능성은
            # 사실상 0 이지만 결과가 나쁘므로 방어한다.
            _log("AUTO", "⑥ 하차 — 좌표 경로 거부 → 하드코딩 폴백으로 탈출 완주 "
                         + ("(누적 확인 불가)" if not isinstance(ex_cm, float)
                            else f"(누적 {ex_cm:+.1f}cm)")
                         + f". 사유: {(ex_res or {}).get('reason')}"
                         # ⚠ fallback_ok 의 의미가 약하다 — 별건으로 고친다.
                         # _run_scene_moves_legacy 는 _step_abort 와 '스텝이 0.3cm 도
                         # 안 움직임' 둘에서만 ok=False 다. 진동 break 도, 잔여가 남은
                         # 채 끝나도 ok=True 이고, ⑥은 guard_off=True 라 가드 분기도
                         # 죽는다. 즉 fallback_ok=True 는 "엘베에서 나왔다"가 아니라
                         # "명령을 다 쐈고 odom 이 조금 늘었다"에 가깝다. 그런데 이
                         # 값 하나로 구조 절차를 건너뛰고 지도전환·앱종료·/goto 로
                         # 직행한다. 다음 회차에 '나왔는지'의 직접 증거로 바꿔야 한다.
                         )
            _auto_notify("엘리베이터에서 나왔습니다. 정확한 자세는 아니니 "
                         "다음 주행 시작 위치를 확인하세요")
        elif ex_reason != "done":
            # 지도전환·AMCL 초기화·앱 종료·목적지 주행은 하나도 실행하지 않는다.
            _exit_failed(ex_reason, ex_cm, ex_res)
            return

        # 리스 반납 — 지도전환·AMCL초기화·목적지주행은 가드(라이다 충돌가드)가
        # 켜진 상태로 시작해야 함. 앱 종료(8단계)까지 리스를 끌고 가지 않는다.
        if not _grant_elev_lease(False, "하차 완료"):
            _auto_abort_elev()   # 순서 중요: abort가 먼저(문구 "취소" 세팅) →
                                  # 아래 _auto_set("오류",...)로 덮어써야 UI에 사고원인이 남음
            _auto_set("오류", "🚨 엘베 가드 미복원 — 여정 중단(앱 강제종료)")
            return

        # 7) 지도 전환 (자동) — 하차지점 AMCL 초기화는 하지 않는다(2026-09-11 사용자 결정).
        #    사용자: "하차지점 초기화는 끄자. 이건 안 쓰는 기능 같아"
        #    · `엘리베이터 하차지점` 좌표가 `엘리베이터 문앞` 과 29.22cm / 8.00° 어긋나 있어
        #      (FINDINGS #182/#196) 켜 두면 매 회차 틀린 pose 를 AMCL 에 심는다.
        #    · 전 층 yaml 이 같은 지도(all.pgm)라 AMCL 은 캐빈을 오가며 계속 추적한다.
        #    대가: 캐빈 안에서 흔들린 AMCL 을 목적지 주행이 그대로 이어받는다(캐빈 odom 대비
        #    AMCL 28cm 차이 실측). 백엔드 init_exit·_amcl_init_exit 는 남겨 둔다 — 여정과 UI 의
        #    호출처는 이 변경으로 0 이 됐다.
        _auto_set("지도전환", f"{dest_floor}층 지도 전환", phase="exit")
        # 🔴 결과를 본다(2026-09-11 verifier S3 재현). 예전에는 반환을 버렸다 — ok=False 가
        #    HTTP 200 으로 와도, 10s 시간초과가 나도 그대로 /goto 로 갔다. 그러면 틀린 층에서
        #    "🎉 도착"을 선언하고(전 층이 같은 지도 all.pgm 이라 AMCL 이 반박하지 않는다),
        #    _current_floor·확정이 옛 층에 남아 **다음 여정의 ▲▼ 판정이 확정 플래그를 통과한다.**
        #    확인 근거는 **우리가 보낸 이 POST 의 성공 응답**뿐이다. GET /switch_map 의
        #    loaded_floor 는 근거가 못 된다 — 층 지도가 전부 같은 이미지라 틀린 층도 일치로 보인다.
        #    시간초과·예외·본문 없음은 '모름'이고, 모르면 확정을 내린다. (핸들러가 시간초과 뒤에
        #    끝나 확정을 다시 올리는 경우는 load_map 이 실제로 성공했을 때뿐이라 거짓 확정이 아니다.
        #    여정은 어느 쪽이든 아래에서 목적지 주행 전에 멈춘다.)
        _sw_body, _sw_err = _http_self_json("/switch_map", {"floor": dest_floor},
                                            _SWITCH_MAP_TIMEOUT)
        _sw_ok = bool(_sw_body) and _sw_body.get("ok") is True \
            and str(_sw_body.get("floor")) == str(dest_floor)
        _sw_why = ""
        if not _sw_ok:
            _floor_confirmed = False
            _sw_why = (_sw_body or {}).get("error") or _sw_err or f"응답 이상 {_sw_body}"
            _log("AUTO", f"🚨 {dest_floor}층 지도 전환을 확인하지 못했다 — {_sw_why}. 현재 층 확정 "
                         f"해제(_current_floor={_current_floor} 는 그대로, 확정=False)")
        time.sleep(2)

        # 8) 엘베앱 OFF (제어권 회수 + 종료)
        _auto_set("엘베종료", "엘리베이터 앱 종료 + 제어권 회수", phase="exit")
        _http_self("/elevator_app", {"running": False})
        time.sleep(1)

        if not _sw_ok:
            # 층을 모르는 채 목적지로 가지 않는다. 앱 종료(8)까지는 정상 경로와 같게 두었다 —
            # 리스는 이미 반납했고 로봇은 엘리베이터 밖이라, 앱을 남겨 둘 이유가 없다.
            # 음성은 새로 만들지 않는다 — 같은 단계(하차 뒤 목적지 출발 불가)의 기존 문구를 쓴다.
            _auto_notify("엘리베이터에서 나왔지만 목적지로 출발할 수 없습니다. "
                         "도움을 요청하세요")
            _auto_set("오류", f"🚨 {dest_floor}층 지도 전환 확인 실패({_sw_why}) — 층 확정 해제, "
                              "목적지 주행 거부(하차는 완료). 대시보드에서 현재 층을 다시 "
                              "선택하세요", phase="drive")
            return

        # 9) 목적지 주행 (자동)
        _auto_set("주행", f"{dest}로 이동 중...", phase="drive")
        if not _nav_params_restore_normal(f"하차 후 {dest}"):
            # 하차는 끝났으니 사람은 엘리베이터 밖이다. 목적지까지는 못 간다 —
            # 조용히 서 있지 않고 사실대로 알린다(뿌리 B: 실패가 사용자에 전달 안 됨).
            _auto_notify("엘리베이터에서 나왔지만 목적지로 출발할 수 없습니다. "
                         "도움을 요청하세요")
            _auto_set("오류", "🚨 Nav2 파라미터 복원 실패 — 목적지 주행 거부 "
                              "(하차는 완료)", phase="drive")
            return
        _write("iface", f"/goto {dest}")
        _auto_set("완료", f"🎉 {dest} 도착! 여정 완료" if _auto_wait_arrival(dest)
                  else "목적지 도착 실패", phase="done")
    except Exception as e:
        _jr("exception")                        # traceback 전문 — repr 만으로는 줄번호가 없다
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
        _jr_end()     # 블랙박스 run_end — finally 맨 끝(리스 반납 결과까지 담는다)


# ── 여정 블랙박스 훅 보조 ─────────────────────────────────────────────────────────
# 전부 **메모리 값만** 읽고 스스로 예외를 삼킨다(여정 스레드에서 불린다).
# 파일·/proc·pgrep 이 드는 것은 deferred 로 writer 스레드에 넘긴다.

def _jr_pose():
    """pose 복사본 + 신선도. amcl_pose 는 정지 중 갱신되지 않는 게 정상이라 pose_age 하나로는
    '멈춤'과 '측위 사망'을 못 가른다 → 마지막 0 아닌 cmd_vel 경과를 나란히 둔다
    ("명령은 나가는데 pose 가 안 온다"만이 측위 사망 시그니처다)."""
    try:
        now = time.monotonic()
        pa = _ready.get("amcl") or 0.0
        return dict(_robot_pose,
                    pose_age_s=(round(now - pa, 1) if pa > 0 else None),
                    cmd_age_s=(round(now - _last_move_cmd, 1) if _last_move_cmd > 0 else None))
    except Exception:
        return None


def _jr_sample():
    """1Hz 샘플 — 여정 스레드가 아니라 샘플러 스레드에서 불린다."""
    return {"pose": _jr_pose(), "step": _AUTO.get("step"), "phase": _AUTO.get("phase"),
            "waiting": _AUTO.get("waiting"), "floor": _current_floor,
            "lease_held": _elev_lease_held}


def _jr_procs():
    """writer 스레드에서 — 세 프로세스가 **실제로 로드한** 코드의 정체(/proc·mtime·md5).
    엘베앱은 이미 떠 있으면 재사용되므로(아래 _auto_run) pid 로 기동 시각을 따로 잰다."""
    out = {"dash": _jlog.proc_identity(os.getpid(), __file__, at_import=_DASH_SRC)}
    ip = _procs.get("iface")
    out["iface"] = _jlog.proc_identity(
        getattr(ip, "pid", None) if (ip is not None and ip.poll() is None) else None,
        THIS_DIR / "interface.py")
    ep = _procs.get("elevator")
    if ep is not None and ep.poll() is None:
        pids, via = [ep.pid], "popen"
    else:
        pids, via = _elev_app_pids(), "proc_scan"
    out["elev"] = _jlog.proc_identity(pids[0] if pids else None,
                                      THIS_DIR / "elevator_button_press/main.py",
                                      found=len(pids), via=via)
    return out


def _jr_begin(dest, remote_addr=None):
    """run_start — /auto_goto 가 여정 스레드를 띄우기 **직전**(요청 스레드)."""
    if _JR is None:
        return
    try:
        _JR.begin(dest, {
            "remote_addr": remote_addr, "cur_floor": _current_floor,
            "floor_confirmed": _floor_confirmed, "loaded_map_path": _loaded_map_path,
            "manual_mode": _manual_mode, "lease_held": _elev_lease_held,
            "rescue_hold": _rescue_hold, "battery": dict(_battery), "pose": _jr_pose(),
            "readiness": {"amcl": _readiness_amcl(),
                          "battery": _readiness_signal("battery", "배터리"),
                          "handle": _readiness_signal("handle", "손잡이"),
                          "nav2": _readiness_polled("nav2", "nav2"),
                          "gripper_camera": _readiness_polled("gripper_camera", "그리퍼캠"),
                          "elev_app": _readiness_polled("elev_app", "엘베앱")},
        }, deferred={
            "dest_floor": lambda: (_loc(dest) or {}).get("floor"),   # yaml 읽기 — writer 에서
            "procs": _jr_procs,
        }, sample_fn=_jr_sample)
    except Exception:
        pass


def _jr_end():
    """run_end — `_auto_run` finally 맨 끝. 물리 상태는 '실패한 단계부터 재개'의 입력이 된다."""
    if _JR is None:
        return
    try:
        _JR.end({"arm_safe": _AUTO.get("arm_safe"), "lease_held": _elev_lease_held,
                 "rescue_hold": _rescue_hold, "cancel_flag": bool(_AUTO.get("cancel")),
                 "mode": _AUTO.get("mode"), "dest_floor": _AUTO.get("dest_floor"),
                 "floor": _current_floor, "floor_confirmed": _floor_confirmed,
                 "pose": _jr_pose()},
                deferred={"elev_running": _elev_app_running})   # pgrep — writer 에서
    except Exception:
        pass


def _jr_gate_arrival(name, tx, ty, t0, stable_since, tol, timeout, why, result):
    """_auto_wait_arrival 의 판정 — 마지막 거리·정지 지속·경과·pose 신선도."""
    if _JR is None:
        return
    try:
        now = time.monotonic()
        _JR.gate("arrival", inputs={"name": name, "x": tx, "y": ty, "tol": tol,
                                    "timeout": timeout},
                 result=result, why=why,
                 d_last=(_dist_to(tx, ty) if tx is not None else None),
                 settled_s=(round(now - stable_since, 2) if stable_since else None),
                 elapsed_s=(round(now - t0, 1) if t0 else None), pose=_jr_pose())
    except Exception:
        pass


def _jr_confirm_id(step, msg):
    """'다음' 자리 식별자 — 리포트가 자리별로 묶는 키
    (wip/20260911-confirm-points-inventory.md 의 C1~C9). `_auto_set(..., wait=True)` 의 단계
    이름·문구로 가른다. 문구를 바꾸면 여기도 같이 볼 것 — 못 가르면 'C?_<단계>' 로 남아
    리포트에서 바로 보인다."""
    m = msg or ""
    if step in ("③ 문열림", "④ 엘리베이터 안") and "전송 실패" in m:
        return "C5_scene_post_fail"
    if step == "② 문앞정렬":
        return "C3_front_arrive_fail" if "도착 실패" in m else "C4_front_arrived"
    return {"① 호출": "C1_call_press", "재시도": "C2_press_retry_manual",
            "③ 문열림": "C6_door_open", "④ 엘리베이터 안": "C7_ride_in",
            "⑤ 층선택": "C8_floor_press", "이동중": "C9_moving_exit"}.get(step, f"C?_{step}")


if _JR is not None:
    _JR.pose_fn = _jr_pose
    _JR.confirm_id_fn = _jr_confirm_id


@app.after_request
def _jr_after_request(resp):
    """블랙박스 — 응답을 **읽기만** 한다. 무엇이 터져도 resp 를 그대로 돌려준다.
    두 가지를 잡는다(2026-09-11 verifier):
      · /auto_goto 거부 사유 — 지금은 화면 토스트로만 사라진다(여정 파일이 안 생기는 실패).
      · /switch_map 응답 본문 — ok=False 가 HTTP 200 으로 와서 `_http_self` 가 True 로 넘긴다.
        여정의 하차 후 지도 전환이 그 반환을 보지 않는다. 여기서는 **기록만** 한다."""
    try:
        if _JR is not None and request.method == "POST":
            if request.path == "/switch_map":
                _JR.gate("switch_map_resp", status=resp.status_code,
                         body=resp.get_json(silent=True), req=request.get_json(silent=True))
            elif request.path == "/auto_goto":
                body = resp.get_json(silent=True) or {}
                if not body.get("ok"):
                    _JR.reject((request.get_json(silent=True) or {}).get("dest"),
                               body.get("error"), status=resp.status_code,
                               remote_addr=request.remote_addr)
    except Exception:
        pass
    return resp

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
    _jr_begin(dest, request.remote_addr)     # 블랙박스 run_start — 스레드 시작 직전
    threading.Thread(target=_auto_run, args=(dest,), daemon=True).start()
    return jsonify(ok=True, dest=dest)

@app.route("/auto_confirm", methods=["POST"])
def auto_confirm():
    """블로커 단계 '다음 확인' — 대기 해제.

    안무가 도는 동안(waiting=False)에는 아무 일도 하지 않는다. UI가 버튼을 비활성으로
    두지만 그건 화면일 뿐이고, 여기서도 막아야 잠금이 진짜 잠금이 된다.
    그때 진행하려면 /auto_force 를 써야 한다 — 그쪽은 로봇을 먼저 멈춘다."""
    with _auto_lock:
        if _AUTO.get("active") and not _AUTO.get("waiting"):
            step = _AUTO.get("step", "")
            locked = True
        else:
            step = _AUTO.get("step", "")      # 블랙박스·로그용 — 판정에 안 쓴다
            _AUTO["waiting"] = False
            locked = False
    if locked:
        _log("AUTO", f"'다음 확인' 무시 — [{step}] 동작 중이라 잠겨 있다 "
                     "(그래도 넘어가려면 '강제로 넘어가기')")
        _jr("human", "confirm_locked", step=step, remote_addr=request.remote_addr)
        return jsonify(ok=False, error="동작 중 — 잠김"), 409
    if _AUTO.get("active"):
        # 받아들인 클릭도 남긴다 — 예전엔 잠겨서 무시된 클릭만 로그가 있어 "누가 언제 눌러
        # 넘어갔나"가 없었다(2026-09-11 advisor). 대기 시간은 블랙박스가 잰다.
        _w = _jr("human", "confirm", step=step, remote_addr=request.remote_addr)
        _log("AUTO", f"'다음 확인' 수락 — [{step}]"
                     + (f" (대기 {_w:.0f}s)" if isinstance(_w, (int, float)) else ""))
    return jsonify(ok=True)


@app.route("/auto_force", methods=["POST"])
def auto_force():
    """'강제로 넘어가기' — 진행 중인 안무를 **먼저 멈추고**, 그다음 대기를 푼다.

    순서를 바꾸면 안 된다. 로봇이 도는 채로 다음 씬으로 넘어가면, 이 잠금이 막으려던
    바로 그 상황이 된다. 정지가 먼저, 진행이 나중이다.
    """
    with _auto_lock:
        if not _AUTO.get("active"):
            return jsonify(ok=False, error="자동 여정이 진행 중이 아닙니다"), 409
        step = _AUTO.get("step", "")
    # 1) 정지가 먼저 (_step_abort → 안무 스레드와 현재 스텝 중단)
    stopped = _elev_post("/step_stop", {}, timeout=3) is not None
    # 2) 무엇을 어떤 상태에서 강제했는지 남긴다 — 다음 로그 판독에 이게 필요하다
    res = (_elev_status(timeout=1.5) or {}).get("scene_result") or {}
    where = (f"씬{res.get('n')} " + ("실행 중" if res.get("running") else
             ("완료" if res.get("ok") else f"미달·중단({res.get('reason')})"))
             + (f" — {_scene_res_txt(res)}" if _scene_res_txt(res) else "")
             ) if res else "엘베앱 상태 조회 실패"
    _log("AUTO", f"⏭ 강제로 넘어가기 — [{step}] 에서 사람이 눌렀다. "
                 + ("베이스 정지 요청 전송됨" if stopped
                    else "🚨 정지 요청 전송 실패 — 로봇이 계속 움직일 수 있다")
                 + f" / 그 시점: {where}")
    _jr("human", "force", step=step, stopped=stopped, where=where,
        remote_addr=request.remote_addr)
    # 3) 그다음에 진행
    with _auto_lock:
        _AUTO["force"] = True
        _AUTO["waiting"] = False
    return jsonify(ok=True, stopped=stopped)

@app.route("/auto_cancel", methods=["POST"])
def auto_cancel():
    with _auto_lock:
        _AUTO["cancel"] = True; _AUTO["waiting"] = False
    _jr("human", "cancel", step=_AUTO.get("step"), remote_addr=request.remote_addr)
    _write("iface", "/cancel")
    return jsonify(ok=True)

@app.route("/auto_status")
def auto_status():
    with _auto_lock:
        return jsonify(**{k: _AUTO[k] for k in
                          ("active", "dest", "step", "msg", "waiting",
                           "force", "phase", "dest_floor", "mode")})

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
    _g = _journey_gate('장소 클릭 주행')
    if _g:
        return _g
    name = ((request.json or {}).get("name") or "").strip()
    if not name:
        return jsonify(ok=False, error="이름 없음"), 400
    if _manual_mode:
        return jsonify(ok=False, error="수동 모드에서는 불가 — 자동 모드로 전환하세요")
    if not _nav_params_restore_normal(f"수동 /goto {name}"):
        return jsonify(ok=False, error="Nav2 파라미터를 평소값으로 되돌리지 못했습니다 "
                                       "— 주행을 거부했습니다(로그 확인)"), 409
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

# 런치 경로를 여기에 다시 적지 않는다 — _SYS_PROC_DEFS의 cmd에서 뽑는다.
# 두 곳에 적으면 한쪽만 바뀌었을 때 탐지가 '조용히' 죽고 이중 기동 방지가 통째로
# 사라진다(무음 실패라 미러 드리프트 중에서도 나쁜 쪽이다).
# 뽑기에 실패하면 빈 문자열이 되어 "" in cmdline 이 항상 참이 되는 참사가 나므로,
# 실패를 플래그로 남기고 _sys_start_gate가 fail-closed로 거부한다.
import re as _re_sys
_m_launch = _re_sys.search(r"/\S+\.launch\.xml", _SYS_PROC_DEFS["launch"]["cmd"][-1])
_LAUNCH_XML = _m_launch.group(0) if _m_launch else "\0"   # 절대 매치되지 않는 값
_LAUNCH_XML_OK = _m_launch is not None

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
        # [전제] 규칙이 인자 '위치'에 묶여 있다 — python3 -u .../ros2 launch 나
        # stdbuf 래핑처럼 토큰이 하나만 밀리면 못 잡는다. 지금 성립하는 이유:
        # /opt/ros/humble/bin/ros2가 shebang #!/usr/bin/python3 스크립트라
        # exec ros2 launch ... 가 커널에서 python3 .../ros2 launch ... 로 펼쳐지고,
        # 그래서 대시보드가 띄우는 형태와 사람이 터미널에서 치는 형태가 같은 argv가
        # 된다. 이 전제가 깨지면 탐지가 무음으로 죽는다.
        # [예정] colcon 패키지로 전환하면 패키지 상대 형태
        # (ros2 launch blind_nav_system ...)도 같이 봐야 한다. 지금은 colcon이
        # 아니라 절대경로가 유일한 실행 형태다.
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
    if name == "launch" and not _LAUNCH_XML_OK:
        return ("런치 경로를 확인하지 못해 시작하지 않습니다 — 탐지가 불가능한 상태라 "
                "이중 기동을 막을 수 없습니다"), 409
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


def _pgid_members(pgid: int) -> list:
    """그 프로세스 그룹에 아직 살아 있는(좀비 아닌) PID 들. /proc 순회다.

    외부에서 인수한 프로세스는 우리 자식이 아니라 `Popen.wait()` 이 없다. 그래서
    "다 죽었나"를 이걸로 센다. pgrep 을 쓰지 않는 이유는 _scan_sys_ext 주석과 같다.
    """
    out = []
    try:
        for d in os.listdir("/proc"):
            if not d.isdigit():
                continue
            pid = int(d)
            try:
                if os.getpgid(pid) == pgid and not _proc_zombie(pid):
                    out.append(pid)
            except Exception:
                continue      # 그새 죽음 — 정상
    except Exception:
        return []
    return out


def _wait_pgid_gone(pgid: int, timeout: float):
    """그룹이 비워질 때까지 대기. 시간이 차면 `subprocess.TimeoutExpired` 를 던진다.

    예외 형태를 `Popen.wait` 과 **일부러 똑같이** 맞췄다 — 그래야 종료 순서를 한 번만
    적고 대기 방법만 바꿔 끼울 수 있다. 두 번 적으면 한쪽만 고치는 드리프트가 난다
    (이 파일의 `_LAUNCH_XML` 주석이 경계하는 그 실패 모양이다).
    """
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if not _pgid_members(pgid):
            return
        time.sleep(0.3)
    raise subprocess.TimeoutExpired(f"pgid {pgid}", timeout)


def _pgid_stop_seq(pid: int, label: str, kill_timeout: int, waiter, owned: str) -> bool:
    """**종료 순서의 단일 구현.** SIGINT → kill_timeout 대기 → SIGKILL. 정상 종료면 True.

    `waiter(timeout)` 가 종료를 기다리고 시간이 차면 TimeoutExpired 를 던진다
    (내 자식이면 `proc.wait`, 인수한 외부 프로세스면 `_wait_pgid_gone`).
    `owned` 는 로그 문구용 — "내가 띄운 것"과 "인수한 것"을 사람이 구분해야 한다.

    SIGINT 를 쓰는 이유(SIGTERM 금지)는 _kill_proc_group 독스트링에 있다.
    """
    pgid = os.getpgid(pid)
    # 자기 그룹엔 절대 보내지 않는다 — 대시보드가 자살한다.
    if pgid in (0, 1) or pgid == os.getpgid(0):
        _log("SYS", f"{label} 종료 거부 — PGID {pgid} 가 대시보드 자신의 그룹이거나 비정상")
        return False
    try:
        with open(f"/proc/{pid}/comm") as f:
            pname = f.read().strip()
    except Exception:
        pname = "?"
    _log("SYS", f"{label} SIGINT → PID={pid}({pname}) PGID={pgid} "
                f"[{owned}] (최대 {kill_timeout}s 대기)")
    os.killpg(pgid, signal.SIGINT)
    try:
        t0 = time.monotonic()
        waiter(kill_timeout)
        _log("SYS", f"{label} 정상 종료 ({time.monotonic() - t0:.1f}s) [{owned}]")
        return True
    except subprocess.TimeoutExpired:
        _log("SYS", f"{label} {kill_timeout}s 초과 → SIGKILL 강제 종료 [{owned}]")
        os.killpg(pgid, signal.SIGKILL)
        try:
            waiter(3)
        except Exception:
            pass
        return False


def _kill_ext_pgid(pid: int, label: str, kill_timeout: int = 6) -> bool:
    """**다른 대시보드가 띄워 고아가 된** 런치/RViz 를 인수해서 끈다.

    왜 필요한가 — 대시보드는 `Popen(start_new_session=True)` 로 런치를 띄우므로,
    대시보드 창을 X 로 닫으면 **런치는 모터·라이다·카메라를 돌린 채 살아남는다.**
    새로 띄운 대시보드는 `_sys_procs` 가 비어 있어 그것을 "내 것"으로 보지 못하고,
    그러면 **끌 수단이 사라진다.** 2026-09-10 에 사용자가 실제로 그 상태에 빠졌고
    (`로봇이 도는데 끌 방법이 없다`) 사람이 터미널에서 손으로 killpg 해야 했다.
    물리 안전 문제라서, 소유권이 없어도 끄는 것은 허용한다.

    🔴 **자동으로는 절대 죽이지 않는다.** 기동·폴링은 표시만 하고, 이 함수는 사람이
       버튼을 누른 경로에서만 불린다. 대시보드를 켜는 것만으로 로봇이 멈추면 그게 더
       위험하다.
    🔴 남의 프로세스는 안 건드린다 — 대상은 `_scan_sys_ext` 가 **우리 런치 파일 경로·
       우리 rviz 바이너리**를 argv 에서 확인한 것뿐이다(pgrep 금지, argv 판별).
    종료 순서는 내 자식과 **같은 구현**을 쓴다(`_pgid_stop_seq`).
    """
    if not _proc_alive(pid):
        _log("SYS", f"{label} 인수 종료 — 이미 사라졌다 (PID {pid})")
        return True
    try:
        pgid = os.getpgid(pid)
        return _pgid_stop_seq(pid, label, kill_timeout,
                              lambda t: _wait_pgid_gone(pgid, t), "인수(외부)")
    except (ProcessLookupError, OSError) as e:
        _log("SYS", f"{label} 인수 종료 실패 — {e!r}")
        return False


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
        # 순서는 _pgid_stop_seq 한 곳에만 있다. 여기서는 **대기 방법**만 준다 —
        # proc.wait 는 자식을 reap 까지 하므로 내 자식에는 그게 맞다.
        return _pgid_stop_seq(proc.pid, label, kill_timeout,
                              lambda t: proc.wait(timeout=t), "내가 띄움")
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
        # 내가 띄운 게 아니면 여기서 끝낸다. p가 None인 채로 내려가면
        # _kill_proc_group(None, ...)이 곧바로 True를 돌려주고, 그 뒤 pgrep으로
        # 잡힌 '외부 런치의' rplidar에 SIGKILL + 모터 정지까지 나간다 — 남의
        # 프로세스를 죽이지 않는다는 이 기능의 불변식이 정확히 여기서 깨진다.
        # UI는 external이면 버튼을 비활성으로 두지만 라우트는 열려 있다(오래된
        # 페이지·curl로 도달 가능). 진입만 막는 것이고 종료 로직은 손대지 않는다.
        _ext_pid = None
        if p is None and name in _SYS_EXT_GATE:
            show, gate = _scan_sys_ext()
            _ext_pid = (gate or {}).get(name)
            if gate is None:
                # 판정 불가는 여전히 거부다(fail-closed) — 무엇을 끄는지 모르는 채로
                # killpg 를 쏘지 않는다.
                _log("SYS", f"{defn['label']} 종료 거부 — 외부 여부 판정 불가")
                return jsonify(ok=False, error=(
                    "실행 여부를 확인하지 못해 종료하지 않습니다 — 터미널에서 확인하세요."
                )), 409
            if _ext_pid is None:
                _log("SYS", f"{defn['label']} 종료 요청 — 실행 중인 것이 없다")
                return jsonify(ok=True, running=False)
            # 🔴 여기서 **인수한다.** 예전에는 거부했는데("터미널에서 끄세요"),
            #    2026-09-10 에 사용자가 대시보드를 X 로 닫아 **로봇이 도는데 끌 수단이
            #    없는 상태**에 빠졌다. 모터·라이다가 도는 것을 끄는 수단이 화면에서
            #    사라지는 것이 물리 안전 문제라, 소유권이 없어도 끄게 한다.
            #    대상은 argv 로 우리 런치 파일·우리 rviz 바이너리를 확인한 것뿐이다.
            _log("SYS", f"{defn['label']} 인수 종료 시작 — PID {_ext_pid} "
                        "(이 대시보드가 띄운 것이 아니다. 다른 대시보드가 띄우고 "
                        "닫혀서 고아로 남은 것을 인수한다)")
        _sys_procs.pop(name, None)
        kill_timeout = defn.get("kill_timeout", 6)
        auto_free    = defn.get("auto_free_lock", False)

        def _do_kill():
            import time as _t
            # 내 자식이면 Popen 경로, 인수한 것이면 pgid 경로 — **순서는 같은 구현**이다.
            # 뒤의 rplidar 정리·모터 정지·filelock 해제는 양쪽에서 그대로 돈다(고아
            # 런치를 끈 뒤가 오히려 그 정리가 더 필요한 상황이다).
            graceful = (_kill_proc_group(p, defn["label"], kill_timeout=kill_timeout)
                        if p is not None else
                        _kill_ext_pgid(_ext_pid, defn["label"], kill_timeout=kill_timeout))
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

    # 여정 중에는 **시작 계열만** 막는다(free·home·battery 스크립트, launch·rviz 기동).
    # 종료는 열어 둔다 — launch 종료는 화면에서 모터·라이다를 끌 마지막 수단이고
    # (2026-09-10 대시보드를 닫아 끌 수단이 사라진 사건), rviz 종료는 부하만 줄인다.
    _g = _journey_gate(f"{defn['label']} 시작")
    if _g:
        return _g
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

    # 🔴 재시도 로그를 접는다. 2026-09-10 실측: 3초마다 같은 줄을 찍어 7일 누적
    #    10,307줄(세션 최대 2,474줄)이 됐는데 **아무 정보도 못 줬다** — 정보가 없어서가
    #    아니라 같은 정보가 너무 많아서다. 첫 실패 1줄 + 사유가 바뀔 때 + 주기 요약만.
    _wait, _n, _last_msg, _last_sum = 3.0, 0, None, time.monotonic()
    _t0 = time.monotonic()
    # 장치 경로가 **없다가 생기면** 백오프를 즉시 되감는다 — 사람이 손잡이를 꽂은
    # 순간이 그것이고, 그때 최대 60초를 기다리게 하면 "꽂았는데 안 되네"가 된다.
    _path_seen = os.path.exists(SERIAL_PORT)
    while ser is None:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
            _log("ARD", f"시리얼 연결됨: {SERIAL_PORT}"
                        + (f" — 실패 {_n}회 / {(time.monotonic() - _t0) / 60:.0f}분 뒤 복구"
                           if _n else ""))
        except Exception as e:
            _n += 1
            _msg = str(e)
            if _n == 1:
                _log("ARD", f"🚨 손잡이 시리얼 연결 실패 — {SERIAL_PORT} ({_msg}). "
                            f"재시도는 계속하되 로그는 접는다 "
                            f"(3s→{_SERIAL_BACKOFF_MAX:.0f}s 백오프 · "
                            f"{_SERIAL_SUM_SEC / 60:.0f}분마다 1줄). "
                            "준비바의 '손잡이' 칸이 고장 여부를 말한다")
            elif _msg != _last_msg:
                _log("ARD", f"손잡이 시리얼 실패 **사유 변경**({_n}회째): {_msg}")
            elif time.monotonic() - _last_sum >= _SERIAL_SUM_SEC:
                _last_sum = time.monotonic()
                _log("ARD", f"손잡이 시리얼 여전히 안 붙는다 — {_n}회 누적, "
                            f"{(time.monotonic() - _t0) / 60:.0f}분째 (대기 {_wait:.0f}s)")
            _last_msg = _msg
            time.sleep(_wait)
            _now_seen = os.path.exists(SERIAL_PORT)
            if _now_seen and not _path_seen:
                _log("ARD", f"장치 경로가 나타났다({SERIAL_PORT}) — 백오프 되감고 즉시 재시도")
                _wait = 3.0
            else:
                _wait = min(_wait * 2.0, _SERIAL_BACKOFF_MAX)
            _path_seen = _now_seen

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
    _jr("boot")   # 코드 정체 한 줄([CODE]) — git 은 백그라운드, run_start 가 그 캐시를 싣는다
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
