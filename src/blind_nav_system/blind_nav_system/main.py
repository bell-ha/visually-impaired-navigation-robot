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

def _log(src: str, msg: str):
    entry = {"t": time.strftime("%H:%M:%S"), "src": src, "msg": msg.rstrip()}
    with _log_lock:
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

def _write(name: str, cmd: str):
    p = _procs.get(name)
    if p and p.poll() is None:
        try:
            p.stdin.write(cmd + "\n")
            p.stdin.flush()
        except Exception:
            pass

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
_ready = {"amcl": 0.0, "battery": 0.0, "handle": 0.0, "nav2": 0.0, "elev_app": 0.0}
# 2a-2: 폴링 신호는 "답의 내용"이 age와 독립 → 값을 따로 저장(2a-1의 '값 저장 금지'는 콜백형에만 적용).
#   _ready[key]=monotonic() = "마지막으로 물어본 시각"(폴러 건강),  _ready_val[key] = 판정결과(대상 건강).
_ready_val = {"nav2": None, "elev_app": None}

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

def _readiness_poll_loop():
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
                    if st and st.get("lease_expired"):
                        _ready_val["elev_app"] = {"status": "bad",
                                                  "detail": "엘베 리스 만료 — 제어권 재부여 필요"}
                    else:
                        _ready_val["elev_app"] = {"status": "ok", "detail": "응답"}
                else:
                    _ready_val["elev_app"] = {"status": "bad", "detail": f"HTTP {r.status_code}"}
            except Exception:
                _ready_val["elev_app"] = {"status": "unknown", "detail": "무응답(미기동/접속거부)"}
        except Exception:
            time.sleep(3.0)
            continue
        finally:
            # 성공/타임아웃/예외 무관 — 폴러가 살아있다는 증거로 매 사이클 갱신
            _ready["nav2"] = time.monotonic()
            _ready["elev_app"] = time.monotonic()
        time.sleep(3.0)

def init_ros():
    global _cmd_node, _cmd_pub, _map_client, _init_pub, _arm_client, _lc_loc_client, _lc_nav_client
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
        _diag.attach(
            _cmd_node, _diaglog,
            cmd_vel_topic="/stretch/cmd_vel",
            own_node_name="main_web_cmdvel",
            expected_nodes=["elevator_tracker"],
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

def _manual_loop():
    while True:
        with _manual_lock:
            active = _manual_active
            lx = _manual_cmd["lx"]
            az = _manual_cmd["az"]
        # 수동 활성 상태일 때만 퍼블리시 — 자동 모드에서는 Nav2가 직접 제어
        if active:
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
    def gen():
        sent = 0
        while True:
            with _log_lock:
                entries = list(_LOG_BUF)
            for e in entries[sent:]:
                yield f"data: {json.dumps(e, ensure_ascii=False)}\n\n"
            sent = len(entries)
            time.sleep(0.1)
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

@app.route("/cmd", methods=["POST"])
def cmd():
    global _manual_active
    data = request.json or {}
    lx = float(data.get("lx", 0.0))
    az = float(data.get("az", 0.0))
    with _manual_lock:
        _manual_cmd["lx"] = lx
        _manual_cmd["az"] = az
        _manual_active = data.get("active", True)
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
        threading.Thread(target=_set_elev_authority,
                         args=(granted, "UI 토글"), daemon=True).start()
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
        threading.Thread(target=_set_elev_authority,
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
_READY_THRESH = {"amcl": 30.0, "battery": 15.0, "handle": 5.0, "nav2": 12.0, "elev_app": 12.0}

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
        gripper_camera={"status": "unknown", "age_sec": None, "detail": "미구현"},
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
    return jsonify(running=_elev_app_running(), authority=_elev_authority)

@app.route("/elevator_app", methods=["POST"])
def elevator_app():
    global _elev_lease_held, _lease_renewer_thread
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
        if _lease_renewer_thread is not None:
            _lease_renewer_thread.join(timeout=1.0)
            _lease_renewer_thread = None
        # 프로세스가 죽으면 리스는 물리적으로 소멸 — abort·수동종료·8단계가 전부
        # 이 라우트를 지나므로 여기 한 곳만 리셋하면 죽은 앱에 finally가 재POST해
        # 가짜 🚨를 내는 것도 자연히 없어짐(_elev_lease_held=False라 no-op)
        _elev_lease_held = False
        _log("MAIN", "엘리베이터 앱 종료 + 제어권 회수 → 주행(nav) 복귀")
        return jsonify(ok=True, running=False)

    # 이미 켜져 있음 → 앱만 켜진 상태 유지, 제어권은 건드리지 않음(엘베앱 켜기 ≠ 제어권 주기)
    if desired is True and running:
        return jsonify(ok=True, running=True)

    # 시작 (경로는 __file__ 기준이라 cwd 무관, 부모 env(ROS·FastDDS) 상속)
    proc = subprocess.Popen(
        [sys.executable, "-u", str(THIS_DIR / "elevator_button_press/main.py")],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    _procs["elevator"] = proc
    threading.Thread(target=_capture, args=(proc, "ELEV"), daemon=True).start()
    # 앱만 켠다 — 제어권은 안 줌(엘베앱 켜기 ≠ 제어권 주기). 수동 사용 시 UI의
    # "제어권 부여" 토글로, 자동 여정 중엔 _auto_run이 _grant_elev_lease로 부여.
    threading.Thread(target=_wait_elev_app_up, daemon=True).start()
    _log("MAIN", f"엘리베이터 앱 시작 PID={proc.pid} (제어권은 별도 부여 필요)")
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
    """누르기 실행(POST /press). 성공 시 True."""
    r = _elev_post("/press", {}, timeout=15)
    return bool(r and r.get("ok"))

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
    취소 존중. 완료 True / 타임아웃·취소 False(그래도 흐름은 계속)."""
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

def _auto_run(dest):
    """반자동 여정 상태머신 (백그라운드 스레드)."""
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
        _elev_scene(0)                          # place=hall + 인식자세(블로킹)
        _elev_select("^" if up else "s")        # 호출버튼 자동 선택 (^=상행 s=하행)
        if _elev_wait_ready():
            _auto_set("① 호출", f"정렬 완료 ✅ — '다음' 누르면 호출({dir_txt}) 누름", wait=True)
        else:
            _auto_set("① 호출", "⚠ 자동정렬 실패 — 엘베UI서 수동 정렬 후 '다음'", wait=True)
        if not _auto_wait_confirm(): _auto_abort_elev(); return
        _elev_press()                           # 다음 → 호출버튼 누르기
        _auto_set("① 호출", "호출 버튼 누르는 중... (팔 복귀까지 대기)")
        _elev_wait_press_done()                 # 눌림+팔복귀 완료까지 대기(이동 안전)

        # ② 문앞 정렬: 전진 56.5 + 우회전 90° (자동 안무)
        _auto_set("② 문앞정렬", "문 앞으로 정렬 중(전진·회전)... 완료되면 '다음'",
                  wait=True, phase="front")
        _elev_scene(1)
        if not _auto_wait_confirm(): _auto_abort_elev(); return

        # ③ 문 열림 대기 (자동 감지)
        _auto_set("③ 문열림", "문 열림 대기 중... 문 열리면 '다음'", wait=True, phase="door")
        _elev_scene(2)
        if not _auto_wait_confirm(): _auto_abort_elev(); return

        # ④ 탑승: 전진 185 (자동 안무)
        _auto_set("④ 탑승", "탑승(전진) 중... 다 탔으면 '다음'", wait=True, phase="ride")
        _elev_scene(3)
        if not _auto_wait_confirm(): _auto_abort_elev(); return

        # ⑤ 층 press: 인식자세 → 목적층 자동선택 → 정렬 → '다음'에 누르기
        _auto_set("⑤ 층선택", f"인식 자세 + {dest_floor}층 버튼 자동 선택·정렬 중...",
                  phase="floor")
        _elev_scene(4)                          # place=cab + 인식자세
        _elev_select(dest_floor)                # 층버튼 자동 선택
        if _elev_wait_ready():
            _auto_set("⑤ 층선택", f"정렬 완료 ✅ — '다음' 누르면 {dest_floor}층 누름", wait=True)
        else:
            _auto_set("⑤ 층선택", "⚠ 자동정렬 실패 — 엘베UI서 수동 정렬 후 '다음'", wait=True)
        if not _auto_wait_confirm(): _auto_abort_elev(); return
        _elev_press()                           # 다음 → 층버튼 누르기
        _auto_set("⑤ 층선택", f"{dest_floor}층 버튼 누르는 중... (팔 복귀까지 대기)")
        _elev_wait_press_done()                 # 눌림+팔복귀 완료까지 대기

        # 엘베 이동 대기 → ⑥ 하차: 후진 186 (자동 안무)
        _auto_set("이동중", f"{dest_floor}층 이동 중 — 도착·하차 준비되면 '다음'",
                  wait=True, phase="moving")
        if not _auto_wait_confirm(): _auto_abort_elev(); return
        _auto_set("⑥ 하차", "하차(후진) 중...", phase="exit")
        _elev_scene(5)
        time.sleep(3)

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
            if _elev_lease_held and not _grant_elev_lease(False, "여정 종료(finally)"):
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


def _proc_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except (ProcessLookupError, OSError):
        return False


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
    return jsonify({
        name: bool(_sys_procs.get(name) and _sys_procs[name].poll() is None)
        for name in _SYS_PROC_DEFS
    })

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

@app.route("/net_mode", methods=["POST"])
def set_net_mode():
    offline = bool((request.json or {}).get("offline", False))
    _write("iface", "/offline" if offline else "/online")
    _log("WEB", f"네트워크 모드: {'🔴 오프라인' if offline else '🟢 온라인'}")
    return jsonify(ok=True, offline=offline)

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
