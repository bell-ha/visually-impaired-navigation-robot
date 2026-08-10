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
from flask import Flask, Response, jsonify, request

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
THIS_DIR = Path(__file__).resolve().parent

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
    _procs["iface"] = iface
    _procs["vision"] = vision
    threading.Thread(target=_capture, args=(iface, "IFACE"), daemon=True).start()
    threading.Thread(target=_capture, args=(vision, "VISION"), daemon=True).start()
    _log("MAIN", f"interface.py PID={iface.pid}, vision_assistant.py PID={vision.pid}")

# ── ROS2 cmd_vel 퍼블리셔 ─────────────────────────────────────────────────────
_cmd_node = None
_cmd_pub  = None
_manual_mode = False
_manual_lock = threading.Lock()

_backup_warn_until = 0.0   # 후진 안내 디바운스

# ── 배터리 상태 ────────────────────────────────────────────────────────────────
_battery = {"pct": None, "voltage": None, "charging": None}

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
    import math
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    _robot_pose.update(x=round(p.x, 3), y=round(p.y, 3),
                       z=round(q.z, 4), w=round(q.w, 4),
                       yaw_deg=round(math.degrees(yaw), 1))

def _battery_callback(msg):
    pct = round(msg.percentage * 100) if msg.percentage <= 1.0 else round(msg.percentage)
    _battery["pct"]      = pct
    _battery["voltage"]  = round(msg.voltage, 1)
    _battery["charging"] = msg.power_supply_status == 1  # CHARGING=1

def _cmdvel_callback(msg):
    global _backup_warn_until
    if _manual_mode:
        return   # 수동 모드에서는 안내 생략
    if msg.linear.x < -0.01:
        now = time.monotonic()
        if now > _backup_warn_until:
            _backup_warn_until = now + 10.0
            _write("iface", "/backup")
            _log("MAIN", "후진 감지 → TTS 안내")

def init_ros():
    global _cmd_node, _cmd_pub, _map_client, _init_pub
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
    # 진단 계측 부착 (cmd_vel 퍼블리셔 수·엘리베이터 노드 존재 추적)
    if _diag is not None and _diaglog is not None:
        _diag.attach(
            _cmd_node, _diaglog,
            cmd_vel_topic="/stretch/cmd_vel",
            own_node_name="main_web_cmdvel",
            expected_nodes=["elevator_tracker"],
            rosout=True,   # nav2가 왜 복구를 부르는지(/rosout) 파일에 기록
        )
    threading.Thread(target=rclpy.spin, args=(_cmd_node,), daemon=True).start()
    _log("MAIN", "ROS2 cmd_vel 퍼블리셔/구독자 시작")

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

@app.route("/")
def index():
    return HTML

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

def _set_elev_authority(granted: bool, reason: str = ""):
    """엘리베이터 앱에 이동 제어권 부여/회수. 회수 시 엘리베이터는 즉시 전면
    정지(바퀴 정지·타겟 해제·안무 중단)하고 이후 모든 이동을 거부한다.
    구버전 엘리베이터 앱(엔드포인트 없음)이면 /reset 폴백. 꺼져 있으면 무시."""
    global _elev_authority
    # 대시보드 의도를 항상 반영 — 엘리베이터 앱(5000)이 안 떠서 POST가 실패해도
    # 토글이 되돌아오지 않게. (JS 폴링이 이 값으로 토글을 동기화하므로)
    _elev_authority = granted
    if granted:
        # 엘리베이터가 팔(트래젝토리 액션)을 써야 함 → armleft를 반드시 종료(팔 넘겨줌)
        _stop_armleft_proc()
        _log("MAIN", "엘리베이터 제어권 부여 → armleft 자동 종료")
    try:
        import urllib.request
        req = urllib.request.Request(
            "http://localhost:5000/authority",
            data=json.dumps({"granted": granted}).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        urllib.request.urlopen(req, timeout=1)
        _log("MAIN", f"엘리베이터 제어권 {'부여' if granted else '회수'}"
                     + (f" ({reason})" if reason else ""))
    except Exception:
        if not granted:
            try:   # 폴백: 구버전 앱이면 최소한 타겟 해제
                import urllib.request
                urllib.request.urlopen(urllib.request.Request(
                    "http://localhost:5000/reset", method="POST"), timeout=1)
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

# ── HTML ──────────────────────────────────────────────────────────────────────
HTML = """<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>로봇 대시보드</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: 'Segoe UI', sans-serif; background: #0f172a; color: #e2e8f0; height: 100vh; display: flex; flex-direction: column; }
  header { background: #1e293b; padding: 12px 20px; display: flex; align-items: center; gap: 16px; border-bottom: 1px solid #334155; }
  header h1 { font-size: 1.1rem; color: #94a3b8; flex: 1; }
  .battery { font-size: 0.85rem; font-weight: 600; color: #94a3b8; white-space: nowrap; }
  .battery.ok   { color: #6ee7b7; }
  .battery.low  { color: #fbbf24; }
  .battery.crit { color: #f87171; }
  .battery.charging { color: #60a5fa; }
  .badge { padding: 4px 12px; border-radius: 20px; font-size: 0.78rem; font-weight: 600; }
  .badge-auto   { background: #065f46; color: #6ee7b7; }
  .badge-manual { background: #7c2d12; color: #fdba74; }
  .layout { display: grid; grid-template-columns: 1fr auto; flex: 1; overflow: hidden; }

  /* 로그 패널 */
  #log-panel { padding: 12px; overflow-y: auto; font-family: monospace; font-size: 0.8rem; background: #0f172a; }
  .log-entry { padding: 2px 0; border-bottom: 1px solid #1e293b; white-space: pre-wrap; word-break: break-all; }
  .src-IFACE    { color: #7dd3fc; }
  .src-VISION   { color: #86efac; }
  .src-HW       { color: #fbbf24; }
  .src-MAIN     { color: #c084fc; }
  .src-WEB      { color: #fb7185; }
  .src-OBSTACLE { color: #fb923c; }
  .src-MAP      { color: #22d3ee; }
  .src-POSE     { color: #f0abfc; }
  .src-ARM      { color: #fca5a5; }

  /* 의자 감지 상태 카드 */
  .chair-card { background: #0f172a; border: 1px solid #334155; border-radius: 10px; padding: 10px 14px; }
  .chair-card.detected { border-color: #fb923c; }
  .chair-dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: #475569; margin-right: 6px; vertical-align: middle; }
  .chair-dot.on { background: #fb923c; box-shadow: 0 0 6px #fb923c; }
  .chair-status { font-size: 0.82rem; color: #e2e8f0; font-weight: 600; }
  .chair-detail { font-size: 0.75rem; color: #94a3b8; margin-top: 4px; }
  .ts { color: #475569; margin-right: 6px; }

  /* 컨트롤 패널 */
  .ctrl-panel { background: #1e293b; border-left: 1px solid #334155; padding: 16px; width: 320px; overflow-y: auto; }
  .ctrl-col > div { padding: 14px 0; border-bottom: 1px solid #1e293b; }
  .ctrl-col:first-child > div:first-child { padding-top: 0; }
  .ctrl-col:last-child > div:last-child  { border-bottom: none; padding-bottom: 0; }
  .ctrl-panel h2 { font-size: 0.68rem; font-weight: 700; color: #475569; text-transform: uppercase; letter-spacing: 0.08em; margin-bottom: 10px; }


  /* 버튼 통일 시스템 */
  .btn { display: block; width: 100%; border: none; border-radius: 6px; padding: 9px 14px; font-size: 0.82rem; font-weight: 600; cursor: pointer; text-align: center; color: #fff; transition: opacity .15s; }
  .btn:hover  { opacity: 0.85; }
  .btn:active { opacity: 0.70; }
  .btn-sm  { padding: 5px 12px; font-size: 0.78rem; width: auto; display: inline-block; }
  .btn-primary { background: #2563eb; }
  .btn-danger  { background: #dc2626; }
  .btn-neutral { background: #334155; }
  .mode-btns { display: flex; gap: 8px; }

  /* 방향 패드 */
  .dpad { display: grid; grid-template-columns: repeat(3, 64px); grid-template-rows: repeat(3, 64px); gap: 6px; justify-content: center; }
  .dpad-btn { background: #334155; border: none; border-radius: 10px; color: #e2e8f0; font-size: 1.4rem; cursor: pointer; user-select: none; transition: background .1s; }
  .dpad-btn.pressed { background: #2563eb; box-shadow: 0 0 0 3px #3b82f6; }
  .dpad-center { background: #1e293b; cursor: default; }
  .speed-wrap { display: flex; align-items: center; gap: 8px; }
  .speed-wrap label { font-size: 0.75rem; color: #94a3b8; white-space: nowrap; }
  input[type=range] { flex: 1; accent-color: #3b82f6; }
  .speed-val { font-size: 0.75rem; color: #94a3b8; width: 32px; text-align: right; }

  /* 토글 스위치 */
  .toggle-row { display: flex; align-items: center; justify-content: space-between; padding: 7px 0; border-bottom: 1px solid #1e293b; }
  .toggle-row:last-child { border-bottom: none; }
  .toggle-label { font-size: 0.82rem; color: #e2e8f0; }
  .kbd { display:inline-block; background:#334155; color:#cbd5e1; border-radius:4px;
         padding:0 5px; font-size:0.65rem; font-family:monospace; margin-left:6px; vertical-align:1px; }
  .toggle-switch { position: relative; width: 44px; height: 24px; flex-shrink: 0; }
  .toggle-switch input { opacity: 0; width: 0; height: 0; }
  .toggle-slider { position: absolute; inset: 0; background: #475569; border-radius: 24px; cursor: pointer; transition: background .2s; }
  .toggle-slider::before { content: ''; position: absolute; width: 18px; height: 18px; left: 3px; top: 3px; background: #fff; border-radius: 50%; transition: transform .2s; }
  .toggle-switch input:checked + .toggle-slider { background: #2563eb; }
  .toggle-switch input:checked + .toggle-slider::before { transform: translateX(20px); }

  /* 로그 탭 */
  .log-container { display: flex; flex-direction: column; overflow: hidden; }
  .tab-bar { display: flex; gap: 4px; padding: 8px 12px; background: #1e293b; border-bottom: 1px solid #334155; flex-wrap: wrap; }
  .tab-btn { background: #334155; border: none; border-radius: 6px; color: #94a3b8; font-size: 0.75rem; font-weight: 600; padding: 4px 10px; cursor: pointer; transition: background .15s; }
  .tab-btn.active { background: #2563eb; color: #fff; }
  .tab-btn:hover:not(.active) { background: #475569; }
  .tab-badge { background: #ef4444; color: #fff; border-radius: 10px; font-size: 0.65rem; padding: 1px 5px; margin-left: 4px; vertical-align: middle; }
  .src-ARD  { color: #4ade80; }
  .src-SYS  { color: #a78bfa; }
  .src-ROS2 { color: #38bdf8; }
  .src-RVIZ { color: #34d399; }
  .src-BATT { color: #fbbf24; }
  .src-FREE { color: #f87171; }
  .src-HOME { color: #fb923c; }
  .sys-proc-row { display:flex; align-items:center; gap:8px; padding:5px 0; border-bottom:1px solid #1e293b; }
  .sys-proc-row:last-child { border-bottom:none; }
  .sys-proc-label { flex:1; font-size:0.82rem; color:#cbd5e1; }
  .sys-dot { width:8px; height:8px; border-radius:50%; background:#374151; flex-shrink:0; }
  .sys-dot.on { background:#22c55e; box-shadow:0 0 4px #22c55e; }
  /* tab 배지 — 숫자 없이 점(dot)만 */
  .tab-badge { display:inline-block; width:6px; height:6px; border-radius:50%; background:#ef4444; margin-left:4px; vertical-align:middle; }

  /* ── 반응형: 반드시 스타일시트 맨 끝에 위치 (동일 특이도 규칙을 덮어쓰기 위해) ── */
  /* 화면 높이가 낮으면 패널을 2열로 펼쳐 스크롤 없이 모두 보이게 */
  @media (max-height: 1300px) and (min-width: 1100px) {
    .ctrl-panel { width: 660px; display: flex; align-items: flex-start; }
    .ctrl-col { flex: 1; min-width: 0; }
    .ctrl-col + .ctrl-col { border-left: 1px solid #334155; padding-left: 26px; margin-left: 26px; }
    .ctrl-col > div:first-child { padding-top: 0; }
    .ctrl-col > div:last-child  { border-bottom: none; }
  }
  /* 더 낮으면 컴팩트 모드: 여백/버튼/패드 축소 */
  @media (max-height: 780px) {
    .ctrl-panel { padding: 8px 14px; }
    .ctrl-col > div, .ctrl-col:first-child > div:first-child { padding: 6px 0; }
    .ctrl-panel h2 { margin-bottom: 4px; }
    .btn { padding: 5px 10px; font-size: 0.78rem; }
    .btn-sm { padding: 3px 10px; font-size: 0.74rem; }
    .dpad { grid-template-columns: repeat(3, 44px); grid-template-rows: repeat(3, 44px); gap: 4px; }
    .dpad-btn { font-size: 1.05rem; border-radius: 8px; }
    .sys-proc-row { padding: 2px 0; }
    header { padding: 6px 20px; }
    #kbd-help { font-size: 0.66rem !important; line-height: 1.5 !important;
                padding: 5px 8px !important; margin-top: 6px !important; }
    #manual-section .speed-wrap { margin-bottom: 6px !important; }
    #manual-section .btn-danger { margin-top: 6px !important; font-size: 0.8rem !important; }
    #sys-proc-list > div:last-child { margin-top: 6px !important; padding-top: 6px !important; }
    .chair-card { padding: 6px 12px; }
    .toggle-row { padding: 2px 0; }
    .toggle-switch { width: 38px; height: 20px; }
    .toggle-slider::before { width: 14px; height: 14px; }
    .toggle-switch input:checked + .toggle-slider::before { transform: translateX(18px); }
    #speed-settings { gap: 4px !important; margin-top: 0 !important; }
  }
  /* 좁은 화면에서는 1열로 복귀 (로그 영역 확보) */
  @media (max-width: 1099px) {
    .ctrl-panel { width: 320px; display: block; }
    .ctrl-col + .ctrl-col { border-left: none; padding-left: 0; margin-left: 0; }
  }
</style>
</head>
<body>
<header>
  <h1>시각장애인 안내 로봇 대시보드</h1>
  <span class="badge badge-auto" id="mode-badge">자동 모드</span>
</header>
<div class="layout">
  <div class="log-container">
    <div class="tab-bar">
      <button class="tab-btn active" id="tab-ALL"    onclick="setTab('ALL')">전체</button>
      <button class="tab-btn"        id="tab-IFACE"  onclick="setTab('IFACE')">인터페이스<span class="tab-badge" id="badge-IFACE"  style="display:none"></span></button>
      <button class="tab-btn"        id="tab-VISION" onclick="setTab('VISION')">비전<span class="tab-badge" id="badge-VISION" style="display:none"></span></button>
      <button class="tab-btn"        id="tab-HW"     onclick="setTab('HW')">하드웨어<span class="tab-badge" id="badge-HW"     style="display:none"></span></button>
      <button class="tab-btn"        id="tab-MAIN"   onclick="setTab('MAIN')">메인<span class="tab-badge" id="badge-MAIN"   style="display:none"></span></button>
      <button class="tab-btn"        id="tab-WEB"    onclick="setTab('WEB')">웹<span class="tab-badge" id="badge-WEB"    style="display:none"></span></button>
      <button class="tab-btn"        id="tab-ARM"    onclick="setTab('ARM')">팔고정<span class="tab-badge" id="badge-ARM"    style="display:none"></span></button>
      <button class="tab-btn"        id="tab-SYS"    onclick="setTab('SYS')">시스템<span class="tab-badge" id="badge-SYS"    style="display:none"></span></button>
      <button class="tab-btn"        id="tab-ROS2"   onclick="setTab('ROS2')">ROS2<span class="tab-badge" id="badge-ROS2"   style="display:none"></span></button>
      <button class="tab-btn"        id="tab-RVIZ"   onclick="setTab('RVIZ')">RViz<span class="tab-badge" id="badge-RVIZ"   style="display:none"></span></button>
      <button class="tab-btn"        id="tab-BATT"   onclick="setTab('BATT')">배터리<span class="tab-badge" id="badge-BATT"   style="display:none"></span></button>
      <button class="tab-btn"        id="tab-FREE"   onclick="setTab('FREE')">정리<span class="tab-badge" id="badge-FREE"   style="display:none"></span></button>
      <button class="tab-btn"        id="tab-HOME"   onclick="setTab('HOME')">홈위치<span class="tab-badge" id="badge-HOME"   style="display:none"></span></button>
      <button class="tab-btn"        id="tab-ARD"    onclick="setTab('ARD')">아두이노<span class="tab-badge" id="badge-ARD"    style="display:none"></span></button>
      <button class="tab-btn"        id="tab-MAP"      onclick="setTab('MAP')">지도<span class="tab-badge" id="badge-MAP"      style="display:none"></span></button>
      <button class="tab-btn"        id="tab-POSE"     onclick="setTab('POSE')">위치<span class="tab-badge" id="badge-POSE"     style="display:none"></span></button>
      <button class="tab-btn"        id="tab-OBSTACLE" onclick="setTab('OBSTACLE')">장애물<span class="tab-badge" id="badge-OBSTACLE" style="display:none"></span></button>
    </div>
    <div id="log-panel"></div>
  </div>
  <div class="ctrl-panel">
   <div class="ctrl-col">
    <div>
      <h2>모드 전환</h2>
      <div class="mode-btns">
        <button class="btn btn-primary btn-sm" id="btn-mode-auto"   onclick="setMode(false)">자동</button>
        <button class="btn btn-neutral btn-sm" id="btn-mode-manual" onclick="setMode(true)">수동</button>
      </div>
    </div>

    <div id="location-section">
      <h2>장소 목록</h2>
      <div id="location-list" style="display:flex;flex-direction:column;gap:4px;margin-top:6px;max-height:320px;overflow-y:auto"></div>
    </div>

    <div id="manual-section" style="display:none">
      <h2>수동 조작</h2>
      <div class="speed-wrap" style="margin-bottom:10px">
        <label>속도</label>
        <input type="range" id="speed" min="5" max="40" value="15"
               oninput="setSpeed(parseInt(this.value))">
        <span class="speed-val" id="spv">15</span>
      </div>
      <div class="dpad">
        <div></div>
        <button class="dpad-btn" id="btn-fwd"   onclick="toggleCmd('fwd',   1,    0)">↑</button>
        <div></div>
        <button class="dpad-btn" id="btn-left"  onclick="toggleCmd('left',  0,    1)">←</button>
        <div class="dpad-btn dpad-center"></div>
        <button class="dpad-btn" id="btn-right" onclick="toggleCmd('right', 0,   -1)">→</button>
        <div></div>
        <button class="dpad-btn" id="btn-bwd"   onclick="toggleCmd('bwd',  -0.5,  0)">↓</button>
        <div></div>
      </div>
      <button class="btn btn-danger" style="margin-top:10px;font-size:0.9rem" onclick="emergencyStop()">■ 비상 정지</button>
      <div id="kbd-help" style="margin-top:10px;background:#1e293b;border-radius:6px;padding:8px 10px;font-size:0.72rem;color:#94a3b8;line-height:1.7">
        ⌨️ <b style="color:#cbd5e1">키보드 단축키</b><br>
        방향키 ↑↓←→ : 이동 토글 (같은 키 다시 누르면 정지)<br>
        키패드 + / − : 속도 조절 (±5)<br>
        숫자 1~4 : 기능 설정 토글 · Shift+1~5 : 시스템 프로세스
      </div>
    </div>

    <div>
      <h2>빠른 액션</h2>
      <div style="display:flex;flex-direction:column;gap:8px">
        <button class="btn btn-primary" onclick="sendButton()">목적지 입력 (버튼1)</button>
        <button class="btn btn-primary" onclick="sendVision()">시각 분석 (버튼2)</button>
        <button class="btn btn-neutral" onclick="sendPull()">당김 트리거</button>
      </div>
    </div>
   </div>

   <div class="ctrl-col">
    <div>
      <h2>기능 설정</h2>
      <div style="margin-top:4px">
        <div class="toggle-row">
          <span class="toggle-label">팔 위치 고정<span class="kbd">1</span></span>
          <label class="toggle-switch">
            <input type="checkbox" id="armleft-toggle" onchange="toggleArmleft()">
            <span class="toggle-slider"></span>
          </label>
        </div>
        <div class="toggle-row">
          <span class="toggle-label">사회적 회피<span class="kbd">2</span></span>
          <label class="toggle-switch">
            <input type="checkbox" id="social-nav-toggle" onchange="toggleSocialNav()">
            <span class="toggle-slider"></span>
          </label>
        </div>
        <!-- 장애물 밀기 토글 제거 (2026-07-24 사용자 결정) — 백엔드 코드는 휴면 상태로 보존 -->
        <div class="toggle-row">
          <span class="toggle-label">온라인 모드<span class="kbd">4</span></span>
          <label class="toggle-switch">
            <input type="checkbox" id="net-mode-toggle" checked onchange="toggleNetMode()">
            <span class="toggle-slider"></span>
          </label>
        </div>
        <div class="toggle-row">
          <span class="toggle-label">🛗 엘리베이터 제어권<span class="kbd">5</span></span>
          <label class="toggle-switch">
            <input type="checkbox" id="elev-auth-toggle" onchange="toggleElevAuthority()">
            <span class="toggle-slider"></span>
          </label>
        </div>
        <div class="toggle-row">
          <span class="toggle-label">🗺 지도(층)</span>
          <span style="display:flex;gap:6px;align-items:center;">
            <select id="floor-sel" style="background:#223;color:#cde;border:1px solid #345;border-radius:4px;padding:3px 6px;">
              <option value="5" selected>5층</option>
              <option value="4">4층</option>
              <option value="3">3층</option>
            </select>
            <label style="font-size:0.68rem;color:#9ab;display:flex;align-items:center;gap:3px;cursor:pointer;"
                   title="전환 직후 AMCL 위치를 엘리베이터 하차지점으로 자동 설정 — 엘리베이터로 층 이동한 직후에만 켜두세요">
              <input type="checkbox" id="init-exit-chk" checked>하차지점 초기화
            </label>
            <button onclick="switchMap()" style="background:#345;color:#dee;border:none;border-radius:6px;padding:5px 12px;cursor:pointer;font-size:0.8rem;">전환</button>
          </span>
        </div>
        <div class="toggle-row">
          <span class="toggle-label">📍 현재 위치</span>
          <span id="pose-result" style="font-size:0.72rem;color:#8ac;margin:0 8px;
                flex:1;text-align:right;user-select:all;"></span>
          <button onclick="queryPose()" style="background:#345;color:#dee;border:none;
                  border-radius:6px;padding:5px 12px;cursor:pointer;font-size:0.8rem;">조회</button>
        </div>
      </div>
    </div>

    <div>
      <h2>의자 감지</h2>
      <div class="chair-card" id="chair-card">
        <div>
          <span class="chair-dot" id="chair-dot"></span>
          <span class="chair-status" id="chair-status">감지 없음</span>
        </div>
        <div class="chair-detail" id="chair-detail"></div>
      </div>
    </div>

    <div>
      <h2>시스템 프로세스</h2>
      <div id="sys-proc-list" style="margin-top:6px">
        <div class="sys-proc-row" id="sysrow-launch">
          <span class="sys-dot" id="sysdot-launch"></span>
          <span class="sys-proc-label">ROS2 Launch<span class="kbd">⇧1</span></span>
          <button class="btn btn-primary btn-sm" id="sysbtn-launch" onclick="toggleSysProc('launch')">시작</button>
        </div>
        <div class="sys-proc-row" id="sysrow-rviz">
          <span class="sys-dot" id="sysdot-rviz"></span>
          <span class="sys-proc-label">RViz2<span class="kbd">⇧2</span></span>
          <button class="btn btn-primary btn-sm" id="sysbtn-rviz" onclick="toggleSysProc('rviz')">시작</button>
        </div>
        <div class="sys-proc-row" id="sysrow-battery">
          <span class="sys-dot" id="sysdot-battery"></span>
          <span class="sys-proc-label">배터리 확인<span class="kbd">⇧3</span></span>
          <button class="btn btn-neutral btn-sm" id="sysbtn-battery" onclick="toggleSysProc('battery')">실행</button>
        </div>
        <div class="sys-proc-row" id="sysrow-free">
          <span class="sys-dot" id="sysdot-free"></span>
          <span class="sys-proc-label">프로세스 정리<span class="kbd">⇧4</span></span>
          <button class="btn btn-neutral btn-sm" id="sysbtn-free" onclick="toggleSysProc('free')">실행</button>
        </div>
        <div class="sys-proc-row" id="sysrow-home">
          <span class="sys-dot" id="sysdot-home"></span>
          <span class="sys-proc-label">홈 위치<span class="kbd">⇧5</span></span>
          <button class="btn btn-neutral btn-sm" id="sysbtn-home" onclick="toggleSysProc('home')">실행</button>
        </div>
        <div style="margin-top:12px;padding-top:12px;border-top:1px solid #1e293b">
          <button id="killall-btn" class="btn btn-danger" onclick="killAllRobot()">
            전체 로봇 프로세스 강제 종료
          </button>
        </div>
      </div>
    </div>

    <div>
      <h2>속도 설정</h2>
      <div id="speed-settings" style="display:flex;flex-direction:column;gap:10px;margin-top:4px">
        <div>
          <label style="color:#94a3b8;font-size:0.8rem">로봇 속도: <span id="robot-speed-val">0.26</span> m/s</label>
          <input type="range" min="0.10" max="0.50" step="0.02" value="0.26" style="width:100%"
                 oninput="setRobotSpeed(this.value)">
        </div>
        <div>
          <label style="color:#94a3b8;font-size:0.8rem">TTS 속도: <span id="tts-speed-val">1.5</span>x</label>
          <input type="range" min="0.5" max="2.0" step="0.1" value="1.5" style="width:100%"
                 oninput="setTtsSpeed(this.value)">
        </div>
      </div>
    </div>
   </div>
  </div>
</div>

<script>
let manualMode = false;
let activeDir = null;   // 현재 눌린 방향 ('fwd'|'bwd'|'left'|'right'|null)
let cmdInterval = null;
let currentLx = 0, currentAz = 0;

function setMode(manual) {
  manualMode = manual;
  fetch('/mode', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({manual})});
  document.getElementById('mode-badge').textContent = manual ? '수동 모드' : '자동 모드';
  document.getElementById('mode-badge').className = 'badge ' + (manual ? 'badge-manual' : 'badge-auto');
  document.getElementById('manual-section').style.display = manual ? 'block' : 'none';
  document.getElementById('location-section').style.display = manual ? 'none' : 'block';
  // 활성 모드 버튼 강조
  document.getElementById('btn-mode-auto').className   = manual ? 'btn btn-neutral btn-sm' : 'btn btn-primary btn-sm';
  document.getElementById('btn-mode-manual').className = manual ? 'btn btn-primary btn-sm' : 'btn btn-neutral btn-sm';
  if (!manual) { _stopMove(); loadLocations(); }
}

function loadLocations() {
  fetch('/locations').then(r => r.json()).then(d => {
    const el = document.getElementById('location-list');
    const item = loc => loc.confirmed
      ? `<div class="loc-item" data-name="${escHtml(loc.name)}" style="background:#14532d;border-radius:6px;padding:5px 10px;font-size:0.78rem;color:#86efac;cursor:pointer" title="클릭 = 즉시 출발 (개발용 임시방편 — 최종판은 확인 절차 복원 예정)">✅ ${escHtml(loc.name)}</div>`
      : `<div class="loc-item" data-name="${escHtml(loc.name)}" style="background:#1e293b;border-radius:6px;padding:5px 10px;font-size:0.78rem;color:#475569;cursor:pointer" title="클릭 = 즉시 출발 (개발용 임시방편 — 최종판은 확인 절차 복원 예정)">⬜ ${escHtml(loc.name)}</div>`;
    // 층별 그룹: 공용(엘리베이터 지점, floor 없음) → 5층 → 4층 → 3층
    const groups = [[null, '🛗 공용'], [5, '5층'], [4, '4층'], [3, '3층']];
    el.innerHTML = groups.map(([f, label]) => {
      const items = d.locations.filter(l => (l.floor ?? null) === f);
      if (!items.length) return '';
      return `<div style="color:#89a;font-size:0.72rem;font-weight:700;margin:8px 0 2px;`
           + `border-bottom:1px solid #26263a;padding-bottom:2px;position:sticky;top:0;`
           + `background:#0f172a;">${label} (${items.length})</div>`
           + items.map(item).join('');
    }).join('');
    // 클릭 = 즉시 출발 (오클릭 방지용 브라우저 확인 팝업만 거침)
    el.querySelectorAll('.loc-item').forEach(div => div.onclick = () => {
      const n = div.dataset.name;
      if (!confirm(n + ' (으)로 즉시 출발합니다. 진행할까요?')) return;
      fetch('/goto', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({name: n})})
        .then(r => r.json()).then(d => { if (!d.ok) alert(d.error); });
    });
  }).catch(() => {});
}
loadLocations();

function getSpeed() { return parseInt(document.getElementById('speed').value) / 100; }

const DIR_MUL = {fwd:[1,0], bwd:[-0.5,0], left:[0,1], right:[0,-1]};

function setSpeed(v) {
  v = Math.max(5, Math.min(40, v));
  document.getElementById('speed').value = v;
  document.getElementById('spv').textContent = v;
  // 주행 중이면 새 속도 즉시 반영
  if (activeDir) {
    const [lxMul, azMul] = DIR_MUL[activeDir];
    currentLx = lxMul * v / 100;
    currentAz = azMul * v / 100 * 2;
  }
}

// 토글: 같은 버튼 재클릭 → 정지 / 다른 버튼 클릭 → 방향 전환
function toggleCmd(dir, lxMul, azMul) {
  if (!manualMode) return;
  if (activeDir === dir) {
    _stopMove();
  } else {
    clearInterval(cmdInterval);
    activeDir = dir;
    const spd = getSpeed();
    currentLx = lxMul * spd;
    currentAz = azMul * spd * 2;
    _sendCmd(true);
    cmdInterval = setInterval(() => _sendCmd(true), 100);
    _updateDpad();
  }
}

function _stopMove() {
  clearInterval(cmdInterval);
  cmdInterval = null;
  activeDir = null;
  currentLx = 0; currentAz = 0;
  _sendCmd(false);
  _updateDpad();
}

function _sendCmd(active) {
  fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({lx: currentLx, az: currentAz, active})});
}

function _updateDpad() {
  ['fwd','bwd','left','right'].forEach(d => {
    const btn = document.getElementById('btn-' + d);
    if (btn) btn.classList.toggle('pressed', d === activeDir);
  });
}

function emergencyStop() {
  _stopMove();
  fetch('/stop', {method:'POST'});
}

function sendButton() { fetch('/button', {method:'POST'}); }
function sendVision() { fetch('/vision', {method:'POST'}); }
function sendPull()   { fetch('/pull',   {method:'POST'}); }

let armleftRunning = false;
function toggleArmleft() {
  const desired = document.getElementById('armleft-toggle').checked;
  fetch('/armleft', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({running: desired})
  }).then(r => r.json()).then(d => {
    armleftRunning = d.running;
    document.getElementById('armleft-toggle').checked = armleftRunning;
  }).catch(() => {
    document.getElementById('armleft-toggle').checked = armleftRunning;
  });
}
// 페이지 로드 시 실제 서버 상태로 초기화
fetch('/armleft_status').then(r => r.json()).then(d => {
  armleftRunning = d.running;
  document.getElementById('armleft-toggle').checked = armleftRunning;
});

let socialNavOn = false;   // 기본 비활성 (서버 디폴트와 일치)
function toggleSocialNav() {
  fetch('/toggle_social_nav', {method:'POST'}).then(r => r.json()).then(d => {
    socialNavOn = d.enabled;
    const t = document.getElementById('social-nav-toggle');
    if (t) t.checked = socialNavOn;
  });
}
// toggleObstaclePush 제거 (2026-07-24) — UI 토글 삭제됨

function switchMap() {
  const f   = document.getElementById('floor-sel').value;
  const ini = document.getElementById('init-exit-chk').checked;
  if (!confirm(`${f}층 지도로 전환할까요?` + (ini ? ' (AMCL 위치가 엘리베이터 하차지점으로 초기화됩니다)' : ''))) return;
  fetch('/switch_map', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({floor: f, init_exit: ini})})
    .then(r => r.json())
    .then(d => { if (!d.ok) alert('지도 전환 실패: ' + d.error); })
    .catch(() => alert('지도 전환 요청 실패'));
}

function queryPose() {
  fetch('/robot_pose').then(r => r.json()).then(d => {
    const el = document.getElementById('pose-result');
    if (!d.ok) { el.style.color = '#f66'; el.textContent = d.error; return; }
    el.style.color = '#8ac';
    // location.yaml에 그대로 붙여넣는 형식 (클릭하면 전체 선택됨)
    el.textContent = `x: ${d.x}  y: ${d.y}  z: ${d.z}  w: ${d.w}  (${d.yaw_deg}°)`;
  }).catch(() => {});
}

function toggleElevAuthority() {
  const on = document.getElementById('elev-auth-toggle').checked;
  fetch('/elev_authority', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({granted: on})});
}

let offlineMode = false;
function toggleNetMode() {
  offlineMode = !document.getElementById('net-mode-toggle').checked;
  fetch('/net_mode', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({offline: offlineMode})});
}
function setRobotSpeed(v) {
  document.getElementById('robot-speed-val').textContent = parseFloat(v).toFixed(2);
  fetch('/robot_speed', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({speed: parseFloat(v)})});
}
function setTtsSpeed(v) {
  document.getElementById('tts-speed-val').textContent = parseFloat(v).toFixed(1);
  fetch('/tts_speed', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({speed: parseFloat(v)})});
}

// 로그 SSE + 탭 필터
const logPanel = document.getElementById('log-panel');
let allLogs = [];
let currentTab = 'ALL';
const SOURCES = ['IFACE','VISION','HW','MAIN','WEB','ARM','SYS','ROS2','RVIZ','BATT','FREE','HOME','ARD','MAP','POSE','OBSTACLE'];
const unread = Object.fromEntries(SOURCES.map(s => [s, 0]));

function setTab(tab) {
  currentTab = tab;
  if (tab === 'ALL') SOURCES.forEach(s => { unread[s] = 0; });
  else unread[tab] = 0;
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
  document.getElementById('tab-' + tab).classList.add('active');
  renderLogs();
  updateBadges();
}

function renderLogs() {
  const filtered = currentTab === 'ALL' ? allLogs : allLogs.filter(d => d.src === currentTab);
  logPanel.innerHTML = '';
  filtered.forEach(appendLogEntry);
  logPanel.scrollTop = logPanel.scrollHeight;
}

function appendLogEntry(d) {
  const div = document.createElement('div');
  div.className = 'log-entry';
  div.innerHTML = `<span class="ts">${d.t}</span><span class="src-${d.src}">[${d.src}]</span> ${escHtml(d.msg)}`;
  logPanel.appendChild(div);
}

function updateBadges() {
  SOURCES.forEach(s => {
    const b = document.getElementById('badge-' + s);
    if (!b) return;
    // 숫자 없이 점(dot)만 표시 — 읽지 않은 메시지 있을 때만
    b.style.display = unread[s] > 0 ? 'inline-block' : 'none';
  });
}

const es = new EventSource('/logs');
es.onmessage = e => {
  const d = JSON.parse(e.data);
  allLogs.push(d);
  if (currentTab === 'ALL' || d.src === currentTab) {
    appendLogEntry(d);
    logPanel.scrollTop = logPanel.scrollHeight;
  } else {
    unread[d.src] = (unread[d.src] || 0) + 1;
    updateBadges();
  }
};


function escHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

// 의자 감지 상태 폴링
const DECISION_LABEL = {push:'밀기', detour:'우회', probing:'탐침 중'};
setInterval(() => {
  fetch('/obstacle_status').then(r => r.json()).then(d => {
    const card   = document.getElementById('chair-card');
    const dot    = document.getElementById('chair-dot');
    const status = document.getElementById('chair-status');
    const detail = document.getElementById('chair-detail');
    if (d.detected) {
      card.classList.add('detected');
      dot.classList.add('on');
      status.textContent = `의자 감지 (${d.dist ?? '?'} m)`;
      const dec = d.decision ? DECISION_LABEL[d.decision] || d.decision : '판단 대기';
      detail.textContent = `결정: ${dec}`;
    } else {
      card.classList.remove('detected');
      dot.classList.remove('on');
      status.textContent = '감지 없음';
      detail.textContent = '';
    }
  }).catch(() => {});
}, 1000);

// 엘리베이터 제어권 토글을 서버 상태와 동기화 (수동 전환 자동 회수 등 반영)
setInterval(() => {
  fetch('/elev_authority').then(r => r.json()).then(d => {
    const t = document.getElementById('elev-auth-toggle');
    if (t && document.activeElement !== t) t.checked = !!d.granted;
  }).catch(() => {});
}, 2000);

// ── 시스템 프로세스 ──────────────────────────────────────────────────────────
const SYS_NAMES = ['launch','rviz','battery','free','home'];
const sysState = Object.fromEntries(SYS_NAMES.map(n => [n, false]));

function toggleSysProc(name) {
  const running = sysState[name];
  fetch(`/sys_proc/${name}`, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action: 'toggle'})
  }).then(r => r.json()).then(d => updateSysProcUI(name, d.running));
}

function updateSysProcUI(name, running) {
  sysState[name] = running;
  const dot = document.getElementById('sysdot-' + name);
  const btn = document.getElementById('sysbtn-' + name);
  if (!dot || !btn) return;
  const oneShot = ['battery','free','home'].includes(name);
  if (running) {
    dot.classList.add('on');
    btn.textContent = '종료';
    btn.className = 'btn btn-danger btn-sm';
  } else {
    dot.classList.remove('on');
    btn.className = oneShot ? 'btn btn-neutral btn-sm' : 'btn btn-primary btn-sm';
    btn.textContent = oneShot ? '실행' : '시작';
  }
}

function pollSysProcStatus() {
  fetch('/sys_proc_status').then(r => r.json()).then(d => {
    SYS_NAMES.forEach(name => updateSysProcUI(name, d[name] || false));
  }).catch(() => {});
}
pollSysProcStatus();
setInterval(pollSysProcStatus, 2000);

function killAllRobot() {
  const btn = document.getElementById('killall-btn');
  if (!confirm('터미널 포함 모든 로봇 프로세스를 종료합니다. 계속할까요?')) return;
  btn.disabled = true;
  btn.textContent = '정리 중...';
  fetch('/killall_robot', {method:'POST'})
    .then(r => r.json())
    .then(() => {
      setTimeout(() => {
        btn.disabled = false;
        btn.textContent = '전체 로봇 프로세스 강제 종료';
        pollSysProcStatus();
      }, 5000);
    });
}

// 키보드 지원
//   방향키      : 이동 토글 (같은 키 다시 = 정지)  [수동 모드]
//   키패드 +,-  : 속도 조절                        [수동 모드]
//   1~4         : 기능 설정 토글                   [항상]
//   Shift+1~5   : 시스템 프로세스                  [항상, 종료 시 confirm]
const keyToDir = {'ArrowUp':['fwd',1,0],'ArrowDown':['bwd',-0.5,0],
                  'ArrowLeft':['left',0,1],'ArrowRight':['right',0,-1]};
const FEATURE_KEYS = {1:'armleft-toggle', 2:'social-nav-toggle',
                      4:'net-mode-toggle'};   // 3(장애물 밀기) 제거 (2026-07-24)
const SYS_KEYS   = {1:'launch', 2:'rviz', 3:'battery', 4:'free', 5:'home'};
const SYS_LABELS = {launch:'ROS2 Launch', rviz:'RViz2', battery:'배터리 확인',
                    free:'프로세스 정리', home:'홈 위치'};

// e.code 기반이라 한/영 상태와 무관하게 동작 (Digit1/Numpad1 → '1')
function digitOf(e) {
  const m = e.code.match(/^(?:Digit|Numpad)([1-9])$/);
  return m ? m[1] : null;
}

document.addEventListener('keydown', e => {
  if (/^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName) && e.target.type !== 'range'
      && e.target.type !== 'checkbox') return;

  const digit = digitOf(e);

  // Shift+숫자 : 시스템 프로세스
  if (e.shiftKey && digit && SYS_KEYS[digit]) {
    if (e.repeat) return;
    const name = SYS_KEYS[digit];
    if (sysState[name] && !confirm(`${SYS_LABELS[name]}을(를) 종료할까요?`)) return;
    toggleSysProc(name);
    e.preventDefault();
    return;
  }

  // 숫자 : 기능 설정 토글
  if (!e.shiftKey && digit && FEATURE_KEYS[digit]) {
    if (e.repeat) return;
    const cb = document.getElementById(FEATURE_KEYS[digit]);
    if (cb) cb.click();  // checked 토글 + onchange 핸들러 실행
    e.preventDefault();
    return;
  }

  // 이하 수동 모드 전용
  if (!manualMode) return;
  if (keyToDir[e.key]) {
    if (e.repeat) return;  // 키를 꾹 눌러도 토글이 반복되지 않게
    const [dir, lx, az] = keyToDir[e.key];
    toggleCmd(dir, lx, az);
    e.preventDefault();
  } else if (e.key === '+') {
    setSpeed(parseInt(document.getElementById('speed').value) + 5);
    e.preventDefault();
  } else if (e.key === '-') {
    setSpeed(parseInt(document.getElementById('speed').value) - 5);
    e.preventDefault();
  }
});
</script>
</body>
</html>"""

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
        _diaglog = _diag.DiagLogger("system")
        _diaglog.boot_snapshot()
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
