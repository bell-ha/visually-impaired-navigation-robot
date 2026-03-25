#!/usr/bin/env python3
"""
메인 라우터 + 웹 대시보드
- 시리얼 파싱 → interface.py(버튼1/압력) / vision_assistant.py(버튼2)
- 브라우저에서 실시간 로그 확인 + 수동 조작 가능
- 실행: python3 main.py  →  http://localhost:8080 자동 오픈
"""
import collections
import json
import math
import subprocess
import sys
import threading
import time
import webbrowser
from pathlib import Path

import serial
from flask import Flask, Response, jsonify, request

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
THIS_DIR = Path(__file__).resolve().parent
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
# vision_assistant.py 스피커: index 1 (별도 장치) — 두 TTS 동시 재생 가능
VISION_SPEAKER_INDEX = 1

# ── Pull 감지 상수 ────────────────────────────────────────────────────────────
_GRIP_ARM   = 3000
_PULL_TRIG  = 3700
_QUICK_SEC  = 0.25
_GRIP_RESET = 2900
_DEBOUNCE   = 0.25

# ── ROS2 cmd_vel ──────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    _ROS_OK = True
except ImportError:
    _ROS_OK = False

# ── 로그 버퍼 ─────────────────────────────────────────────────────────────────
_LOG_BUF: collections.deque = collections.deque(maxlen=300)
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

def _capture(proc: subprocess.Popen, name: str):
    for line in proc.stdout:
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

def init_ros():
    global _cmd_node, _cmd_pub
    if not _ROS_OK:
        return
    rclpy.init()
    _cmd_node = rclpy.create_node("main_web_cmdvel")
    _cmd_pub  = _cmd_node.create_publisher(Twist, "/stretch/cmd_vel", 10)
    threading.Thread(target=rclpy.spin, args=(_cmd_node,), daemon=True).start()
    _log("MAIN", "ROS2 cmd_vel 퍼블리셔 시작")

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

@app.route("/mode", methods=["POST"])
def set_mode():
    global _manual_mode, _manual_active
    data = request.json or {}
    _manual_mode = data.get("manual", False)
    if _manual_mode:
        # 수동 전환 시: 진행 중인 목적지 취소 + LOCKED 상태로
        _write("iface", "/cancel")
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

# ── 시리얼 루프 ───────────────────────────────────────────────────────────────
def serial_loop():
    pull_detect = _make_pull_detector()
    last_pull_t = 0.0
    ser = None

    while ser is None:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
            _log("MAIN", f"시리얼 연결됨: {SERIAL_PORT}")
        except Exception as e:
            _log("MAIN", f"시리얼 연결 실패({e}), 3초 후 재시도...")
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

            if tag == "TRIG":
                btn = parts[1].strip()
                if btn == "1":
                    _write("iface", "/button")
                    _log("HW", "버튼1")
                elif btn == "2":
                    _write("vision", "/vision")
                    _log("HW", "버튼2 → 시각 분석")

            elif tag == "DATA" and len(parts) >= 4:
                try:
                    pressure = int(parts[3].strip())
                except ValueError:
                    continue
                now = time.monotonic()
                if pull_detect(pressure) and (now - last_pull_t) > _DEBOUNCE:
                    last_pull_t = now
                    _write("iface", "/pull")
                    _log("HW", "당김 감지")
    except Exception as e:
        _log("MAIN", f"시리얼 오류: {e}")
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
  header h1 { font-size: 1.1rem; color: #94a3b8; }
  .badge { padding: 4px 12px; border-radius: 20px; font-size: 0.78rem; font-weight: 600; }
  .badge-auto   { background: #065f46; color: #6ee7b7; }
  .badge-manual { background: #7c2d12; color: #fdba74; }
  .layout { display: grid; grid-template-columns: 1fr 320px; flex: 1; overflow: hidden; }

  /* 로그 패널 */
  #log-panel { padding: 12px; overflow-y: auto; font-family: monospace; font-size: 0.8rem; background: #0f172a; }
  .log-entry { padding: 2px 0; border-bottom: 1px solid #1e293b; white-space: pre-wrap; word-break: break-all; }
  .src-IFACE  { color: #7dd3fc; }
  .src-VISION { color: #86efac; }
  .src-HW     { color: #fbbf24; }
  .src-MAIN   { color: #c084fc; }
  .src-WEB    { color: #fb7185; }
  .ts { color: #475569; margin-right: 6px; }

  /* 컨트롤 패널 */
  .ctrl-panel { background: #1e293b; border-left: 1px solid #334155; padding: 16px; display: flex; flex-direction: column; gap: 16px; overflow-y: auto; }
  .ctrl-panel h2 { font-size: 0.9rem; color: #94a3b8; margin-bottom: 4px; }
  .mode-btns { display: flex; gap: 8px; }
  .btn { border: none; border-radius: 8px; padding: 8px 14px; font-size: 0.82rem; cursor: pointer; font-weight: 600; transition: opacity .15s; }
  .btn:hover { opacity: 0.8; }
  .btn-auto   { background: #059669; color: #fff; }
  .btn-manual { background: #ea580c; color: #fff; }
  .btn-stop   { background: #dc2626; color: #fff; width: 100%; padding: 12px; font-size: 1rem; }
  .btn-nav    { background: #1d4ed8; color: #fff; width: 100%; }
  .btn-vision { background: #7c3aed; color: #fff; width: 100%; }

  /* 방향 패드 */
  .dpad { display: grid; grid-template-columns: repeat(3, 64px); grid-template-rows: repeat(3, 64px); gap: 6px; justify-content: center; }
  .dpad-btn { background: #334155; border: none; border-radius: 10px; color: #e2e8f0; font-size: 1.4rem; cursor: pointer; user-select: none; transition: background .1s; }
  .dpad-btn.pressed { background: #2563eb; box-shadow: 0 0 0 3px #3b82f6; }
  .dpad-center { background: #1e293b; cursor: default; }
  .speed-wrap { display: flex; align-items: center; gap: 8px; }
  .speed-wrap label { font-size: 0.75rem; color: #94a3b8; white-space: nowrap; }
  input[type=range] { flex: 1; accent-color: #3b82f6; }
  .speed-val { font-size: 0.75rem; color: #94a3b8; width: 32px; text-align: right; }
</style>
</head>
<body>
<header>
  <h1>시각장애인 안내 로봇 대시보드</h1>
  <span class="badge badge-auto" id="mode-badge">자동 모드</span>
</header>
<div class="layout">
  <div id="log-panel"></div>
  <div class="ctrl-panel">
    <div>
      <h2>모드 전환</h2>
      <div class="mode-btns">
        <button class="btn btn-auto"   onclick="setMode(false)">자동</button>
        <button class="btn btn-manual" onclick="setMode(true)">수동</button>
      </div>
    </div>

    <div id="manual-section" style="display:none">
      <h2>수동 조작</h2>
      <div class="speed-wrap" style="margin-bottom:10px">
        <label>속도</label>
        <input type="range" id="speed" min="5" max="40" value="15"
               oninput="document.getElementById('spv').textContent=this.value">
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
      <button class="btn btn-stop" style="margin-top:10px" onclick="emergencyStop()">■ 비상 정지</button>
    </div>

    <div>
      <h2>빠른 액션</h2>
      <div style="display:flex;flex-direction:column;gap:8px">
        <button class="btn btn-nav"    onclick="sendButton()">버튼1 (목적지 입력)</button>
        <button class="btn btn-vision" onclick="sendVision()">버튼2 (시각 분석)</button>
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
  if (!manual) _stopMove();  // 자동 전환 시 이동 정지
}

function getSpeed() { return parseInt(document.getElementById('speed').value) / 100; }

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

// 로그 SSE
const logPanel = document.getElementById('log-panel');
const es = new EventSource('/logs');
es.onmessage = e => {
  const d = JSON.parse(e.data);
  const div = document.createElement('div');
  div.className = 'log-entry';
  div.innerHTML =
    `<span class="ts">${d.t}</span>` +
    `<span class="src-${d.src}">[${d.src}]</span> ${escHtml(d.msg)}`;
  logPanel.appendChild(div);
  logPanel.scrollTop = logPanel.scrollHeight;
};

function escHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

// 키보드 지원 (방향키 = 토글)
const keyToDir = {'ArrowUp':['fwd',1,0],'ArrowDown':['bwd',-0.5,0],
                  'ArrowLeft':['left',0,1],'ArrowRight':['right',0,-1]};
document.addEventListener('keydown', e => {
  if (!manualMode || !keyToDir[e.key]) return;
  const [dir, lx, az] = keyToDir[e.key];
  if (activeDir !== dir) toggleCmd(dir, lx, az);
  e.preventDefault();
});
</script>
</body>
</html>"""

# ── 진입점 ────────────────────────────────────────────────────────────────────
def main():
    init_ros()
    start_subprocesses()

    threading.Thread(target=serial_loop, daemon=True).start()
    threading.Thread(target=_manual_loop, daemon=True).start()

    threading.Timer(1.5, lambda: webbrowser.open("http://localhost:8080")).start()
    _log("MAIN", "대시보드: http://localhost:8080")

    app.run(host="0.0.0.0", port=8080, threaded=True)


if __name__ == "__main__":
    main()
