#!/usr/bin/env python3
"""
엘리베이터 버튼 추적 메인 스크립트

실행: python3 main.py
브라우저: http://localhost:5000
"""

import os, json, subprocess, tempfile, threading, webbrowser
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from flask import Flask, Response, jsonify, request, render_template_string

VENV_PYTHON   = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../venv/bin/python3"))
INFER_SCRIPT  = os.path.join(os.path.dirname(__file__), "ocr_rcnn_infer.py")
INFER_SERVER  = os.path.join(os.path.dirname(__file__), "ocr_rcnn_server.py")

# 그리퍼 카메라 고정 자세 (전방을 향하는 값)
WRIST_PITCH_DEFAULT = 0.07
WRIST_YAW_DEFAULT   = 0.03

# 영구 추론 프로세스 (모델을 한 번만 로딩)
_infer_proc = None
_infer_lock = threading.Lock()


def start_infer_server():
    global _infer_proc
    print("OCR 모델 로딩 중... (최초 1회, 수초 소요)", flush=True)
    _infer_proc = subprocess.Popen(
        [VENV_PYTHON, INFER_SERVER],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    # ButtonRecognizer가 stdout에 초기화 메시지를 출력할 수 있으므로
    # "READY" 줄이 나올 때까지 읽어서 무시
    for _ in range(20):
        line = _infer_proc.stdout.readline().strip()
        if line == "READY":
            print("OCR 모델 준비 완료.", flush=True)
            return True
    print("OCR 서버 시작 타임아웃", flush=True)
    return False


def infer_image(image_path: str) -> list:
    with _infer_lock:
        _infer_proc.stdin.write(image_path + "\n")
        _infer_proc.stdin.flush()
        line = _infer_proc.stdout.readline()
    return json.loads(line).get("detections", [])

# 두 카메라 동시 표시:
#   gripper(D405) — 버튼 인식/추적/거리 (OCR 대상)
#   body(D435i)   — 전방 모니터링 (멀리서 패널 찾기용, 표시만)
# D405 color는 보정된 image_rect_raw 토픽으로 발행됨 (확인: ros2 topic hz)
GRIPPER_TOPIC = "/gripper_camera/color/image_rect_raw"
BODY_TOPIC    = "/camera/camera/color/image_raw"

IMAGE_W, IMAGE_H = 640, 480
CX, CY    = IMAGE_W // 2, IMAGE_H // 2
DEAD_ZONE = 40
KP_YAW    = 0.0008
KP_LIFT   = 0.0003

# D405 그리퍼 카메라가 90° 돌아간 상태로 장착됨 → 정방향으로 보정.
# 화면이 반대 방향으로 돌아가 있으면 아래 값을 cv2.ROTATE_90_COUNTERCLOCKWISE 로 바꾸세요.
# (선택지: None / cv2.ROTATE_90_CLOCKWISE / cv2.ROTATE_90_COUNTERCLOCKWISE / cv2.ROTATE_180)
# image_rect_raw(보정 스트림)는 이미 정방향 → 회전 불필요 (CW 넣었더니 오히려 90° 돌아갔음)
CAMERA_ROTATE = None
# body(D435i) 머리 카메라는 90° 회전 장착 (people_tracker와 동일). 방향 틀리면 바꾸세요.
BODY_ROTATE   = cv2.ROTATE_90_CLOCKWISE

# ── 공유 상태 ─────────────────────────────────────────────────────────
state = {
    "phase":        "SELECT",
    "target_text":  None,
    "detections":   [],
    "centered":     False,
    "lift":         None,
    "yaw":          None,
    "jpeg_frame":      None,   # gripper 주석 프레임 (버튼 박스/거리)
    "jpeg_frame_body": None,   # body 원본 프레임 (모니터링)
    "target_dist":  None,   # 목표 버튼까지 거리 (m, D405 depth)
}
state_lock = threading.Lock()

# ── HTML ─────────────────────────────────────────────────────────────
HTML = """
<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
  <title>Elevator Button Tracker</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #111; color: #eee; font-family: sans-serif;
           display: flex; flex-direction: column; align-items: center;
           padding: 20px; gap: 14px; }
    h2   { font-size: 1.2rem; color: #aef; }
    #banner { font-size: 1rem; padding: 8px 20px; border-radius: 6px;
              background: #222; color: #fff; min-width: 500px; text-align: center; }
    #banner.tracking { background: #1a2a7a; }
    #banner.centered { background: #1a5a1a; }
    #cam-row { display: flex; gap: 10px; align-items: center; font-size: 0.9rem; color: #aaa; }
    #cam-row select { background: #333; color: #eee; border: 1px solid #555;
                      padding: 5px 10px; border-radius: 6px; font-size: 0.9rem; cursor: pointer; }
    #stream { width: 640px; height: 480px; display: block; background: #000; }
    #buttons { display: flex; flex-wrap: wrap; gap: 10px; justify-content: center;
               max-width: 660px; }
    .btn { padding: 10px 20px; border: none; border-radius: 8px; font-size: 1.1rem;
           cursor: pointer; background: #2a5; color: #fff; transition: transform .1s; }
    .btn:hover { transform: scale(1.08); background: #3b6; }
    .btn.target { background: #d33; outline: 3px solid #f88; }
    #reset { background: #555; padding: 8px 18px; border: none; border-radius: 6px;
             color: #eee; cursor: pointer; font-size: 0.9rem; }
    #reset:hover { background: #777; }
    #pose-btn { background: #336; padding: 8px 18px; border: none; border-radius: 6px;
                color: #ccf; cursor: pointer; font-size: 0.9rem; }
    #pose-btn:hover { background: #558; }
    #pitch-row { display:flex; gap:8px; align-items:center; font-size:0.9rem; color:#aaa; }
    #pitch-row input[type=range] { width:220px; accent-color:#88f; }
    #pitch-row input[type=number] { width:70px; background:#333; color:#eee;
                                    border:1px solid #555; border-radius:4px;
                                    padding:4px 6px; font-size:0.9rem; }
    #pitch-apply { background:#446; color:#ccf; border:none; border-radius:4px;
                   padding:5px 12px; cursor:pointer; font-size:0.9rem; }
    #pitch-apply:hover { background:#668; }
    #info { font-size: 0.8rem; color: #666; }
  </style>
</head>
<body>
  <h2>🛗 Elevator Button Tracker</h2>

  <div id="banner">Waiting for camera...</div>
  <div style="display:flex; gap:14px; align-items:flex-start; justify-content:center;">
    <figure style="margin:0; text-align:center;">
      <img id="stream" src="/video?cam=gripper" alt="gripper stream">
      <figcaption style="color:#8ad; font-size:0.85rem; margin-top:4px;">
        🤏 gripper (D405) — 버튼 인식·추적·거리</figcaption>
    </figure>
    <figure style="margin:0; text-align:center;">
      <img id="stream-body" src="/video?cam=body" alt="body stream"
           style="height:480px; background:#000;">
      <figcaption style="color:#8ad; font-size:0.85rem; margin-top:4px;">
        👁 body (D435i) — 전방 모니터링</figcaption>
    </figure>
  </div>

  <div id="buttons"></div>
  <div style="display:flex;gap:10px;align-items:center;">
    <button id="reset" onclick="resetTarget()">Reset (재선택)</button>
    <button id="pose-btn" onclick="setWristForward()">그리퍼 전방 고정 (0.0)</button>
  </div>

  <div id="pitch-row">
    wrist_pitch:
    <input type="range" id="pitch-slider" min="-1.57" max="0.5" step="0.01" value="0.07"
           oninput="document.getElementById('pitch-val').value=parseFloat(this.value).toFixed(2)">
    <input type="number" id="pitch-val" value="0.07" step="0.01" min="-1.57" max="0.5"
           oninput="document.getElementById('pitch-slider').value=this.value">
    <button id="pitch-apply" onclick="applyPitch()">적용</button>
  </div>
  <div id="pitch-row">
    wrist_yaw &nbsp;:
    <input type="range" id="yaw-slider" min="-2.88" max="1.67" step="0.01" value="0.03"
           oninput="document.getElementById('yaw-val').value=parseFloat(this.value).toFixed(2)">
    <input type="number" id="yaw-val" value="0.03" step="0.01" min="-2.88" max="1.67"
           oninput="document.getElementById('yaw-slider').value=this.value">
    <button id="pitch-apply" onclick="applyYaw()">적용</button>
  </div>
  <div id="info">감지된 버튼을 클릭하면 추적을 시작합니다.</div>

  <script>
    function selectButton(text) {
      fetch('/select', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({text})});
    }
    function resetTarget() { fetch('/reset', {method:'POST'}); }
    function setWristForward() {
      const pitch = 0.07, yaw = 0.03;
      document.getElementById('pitch-slider').value = pitch;
      document.getElementById('pitch-val').value = pitch.toFixed(2);
      document.getElementById('yaw-slider').value = yaw;
      document.getElementById('yaw-val').value = yaw.toFixed(2);
      fetch('/wrist_pitch', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({pitch})});
      fetch('/wrist_yaw',   {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({yaw})});
    }
    function applyPitch() {
      const v = parseFloat(document.getElementById('pitch-val').value);
      fetch('/wrist_pitch', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({pitch: v})});
    }
    function applyYaw() {
      const v = parseFloat(document.getElementById('yaw-val').value);
      fetch('/wrist_yaw', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({yaw: v})});
    }
    function poll() {
      fetch('/status').then(r => r.json()).then(s => {
        const banner = document.getElementById('banner');
        if (s.centered) {
          banner.textContent = `✅ CENTERED — '${s.target}' 도달  |  거리 ${s.dist!=null ? s.dist+'m' : '?'}`;
          banner.className = 'centered';
        } else if (s.target) {
          banner.textContent = `🎯 TRACKING '${s.target}'  |  x:${s.ex??'?'}  y:${s.ey??'?'}  |  거리 ${s.dist!=null ? s.dist+'m' : '?'}`;
          banner.className = 'tracking';
        } else {
          banner.textContent = '아래에서 목표 버튼을 클릭하세요';
          banner.className = '';
        }

        const box = document.getElementById('buttons');
        box.innerHTML = '';
        s.detections.forEach(d => {
          const b = document.createElement('button');
          b.className = 'btn' + (d.text === s.target ? ' target' : '');
          b.textContent = d.text;
          b.title = `score: ${d.score}`;
          b.onclick = () => selectButton(d.text);
          box.appendChild(b);
        });

      }).catch(()=>{});
    }
    setInterval(poll, 600);
    poll();
  </script>
</body>
</html>
"""

# ── Flask ─────────────────────────────────────────────────────────────
app = Flask(__name__)

@app.route("/")
def index():
    from flask import render_template_string
    return render_template_string(HTML)

@app.route("/video")
def video():
    # ?cam=gripper (기본) 또는 ?cam=body — 두 스트림을 동시에 제공
    cam = request.args.get("cam", "gripper")
    key = "jpeg_frame_body" if cam == "body" else "jpeg_frame"
    label = "body (D435i)" if cam == "body" else "gripper (D405)"
    def gen():
        while True:
            with state_lock:
                frame = state[key]
            if frame is None:
                blank = np.zeros((IMAGE_H, IMAGE_W, 3), dtype=np.uint8)
                cv2.putText(blank, f"Waiting for {label}...", (110, 230),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (180, 180, 180), 2)
                cv2.putText(blank, "Check ROS2 camera topic", (150, 270),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
                _, enc = cv2.imencode(".jpg", blank)
                frame = enc.tobytes()
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            threading.Event().wait(0.1)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/status")
def status():
    with state_lock:
        s = state.copy()
    target = s["target_text"]
    ex = ey = None
    if target:
        det = next((d for d in s["detections"] if d["text"] == target), None)
        if det:
            b = det["box"]
            ex = round((b["x1"]+b["x2"])/2 - CX)
            ey = round((b["y1"]+b["y2"])/2 - CY)
    return jsonify(phase=s["phase"], target=target, detections=s["detections"],
                   centered=s["centered"], ex=ex, ey=ey,
                   dist=s.get("target_dist"))

@app.route("/select", methods=["POST"])
def select():
    text = request.json.get("text", "")
    with state_lock:
        state["target_text"] = text
        state["phase"]       = "TRACK"
        state["centered"]    = False
    return jsonify(ok=True)

@app.route("/reset", methods=["POST"])
def reset():
    with state_lock:
        state["target_text"] = None
        state["phase"]       = "SELECT"
        state["centered"]    = False
    return jsonify(ok=True)

@app.route("/wrist_forward", methods=["POST"])
def wrist_forward():
    node = _node_ref[0]
    if node:
        node.set_wrist_pitch(WRIST_PITCH_DEFAULT)
        node.set_wrist_yaw(WRIST_YAW_DEFAULT)
    return jsonify(ok=True)

@app.route("/wrist_pitch", methods=["POST"])
def wrist_pitch():
    pitch = float(request.json.get("pitch", 0.0))
    pitch = max(-1.57, min(0.5, pitch))
    node = _node_ref[0]
    if node:
        node.set_wrist_pitch(pitch)
    return jsonify(ok=True, pitch=pitch)

@app.route("/wrist_yaw", methods=["POST"])
def wrist_yaw():
    yaw = float(request.json.get("yaw", 0.0))
    yaw = max(-2.88, min(1.67, yaw))
    node = _node_ref[0]
    if node:
        node.set_wrist_yaw(yaw)
    return jsonify(ok=True, yaw=yaw)

_node_ref = [None]

# ── ROS2 노드 ─────────────────────────────────────────────────────────
class ElevatorTracker(Node):
    def __init__(self):
        super().__init__("elevator_tracker")
        self.bridge         = CvBridge()
        self._processing    = False
        self._goal_done     = True

        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)

        # ── D405 depth (color 정렬) — 버튼까지 거리 측정용 (press 준비 0단계) ──
        # 로봇을 움직이지 않음. 화면에 거리 숫자만 표시.
        self._depth_frame = None   # 회전 보정된 uint16 배열 (mm)
        self._depth_lock  = threading.Lock()
        self.create_subscription(
            Image, "/gripper_camera/aligned_depth_to_color/image_raw",
            self._on_depth, qos_profile_sensor_data,
        )

        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory"
        )
        # 시작 3초 후 wrist 초기 자세 자동 설정 (액션 서버 준비 대기)
        self.create_timer(3.0, self._init_wrist_once)
        self._wrist_initialized = False

        # 두 카메라 동시 구독 (전환 없음)
        self.create_subscription(Image, GRIPPER_TOPIC, self._on_image, 10)
        self.create_subscription(Image, BODY_TOPIC, self._on_image_body,
                                 qos_profile_sensor_data)
        self.get_logger().info(f"gripper: {GRIPPER_TOPIC}")
        self.get_logger().info(f"body   : {BODY_TOPIC}")
        self.get_logger().info("Ready — open http://localhost:5000")

    def _init_wrist_once(self):
        if self._wrist_initialized:
            return
        self._wrist_initialized = True
        self.set_wrist_pitch(WRIST_PITCH_DEFAULT)
        self.set_wrist_yaw(WRIST_YAW_DEFAULT)
        self.get_logger().info(
            f"Wrist initialized: pitch={WRIST_PITCH_DEFAULT}, yaw={WRIST_YAW_DEFAULT}"
        )

    def _on_image_body(self, msg):
        """body(D435i) 프레임 — 모니터링용 표시만 (OCR/제어 없음)."""
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if BODY_ROTATE is not None:
                raw = cv2.rotate(raw, BODY_ROTATE)
            # 회전 후 세로형 비율 유지, 높이 480에 맞춤
            h, w = raw.shape[:2]
            frame = cv2.resize(raw, (max(1, int(w * IMAGE_H / h)), IMAGE_H))
            _, jpeg = cv2.imencode(".jpg", frame)
            with state_lock:
                state["jpeg_frame_body"] = jpeg.tobytes()
        except Exception as e:
            self.get_logger().warn(f"body 프레임 오류: {e}")

    def _on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == "joint_lift":
                with state_lock: state["lift"] = pos
            elif name == "joint_wrist_yaw":
                with state_lock: state["yaw"] = pos

    def _on_depth(self, msg):
        """aligned depth 프레임 저장. color와 동일하게 회전 보정해 좌표계를 맞춤."""
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                (msg.height, msg.width))
            if CAMERA_ROTATE is not None:
                arr = cv2.rotate(arr, CAMERA_ROTATE)
            with self._depth_lock:
                self._depth_frame = arr
        except Exception as e:
            self.get_logger().warn(f"depth 오류: {e}")

    def _depth_at(self, x_disp: int, y_disp: int) -> float | None:
        """
        표시 프레임(640x480) 좌표 → depth 배열 좌표로 변환 후
        11x11 패치 중앙값 거리(m) 반환. 측정 불가 시 None.
        """
        with self._depth_lock:
            d = self._depth_frame
        if d is None:
            return None
        h, w = d.shape
        u = int(x_disp * w / IMAGE_W)
        v = int(y_disp * h / IMAGE_H)
        u = max(0, min(u, w - 1))
        v = max(0, min(v, h - 1))
        patch = d[max(0, v - 5):v + 6, max(0, u - 5):u + 6]
        valid = patch[patch > 0]
        if len(valid) < 5:
            return None
        D = float(np.median(valid)) / 1000.0
        # D405 근거리 특화: 5cm~1.5m 범위만 신뢰
        return D if 0.05 < D < 1.5 else None

    def _on_image(self, msg):
        try:
            raw   = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # 회전 보정을 리사이즈 전에 적용 → 화면/OCR/서보가 모두 같은 정방향 프레임 사용
            if CAMERA_ROTATE is not None:
                raw = cv2.rotate(raw, CAMERA_ROTATE)
            frame = cv2.resize(raw, (IMAGE_W, IMAGE_H))
        except Exception as e:
            self.get_logger().error(str(e))
            return

        # 매 프레임 즉시 화면 업데이트 (직전 추론 결과 바운딩박스 재사용)
        with state_lock:
            detections = state["detections"]
        annotated = self._annotate(frame, detections)
        _, jpeg = cv2.imencode(".jpg", annotated)
        with state_lock:
            state["jpeg_frame"] = jpeg.tobytes()

        # OCR 추론은 백그라운드 스레드에서 (이전 추론 중이면 건너뜀)
        if not self._processing:
            self._processing = True
            threading.Thread(
                target=self._run_inference, args=(frame.copy(),), daemon=True
            ).start()

    def _run_inference(self, frame):
        try:
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as f:
                tmp = f.name
            cv2.imwrite(tmp, frame)
            detections = infer_image(tmp)
            os.unlink(tmp)

            with state_lock:
                state["detections"] = detections
                phase    = state["phase"]
                centered = state["centered"]
                lift     = state["lift"]

            if phase == "TRACK" and not centered and self._goal_done and lift is not None:
                self._servo_step(detections, lift)

        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            self._processing = False

    def _annotate(self, frame, detections):
        vis = frame.copy()
        cv2.line(vis, (CX-30, CY), (CX+30, CY), (255,255,0), 1)
        cv2.line(vis, (CX, CY-30), (CX, CY+30), (255,255,0), 1)
        cv2.circle(vis, (CX, CY), DEAD_ZONE, (255,255,0), 1)

        with state_lock:
            target = state["target_text"]; phase = state["phase"]
            centered = state["centered"]

        for d in detections:
            b = d["box"]
            x1,y1,x2,y2 = int(b["x1"]),int(b["y1"]),int(b["x2"]),int(b["y2"])
            bcx,bcy = (x1+x2)//2,(y1+y2)//2
            is_target = (phase == "TRACK" and d["text"] == target)
            color = (0,0,255) if is_target else (0,210,80)
            cv2.rectangle(vis, (x1,y1), (x2,y2), color, 3 if is_target else 2)
            label = f"{d['text']} {d['score']:.2f}"
            cv2.rectangle(vis, (x1,y1-20), (x1+len(label)*9,y1), color, -1)
            cv2.putText(vis, label, (x1+2,y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)
            if is_target:
                cv2.arrowedLine(vis, (CX,CY), (bcx,bcy), (0,80,255), 2, tipLength=0.2)
                # 버튼까지 거리 (D405 depth) — press 준비용 표시
                dist = self._depth_at(bcx, bcy)
                with state_lock:
                    state["target_dist"] = round(dist, 3) if dist else None
                dist_txt = f"{dist:.2f}m" if dist else "?m"
                cv2.putText(vis, dist_txt, (x2+6, bcy+6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                cv2.putText(vis, f"x:{bcx-CX:+d} y:{bcy-CY:+d} d:{dist_txt}", (10,IMAGE_H-12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,80,255), 2)
        if centered:
            cv2.putText(vis, "CENTERED", (CX-70,CY-50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,80), 3)
        return vis

    def _servo_step(self, detections, lift):
        with state_lock:
            target = state["target_text"]
        det = next((d for d in detections if d["text"] == target), None)
        if not det: return
        b  = det["box"]
        ey = (b["y1"]+b["y2"])/2 - CY   # 상하 오차만 사용
        if abs(ey) < DEAD_ZONE:
            with state_lock: state["centered"] = True
            self.get_logger().info("CENTERED!")
            return
        # lift만 조정 — wrist pitch/yaw는 고정
        new_lift = max(0.15, min(1.10, float(lift) - KP_LIFT * ey))
        self._send_goal(new_lift)

    def _send_single_joint(self, joint_name: str, position: float, duration_sec: int = 2):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server not available")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint_name]
        pt = JointTrajectoryPoint()
        pt.positions = [position]
        pt.time_from_start.sec = duration_sec
        goal.trajectory.points = [pt]
        self.action_client.send_goal_async(goal)
        self.get_logger().info(f"{joint_name} -> {position:.2f} rad")

    def set_wrist_pitch(self, pitch: float):
        self._send_single_joint("joint_wrist_pitch", pitch)

    def set_wrist_yaw(self, yaw: float):
        self._send_single_joint("joint_wrist_yaw", yaw)

    def _send_goal(self, lift):
        if not self.action_client.wait_for_server(timeout_sec=1.0): return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_lift"]
        pt = JointTrajectoryPoint()
        pt.positions = [lift]
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]
        self._goal_done = False
        self.action_client.send_goal_async(goal).add_done_callback(self._on_goal_response)
        self.get_logger().info(f"-> lift={lift:.3f}")

    def _on_goal_response(self, fut):
        h = fut.result()
        if h.accepted:
            h.get_result_async().add_done_callback(lambda _: setattr(self,"_goal_done",True))
        else:
            self._goal_done = True


def main():
    if not start_infer_server():
        print("추론 서버 시작 실패. 종료합니다.")
        return

    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False),
        daemon=True,
    ).start()

    # 브라우저 자동 오픈 (Flask 뜰 시간 잠깐 준 뒤)
    threading.Timer(1.5, lambda: webbrowser.open("http://localhost:5000")).start()
    print("대시보드: http://localhost:5000", flush=True)

    rclpy.init()
    node = ElevatorTracker()
    _node_ref[0] = node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass


if __name__ == "__main__":
    main()
