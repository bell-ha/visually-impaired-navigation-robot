#!/usr/bin/env python3
"""
엘리베이터 버튼 추적 메인 스크립트

실행: python3 main.py
브라우저: http://localhost:5000
"""

import os, json, subprocess, tempfile, threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from flask import Flask, Response, jsonify, request, render_template_string

VENV_PYTHON   = os.path.expanduser("~/venv_ocr/bin/python3")
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

CAMERA_OPTIONS = {
    "gripper (D405)": "/gripper_camera/color/image_raw",
    "body (D435i)":   "/camera/camera/color/image_rect_raw",
}

IMAGE_W, IMAGE_H = 640, 480
CX, CY    = IMAGE_W // 2, IMAGE_H // 2
DEAD_ZONE = 40
KP_YAW    = 0.0008
KP_LIFT   = 0.0003

# ── 공유 상태 ─────────────────────────────────────────────────────────
state = {
    "phase":        "SELECT",
    "target_text":  None,
    "detections":   [],
    "centered":     False,
    "lift":         None,
    "yaw":          None,
    "jpeg_frame":   None,
    "camera_key":   "gripper (D405)",
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

  <div id="cam-row">
    카메라:
    <select id="cam-sel" onchange="switchCamera(this.value)">
      {% for key in cameras %}
      <option value="{{ key }}">{{ key }}</option>
      {% endfor %}
    </select>
  </div>

  <div id="banner">Waiting for camera...</div>
  <img id="stream" src="/video" alt="camera stream">

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
    function switchCamera(key) {
      fetch('/camera', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({key})});
      // 스트림 재연결
      const img = document.getElementById('stream');
      img.src = '/video?' + Date.now();
    }

    function poll() {
      fetch('/status').then(r => r.json()).then(s => {
        const banner = document.getElementById('banner');
        if (s.centered) {
          banner.textContent = `✅ CENTERED — '${s.target}' 도달`;
          banner.className = 'centered';
        } else if (s.target) {
          banner.textContent = `🎯 TRACKING '${s.target}'  |  x:${s.ex??'?'}  y:${s.ey??'?'}`;
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

        // 카메라 선택 동기화
        document.getElementById('cam-sel').value = s.camera_key;
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
    return render_template_string(HTML, cameras=list(CAMERA_OPTIONS.keys()))

@app.route("/video")
def video():
    def gen():
        while True:
            with state_lock:
                frame = state["jpeg_frame"]
            if frame is None:
                blank = np.zeros((IMAGE_H, IMAGE_W, 3), dtype=np.uint8)
                cv2.putText(blank, "Waiting for camera...", (130, 230),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
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
                   centered=s["centered"], ex=ex, ey=ey, camera_key=s["camera_key"])

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

@app.route("/camera", methods=["POST"])
def camera():
    key = request.json.get("key", "")
    if key in CAMERA_OPTIONS:
        with state_lock:
            state["camera_key"]  = key
            state["jpeg_frame"]  = None   # 스트림 리셋
        # 노드에 카메라 변경 신호
        _node_ref[0].switch_camera(CAMERA_OPTIONS[key])
    return jsonify(ok=True)

_node_ref = [None]

# ── ROS2 노드 ─────────────────────────────────────────────────────────
class ElevatorTracker(Node):
    def __init__(self):
        super().__init__("elevator_tracker")
        self.bridge         = CvBridge()
        self._processing    = False
        self._goal_done     = True
        self._img_sub       = None
        self._pending_topic = None   # Flask 스레드 → ROS2 스레드 카메라 전환 요청

        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory"
        )
        # 타이머로 카메라 전환 처리 (ROS2 스핀 스레드에서 안전하게 실행)
        self.create_timer(0.2, self._apply_pending_camera)
        # 시작 3초 후 wrist 초기 자세 자동 설정 (액션 서버 준비 대기)
        self.create_timer(3.0, self._init_wrist_once)
        self._wrist_initialized = False
        # 기본 카메라로 구독 시작
        self._do_switch_camera(CAMERA_OPTIONS["gripper (D405)"])
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

    def switch_camera(self, topic):
        # Flask 스레드에서 호출 — pending만 설정, 실제 전환은 타이머에서 처리
        self._pending_topic = topic

    def _apply_pending_camera(self):
        # ROS2 스핀 스레드에서 실행되는 타이머 콜백 — 구독 생성/삭제 안전
        if self._pending_topic is None:
            return
        topic = self._pending_topic
        self._pending_topic = None
        self._do_switch_camera(topic)

    def _do_switch_camera(self, topic):
        if self._img_sub:
            self.destroy_subscription(self._img_sub)
            self._img_sub = None
        self._img_sub = self.create_subscription(Image, topic, self._on_image, 10)
        self.get_logger().info(f"Camera switched to: {topic}")

    def _on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == "joint_lift":
                with state_lock: state["lift"] = pos
            elif name == "joint_wrist_yaw":
                with state_lock: state["yaw"] = pos

    def _on_image(self, msg):
        try:
            raw   = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
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
                cv2.putText(vis, f"x:{bcx-CX:+d} y:{bcy-CY:+d}", (10,IMAGE_H-12),
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
