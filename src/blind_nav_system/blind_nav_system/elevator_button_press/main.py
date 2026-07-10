#!/usr/bin/env python3
"""
엘리베이터 버튼 추적 메인 스크립트

실행: python3 main.py
브라우저: http://localhost:5000
"""

import os, json, subprocess, tempfile, threading, time, webbrowser
import collections
import logging
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from flask import Flask, Response, jsonify, request, render_template_string

VENV_PYTHON   = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../venv/bin/python3"))
INFER_SCRIPT  = os.path.join(os.path.dirname(__file__), "ocr_rcnn_infer.py")
INFER_SERVER  = os.path.join(os.path.dirname(__file__), "ocr_rcnn_server.py")

# 그리퍼 카메라 고정 자세 (전방을 향하는 값)
# [실험 C 2026-07-09] 정반사(glare) 회피: 기존 0.07에서 5° 하향(-0.02).
# "중앙 정렬 시에만 인식 실패" = 카메라⊥패널 정반사 기하 → 각도를 깨서 원인 제거.
# 효과 없으면 0.07로 번복. 변경 후 착지 세로 오차는 캘리브레이션 press 1회로 재보정 필요.
WRIST_PITCH_DEFAULT = -0.02
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
# gripper 토픽 이름이 실행 방식/부팅에 따라 달라질 수 있어 후보를 전부 구독하고
# 실제 프레임이 오는 쪽을 사용한다 (한쪽만 발행되므로 충돌 없음).
# 현재 런치(직접 노드 + 시리얼 고정) 기준: /camera/gripper_camera/color/image_rect_raw
GRIPPER_TOPICS = [
    "/camera/gripper_camera/color/image_rect_raw",
    "/camera/gripper_camera/color/image_raw",
    "/gripper_camera/color/image_raw",        # (구버전 include 방식 대비 예비)
    "/gripper_camera/color/image_rect_raw",
]
GRIPPER_DEPTH_TOPICS = [
    "/camera/gripper_camera/aligned_depth_to_color/image_raw",
    "/gripper_camera/aligned_depth_to_color/image_raw",   # (예비)
]
BODY_TOPIC    = "/camera/camera/color/image_raw"

IMAGE_W, IMAGE_H = 640, 480
CX, CY    = IMAGE_W // 2, IMAGE_H // 2
# 지름 2cm 버튼 대응: 30cm 거리에서 15px ≈ ±1cm (기존 40px ≈ ±2.8cm는 버튼보다 컸음)
DEAD_ZONE = 15
# 탐지 기억: 라벨이 한 프레임 사라져도 이 시간 동안 목록·선택 유지 (깜빡임 방지)
# 유지 박스(~표시)로는 로봇이 움직이지 않으므로 길게 잡아도 안전
DET_PERSIST_SEC = 4.0
KP_LIFT   = 0.0003

# 조준 오프셋(픽셀): 카메라 광축과 "닫힌 손끝"의 위치가 달라서 생기는 고정 빗나감 보정.
# 캘리브레이션: 십자 표시된 종이에 press 테스트 → 손끝이 십자에서 (가로, 세로) 몇 cm
# 빗나갔는지 측정 → 30cm 거리 기준 1cm ≈ 14px 로 환산해 아래 값 조정.
# (양수 X = 조준점을 오른쪽으로, 양수 Y = 아래로 이동)
# [캘리브레이션 모델 2026-07-07] 손끝 착지 오프셋 = 거리의 함수 (사용자 아이디어: 1차 보정)
#   조준 오프셋(px) = A/d + B
#   - A/d 항: 카메라↔손끝 고정 물리 오프셋(cm) — 픽셀로는 1/d 로 줄어듦
#   - B 항 : 카메라 광축↔팔 진행축 각도 어긋남 — 픽셀로는 거리 무관 상수
# 실측 3점 최소제곱 fit (0.271m:+16px / 0.35m:-2.3px / 0.427m:-3.7px)
# → 전 구간(0.27~0.43m) 계통 오차 ±3mm 수준. 운용 press 거리는 0.25~0.45m 권장.
AIM_X_A, AIM_X_B = 4.3, 0.0     # 좌우: 고정 ~1cm 오프셋만
# [보정 2026-07-08] ROI 추적 도입 후 "항상 1cm 아래 누름" 실측 → 물리항 +1cm (4.2)
AIM_Y_A, AIM_Y_B = 13.2, -42.0  # 상하: 3점 fit + 1cm 물리 보정
# 2026-07-10: 19.5 → 13.2 (−6.3 = 1.5cm 상당). READY 동결+직행 press로 다른 변수를
# 다 제거한 상태에서 "일관되게 1.5cm 위를 누름" 실측 → 고정 물리 오프셋(A/d 항)을
# 1.5cm만큼 하향. 방향이 남으면 같은 방법으로 재보정 (1.0cm ≈ A −4.2)
AIM_DIST_DEFAULT = 0.30          # 거리 미측정 시 가정값

def _aim_offsets(dist):
    """현재 목표 거리(m)에 맞는 조준 오프셋(px) 반환."""
    d = dist if (dist and 0.10 < dist < 1.0) else AIM_DIST_DEFAULT
    return (int(round(AIM_X_A / d + AIM_X_B)),
            int(round(AIM_Y_A / d + AIM_Y_B)))


# 허용 정렬 오차를 픽셀이 아니라 "실거리(cm)"로 고정 — 완전 정확 요구 대응.
# 픽셀 고정(15px)이면 0.4m에서 1.4cm나 허용돼 2cm 버튼 가장자리를 침.
TOL_CM = 0.6   # 어느 거리에서든 ±0.6cm 안에 들어야 CENTERED (사용자 요청으로 강화)

def _dead_zone_px(dist):
    """거리(m)에 따른 데드존 픽셀 수 (±TOL_CM 상당)."""
    d = dist if (dist and 0.10 < dist < 1.0) else AIM_DIST_DEFAULT
    return max(8, int(round(TOL_CM * 4.2 / d)))

# ── 누르기(press) 파라미터 ────────────────────────────────────────────
ARM_JOINT          = "wrist_extension"  # 팔 뻗기 관절 (실측 확인)
# [실측 보정 2026-07-09] press "허공" 판정 + 손끝이 표면 1cm 앞 정지 실측 → 2.5cm 하향
FINGER_STANDOFF    = 0.145  # m — 카메라 렌즈에서 닫힌 손가락 끝까지 거리
PRESS_CLOSE_DIST   = 0.20   # m — 여기까지는 그리퍼 연 채 접근 (열린 손끝이 벽 ~3cm 앞)
PRESS_READY_DIST   = 0.25   # m — 자동 접근이 "누르기 직전"(PRESS_CLOSE_DIST 0.20)까지 끝난
                            #     상태에서만 누르기 활성 + 로봇 완전 동결 (사용자 최종 결정:
                            #     "버튼 정하면 진짜 누르기 직전까지 가고, 그때만 누르게")
DET_MAX_DEPTH      = 1.5    # m — 이보다 먼 탐지는 버튼일 수 없음 (팔 0.5m + 접근 여유로도 불가)
                            #     → 복도 건너편 표지판·반사 오탐을 depth로 제거

# ── 높이 사전 지식 (접근성 규정: 호출/조작반 0.8~1.2m) ──
# 실전 모드: 타겟 선택 시 lift를 규정 높이로 선점프 → 탐색 시간 대폭 단축.
# 실험실 태블릿(의자 높이 ~0.7m)에선 방해되므로 False. ★현장 나갈 때 True로★
USE_HEIGHT_PRIOR = False
LIFT_PRIOR_CALL  = 0.80   # 호출 버튼(▲▼) 예상 lift
LIFT_PRIOR_PANEL = 0.85   # 차내 층 버튼 예상 lift
# (lift↔카메라 실높이 오프셋은 현장 첫 측정으로 보정)
PRESS_DEPTH        = 0.015  # m — 버튼 표면을 지나 밀어넣는 깊이 (버튼 스트로크)
PRESS_DIST_MAX     = 0.60   # m — 이보다 멀면 누르기 거부
ARM_EXT_MIN, ARM_EXT_MAX = 0.00, 0.50  # wrist_extension 안전 범위

# ── 베이스 전후 X정렬 (라이다 가드) ────────────────────────────────────
# 버튼의 좌우 오차를 로봇 전/후진으로 잡는다 (벽과 평행하게 선 상태 전제).
# 안전 3중: 이동 방향 ±30° 라이다 섹터 여유 확인 / 1회·누적 이동 상한 / 토글 ON일 때만.
LASER_YAW_OFFSET = 3.141592653589793  # 실측 TF: laser가 base 기준 180° 회전 장착
BASE_STEP_MAX    = 0.02   # m — 1회 이동 상한
BASE_TRAVEL_MAX  = 0.15   # m — 누적 이동 상한 (넘으면 정지, 주차가 틀렸다는 뜻)
# 0.02는 정지 마찰을 못 이겨 바퀴가 헛돌면서 누적치만 쌓임 (10cm 이동했다는데 오차 그대로 실측)
BASE_SPEED       = 0.04   # m/s
# 라이다는 로봇 "중심" 기준이고 몸통 끝은 중심에서 ~17cm.
# 라이다는 2D 단면이라 책상 상판처럼 위에서 튀어나온 건 못 봄 — 여유를 너무 줄이지 말 것.
CLEAR_DIST       = 0.30   # m — 이동 방향 최소 여유 (이보다 가까우면 이동 거부)
# 로봇 자기 부속물(허브에 꽂힌 케이블 등)이 몸통 밖 ~24cm까지 잡히는 것 실측
# → 0.25 미만은 자기 몸/부속물로 간주해 무시. 차단 유효 띠 = 0.25~0.30m.
# [트레이드오프] 25cm 안으로 갑자기 들어온 진짜 장애물도 무시됨 — 개발용 절충.
#   최종판에선 "정렬 시작 시 스캔 스냅샷 대비 가까워진 것만 차단" 방식으로 교체 예정.
SELF_HIT_MIN     = 0.25   # m — 이보다 가까운 레이는 로봇 자기 몸/부속물로 간주(무시)
GUARD_HALF_ANG   = 0.5236 # rad(30°) — 이동 방향 ± 이 각도 섹터를 검사
# [실측 확정 2026-07-08] 그리퍼가 로봇 오른쪽을 향함 → 화면 오른쪽(+ex) = 로봇 뒤쪽 → 후진(-1)
# (동쪽을 보고 서면 화면의 오른쪽 = 남쪽 = 몸의 뒤와 같은 기하)
BASE_X_SIGN      = -1

# 그리퍼: 열고 인식(시야 확보) → 닫고 누르기(손끝 모아 포인터로)
GRIPPER_JOINT   = "gripper_aperture"  # 벌림 폭(m) 단위의 가상 관절 (stretch_driver 지원)
# 이 로봇 실측: range_t 0~8700 (robotis 약 -51~+100) → 최대 벌림 ≈ 0.128m
# (gripper_conversion 주석: "aperture is 12.5cm wide when open")
GRIPPER_OPEN_M  = 0.125  # m — 완전 활짝 (인식 모드, 손가락이 시야 밖으로)
GRIPPER_CLOSE_M = 0.00   # m — 누르기 모드 (손끝 모음)

# 카메라 회전은 코드에 고정하지 않고 웹 UI "↻ 회전" 버튼으로 런타임 조절한다.
# (토픽/장착 상태에 따라 방향이 달라지는 문제를 클릭으로 해결)
# state["rot_grip"] / state["rot_body"] = 시계방향 90° 단위 회전 횟수 (0~3)
# 인식 라벨별 색 구분 (같은 글자 = 같은 색, 목표는 항상 빨강)
_LABEL_PALETTE = [
    (0, 210, 80),    # 초록
    (255, 160, 0),   # 주황
    (0, 180, 255),   # 하늘
    (200, 80, 255),  # 보라
    (80, 220, 220),  # 청록
    (255, 220, 0),   # 노랑
    (255, 105, 180), # 분홍
]
def _label_color(text: str):
    if not text:
        return _LABEL_PALETTE[0]
    return _LABEL_PALETTE[sum(map(ord, text)) % len(_LABEL_PALETTE)]


def _txt(img, text, org, scale, color, thick=2):
    """검은 외곽선 + 밝은 본문 — 어떤 배경에서도 읽히는 오버레이 텍스트."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)


def _rotate_steps(img, steps: int):
    """이미지를 시계방향 90° × steps 만큼 회전."""
    steps = steps % 4
    if steps == 1:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if steps == 2:
        return cv2.rotate(img, cv2.ROTATE_180)
    if steps == 3:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img

# ── 공유 상태 ─────────────────────────────────────────────────────────
state = {
    "phase":        "SELECT",
    "target_text":  None,
    "detections":   [],
    "centered":     False,
    "lift":         None,
    "yaw":          None,
    "jpeg_frame":       None,  # gripper 주석 프레임 (버튼 박스/거리)
    "jpeg_frame_body":  None,  # body 원본 프레임 (모니터링)
    "jpeg_frame_depth": None,  # gripper depth 컬러맵 (디버깅)
    "jpeg_frame_ocr":   None,  # OCR에 실제로 들어가는 이미지 (줌/샤프닝 보정 후)
    "target_dist":  None,   # 목표 버튼까지 거리 (m, D405 depth)
    "rot_grip":     0,      # gripper 회전 (CW 90° 단위, 웹 버튼으로 조절)
    "rot_body":     0,      # body 회전
    "place":        "hall", # 장소 모드: hall(홀, ▲▼만) / cab(차내, 숫자만) — 팔레트·prior 분기
    "arm_ext":      None,   # 현재 팔 뻗기 위치 (wrist_extension, m)
    "wall_tilt":    None,   # 벽 기울기 각도(°) — +면 오른쪽이 멂 (평행 정렬용)
    "wall_flat":    None,   # 앞면이 평면인가 (False면 기울기 측정 거부)
    "base_align":   False,  # 베이스 전후 X정렬 ON/OFF (기본 OFF — 명시적으로 켜야 움직임)
    "guard_off":    False,  # 라이다 가드 해제 (개발용 — 재시작 시 항상 가드 ON 복귀)
    "jpeg_fail":    None,   # 타겟 인식 실패 순간의 OCR 입력 스냅샷 (디버그: /fail.jpg)
    "base_travel":  0.0,    # 누적 이동량 (m)
    "align_note":   None,   # X정렬 상태 메시지
    "pressing":     False,  # 누르기 시퀀스 진행 중
    "press_status": None,   # 누르기 진행 메시지
}
state_lock = threading.Lock()

# 판단/행동 로그 버퍼 (웹 패널 표시 + 복사용). HTTP 접근 로그는 침묵시킴.
_DECISIONS = collections.deque(maxlen=400)
logging.getLogger("werkzeug").setLevel(logging.ERROR)

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
    /* 슬라이드 토글: 상태가 한눈에 보이는 진짜 스위치 */
    .tgl { display:flex; align-items:center; gap:8px; cursor:pointer; user-select:none; }
    .tgl-name  { color:#bbb; font-size:0.9rem; }
    .tgl-state { font-size:0.8rem; color:#777; min-width:32px; font-weight:700; }
    .tgl-track { width:46px; height:24px; border-radius:12px; background:#3a3a44;
                 position:relative; transition:background .15s; flex-shrink:0; }
    .tgl-knob  { width:18px; height:18px; border-radius:50%; background:#ddd;
                 position:absolute; top:3px; left:3px; transition:left .15s; }
    .tgl.on .tgl-track { background:#2a8f2a; }
    .tgl.amber.on .tgl-track { background:#b8651a; }
    .tgl.on .tgl-knob { left:25px; }
    .tgl.warn .tgl-track { background:#b22; }
  </style>
</head>
<body>
  <h2>🛗 Elevator Button Tracker</h2>

  <div id="banner">Waiting for camera...</div>
  <!-- 4패널 2x2 배치: 위 = gripper + depth, 아래 = OCR입력 + body -->
  <div style="display:flex; flex-direction:column; gap:16px; align-items:center;">
    <div style="display:flex; gap:16px; align-items:flex-start; justify-content:center;">
      <figure style="margin:0; text-align:center;">
        <img id="stream" src="/video?cam=gripper" alt="gripper stream"
             style="width:640px; background:#000;">
        <figcaption style="color:#8ad; font-size:0.95rem; margin-top:4px;">
          🤏 gripper (D405) — 버튼 인식·추적·거리
          <span id="tilt-info" style="margin-left:8px;font-weight:700;"></span>
          <button onclick="rotateCam('gripper')"
                  style="margin-left:8px;background:#334;color:#cdf;border:none;border-radius:5px;padding:3px 10px;cursor:pointer;">↻ 회전</button>
        </figcaption>
      </figure>
      <figure style="margin:0; text-align:center;">
        <img id="stream-depth" src="/video?cam=depth" alt="depth stream"
             style="width:640px; background:#000;">
        <figcaption style="color:#8ad; font-size:0.95rem; margin-top:4px;">
          🌊 depth — 파랑=가까움 · 빨강=멂 · 검정=측정불가</figcaption>
      </figure>
    </div>
    <div style="display:flex; gap:16px; align-items:flex-start; justify-content:center;">
      <figure style="margin:0; text-align:center;">
        <img id="stream-ocr" src="/video?cam=ocr" alt="ocr stream"
             style="width:640px; background:#000;">
        <figcaption style="color:#8ad; font-size:0.95rem; margin-top:4px;">
          🔬 OCR 입력 — 보정 후 (모델이 보는 그대로)</figcaption>
      </figure>
      <figure style="margin:0; text-align:center;">
        <img id="stream-body" src="/video?cam=body" alt="body stream"
             style="height:480px; background:#000;">
        <figcaption style="color:#8ad; font-size:0.95rem; margin-top:4px;">
          👁 body (D435i) — 전방 모니터링
          <button onclick="rotateCam('body')"
                  style="margin-left:8px;background:#334;color:#cdf;border:none;border-radius:5px;padding:3px 10px;cursor:pointer;">↻ 회전</button>
        </figcaption>
      </figure>
    </div>
  </div>

  <!-- 고정 타겟 팔레트: 인식 여부와 무관하게 항상 선택 가능 -->
  <div id="buttons" style="display:flex;gap:10px;"></div>
  <script>
    // [표시 라벨, 모델 토큰] — 화살표는 OCR 차셋의 기호 토큰으로 매핑 (^=▲, s=▼ 추정)
    const TARGETS = [['1','1'],['2','2'],['3','3'],['4','4'],['5','5'],
                     ['▲','^'],['▼','s']];
    let currentTarget = null;   // poll()에서 서버 상태로 갱신
    (function initPalette() {
      const box = document.getElementById('buttons');
      TARGETS.forEach(([label, tok], i) => {
        const b = document.createElement('button');
        b.className = 'btn'; b.id = 'tgt-' + i; b.textContent = label;
        b.dataset.tok = tok;
        b.style.minWidth = '52px'; b.style.fontSize = '1.2rem';
        // 재클릭 = 선택 해제 (토글)
        b.onclick = () => { currentTarget === tok ? resetTarget() : selectButton(tok); };
        box.appendChild(b);
      });
    })();
  </script>
  <!-- 동작 행: 크게, 자주 쓰는 것 -->
  <div style="display:flex;gap:12px;align-items:center;">
    <button id="press-btn" onclick="doPress()" disabled
            style="background:#a22;color:#fff;border:none;border-radius:10px;
                   padding:14px 36px;cursor:pointer;font-size:1.2rem;font-weight:800;">
      🔴 버튼 누르기</button>
    <button onclick="setGripper(true)"
            style="background:#264;color:#cfc;border:none;border-radius:6px;padding:10px 16px;cursor:pointer;">✋ 열기</button>
    <button onclick="setGripper(false)"
            style="background:#642;color:#fcc;border:none;border-radius:6px;padding:10px 16px;cursor:pointer;">✊ 닫기</button>
    <button id="reset" onclick="resetTarget()"
            style="background:#445;color:#dde;border:none;border-radius:6px;padding:10px 16px;cursor:pointer;">↺ 재선택</button>
  </div>
  <!-- 모드 행: 토글류 -->
  <div style="display:flex;gap:10px;align-items:center;">
    <span style="color:#567;font-size:0.8rem;">모드</span>
    <button id="place-btn" onclick="togglePlace()"
            style="background:#246;color:#ade;border:none;border-radius:6px;padding:7px 13px;cursor:pointer;font-weight:700;">🏢 홀 (호출 ▲▼)</button>
    <div class="tgl amber" id="align-tgl" onclick="toggleBaseAlign()"
         title="바퀴 이동 허용 (전후진 정렬·회전) — OFF면 바퀴 절대 안 움직임">
      <span class="tgl-name">🚗 몸체이동</span>
      <div class="tgl-track"><div class="tgl-knob"></div></div>
      <span class="tgl-state" id="align-state">OFF</span>
    </div>
    <div class="tgl" id="guard-tgl" onclick="toggleGuard()"
         title="라이다 장애물 가드 — 끄면 개발용 위험 모드 (재시작 시 자동 복귀)">
      <span class="tgl-name">🛡 가드</span>
      <div class="tgl-track"><div class="tgl-knob"></div></div>
      <span class="tgl-state" id="guard-state">ON</span>
    </div>
    <button id="pose-btn" onclick="setWristForward()"
            style="background:#334;color:#cdf;border:none;border-radius:6px;padding:7px 13px;cursor:pointer;">🏠 홈 포즈 (전방+팔접기)</button>
  </div>
  <div id="align-note" style="color:#fc4;font-size:0.9rem;min-height:1.2em;"></div>

  <!-- 판단 로그 패널: 로봇이 "무엇을 감지하고 어떤 행동을 왜 했는지" -->
  <div style="width:900px;max-width:95vw;">
    <div style="display:flex;justify-content:space-between;align-items:center;">
      <span style="color:#89a;font-size:0.85rem;font-weight:700;">🧠 판단 로그</span>
      <a href="/decisions" target="_blank"
         style="color:#68c;font-size:0.78rem;">전체 텍스트 열기 (복사용)</a>
    </div>
    <pre id="dlog" style="background:#0d1117;color:#9fb;border:1px solid #234;
         border-radius:8px;padding:8px 12px;height:180px;overflow-y:auto;
         font-size:0.78rem;line-height:1.5;margin:4px 0;white-space:pre-wrap;"></pre>
  </div>
  <script>
    setInterval(() => {
      fetch('/decisions').then(r => r.text()).then(t => {
        const el = document.getElementById('dlog');
        const atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 40;
        el.textContent = t;
        if (atBottom) el.scrollTop = el.scrollHeight;   // 보고 있으면 자동 따라가기
      }).catch(()=>{});
    }, 1000);
  </script>

  <div id="pitch-row">
    wrist_pitch:
    <input type="range" id="pitch-slider" min="-1.57" max="0.5" step="0.01" value="-0.02"
           oninput="document.getElementById('pitch-val').value=parseFloat(this.value).toFixed(2)">
    <input type="number" id="pitch-val" value="-0.02" step="0.01" min="-1.57" max="0.5"
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
  <div id="pitch-row">
    lift (높이):
    <input type="range" id="lift-slider" min="0.15" max="1.10" step="0.01" value="0.60"
           oninput="document.getElementById('lift-val').value=parseFloat(this.value).toFixed(2)">
    <input type="number" id="lift-val" value="0.60" step="0.01" min="0.15" max="1.10"
           oninput="document.getElementById('lift-slider').value=this.value">
    <button id="pitch-apply" onclick="applyLift()">적용</button>
  </div>
  <div id="info">감지된 버튼을 클릭하면 추적을 시작합니다.</div>

  <script>
    // 파이썬 _label_color와 동일한 규칙 (BGR→RGB 변환된 동일 팔레트)
    const LABEL_COLORS = ['rgb(80,210,0)','rgb(0,160,255)','rgb(255,180,0)',
                          'rgb(255,80,200)','rgb(220,220,80)','rgb(0,220,255)','rgb(180,105,255)'];
    function labelColor(t) {
      if (!t) return LABEL_COLORS[0];
      let s = 0; for (const c of t) s += c.charCodeAt(0);
      return LABEL_COLORS[s % LABEL_COLORS.length];
    }
    function selectButton(text) {
      fetch('/select', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({text})});
    }
    function resetTarget() { fetch('/reset', {method:'POST'}); }
    function rotateCam(cam) {
      fetch('/rotate', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({cam})});
    }
    function setTgl(id, on, warnOff, stateId) {
      const t = document.getElementById(id);
      if (!t) return;
      t.classList.toggle('on', on);
      t.classList.toggle('warn', warnOff && !on);
      const st = document.getElementById(stateId);
      if (st) {
        st.textContent = on ? 'ON' : 'OFF';
        st.style.color = on ? '#6d6' : (warnOff ? '#f66' : '#777');
      }
    }
    function toggleGuard() {
      fetch('/guard', {method:'POST'}).then(r => r.json())
        .then(d => setTgl('guard-tgl', !d.guard_off, true, 'guard-state'));
    }
    function toggleBaseAlign() {
      fetch('/base_align', {method:'POST'}).then(r => r.json())
        .then(d => setTgl('align-tgl', d.enabled, false, 'align-state'));
    }
    function togglePlace() {
      fetch('/place', {method:'POST'});   // 라벨/팔레트는 poll()이 서버 상태로 동기화
    }
    function setGripper(open) {
      fetch('/gripper', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({open})});
    }
    function doPress() {
      fetch('/press', {method:'POST'}).then(r => r.json()).then(d => {
        if (!d.ok) alert('누르기 거부: ' + d.error);
      });
    }
    function setWristForward() {
      const pitch = -0.02, yaw = 0.03;   // WRIST_PITCH/YAW_DEFAULT와 일치
      document.getElementById('pitch-slider').value = pitch;
      document.getElementById('pitch-val').value = pitch.toFixed(2);
      document.getElementById('yaw-slider').value = yaw;
      document.getElementById('yaw-val').value = yaw.toFixed(2);
      // 손목 각도 + 팔 완전 수납까지 한 번에 (홈 포즈)
      fetch('/wrist_forward', {method:'POST'});
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
    let liftSynced = false;
    function applyLift() {
      const v = parseFloat(document.getElementById('lift-val').value);
      fetch('/lift', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({lift: v})});
    }
    function poll() {
      fetch('/status').then(r => r.json()).then(s => {
        const banner = document.getElementById('banner');
        // X정렬 상태 메시지
        const an = document.getElementById('align-note');
        if (an) an.textContent = s.align_note || '';
        // 토글을 서버 상태와 동기화 (자동 OFF 등 서버측 변경 반영)
        setTgl('align-tgl', !!s.base_align, false, 'align-state');
        setTgl('guard-tgl', !s.guard_off, true, 'guard-state');
        // 벽 평행도 표시 (±3° 안 = 평행 OK)
        const ti = document.getElementById('tilt-info');
        if (ti) {
          if (s.wall_flat === false) {
            ti.style.color = '#f66';
            ti.textContent = '⚠ 평면 아님 — 기울기 측정 불가';
          }
          else if (s.wall_tilt == null) { ti.textContent = ''; }
          else {
            const a = Math.abs(s.wall_tilt);
            ti.style.color = a < 3 ? '#4e8' : (a < 7 ? '#fc4' : '#f66');
            ti.textContent = `벽 기울기 ${s.wall_tilt > 0 ? '+' : ''}${s.wall_tilt}°` +
                             (a < 3 ? ' ✓ 평행' : (s.wall_tilt > 0 ? ' (오른쪽이 멂)' : ' (왼쪽이 멂)'));
          }
        }
        // 최초 1회: 현재 팔 높이를 슬라이더에 반영
        if (!liftSynced && s.lift != null) {
          document.getElementById('lift-slider').value = s.lift.toFixed(2);
          document.getElementById('lift-val').value = s.lift.toFixed(2);
          liftSynced = true;
        }
        const pbtn = document.getElementById('press-btn');
        if (pbtn) {
          pbtn.disabled = s.pressing || !(s.centered && s.dist != null && s.dist <= 0.25);
          pbtn.style.opacity = pbtn.disabled ? 0.4 : 1.0;
        }
        if (s.pressing) {
          banner.textContent = `🤖 ${s.press_status || '누르기 진행 중...'}`;
          banner.className = 'tracking';
        } else if (s.centered && s.ex != null &&
                   (Math.abs(s.ex) > s.dz || Math.abs(s.ey) > s.dz)) {
          banner.textContent = `🟡 정렬 유지 중 — 경계 (x:${s.ex} y:${s.ey}, 허용±${s.dz}px)`;
          banner.className = 'tracking';
        } else if (s.centered && s.dist != null && s.dist <= 0.25) {
          const done = (s.press_status && s.press_status.startsWith('✅')) ? '  |  ' + s.press_status : '';
          banner.textContent = `✅ 준비 완료 — 로봇 정지, '${s.target}' 지금 누르세요  |  거리 ${s.dist}m` + done;
          banner.className = 'centered';
        } else if (s.centered) {
          banner.textContent = `🎯 정조준 — 자동 접근 중 (${s.dist!=null ? s.dist+'m' : '?'} → 25cm 도달 시 정지+누르기 활성)`;
          banner.className = 'tracking';
        } else if (s.target && s.ex == null) {
          banner.textContent = `🔍 '${s.target}' 찾는 중 — 패널(숫자 군집) 추적`;
          banner.className = 'tracking';
        } else if (s.target) {
          banner.textContent = `🎯 TRACKING '${s.target}'  |  x:${s.ex??'?'}  y:${s.ey??'?'}  |  거리 ${s.dist!=null ? s.dist+'m' : '?'}`;
          banner.className = 'tracking';
        } else {
          banner.textContent = '아래에서 목표 버튼을 클릭하세요';
          banner.className = '';
        }

        currentTarget = s.target;   // 토글 판단용 서버 상태 동기화
        // 장소 모드: 버튼 라벨 동기화 + 팔레트 활성 범위 (홀=▲▼만 / 차내=숫자만)
        const isHall = s.place !== 'cab';
        const plb = document.getElementById('place-btn');
        if (plb) {
          plb.textContent = isHall ? '🏢 홀 (호출 ▲▼)' : '🛗 차내 (층 숫자)';
          plb.style.background = isHall ? '#246' : '#453';
          plb.style.color = isHall ? '#ade' : '#fd8';
        }
        // 고정 팔레트 상태 갱신: 인식됨=밝은색, 미인식=어둡게, 선택=빨강 테두리
        const detSet = new Set(s.detections.map(d => d.text));
        TARGETS.forEach(([label, tok], i) => {
          const b = document.getElementById('tgt-' + i);
          if (!b) return;
          const isArrow = (tok === '^' || tok === 's');
          const allowed = isHall ? isArrow : !isArrow;
          b.disabled = !allowed;
          if (!allowed) {
            b.style.background = '#1a1a20'; b.style.color = '#444';
            b.style.outline = 'none'; b.title = '현재 장소 모드에서 비활성';
            return;
          }
          const det = detSet.has(tok);
          const sel = (tok === s.target);
          b.style.background = sel ? '#d33' : (det ? labelColor(tok) : '#2a2a33');
          b.style.color      = sel ? '#fff' : (det ? '#000' : '#667');
          b.style.outline    = sel ? '3px solid #f88' : 'none';
          b.title = det ? '인식됨' : '미인식 — 선택하면 패널을 추적하며 찾음';
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
    return render_template_string(HTML)

@app.route("/video")
def video():
    # ?cam=gripper (기본) 또는 ?cam=body — 두 스트림을 동시에 제공
    cam = request.args.get("cam", "gripper")
    key = {"body": "jpeg_frame_body", "depth": "jpeg_frame_depth",
           "ocr": "jpeg_frame_ocr"}.get(cam, "jpeg_frame")
    label = {"body": "body (D435i)", "depth": "gripper depth",
             "ocr": "OCR input"}.get(cam, "gripper (D405)")
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
            time.sleep(0.1)
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
            _ox, _oy = _aim_offsets(s.get("target_dist"))
            ex = round((b["x1"]+b["x2"])/2 - (CX + _ox))
            ey = round((b["y1"]+b["y2"])/2 - (CY + _oy))
    return jsonify(phase=s["phase"], target=target, detections=s["detections"],
                   centered=s["centered"], ex=ex, ey=ey,
                   dist=s.get("target_dist"),
                   pressing=s.get("pressing"), press_status=s.get("press_status"),
                   lift=s.get("lift"), wall_tilt=s.get("wall_tilt"),
                   wall_flat=s.get("wall_flat"),
                   base_align=s.get("base_align"), align_note=s.get("align_note"),
                   guard_off=s.get("guard_off"), place=s.get("place"),
                   dz=_dead_zone_px(s.get("target_dist")))

@app.route("/select", methods=["POST"])
def select():
    text = request.json.get("text", "")
    with state_lock:
        _pl0 = state["place"]
    if (_pl0 == "hall") != (text in ("^", "s")):
        return jsonify(ok=False, error="현재 장소 모드에서 선택할 수 없는 타겟")
    with state_lock:
        state["target_text"] = text
        state["phase"]       = "TRACK"
        state["centered"]    = False
        state["centered_ts"] = 0.0   # 이전 타겟의 CENTERED 관용 창 무효화
    node = _node_ref[0]
    if node:
        node._roi_miss  = 0   # 새 타겟 → ROI 집중 추적 즉시 재개 가능
        node._croi_miss = 0   # 군집 추적도 리셋
        node._target_lock = None   # 위치 잠금도 새로 시작
        node._rot_count   = 0      # 수직 정렬 시도 횟수 리셋
        node._dlog(f"[TARGET] '{text}' 선택 — 탐색·정렬·접근 시작")
        if USE_HEIGHT_PRIOR:
            prior = LIFT_PRIOR_CALL if _pl0 == "hall" else LIFT_PRIOR_PANEL
            with state_lock:
                cur_lift = state["lift"]
            if cur_lift is not None and abs(cur_lift - prior) > 0.12:
                node._dlog(f"[PRIOR] 규정 높이 선이동: lift {cur_lift:.2f}→{prior:.2f}")
                node._send_goal(["joint_lift"], [prior])
    return jsonify(ok=True)

@app.route("/decisions")
def decisions():
    """판단 로그 전체를 일반 텍스트로 — 복사해서 공유하기 좋게."""
    with state_lock:
        txt = "\n".join(_DECISIONS)
    return Response(txt or "(로그 없음)", mimetype="text/plain; charset=utf-8")

@app.route("/fail.jpg")
def fail_snapshot():
    """가장 최근 '타겟 인식 실패' 순간의 OCR 입력 이미지 (디버그용)."""
    with state_lock:
        j = state.get("jpeg_fail")
    if j is None:
        blank = np.zeros((IMAGE_H, IMAGE_W, 3), dtype=np.uint8)
        cv2.putText(blank, "no failure captured yet", (150, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (150, 150, 150), 2)
        _, e = cv2.imencode(".jpg", blank)
        j = e.tobytes()
    return Response(j, mimetype="image/jpeg")

@app.route("/guard", methods=["POST"])
def guard_toggle():
    """라이다 가드 해제/복귀 (개발용). 재시작하면 항상 가드 ON으로 복귀."""
    with state_lock:
        state["guard_off"] = not state["guard_off"]
        off = state["guard_off"]
        state["align_note"] = ("⚠⚠ 라이다 가드 해제됨 — 장애물 확인 없이 이동함 (개발용)!"
                               if off else "라이다 가드 복귀")
    return jsonify(ok=True, guard_off=off)

@app.route("/base_align", methods=["POST"])
def base_align_toggle():
    """베이스 전후 X정렬 ON/OFF. 켤 때 누적 이동량 리셋."""
    with state_lock:
        state["base_align"] = not state["base_align"]
        on = state["base_align"]
        if on:
            state["base_travel"] = 0.0
            node = _node_ref[0]
            if node:
                node._rot_warned = False
            state["align_note"]  = "몸체 이동 허용 — 전후진·회전 모두 라이다 가드 하에 동작"
        else:
            state["align_note"]  = None
    return jsonify(ok=True, enabled=on)

@app.route("/place", methods=["POST"])
def place_toggle():
    """장소 모드 토글: 홀(밖, 호출 ▲▼) ↔ 차내(안, 층 숫자). 전환 시 타겟 해제."""
    with state_lock:
        state["place"] = "cab" if state["place"] == "hall" else "hall"
        pl = state["place"]
        state["target_text"] = None
        state["phase"]       = "SELECT"
        state["centered"]    = False
    node = _node_ref[0]
    if node:
        node._dlog(f"[MODE] 장소: {'🛗 차내 (층 숫자)' if pl == 'cab' else '🏢 홀 (호출 ▲▼)'}")
    return jsonify(ok=True, place=pl)

@app.route("/gripper", methods=["POST"])
def gripper_ctrl():
    """그리퍼 수동 열기/닫기 (테스트·튜닝용)."""
    do_open = bool((request.json or {}).get("open", True))
    node = _node_ref[0]
    if node:
        node.set_gripper(GRIPPER_OPEN_M if do_open else GRIPPER_CLOSE_M)
    return jsonify(ok=True, open=do_open)

@app.route("/lift", methods=["POST"])
def lift_ctrl():
    """팔 높이(joint_lift) 수동 조정."""
    pos = float(request.json.get("lift", 0.6))
    pos = max(0.15, min(1.10, pos))
    node = _node_ref[0]
    if node:
        node.set_lift(pos)
    return jsonify(ok=True, lift=pos)

@app.route("/press", methods=["POST"])
def press():
    """누르기 시퀀스 — 사용자가 명시적으로 클릭할 때만 실행."""
    node = _node_ref[0]
    if node is None:
        return jsonify(ok=False, error="노드 준비 안 됨")
    err = node.start_press()
    return jsonify(ok=(err is None), error=err)

@app.route("/rotate", methods=["POST"])
def rotate():
    """해당 카메라 화면을 시계방향 90° 회전 (누를 때마다 +90°)."""
    cam = (request.json or {}).get("cam", "gripper")
    key = "rot_body" if cam == "body" else "rot_grip"
    with state_lock:
        state[key] = (state[key] + 1) % 4
        val = state[key]
    return jsonify(ok=True, cam=cam, rot=val * 90)

@app.route("/reset", methods=["POST"])
def reset():
    with state_lock:
        state["target_text"] = None
        state["phase"]       = "SELECT"
        state["centered"]    = False
    return jsonify(ok=True)

@app.route("/wrist_forward", methods=["POST"])
def wrist_forward():
    """홈 포즈: 손목 전방 + 팔 완전 수납 — 반드시 '한 번의 목표'로 묶어 전송.
    (따로 보내면 컨트롤러가 이전 목표를 선점·취소해 마지막 것만 실행됨)"""
    node = _node_ref[0]
    if node:
        node._send_goal(
            ["joint_wrist_pitch", "joint_wrist_yaw", ARM_JOINT],
            [WRIST_PITCH_DEFAULT, WRIST_YAW_DEFAULT, ARM_EXT_MIN])
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
        self._det_mem       = {}     # 탐지 기억 (라벨 → 마지막 박스/시각)

        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        # wrist_extension(팔 뻗기 합산값)은 /stretch/joint_states 에만 있음 (실측 확인)
        self.create_subscription(JointState, "/stretch/joint_states", self._on_joints, 10)

        # ── D405 depth (color 정렬) — 버튼까지 거리 측정용 (press 준비 0단계) ──
        # 로봇을 움직이지 않음. 화면에 거리 숫자만 표시.
        self._depth_frame = None   # 회전 보정된 uint16 배열 (mm)
        self._depth_lock  = threading.Lock()
        self._letterbox   = None   # (scale, off_x, off_y) — 표시↔원본 좌표 변환
        for t in GRIPPER_DEPTH_TOPICS:
            self.create_subscription(Image, t, self._on_depth, qos_profile_sensor_data)

        # 베이스 X정렬용: 라이다 + 바퀴 명령 (토글 ON일 때만 사용)
        self._scan    = None
        self._nudging = False
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self._cmd_pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)
        # 오도메트리 — _base_move 실이동 피드백용 (구독 누락으로 침묵 사망하던 버그 수정)
        self._odom_xy = None
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)

        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory"
        )
        # 시작 3초 후 wrist 초기 자세 자동 설정 (액션 서버 준비 대기)
        self.create_timer(3.0, self._init_wrist_once)
        self._wrist_initialized = False

        # 두 카메라 동시 구독 (전환 없음)
        # gripper는 토픽 이름이 부팅마다 달라서 후보 전부 구독 (발행되는 쪽만 프레임 옴)
        for t in GRIPPER_TOPICS:
            self.create_subscription(Image, t, self._on_image, 10)
            self._dlog(f"gripper: {t}")
        self.create_subscription(Image, BODY_TOPIC, self._on_image_body,
                                 qos_profile_sensor_data)
        self._dlog(f"body   : {BODY_TOPIC}")
        self._dlog("Ready — open http://localhost:5000")

    def _init_wrist_once(self):
        if self._wrist_initialized:
            return
        self._wrist_initialized = True
        self.set_wrist_pitch(WRIST_PITCH_DEFAULT)
        self.set_wrist_yaw(WRIST_YAW_DEFAULT)
        # 인식 모드: 그리퍼를 열어 카메라 시야 확보
        self.set_gripper(GRIPPER_OPEN_M)
        self._dlog(
            f"Wrist initialized: pitch={WRIST_PITCH_DEFAULT}, yaw={WRIST_YAW_DEFAULT}, gripper open"
        )

    def _on_image_body(self, msg):
        """body(D435i) 프레임 — 모니터링용 표시만 (OCR/제어 없음)."""
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with state_lock:
                rot = state["rot_body"]
            raw = _rotate_steps(raw, rot)
            # 회전 후 비율 유지, 높이 480에 맞춤
            h, w = raw.shape[:2]
            frame = cv2.resize(raw, (max(1, int(w * IMAGE_H / h)), IMAGE_H))
            _, jpeg = cv2.imencode(".jpg", frame)
            with state_lock:
                state["jpeg_frame_body"] = jpeg.tobytes()
        except Exception as e:
            self.get_logger().warn(f"body 프레임 오류: {e}")

    # ── 베이스 전후 X정렬 (라이다 가드) ──────────────────────────────
    def _on_scan(self, msg):
        self._scan = msg

    def _on_odom(self, msg):
        p_ = msg.pose.pose.position
        self._odom_xy = (p_.x, p_.y)

    def _clearance(self, direction: float):
        """이동 방향(+1 전진 / -1 후진) ±30° 섹터의 최소 장애물 거리(m).
        로봇 자기 몸(SELF_HIT_MIN 미만)은 무시. 스캔 없으면 None."""
        s = self._scan
        if s is None:
            return None
        r = np.asarray(s.ranges, dtype=float)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        base_ang = (ang + LASER_YAW_OFFSET + np.pi) % (2 * np.pi) - np.pi
        target = 0.0 if direction > 0 else np.pi
        diff = np.abs((base_ang - target + np.pi) % (2 * np.pi) - np.pi)
        m = (diff < GUARD_HALF_ANG) & np.isfinite(r) & (r > SELF_HIT_MIN)
        return float(r[m].min()) if m.any() else 99.0

    def _set_align_note(self, msg):
        with state_lock:
            state["align_note"] = msg
        self._dlog(f"[ALIGN] {msg}")

    def _base_move(self, move_m: float) -> bool:
        """가드 확인 후 베이스를 move_m(m)만큼 이동 (블로킹, 이동 중 감시).
        press 근접 재정렬 등 통제된 문맥에서 직접 호출용. 성공 여부 반환."""
        self._last_motion_ts = time.time()   # [A안] 이동 발생 기록
        direction = 1.0 if move_m > 0 else -1.0
        with state_lock:
            guard_off = state["guard_off"]
            if not state["base_align"]:
                return False   # 몸체 이동 마스터 스위치 OFF — 바퀴 절대 금지
        if not guard_off:
            c = self._clearance(direction)
            if c is None or c < CLEAR_DIST:
                return False
        # 오도메트리 피드백: 실제 이동 거리를 재면서 목표 도달 시 정지
        # (시간 제어는 정지마찰 때문에 5mm 명령→2mm 실주행 같은 언더슈트 발생 — 실측)
        start_xy = self._odom_xy
        msg = Twist(); msg.linear.x = direction * BASE_SPEED
        t0 = time.time()
        timeout = abs(move_m) / BASE_SPEED * 3.0 + 1.0
        while time.time() - t0 < timeout:
            if not guard_off:
                c = self._clearance(direction)
                if c is not None and c < CLEAR_DIST:
                    self._cmd_pub.publish(Twist())
                    return False
            if start_xy is not None and self._odom_xy is not None:
                dx_ = self._odom_xy[0] - start_xy[0]
                dy_ = self._odom_xy[1] - start_xy[1]
                if (dx_ * dx_ + dy_ * dy_) ** 0.5 >= abs(move_m):
                    break   # 실거리 도달 → 정지
            elif time.time() - t0 > abs(move_m) / BASE_SPEED:
                break       # 오도메트리 없으면 시간 기준 폴백
            self._cmd_pub.publish(msg)
            time.sleep(1 / 15)
        self._cmd_pub.publish(Twist())
        return True

    def _rotate_seq(self, ang_rad: float):
        """[원자적] 팔 후퇴 → 회전. 실행 중 _nudging으로 접근/이동 차단 —
        후퇴와 회전 사이에 접근이 팔을 도로 뻗던 무한 루프 방지."""
        self._nudging = True
        try:
            with state_lock:
                _ext = state["arm_ext"]
            if _ext is not None and _ext > 0.02:
                self._dlog("[ROTATE] 회전 전 팔 후퇴")
                self._move_joint_wait(ARM_JOINT, ARM_EXT_MIN, 2, 8.0)
            self._rotate_base(ang_rad)
        finally:
            self._nudging = False

    def _rotate_base(self, ang_rad: float) -> bool:
        """제자리 회전으로 벽과 수직 맞추기. ang>0 = 반시계. 1회 최대 ±15°.
        가드: 주변 30cm 내 장애물이 있으면 회전 거부 (회전 반경 안전)."""
        ang_rad = max(-0.26, min(0.26, ang_rad))
        self._last_motion_ts = time.time()   # [A안] 이동 발생 기록
        with state_lock:
            guard_off = state["guard_off"]
            if not state["base_align"]:
                return False   # 몸체 이동 마스터 스위치 OFF — 회전도 금지
        if not guard_off:
            s = self._scan
            if s is not None:
                r = np.asarray(s.ranges, dtype=float)
                m = np.isfinite(r) & (r > SELF_HIT_MIN)
                if m.any() and float(r[m].min()) < 0.30:
                    self._set_align_note("⚠ 주변 30cm 내 장애물 — 회전 보류")
                    return False
        self._nudging = True   # 회전 중 전후진 겹침 방지
        try:
            wz  = 0.25 if ang_rad > 0 else -0.25
            dur = abs(ang_rad) / 0.25
            msg = Twist(); msg.angular.z = wz
            t0 = time.time()
            while time.time() - t0 < dur:
                self._cmd_pub.publish(msg)
                time.sleep(1 / 15)
            self._cmd_pub.publish(Twist())
            return True
        finally:
            self._nudging = False

    def _maybe_base_nudge(self, ex: float, dist):
        """좌우 픽셀 오차 → 안전 확인 후 베이스 소폭 전/후진 (별도 스레드)."""
        with state_lock:
            enabled  = state["base_align"]
            pressing = state["pressing"]
            travel   = state["base_travel"]
        if not enabled or pressing or self._nudging:
            return
        if travel >= BASE_TRAVEL_MAX:
            # 상한 도달 → X정렬 자동 OFF (메시지 반복 방지). 재활성화하면 누적 리셋됨.
            with state_lock:
                state["base_align"] = False
            self._set_align_note(
                f"누적 이동 {travel*100:.0f}cm 초과 — X정렬 자동 OFF. "
                "로봇 주차 위치를 옮긴 뒤 다시 켜세요.")
            return
        d = dist if (dist and 0.1 < dist < 1.0) else 0.30
        # 이득 0.7: 오차 전부가 아니라 70%만 이동 (과보정→반대편 튐→진동 방지)
        move_m = BASE_X_SIGN * ex * d / 420.0 * 0.7
        move_m = max(-BASE_STEP_MAX, min(BASE_STEP_MAX, move_m))
        if abs(move_m) < 0.004:                         # 4mm 미만은 무시
            return
        self._nudging = True
        threading.Thread(target=self._do_nudge, args=(move_m,), daemon=True).start()

    def _do_nudge(self, move_m: float):
        try:
            direction = 1.0 if move_m > 0 else -1.0
            side = "전방" if direction > 0 else "후방"
            with state_lock:
                guard_off = state["guard_off"]
            if not guard_off:
                clear = self._clearance(direction)
                if clear is None:
                    self._set_align_note("라이다 미수신 — 이동 불가"); return
                if clear < CLEAR_DIST:
                    self._set_align_note(f"⚠ {side} {clear:.2f}m — 막힘, 이동 보류"); return

            dur = abs(move_m) / BASE_SPEED
            msg = Twist(); msg.linear.x = direction * BASE_SPEED
            t0 = time.time()
            aborted = False
            while time.time() - t0 < dur:
                if not guard_off:
                    c = self._clearance(direction)       # 이동 중에도 계속 감시
                    if c is not None and c < CLEAR_DIST:
                        self._set_align_note(f"⚠ 이동 중 {side} 막힘 — 즉시 정지")
                        aborted = True
                        break
                self._cmd_pub.publish(msg)
                time.sleep(1 / 15)
            self._cmd_pub.publish(Twist())               # 정지
            # 실제 이동량 = 속도 × 실제 이동 시간 (중단 시 과대집계 방지)
            moved = min(abs(move_m), BASE_SPEED * (time.time() - t0))
            with state_lock:
                state["base_travel"] += moved
                tv = state["base_travel"]
            if not aborted:
                self._set_align_note(f"이동 {move_m*100:+.1f}cm (누적 {tv*100:.1f}cm)")
            # "이동 후" 프레임의 인식이 나올 때까지 대기 — 낡은 오차로 또 이동(진동) 방지
            end_t = time.time()
            while (getattr(self, "_last_infer", 0) < end_t + 1.0
                   and time.time() - end_t < 4.0):
                time.sleep(0.1)
        finally:
            self._nudging = False

    def _dlog(self, msg: str):
        """판단/행동 로그: ROS 터미널 + 웹 로그 패널에 동시 기록."""
        self.get_logger().info(msg)
        with state_lock:
            _DECISIONS.append(f"{time.strftime('%H:%M:%S')} {msg}")

    def _may_explore(self) -> bool:
        """탐색성 이동(스캔·시크·군집접근·회전)의 공통 관문 — 난투극 방지.
        조건: 직전 동작 완료 + 마지막 이동 후 2.5초 경과 + 정지 상태 인식 1회 확보.
        (여러 서브시스템이 각자 쿨다운으로 겹쳐 움직이던 것을 전역 1개로 통합)"""
        now = time.time()
        if not self._goal_done:
            return False
        if now - getattr(self, "_last_motion_ts", 0) < 2.5:
            return False
        if getattr(self, "_last_infer", 0) < getattr(self, "_last_motion_ts", 0) + 0.8:
            return False
        return True

    def _on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == "joint_lift":
                with state_lock: state["lift"] = pos
            elif name == "joint_wrist_yaw":
                with state_lock: state["yaw"] = pos
            elif name == ARM_JOINT:
                with state_lock: state["arm_ext"] = pos

    def _on_depth(self, msg):
        """aligned depth 프레임 저장. color와 동일하게 회전 보정해 좌표계를 맞춤."""
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                (msg.height, msg.width))
            # color와 동일한 회전 적용 → 거리 샘플링 좌표계 일치
            with state_lock:
                rot = state["rot_grip"]
            arr = _rotate_steps(arr, rot)
            with self._depth_lock:
                self._depth_frame = arr

            # 벽 평행도 측정: 중앙 가로 띠의 10개 지점 depth를 직선에 fit.
            # 모든 점이 직선 ±2cm 안이면 "평면(벽)"으로 인정하고 기울기 계산,
            # 잔차가 크면(사람/물건이 튀어나옴 등) "평면 아님"으로 표시 거부.
            h, w = arr.shape
            band = arr[h//2 - 25:h//2 + 25, :]
            xs, ds = [], []
            for x0 in np.linspace(w * 0.25, w * 0.75, 10).astype(int):
                p = band[:, max(0, x0 - 8):x0 + 8]
                v = p[(p > 100) & (p < 1200)]          # 0.1~1.2m 만 신뢰
                if len(v) > 30:
                    xs.append(float(x0))
                    ds.append(float(np.median(v)) / 1000.0)
            tilt, flat = None, None
            if len(xs) >= 7:
                a, b = np.polyfit(xs, ds, 1)           # d ≈ a·x + b
                resid = np.abs(np.polyval((a, b), xs) - np.array(ds))
                flat = bool(resid.max() < 0.02)
                if flat:
                    d_mean = float(np.mean(ds))
                    fx = w * 0.527                     # HFOV≈87° 근사 초점거리(px)
                    tilt = float(np.degrees(np.arctan(a * fx / d_mean)))
            with state_lock:
                state["wall_tilt"] = round(tilt, 1) if tilt is not None else None
                state["wall_flat"] = flat
            if flat and tilt is not None:
                if not hasattr(self, "_tilt_hist"):
                    self._tilt_hist = collections.deque(maxlen=6)
                self._tilt_hist.append((time.time(), tilt))

            # depth 컬러맵 표시 (3프레임마다 = ~5Hz, 부하 최소화)
            # 가까움=파랑 → 멂=빨강 (0~1.5m 정규화), 측정불가=검정
            self._depth_viz_i = getattr(self, "_depth_viz_i", 0) + 1
            if self._depth_viz_i % 3 == 0:
                d8  = np.clip(arr.astype(np.float32) / 1500.0 * 255, 0, 255).astype(np.uint8)
                vis = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
                vis[arr == 0] = (0, 0, 0)
                _, jpeg = cv2.imencode(".jpg", vis)
                with state_lock:
                    state["jpeg_frame_depth"] = jpeg.tobytes()
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
        lb = self._letterbox
        if lb is not None:
            # 레터박스 역변환: 표시 좌표 → 원본(=depth) 좌표
            scale, off_x, off_y = lb
            u = int((x_disp - off_x) / scale)
            v = int((y_disp - off_y) / scale)
        else:
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
        # (주의: 1.5m 밖도 None → "depth 유효"는 곧 "1.5m 이내 실측"을 의미)
        return D if 0.05 < D < 1.5 else None

    def _is_press_ready(self) -> bool:
        """정조준 + 누르기 가능 거리 = '클릭 대기' 상태.
        이때는 모든 자동 이동을 동결한다 — 사용자가 누를 표적이 움직이면
        클릭 타이밍을 잡을 수 없음. 클릭하면 press 시퀀스가 이어받는다."""
        with state_lock:
            return (state["centered"] and state["target_dist"] is not None
                    and state["target_dist"] <= PRESS_READY_DIST)

    def _on_image(self, msg):
        try:
            raw   = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # 회전을 리사이즈 전에 적용 → 화면/OCR/서보/depth가 모두 같은 방향 사용
            with state_lock:
                rot = state["rot_grip"]
            raw   = _rotate_steps(raw, rot)
            # 원본 고해상도 프레임 보존 — ROI/줌이 여기서 직접 잘라 진짜 디테일 확보
            self._raw_full = raw
            # [OCR 인식률] 비율 유지 리사이즈 + 레터박스 — 글자가 늘어나 찌그러지는 것 방지
            h, w  = raw.shape[:2]
            scale = min(IMAGE_W / w, IMAGE_H / h)
            nw, nh = int(w * scale), int(h * scale)
            resized = cv2.resize(raw, (nw, nh))
            frame = np.zeros((IMAGE_H, IMAGE_W, 3), dtype=np.uint8)
            off_x = (IMAGE_W - nw) // 2
            off_y = (IMAGE_H - nh) // 2
            frame[off_y:off_y + nh, off_x:off_x + nw] = resized
            # 표시 좌표 → 원본 좌표 역변환용 (depth 샘플링에 사용)
            self._letterbox = (scale, off_x, off_y)
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
            # 디지털 줌: 중앙 crop → 2배 확대 → OCR → 좌표를 원본 기준으로 역변환
            # (작은 글자의 픽셀 수를 늘려 인식 거리를 ~2배로 확장)
            with state_lock:
                tgt0   = state["target_text"]
                phase0 = state["phase"]
                place0 = state["place"]

            def _enhance(img):
                """고품질 보간(cubic) + 언샤프 마스킹 — 확대 시 글자 경계 보존"""
                up = cv2.resize(img, (IMAGE_W, IMAGE_H),
                                interpolation=cv2.INTER_CUBIC)
                blur = cv2.GaussianBlur(up, (0, 0), 1.5)
                return cv2.addWeighted(up, 1.5, blur, -0.5, 0)

            def _crop_hi(cx0, cy0, cw0, ch0):
                """표시 좌표 사각형을 '원본 고해상도 프레임'에서 잘라 확대.
                카메라가 640x480보다 높으면 보간이 아닌 실제 디테일이 들어감."""
                rawf = getattr(self, "_raw_full", None)
                lb   = getattr(self, "_letterbox", None)
                if rawf is not None and lb is not None:
                    s, lox, loy = lb
                    rx0 = int(max(0, (cx0 - lox) / s))
                    ry0 = int(max(0, (cy0 - loy) / s))
                    rw0 = max(1, int(cw0 / s))
                    rh0 = max(1, int(ch0 / s))
                    Hf, Wf = rawf.shape[:2]
                    rx0 = min(rx0, max(0, Wf - rw0))
                    ry0 = min(ry0, max(0, Hf - rh0))
                    crop = rawf[ry0:ry0 + rh0, rx0:rx0 + rw0]
                    if crop.size:
                        return _enhance(crop)
                return _enhance(frame[cy0:cy0 + ch0, cx0:cx0 + cw0])

            # ── 타겟 집중 추적(ROI): 선택한 버튼 주변만 잘라 고배율 재인식 ──
            # (XMem식 '집중 추적'의 경량판 — 같은 OCR을 타겟 주변 crop에 재사용.
            #  숫자가 입력을 꽉 채워 인식 안정성과 박스 정밀도가 크게 오름)
            roi = None
            self._roi_round = getattr(self, "_roi_round", 0) + 1
            if (phase0 == "TRACK" and tgt0
                    and getattr(self, "_roi_miss", 0) < 2       # 2연속 놓치면 전체화면 복귀
                    and self._roi_round % 5 != 0):              # 5라운드마다 전체화면 (목록 갱신)
                e = self._det_mem.get(tgt0)
                if e and (time.time() - e["ts"]) < 3.0:
                    bb  = e["det"]["box"]
                    bcx = (bb["x1"] + bb["x2"]) / 2
                    bcy = (bb["y1"] + bb["y2"]) / 2
                    hw  = max((bb["x2"] - bb["x1"]) * 2.5, 90)  # 박스의 2.5배 반경
                    hw  = min(hw, IMAGE_W / 2)
                    hh  = hw * IMAGE_H / IMAGE_W                # 4:3 유지
                    x0  = int(max(0, min(IMAGE_W - 2 * hw, bcx - hw)))
                    y0  = int(max(0, min(IMAGE_H - 2 * hh, bcy - hh)))
                    w0, h0 = int(2 * hw), int(2 * hh)
                    if w0 >= 80 and h0 >= 60:
                        roi = (x0, y0, w0, h0)
            roi_kind = "target" if roi is not None else None

            # 타겟이 안 보이면: 숫자 군집(=패널 후보)을 자동 확대해 그 안에서 타겟 탐색
            # ("숫자들이 모여 있으면 패널이고, 내가 고른 숫자가 거기 있겠거니" — 집요 추적)
            if (roi is None and phase0 == "TRACK" and tgt0
                    and getattr(self, "_croi_miss", 0) < 2
                    and self._roi_round % 4 != 0):
                # 패널 단서(비숫자 포함 "버튼같은 것") 기반 — 원거리에서도 유효
                boxes = list(getattr(self, "_panel_boxes", []))
                if time.time() - getattr(self, "_panel_ts", 0) > 2.5:
                    boxes = []
                if len(boxes) >= 2:
                    x1s = min(b["x1"] for b in boxes); x2s = max(b["x2"] for b in boxes)
                    y1s = min(b["y1"] for b in boxes); y2s = max(b["y2"] for b in boxes)
                    ccx, ccy = (x1s + x2s) / 2, (y1s + y2s) / 2
                    hw = max((x2s - x1s) * 0.75, 110); hw = min(hw, IMAGE_W / 2)
                    hh = hw * IMAGE_H / IMAGE_W
                    x0 = int(max(0, min(IMAGE_W - 2 * hw, ccx - hw)))
                    y0 = int(max(0, min(IMAGE_H - 2 * hh, ccy - hh)))
                    w0, h0 = int(2 * hw), int(2 * hh)
                    if w0 >= 80 and h0 >= 60:
                        roi = (x0, y0, w0, h0)
                        roi_kind = "cluster"
            self._roi_last = roi   # 메인 화면 표시용

            if roi is not None:
                x0, y0, w0, h0 = roi
                ocr_input = _crop_hi(x0, y0, w0, h0)          # 원본 해상도에서 잘라 확대
            else:
                ocr_input = frame

            # (OCR 미리보기는 인식 결과 박스까지 그리기 위해 추론 후에 생성 — 아래)
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as f:
                tmp = f.name
            cv2.imwrite(tmp, ocr_input)
            detections = infer_image(tmp)
            os.unlink(tmp)

            # ── 2층 분리 ──
            # raw_dets: 전체 탐지(비숫자 "?" 포함) = "버튼처럼 생긴 것" — 패널 단서용.
            #   글자를 못 읽는 원거리에서도 1단계 탐지기는 버튼 모양을 잡으므로,
            #   이 군집이 "저기가 엘리베이터 패널이다"의 근거가 된다.
            # detections: 숫자만 = 신원 확정 — UI 팔레트·타겟 서보·press용.
            raw_dets   = [d for d in detections if d.get("score", 0) >= 0.4]
            # 유효 라벨 = 층수 숫자(1~2자리) + 호출/문 기호 (^=▲, s=▼ 추정, <>=문)
            SYMBOL_ALLOW = {"^", "s", "<", ">"}
            def _valid_label(t):
                t = t.strip()
                return (t.isdigit() and len(t) <= 2) or t in SYMBOL_ALLOW
            detections = [d for d in raw_dets if _valid_label(d.get("text", ""))]
            # 진단: 처음 보는 라벨은 로그에 (화살표가 실제 무슨 토큰인지 확정용)
            if not hasattr(self, "_seen_labels"):
                self._seen_labels = set()
            for d in raw_dets:
                _t = d.get("text", "").strip()
                if _t and _t not in self._seen_labels:
                    self._seen_labels.add(_t)
                    self._dlog(f"[OCR] 새 라벨 관측: '{_t}' "
                               f"(belief {d.get('belief', 0):.2f})")

            # 타겟이 이번 라운드에 안 잡혔으면 그 순간의 OCR 입력을 스냅샷 저장
            # → 브라우저에서 /fail.jpg 로 열어 "모델이 뭘 보고 놓쳤는지" 직접 확인
            if tgt0 and not any(d["text"] == tgt0 for d in detections):
                _, _fj = cv2.imencode(".jpg", ocr_input)
                with state_lock:
                    state["jpeg_fail"] = _fj.tobytes()

            # ── OCR 미리보기: 인식 박스 + 조준 눈금을 그려서 노출 ──
            # (박스는 아직 OCR 입력 좌표계 — 확대 화면 위에 그대로 그리면 됨)
            preview = ocr_input.copy()
            # 후보지(버튼같은 것, 비숫자 포함)도 회색으로 표시 — 확대 화면에서
            # 모델이 어디서 버튼 모양을 잡는지 보이게
            for d in raw_dets:
                bb = d["box"]
                cv2.rectangle(preview, (int(bb["x1"]), int(bb["y1"])),
                              (int(bb["x2"]), int(bb["y2"])), (140, 140, 140), 1)
            for d in detections:
                bb = d["box"]
                px1, py1 = int(bb["x1"]), int(bb["y1"])
                px2, py2 = int(bb["x2"]), int(bb["y2"])
                pc = (0, 0, 255) if d["text"] == tgt0 else _label_color(d["text"])
                cv2.rectangle(preview, (px1, py1), (px2, py2), pc, 2)
                cv2.putText(preview, d["text"], (px1, max(16, py1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, pc, 2)
            with state_lock:
                _td2 = state["target_dist"]
            _ox2, _oy2 = _aim_offsets(_td2)
            _dz2 = _dead_zone_px(_td2)
            if roi is not None:
                _sx = roi[2] / IMAGE_W
                zx = int((CX + _ox2 - roi[0]) / _sx)
                zy = int((CY + _oy2 - roi[1]) / (roi[3] / IMAGE_H))
                _dz2 = int(_dz2 / _sx)
                cv2.putText(preview, f"ROI x{1/_sx:.1f}", (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                zx, zy = CX + _ox2, CY + _oy2
            if 0 <= zx < IMAGE_W and 0 <= zy < IMAGE_H:
                cv2.line(preview, (zx - 35, zy), (zx + 35, zy), (0, 255, 255), 2)
                cv2.line(preview, (zx, zy - 35), (zx, zy + 35), (0, 255, 255), 2)
                cv2.circle(preview, (zx, zy), _dz2, (0, 255, 255), 2)
            _, _oj = cv2.imencode(".jpg", preview)
            with state_lock:
                state["jpeg_frame_ocr"] = _oj.tobytes()

            if roi is not None:
                # ── 원형 피팅: 확대 이미지에서 버튼 원 테두리를 찾아
                #    박스 중심을 진짜 버튼 중심에 스냅 + 크기도 원에 맞춤 ──
                def _refine_circle(img, bb):
                    x1i, y1i = int(bb["x1"]), int(bb["y1"])
                    x2i, y2i = int(bb["x2"]), int(bb["y2"])
                    m = int(max(x2i - x1i, y2i - y1i) * 0.4) + 4
                    xa, ya = max(0, x1i - m), max(0, y1i - m)
                    crop = img[ya:min(IMAGE_H, y2i + m), xa:min(IMAGE_W, x2i + m)]
                    if crop.size == 0:
                        return None
                    g = cv2.medianBlur(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), 5)
                    rmin = max(6, int((x2i - x1i) * 0.35))
                    rmax = max(rmin + 4, int(max(x2i - x1i, y2i - y1i)))
                    cir = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, dp=1.2,
                                           minDist=1000, param1=120, param2=30,
                                           minRadius=rmin, maxRadius=rmax)
                    if cir is None:
                        return None
                    cx, cy, r = cir[0][0]
                    return (xa + float(cx), ya + float(cy), float(r))

                for d in detections:
                    if d["text"] != tgt0:
                        continue
                    ref = _refine_circle(ocr_input, d["box"])
                    if ref:
                        rcx, rcy, rr = ref
                        d["box"] = {"x1": rcx - rr, "y1": rcy - rr,
                                    "x2": rcx + rr, "y2": rcy + rr}

                # ROI 좌표 → 표시 좌표 역변환 (숫자+비숫자 전체 = raw_dets 기준;
                # detections는 같은 dict를 공유하므로 함께 변환됨)
                x0, y0, w0, h0 = roi
                sx, sy = w0 / IMAGE_W, h0 / IMAGE_H
                for d in raw_dets:
                    bb = d["box"]
                    bb["x1"] = bb["x1"] * sx + x0; bb["x2"] = bb["x2"] * sx + x0
                    bb["y1"] = bb["y1"] * sy + y0; bb["y2"] = bb["y2"] * sy + y0
                # ROI 라운드 실패 카운트 (2연속이면 전체화면 재탐색으로 복귀)
                if roi_kind == "target":
                    if any(d["text"] == tgt0 for d in detections):
                        self._roi_miss = 0
                    else:
                        self._roi_miss = getattr(self, "_roi_miss", 0) + 1
                else:  # cluster: 버튼같은 게 하나라도 잡히면 군집 추적 유지
                    if raw_dets:
                        self._croi_miss = 0
                    else:
                        self._croi_miss = getattr(self, "_croi_miss", 0) + 1

            # ── depth 거리 필터: 팔이 닿을 수 없는 원거리 오탐 제거 ──
            # 버튼처럼 잡혀도 depth가 DET_MAX_DEPTH 밖이면 버튼이 아님.
            # depth 구멍(None)은 판정 불가 → 통과시킴 (반사 심한 패널에서 진짜
            # 버튼의 depth가 안 잡히는 경우 억울한 제거 방지)
            def _too_far(dd):
                bb = dd["box"]
                zd = self._depth_at(int((bb["x1"] + bb["x2"]) / 2),
                                    int((bb["y1"] + bb["y2"]) / 2))
                return zd is not None and zd > DET_MAX_DEPTH
            _n_before  = len(raw_dets)
            raw_dets   = [d for d in raw_dets   if not _too_far(d)]
            detections = [d for d in detections if not _too_far(d)]
            _n_cut = _n_before - len(raw_dets)
            if _n_cut and time.time() - getattr(self, "_far_log_ts", 0) > 5.0:
                self._far_log_ts = time.time()
                self._dlog(f"[FILTER] 원거리(>{DET_MAX_DEPTH}m) 오탐 {_n_cut}개 제거")

            # 패널 단서 저장: "버튼처럼 생긴 것"들의 박스 (표시 좌표계, 원거리 추적용)
            self._panel_boxes = [d["box"] for d in raw_dets]
            self._panel_ts    = time.time()

            # ── 화살표(호출 버튼) 위치 휴리스틱 ──
            # 실측: 이 모델은 화살표를 '?','*','%' 등으로 오독 (글자 판독 불가).
            # → 위치로 결정: 버튼형 탐지 중 ▼(s)=가장 아래, ▲(^)=가장 위.
            #   버튼이 하나뿐이면 그것이 곧 호출 버튼. 글자 불필요 = 반사에도 면역.
            if (place0 == "hall" and tgt0 in ("^", "s")
                    and not any(d["text"] == tgt0 for d in detections)):
                # 합성은 "위치"라는 약한 증거만으로 타겟을 만드므로 depth 실측을 요구:
                # 측정 불가(구멍)나 원거리는 후보 제외 — 검은 바퀴(IR 흡수, depth ?)가
                # "가장 아래 버튼같은 것"으로 ▼ 합성되던 실측 사례(2026-07-10) 차단
                def _synth_ok(dd):
                    bb = dd["box"]
                    zs = self._depth_at(int((bb["x1"] + bb["x2"]) / 2),
                                        int((bb["y1"] + bb["y2"]) / 2))
                    return zs is not None and zs <= DET_MAX_DEPTH
                # 숫자가 여럿 읽히면 지금 보는 건 차내 조작반일 가능성 — 진짜 호출부
                # (화살표만 있는 패널)가 아니므로 합성 생략. 없는 ▲▼를 잡동사니
                # ('*','$' 등 비숫자 탐지)로 지어내던 실측 사례(2026-07-10) 차단.
                _digits_now = sum(1 for dd in detections
                                  if dd.get("text", "").strip().isdigit())
                if _digits_now >= 3:
                    cands_a = []
                    if time.time() - getattr(self, "_synth_skip_ts", 0) > 5.0:
                        self._synth_skip_ts = time.time()
                        self._dlog(f"[SYNTH] 숫자 {_digits_now}개 보임 — 조작반으로 판단, "
                                   "▲▼ 합성 생략 (장소 모드 확인 필요)")
                else:
                    cands_a = [d for d in raw_dets
                               if not d.get("text", "").strip().isdigit()
                               and d.get("score", 0) >= 0.5
                               and _synth_ok(d)]
                if cands_a:
                    pick = (max if tgt0 == "s" else min)(
                        cands_a,
                        key=lambda dd: (dd["box"]["y1"] + dd["box"]["y2"]) / 2)
                    synth = dict(pick)
                    synth["text"]   = tgt0
                    synth["belief"] = max(0.5, pick.get("belief", 0))
                    detections = detections + [synth]
            if roi is None and tgt0 and any(d["text"] == tgt0 for d in detections):
                self._roi_miss = 0   # 전체화면에서 재발견 → ROI 재개

            # ── 탐지 기억(persistence): 프레임 단위 깜빡임 흡수 ──
            # 이번 프레임에 잡힌 라벨은 갱신, 못 잡힌 라벨도 DET_PERSIST_SEC 동안
            # 마지막 박스로 유지(fresh=False). 서보/press는 fresh만 사용.
            now = time.time()

            # ── 위치 잠금: 같은 글자를 주장하는 물리적 다른 버튼(오독) 구분 ──
            # 한 번 잡은 타겟은 위치를 잠그고, 이후 같은 글자는 그 근처에서 온 것만 인정.
            # 멀리서 갑자기 나타난 동명이인(예: '2'가 '3'으로 오독)은 참칭으로 무시.
            if tgt0:
                cands = [d for d in detections if d["text"] == tgt0]
                lk = getattr(self, "_target_lock", None)
                chosen = None
                if cands:
                    # belief(글자 확신도) 최고 후보 — 오독('4'를 '1'로)은 대개 belief가 낮음
                    best = max(cands, key=lambda dd: dd.get("belief", 0))
                    if lk and now - lk["ts"] < 4.0:
                        def _lock_dist(dd):
                            cx0 = (dd["box"]["x1"] + dd["box"]["x2"]) / 2
                            cy0 = (dd["box"]["y1"] + dd["box"]["y2"]) / 2
                            return ((cx0 - lk["cx"])**2 + (cy0 - lk["cy"])**2) ** 0.5
                        near = min(cands, key=_lock_dist)
                        bw = max(20, near["box"]["x2"] - near["box"]["x1"])
                        if _lock_dist(near) <= max(80, bw * 2.0):
                            chosen = near
                        # 잠금 탈취: 다른 위치의 후보가 잠금의 확신보다 뚜렷이 높게
                        # 2라운드 연속 나타나면 → 그쪽이 진짜 (오독에 잠긴 상태 교정)
                        # 단, 잠금 생성 후 4초(탐색 초반)까지만 허용 — 정렬 중 잠금이
                        # 다른 위치로 점프하면 서보가 새 목표를 쫓아 진동·정렬 폭주
                        # (2026-07-08 실측: 탈취 3회 → lift 100초 진동 + X정렬 16cm 소진)
                        if best is not chosen and \
                                now - lk.get("born", 0.0) < 4.0 and \
                                best.get("belief", 0) >= lk.get("bel", 0.5) + 0.15:
                            self._usurp_streak = getattr(self, "_usurp_streak", 0) + 1
                            if self._usurp_streak >= 2:
                                chosen = best
                                self._usurp_streak = 0
                                _zb = self._depth_at(
                                    int((best["box"]["x1"] + best["box"]["x2"]) / 2),
                                    int((best["box"]["y1"] + best["box"]["y2"]) / 2))
                                self._dlog(
                                    f"[LOCK] 더 확신 높은 '{tgt0}' 발견 → 잠금 이동 "
                                    f"(belief {best.get('belief', 0):.2f}, "
                                    f"depth {f'{_zb:.2f}m' if _zb else '?'})")
                        else:
                            self._usurp_streak = 0
                    else:
                        # 신규/만료 잠금: 확신 충분한 후보만 잠금 (오독 참칭 방지)
                        if best.get("belief", 0) >= 0.45:
                            chosen = best
                if chosen is not None:
                    prev = (lk or {}).get("bel")
                    new_b = chosen.get("belief", 0.5)
                    # born = 잠금 "최초 생성" 시각 (갱신에도 유지) — 탈취 허용 창 판정 기준.
                    # 잠금이 4초 이상 끊겼다 재생성되면 새 탐색으로 보고 창을 다시 연다.
                    lk_live = lk is not None and now - lk["ts"] < 4.0
                    _new_lock = not (lk_live and "born" in lk)
                    self._target_lock = {
                        "cx": (chosen["box"]["x1"] + chosen["box"]["x2"]) / 2,
                        "cy": (chosen["box"]["y1"] + chosen["box"]["y2"]) / 2,
                        "ts": now,
                        "born": now if _new_lock else lk["born"],
                        "bel": (0.7 * prev + 0.3 * new_b) if prev is not None else new_b}
                    if _new_lock:
                        # 잠금 대상까지의 depth를 남김 — 원거리 오탐(복도 건너편 물체)에
                        # 잠긴 것인지 로그만으로 판별 가능하게
                        _zl = self._depth_at(int(self._target_lock["cx"]),
                                             int(self._target_lock["cy"]))
                        self._dlog(f"[LOCK] '{tgt0}' 잠금 생성 (belief {new_b:.2f}, "
                                   f"depth {f'{_zl:.2f}m' if _zl else '?'})")
                # 참칭 후보(선택 안 된 동명이인)는 기억·화면에서 제외
                detections = [d for d in detections
                              if d["text"] != tgt0 or d is chosen]

            for d in detections:
                d["fresh"] = True
                self._det_mem[d["text"]] = {"det": d, "ts": now}
            merged = []
            for txt, e in list(self._det_mem.items()):
                # [A안] 타겟은 8초까지 기억 (로봇이 안 움직였으면 그 위치가 유효하므로)
                _keep = 8.0 if txt == tgt0 else DET_PERSIST_SEC
                if now - e["ts"] > _keep:
                    del self._det_mem[txt]
                    continue
                d = dict(e["det"])
                # fresh = "가장 최근 인식 라운드에서 봤다" (OCR이 ~1Hz라 시간 기준은 부적합)
                d["fresh"] = (e["ts"] == now)
                d["age"]   = now - e["ts"]   # 마지막으로 본 지 몇 초 (press 검증용)
                merged.append(d)
            detections = merged
            self._last_infer = now

            with state_lock:
                state["detections"] = detections
                phase    = state["phase"]
                lift     = state["lift"]
                pressing = state["pressing"]

            # centered 상태에서도 계속 호출 — 오차가 다시 벌어지면 CENTERED 해제(재정렬)
            # [중요] press 중엔 서보 잠금: 팔이 뻗는 동안 카메라도 같이 움직여
            # 오차가 요동치는데, 이때 서보가 lift 명령을 쏘면 팔 뻗기가 선점·취소됨
            if phase == "TRACK" and not pressing and self._goal_done and lift is not None:
                self._servo_step(detections, lift)

        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            self._processing = False

    def _annotate(self, frame, detections):
        vis = frame.copy()
        # 패널 단서(버튼같은 것) 회색 박스로 표시 — 원거리에서 뭘 단서로 삼는지 보이게
        if time.time() - getattr(self, "_panel_ts", 0) < 2.5:
            for pb in list(getattr(self, "_panel_boxes", [])):
                cv2.rectangle(vis, (int(pb["x1"]), int(pb["y1"])),
                              (int(pb["x2"]), int(pb["y2"])), (130, 130, 130), 1)
        _roi_now = getattr(self, "_roi_last", None)
        if _roi_now is not None:
            rx, ry, rw, rh = _roi_now
            cv2.rectangle(vis, (rx, ry), (rx+rw, ry+rh), (0, 255, 255), 1)
            cv2.putText(vis, "ROI", (rx + 4, ry + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        with state_lock:
            _td   = state["target_dist"]
            _tilt = state["wall_tilt"]
        _ox, _oy = _aim_offsets(_td)
        ax, ay = CX + _ox, CY + _oy   # 조준점 (거리별 손끝 투영 위치)
        # 벽 기울기 표시 (|3°| 미만 초록 = 평행 OK / 7° 미만 노랑 / 그 이상 빨강)
        if _tilt is not None:
            _tc = (0,255,80) if abs(_tilt) < 3 else ((0,220,255) if abs(_tilt) < 7 else (0,0,255))
            _txt(vis, f"TILT {_tilt:+.1f} deg", (10, 32), 0.8, _tc)
        cv2.line(vis, (ax-30, ay), (ax+30, ay), (255,255,0), 2)
        cv2.line(vis, (ax, ay-30), (ax, ay+30), (255,255,0), 2)
        cv2.circle(vis, (ax, ay), _dead_zone_px(_td), (255,255,0), 2)  # 거리 기반 허용원

        with state_lock:
            target = state["target_text"]; phase = state["phase"]
            centered = state["centered"]

        for d in detections:
            b = d["box"]
            x1,y1,x2,y2 = int(b["x1"]),int(b["y1"]),int(b["x2"]),int(b["y2"])
            bcx,bcy = (x1+x2)//2,(y1+y2)//2
            is_target = (phase == "TRACK" and d["text"] == target)
            fresh = d.get("fresh", True)
            color = (0,0,255) if is_target else _label_color(d["text"])
            # 유지(coast) 중인 박스는 얇게 + 라벨에 ~ 접두사
            cv2.rectangle(vis, (x1,y1), (x2,y2), color,
                          (3 if is_target else 2) if fresh else 1)
            # 모든 인식 박스에 거리 표시 (depth 패치 중앙값 — 비용 무시 가능 수준)
            # → 선택(서보 발동) 없이도 depth 검증 가능
            dist = self._depth_at(bcx, bcy)
            dist_txt = f"{dist:.2f}m" if dist else "?m"
            # 라벨 정리: 목표만 상세(점수/거리), 나머지는 숫자만 작게 — 화면 가림 방지
            if is_target:
                label, fs, bh = f"{'~' if not fresh else ''}{d['text']} {d['score']:.2f} {dist_txt}", 0.5, 20
            else:
                label, fs, bh = f"{'~' if not fresh else ''}{d['text']}", 0.45, 15
            bw = int(len(label) * 18 * fs)
            cv2.rectangle(vis, (x1, y1-bh), (x1+bw, y1), color, -1)
            cv2.putText(vis, label, (x1+2, y1-4),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, (0,0,0), 1)
            if is_target:
                cv2.arrowedLine(vis, (ax,ay), (bcx,bcy), (0,80,255), 2, tipLength=0.2)
                with state_lock:
                    state["target_dist"] = round(dist, 3) if dist else None
                _txt(vis, dist_txt, (x2+6, bcy+8), 0.9, (0,255,255))
                _txt(vis, f"x:{bcx-ax:+d} y:{bcy-ay:+d} d:{dist_txt}", (10, IMAGE_H-14),
                     0.75, (60,160,255))
        if centered:
            # 준비 완료(정지·클릭 대기)와 "정렬만 됨, 아직 접근 중"을 화면에서 구분
            if self._is_press_ready():
                cv2.putText(vis, "READY - PRESS", (CX-110, CY-50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,80), 3)
            else:
                cv2.putText(vis, "ALIGNED (approaching)", (CX-130, CY-50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,220,255), 2)
        return vis

    def _scan_step(self, lift, amp_l=0.05, amp_b=0.03, amp_e=0.04):
        """소폭 탐색 이동 한 스텝: lift 위아래 / 팔(그리퍼) 앞뒤 / 베이스 앞뒤.
        모두 기준점 복귀 포함, 3초에 한 동작.
        - 팔 앞뒤 = 카메라-벽 거리 변화 = 글자 크기 스윕 (인식 스윗스팟 탐색)
        - 팔 전진은 depth로 벽 여유 확인 후에만 (손끝 충돌 방지)
        - 베이스는 가드 로직 그대로 통과 (막힌 방향 자동 생략)"""
        # [읽을 틈 보장] 전역 탐색 관문 + 스캔 자체 쿨다운 4초
        if time.time() - getattr(self, "_scan_move_ts", 0) < 4.0:
            return
        if not self._may_explore():
            return
        if getattr(self, "_scan_home_lift", None) is None:
            self._scan_home_lift = float(lift)   # 스캔 기준점 고정
        if getattr(self, "_scan_home_ext", None) is None:
            with state_lock:
                _e = state["arm_ext"]
            self._scan_home_ext = float(_e) if _e is not None else None
        home = self._scan_home_lift
        eh   = self._scan_home_ext
        pattern = [
            ("lift", home + amp_l), ("lift", home - amp_l), ("lift", home),
            ("arm",  +amp_e), ("arm", 0.0),       # 팔 앞으로 갔다 복귀 (글자 확대)
            ("base", +amp_b), ("base", -amp_b),   # 베이스 앞으로 갔다 복귀
            ("arm",  -amp_e), ("arm", 0.0),       # 팔 뒤로 갔다 복귀 (글자 축소)
            ("base", -amp_b), ("base", +amp_b),   # 베이스 뒤로 갔다 복귀
        ]
        self._scan_seq = getattr(self, "_scan_seq", 0)
        with state_lock:
            _base_ok = state["base_align"]
        for _ in range(len(pattern)):
            kind, val = pattern[self._scan_seq % len(pattern)]
            self._scan_seq += 1
            if kind == "base" and not _base_ok:
                continue   # 몸체이동 OFF — 베이스 스텝은 거부될 것이므로 슬롯 낭비 방지
            break
        self._scan_move_ts = time.time()
        self._dlog(f"[SCAN] 탐색 {self._scan_seq}: {kind} {val:+.2f}")
        if kind == "lift":
            self._send_goal(["joint_lift"], [max(0.15, min(1.10, val))])
        elif kind == "arm":
            if eh is None:
                return
            if val > 0:   # 벽 쪽 전진은 여유 확인 (열린 손끝 ~17cm + 마진)
                wall_d = self._depth_at(CX, CY)
                if wall_d is not None and wall_d - val < 0.24:
                    self._dlog("[SCAN] 팔 전진 생략 — 벽 여유 부족")
                    return
            self._send_goal([ARM_JOINT],
                            [max(ARM_EXT_MIN, min(ARM_EXT_MAX, eh + val))])
        else:
            threading.Thread(target=self._base_move, args=(val,),
                             daemon=True).start()

    def _servo_step(self, detections, lift):
        with state_lock:
            target = state["target_text"]
            tdist  = state["target_dist"]
            _tilt  = state["wall_tilt"]
            _flat  = state["wall_flat"]

        # ── 자동 수직 정렬: 벽 기울기(평면 검증 통과)가 4° 넘으면 그만큼 제자리 회전 ──
        # tilt<0(왼쪽이 멂) → 시계방향("오른쪽으로") / tilt>0 → 반시계. 재측정 반복 수렴.
        _rot_ok = False
        if (_flat and _tilt is not None and abs(_tilt) > 4.0
                and not self._nudging
                and not self._is_press_ready()   # 클릭 대기 중엔 회전 금지 (동결)
                and getattr(self, "_rot_count", 0) < 4
                and self._may_explore()
                and time.time() - getattr(self, "_rot_ts", 0) > 3.0):
            # [진동 방지] 정지 상태에서 잰 기울기 2회가 부호 일치 + 3° 이내로 일관해야 회전
            _lm = getattr(self, "_last_motion_ts", 0)
            _h = [t for ts_, t in list(getattr(self, "_tilt_hist", []))
                  if ts_ > _lm + 0.8]
            if len(_h) >= 2 and _h[-1] * _h[-2] > 0 and abs(_h[-1] - _h[-2]) < 3.0:
                _tilt = (_h[-1] + _h[-2]) / 2   # 평균으로 회전량 결정
                _rot_ok = True
        if _rot_ok:
            # 몸체이동 스위치 OFF면 회전 불가 — 후퇴부터 하고 회전 못 하는
            # "접기↔뻗기 무한 루프" 방지를 위해 여기서 먼저 확인
            with state_lock:
                _can_move = state["base_align"]
            if not _can_move:
                if not getattr(self, "_rot_warned", False):
                    self._rot_warned = True
                    self._dlog(f"[ROTATE] 벽 기울기 {_tilt:+.1f}° 감지했으나 "
                               "몸체이동 OFF — 회전 생략 (켜면 자동 정렬)")
            else:
                self._rot_ts = time.time()
                self._rot_count = getattr(self, "_rot_count", 0) + 1
                self._dlog(f"[ROTATE] 벽 수직 정렬 {self._rot_count}/4: "
                           f"{_tilt:+.1f}° (팔 접고 회전 — 원자적 실행)")
                threading.Thread(target=self._rotate_seq,
                                 args=(float(np.radians(_tilt)),),
                                 daemon=True).start()
                return
        if _flat and _tilt is not None and abs(_tilt) <= 3.0:
            self._rot_count = 0   # 수직 유지 중이면 카운터 회복
        det = next((d for d in detections if d["text"] == target), None)
        # [A안] 관측 신뢰 규칙: fresh거나, "그 관측 이후 로봇이 안 움직였다면" 8초까지 유효.
        # (버튼은 정지해 있고 자기 이동은 전부 기록됨 — 시각은 간헐적 교정자면 충분)
        # 단 "같은 관측으로는 1회만" 행동 — 낡은 박스 기반 반복 이동(폭주) 방지.
        e_mem = self._det_mem.get(target)
        lm = getattr(self, "_last_motion_ts", 0)
        usable = False
        if det is not None:
            if det.get("fresh", False):
                usable = True
            elif det.get("age", 9.9) <= 8.0 and e_mem and e_mem["ts"] > lm + 0.3:
                usable = e_mem["ts"] != getattr(self, "_servo_acted_ts", None)
        if not usable:
            # 클릭 대기(누르기 활성) 중엔 인식이 끊겨도 SEEK/스캔으로 움직이지 않음 — 동결 유지
            if self._is_press_ready():
                return
            # [진동 방지] 방금(4초 내)까지 그 자리에서 타겟을 봤으면 — 움직이지 말고
            # 재인식을 기다린다. (정렬 위치에서 일시적으로 안 읽힐 때, 군집 추적이
            # 시야를 끌고 내려가 "내려가면 읽히고 올라가면 놓치는" 무한 왕복 방지)
            lk_hold = getattr(self, "_target_lock", None)
            if lk_hold and time.time() - lk_hold["ts"] < 4.0:
                return

            # 타겟 미인식 → "버튼처럼 생긴 것들"(비숫자 포함) 군집 = 패널 후보 중심으로
            # 시야를 옮김 (집요 추적). 글자를 못 읽는 원거리에서도 작동.
            boxes = list(getattr(self, "_panel_boxes", []))
            if time.time() - getattr(self, "_panel_ts", 0) > 2.5:
                boxes = []
            if len(boxes) >= 2:
                mx = sum((b["x1"] + b["x2"]) / 2 for b in boxes) / len(boxes)
                my = sum((b["y1"] + b["y2"]) / 2 for b in boxes) / len(boxes)
                sex, sey = mx - CX, my - CY
                # 군집 depth 게이트: 박스별 depth 중 가장 가까운 값으로 판정.
                # ① 최근접이 1.5m 밖 → 패널일 수 없음 (팔 0.5m)
                # ② 전부 측정 불가 → 검은 바퀴 등 IR 흡수 오탐 의심 (실측 사례)
                # 어느 쪽이든 쫓아가지 않고 제자리 대기 (진짜 패널이면 글자 판독이 곧 잡음)
                _zs = [self._depth_at(int((b["x1"] + b["x2"]) / 2),
                                      int((b["y1"] + b["y2"]) / 2)) for b in boxes]
                _zv = [z for z in _zs if z is not None]
                if (not _zv) or min(_zv) > DET_MAX_DEPTH:
                    if time.time() - getattr(self, "_seek_far_ts", 0) > 5.0:
                        self._seek_far_ts = time.time()
                        self._dlog("[SEEK] 군집 depth 전부 측정불가 — 오탐 의심, 접근 안 함"
                                   if not _zv else
                                   f"[SEEK] 군집이 {min(_zv):.2f}m 밖 — 오탐 의심, 접근 안 함")
                    return
                COARSE = 45          # 군집은 대략 중앙이면 충분 (정밀 정렬은 타겟 발견 후)
                if not self._may_explore():
                    return
                if abs(sey) >= COARSE:
                    self._dlog(f"[SEEK] 군집 중심으로 높이 이동 (y{sey:+.0f}px)")
                    self._send_goal(["joint_lift"],
                                    [max(0.15, min(1.10, float(lift) - KP_LIFT * sey))])
                elif abs(sex) >= COARSE:
                    self._dlog(f"[SEEK] 군집 중심으로 전후 이동 (x{sex:+.0f}px)")
                    self._maybe_base_nudge(sex, None)
                else:
                    # 군집은 중앙인데 타겟 글자가 안 읽힘 → 우선 "다가가서" 읽기 시도
                    # (가까울수록 글자가 커져 읽힘. depth로 벽 여유 26cm+ 유지)
                    wall_d = self._depth_at(CX, CY)
                    if (wall_d and wall_d > 0.30 and not self._nudging
                            and self._may_explore()):
                        with state_lock:
                            ext_now = state["arm_ext"]
                        if ext_now is not None:
                            step = min(0.04, wall_d - 0.26)
                            self._approach_sent = time.time()
                            self._dlog(
                                f"[APPROACH] 군집 접근 +{step*100:.0f}cm (벽 {wall_d:.2f}m)")
                            self._send_goal([ARM_JOINT],
                                [max(ARM_EXT_MIN, min(ARM_EXT_MAX,
                                                      float(ext_now) + step))])
                            return
                    # 더 못 다가가는데도 8초+ 안 읽힘 → 시점 흔들기
                    e_t = self._det_mem.get(target)
                    last_seen = e_t["ts"] if e_t else 0
                    if time.time() - last_seen > 8.0:
                        self._scan_step(lift, amp_l=0.03, amp_b=0.02)
                return

            # 아무 단서도 없음 → 더 넓게 두리번 (lift ±5cm, 베이스 ±3cm)
            self._scan_step(lift, amp_l=0.05, amp_b=0.03)
            return
        # 타겟 확보 → 탐색 스캔 상태 리셋
        self._scan_home_lift = None
        self._scan_home_ext  = None
        self._scan_seq = 0
        b  = det["box"]
        # 조준점(카메라 중앙 + 거리별 손끝 오프셋) 기준 2축 오차
        ox, oy = _aim_offsets(tdist)
        ex = (b["x1"]+b["x2"])/2 - (CX + ox)
        ey = (b["y1"]+b["y2"])/2 - (CY + oy)
        dz = _dead_zone_px(tdist)          # 거리 기반 허용오차 (±0.6cm 상당)
        if abs(ex) < dz and abs(ey) < dz:
            with state_lock:
                first = not state["centered"]
                state["centered"] = True
                state["centered_ts"] = time.time()   # press 클릭 관용 창 판정용
            if first:
                self._dlog("CENTERED!")
            # ── 누르기 활성(정조준 + 60cm 이내) 순간부터 로봇 완전 정지 ──
            # 자동 접근·polish 서보·베이스 정렬 전부 동결 (사용자 요청: "초록불이면
            # 무조건 멈춰서 누를 수 있게"). 클릭하면 press 시퀀스가 접근~누르기 담당.
            # 오차가 히스테리시스(+4px)를 넘어 CENTERED가 풀리면 동결도 풀려 재정렬.
            if tdist is not None and tdist <= PRESS_READY_DIST:
                if not getattr(self, "_ready_logged", False):
                    self._ready_logged = True
                    self._dlog(f"[READY] 누르기 활성 ({tdist:.2f}m) — "
                               "자동 이동 전체 동결, 클릭 대기")
                return
            if getattr(self, "_ready_logged", False):
                self._ready_logged = False
            # ── 자동 접근: 정렬됐으면 누르기 직전 거리(20cm)까지 팔을 스스로 전진 ──
            # 살아있는 관측(fresh)일 때만 한 걸음(≤4cm)씩. 정렬이 흐트러지면
            # 아래 재정렬 로직이 잡은 뒤 다시 전진. 접촉 없음(손끝 3cm 앞 정지).
            # 실제 누르기(닫고 꾹)는 여전히 사용자의 명시적 클릭으로만.
            # ── 자동 접근: OCR 없이 "실시간 depth"로 전진 ("한 프레임 문제" 해결) ──
            # 글자는 한 번만 읽혀도 됨 — 정렬(잠금)만 살아 있으면, 남은 전진은
            # 조준점의 depth(매 프레임 갱신, 인식과 무관)가 거리를 대준다.
            lk_ap = getattr(self, "_target_lock", None)
            if (lk_ap and time.time() - lk_ap["ts"] < 20.0
                    and self._goal_done and not self._nudging
                    and time.time() - getattr(self, "_approach_sent", 0) > 1.5):
                d_live = self._depth_at(CX + ox, CY + oy)   # 조준점 실시간 거리
                if d_live and d_live > PRESS_CLOSE_DIST + 0.03:
                    with state_lock:
                        ext_now = state["arm_ext"]
                    if ext_now is not None:
                        # 팔 한계에 닿았는데 아직 멀면 → 사용자에게 알림 (조용한 교착 방지)
                        if float(ext_now) >= ARM_EXT_MAX - 0.005:
                            if time.time() - getattr(self, "_arm_max_ts", 0) > 8.0:
                                self._arm_max_ts = time.time()
                                self._dlog(f"[APPROACH] 팔 한계(0.5m) 도달 — 버튼 {d_live:.2f}m 남음. "
                                           "로봇을 더 가까이 대야 누르기가 활성화됩니다")
                            return
                        step = max(0.02, min(0.06,
                                   0.35 * (d_live - PRESS_CLOSE_DIST)))
                        step = min(step, d_live - PRESS_CLOSE_DIST)
                        self._approach_sent = time.time()
                        self._dlog(
                            f"[APPROACH] 자동 접근(depth) +{step*100:.0f}cm "
                            f"(버튼 {d_live:.2f}m)")
                        self._send_goal([ARM_JOINT],
                            [max(ARM_EXT_MIN, min(ARM_EXT_MAX,
                                                  float(ext_now) + step))])
                        return   # 팔 이동 중엔 다른 보정 안 함 (같은 컨트롤러 선점 방지)

            # ── 정밀 다듬기(polish): 허용 안이라도 "절반 기준"까지 계속 0으로 수렴 ──
            # 확대 화면에서도 버튼이 십자에 붙도록. fresh 관측 + 관측 1회당 1보정.
            act = max(4, int(dz * 0.5))
            if det.get("fresh") and (abs(ex) >= act or abs(ey) >= act):
                e_pol = self._det_mem.get(target)
                if e_pol and e_pol["ts"] != getattr(self, "_polish_ts", None):
                    self._polish_ts = e_pol["ts"]
                    if abs(ey) >= act:
                        self._send_goal(["joint_lift"],
                            [max(0.15, min(1.10,
                                           float(lift) - KP_LIFT * ey * 0.7))])
                    if abs(ex) >= act:
                        self._maybe_base_nudge(ex, tdist)
            return
        # CENTERED는 래치가 아님 — 히스테리시스(+4px) 넘게 벗어나면 해제 후 재정렬
        with state_lock:
            was_centered = state["centered"]
            if was_centered and (abs(ex) > dz + 4 or abs(ey) > dz + 4):
                state["centered"] = False
                was_centered = False
        if was_centered:
            return   # 경계 근처 미세 진동은 무시 (모터 덜덜거림 방지)
        # 이 관측(ts)으로 보정했음을 기록 — 낡은 관측 재사용 방지용
        e_mem = self._det_mem.get(target)
        if e_mem:
            self._servo_acted_ts = e_mem["ts"]
        # 상하(ey) → lift 서보. yaw는 고정(카메라만 돌 뿐 손끝 경로를 못 옮김).
        if abs(ey) >= dz:
            self._send_goal(["joint_lift"],
                            [max(0.15, min(1.10, float(lift) - KP_LIFT * ey))])
        # 좌우(ex) → 라이다-가드 베이스 전/후진 (토글 ON일 때만, 안전 확인 후)
        if abs(ex) >= dz:
            self._maybe_base_nudge(ex, tdist)

    # ── 누르기(press) 시퀀스 ──────────────────────────────────────────
    def _move_joint_wait(self, joint: str, pos: float, sec: int, timeout: float) -> bool:
        """관절 하나를 목표 위치로 움직이고 완료까지 블로킹 대기. 성공 여부 반환.
        (press 전용 — 별도 스레드에서 호출됨)"""
        self._last_motion_ts = time.time()   # [A안] 이동 발생 기록
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            return False
        done = threading.Event()
        ok = {"v": False}
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint]
        pt = JointTrajectoryPoint()
        pt.positions = [float(pos)]
        pt.time_from_start.sec = int(sec)
        goal.trajectory.points = [pt]

        def on_result(_):
            ok["v"] = True
            done.set()

        def on_resp(fut):
            h = fut.result()
            if h.accepted:
                h.get_result_async().add_done_callback(on_result)
            else:
                done.set()

        self.action_client.send_goal_async(goal).add_done_callback(on_resp)
        done.wait(timeout)
        return ok["v"]

    def start_press(self) -> str | None:
        """누르기 시퀀스 시작. 문제 있으면 사유 문자열, 정상 시작이면 None."""
        with state_lock:
            if state["pressing"]:
                return "이미 누르기 진행 중"
            # 깜빡임 관용: 방금(2초 내)까지 CENTERED였으면 클릭 접수 —
            # 진짜 정렬 여부는 아래 라이브 오차 재검증이 판정 (어긋났으면 사유 반환)
            if not state["centered"] and \
                    time.time() - state.get("centered_ts", 0.0) > 2.0:
                return "CENTERED 상태가 아님 (버튼 선택 후 정렬 완료까지 대기)"
            d    = state["target_dist"]
            ext  = state["arm_ext"]
            tgt  = state["target_text"]
            dets = list(state["detections"])
        # ── 누르는 순간의 실제 오차 재검증 (CENTERED 래치를 신뢰하지 않음) ──
        if time.time() - getattr(self, "_last_infer", 0) > 3.0:
            return "인식이 멈춰 있음 (3초+) — OCR 상태 확인"
        # 로봇이 정지 상태이므로 2초 내에 잡힌 박스면 위치 검증에 충분
        # (인식이 깜빡여도 press가 부당하게 거부되지 않도록)
        # [A안] press 검증도 같은 규칙: 2초 내 관측 OR (8초 내 + 그 후 로봇 정지)
        e_pm = self._det_mem.get(tgt)
        lm_p = getattr(self, "_last_motion_ts", 0)
        def _obs_ok(x):
            if x.get("text") != tgt:
                return False
            if x.get("age", 9.9) < 2.0:
                return True
            return (x.get("age", 9.9) < 8.0 and e_pm is not None
                    and e_pm["ts"] > lm_p + 0.3)
        det = next((x for x in dets if _obs_ok(x)), None)
        if det is None:
            # [한 프레임 대응] 최근 관측은 없지만 위치 잠금이 20초 내면 진행 허용.
            # press 내부의 depth 재측정·근접 재정렬·접촉 판정이 최종 검증을 담당.
            lk_pv = getattr(self, "_target_lock", None)
            if not (lk_pv and time.time() - lk_pv["ts"] < 20.0):
                return "버튼 관측이 오래됨 (이동 후 재확인 안 됨) — 재인식 대기"
            self._dlog("[PRESS] 단일 관측(잠금) 기반 진행 — depth가 검증")
        else:
            _ox, _oy = _aim_offsets(d)
            _bx = (det["box"]["x1"] + det["box"]["x2"]) / 2 - (CX + _ox)
            _by = (det["box"]["y1"] + det["box"]["y2"]) / 2 - (CY + _oy)
            _dz = _dead_zone_px(d) + 2   # 경계선 재측정 잡음으로 인한 억울한 거부 방지
            if abs(_bx) >= _dz or abs(_by) >= _dz:
                # CENTERED 해제 → 서보가 즉시 재정렬 (히스테리시스 교착 방지)
                with state_lock:
                    state["centered"] = False
                return (f"정렬 어긋남 (x{_bx:+.0f} y{_by:+.0f}px, 허용±{_dz}) — "
                        "자동 재정렬 시작, 몇 초 뒤 다시 누르세요")
        if d is None:
            return "거리 측정 안 됨 (depth ?m)"
        if d > PRESS_DIST_MAX:
            return f"너무 멂 ({d:.2f}m > {PRESS_DIST_MAX}m) — 로봇을 더 가까이"
        if d > PRESS_READY_DIST:
            return (f"아직 접근 완료 전 ({d:.2f}m) — 자동 접근이 "
                    f"{PRESS_READY_DIST*100:.0f}cm 안쪽까지 간 뒤 누르기가 활성화됩니다")
        if ext is None:
            return "팔 위치 미수신 (joint_states 확인)"
        threading.Thread(target=self._press_sequence, args=(d, ext, tgt),
                         daemon=True).start()
        return None

    def _press_sequence(self, d: float, start_ext: float, tgt: str = None):
        """[열린 채] 근접 → 재측정·근접 재정렬 → [닫고] 누르기 → 복귀 → 열기."""
        def st(msg):
            with state_lock:
                state["press_status"] = msg
            self._dlog(f"[PRESS] {msg}")

        with state_lock:
            state["pressing"] = True
        try:
            # READY 동결 상태(≤PRESS_READY_DIST, 정조준 완료)에서 클릭된 경우:
            # 조준은 이미 끝났고 남은 건 몇 cm 전진뿐 → 접근 중 보정·근접 재정렬을
            # 전부 생략하고 "뻗기→닫기→꾹"만 실행 (사용자 요청: 클릭 후 미세조정 금지)
            direct = d <= PRESS_READY_DIST + 0.02
            # A. 그리퍼 연 채 "정렬을 유지하며" 단계 접근:
            #    5cm 전진 → 이동 후 인식 대기 → 박스↔십자 보정 → 반복
            #    (박스 중앙 = 십자 중앙을 움크리기 직전까지 유지)
            d_now = d
            if direct:
                remain0 = d_now - PRESS_CLOSE_DIST
                if remain0 > 0.01:
                    with state_lock:
                        ext_now0 = state["arm_ext"]
                    if ext_now0 is None:
                        ext_now0 = start_ext
                    st(f"1/6 직행 접근: +{remain0*100:.0f}cm (조정 없음)")
                    if not self._move_joint_wait(
                            ARM_JOINT,
                            max(ARM_EXT_MIN, min(ARM_EXT_MAX,
                                                 float(ext_now0) + remain0)),
                            2, 8.0):
                        st("❌ 접근 실패 — 중단"); return
                    d_now = PRESS_CLOSE_DIST
            for _s in range(0 if direct else 8):
                remain = d_now - PRESS_CLOSE_DIST
                if remain <= 0.01:
                    break
                step = min(0.05, remain)
                with state_lock:
                    ext_now = state["arm_ext"]
                if ext_now is None:
                    ext_now = start_ext
                tgt_ext = max(ARM_EXT_MIN, min(ARM_EXT_MAX, float(ext_now) + step))
                st(f"1/6 정렬 유지 접근 {_s+1}: +{step*100:.0f}cm (버튼까지 ~{d_now*100:.0f}cm)")
                if not self._move_joint_wait(ARM_JOINT, tgt_ext, 2, 8.0):
                    st("❌ 접근 실패 — 중단"); return
                # 이동 "후" 프레임의 인식 대기
                end_t = time.time()
                while (getattr(self, "_last_infer", 0) < end_t + 1.0
                       and time.time() - end_t < 4.0):
                    time.sleep(0.1)
                # 거리 갱신 + 중간 보정 (상하 lift / 좌우 베이스)
                with state_lock:
                    d_read = state["target_dist"]
                    dets_m = list(state["detections"])
                d_now = d_read if (d_read and 0.08 < d_read < 1.0) \
                        else max(PRESS_CLOSE_DIST, d_now - step)
                det_m = next((x for x in dets_m
                              if x.get("text") == tgt and x.get("age", 9.9) < 2.5), None)
                if det_m:
                    oxm, oym = _aim_offsets(d_now)
                    bxm = (det_m["box"]["x1"] + det_m["box"]["x2"]) / 2 - (CX + oxm)
                    bym = (det_m["box"]["y1"] + det_m["box"]["y2"]) / 2 - (CY + oym)
                    dzm = _dead_zone_px(d_now)
                    if abs(bym) > dzm:
                        with state_lock:
                            lift_now = state["lift"]
                        if lift_now is not None:
                            self._move_joint_wait("joint_lift",
                                max(0.15, min(1.10, float(lift_now) - KP_LIFT * bym)),
                                1, 6.0)
                    if abs(bxm) > dzm:
                        mvm = max(-0.02, min(0.02,
                                  BASE_X_SIGN * bxm * d_now / 420.0 * 0.7))
                        if abs(mvm) >= 0.004:
                            self._base_move(mvm)
            with state_lock:
                near_ext = state["arm_ext"] or start_ext

            # B. 가까워진 위치에서 거리 재측정 (depth 갱신 한 사이클 대기)
            st("2/6 거리 재측정 중…")
            time.sleep(1.3)
            with state_lock:
                d2 = state["target_dist"]
            if not (d2 and 0.08 < d2 < 0.35):
                d2 = max(0.05, d_now)        # 재측정 실패 시 접근 루프의 추정값 사용
                st(f"재측정 실패 → 계산값 사용 ({d2:.3f}m)")
            else:
                st(f"재측정: 버튼까지 {d2:.3f}m")

            # B-2. 근접 재정렬: 접근 중 처짐/벽 기울기로 생긴 오차를 "가까운 거리"에서
            #      다시 잡는다. 여기서의 1px는 실거리로 훨씬 작아 정밀함.
            #      (press 중 자동 서보는 잠겨 있으므로 여기서 통제된 lift 보정만 수행)
            if tgt and not direct:
                # 근접 재정렬: 상하(lift) + 좌우(베이스, 가드 확인 포함) 모두 이 거리에서 보정
                # — 접근 중 생긴 드리프트를 복귀 없이 그 자리에서 해결
                # (direct 모드에서는 생략 — READY 동결 시점의 조준을 그대로 신뢰)
                dz2 = _dead_zone_px(d2)
                last_err = None
                aligned  = False
                for _i in range(5):
                    det2 = None
                    t_w = time.time()
                    while time.time() - t_w < 3.0:      # 신선한 타겟 재인식 대기
                        with state_lock:
                            dets2 = list(state["detections"])
                        det2 = next((x for x in dets2
                                     if x.get("text") == tgt and x.get("fresh")), None)
                        if det2: break
                        time.sleep(0.2)
                    if det2 is None:
                        st("근접 재정렬: 재인식 실패 — 기존 조준 유지"); break
                    ox2, oy2 = _aim_offsets(d2)
                    bx2 = (det2["box"]["x1"] + det2["box"]["x2"]) / 2 - (CX + ox2)
                    by2 = (det2["box"]["y1"] + det2["box"]["y2"]) / 2 - (CY + oy2)
                    last_err = (bx2, by2)
                    tx2 = max(6, int(dz2 * 0.75))   # 좌우는 더 엄격 (경계 통과 → 좌우 어긋남 실측)
                    if abs(bx2) <= tx2 and abs(by2) <= dz2:
                        st(f"근접 재정렬 OK (x{bx2:+.0f} y{by2:+.0f}px)")
                        aligned = True
                        break
                    moved = False
                    if abs(by2) > dz2:                   # 상하 → lift
                        with state_lock:
                            lift_now = state["lift"]
                        if lift_now is not None:
                            st(f"근접 재정렬 {_i+1}/5: 높이 보정 (y{by2:+.0f}px)")
                            self._move_joint_wait("joint_lift",
                                max(0.15, min(1.10, float(lift_now) - KP_LIFT * by2)),
                                1, 6.0)
                            moved = True
                    if abs(bx2) > max(6, int(dz2 * 0.75)):   # 좌우 → 베이스 (가드 존중)
                        mv = BASE_X_SIGN * bx2 * d2 / 420.0 * 0.7
                        mv = max(-0.025, min(0.025, mv))
                        if abs(mv) >= 0.004:   # 오도메트리 피드백이라 4mm도 정확
                            st(f"근접 재정렬 {_i+1}/5: 좌우 보정 {mv*100:+.1f}cm (x{bx2:+.0f}px)")
                            if not self._base_move(mv):
                                st("❌ 좌우 보정 불가 (막힘) — 복귀")
                                self._move_joint_wait(ARM_JOINT, start_ext, 4, 12.0)
                                return
                            moved = True
                    if not moved:
                        break
                    # 이동 "후" 프레임 인식이 나올 때까지 대기 (낡은 오차로 과보정 방지)
                    end_t = time.time()
                    while (getattr(self, "_last_infer", 0) < end_t + 1.0
                           and time.time() - end_t < 4.0):
                        time.sleep(0.1)
                if not aligned and last_err and (
                        abs(last_err[0]) > dz2 * 2 or abs(last_err[1]) > dz2 * 2):
                    st(f"❌ 근접 재정렬 실패 (x{last_err[0]:+.0f} y{last_err[1]:+.0f}px) — 복귀")
                    self._move_joint_wait(ARM_JOINT, start_ext, 4, 12.0)
                    return

            # C. 이제서야 그리퍼 닫기 (누르기 직전)
            st("3/6 그리퍼 닫기 (누르기 준비)")
            if not self._move_joint_wait(GRIPPER_JOINT, GRIPPER_CLOSE_M, 2, 8.0):
                st("❌ 그리퍼 닫기 실패 — 복귀")
                self._move_joint_wait(ARM_JOINT, start_ext, 4, 12.0)
                return

            # D. 누르기: (남은 거리 - 손끝 스탠드오프) + 버튼 스트로크
            push = max(ARM_EXT_MIN, min(ARM_EXT_MAX,
                       near_ext + (d2 - FINGER_STANDOFF) + PRESS_DEPTH))
            st(f"4/6 누르는 중… →{push:.3f}m")
            if not self._move_joint_wait(ARM_JOINT, push, 2, 8.0):
                st("❌ 누르기 실패 — 복귀")
            time.sleep(0.4)

            # 접촉 감지 (위치 오차): 명령보다 5mm 이상 못 갔으면 막힌 것 = 닿은 것
            with state_lock:
                actual = state["arm_ext"]
            if actual is not None:
                shortfall = push - actual
                contact = shortfall > 0.005
                st(f"접촉 판정: {'✓ 닿음' if contact else '✗ 허공?'} "
                   f"(명령 {push:.3f} vs 실제 {actual:.3f}, 차이 {shortfall*1000:.0f}mm)")
                time.sleep(0.8)  # 판정 메시지 읽을 시간

            st(f"5/6 복귀 중… →{start_ext:.3f}m")
            self._move_joint_wait(ARM_JOINT, start_ext, 4, 12.0)

            st("6/6 그리퍼 여는 중… (인식 모드 복귀)")
            self._move_joint_wait(GRIPPER_JOINT, GRIPPER_OPEN_M, 2, 8.0)
            st("✅ 누르기 완료")
            # 수행 완료 → 타겟 자동 해제 (안 하면 추적·자동접근이 같은 버튼에 재시작됨)
            with state_lock:
                state["target_text"] = None
                state["phase"]       = "SELECT"
                state["centered"]    = False
        finally:
            with state_lock:
                state["pressing"] = False

    def _send_single_joint(self, joint_name: str, position: float, duration_sec: int = 2):
        self._last_motion_ts = time.time()   # [A안] 수동 조작도 이동 — 관측 기억 무효화
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
        self._dlog(f"{joint_name} -> {position:.2f} rad")

    def set_wrist_pitch(self, pitch: float):
        self._send_single_joint("joint_wrist_pitch", pitch)

    def set_lift(self, lift: float):
        self._send_single_joint("joint_lift", lift, duration_sec=3)

    def set_gripper(self, aperture_m: float):
        self._send_single_joint(GRIPPER_JOINT, aperture_m, duration_sec=2)

    def set_wrist_yaw(self, yaw: float):
        self._send_single_joint("joint_wrist_yaw", yaw)

    def _send_goal(self, joint_names, positions):
        self._last_motion_ts = time.time()   # [A안] 이동 발생 기록
        if not self.action_client.wait_for_server(timeout_sec=1.0): return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]
        self._goal_done = False
        self.action_client.send_goal_async(goal).add_done_callback(self._on_goal_response)
        self._dlog("-> " + ", ".join(
            f"{n}={p:.3f}" for n, p in zip(joint_names, positions)))

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
