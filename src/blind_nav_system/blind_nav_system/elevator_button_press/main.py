#!/usr/bin/env python3
"""
엘리베이터 버튼 추적 메인 스크립트

실행: python3 main.py
브라우저: http://localhost:5000
"""

import os, json, math, shutil, subprocess, tempfile, threading, time, webbrowser
import collections
import logging
import cv2
import numpy as np
# FastDDS 공유메모리 비활성화(UDP 강제) — 어느 터미널에서 시작해도 적용되도록
# 코드에서 직접 주입 (rclpy import 전이어야 함). SHM 반복 고장 방지 — 2026-07-24
os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE",
                      os.path.expanduser("~/.ros/fastdds_no_shm.xml"))
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# ── 진단 로거 (간헐 버그 블랙박스) — 상위 패키지 폴더의 robot_diag.py 사용 ──
import sys as _sys
_sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
try:
    import robot_diag as _diag
except Exception as _e:          # 로거 없어도 본체는 정상 동작해야 함
    _diag = None
    print(f"[경고] robot_diag 로드 실패: {_e}")
_diaglog = None
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, LaserScan, CameraInfo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from flask import Flask, Response, jsonify, request, render_template_string

VENV_PYTHON   = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../venv/bin/python3"))
INFER_SERVER  = os.path.join(os.path.dirname(__file__), "ocr_rcnn_server.py")

# 그리퍼 카메라 고정 자세 (전방을 향하는 값)
# [실험 C 2026-07-09] 정반사(glare) 회피: 기존 0.07에서 5° 하향(-0.02).
# "중앙 정렬 시에만 인식 실패" = 카메라⊥패널 정반사 기하 → 각도를 깨서 원인 제거.
# 효과 없으면 0.07로 번복. 변경 후 착지 세로 오차는 캘리브레이션 press 1회로 재보정 필요.
WRIST_PITCH_DEFAULT = -0.02
WRIST_YAW_DEFAULT   = 0.03
WRIST_YAW_IN        = 3.4    # 이동 단계(②③④⑥) 그리퍼 안쪽 수납 각 — 문틀 충돌 방지.
                             # tools/armleft.py의 검증값 (+3.4 rad = 몸통 안쪽)

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


# 추론 소요 집계 로그 주기(초). 매 추론마다 찍으면 초당 2줄이라 로그가 익사한다.
INFER_LOG_PERIOD = 10.0


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
# 스냅샷용 camera_info — 위 이미지 토픽들과 같은 네임스페이스 후보(부팅마다 갈릴 수 있음)
GRIPPER_INFO_TOPICS = [
    "/camera/gripper_camera/color/camera_info",
    "/gripper_camera/color/camera_info",   # (예비)
]
BODY_INFO_TOPIC = "/camera/camera/color/camera_info"

# ── 고립 관측(A5) — 주기 샘플 상수 ────────────────────────────────────────────
# 지금까지 드라이버 도달성·프레임 도착에 대한 "주기적" 관측이 하나도 없었다.
# 있던 건 명령을 쏜 순간에만 남는 사후 로그뿐 — 아무것도 안 누르는 동안 팔이
# 죽었는지 알 방법이 없었다(8/31 무증상 실패의 정확한 구멍). 여기는 관측·표시
# 전용이고, 이 값으로 재시작·복구 같은 행동을 하지 않는다.
OBS_PERIOD      = 1.0    # 샘플 주기(초)
OBS_STALE_SEC   = 5.0    # body/depth/드라이버 공통 신선도 한계 — 신호별 하드코딩 금지
OBS_BOOT_GRACE  = 10.0   # 기동 직후 유예(초) — 그전 미관측은 unknown(ok로 새면 안 됨)
OBS_NODES_EVERY = 5      # 노드이름 그래프질의 주기(틱) — detail 문자열 전용

# ── 스냅샷 TF 프레임명 — 2026-08-28 실측 확정 (오케 6e) ─────────────────────────
# 그리퍼 [-0.029,-0.341,0.604] / 몸체 [0.045,-0.003,1.322] (base_link 기준, 실측 대조).
# ★함정: 그리퍼 camera_info의 header.frame_id가 "camera_color_optical_frame"(몸체 것)으로
# 잘못 보고됨 — TF lookup에 camera_info의 frame_id를 절대 쓰지 말 것(그리퍼 버튼이 몸체
# 위치 z=1.322로 변환되는 3D 완전 오류가 남). intrinsics(K/D/w/h)는 정확하니 그건 그대로
# 쓰되, 프레임명은 반드시 이 CONFIG로만 (아래 코드도 camera_info.header.frame_id를 전혀
# 안 읽음 — SNAPSHOT_FRAMES만 사용).
SNAPSHOT_FRAMES = {
    "grip_color": "gripper_camera_color_optical_frame",
    "body_color": "camera_color_optical_frame",
    "base": "base_link",
    "odom": "odom",
    "map": "map",   # 측위 켜졌을 때만 lookup 성공 — 안 되면 null(정상, 실패 아님)
}

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
AIM_Y_A, AIM_Y_B = 9.0, -42.0   # 상하: 3점 fit + 실측 물리 보정
# 2026-07-10: 19.5 → 13.2 (실험실, "1.5cm 위 눌림" 실측) → 9.0 (실물 엘리베이터
# 홀 버튼에서 영점 트림 y-1.0cm으로 press 성공 확인 후 그 값을 기본값에 흡수).
# 현 상태 = 영점 트림 0 기준 정답 조준 — 트림은 "여기서 추가로 틀어질 때"만 사용.
# 재보정 환산: 1.0cm ≈ A ±4.2 (빗나간 방향으로 십자 이동 = 위로 눌리면 A 감소)
AIM_DIST_DEFAULT = 0.30          # 거리 미측정 시 가정값

# ── 영점 트림 (현장 자가 보정, 조준경 영점잡기) ──────────────────────────────
# press가 빗나가면 사용자가 UI 화살표로 "빗나간 방향 + cm"를 입력 → 십자가 그
# 방향으로 이동 (십자 = 실제 손끝 낙점이 되도록). 파일 영속 — 앱 재시작에도 유지.
AIM_TRIM_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "aim_trim.json")
_aim_trim = {"x_cm": 0.0, "y_cm": 0.0}   # +x=오른쪽, +y=아래 (화면 기준)
try:
    _aim_trim.update({k: float(v) for k, v in
                      json.load(open(AIM_TRIM_FILE)).items() if k in _aim_trim})
except Exception:
    pass

def _save_aim_trim():
    try:
        with open(AIM_TRIM_FILE, "w") as f:
            json.dump(_aim_trim, f)
    except Exception:
        pass

# ── 차내 버튼 배치 프로필 (버튼 맵) ──────────────────────────────────────────
# 실물 조작반의 버튼 배치(위→아래, 왼→오른쪽)를 사전지식으로 저장.
# 글자가 하나만 읽혀도(앵커) 나머지 버튼의 정체를 상대 위치로 확정하는 데 사용.
# UI에서 편집 가능 (다른 엘리베이터 재사용), 파일 영속.
# 오버레이(cv2) 전용 영문 별칭 — cv2 putText가 한글을 '?'로 깨뜨림. UI/로그는 한글 유지
_OVERLAY_ALIAS = {"문열림": "door", "종": "bell"}

LAYOUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "button_layout.json")
# 격자형(엑셀식): 빈 칸("")으로 실물의 열 위치까지 표현 — 정합 정확도에 직접 기여.
# 기본 3x3, 시연 엘리베이터 실물 배치 (위→아래): 문열림·3 / ·2·5 / 종·1·4
# ("3 바로 아래가 2" — 사용자 실물 확인. 열 정렬은 현장에서 UI로 교정 가능)
_layout_rows = [["문열림", "3", ""], ["", "2", "5"], ["종", "1", "4"]]
try:
    _lr = json.load(open(LAYOUT_FILE)).get("rows")
    if isinstance(_lr, list) and all(isinstance(r, list) and r for r in _lr):
        _layout_rows = [[str(t) for t in r] for r in _lr]
except Exception:
    pass

def _aim_offsets(dist):
    """현재 목표 거리(m)에 맞는 조준 오프셋(px) 반환 (영점 트림 포함).
    1cm = 4.2/d px (px↔m 환산 상수 420 기준) — 트림은 고정 물리 오프셋이라 A/d형."""
    d = dist if (dist and 0.10 < dist < 1.0) else AIM_DIST_DEFAULT
    return (int(round(AIM_X_A / d + AIM_X_B + _aim_trim["x_cm"] * 4.2 / d)),
            int(round(AIM_Y_A / d + AIM_Y_B + _aim_trim["y_cm"] * 4.2 / d)))


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
LIFT_PRIOR_CALL  = 0.94   # 호출 버튼(▲▼) lift — 실측(2026-07-15): 실물 홀 press 0.92~0.97
LIFT_PRIOR_PANEL = 0.94   # 차내 층 버튼 lift — 실측: 실물 차내 press 0.91('5')~0.98('3' 부근)
# (실물 엘리베이터 실측 기반. 태블릿 연습 환경은 0.43~0.50이었음 — 장소 다르면 +/-로 조정)
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
BASE_SPEED       = 0.04   # m/s — press 정렬 미세이동용 (그대로 유지)
MANUAL_SPEED     = 0.10   # m/s — 조종 패드·자동 안무 전용 (2026-07-15 사용자 요청 증속)
BOARD_SPEED      = 0.20   # m/s — 긴 직진(≥30cm, 탑승·하차 안무) 전용 증속.
                          # 0.10이면 185cm에 ~19s가 걸려 닫히는 문에 걸림
                          # (2026-07-21 사용자 요청). 최대 0.3의 2/3 — 비상정지
                          # 제동거리 여유 유지. 이동 중 26cm 감시는 그대로 연속 작동.
MANUAL_ROT_SPEED = 0.30   # rad/s — 수동/자동 회전 속도
# 라이다는 로봇 "중심" 기준이고 몸통 끝은 중심에서 ~17cm.
# 라이다는 2D 단면이라 책상 상판처럼 위에서 튀어나온 건 못 봄 — 여유를 너무 줄이지 말 것.
CLEAR_DIST       = 0.30   # m — 이동 방향 최소 여유 (이보다 가까우면 이동 거부)
CLEAR_MARGIN     = 0.15   # m — 이동량 비례 가드: 여유 ≥ 이동량 + 이 값이면 허용.
                          #     엘리베이터 차내처럼 좁은 곳(뒤 벽 26cm)에서 2cm 미세
                          #     정렬까지 30cm 기준으로 막혀 정렬 불가하던 문제 해결
                          #     (2026-07-13, 사용자 승인 — 라이다 기반 안전은 유지)
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
    "rot_body":     1,      # body 회전 — D435i가 몸체에 90° 돌아가 장착돼 있어
                            # 기본 1(CW 90°)로 시작해야 사무실 전경이 바로 보임
    "place":        "hall", # 장소 모드: hall(홀, ▲▼만) / cab(차내, 숫자만) — 팔레트·prior 분기
    "scene":        None,   # 여정 단계 (0~5, SCENES 인덱스) — 조종 패드 티칭 구간 표시
    "scene_acc":    {"fwd_cm": 0.0, "rot_deg": 0.0},  # 현재 단계 누적 이동량 (티칭 기록)
    "scene_ts":     0.0,    # 현재 단계 시작 시각 — press 완료가 이 이후여야 다음 단계 해제
    "press_ok_ts":  0.0,    # 마지막 press ✅ 완료 시각
    "door_base":    None,   # ③ 문대기 진입 시점 전방 여유 (닫힌 문 기준선)
    "door_open":    False,  # ③에서 여유 점프(+0.5m, 2연속) 감지 → ④ 탑승 해제
    "door_streak":  0,      # 문 열림 판정 연속 관측 수 (노이즈 방지)
    "arm_ext":      None,   # 현재 팔 뻗기 위치 (wrist_extension, m)
    "wall_tilt":    None,   # 벽 기울기 각도(°) — +면 오른쪽이 멂 (평행 정렬용)
    "wall_flat":    None,   # 앞면이 평면인가 (False면 기울기 측정 거부)
    "base_align":   False,  # 베이스 전후 X정렬 ON/OFF (기본 OFF — 명시적으로 켜야 움직임)
    "guard_off":    False,  # 라이다 가드 해제 (개발용 — 재시작 시 항상 가드 ON 복귀)
    "jpeg_fail":    None,   # 타겟 인식 실패 순간의 OCR 입력 스냅샷 (디버그: /fail.jpg)
    "base_travel":  0.0,    # 누적 이동량 (m)
    "press_ready":  False,  # 정조준+근접 래치 (진입 0.25/유지 0.28 히스테리시스)
    "align_note":   None,   # X정렬 상태 메시지
    "pressing":     False,  # 누르기 시퀀스 진행 중
    "press_status": None,   # 누르기 진행 메시지
    "authority":    False,   # 이동 제어권 — 대시보드(8080)가 부여/회수하는 주도권.
                            # False면 모든 이동(서보·스캔·패드·안무·press) 거부,
                            # 카메라·OCR·UI 관찰은 유지. fail-closed: 기본 False,
                            # /authority로 명시적으로 줄 때만 True(main()의
                            # --standalone 플래그로 켠 단독 모드는 예외).
    "lease_deadline": 0.0,  # 리스 만료 시각(monotonic) — 대시보드 하트비트(2s)가 갱신,
                            # 워치독이 이 시각을 넘기면(LEASE_TTL=6s 무갱신) 자체 회수
    "lease_expired":  False,  # 마지막 회수가 워치독(리스 만료)이었는지 — /status로 노출,
                              # 대시보드가 "제어권 재부여 필요" 표시에 사용
}
# RLock(재진입 허용): _dlog가 내부에서 이 락을 잡으므로, 락 보유 중 _dlog 호출이
# 일반 Lock이면 자기 자신과 교착 → 앱 전체 동결 (2026-07-15 ③ 문대기 동결 사건)
state_lock = threading.RLock()

def _authority_ok() -> bool:
    """이동 제어권 확인 — 모든 이동 진입점의 공통 관문."""
    with state_lock:
        return bool(state.get("authority", False))

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
           height: 100vh; overflow: hidden;   /* 앱 레이아웃 — 페이지 스크롤 없음 */
           display: flex; flex-direction: column; align-items: stretch;
           padding: 0; gap: 0; }
    h2   { font-size: 1.2rem; color: #aef; }
    #banner { font-size: 0.95rem; padding: 7px 14px; border-radius: 6px;
              background: #222; color: #fff; min-width: 0; text-align: center; }
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
    .tgl-name  { color:#bbb; font-size:0.82rem; white-space:nowrap; }
    .tgl-state { font-size:0.8rem; color:#777; min-width:32px; font-weight:700; }
    .tgl-track { width:46px; height:24px; border-radius:12px; background:#3a3a44;
                 position:relative; transition:background .15s; flex-shrink:0; }
    .tgl-knob  { width:18px; height:18px; border-radius:50%; background:#ddd;
                 position:absolute; top:3px; left:3px; transition:left .15s; }
    .tgl.on .tgl-track { background:#2a8f2a; }
    .tgl.amber.on .tgl-track { background:#b8651a; }
    .tgl.on .tgl-knob { left:25px; }
    .tgl.warn .tgl-track { background:#b22; }
    /* 좁은 창: 2열 그리드 → 1열 + 페이지 스크롤 허용 (로그·컨트롤 접근 보장) */
    @media (max-width: 1100px) {
      body { overflow: auto !important; height: auto !important; }
      #main-grid { grid-template-columns: 1fr !important; }
      #dlog { min-height: 30vh !important; }
    }
    /* 로그 전체화면 오버레이 (⛶ 확대 버튼 토글) */
    #dlog.expanded {
      position: fixed; left: 2vw; top: 5vh; width: 96vw; height: 90vh;
      z-index: 999; font-size: 0.9rem; box-shadow: 0 0 40px #000;
      border-color: #46a;
    }
  </style>
</head>
<body>
  <!-- 헤더: 제목+배너+누르기 — 100vh 레이아웃이라 항상 보임 -->
  <div style="display:flex;gap:10px;align-items:center;padding:6px 10px;flex-shrink:0;
              background:#101018;border-bottom:1px solid #26263a;">
    <span style="font-size:1rem;font-weight:700;color:#aef;white-space:nowrap;">🛗 Elevator</span>
    <div id="banner" style="flex:1;margin:0;">Waiting for camera...</div>
    <button id="press-btn2" onclick="doPress()" disabled
            style="background:#a22;color:#fff;border:none;border-radius:8px;
                   padding:8px 22px;cursor:pointer;font-weight:800;white-space:nowrap;">
      🔴 누르기 <span style="font-size:0.75rem;opacity:0.7;">Space</span></button>
  </div>
  <!-- 본문 그리드: 좌 = 영상+로그 / 우 = 컨트롤 전체 표시 (양쪽 다 스크롤 없음) -->
  <div id="main-grid" style="display:grid;grid-template-columns:1fr 390px;flex:1;min-height:0;gap:10px;padding:8px;">
    <div style="display:flex;flex-direction:column;min-width:0;min-height:0;gap:4px;">
  <!-- 영상: gripper 크게(조준 확인) + 우측 썸네일 3개 세로 -->
  <div style="display:flex;gap:6px;min-height:0;flex-shrink:0;">
    <figure style="margin:0;text-align:center;flex:2.6;min-width:0;">
      <img id="stream" src="/video?cam=gripper" alt="gripper stream"
           style="width:100%;max-height:46vh;object-fit:contain;background:#000;">
      <figcaption style="color:#8ad;font-size:0.75rem;">
        🤏 gripper <span id="tilt-info" style="margin-left:6px;font-weight:700;"></span>
        <button onclick="rotateCam('gripper')"
                style="margin-left:6px;background:#334;color:#cdf;border:none;border-radius:5px;padding:1px 8px;cursor:pointer;font-size:0.72rem;">↻</button>
      </figcaption>
    </figure>
    <div style="flex:1;display:flex;flex-direction:column;gap:4px;min-width:0;">
      <img id="stream-depth" src="/video?cam=depth" alt="depth" title="depth"
           style="width:100%;background:#000;">
      <img id="stream-ocr" src="/video?cam=ocr" alt="ocr" title="OCR 입력"
           style="width:100%;background:#000;">
      <div style="position:relative;">
        <img id="stream-body" src="/video?cam=body" alt="body" title="body 전방"
             style="width:100%;background:#000;display:block;">
        <button onclick="rotateCam('body')" title="body 회전"
                style="position:absolute;bottom:3px;right:3px;background:#334a;color:#cdf;border:none;border-radius:4px;padding:1px 7px;cursor:pointer;font-size:0.7rem;">↻</button>
      </div>
    </div>
  </div>
  <!-- 판단 로그: 좌컬럼의 남는 높이를 전부 사용 (내용만 내부 스크롤) -->
  <div style="flex:1;min-height:50px;display:flex;flex-direction:column;">
    <div style="display:flex;justify-content:space-between;align-items:center;">
      <span style="color:#89a;font-size:0.78rem;font-weight:700;">🧠 판단 로그</span>
      <div style="display:flex;gap:10px;align-items:center;">
        <button id="log-zoom-btn" onclick="toggleLogZoom()" title="로그를 전체화면으로 확대/복귀"
                style="background:#234;color:#9fc;border:none;border-radius:5px;
                       padding:2px 10px;cursor:pointer;font-size:0.72rem;">⛶ 확대</button>
        <a href="/decisions" target="_blank" style="color:#68c;font-size:0.72rem;">전체 열기</a>
      </div>
    </div>
    <pre id="dlog" style="background:#0d1117;color:#9fb;border:1px solid #234;
         border-radius:8px;padding:6px 10px;flex:1;min-height:0;overflow-y:auto;
         font-size:0.74rem;line-height:1.45;margin:2px 0 0;white-space:pre-wrap;"></pre>
  </div>
  </div><!-- 좌컬럼 끝 -->
  <div style="display:flex;flex-direction:column;gap:7px;min-height:0;">  <!-- 우컬럼: 컨트롤 전체 표시 -->

  <!-- 고정 타겟 팔레트: 인식 여부와 무관하게 항상 선택 가능 -->
  <div id="buttons" style="display:flex;gap:10px;"></div>
  <!-- 배치 편집기: 엑셀형 격자 — 마우스 전용이라 접이식 (평소 숨김) -->
  <details style="border:1px solid #26263a;border-radius:8px;padding:3px 8px;">
    <summary style="cursor:pointer;color:#89a;font-size:0.78rem;">🗺 배치 편집기 (펼치기)</summary>
  <div style="display:flex;gap:12px;align-items:flex-start;background:#14141b;border-radius:10px;padding:8px 6px;">
    <span style="color:#567;font-size:0.8rem;padding-top:8px;white-space:nowrap;">🗺 배치</span>
    <div>
      <div id="layout-grid" style="display:flex;flex-direction:column;gap:4px;"></div>
      <div style="display:flex;gap:6px;margin-top:8px;align-items:center;">
        <button onclick="gridResize(1,0)"  style="background:#2a2a3a;color:#9ab;border:none;border-radius:5px;padding:4px 10px;cursor:pointer;font-size:0.78rem;">행+</button>
        <button onclick="gridResize(-1,0)" style="background:#2a2a3a;color:#9ab;border:none;border-radius:5px;padding:4px 10px;cursor:pointer;font-size:0.78rem;">행−</button>
        <button onclick="gridResize(0,1)"  style="background:#2a2a3a;color:#9ab;border:none;border-radius:5px;padding:4px 10px;cursor:pointer;font-size:0.78rem;">열+</button>
        <button onclick="gridResize(0,-1)" style="background:#2a2a3a;color:#9ab;border:none;border-radius:5px;padding:4px 10px;cursor:pointer;font-size:0.78rem;">열−</button>
        <button onclick="saveLayout()" style="background:#265;color:#cfd;border:none;border-radius:6px;padding:5px 16px;cursor:pointer;font-weight:700;">저장</button>
        <span style="color:#678;font-size:0.72rem;">실물 그대로 (위→아래) · 빈 칸은 비워두기</span>
      </div>
    </div>
  </div>
  </details>
  <script>
    // 팔레트 = 홀(▲▼) + 차내(실물 배치 격자, /layout 서버 저장 — UI에서 편집)
    // 배치가 곧 "버튼 맵" 사전지식: 앵커 한 글자만 읽혀도 나머지 버튼 위치 확정에 쓰임
    let TARGETS = [];             // [표시라벨, 토큰] — poll()이 상태 갱신에 사용
    let currentTarget = null;     // poll()에서 서버 상태로 갱신
    const KEY_HINT = {'1':'1','2':'2','3':'3','4':'4','5':'5','^':'6','s':'7'};
    function buildPalette(rows) {
      TARGETS = [['▲','^'],['▼','s']];
      rows.forEach(r => r.forEach(tok => { if (tok && tok.trim()) TARGETS.push([tok, tok]); }));
      const box = document.getElementById('buttons');
      box.innerHTML = '';
      let idx = 0;
      const mk = (label, tok) => {
        const b = document.createElement('button');
        b.className = 'btn'; b.id = 'tgt-' + (idx++);
        b.dataset.tok = tok;
        const key = KEY_HINT[tok];
        b.innerHTML = label + (key ?
          `<div style="font-size:0.6rem;color:#79a;font-weight:400;line-height:1;">${key}</div>` : '');
        b.style.minWidth = '52px';
        b.style.fontSize = label.length > 1 ? '0.85rem' : '1.2rem';
        b.onclick = () => { currentTarget === tok ? resetTarget() : selectButton(tok); };
        return b;
      };
      const hall = document.createElement('div');
      // 실물 호출부처럼 세로 배치: ▲ 위 / ▼ 아래
      hall.style.cssText = 'display:flex;flex-direction:column;gap:6px;align-items:flex-start;';
      TARGETS.slice(0, 2).forEach(([l, t]) => hall.appendChild(mk(l, t)));
      const sep = document.createElement('div');
      sep.style.cssText = 'width:1px;background:#334;align-self:stretch;margin:0 6px;';
      const cab = document.createElement('div');
      cab.style.cssText = 'display:flex;flex-direction:column;gap:6px;';
      rows.forEach(r => {
        const rd = document.createElement('div');
        rd.style.cssText = 'display:flex;gap:6px;';
        r.forEach(tok => {
          if (tok && tok.trim()) rd.appendChild(mk(tok, tok));
          else {   // 빈 칸: 자리를 유지해서 팔레트가 실물 모양 그대로 보이게
            const sp = document.createElement('div');
            sp.style.cssText = 'min-width:52px;';
            rd.appendChild(sp);
          }
        });
        cab.appendChild(rd);
      });
      box.appendChild(hall); box.appendChild(sep); box.appendChild(cab);
    }
    // ── 배치 편집기 (엑셀형 격자) ──
    let gridRows = 3, gridCols = 3, gridVals = [];
    function renderGrid() {
      const g = document.getElementById('layout-grid');
      g.innerHTML = '';
      for (let r = 0; r < gridRows; r++) {
        const rd = document.createElement('div');
        rd.style.cssText = 'display:flex;gap:4px;';
        for (let c = 0; c < gridCols; c++) {
          const inp = document.createElement('input');
          inp.id = `lg-${r}-${c}`;
          inp.value = (gridVals[r] && gridVals[r][c]) || '';
          inp.placeholder = '·';
          inp.maxLength = 4;
          inp.style.cssText = 'width:58px;height:36px;text-align:center;' +
            'background:#181820;color:#cde;border:1px solid #345;border-radius:7px;font-size:0.9rem;';
          rd.appendChild(inp);
        }
        g.appendChild(rd);
      }
    }
    function syncGridVals() {
      for (let r = 0; r < gridRows; r++) {
        gridVals[r] = gridVals[r] || [];
        for (let c = 0; c < gridCols; c++) {
          const el = document.getElementById(`lg-${r}-${c}`);
          if (el) gridVals[r][c] = el.value.trim();
        }
      }
    }
    function gridResize(dr, dc) {
      syncGridVals();
      gridRows = Math.max(1, Math.min(6, gridRows + dr));
      gridCols = Math.max(1, Math.min(6, gridCols + dc));
      renderGrid();
    }
    function saveLayout() {
      syncGridVals();
      const rows = [];
      for (let r = 0; r < gridRows; r++) {
        const row = [];
        for (let c = 0; c < gridCols; c++) row.push((gridVals[r] && gridVals[r][c]) || '');
        rows.push(row);
      }
      fetch('/layout', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({rows})})
        .then(r => r.json())
        .then(d => { if (d.ok) buildPalette(d.rows); else flashMotionErr(d.error); })
        .catch(() => flashMotionErr('앱 응답 없음'));
    }
    function loadLayout() {
      fetch('/layout').then(r => r.json()).then(d => {
        const rows = d.rows;
        gridRows = Math.max(3, rows.length);
        gridCols = Math.max(3, ...rows.map(r => r.length));
        gridVals = rows.map(r => r.slice());
        renderGrid();
        buildPalette(rows);
      });
    }
    loadLayout();
  </script>
  <!-- 동작 행: 컴팩트 — 글자 줄바꿈 금지, 버튼 단위로만 감김 -->
  <div style="display:flex;gap:6px;align-items:center;flex-wrap:wrap;">
    <button id="press-btn" onclick="doPress()" disabled
            style="background:#a22;color:#fff;border:none;border-radius:8px;white-space:nowrap;
                   padding:8px 16px;cursor:pointer;font-size:0.95rem;font-weight:800;">
      🔴 누르기 <span style="font-size:0.72rem;opacity:0.7;">Space</span></button>
    <button onclick="setGripper(true)"
            style="background:#264;color:#cfc;border:none;border-radius:6px;padding:7px 10px;cursor:pointer;white-space:nowrap;font-size:0.85rem;">✋ 열기 <span style="font-size:0.7rem;opacity:0.7;">O</span></button>
    <button onclick="setGripper(false)"
            style="background:#642;color:#fcc;border:none;border-radius:6px;padding:7px 10px;cursor:pointer;white-space:nowrap;font-size:0.85rem;">✊ 닫기 <span style="font-size:0.7rem;opacity:0.7;">C</span></button>
    <button id="reset" onclick="resetTarget()"
            style="background:#445;color:#dde;border:none;border-radius:6px;padding:7px 10px;cursor:pointer;white-space:nowrap;font-size:0.85rem;">↺ 재선택 <span style="font-size:0.7rem;opacity:0.7;">Esc</span></button>
  </div>
  <!-- 모드 행: 토글류 (컴팩트) -->
  <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;">
    <button id="place-btn" onclick="togglePlace()"
            style="background:#246;color:#ade;border:none;border-radius:6px;padding:6px 10px;cursor:pointer;font-weight:700;white-space:nowrap;font-size:0.85rem;">🏢 홀 (호출 ▲▼)</button>
    <div class="tgl amber" id="align-tgl" onclick="toggleBaseAlign()"
         title="바퀴 이동 허용 (전후진 정렬·회전) — OFF면 바퀴 절대 안 움직임">
      <span class="tgl-name">🚗 몸체이동 <span style="font-size:0.7rem;opacity:0.7;">⇧1</span></span>
      <div class="tgl-track"><div class="tgl-knob"></div></div>
      <span class="tgl-state" id="align-state">OFF</span>
    </div>
    <div class="tgl" id="guard-tgl" onclick="toggleGuard()"
         title="라이다 장애물 가드 — 끄면 개발용 위험 모드 (재시작 시 자동 복귀)">
      <span class="tgl-name">🛡 가드 <span style="font-size:0.7rem;opacity:0.7;">⇧2</span></span>
      <div class="tgl-track"><div class="tgl-knob"></div></div>
      <span class="tgl-state" id="guard-state">ON</span>
    </div>
    <button id="pose-btn" onclick="setWristForward()" title="홈 포즈: 손목 전방 + 팔 완전 수납"
            style="background:#334;color:#cdf;border:none;border-radius:6px;padding:6px 10px;cursor:pointer;white-space:nowrap;font-size:0.85rem;">🏠 홈 <span style="font-size:0.7rem;opacity:0.7;">H</span></button>
  </div>
  <!-- 여정 단계 탭: 클릭=단계 전환(누적 매듭+리셋), ①⑤는 홀/차내 모드 자동 -->
  <div style="display:flex;gap:4px;flex-wrap:wrap;align-items:center;">
    <span style="color:#567;font-size:0.8rem;">🧭 여정</span>
    <button id="scn-0" onclick="setScene(0)" style="background:#2a2a3e;color:#89a;border:none;border-radius:5px;padding:4px 7px;cursor:pointer;font-size:0.75rem;white-space:nowrap;">① 호출</button>
    <button id="scn-1" onclick="setScene(1)" style="background:#2a2a3e;color:#89a;border:none;border-radius:5px;padding:4px 7px;cursor:pointer;font-size:0.75rem;white-space:nowrap;">② 문앞</button>
    <button id="scn-2" onclick="setScene(2)" style="background:#2a2a3e;color:#89a;border:none;border-radius:5px;padding:4px 7px;cursor:pointer;font-size:0.75rem;white-space:nowrap;">③ 문대기</button>
    <button id="scn-3" onclick="setScene(3)" style="background:#2a2a3e;color:#89a;border:none;border-radius:5px;padding:4px 7px;cursor:pointer;font-size:0.75rem;white-space:nowrap;">④ 탑승</button>
    <button id="scn-4" onclick="setScene(4)" style="background:#2a2a3e;color:#89a;border:none;border-radius:5px;padding:4px 7px;cursor:pointer;font-size:0.75rem;white-space:nowrap;">⑤ 층</button>
    <button id="scn-5" onclick="setScene(5)" style="background:#2a2a3e;color:#89a;border:none;border-radius:5px;padding:4px 7px;cursor:pointer;font-size:0.75rem;white-space:nowrap;">⑥ 하차</button>
    <button id="next-btn" onclick="nextScene()"
            title="다음 단계로 진행 — 조건 미충족이면 경고만 남기고 진행"
            style="background:#2a2a3e;color:#9ab;border:none;border-radius:6px;
                   padding:4px 14px;cursor:pointer;font-size:0.8rem;font-weight:800;
                   white-space:nowrap;transition:background .2s, box-shadow .2s;">
      ▶ 다음 <span style="font-size:0.68rem;opacity:0.7;">N</span></button>
    <span id="scn-hint" style="font-size:0.74rem;color:#9ab;white-space:nowrap;"></span>
  </div>
  <!-- 조종 패드: 클릭 1번 = 스텝 1번 (⇧1 무관, 낮춘 가드 — 이동 중 26cm 비상정지) -->
  <div id="pad-row" style="display:flex;gap:5px;align-items:center;flex-wrap:wrap;border-radius:6px;padding:2px;">
    <span style="color:#567;font-size:0.8rem;">🕹</span>
    <button onclick="stepRot(1)"   style="background:#334;color:#cdf;border:none;border-radius:5px;padding:5px 8px;cursor:pointer;font-size:0.8rem;white-space:nowrap;">↺ 좌 <span style="font-size:0.68rem;opacity:0.7;">A</span></button>
    <button onclick="stepMove(1)"  style="background:#264;color:#cfc;border:none;border-radius:5px;padding:5px 8px;cursor:pointer;font-size:0.8rem;white-space:nowrap;">▲ 전진 <span style="font-size:0.68rem;opacity:0.7;">W</span></button>
    <button onclick="stepMove(-1)" style="background:#442;color:#ffc;border:none;border-radius:5px;padding:5px 8px;cursor:pointer;font-size:0.8rem;white-space:nowrap;">▼ 후진 <span style="font-size:0.68rem;opacity:0.7;">S</span></button>
    <button onclick="stepRot(-1)"  style="background:#334;color:#cdf;border:none;border-radius:5px;padding:5px 8px;cursor:pointer;font-size:0.8rem;white-space:nowrap;">↻ 우 <span style="font-size:0.68rem;opacity:0.7;">D</span></button>
    <select id="step-cm" title="전·후진 스텝 크기" onchange="this.blur()" style="background:#223;color:#cde;border:1px solid #345;border-radius:4px;padding:4px;">
      <option value="1">1cm</option><option value="5" selected>5cm</option>
      <option value="10">10cm</option><option value="30">30cm</option>
    </select>
    <select id="step-deg" title="회전 스텝 크기" onchange="this.blur()" style="background:#223;color:#cde;border:1px solid #345;border-radius:4px;padding:4px;">
      <option value="5">5°</option><option value="10" selected>10°</option>
      <option value="45">45°</option><option value="90">90°</option>
    </select>
    <span id="clr-info" style="font-size:0.76rem;color:#8ac;white-space:nowrap;"></span>
    <span id="acc-info" style="font-size:0.76rem;color:#a9c;white-space:nowrap;"></span>
  </div>
  <!-- 영점 조절: press가 빗나간 "방향"을 입력 → 십자가 그쪽으로 이동 (파일 영속) -->
  <div style="display:flex;gap:8px;align-items:center;">
    <span style="color:#567;font-size:0.8rem;">🎯 영점</span>
    <span style="color:#89a;font-size:0.75rem;white-space:nowrap;" title="실제 눌린 지점이 버튼 기준 어느 쪽이었는지 (⇧+방향키)">빗나간 방향:</span>
    <button onclick="aimTrim(0,-1)" style="background:#345;color:#dee;border:none;border-radius:5px;padding:5px 11px;cursor:pointer;">↑</button>
    <button onclick="aimTrim(0,1)"  style="background:#345;color:#dee;border:none;border-radius:5px;padding:5px 11px;cursor:pointer;">↓</button>
    <button onclick="aimTrim(-1,0)" style="background:#345;color:#dee;border:none;border-radius:5px;padding:5px 11px;cursor:pointer;">←</button>
    <button onclick="aimTrim(1,0)"  style="background:#345;color:#dee;border:none;border-radius:5px;padding:5px 11px;cursor:pointer;">→</button>
    <select id="trim-step" onchange="this.blur()" style="background:#223;color:#cde;border:1px solid #345;border-radius:4px;padding:4px;">
      <option value="0.5" selected>0.5cm</option>
      <option value="1.0">1.0cm</option>
    </select>
    <span id="trim-now" style="color:#8ac;font-size:0.78rem;min-width:130px;"></span>
    <button onclick="aimTrim(0,0,true)" style="background:#434;color:#daa;border:none;border-radius:5px;padding:4px 9px;cursor:pointer;font-size:0.75rem;">리셋</button>
  </div>
  <div id="align-note" style="color:#fc4;font-size:0.9rem;min-height:1.2em;"></div>

  <script>
    function toggleLogZoom() {
      const el = document.getElementById('dlog');
      const on = el.classList.toggle('expanded');
      document.getElementById('log-zoom-btn').textContent = on ? '✕ 닫기' : '⛶ 확대';
      if (on) el.scrollTop = el.scrollHeight;   // 확대 직후 최신 로그로
    }
    // 확대 상태에서 로그 바깥(오버레이 밖) 클릭 시 자동 복귀
    document.addEventListener('click', e => {
      const el = document.getElementById('dlog');
      if (el && el.classList.contains('expanded')
          && !el.contains(e.target)
          && e.target.id !== 'log-zoom-btn') toggleLogZoom();
    });
    setInterval(() => {
      fetch('/decisions').then(r => r.text()).then(t => {
        const el = document.getElementById('dlog');
        const atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 40;
        el.textContent = t;
        if (atBottom) el.scrollTop = el.scrollHeight;   // 보고 있으면 자동 따라가기
      }).catch(()=>{});
    }, 1000);
  </script>

  <details style="border:1px solid #26263a;border-radius:8px;padding:3px 8px;">
    <summary style="cursor:pointer;color:#89a;font-size:0.78rem;">🎚 손목·리프트 슬라이더 (lift: +/− 키)</summary>
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
    lift (높이) <span style="font-size:0.72rem;opacity:0.7;">+/−</span>:
    <input type="range" id="lift-slider" min="0.15" max="1.10" step="0.01" value="0.60"
           oninput="document.getElementById('lift-val').value=parseFloat(this.value).toFixed(2)">
    <input type="number" id="lift-val" value="0.60" step="0.01" min="0.15" max="1.10"
           oninput="document.getElementById('lift-slider').value=this.value">
    <button id="pitch-apply" onclick="applyLift()">적용</button>
  </div>
  <div id="info" style="font-size:0.75rem;color:#789;">감지된 버튼을 클릭하면 추적을 시작합니다.</div>
  </details>
  </div><!-- 우컬럼 끝 -->
  </div><!-- 본문 그리드 끝 -->

  <script>
    // 파이썬 _label_color와 동일한 규칙 (BGR→RGB 변환된 동일 팔레트)
    const LABEL_COLORS = ['rgb(80,210,0)','rgb(0,160,255)','rgb(255,180,0)',
                          'rgb(255,80,200)','rgb(220,220,80)','rgb(0,220,255)','rgb(180,105,255)'];
    function labelColor(t) {
      if (!t) return LABEL_COLORS[0];
      let s = 0; for (const c of t) s += c.charCodeAt(0);
      return LABEL_COLORS[s % LABEL_COLORS.length];
    }
    // 모션 명령 실패를 짧게 알리는 비침습 토스트 (슬라이더는 자주 쏘므로 alert 대신)
    // ★불변식: 이 페이지에 블로킹 모달(alert/confirm)을 두지 않는다. 모달은 JS를
    // 멈춰서 그 동안 Esc(=/step_stop, keydown 핸들러)가 안 먹는다 — 팔이 패널 앞에서
    // 움직이는 중에 뜨면 중단키를 뺏긴다. 실패 통지는 전부 이 토스트로.
    let _motionErrTimer = null;
    // ms: 표시 시간(기본 2.5초). 토스트는 엘리먼트·타이머를 하나만 쓰므로
    // 뒤에 온 것이 앞의 것을 덮는다 — 단발·고중요도 통지는 더 길게 준다.
    function flashMotionErr(msg, ms) {
      let el = document.getElementById('motion-err-toast');
      if (!el) {
        el = document.createElement('div');
        el.id = 'motion-err-toast';
        el.style.cssText = 'position:fixed;top:12px;left:50%;transform:translateX(-50%);'
          + 'background:#c0392b;color:#fff;padding:8px 16px;border-radius:6px;'
          + 'font-weight:bold;font-size:15px;z-index:9999;box-shadow:0 2px 8px rgba(0,0,0,.4);'
          + 'max-width:80%;text-align:center;';
        document.body.appendChild(el);
      }
      el.textContent = '⚠ ' + msg;
      el.style.display = 'block';
      if (_motionErrTimer) clearTimeout(_motionErrTimer);
      _motionErrTimer = setTimeout(() => { el.style.display = 'none'; }, ms || 2500);
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
      // 라벨/팔레트는 poll()이 서버 상태로 동기화. 다만 lift 선이동은 poll에
      // 안 나오므로(모드만 바뀌고 팔은 그대로) 실패를 여기서 직접 알린다.
      fetch('/place', {method:'POST'})
        .then(r => r.json())
        .then(d => { if (d.lift_ok === false) flashMotionErr('규정 높이 이동 실패'); })
        .catch(() => flashMotionErr('앱 응답 없음'));
    }
    function setGripper(open) {
      fetch('/gripper', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({open})})
        .then(r=>r.json()).then(d=>{ if(!d.ok) flashMotionErr(d.error||'모션 명령 실패'); })
        .catch(()=>flashMotionErr('앱 응답 없음'));
    }
    function doPress() {
      fetch('/press', {method:'POST'}).then(r => r.json()).then(d => {
        // 누르기 거부는 단발이고 놓치면 안 되는 통지 — 슬라이더 토스트에
        // 곧바로 덮이지 않게 문구를 구분하고 더 오래 띄운다.
        if (!d.ok) flashMotionErr('누르기 거부 — ' + d.error, 6000);
      })
      .catch(() => flashMotionErr('앱 응답 없음'));
    }
    function setWristForward() {
      const pitch = -0.02, yaw = 0.03;   // WRIST_PITCH/YAW_DEFAULT와 일치
      document.getElementById('pitch-slider').value = pitch;
      document.getElementById('pitch-val').value = pitch.toFixed(2);
      document.getElementById('yaw-slider').value = yaw;
      document.getElementById('yaw-val').value = yaw.toFixed(2);
      // 손목 각도 + 팔 완전 수납까지 한 번에 (홈 포즈)
      fetch('/wrist_forward', {method:'POST'})
        .then(r=>r.json()).then(d=>{ if(!d.ok) flashMotionErr(d.error||'모션 명령 실패'); })
        .catch(()=>flashMotionErr('앱 응답 없음'));
    }
    function aimTrim(sx, sy, reset=false) {
      const st = parseFloat(document.getElementById('trim-step').value);
      fetch('/aim_trim', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify(reset ? {reset:true} : {dx_cm: sx*st, dy_cm: sy*st})})
        .then(r=>r.json()).then(d=>{ showTrim(d.x_cm, d.y_cm); });
    }
    function showTrim(x, y) {
      document.getElementById('trim-now').textContent =
        (x===0 && y===0) ? '누적 없음'
        : `누적 x${x>=0?'+':''}${x} y${y>=0?'+':''}${y}cm`;
    }
    // ── 키보드 단축키 (e.code 기반: 한/영 전환 상태와 무관) ──
    // 1~5=층, 6=▲, 7=▼, Space=누르기, Esc=재선택+스텝정지, O/C=그리퍼, H=홈, M=모드,
    // W/S/A/D=조종 패드 스텝, +/-=lift 미세조정, ⇧1=몸체이동, ⇧2=가드, ⇧방향키=영점 트림
    // 영점 리셋·가드류는 실수 비용이 커서 ⇧ 조합 또는 마우스 전용
    const KEY_TOK = {Digit1:'1',Digit2:'2',Digit3:'3',Digit4:'4',Digit5:'5',
                     Numpad1:'1',Numpad2:'2',Numpad3:'3',Numpad4:'4',Numpad5:'5',
                     Digit6:'^',Digit7:'s',Numpad6:'^',Numpad7:'s'};
    function liftNudge(d) {
      const sl = document.getElementById('lift-slider');
      const v = Math.max(0.15, Math.min(1.10, parseFloat(sl.value) + d));
      sl.value = v.toFixed(2);
      document.getElementById('lift-val').value = v.toFixed(2);
      applyLift();
    }
    function setScene(n) {
      fetch('/scene', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({n})});
    }
    let curScene = null;   // poll()이 서버 상태로 갱신 — "▶ 다음" 버튼용
    function nextScene() {
      // 여정 시작 전이면 ①부터, 아니면 다음 단계로. 조건 미충족이어도 진행
      // (서버가 경고 로그만 남김 — "금지 대신 경고" 원칙)
      const n = curScene == null ? 0 : Math.min(5, curScene + 1);
      setScene(n);
    }
    function stepMove(sign) {
      const cm = parseFloat(document.getElementById('step-cm').value);
      fetch('/step_move', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({cm: sign * cm})});
    }
    function stepRot(sign) {
      const dg = parseFloat(document.getElementById('step-deg').value);
      fetch('/step_move', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({deg: sign * dg})});
    }
    document.addEventListener('keydown', e => {
      const t = e.target.tagName;
      if (t === 'INPUT' || t === 'SELECT' || t === 'TEXTAREA') return; // 입력창 보호
      if (e.shiftKey) {
        if (e.code === 'Digit1') { toggleBaseAlign(); e.preventDefault(); return; }
        if (e.code === 'Digit2') { toggleGuard();     e.preventDefault(); return; }
        const TR = {ArrowUp:[0,-1], ArrowDown:[0,1], ArrowLeft:[-1,0], ArrowRight:[1,0]};
        if (TR[e.code]) { aimTrim(...TR[e.code]); e.preventDefault(); }
        return;
      }
      if (e.code === 'Space')  { doPress();          e.preventDefault(); return; }
      if (e.code === 'Escape') { fetch('/step_stop', {method:'POST'}); resetTarget(); return; }
      if (e.code === 'KeyO')   { setGripper(true);   return; }
      if (e.code === 'KeyC')   { setGripper(false);  return; }
      if (e.code === 'KeyH')   { setWristForward();  return; }
      if (e.code === 'KeyM')   { togglePlace();      return; }
      if (e.code === 'KeyN')   { nextScene();        return; }
      if (e.code === 'Equal' || e.code === 'NumpadAdd')      { liftNudge(+0.01); return; }
      if (e.code === 'Minus' || e.code === 'NumpadSubtract') { liftNudge(-0.01); return; }
      // 조종 패드 스텝: W/S=전/후진, A/D=좌/우회전 (스텝 크기는 셀렉트 박스)
      if (e.code === 'KeyW') { stepMove(1);  return; }
      if (e.code === 'KeyS') { stepMove(-1); return; }
      if (e.code === 'KeyA') { stepRot(1);   return; }
      if (e.code === 'KeyD') { stepRot(-1);  return; }
      const tok = KEY_TOK[e.code];
      if (tok) { currentTarget === tok ? resetTarget() : selectButton(tok); }
    });
    function applyPitch() {
      const v = parseFloat(document.getElementById('pitch-val').value);
      fetch('/wrist_pitch', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({pitch: v})})
        .then(r=>r.json()).then(d=>{ if(!d.ok) flashMotionErr(d.error||'모션 명령 실패'); })
        .catch(()=>flashMotionErr('앱 응답 없음'));
    }
    function applyYaw() {
      const v = parseFloat(document.getElementById('yaw-val').value);
      fetch('/wrist_yaw', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({yaw: v})})
        .then(r=>r.json()).then(d=>{ if(!d.ok) flashMotionErr(d.error||'모션 명령 실패'); })
        .catch(()=>flashMotionErr('앱 응답 없음'));
    }
    let liftSynced = false;
    function applyLift() {
      const v = parseFloat(document.getElementById('lift-val').value);
      fetch('/lift', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({lift: v})})
        .then(r=>r.json()).then(d=>{ if(!d.ok) flashMotionErr(d.error||'모션 명령 실패'); })
        .catch(()=>flashMotionErr('앱 응답 없음'));
    }
    function poll() {
      fetch('/status').then(r => r.json()).then(s => {
        const banner = document.getElementById('banner');
        banner.style.background = '';   // 제어권 배너의 배경 오버라이드 초기화
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
        ['press-btn', 'press-btn2'].forEach(pid => {   // 본문 + 상단 고정바 동기
          const pbtn = document.getElementById(pid);
          if (pbtn) {
            pbtn.disabled = s.pressing || !s.ready;   // 서버 READY 래치와 정확히 동기
            pbtn.style.opacity = pbtn.disabled ? 0.4 : 1.0;
          }
        });
        if (s.authority === false) {
          banner.textContent = '⛔ 제어권 없음 — 대시보드(8080)에서 엘리베이터 제어권을 켜세요 (관찰만 가능)';
          banner.className = '';
          banner.style.background = '#5a1a1a';
        } else if (s.pressing) {
          banner.textContent = `🤖 ${s.press_status || '누르기 진행 중...'}`;
          banner.className = 'tracking';
        } else if (s.centered && s.ex != null &&
                   (Math.abs(s.ex) > s.dz || Math.abs(s.ey) > s.dz)) {
          banner.textContent = `🟡 정렬 유지 중 — 경계 (x:${s.ex} y:${s.ey}, 허용±${s.dz}px)`;
          banner.className = 'tracking';
        } else if (s.ready) {
          const done = (s.press_status && s.press_status.startsWith('✅')) ? '  |  ' + s.press_status : '';
          banner.textContent = `✅ 준비 완료 — 로봇 정지, '${s.target}' 지금 누르세요  |  거리 ${s.dist}m`
            + (s.lock_shape ? '  · ⚠모양 추론(글자 미확인)' : '') + done;
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

        if (s.trim_x != null) showTrim(s.trim_x, s.trim_y);   // 영점 누적 표시 동기화
        currentTarget = s.target;   // 토글 판단용 서버 상태 동기화
        // 조종 패드: 전·후방 여유 (초록>50cm / 노랑>30cm / 빨강) + 단계 누적
        const clrCol = v => v == null ? '#888' : (v > 0.5 ? '#4e8' : (v > 0.3 ? '#fc4' : '#f66'));
        const ci = document.getElementById('clr-info');
        if (ci) ci.innerHTML =
          `앞 <b style="color:${clrCol(s.clear_f)}">${s.clear_f == null ? '?' : s.clear_f.toFixed(2)}m</b>` +
          ` 뒤 <b style="color:${clrCol(s.clear_b)}">${s.clear_b == null ? '?' : s.clear_b.toFixed(2)}m</b>`;
        const ai = document.getElementById('acc-info');
        if (ai && s.scene_acc) ai.textContent =
          `누적 ${s.scene_acc.fwd_cm >= 0 ? '+' : ''}${s.scene_acc.fwd_cm.toFixed(1)}cm / ` +
          `${s.scene_acc.rot_deg >= 0 ? '+' : ''}${Math.round(s.scene_acc.rot_deg)}°`;
        // 여정 단계: 현재=초록, 조건 충족된 다음 단계=파랑 테두리.
        // 잠금은 안 함 (연습 환경 배려) — 조건 없이 건너뛰면 서버가 ⚠ 로그만 남김
        const cur = s.scene;
        for (let i = 0; i < 6; i++) {
          const b = document.getElementById('scn-' + i);
          if (!b) continue;
          const on = (cur === i);
          b.style.background = on ? '#265' : '#2a2a3e';
          b.style.color      = on ? '#cfc' : '#89a';
          b.style.outline    = on ? '1px solid #4e8'
                             : (cur != null && i === cur + 1 && s.scene_next_ok
                                ? '1px solid #46a' : 'none');
        }
        // "▶ 다음" 버튼: 조건 충족 = 초록 발광 (지금 눌러도 됨), 평소 = 회색
        curScene = cur;
        const nb = document.getElementById('next-btn');
        if (nb) {
          nb.disabled = (cur === 5);
          nb.style.opacity = nb.disabled ? 0.35 : 1.0;
          if (cur != null && cur < 5 && s.scene_next_ok) {
            nb.style.background = '#2a8f2a'; nb.style.color = '#fff';
            nb.style.boxShadow  = '0 0 12px #2a8f2a';
          } else {
            nb.style.background = '#2a2a3e'; nb.style.color = '#9ab';
            nb.style.boxShadow  = 'none';
          }
        }
        // 단계별 안내 + 이동 단계에서 조종 패드 강조
        const HINTS = ['▼/▲ 선택 → Space', '🤖 자동: 전진 56cm→우회전 90° · Esc=중단',
                       '🚪 문 열림 대기 중…', '🤖 자동: 전진 185cm · Esc=중단',
                       '층 선택 → Space', '🤖 자동: 후진 186cm · Esc=중단'];
        const sh = document.getElementById('scn-hint');
        if (sh) {
          if (cur == null) sh.textContent = '';
          else if (cur === 2 && s.door_open) { sh.textContent = '🚪 문 열림! → ④ 탑승'; sh.style.color = '#4e8'; }
          else if (cur === 2) {
            sh.textContent = `🚪 닫힘 — 앞 ${s.clear_f == null ? '?' : s.clear_f.toFixed(2)}m` +
                             ` (기준 ${s.door_base == null ? '?' : s.door_base.toFixed(2)}m, +0.5 점프=열림)`;
            sh.style.color = '#fc4';
          }
          else if ((cur === 0 || cur === 4) && s.scene_next_ok) { sh.textContent = '✅ press 완료 → 다음 단계'; sh.style.color = '#4e8'; }
          else { sh.textContent = HINTS[cur]; sh.style.color = '#9ab'; }
        }
        const pr = document.getElementById('pad-row');
        if (pr) pr.style.outline =
          (cur != null && cur !== 0 && cur !== 4) ? '1px solid #46a' : 'none';
        // 장소 모드: 버튼 라벨 동기화 + 팔레트 활성 범위 (홀=▲▼만 / 차내=숫자만)
        const isHall = s.place !== 'cab';
        const plb = document.getElementById('place-btn');
        if (plb) {
          plb.textContent = (isHall ? '🏢 홀 (호출 ▲▼)' : '🛗 차내 (층 숫자)') + '  M';
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

@app.route("/ping")
def ping():
    return "ok", 200

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

def _obs_age(node):
    """고립 관측 캐시를 잰 지 몇 초 됐는지. Flask 스레드(=executor와 독립)에서
    계산해 '초 단위 나이'로만 내보낸다 — monotonic 값 자체를 보내면 받는 쪽의
    시계와 원점이 달라 비교가 불가능하다(프로세스가 다르면 기준이 다르다).
    None = 아직 한 번도 안 쟀음(받는 쪽에서 '모름'으로 다뤄야 하고, 정상으로
    읽으면 안 된다). 스핀이 굶어 타이머가 멈추면 이 값이 계속 커진다."""
    if node is None:
        return None
    last = getattr(node, "_obs_mono", None)
    if last is None:
        return None
    return round(time.monotonic() - last, 1)

@app.route("/status")
def status():
    with state_lock:
        s = state.copy()
    # 전·후방 라이다 여유 (조종 패드 실시간 표시용 — 문 열림도 이 값의 점프로 보임)
    _n = _node_ref[0]
    clear_f = clear_b = None
    if _n is not None:
        try:
            _cf = _n._clearance(1.0); _cb = _n._clearance(-1.0)
            clear_f = round(_cf, 2) if _cf is not None else None
            clear_b = round(_cb, 2) if _cb is not None else None
        except Exception:
            pass
    # ③ 문대기: 전방 여유 점프 감시 → 문 열림 판정.
    # 실측(2026-07-15): 이 홀 대기 위치에서 닫힘 0.97m / 열림 1.66m —
    # 절대 기준 1.5m + 점프 +0.5m + 2연속 관측 (0.85m/1회는 닫힌 문을 오판했음)
    if s.get("scene") == 2 and _n is not None and clear_f is not None:
        # 문 감지는 좁은 원뿔(±10°)로 재측정 — ±30° min은 문 옆 벽·문틀에 잡혀
        # 문이 열려도 값이 안 뜀 (2026-07-21 현장 실측: 열림에도 0.95→0.97m).
        # 0.95m 거리에서 ±10° = 좌우 ±17cm → 문 폭(~90cm) 한가운데만 본다.
        # UI 표시(clear_f)도 이 값으로 통일 — 기준선과 같은 자로 재야 혼란 없음.
        try:
            _df = _n._clearance(1.0, half_ang=0.175)
            if _df is not None:
                clear_f = round(_df, 2)
        except Exception:
            pass
        base_set_now = False
        with state_lock:
            base = state.get("door_base")
            if base is None:
                state["door_base"] = clear_f
                state["door_streak"] = 0
                # 진입 시점부터 아주 멀면(≥1.5m) 문이 열려 있던 것으로 판단
                if clear_f >= 1.5:
                    state["door_open"] = True
                base = clear_f
                opened_now = state["door_open"]
                base_set_now = not opened_now   # 로그는 락 밖에서 (교착 방지)
            elif not state["door_open"] and (
                    clear_f - base > 0.5
                    # 절대 규칙(1.5m)은 기준선이 진짜 닫힌 문 거리(<1.0m)일 때만 —
                    # 기준선이 원래 큰 장소에서 +11cm에 열림 오판하는 것 방지
                    or (base < 1.0 and clear_f >= 1.5)):
                streak = state.get("door_streak", 0) + 1
                state["door_streak"] = streak
                opened_now = streak >= 2          # 노이즈 방지: 2연속 관측
                if opened_now:
                    state["door_open"] = True
            else:
                state["door_streak"] = 0
                opened_now = False
        if base_set_now:
            _n._dlog(f"[SCENE] 문 닫힘 기준선 {base:.2f}m 설정 — "
                     "+0.5m 점프(2연속) 또는 1.5m 이상이면 열림 판정")
        if opened_now:
            _n._dlog(("[SCENE] 🚪 진입 시 이미 열림으로 판단 "
                      f"(전방 여유 {clear_f:.2f}m ≥ 0.85m)"
                      if base == clear_f else
                      f"[SCENE] 🚪 문 열림 감지 (전방 여유 {base:.2f}→{clear_f:.2f}m)")
                     + " — ④ 탑승 활성화")
        with state_lock:
            s["door_open"] = state["door_open"]
    # 다음 단계 해제 조건: ①⑤=press 완료 / ③=문 열림 / ②④=사용자 판단(항상 가능)
    sc = s.get("scene")
    next_ok = False
    if sc in (1, 3):
        next_ok = True
    elif sc in (0, 4):
        next_ok = (s.get("press_ok_ts") or 0) > (s.get("scene_ts") or 0)
    elif sc == 2:
        next_ok = bool(s.get("door_open"))
    # ▶ 다음 준비 알림: 게이트 있는 단계(①⑤ press·③ 문열림)에서 조건이
    # 충족되는 순간 1회만 로그 — ②④⑥은 항상 진행 가능이라 알림 생략
    if _n is not None and sc in (0, 2, 4):
        if next_ok and getattr(_n, "_next_ok_prev", None) != (sc, True):
            _n._dlog(f"[SCENE] ▶ 다음 단계 준비 완료 — {SCENES[sc + 1]} 진행 가능 "
                     "(▶ 다음 버튼 또는 N키)")
        _n._next_ok_prev = (sc, next_ok)
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
                   trim_x=_aim_trim["x_cm"], trim_y=_aim_trim["y_cm"],
                   ready=bool(s.get("centered") and s.get("press_ready")),
                   lock_shape=bool(s.get("lock_shape")),
                   dz=_dead_zone_px(s.get("target_dist")),
                   scene=s.get("scene"), scene_acc=s.get("scene_acc"),
                   authority=bool(s.get("authority", False)),
                   lease_expired=bool(s.get("lease_expired")),
                   camera_missing=(_n._camera_missing_check() if _n is not None else None),
                   infer_ms=s.get("infer_ms"),        # 마지막 추론 소요(ms)
                   infer_boxes=s.get("infer_boxes"),  # 그때 검출된 박스 수
                   arm_ext=s.get("arm_ext"),   # 팔 뻗기 현재값 — UI 슬라이더 동기화용
                   obs=(_n._obs if _n is not None else None),   # 고립 관측(A5) 캐시 — 판정은 _obs_tick이 함
                   obs_age=_obs_age(_n),   # 위 캐시를 잰 지 몇 초 됐나(이 프로세스 안에서 계산)
                   clear_f=clear_f, clear_b=clear_b,
                   door_open=bool(s.get("door_open")), scene_next_ok=next_ok,
                   door_base=s.get("door_base"))

@app.route("/layout", methods=["GET", "POST"])
def layout_route():
    """차내 버튼 배치 조회/수정. rows = [["종","1","4"],["2","5"],["문열림","3"]] 형식."""
    global _layout_rows
    if request.method == "POST":
        rows = (request.json or {}).get("rows")
        if not (isinstance(rows, list) and rows
                and all(isinstance(r, list) and r
                        and all(isinstance(t, str) for t in r)
                        for r in rows)
                and any(t.strip() for r in rows for t in r)):
            return jsonify(ok=False, error="형식 오류 — 버튼이 하나 이상 있어야 합니다"), 400
        _layout_rows = [[t.strip() for t in r] for r in rows]
        try:
            with open(LAYOUT_FILE, "w") as f:
                json.dump({"rows": _layout_rows}, f, ensure_ascii=False)
        except Exception:
            pass
        node = _node_ref[0]
        if node:
            node._map_streak = 0   # 배치가 바뀌면 정합을 처음부터 다시
            node._dlog("[MAP] 배치 수정: " + " / ".join(" ".join(r) for r in _layout_rows))
    return jsonify(ok=True, rows=_layout_rows)

@app.route("/aim_trim", methods=["POST"])
def aim_trim():
    """영점 트림: {dx_cm, dy_cm} 누적 or {reset: true}. 모든 조작은 판단 로그에 기록."""
    data = request.json or {}
    if data.get("reset"):
        _aim_trim["x_cm"] = _aim_trim["y_cm"] = 0.0
    else:
        _aim_trim["x_cm"] = round(_aim_trim["x_cm"] + float(data.get("dx_cm", 0)), 2)
        _aim_trim["y_cm"] = round(_aim_trim["y_cm"] + float(data.get("dy_cm", 0)), 2)
    _save_aim_trim()
    node = _node_ref[0]
    if node:
        node._dlog(f"[TRIM] 영점 {'리셋' if data.get('reset') else '보정'} → "
                   f"누적 x{_aim_trim['x_cm']:+.1f}cm y{_aim_trim['y_cm']:+.1f}cm "
                   "(+x=오른쪽 +y=아래로 십자 이동)")
    return jsonify(ok=True, x_cm=_aim_trim["x_cm"], y_cm=_aim_trim["y_cm"])

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
        state["press_ready"]  = False
        # X정렬 예산 = "타겟 시도당 16cm" — 이전 시도(특히 SEEK)가 먹은 예산 복구.
        # 예산 소진으로 자동 OFF됐던 경우엔 자동 재활성 (사용자가 끈 것은 존중)
        state["base_travel"] = 0.0
        if state.get("align_auto_off"):
            state["base_align"]     = True
            state["align_auto_off"] = False
            state["align_note"]     = "새 타겟 — X정렬 예산 리셋·자동 재활성"
    node = _node_ref[0]
    if node:
        node._roi_miss  = 0   # 새 타겟 → ROI 집중 추적 즉시 재개 가능
        node._croi_miss = 0   # 군집 추적도 리셋
        node._target_lock = None   # 위치 잠금도 새로 시작
        node._rot_count   = 0      # 수직 정렬 시도 횟수 리셋
        node._dlog(f"[TARGET] '{text}' 선택 — 탐색·정렬·접근 시작")
        # 인터락: 팔 고정(armleft)과의 트래젝토리 goal 선점 전쟁 방지 —
        # 타겟 작업 시작 시 대시보드에 자동 해제 요청 (꺼져 있으면 조용히 무시)
        def _stop_armleft():
            try:
                import urllib.request
                urllib.request.urlopen(urllib.request.Request(
                    "http://localhost:8080/armleft",
                    data=b'{"running": false}',
                    headers={"Content-Type": "application/json"},
                    method="POST"), timeout=1)
            except Exception:
                pass
        threading.Thread(target=_stop_armleft, daemon=True).start()
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

def _set_place(pl, send_lift=True):
    """장소 모드 설정 (hall/cab). 바뀔 때만 타겟 해제 + lift 규정 높이 선이동.
    /place 토글과 여정 단계(①⑤ 자동 모드)가 공유하는 단일 경로.
    send_lift=False면 lift goal 생략 — 호출측이 자세 goal에 lift를 합쳐 보낼 때
    (goal을 따로 쏘면 단일-goal 서버가 앞의 것을 선점·유실시키므로).

    반환: lift 선이동 결과. None=시도 안 함(모드 그대로/생략/이미 그 높이),
    True=goal 전송됨, False=못 보냄. 지금까지 이 결과는 버려져서, lift가 안
    움직여도 화면엔 아무 표시가 없었다(모드만 바뀌고 팔은 그대로)."""
    with state_lock:
        if state["place"] == pl:
            return None
        state["place"] = pl
        state["target_text"] = None
        state["phase"]       = "SELECT"
        state["centered"]    = False
        state["press_ready"]  = False
    node = _node_ref[0]
    if node:
        node._dlog(f"[MODE] 장소: {'🛗 차내 (층 숫자)' if pl == 'cab' else '🏢 홀 (호출 ▲▼)'}")
        # 모드 전환 = 새 패널 앞에 섰다는 뜻 → lift를 실측 규정 높이로 선이동
        # (탐색 스캔이 0.5m 아래에서 헤매는 시간 절약; 타겟은 방금 해제돼 서보와 충돌 없음)
        prior = LIFT_PRIOR_CALL if pl == "hall" else LIFT_PRIOR_PANEL
        with state_lock:
            cur_lift = state["lift"]
        if send_lift and (cur_lift is None or abs(cur_lift - prior) > 0.03):
            node._dlog(f"[MODE] 규정 높이 선이동: lift "
                       f"{('%.2f' % cur_lift) if cur_lift is not None else '?'}→{prior:.2f}")
            sent = node._send_goal(["joint_lift"], [prior])
            if not sent:
                node._dlog("[MODE] ⛔ 규정 높이 선이동 실패 — lift 명령이 안 나갔다")
            return sent
        return None      # 생략(send_lift=False) 또는 이미 그 높이 — 시도 안 함
    # ROS 노드가 없으면 모드만 바뀌고 lift는 못 간다 — 시도했어야 하는데 못 간
    # 것이므로 False(=실패)다. None(=시도 안 함)으로 뭉뚱그리지 않는다.
    return False if send_lift else None

@app.route("/place", methods=["POST"])
def place_toggle():
    """장소 모드 토글: 홀(밖, 호출 ▲▼) ↔ 차내(안, 층 숫자). 전환 시 타겟 해제."""
    with state_lock:
        pl = "cab" if state["place"] == "hall" else "hall"
    lift_ok = _set_place(pl)
    # ok는 장소 토글의 성공 여부다 — 토글 자체는 진짜 성공했으니 뒤집지 않는다.
    # lift가 갔는지는 별개 사실이라 별개 필드로 알린다(전송성공≠동작완료:
    # lift_ok=True도 "goal을 보냈다"까지이고 "그 높이에 도달했다"는 아니다).
    return jsonify(ok=True, place=pl, lift_ok=lift_ok)

# ── 여정 단계 + 조종 패드 (리허설 티칭용) ──────────────────────
SCENES = ["① 호출 press", "② 문앞 정렬", "③ 문 열림 대기",
          "④ 탑승", "⑤ 층 press", "⑥ 하차"]
# 단계별 자동 안무 (2026-07-15 16:35 리허설 티칭값) — 단계 클릭 시 자동 재생.
# Esc로 중단 → 패드 보정 → 같은 단계 재클릭 시 잔여만 이어서 실행 (scene_acc 기준).
# ※ press 정렬이 매번 다른 위치에서 끝나므로(이번엔 X정렬 11.5cm) 몇 cm 보정은 정상.
SCENE_MOVES = {
    1: [("fwd", 56.5), ("rot", -90.0)],   # ② 문앞: 전진 56.5cm → 우회전 90°
    3: [("fwd", 185.0)],                  # ④ 탑승: 전진 185cm
    5: [("fwd", -186.0)],                 # ⑥ 하차: 후진 186cm
}

@app.route("/scene", methods=["POST"])
def scene_set():
    """여정 단계 전환 — 이전 단계의 누적 이동량을 로그로 매듭짓고 카운터 리셋.
    성공 리허설의 [SCENE] 매듭 로그가 곧 시나리오 고정 거리의 티칭값이 된다."""
    n = int((request.json or {}).get("n", 0))
    n = max(0, min(len(SCENES) - 1, n))
    node = _node_ref[0]
    with state_lock:
        prev = state.get("scene")
        acc  = dict(state.get("scene_acc") or {})
        _pok   = state.get("press_ok_ts") or 0   # 경고 판정용 — 덮어쓰기 전에 캡처
        _sts   = state.get("scene_ts") or 0
        _dopen = bool(state.get("door_open"))
        state["scene"] = n
        if prev != n:      # 같은 단계 재클릭 = 이어하기 → 누적 유지 (잔여만 재실행)
            state["scene_acc"] = {"fwd_cm": 0.0, "rot_deg": 0.0}
            state["scene_ts"]  = time.time()
            if n == 2:                   # ③ 문대기: 닫힌 문 기준선 새로 측정
                state["door_base"]   = None
                state["door_open"]   = False
                state["door_streak"] = 0
    # 자동 안무 진행 중(_auto_busy)에 "같은 버튼"을 다시 누름 = 정지 토글.
    # → 안무만 중단하고 자세변경·재실행은 생략 (Esc와 별개로 버튼으로도 멈춤).
    #   이후 같은 버튼을 또 누르면 아래 일반 경로로 잔여부터 재개(기존 이어하기 유지).
    if node and prev == n and getattr(node, "_auto_busy", False):
        node._step_abort = True
        node._dlog(f"[SCENE] {SCENES[n]} — 같은 버튼 재클릭 → 자동 동작 정지 (다시 누르면 재개)")
        return jsonify(ok=True, scene=n, stopped=True)
    if node:
        if prev is not None and prev != n and (abs(acc.get("fwd_cm", 0)) > 0.5
                                               or abs(acc.get("rot_deg", 0)) > 1):
            node._dlog(f"[SCENE] {SCENES[prev]} 매듭 — 누적: "
                       f"전진 {acc['fwd_cm']:+.1f}cm · 회전 {acc['rot_deg']:+.0f}°")
        node._dlog(f"[SCENE] {SCENES[n]} " + ("재개" if prev == n else "시작"))
        # 조건 미충족 전진은 차단하지 않고 경고만 (금지 대신 경고 원칙 — 연습 환경 배려)
        if prev is not None and n > prev:
            if prev in (0, 4) and not (_pok > _sts):
                node._dlog(f"[SCENE] ⚠ {SCENES[prev]} press 미완료 상태로 진행")
            elif prev == 2 and n == 3 and not _dopen:
                node._dlog("[SCENE] ⚠ 문 열림 미감지 상태로 탑승 시작")
    # 단계별 자세 — 반드시 한 goal로 묶어 전송 (단일-goal 서버의 선점·유실 방지)
    _auth = _authority_ok()
    if node and not _auth:
        node._dlog("[SCENE] ⛔ 제어권 없음 — 자세 변경·자동 안무 생략 (단계 표시만 전환)")
    if n in (0, 4):
        # press 단계: 홀/차내 모드 + 그리퍼 전방(인식 자세) + 열기 + lift 규정 높이
        _set_place("hall" if n == 0 else "cab", send_lift=False)
        if node and _auth:
            prior = LIFT_PRIOR_CALL if n == 0 else LIFT_PRIOR_PANEL
            node._dlog("[SCENE] 인식 자세 — 그리퍼 닫고→손목 전방→그리퍼 열기 (충돌·과부하 방지)")
            # #1 과부하 방지: 손목 회전은 반드시 그리퍼 닫힌 채로. 몸통 근처에서 그리퍼가
            # 열린 채 회전하면 손가락이 몸통에 닿아 과부하 → 닫기→회전→열기로 순서 분리.
            if not node._move_joint_wait(GRIPPER_JOINT, GRIPPER_CLOSE_M, 1, 4.0):  # 1) 닫기 보장
                node._dlog("[SCENE] ⚠ 그리퍼 닫기 실패 — 이후 손목 회전 과부하 위험")
            if not node._move_joint_wait("joint_wrist_yaw", WRIST_YAW_DEFAULT, 2, 8.0):  # 2) 손목 전방(닫힌 채)
                node._dlog("[SCENE] ⚠ 손목 전방 회전 실패 — 인식 자세 미완성")
            if not node._send_goal(                                                # 3) 그리퍼 열기 + lift
                    ["joint_wrist_pitch", "joint_wrist_roll", GRIPPER_JOINT, "joint_lift"],
                    [WRIST_PITCH_DEFAULT, 0.0, GRIPPER_OPEN_M, prior]):
                node._dlog("[SCENE] ⚠ 그리퍼 열기·lift 전송 실패 — 인식 자세 미완성")
    else:
        # 이동 단계(②③④⑥): 타겟 강제 해제 — press 실패 경로는 타겟을 유지하므로,
        # 그 상태로 주행에 들어가면 서보/자동접근(phase TRACK 잔존)이 주행 중
        # 팔을 도로 뻗을 수 있음 (문틀 충돌 위험). 이동 단계 = 추적 종료가 원칙.
        with state_lock:
            if state["target_text"] is not None:
                state["target_text"] = None
                state["phase"]       = "SELECT"
                state["centered"]    = False
                state["press_ready"] = False
                if node:
                    _DECISIONS.append(f"{time.strftime('%H:%M:%S')} "
                                      f"[SCENE] 이동 단계 진입 — 타겟 자동 해제 (추적 종료)")
        # 이동 단계(②③④⑥): 팔 수납 + 그리퍼 안쪽 + 닫기 — 문틀·벽 충돌 방지
        if node and _auth:
            node._dlog("[SCENE] 이동 자세 — 그리퍼 먼저 닫고→팔 수납·손목 안쪽 (충돌·과부하 방지)")
            # #1 과부하 방지: 손목을 안쪽(WRIST_YAW_IN)으로 돌리기 전에 그리퍼를 먼저 닫는다.
            # 열린 채 안쪽으로 돌면 손가락이 몸통에 닿아 wrist_yaw 서보 과부하.
            if not node._move_joint_wait(GRIPPER_JOINT, GRIPPER_CLOSE_M, 1, 4.0):
                node._dlog("[SCENE] ⚠ 그리퍼 닫기 실패 — 이후 손목 안쪽 회전 과부하 위험")
            if not node._send_goal(
                    ["joint_wrist_pitch", "joint_wrist_yaw", "joint_wrist_roll", ARM_JOINT],
                    [WRIST_PITCH_DEFAULT, WRIST_YAW_IN, 0.0, ARM_EXT_MIN]):
                node._dlog("[SCENE] ⚠ 이동 자세(팔 수납·손목 안쪽) 전송 실패 — 충돌 위험")
    # 단계 전환 = 진행 중이던 자동 안무·수동 스텝 즉시 취소 (새 의도가 우선)
    if node:
        node._step_abort = True
    # 자동 안무 재생 (②④⑥): 티칭값 − 누적 = 잔여를 자동 실행
    if node and n in SCENE_MOVES:
        threading.Thread(target=node._run_scene_moves, args=(n,), daemon=True).start()
    return jsonify(ok=True, scene=n)

@app.route("/step_move", methods=["POST"])
def step_move():
    """조종 패드 단발 스텝: {cm: ±n} 전/후진 또는 {deg: ±n} 좌(+)/우(−) 회전.
    ⇧1(몸체이동) 토글과 무관한 수동 전용 통로 — 사용자가 지켜보며 1스텝씩 명령.
    가드는 낮춰 적용(시작 여유 ≥ 이동량+5cm, 이동 중 26cm 비상정지), ⇧2로 해제 가능."""
    d = request.json or {}
    node = _node_ref[0]
    if node is None:
        return jsonify(ok=False, error="node 없음"), 503
    cm  = float(d.get("cm", 0.0))
    deg = float(d.get("deg", 0.0))
    threading.Thread(target=node._manual_step, args=(cm, deg), daemon=True).start()
    return jsonify(ok=True)

@app.route("/step_stop", methods=["POST"])
def step_stop():
    """진행 중인 수동 스텝 즉시 중단 (Esc)."""
    node = _node_ref[0]
    if node:
        node._step_abort = True
    return jsonify(ok=True)

@app.route("/gripper", methods=["POST"])
def gripper_ctrl():
    """그리퍼 수동 열기/닫기 (테스트·튜닝용)."""
    do_open = bool((request.json or {}).get("open", True))
    node = _node_ref[0]
    if node is None:
        return jsonify(ok=False, open=do_open, error="node 없음"), 503
    ok = node.set_gripper(GRIPPER_OPEN_M if do_open else GRIPPER_CLOSE_M)
    if not ok:
        return jsonify(ok=False, open=do_open, error="모션 명령 실패(액션서버 없음/고립 가능)")
    return jsonify(ok=True, open=do_open)

@app.route("/lift", methods=["POST"])
def lift_ctrl():
    """팔 높이(joint_lift) 수동 조정. /arm_ext와 같은 순서로 검사한다 —
    관문 먼저, 값 파싱은 그 뒤. 파싱이 앞에 있으면 제어권도 없는 요청의
    잘못된 값이 500을 내고, 프록시는 그걸 502로 덮어 원인이 사라진다."""
    node = _node_ref[0]
    if node is None:
        return jsonify(ok=False, error="node 없음"), 503
    # 다른 이동 진입점과 같은 관문 — 여기만 빠져 있어서 제어권 없이도 팔이
    # 올라갔다. 리스를 안 쥔 쪽이 관절을 움직이면 소유자가 둘이 된다.
    if not _authority_ok():
        return jsonify(ok=False,
                       error="제어권 없음 — 대시보드(8080)에서 엘리베이터 제어권을 부여하세요"), 403
    try:
        pos = float((request.json or {}).get("lift"))
    except (TypeError, ValueError):
        # 본문이 없거나 lift가 없으면 float(None)이 TypeError를 낸다 — 예전엔
        # 이게 500으로 나가 프록시의 502에 가려졌다. 400으로 정직하게 말한다.
        return jsonify(ok=False, error="lift 값이 숫자가 아님"), 400
    if not math.isfinite(pos):
        # nan은 아래 클램프를 그대로 통과해 최대 높이가 된다. /arm_ext와 달리
        # 리프트에는 1회 이동 상한이 없어서 그대로 전 구간을 올라간다.
        return jsonify(ok=False, error="lift 값이 유한한 숫자가 아님(nan/inf)"), 400
    pos = max(0.15, min(1.10, pos))
    ok = node.set_lift(pos)
    if ok:
        # /arm_ext와 같은 표식 — 리프트도 3초짜리 궤적으로 팔뭉치를 옮긴다.
        # 여기만 열어두면 팔과 바퀴 동시 이동 구멍이 그대로 남는다.
        node._arm_cmd_ts = time.time()
    if not ok:
        return jsonify(ok=False, lift=pos, error="모션 명령 실패(액션서버 없음/고립 가능)")
    return jsonify(ok=True, lift=pos)

ARM_EXT_STEP_MAX = 0.05   # 한 번의 수동 명령으로 허용하는 최대 이동(m)
ARM_CMD_BLOCK_SEC = 2.5   # 팔 명령 후 베이스를 막는 시간(초) — 궤적 2초 + 여유
ARM_EXT_FRESH_SEC = 1.0   # 팔 현재값이 이보다 오래되면 상한을 못 믿는다

@app.route("/arm_ext", methods=["POST"])
def arm_ext_ctrl():
    """팔 뻗기(wrist_extension) 수동 조정 — 매핑용 근접 촬영.

    팔이 사람·패널 코앞에서 움직이므로 관문을 전부 지난 뒤에만 움직인다.
    한 번에 갈 수 있는 거리는 서버가 자른다 — 브라우저가 보낸 목표값을 그대로
    믿으면 슬라이더가 튀거나 오조작 한 번에 팔이 끝까지 나간다. 현재값을 아는
    쪽은 서버이므로 상한도 서버가 강제한다."""
    node = _node_ref[0]
    if node is None:
        return jsonify(ok=False, error="node 없음"), 503
    if not _authority_ok():
        return jsonify(ok=False,
                       error="제어권 없음 — 대시보드(8080)에서 엘리베이터 제어권을 부여하세요"), 403
    with state_lock:
        pressing = state["pressing"]
        cur = state.get("arm_ext")
    if pressing:
        return jsonify(ok=False, error="누르기 진행 중 — 팔 수동조작 불가"), 409
    # 베이스와 상호배타: 자동 안무·정렬이 도는 중에 팔을 움직이면 두 명령이
    # 겹쳐 궤적이 선점되고, 무엇보다 사람이 예측할 수 없는 합성 동작이 된다.
    if getattr(node, "_step_busy", False) or getattr(node, "_nudging", False):
        return jsonify(ok=False, error="베이스 이동/정렬 진행 중 — 팔 수동조작 불가"), 409
    if cur is None:
        # 현재값을 모르면 한 걸음 상한을 강제할 수 없다 — 모르면 안 움직인다.
        return jsonify(ok=False, error="팔 위치 미수신(joint_states) — 상한을 강제할 수 없어 거부"), 409
    last = getattr(node, "_arm_ext_mono", None)
    if last is None or (time.monotonic() - last) > ARM_EXT_FRESH_SEC:
        # 값이 있어도 언제 잰 건지 모르면 없는 것과 같다. joint_states가 끊긴
        # 동안 얼어붙은 현재값으로 ±5cm를 재면 상한이 그대로 무력화된다.
        return jsonify(ok=False,
                       error="팔 위치가 오래됨(joint_states 끊김) — 상한을 강제할 수 없어 거부"), 409
    try:
        target = float((request.json or {}).get("arm_ext"))
    except (TypeError, ValueError):
        return jsonify(ok=False, error="arm_ext 값이 숫자가 아님"), 400
    if not math.isfinite(target):
        # nan/inf는 float()를 통과하고 min/max도 통과해 상한까지 그대로 간다.
        # 브라우저를 안 믿는다는 설계라면 여기서 걸러야 한다.
        return jsonify(ok=False, error="arm_ext 값이 유한한 숫자가 아님(nan/inf)"), 400
    target = max(ARM_EXT_MIN, min(ARM_EXT_MAX, target))     # 절대 안전범위
    capped = target
    if abs(target - cur) > ARM_EXT_STEP_MAX:                # 1회 이동 상한
        capped = cur + (ARM_EXT_STEP_MAX if target > cur else -ARM_EXT_STEP_MAX)
        capped = max(ARM_EXT_MIN, min(ARM_EXT_MAX, capped))
    node._dlog(f"[ARM] 수동 뻗기: {cur:.3f}→{capped:.3f}m"
               + (f" (요청 {target:.3f} — 1회 {ARM_EXT_STEP_MAX*100:.0f}cm 상한으로 자름)"
                  if capped != target else ""))
    # 비블로킹 전송(_move_joint_wait는 완료까지 최대 8초 붙잡아 UI가 굳는다).
    # 덤으로 _send_single_joint가 _last_motion_ts를 갱신하므로, 팔이 막 움직인
    # 직후의 스냅샷은 기존 정착 게이트에 그대로 걸린다(선명한 사진 + 낡은 좌표
    # 짝짓기 방지) — 따로 배선하지 않는다.
    ok = node._send_single_joint(ARM_JOINT, capped, duration_sec=2)
    if ok:
        # 베이스 배타용 전용 표식 — 전송은 즉시 돌아오지만 궤적은 몇 초 더 간다.
        # _last_motion_ts를 재사용하면 베이스가 자기 이동으로 자기를 막는다.
        node._arm_cmd_ts = time.time()
    if not ok:
        return jsonify(ok=False, arm_ext=capped, requested=target,
                       error="모션 명령 실패(액션서버 없음/고립 가능)")
    # ok는 "goal을 보냈다"까지지 "그 위치에 도달했다"가 아니다 — 실제 위치는
    # joint_states로 확인한다(전송성공≠완료).
    return jsonify(ok=True, arm_ext=capped, requested=target,
                   capped=(capped != target))

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
        state["press_ready"]  = False
    return jsonify(ok=True)

LEASE_TTL = 6.0   # 리스 만료 임계값(초) — 대시보드 하트비트 주기(2s)의 3배 여유

def _revoke_authority(reason: str, expired: bool = False):
    """제어권 강제 회수(공통 경로) — 즉시 전면 정지 + 안전 복구(가드 ON).
    /authority 핸들러(대시보드 요청)와 워치독(리스 만료)이 이 함수 하나를 공유한다
    (중복 금지 — 회수 로직이 두 곳에 있으면 한쪽만 고치는 실수가 난다).
    state_lock은 RLock이지만 _dlog가 내부에서 다시 이 락을 잡으므로(L321-323 부근
    동결사건 주석) — publish/dlog는 반드시 락을 놓은 뒤 실행한다."""
    node = _node_ref[0]
    with state_lock:
        prev = bool(state.get("authority", False))
        state["authority"]     = False
        state["target_text"]   = None
        state["phase"]         = "SELECT"
        state["centered"]      = False
        state["press_ready"]   = False
        state["guard_off"]     = False   # 엘베 밖 = 충돌 보호 가드 다시 ON
        state["lease_expired"] = expired
    if node and prev:
        node._step_abort = True                # 수동 스텝·자동 안무 즉시 탈출
        try:
            node._cmd_pub.publish(Twist())     # 바퀴 정지 (안전 최우선)
        except Exception:
            pass
        node._dlog(f"[AUTH] ⛔ 제어권 회수됨 ({reason}) — 모든 이동 중단·차단, 관찰만 가능")

def _lease_watchdog():
    """리스 deadman — 대시보드 하트비트(2s)가 끊기면 LEASE_TTL(6s) 후 엘베가 스스로
    권한을 내리고 가드를 복원한다. 순수 데몬 스레드(ROS 타이머 아님) — 감시자가
    ROS 스핀 스레드와 운명을 같이하면 정작 감시해야 할 상황(스핀 죽음)에서 같이
    죽는다. 만료 판정만 락 안에서, 회수 실행(_revoke_authority)은 락 밖에서."""
    while True:
        time.sleep(0.5)
        with state_lock:
            if not state.get("authority"):
                continue
            if time.monotonic() < state.get("lease_deadline", 0):
                continue
        _revoke_authority("리스 만료 — 대시보드 접촉 끊김", expired=True)

@app.route("/authority", methods=["GET", "POST"])
def authority_route():
    """이동 제어권 부여/회수 — 대시보드(8080)가 호출하는 주도권 관리 API.
    회수 시: 진행 중인 모든 이동 즉시 중단 + 바퀴 정지 + 타겟 해제."""
    if request.method == "GET":
        with state_lock:
            return jsonify(granted=bool(state.get("authority", False)))
    granted = bool((request.json or {}).get("granted", False))
    if not granted:
        _revoke_authority("대시보드")
        return jsonify(ok=True, granted=False)
    node = _node_ref[0]
    with state_lock:
        prev = bool(state.get("authority", False))
        # 엘베 모드 진입: 좁은 엘베에서 베이스 전후 이동이 필요하고, 라이다 가드가
        # 켜져있으면 벽/문을 장애물로 보고 이동을 막음 → 몸체이동 ON + 가드 OFF 자동.
        state["authority"]      = True
        state["base_align"]     = True
        state["guard_off"]      = True
        state["lease_deadline"] = time.monotonic() + LEASE_TTL  # 하트비트(재POST)마다 갱신
        state["lease_expired"]  = False
    if node and not prev:
        node._dlog("[AUTH] ✅ 제어권 부여됨 — 이동 가능 (몸체이동 ON + 가드 OFF 자동)")
    return jsonify(ok=True, granted=True)

@app.route("/wrist_forward", methods=["POST"])
def wrist_forward():
    """홈 포즈: 손목 전방 + 팔 완전 수납 — 반드시 '한 번의 목표'로 묶어 전송.
    (따로 보내면 컨트롤러가 이전 목표를 선점·취소해 마지막 것만 실행됨)"""
    node = _node_ref[0]
    if node is None:
        return jsonify(ok=False, error="node 없음"), 503
    ok = node._send_goal(
        ["joint_wrist_pitch", "joint_wrist_yaw", "joint_wrist_roll", ARM_JOINT],
        [WRIST_PITCH_DEFAULT, WRIST_YAW_DEFAULT, 0.0, ARM_EXT_MIN])
    if not ok:
        return jsonify(ok=False, error="모션 명령 실패(액션서버 없음/고립 가능)")
    return jsonify(ok=True)

@app.route("/wrist_pitch", methods=["POST"])
def wrist_pitch():
    pitch = float(request.json.get("pitch", 0.0))
    pitch = max(-1.57, min(0.5, pitch))
    node = _node_ref[0]
    if node is None:
        return jsonify(ok=False, pitch=pitch, error="node 없음"), 503
    ok = node.set_wrist_pitch(pitch)
    if not ok:
        return jsonify(ok=False, pitch=pitch, error="모션 명령 실패(액션서버 없음/고립 가능)")
    return jsonify(ok=True, pitch=pitch)

@app.route("/wrist_yaw", methods=["POST"])
def wrist_yaw():
    yaw = float(request.json.get("yaw", 0.0))
    yaw = max(-2.88, min(1.67, yaw))
    node = _node_ref[0]
    if node is None:
        return jsonify(ok=False, yaw=yaw, error="node 없음"), 503
    ok = node.set_wrist_yaw(yaw)
    if not ok:
        return jsonify(ok=False, yaw=yaw, error="모션 명령 실패(액션서버 없음/고립 가능)")
    return jsonify(ok=True, yaw=yaw)

def _camera_info_dict(msg):
    if msg is None:
        return None
    return {"K": list(msg.k), "D": list(msg.d), "w": msg.width, "h": msg.height,
            "distortion_model": msg.distortion_model}

def _stamp_to_sec(stamp):
    return None if stamp is None else stamp.sec + stamp.nanosec * 1e-9

def _save_snapshot(node, out_dir, dash_payload):
    """VLM 매핑용 원본 프레임 저장 — daemon 스레드에서 실행(PNG16 인코딩 GIL이
    OCR 스레드를 막지 않게). 모르는 값은 0/단위행렬 대체 없이 null + missing[]."""
    missing = []
    try:
        os.makedirs(out_dir, exist_ok=True)

        grip_raw, grip_stamp = node._last_grip_raw, node._last_grip_stamp
        body_raw, body_stamp = node._last_body_raw, node._last_body_stamp
        with state_lock:
            rot_grip, rot_body = state["rot_grip"], state["rot_body"]
        with node._depth_lock:
            # 회전 전 센서 네이티브(#P6) — grip_raw·camera_info K와 같은 축이어야 픽셀→3D가 맞음
            depth_frame = node._last_grip_depth_native
            depth_stamp = node._depth_stamp
        grip_info, body_info = node._grip_color_info, node._body_color_info
        joint_msg = node._last_joint_state

        cameras = {"gripper": {}, "body": {}}

        if grip_raw is not None:
            cv2.imwrite(os.path.join(out_dir, "grip_color.jpg"), grip_raw, [cv2.IMWRITE_JPEG_QUALITY, 95])
            cameras["gripper"]["color"] = "grip_color.jpg"
        else:
            missing.append("grip_color")
            cameras["gripper"]["color"] = None
        cameras["gripper"]["color_stamp"] = _stamp_to_sec(grip_stamp)

        if depth_frame is not None:
            cv2.imwrite(os.path.join(out_dir, "grip_depth.png"), depth_frame)   # PNG16 — JPEG 금지
            cameras["gripper"]["depth"] = "grip_depth.png"
        else:
            missing.append("grip_depth")
            cameras["gripper"]["depth"] = None
        cameras["gripper"]["depth_stamp"] = _stamp_to_sec(depth_stamp)
        cameras["gripper"]["aligned"] = True   # 후보 토픽이 aligned_depth_to_color 고정
        cameras["gripper"]["color_info"] = _camera_info_dict(grip_info)
        if grip_info is None:
            missing.append("grip_color_info")
        cameras["gripper"]["frame_color"] = SNAPSHOT_FRAMES["grip_color"]
        cameras["gripper"]["orientation"] = "sensor_native"   # 회전 미적용 — camera_info K와 축 일치(#P6)

        if body_raw is not None:
            cv2.imwrite(os.path.join(out_dir, "body_color.jpg"), body_raw, [cv2.IMWRITE_JPEG_QUALITY, 95])
            cameras["body"]["color"] = "body_color.jpg"
        else:
            missing.append("body_color")
            cameras["body"]["color"] = None
        cameras["body"]["color_stamp"] = _stamp_to_sec(body_stamp)
        cameras["body"]["depth"] = None   # 몸체는 depth 미구독(#21 부하 방지) — 항상 없음
        missing.append("body_depth")
        cameras["body"]["color_info"] = _camera_info_dict(body_info)
        if body_info is None:
            missing.append("body_color_info")
        cameras["body"]["frame_color"] = SNAPSHOT_FRAMES["body_color"]
        cameras["body"]["orientation"] = "sensor_native"   # 회전 미적용 — camera_info K와 축 일치(#P6)

        joint_states = dict(zip(joint_msg.name, joint_msg.position)) if joint_msg is not None else None
        if joint_states is None:
            missing.append("joint_states")

        # ── on-demand TF: 이 요청만을 위해 생성 → 캐시 채울 시간 대기 → lookup → 파괴 ──
        import tf2_ros
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
        from rclpy.duration import Duration
        from rclpy.time import Time as RclpyTime

        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, node)
        time.sleep(1.0)   # 0.5~1.0초 — TF 캐시가 찰 시간

        tf_out, tf_errors = {}, {}

        def _lu(target, source, key, attempts=3, retry_wait=0.3):
            # 실측: 리스너 생성 직후 static TF 캐시가 아직 안 찼으면 첫 lookup이
            # "frame does not exist"로 실패할 수 있음 — 짧게 재시도(그래도 안 되면
            # null+errors, 크래시나 전체 실패로 만들지 않음).
            last_err = None
            for i in range(attempts):
                try:
                    t = buffer.lookup_transform(target, source, RclpyTime(), timeout=Duration(seconds=1.0))
                    tr, rot = t.transform.translation, t.transform.rotation
                    tf_out[key] = {
                        "translation": {"x": tr.x, "y": tr.y, "z": tr.z},
                        "rotation": {"x": rot.x, "y": rot.y, "z": rot.z, "w": rot.w},
                    }
                    return
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    last_err = e
                    if i < attempts - 1:
                        time.sleep(retry_wait)
            tf_out[key] = None
            tf_errors[key] = repr(last_err)
            missing.append(f"tf:{key}")

        _lu(SNAPSHOT_FRAMES["base"], SNAPSHOT_FRAMES["grip_color"], "base_link__grip_color")
        _lu(SNAPSHOT_FRAMES["base"], SNAPSHOT_FRAMES["body_color"], "base_link__body_color")
        _lu(SNAPSHOT_FRAMES["odom"], SNAPSHOT_FRAMES["base"], "odom__base_link")
        _lu(SNAPSHOT_FRAMES["map"], SNAPSHOT_FRAMES["base"], "map__base_link")   # 안 되면 null(실패 아님, 미측위일 뿐)
        tf_out["errors"] = tf_errors
        try:
            listener.unregister()
        except Exception:
            pass

        meta = {
            "label": dash_payload.get("label"),
            "primary_frame": SNAPSHOT_FRAMES["base"],
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "dash_time": dash_payload.get("dash_time"),
            "floor": dash_payload.get("floor"),
            "battery": dash_payload.get("battery"),
            "amcl_pose": dash_payload.get("amcl_pose"),
            "localization_available": tf_out.get("map__base_link") is not None,
            "rot_grip": rot_grip, "rot_body": rot_body,   # 정보용 — 이미지는 sensor_native(회전 미적용)
            "joint_states": joint_states,
            "cameras": cameras,
            "tf": tf_out,
            "missing": sorted(set(missing)),
        }
        with open(os.path.join(out_dir, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        node._dlog(f"[SNAPSHOT] 저장 완료: {out_dir} missing={meta['missing']}")
    except Exception as e:
        node.get_logger().error(f"스냅샷 저장 실패: {e!r}")
        try:
            node._dlog(f"[SNAPSHOT] 저장 실패: {e!r}")
        except Exception:
            pass
    finally:
        node._snapshot_in_progress = False

@app.route("/snapshot", methods=["POST"])
def snapshot_route():
    """VLM 매핑용 원본 프레임 스냅샷 — 표시용 jpeg가 아니라 raw + camera_info + TF.
    게이트: 자동안무/베이스정렬/누르기 중이거나 이미 저장 중이면 거부(모션블러·시점불일치 방지)."""
    node = _node_ref[0]
    if node is None:
        return jsonify(ok=False, error="ROS 노드 미초기화"), 503
    if getattr(node, "_auto_busy", False) or getattr(node, "_nudging", False):
        return jsonify(ok=False, error="자동 안무/베이스 정렬 진행 중 — 스냅샷 불가"), 409
    with state_lock:
        pressing = state["pressing"]
    if pressing:
        return jsonify(ok=False, error="누르기 진행 중 — 스냅샷 불가"), 409
    if node._snapshot_in_progress:
        return jsonify(ok=False, error="이미 스냅샷 저장 중"), 409

    try:
        free = shutil.disk_usage(os.path.expanduser("~")).free
    except Exception as e:
        return jsonify(ok=False, error=f"디스크 확인 실패: {e!r}"), 500
    if free < 1 * 1024 ** 3:
        return jsonify(ok=False, error=f"디스크 여유 부족({free / 1e9:.2f}GB < 1GB) — 저장 거부"), 507

    data = request.json or {}
    label = "".join(c for c in str(data.get("label") or "snap") if c.isalnum() or c in "-_") or "snap"
    ts = time.strftime("%Y%m%dT%H%M%S")
    out_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "snapshots", f"{ts}_{label}"))

    node._snapshot_in_progress = True
    threading.Thread(target=_save_snapshot, args=(node, out_dir, data), daemon=True).start()
    return jsonify(ok=True, dir=out_dir)

_node_ref = [None]

# ── ROS2 노드 ─────────────────────────────────────────────────────────
class ElevatorTracker(Node):
    def __init__(self):
        super().__init__("elevator_tracker")
        self.bridge         = CvBridge()
        self._processing    = False
        self._goal_done     = True
        self._goal_ok       = True   # 마지막 goal이 '수락'됐는지 — 거부/예외를 완료와 구분
        self._det_mem       = {}     # 탐지 기억 (라벨 → 마지막 박스/시각)

        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        # wrist_extension(팔 뻗기 합산값)은 /stretch/joint_states 에만 있음 (실측 확인)
        self.create_subscription(JointState, "/stretch/joint_states", self._on_joints, 10)

        # ── D405 depth (color 정렬) — 버튼까지 거리 측정용 (press 준비 0단계) ──
        # 로봇을 움직이지 않음. 화면에 거리 숫자만 표시.
        self._depth_frame = None   # 회전 보정된 uint16 배열 (mm)
        self._last_grip_depth_native = None   # 스냅샷용 — 회전 전(센서 네이티브, #P6)
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
        # (비활성화) 시작 시 손목 자동 전방 회전 안 함 — 실행 즉시 아무것도 안 움직이게.
        # armleft가 손목 잡고 있을 때 main 켜면 손목이 확 도는 문제 방지. 손목 자세는
        # 사용자가 수동으로(손목 전방 버튼 / /wrist_forward) 지정.
        # self.create_timer(3.0, self._init_wrist_once)   # ← 되살리려면 주석 해제
        self._wrist_initialized = False

        # 두 카메라 동시 구독 (전환 없음)
        # gripper는 토픽 이름이 부팅마다 달라서 후보 전부 구독 (발행되는 쪽만 프레임 옴)
        for t in GRIPPER_TOPICS:
            self.create_subscription(Image, t, self._on_image, 10)
            self._dlog(f"gripper: {t}")
        self.create_subscription(Image, BODY_TOPIC, self._on_image_body,
                                 qos_profile_sensor_data)
        self._dlog(f"body   : {BODY_TOPIC}")

        # ── 스냅샷용 원본 프레임(리사이즈·레터박스 전) + camera_info ────────────
        # 표시용 jpeg(state["jpeg_frame*"])는 리사이즈·회전·레터박스가 들어가 VLM
        # 3D 매핑에 못 씀 — 여기 raw는 참조만 들고 있음(복사 아님).
        self._last_grip_raw = None
        self._last_grip_stamp = None
        self._last_body_raw = None
        self._last_body_stamp = None
        self._depth_stamp = None
        self._last_joint_state = None
        self._snapshot_in_progress = False
        self._grip_color_info = None
        self._body_color_info = None

        # ── 카메라 미수신 안전망 — "대기"로 표시돼 정상처럼 보이다 헛걸음시킨 사고 방지 ──
        self._boot_mono = time.monotonic()
        self._last_grip_frame_mono = None
        self._camera_missing_logged = False

        # ── 고립 관측(A5) — 드라이버 도달성 + body/depth 프레임 신선도 주기 샘플 ──
        # 판정은 전부 _obs_tick 안에서만 하고 /status는 이 캐시를 읽기만 한다.
        # 관측·표시 전용 — 여기서 재시작/리스/누르기 같은 행동은 하지 않는다.
        self._last_body_frame_mono = None
        self._last_depth_frame_mono = None
        self._obs_ticks = 0
        self._obs_nodes = None
        self._obs = {"driver": "unknown", "body": "unknown",
                     "depth": "unknown", "detail": "관측 시작 전"}
        self._obs_mono = None    # _obs를 마지막으로 갱신한 시각(신선도 판단용)
        self._arm_ext_mono = None   # state["arm_ext"]를 마지막으로 받은 시각
        self._arm_cmd_ts = 0.0      # 마지막 팔 수동 명령 전송 시각(베이스 배타용)
        # 관측 전용 신규 타이머 — 위(L1907)의 비활성 손목 초기화 타이머와 무관하며
        # 어떤 관절도 움직이지 않는다(server_is_ready·monotonic 차·캐시 기록뿐).
        self.create_timer(OBS_PERIOD, self._obs_tick)

        for t in GRIPPER_INFO_TOPICS:
            self.create_subscription(CameraInfo, t, self._on_grip_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, BODY_INFO_TOPIC, self._on_body_info,
                                 qos_profile_sensor_data)

        self._dlog("Ready — open http://localhost:5000")

    def _on_grip_info(self, msg):
        self._grip_color_info = msg

    def _on_body_info(self, msg):
        self._body_color_info = msg

    def _obs_tick(self):
        """고립 관측 샘플(A5) — 드라이버 액션서버 도달성 + body/depth 프레임 신선도.

        왜 주기 샘플인가: 지금까지 도달성 확인은 명령을 쏘는 순간에만 일어났다.
        그래서 아무 조작도 안 하는 동안 드라이버가 사라져도 화면은 멀쩡했고,
        누르기를 시켜본 뒤에야 알았다(8/31 무증상 실패).

        판정은 이 콜백 '한 곳'에서만 하고 /status는 결과 캐시를 읽기만 한다 —
        라우트 안에서 판정을 돌리면 폴링 빈도에 따라 판정 시점이 흔들린다.
        관측·표시 전용: 재시작·리스·누르기 등 어떤 행동도 하지 않는다.
        스핀 스레드에서 돌므로 값싼 호출만 쓴다(wait_for_server 같은 대기 금지)."""
        now = time.monotonic()
        booting = (now - self._boot_mono) <= OBS_BOOT_GRACE

        def fresh(last):
            # 미관측(기동 유예 중 아직 한 장도 못 받음)은 unknown — ok로 새면 안 된다.
            if last is None:
                return "unknown" if booting else "stale"
            return "ok" if (now - last) <= OBS_STALE_SEC else "stale"

        body  = fresh(self._last_body_frame_mono)
        depth = fresh(self._last_depth_frame_mono)

        # server_is_ready()는 이미 발견된 그래프를 즉시 조회만 한다(대기 없음).
        # _send_goal이 쓰는 바로 그 핸들이라, 여기 ok면 명령이 갈 상대가 있다는 뜻.
        # 단 "명령이 도달한다"까지이고 "관절이 실제로 움직였다"는 아니다.
        try:
            ready = self.action_client.server_is_ready()
        except Exception:
            ready = None
        if ready is None:
            driver = "unknown"
        elif ready:
            driver = "ok"
        else:
            driver = "unknown" if booting else "stale"   # 기동 직후는 DDS 발견 전일 뿐

        # 노드 수는 detail 문자열 재료로만 쓴다 — 판정·트리거로 승격하지 않는다
        # (대시보드 쪽이 자기고립되면 전부 0으로 보여 오탐이 되는 구조라 폐기한 방식).
        self._obs_ticks += 1
        if self._obs_ticks % OBS_NODES_EVERY == 1:
            try:
                self._obs_nodes = len(self.get_node_names())
            except Exception:
                self._obs_nodes = None

        def age(last):
            return "—" if last is None else f"{now - last:.1f}s"
        detail = (f"노드 {self._obs_nodes if self._obs_nodes is not None else '?'}개 · "
                  f"body {age(self._last_body_frame_mono)} · "
                  f"depth {age(self._last_depth_frame_mono)} · 한계 {OBS_STALE_SEC:.0f}s")

        prev = self._obs
        # 통째 교체(부분 수정 아님) — 읽는 쪽은 참조 하나만 잡으면 일관된 스냅샷이 된다.
        self._obs = {"driver": driver, "body": body, "depth": depth, "detail": detail}
        # 이 타이머는 executor에서 돈다 — 스핀이 굶으면 위 캐시가 얼어붙는데
        # /status는 캐시만 읽으므로 얼어붙은 ok가 정상처럼 나간다. 갱신 시각을
        # 남겨 읽는 쪽이 "언제 잰 값인지"로 걸러낼 수 있게 한다(판정은 안 한다).
        self._obs_mono = now

        # 전이할 때만 1회 로그(_camera_missing_check의 🚨 패턴) — 매 초 찍으면 로그가
        # 묻혀 정작 사고 때 안 보인다. state_lock은 잡지 않는다(_dlog는 디스크 I/O).
        for key, label in (("driver", "드라이버 액션서버"),
                           ("body", "body 카메라"), ("depth", "depth 프레임")):
            was, is_ = prev.get(key), self._obs[key]
            if was == is_:
                continue
            if is_ == "stale":
                self._dlog(f"🚨 {label} 관측 끊김 — {OBS_STALE_SEC:.0f}초 이상 "
                           f"무응답/무수신 (관측만 함, 자동복구 없음)")
            elif was == "stale" and is_ == "ok":
                self._dlog(f"✅ {label} 관측 복구")

    def _camera_missing_check(self) -> bool:
        """그리퍼 카메라 미수신 감지 — 기동 10초 유예 후, 프레임을 한 번도
        못 받았거나 5초 넘게 끊겼으면 True. "대기"로 속아 헛걸음시킨 사고(구독
        실패해도 정상처럼 보임) 재발 방지. 상태 전이(정상→미수신) 때만 1회 🚨
        로그(복구되면 리셋 — 재발 시 다시 경고, 스팸은 안 남)."""
        now = time.monotonic()
        if now - self._boot_mono <= 10.0:
            return False
        last = self._last_grip_frame_mono
        missing = (last is None) or (now - last > 5.0)
        if missing and not self._camera_missing_logged:
            self._camera_missing_logged = True
            self._dlog("🚨 카메라 미수신 — 엘베앱 재시작 필요 (그리퍼 0프레임)")
        elif not missing:
            self._camera_missing_logged = False
        return missing

    def _init_wrist_once(self):
        if self._wrist_initialized:
            return
        # 앱 재시작 직후엔 DDS가 드라이버 액션 서버를 아직 발견 못 했을 수 있음 —
        # 이번 틱은 건너뛰고 3초 타이머가 다시 부를 때 재시도 (발견 후에만 초기화)
        if not self.action_client.wait_for_server(timeout_sec=0.5):
            self._dlog("드라이버 액션 서버 대기 중… (3초 후 재시도)")
            return
        self._wrist_initialized = True
        # 손목만 전방으로. 그리퍼는 여기서 안 건드림 — armleft가 닫아둔 상태를 유지하고,
        # 시작 시 자동으로 열면 손목 회전과 겹쳐 충돌·과부하 위험. 그리퍼는 사용자가 수동으로.
        self._send_goal(
            ["joint_wrist_pitch", "joint_wrist_yaw", "joint_wrist_roll"],
            [WRIST_PITCH_DEFAULT, WRIST_YAW_DEFAULT, 0.0])
        self._dlog(
            f"Wrist initialized: pitch={WRIST_PITCH_DEFAULT}, yaw={WRIST_YAW_DEFAULT}, "
            "roll=0 (그리퍼는 수동)"
        )

    def _on_image_body(self, msg):
        """body(D435i) 프레임 — 모니터링용 표시만 (OCR/제어 없음)."""
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # 스냅샷용 — 회전 적용 "전" 센서 네이티브(참조, 복사 아님). camera_info의 K는
            # 센서 원본 기준이라, 이미지도 원본이어야 픽셀↔K가 일치해 VLM 3D가 안 틀어짐
            # (회전 후 프레임을 저장하면 몸체는 rot_body 기본값 자체가 0이 아니라 상시 불일치).
            self._last_body_raw = raw
            self._last_body_stamp = msg.header.stamp
            # 프레임 "도착" 시각 — 신선도 판정에 header.stamp를 쓰면 발행측 시계·
            # 정지한 스탬프에 속는다. 도착 기준 monotonic만 쓴다(고립 관측용).
            self._last_body_frame_mono = time.monotonic()
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
        q_ = msg.pose.pose.orientation
        # 쿼터니언 → yaw — 회전 스텝의 실회전량 피드백용 (시간 개루프 언더슛 해결)
        self._odom_yaw = float(np.arctan2(2.0 * (q_.w * q_.z + q_.x * q_.y),
                                          1.0 - 2.0 * (q_.y * q_.y + q_.z * q_.z)))

    def _clearance(self, direction: float, half_ang: float = GUARD_HALF_ANG):
        """이동 방향(+1 전진 / -1 후진) ±half_ang 섹터의 최소 장애물 거리(m).
        로봇 자기 몸(SELF_HIT_MIN 미만)은 무시. 스캔 없으면 None.
        기본 ±30°는 이동 가드용. 문 열림 감지는 ±10°로 좁혀 호출할 것 —
        30°는 0.95m에서 좌우 55cm씩 퍼져 문 옆 벽을 계속 잡는다."""
        s = self._scan
        if s is None:
            return None
        r = np.asarray(s.ranges, dtype=float)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        base_ang = (ang + LASER_YAW_OFFSET + np.pi) % (2 * np.pi) - np.pi
        target = 0.0 if direction > 0 else np.pi
        diff = np.abs((base_ang - target + np.pi) % (2 * np.pi) - np.pi)
        m = (diff < half_ang) & np.isfinite(r) & (r > SELF_HIT_MIN)
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
            if not state["base_align"] or not state.get("authority", False):
                return False   # 몸체이동 OFF 또는 제어권 없음 — 바퀴 절대 금지
        if not guard_off:
            c = self._clearance(direction)
            # 이동량 비례 가드: 이동 후에도 CLEAR_MARGIN 이격이 남으면 허용
            if c is None or c < abs(move_m) + CLEAR_MARGIN:
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
                if c is not None and c < CLEAR_MARGIN:   # 이동 중 최소 이격 침범 → 즉시 정지
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
            if not state["base_align"] or not state.get("authority", False):
                return False   # 몸체이동 OFF 또는 제어권 없음 — 회전도 금지
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

    # ── 조종 패드 수동 스텝 (여정 티칭용) ──────────────────────
    def _manual_step(self, cm, deg):
        """단발 스텝 (별도 스레드). base_align(⇧1)과 무관한 수동 전용 통로 —
        클릭 1번 = 정해진 양 1번이라 마스터 스위치를 요구할 이유가 없음.
        가드는 낮춰 적용: 시작 여유 ≥ 이동량+5cm, 이동 중 26cm 침범 시 비상정지.
        (라이다가 25cm 미만 레이는 자기 몸으로 보고 버리므로 26cm이 측정 바닥.)
        guard_off(⇧2)면 생략. 결과는 [MOVE] 로그 + scene_acc 누적 = 티칭 기록."""
        if not _authority_ok():
            self._dlog("[MOVE] ⛔ 제어권 없음 — 대시보드에서 엘리베이터 제어권을 부여하세요")
            return
        if getattr(self, "_step_busy", False):
            self._dlog("[MOVE] 이미 이동 중 — 무시 (멈추려면 Esc)")
            return
        with state_lock:
            pressing_now = state.get("pressing")
            guard_off = state["guard_off"]
        if pressing_now:
            self._dlog("[MOVE] press 진행 중 — 스텝 거부")
            return
        # 팔 수동 명령은 비블로킹이라 응답이 돌아온 뒤에도 궤적이 몇 초 더 간다.
        # 그 창에 바퀴가 움직이면 이 기능이 막으려던 합성 동작(팔+베이스 동시)이
        # 그대로 난다. duration_sec=2 궤적을 덮도록 2.5초 잡는다.
        if time.time() - getattr(self, "_arm_cmd_ts", 0.0) < ARM_CMD_BLOCK_SEC:
            self._dlog("[MOVE] 팔 수동 이동 중 — 스텝 거부 (팔과 바퀴 동시 이동 금지)")
            return
        self._step_busy  = True
        self._step_abort = False
        self._nudging    = True     # 자동 서보·접근과 바퀴 명령 겹침 방지
        try:
            if abs(deg) >= 1.0:
                self._manual_rot(deg, guard_off)
            elif abs(cm) >= 0.5:
                self._manual_trans(cm, guard_off)
        finally:
            self._nudging   = False
            self._step_busy = False

    def _manual_trans(self, cm, guard_off):
        move_m = max(-0.5, min(0.5, cm / 100.0))
        direction = 1.0 if move_m > 0 else -1.0
        lbl  = "전진" if direction > 0 else "후진"
        side = "전방" if direction > 0 else "후방"
        # 긴 직진(≥30cm, 탑승·하차 안무)은 증속 — 문에 걸리는 것 방지
        spd = BOARD_SPEED if abs(move_m) >= 0.30 else MANUAL_SPEED
        c0 = self._clearance(direction)
        if not guard_off and (c0 is None or c0 < abs(move_m) + 0.05):
            self._dlog(f"[MOVE] {lbl} {abs(move_m)*100:.0f}cm 거부 — {side} 여유 "
                       f"{'측정불가' if c0 is None else '%.2fm' % c0} < 이동+5cm")
            return
        self._last_motion_ts = time.time()
        start_xy = self._odom_xy
        msg = Twist(); msg.linear.x = direction * spd
        t0 = time.time()
        timeout = abs(move_m) / spd * 3.0 + 1.0
        stopped = None
        while time.time() - t0 < timeout:
            if getattr(self, "_step_abort", False):
                stopped = "사용자 정지"; break
            if not guard_off:
                c = self._clearance(direction)
                if c is not None and c < 0.26:
                    stopped = f"비상정지 {c:.2f}m"; break
            if start_xy is not None and self._odom_xy is not None:
                dx_ = self._odom_xy[0] - start_xy[0]
                dy_ = self._odom_xy[1] - start_xy[1]
                if (dx_*dx_ + dy_*dy_) ** 0.5 >= abs(move_m):
                    break
            elif time.time() - t0 > abs(move_m) / spd:
                break
            self._cmd_pub.publish(msg)
            time.sleep(1 / 15)
        self._cmd_pub.publish(Twist())
        moved = abs(move_m)   # 오도메트리 없으면 명령값으로 기록
        if start_xy is not None and self._odom_xy is not None:
            dx_ = self._odom_xy[0] - start_xy[0]
            dy_ = self._odom_xy[1] - start_xy[1]
            moved = (dx_*dx_ + dy_*dy_) ** 0.5
        with state_lock:
            acc = state.get("scene_acc") or {"fwd_cm": 0.0, "rot_deg": 0.0}
            acc["fwd_cm"] = acc.get("fwd_cm", 0.0) + direction * moved * 100.0
            state["scene_acc"] = acc
            acc_now = acc["fwd_cm"]
        c1 = self._clearance(direction)
        self._dlog(f"[MOVE] {lbl} {direction*moved*100:+.1f}cm"
                   + (f" ({stopped})" if stopped else "")
                   + f" — 단계 누적 {acc_now:+.1f}cm · {side} 여유 "
                   + ("?" if c1 is None else f"{c1:.2f}m"))

    def _manual_rot(self, deg, guard_off):
        ang = max(-1.6, min(1.6, deg * 3.14159265 / 180.0))   # 1회 최대 ~90°
        lbl = "좌회전" if ang > 0 else "우회전"
        # 팔 뻗은 채 회전 금지 (기존 안전 규칙) — 자동으로 먼저 수납
        with state_lock:
            _ext = state["arm_ext"]
        if _ext is not None and _ext > 0.02:
            self._dlog("[MOVE] 회전 전 팔 수납 (안전)")
            self._move_joint_wait(ARM_JOINT, ARM_EXT_MIN, 2, 8.0)
        if not guard_off:
            s = self._scan
            if s is not None:
                r = np.asarray(s.ranges, dtype=float)
                m = np.isfinite(r) & (r > SELF_HIT_MIN)
                if m.any() and float(r[m].min()) < 0.26:
                    self._dlog(f"[MOVE] {lbl} 거부 — 주변 {float(r[m].min()):.2f}m "
                               "장애물 (26cm 미만)")
                    return
        self._last_motion_ts = time.time()
        # 소각도(<15°)는 저속 회전 — 관성 오버슛을 줄여 미세 보정이 수렴하게
        wz_mag = MANUAL_ROT_SPEED if abs(ang) >= 0.26 else 0.15
        wz  = wz_mag if ang > 0 else -wz_mag
        dur = abs(ang) / wz_mag
        msg = Twist(); msg.angular.z = wz
        t0 = time.time()
        stopped = None
        # 오도메트리 yaw 피드백: 시간 개루프는 가감속 램프 때문에 항상 언더슛
        # (2026-07-21 현장 실측: -90° 안무 후 -10° 수동 보정 필요했음).
        # 전진(_manual_trans)이 odom 거리 피드백으로 정확했던 것과 같은 방식.
        # odom 미수신이면 기존 시간 방식으로 폴백.
        yaw_prev = getattr(self, "_odom_yaw", None)
        use_odom = yaw_prev is not None
        turned = 0.0
        # 관성 리드: 정지 명령 후에도 ~0.2s분 더 돎 → 그만큼 일찍 명령을 끊고
        # 관성이 나머지를 채우게 한다. 없으면 +5° 오버슛 ↔ 반대 보정의 무한
        # 진동 (2026-07-21 ② 안무 -85°↔-95° 왕복 실측)
        stop_lead = 0.23 * wz_mag
        while time.time() - t0 < (dur * 3.0 + 1.0 if use_odom else dur):
            if getattr(self, "_step_abort", False):
                stopped = "사용자 정지"; break
            if use_odom:
                cur = getattr(self, "_odom_yaw", None)
                if cur is not None:
                    turned += (cur - yaw_prev + np.pi) % (2 * np.pi) - np.pi
                    yaw_prev = cur
                    if abs(turned) >= abs(ang) - stop_lead:
                        break   # 목표 직전 정지 → 관성이 잔여분을 채움
            self._cmd_pub.publish(msg)
            time.sleep(1 / 15)
        self._cmd_pub.publish(Twist())
        if use_odom:
            time.sleep(0.3)   # 관성 잔여 회전까지 실측에 포함
            cur = getattr(self, "_odom_yaw", None)
            if cur is not None:
                turned += (cur - yaw_prev + np.pi) % (2 * np.pi) - np.pi
            done_deg = float(np.degrees(turned))
        else:
            done_deg = deg * (min(1.0, (time.time() - t0) / dur) if dur > 0 else 0.0)
        with state_lock:
            acc = state.get("scene_acc") or {"fwd_cm": 0.0, "rot_deg": 0.0}
            acc["rot_deg"] = acc.get("rot_deg", 0.0) + done_deg
            state["scene_acc"] = acc
            acc_now = acc["rot_deg"]
        self._dlog(f"[MOVE] {lbl} {done_deg:+.0f}°"
                   + (f" ({stopped})" if stopped else "")
                   + f" — 단계 누적 회전 {acc_now:+.0f}°")

    def _run_scene_moves(self, n):
        """여정 단계 자동 안무 재생 (별도 스레드) — 티칭값에서 단계 누적을 뺀
        잔여만 실행. Esc(_step_abort)로 중단 → 패드 보정 → 같은 단계 재클릭 시
        잔여부터 이어감. 가드 거부/비상정지로 진행이 멈추면 남은 양을 로그로 알림."""
        moves = SCENE_MOVES.get(n)
        if not moves:
            return
        if not _authority_ok():
            self._dlog("[AUTO] ⛔ 제어권 없음 — 자동 안무 생략 (대시보드에서 부여 필요)")
            return
        # 이전 자동 안무 스레드가 있으면 종료를 기다림 (중복 주행 방지)
        t0 = time.time()
        while getattr(self, "_auto_busy", False) and time.time() - t0 < 3.0:
            time.sleep(0.05)
        self._auto_busy = True
        try:
            # 이동 자세 goal(그리퍼 스윙)이 끝날 때까지 잠깐 대기 후 주행
            t0 = time.time()
            while not self._goal_done and time.time() - t0 < 5.0:
                time.sleep(0.1)
            self._step_abort = False
            self._run_scene_moves_inner(n, moves)
        finally:
            self._auto_busy = False

    def _run_scene_moves_inner(self, n, moves):
        for kind, target in moves:
            key = "fwd_cm" if kind == "fwd" else "rot_deg"
            with state_lock:
                done = (state.get("scene_acc") or {}).get(key, 0.0)
            remain = target - done
            tol = 1.0 if kind == "fwd" else 3.0
            if abs(remain) < tol:
                continue
            self._dlog(f"[AUTO] {SCENES[n]} — "
                       + (f"{'전진' if remain > 0 else '후진'} {abs(remain):.0f}cm"
                          if kind == "fwd" else f"회전 {remain:+.0f}°")
                       + " 자동 실행 (Esc=중단)")
            flips, last_sign = 0, (1 if remain > 0 else -1)
            while abs(remain) >= tol:
                if getattr(self, "_step_abort", False):
                    self._dlog("[AUTO] 중단됨 — 패드 보정 후 같은 단계를 다시 누르면 "
                               f"잔여({abs(remain):.0f}{'cm' if kind == 'fwd' else '°'})부터 이어감")
                    return
                if kind == "fwd":
                    # 50cm 단위(클램프 최대) — 25cm 쪼개기는 정지·재출발 오버헤드만
                    # 만들었음. 이동 중 26cm 비상정지 감시는 스텝 크기와 무관하게 연속.
                    step = max(-50.0, min(50.0, remain))
                    self._manual_step(step, 0.0)
                else:
                    step = max(-90.0, min(90.0, remain))
                    self._manual_step(0.0, step)
                with state_lock:
                    new_done = (state.get("scene_acc") or {}).get(key, 0.0)
                if abs(new_done - done) < 0.3:   # 스텝이 거부되거나 전혀 못 움직임
                    self._dlog(f"[AUTO] 진행 불가 (가드/장애물) — 잔여 "
                               f"{abs(target - new_done):.0f}{'cm' if kind == 'fwd' else '°'}. "
                               "패드로 상황 정리 후 단계 재클릭")
                    return
                done = new_done
                remain = target - done
                # 진동 감지: 잔여 부호가 2번 뒤집히면 (오버슛↔보정 왕복) 그만 —
                # 무한 왕복으로 시간·배터리를 태우는 것보다 ±수° 오차가 낫다
                # (2026-07-21 ② 안무 -85°↔-95° 16회 왕복 실측)
                s_ = 1 if remain > 0 else -1
                if abs(remain) >= tol and s_ != last_sign:
                    flips += 1
                    if flips >= 2:
                        self._dlog(f"[AUTO] 잔여 {remain:+.0f}"
                                   f"{'cm' if kind == 'fwd' else '°'} — 진동 감지, "
                                   "이 정도로 마침 (필요하면 패드로 미세 보정)")
                        break
                last_sign = s_
        self._dlog(f"[AUTO] {SCENES[n]} 안무 완료 ✓")

    def _maybe_base_nudge(self, ex: float, dist):
        """좌우 픽셀 오차 → 안전 확인 후 베이스 소폭 전/후진 (별도 스레드)."""
        with state_lock:
            enabled  = state["base_align"] and state.get("authority", False)
            pressing = state["pressing"]
            travel   = state["base_travel"]
        if not enabled or pressing or self._nudging:
            return
        if travel >= BASE_TRAVEL_MAX:
            # 상한 도달 → X정렬 자동 OFF (메시지 반복 방지). 재활성화하면 누적 리셋됨.
            with state_lock:
                state["base_align"] = False
                state["align_auto_off"] = True   # 예산 소진 OFF — 사용자 OFF와 구분,
                                                 # 새 타겟 선택 시 자동 재활성 대상
            self._set_align_note(
                f"누적 이동 {travel*100:.0f}cm 초과 — X정렬 자동 OFF. "
                "로봇 주차 위치를 옮긴 뒤 다시 켜세요. (새 타겟 선택 시 자동 복구)")
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
                # 이동량 비례 가드: 이동 후에도 CLEAR_MARGIN 이격이 남으면 허용
                if clear < abs(move_m) + CLEAR_MARGIN:
                    self._set_align_note(
                        f"⚠ {side} {clear:.2f}m — 막힘, 이동 보류 "
                        f"(필요 {abs(move_m)+CLEAR_MARGIN:.2f}m)"); return

            dur = abs(move_m) / BASE_SPEED
            msg = Twist(); msg.linear.x = direction * BASE_SPEED
            t0 = time.time()
            aborted = False
            while time.time() - t0 < dur:
                if not guard_off:
                    c = self._clearance(direction)       # 이동 중에도 계속 감시
                    if c is not None and c < CLEAR_MARGIN:   # 최소 이격 침범 → 즉시 정지
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
        """판단/행동 로그: ROS 터미널 + 웹 로그 패널 + 진단 파일에 동시 기록."""
        self.get_logger().info(msg)
        with state_lock:
            _DECISIONS.append(f"{time.strftime('%H:%M:%S')} {msg}")
        # 진단 파일(robot_diag)에도 보존 — 인식/판단 이벤트를 사후 수치분석용으로
        # 파일에 남긴다. 디스크 I/O는 반드시 state_lock '밖'에서(디스크 정체 시
        # 앱 전체가 락에 묶여 동결되던 지점 — 반복 사고). _diaglog는 모듈 전역.
        if _diaglog:
            # echo=False: _dlog가 이미 get_logger().info로 터미널에 찍는다.
            # echo 기본 True면 print까지 나가 같은 줄이 대시보드에 2번 뜬다.
            try: _diaglog.log("DEC", msg, echo=False)
            except Exception: pass

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
        self._last_joint_state = msg   # 스냅샷용 — 전체 name/position (오프라인 FK 보험)
        for name, pos in zip(msg.name, msg.position):
            if name == "joint_lift":
                with state_lock: state["lift"] = pos
            elif name == "joint_wrist_yaw":
                with state_lock: state["yaw"] = pos
            elif name == ARM_JOINT:
                with state_lock: state["arm_ext"] = pos
                # 이 값이 언제 잰 것인지. joint_states가 끊기면 state["arm_ext"]는
                # 옛값 그대로 남는데, 그 값을 기준으로 ±5cm를 재면 상한이 무의미해진다
                # (실제 0.05m인데 0.45m로 알고 있으면 '5cm'가 40cm 이동이 된다).
                self._arm_ext_mono = time.monotonic()

    def _on_depth(self, msg):
        """aligned depth 프레임 저장. color와 동일하게 회전 보정해 좌표계를 맞춤."""
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                (msg.height, msg.width))
            # 스냅샷용 — 회전 적용 "전" 센서 네이티브(#P6, grip color·camera_info와 축 일치).
            # 거리 샘플링용 self._depth_frame(회전 적용본, 아래)과는 별개 경로.
            depth_native = arr.copy()
            # 도착 시각(고립 관측용) — _depth_lock '밖' 단순 대입. float 대입은
            # GIL 원자라 락이 필요 없고, 락 범위를 넓히면 depth 경로가 더 잘 막힌다.
            self._last_depth_frame_mono = time.monotonic()
            # color와 동일한 회전 적용 → 거리 샘플링 좌표계 일치
            with state_lock:
                rot = state["rot_grip"]
            arr = _rotate_steps(arr, rot)
            with self._depth_lock:
                self._depth_frame = arr
                self._depth_stamp = msg.header.stamp
                self._last_grip_depth_native = depth_native

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
        """정조준 + 누르기 가능 거리 = '클릭 대기' 상태 (히스테리시스 래치).
        이때는 모든 자동 이동을 동결한다 — 사용자가 누를 표적이 움직이면
        클릭 타이밍을 잡을 수 없음. 클릭하면 press 시퀀스가 이어받는다."""
        with state_lock:
            return bool(state["centered"] and state.get("press_ready"))

    def _apply_button_map(self, detections, raw_dets, place0):
        """버튼 맵 정합: 탐지 박스들을 실물 배치(_layout_rows)에 사상해서
        글자가 안 읽힌 버튼의 정체를 위치로 확정한 합성 탐지를 추가.

        오독 앵커 방어 3중:
        ① 모양 모순 — 앵커 가설로 사상했을 때 격자 밖으로 나가는 박스는 감점
        ② belief 가중 투표 — 여러 앵커가 충돌하면 (읽힌 글자들의 일치 합)이 큰 가설 승
        ③ 2프레임 연속 같은 정합일 때만 발동 — 지나가는 오독 1방 무시
        """
        rows = _layout_rows
        if place0 != "cab" or not rows:
            return detections
        boxes = [d for d in raw_dets if d.get("score", 0) >= 0.4]
        # ③ depth 유효 박스가 3개+면 depth 미측정 박스는 격자에서 제외 —
        # 원거리 오탐(멀리 있는 '6', depth None으로 1.5m 필터 통과)이 격자·피치를
        # 오염시키던 것 방지. 유효 박스가 적으면(depth 전반 실패) 기존 동작 유지
        _bv = [d for d in boxes
               if self._depth_at(int((d["box"]["x1"] + d["box"]["x2"]) / 2),
                                 int((d["box"]["y1"] + d["box"]["y2"]) / 2))
               is not None]
        if len(_bv) >= 3:
            boxes = _bv
        if len(boxes) < 3:
            return detections
        # 1) 버튼 피치(간격) 추정 — 픽셀 기하 정합의 자(尺).
        # 순번(몇 번째 박스) 기반이 아니라 픽셀 거리 기반이라, 그리퍼가 일부
        # 버튼을 가려도 나머지 박스의 셀 판정이 밀리지 않는다 (가림 면역).
        bs = sorted(boxes, key=lambda d: (d["box"]["y1"] + d["box"]["y2"]) / 2)
        med_w = float(np.median([d["box"]["x2"] - d["box"]["x1"] for d in boxes]))
        med_h = float(np.median([d["box"]["y2"] - d["box"]["y1"] for d in boxes]))
        tol = max(10.0, med_h * 0.7)
        obs_rows: list = []
        for d in bs:
            cy = (d["box"]["y1"] + d["box"]["y2"]) / 2
            if obs_rows and abs(cy - obs_rows[-1]["cy"]) <= tol:
                obs_rows[-1]["items"].append(d)
                n = len(obs_rows[-1]["items"])
                obs_rows[-1]["cy"] += (cy - obs_rows[-1]["cy"]) / n
            else:
                obs_rows.append({"cy": cy, "items": [d]})
        xg, yg = [], []
        for r in obs_rows:
            cs = sorted((d["box"]["x1"] + d["box"]["x2"]) / 2 for d in r["items"])
            xg += [b - a for a, b in zip(cs, cs[1:])]
        rcys = [r["cy"] for r in obs_rows]
        yg = [b - a for a, b in zip(rcys, rcys[1:])]
        pitch_x = float(np.median(xg)) if xg else med_w * 1.6
        pitch_y = float(np.median(yg)) if yg else med_h * 1.6
        if pitch_x < med_w * 0.8:   # 겹침 등 비정상 측정 방어
            pitch_x = med_w * 1.6
        if pitch_y < med_h * 0.8:
            pitch_y = med_h * 1.6

        def _ctr(d):
            return ((d["box"]["x1"] + d["box"]["x2"]) / 2,
                    (d["box"]["y1"] + d["box"]["y2"]) / 2)

        # 2) 앵커: 읽힌 글자가 배치에 존재하는 탐지 (belief 0.45+)
        cell_of = {tok: (ri, ci) for ri, row in enumerate(rows)
                   for ci, tok in enumerate(row) if tok.strip()}
        anchors = [(d, d.get("text", "").strip()) for d in detections
                   if d.get("text", "").strip() in cell_of
                   and d.get("belief", 0) >= 0.45]
        if not anchors:
            # ── 앵커 우선 규칙: 최근(15초)에 앵커 기반 정합이 커밋됐다면 모양
            # 전용 정합은 개입 금지. 앵커는 급이 다른 증거인데, 앵커 판독 주기
            # (5~7초 실측) 사이사이에 모양 셔플이 맵을 덮어써 타겟 예상 위치가
            # 반 칸씩 점프 → 서보 무한 왕복을 만들던 것 차단
            # (2026-07-24 실측: 아랫줄 '4' 1분51초 수렴 실패, lift 0.87↔0.92 왕복)
            if time.time() - getattr(self, "_map_anchored_ts", 0.0) < 15.0:
                self._map_streak = 0
                return detections
            # ── 모양 전용 정합 (판독 가뭄 폴백) ──────────────────────────────
            # 글자가 하나도 안 읽혀도 박스 패턴이 배치와 "유일하게" 끼워 맞으면
            # 라벨 추론. 실측 동기: 차내에서 박스 5개가 배치 패턴 그대로 잡혔는데
            # 앵커 부재로 몇 분간 침묵 (2026-07-15 11:21). 채택 조건이 앵커
            # 정합보다 훨씬 엄격: 하드 모순 0 + 유일해 + 박스 5개+ + 2프레임.
            # 라벨은 belief 0.5 + shape 표식 (화면 '?' 접두, 잠금 시 배너 표시).
            if len(boxes) < 5:
                self._map_streak = 0
                return detections
            fits = []
            b0 = boxes[0]
            b0x, b0y = _ctr(b0)
            for tok0, (r0, c0) in cell_of.items():
                # 가설: "박스0 = 셀 (r0,c0)" — 전 박스가 무모순으로 들어맞아야 생존
                assign: dict = {}
                ok = True
                for bd in boxes:
                    bx, by = _ctr(bd)
                    fc = c0 + (bx - b0x) / pitch_x
                    fr = r0 + (by - b0y) / pitch_y
                    c2, r2 = round(fc), round(fr)
                    if (abs(fc - c2) > 0.35 or abs(fr - r2) > 0.35
                            or not (0 <= r2 < len(rows) and 0 <= c2 < len(rows[r2]))
                            or not rows[r2][c2].strip()
                            or (r2, c2) in assign):
                        ok = False
                        break
                    assign[(r2, c2)] = bd
                if ok and len(assign) >= 5:
                    key = tuple(sorted(rc for rc in assign))
                    if key not in [k for k, _ in fits]:
                        fits.append((key, assign))
            if len(fits) != 1:
                if time.time() - getattr(self, "_shape_log_ts", 0) > 8.0:
                    self._shape_log_ts = time.time()
                    self._dlog("[MAP] 모양 정합 보류 — "
                               + (f"배치 일치 {len(fits)}가지 (모호)" if fits
                                  else "무모순 일치 없음")
                               + f" (박스 {len(boxes)}개, 앵커 0개)")
                self._map_streak = 0
                return detections
            _assign_s = fits[0][1]
            matched_s = [(bd, rows[r][c].strip())
                         for (r, c), bd in _assign_s.items()]
            sig = ("SHAPE",) + tuple(sorted(tok for _, tok in matched_s))
            if getattr(self, "_map_sig", None) == sig:
                self._map_streak = getattr(self, "_map_streak", 0) + 1
            else:
                self._map_sig, self._map_streak = sig, 1
            if self._map_streak < 2:
                return detections
            # 유도용 셀 좌표 (점프 가드는 앵커 정합과 동일 적용)
            (r_a, c_a), bd_a = next(iter(_assign_s.items()))
            _bax, _bay = _ctr(bd_a)
            _nc = {tok.strip(): (_bax + (ci - c_a) * pitch_x,
                                 _bay + (ri - r_a) * pitch_y)
                   for ri, row in enumerate(rows)
                   for ci, tok in enumerate(row) if tok.strip()}
            _oldc = getattr(self, "_map_cells", None)
            _acc = True
            if _oldc and time.time() - getattr(self, "_map_cells_ts", 0) < 12.0:
                _sh = set(_nc) & set(_oldc)
                if _sh and max(abs(_nc[t][0] - _oldc[t][0])
                               + abs(_nc[t][1] - _oldc[t][1]) for t in _sh) > 100.0:
                    _pd = getattr(self, "_cells_pending", None)
                    _acc = bool(_pd and all(
                        t in _nc and abs(_nc[t][0] - _pd[t][0])
                        + abs(_nc[t][1] - _pd[t][1]) < 60.0 for t in _pd))
                    if not _acc:
                        self._cells_pending = _nc
            if _acc:
                self._map_cells, self._map_cells_ts = _nc, time.time()
                self._cells_pending = None
            out = list(detections)
            _added_s = []
            for bd, tok in matched_s:
                synth = dict(bd)
                synth["text"] = tok
                synth["belief"] = 0.5
                synth["shape"] = True   # 모양 추론 표식 — 화면 '?' 접두, 잠금 배너 표시
                out.append(synth)
                _added_s.append(tok)
            if time.time() - getattr(self, "_map_log_ts", 0) > 5.0:
                self._map_log_ts = time.time()
                self._dlog(f"[MAP] 모양 전용 정합 (앵커 0, 박스 {len(boxes)}개 유일 일치, "
                           f"streak {self._map_streak}) → {', '.join(_added_s)} 위치 추론")
            return out
        # 3) 가설 채점: 앵커 셀 기준으로 각 박스의 (행,열)을 "픽셀 거리/피치"로
        # 직접 계산 → 격자점에서 벗어남/빈 칸 위/한 칸 중복 = 모순 감점,
        # 읽힌 글자와 배치 일치 = belief 가중 투표
        scored = []
        for ad, at in anchors:
            ar, ac = cell_of[at]
            ax, ay = _ctr(ad)
            assign: dict = {}
            conflicts, votes = 0.0, 0.0
            _gr_h, _gr_w = len(rows), max(len(r) for r in rows)
            for bd in boxes:
                bx, by = _ctr(bd)
                fc = ac + (bx - ax) / pitch_x
                fr = ar + (by - ay) / pitch_y
                c2, r2 = round(fc), round(fr)
                res = abs(fc - c2) + abs(fr - r2)
                # 패널 영역에서 멀리 떨어진 박스는 "무관" (감점 0) — 실물 환경의
                # 원거리 요소가 올바른 정합까지 깎아내리던 것 방지. 격자 근처
                # (앵커 기준 한 격자 폭 이내)의 이탈만 모양 모순으로 취급
                _near = (abs(fr - ar) < _gr_h + 0.6 and abs(fc - ac) < _gr_w + 0.6)
                if abs(fc - c2) > 0.35 or abs(fr - r2) > 0.35:
                    if _near:
                        conflicts += 1.0      # 격자점에서 벗어난 근처 박스 = 불일치
                    continue
                if not (0 <= r2 < len(rows) and 0 <= c2 < len(rows[r2])) \
                        or not rows[r2][c2].strip():
                    if _near:
                        conflicts += 1.0      # 격자 밖/빈 칸 위 (근처) = 모양 모순
                    continue
                if (r2, c2) in assign:
                    conflicts += 1.0          # 한 칸에 박스 둘 = 모순 (가까운 쪽 유지)
                    if assign[(r2, c2)][1] <= res:
                        continue
                assign[(r2, c2)] = (bd, res)
            matched = [(bd, rows[r][c].strip())
                       for (r, c), (bd, _) in assign.items()]
            for bd, tok in matched:
                bt = bd.get("text", "").strip()
                if bt in cell_of:
                    if bt == tok:
                        votes += bd.get("belief", 0.3)
                    else:
                        # 글자 불일치 감점은 belief 비례 — 저신뢰 오독(실물 금속에서
                        # 프레임마다 다발)이 올바른 정합 점수를 인플레이션 감점하던 것
                        # 방지. 고신뢰 판독과의 모순만 크게 친다
                        conflicts += min(1.0, bd.get("belief", 0.3))
            score = len(matched) + votes * 2.0 - conflicts * 2.0
            scored.append((score, matched, ad, at))
        # ── 모순 판독 강등 ("1이 맨 윗줄에 잡히면 안 된다" — 사용자 설계) ──
        # 자기 가설 점수가 명백히 음수 = 그 판독이 맞다면 박스들이 배치와
        # 기하적으로 모순 → 오독 확정. 삭제하지 않고 강등만: 잠금·앵커 후보
        # 제외 + 화면 회색(x접두) 표시. 오판 시에도 정보 손실 없음.
        # 안전장치: 박스 5개+(구조 증거 충분) / belief 0.8+ 판독은 불가침
        if len(boxes) >= 5:
            for score, _m, ad, at in scored:
                if score < -2.0 and ad.get("belief", 0) < 0.8 \
                        and not ad.get("suspect"):   # 문턱 -2: 경계선 아군 오사 방지
                    ad["suspect"] = True
                    if time.time() - getattr(self, "_suspect_log_ts", 0) > 5.0:
                        self._suspect_log_ts = time.time()
                        self._dlog(f"[MAP] '{at}' 판독 강등 — 위치가 배치와 모순 "
                                   "(잠금 후보 제외)")
        alive = [t for t in scored if not t[2].get("suspect")]
        if not alive:
            self._map_streak = 0
            return detections
        score, matched, ad_best, at = max(alive, key=lambda x: x[0])
        if score < 4.0 or len(matched) < 3:
            self._map_streak = 0
            return detections
        sig = tuple(sorted(tok for _, tok in matched))
        if getattr(self, "_map_sig", None) == sig:
            self._map_streak = getattr(self, "_map_streak", 0) + 1
        else:
            self._map_sig, self._map_streak = sig, 1
        if self._map_streak < 2:
            return detections
        # 앵커 기반 정합 커밋 — 이후 15초간 모양 전용 정합의 맵 개입을 차단하는 기준
        self._map_anchored_ts = time.time()
        # 맵 유도 탐색용: 모든 셀(가려진 버튼 포함)의 예상 화면 좌표 저장 —
        # 타겟이 안 보일 때 SEEK가 군집 중심 대신 이 좌표로 시야를 유도
        _axp, _ayp = _ctr(ad_best)
        _arb, _acb = cell_of[at]
        _nc = {
            tok.strip(): (_axp + (ci - _acb) * pitch_x,
                          _ayp + (ri - _arb) * pitch_y)
            for ri, row in enumerate(rows)
            for ci, tok in enumerate(row) if tok.strip()}
        # ② 점프 가드: 약한/오독 앵커가 이긴 프레임에 유도 좌표가 100px+ 점프해
        # 로봇을 엉뚱한 곳으로 끌던 것(y-119px 점프 실측, 16:40:35) 방지 —
        # 큰 점프는 연속 2회 같은 곳을 가리킬 때만 수용, 아니면 이전 좌표 유지
        _oldc = getattr(self, "_map_cells", None)
        _accept = True
        if _oldc and time.time() - getattr(self, "_map_cells_ts", 0) < 12.0:
            _shared = set(_nc) & set(_oldc)
            if _shared and max(
                    abs(_nc[t][0] - _oldc[t][0]) + abs(_nc[t][1] - _oldc[t][1])
                    for t in _shared) > 100.0:
                _pd = getattr(self, "_cells_pending", None)
                _accept = bool(_pd and all(
                    t in _nc and abs(_nc[t][0] - _pd[t][0])
                    + abs(_nc[t][1] - _pd[t][1]) < 60.0 for t in _pd))
                if not _accept:
                    self._cells_pending = _nc
        if _accept:
            self._map_cells = _nc
            self._map_cells_ts = time.time()
            self._cells_pending = None
        # 4) 아직 정체 없는 셀에 합성 탐지 부여 (진짜 관측은 건드리지 않음)
        # 강등(suspect)된 판독은 이름 선점권 없음 — 진짜 자리에 맵 라벨이 붙게
        have = {d.get("text", "").strip() for d in detections
                if not d.get("suspect")}
        out = list(detections)
        added = []
        for bd, tok in matched:
            if tok in have or bd.get("text", "").strip() == tok:
                continue
            synth = dict(bd)
            synth["text"] = tok
            synth["belief"] = max(0.5, min(0.75, bd.get("belief", 0.3)))
            out.append(synth)
            have.add(tok)
            added.append(tok)
        if added and time.time() - getattr(self, "_map_log_ts", 0) > 5.0:
            self._map_log_ts = time.time()
            self._dlog(f"[MAP] 배치 정합 (앵커 '{at}') → {', '.join(added)} 위치 확정")
        return out

    def _on_image(self, msg):
        try:
            raw   = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # 스냅샷용 — 회전 적용 "전" 센서 네이티브(참조, 복사 아님). camera_info의 K는
            # 센서 원본 기준이라 이미지도 원본이어야 픽셀↔K가 일치(#P6). 표시/OCR/서보용
            # 회전 프레임(_raw_full, 아래)과는 별개 경로 — 그쪽은 그대로 유지.
            self._last_grip_raw = raw
            self._last_grip_stamp = msg.header.stamp
            self._last_grip_frame_mono = time.monotonic()   # 카메라 미수신 감지용
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

    def _infer_note(self, ms, boxes):
        """추론 1회 소요를 /status에 싣고, 약 10초마다 창 집계를 한 줄 남긴다.

        추론 소요 T는 지금까지 코드 어디에도 안 찍혔다 — _last_infer는 시각만
        갱신하고 로그도 /status도 없었다. T를 모르는 채로 T에 영향 주는 손잡이
        (사진모드·스레드 상한·격리)를 돌리면 이후 판단이 전부 추측이 된다.
        박스 수를 같이 싣는 이유: predict()는 검출 1회 + 박스마다 OCR 1회라
        T가 박스 수에 비례한다. 빈 화면 T와 캐빈 T는 다른 수치다."""
        with state_lock:
            state["infer_ms"]    = int(round(ms))
            state["infer_boxes"] = boxes
        w = getattr(self, "_infer_win", None)
        if w is None:
            w = self._infer_win = {"t0": time.monotonic(), "n": 0,
                                   "sum": 0.0, "max": 0.0, "box": 0}
        w["n"]   += 1
        w["sum"] += ms
        w["box"] += boxes
        w["max"]  = max(w["max"], ms)
        now = time.monotonic()
        if now - w["t0"] >= INFER_LOG_PERIOD:
            self._dlog(f"[INFER] {w['n']}회 — 평균 {w['sum'] / w['n']:.0f}ms · "
                       f"최대 {w['max']:.0f}ms · 평균 박스 {w['box'] / w['n']:.1f}개")
            self._infer_win = {"t0": now, "n": 0, "sum": 0.0, "max": 0.0, "box": 0}

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
            _t_infer   = time.monotonic()
            detections = infer_image(tmp)
            _infer_ms  = (time.monotonic() - _t_infer) * 1000.0
            _infer_n   = len(detections)   # 필터 전 원본 개수 — T의 실제 비용 단위
            os.unlink(tmp)
            self._infer_note(_infer_ms, _infer_n)

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
                for d in detections:
                    if d["text"] != tgt0:
                        continue
                    ref = self._refine_circle(ocr_input, d["box"])
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
            # ③ depth 유효 박스 3개+면 미측정 박스 제외 — 원거리 오탐이 군집 중심을
            # 끌어당기던 것 방지 (전반 실패 시엔 전부 유지)
            _pb_ok, _pb_no = [], []
            for d in raw_dets:
                _b = d["box"]
                _z = self._depth_at(int((_b["x1"] + _b["x2"]) / 2),
                                    int((_b["y1"] + _b["y2"]) / 2))
                (_pb_ok if _z is not None else _pb_no).append(_b)
            self._panel_boxes = _pb_ok if len(_pb_ok) >= 3 else (_pb_ok + _pb_no)
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
            # ── 문 심볼 별칭: ◄► 를 OCR이 '<>','><' 등으로 잘 읽음 (실측 belief 0.90).
            # 차내 배치에 문열림이 있으면 최고 belief 심볼 판독을 문열림 후보로 승격 —
            # 합성 의존을 벗어나 직접 잠금·앵커 가능. 위치가 배치와 모순이면
            # 아래 맵의 강등 검증이 걸러줌 (안전장치 동일 적용)
            if place0 == "cab" and any("문열림" in r for r in _layout_rows) \
                    and not any(d.get("text") == "문열림" for d in detections):
                _door = [d for d in raw_dets
                         if d.get("text", "").strip() in ("<", ">", "<>", "><")
                         and d.get("score", 0) >= 0.4]
                if _door:
                    _alias = dict(max(_door, key=lambda d: d.get("belief", 0)))
                    _alias["text"] = "문열림"
                    detections = detections + [_alias]

            # ── 버튼 맵: 차내에서 배치 사전지식으로 안 읽힌 버튼의 정체 확정 ──
            detections = self._apply_button_map(detections, raw_dets, place0)

            # ── 타겟 원형 스냅 (출처 무관): 모양 정합·합성 박스는 1단계 탐지기의
            # raw 박스라 점자·라벨까지 포함해 중심이 위로 치우침 — 실판독 타겟만
            # ROI 경로에서 스냅되던 것을 최종 타겟에 항상 적용.
            # (2026-07-21 실측: 모양 추론 '4' press가 버튼 살짝 위 빈 곳을 누름.
            #  같은 세션 실판독 '5'는 정확 — 스냅 유무가 유일한 차이였음)
            if tgt0:
                for d in detections:
                    if d.get("text") != tgt0 or d.get("suspect"):
                        continue
                    ref3 = self._refine_circle(frame, d["box"])
                    if ref3:
                        rcx3, rcy3, rr3 = ref3
                        _oldcy = (d["box"]["y1"] + d["box"]["y2"]) / 2
                        d["box"] = {"x1": rcx3 - rr3, "y1": rcy3 - rr3,
                                    "x2": rcx3 + rr3, "y2": rcy3 + rr3}
                        if d.get("shape") and abs(rcy3 - _oldcy) > 4 and \
                                time.time() - getattr(self, "_snap_log_ts", 0) > 8.0:
                            self._snap_log_ts = time.time()
                            self._dlog(f"[SNAP] '{tgt0}' 합성 박스를 버튼 원 중심으로 "
                                       f"교정 (y{rcy3 - _oldcy:+.0f}px)")

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
                cands = [d for d in detections
                         if d["text"] == tgt0 and not d.get("suspect")]
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
                    with state_lock:
                        state["lock_shape"] = bool(chosen.get("shape"))
                    if _new_lock:
                        # 잠금 대상까지의 depth를 남김 — 원거리 오탐(복도 건너편 물체)에
                        # 잠긴 것인지 로그만으로 판별 가능하게. 모양 추론 근거면 명시
                        _zl = self._depth_at(int(self._target_lock["cx"]),
                                             int(self._target_lock["cy"]))
                        self._dlog(f"[LOCK] '{tgt0}' 잠금 생성 (belief {new_b:.2f}, "
                                   f"depth {f'{_zl:.2f}m' if _zl else '?'}"
                                   f"{', 모양 추론' if chosen.get('shape') else ''})")
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

            # ── [OCR] 인식 라운드 구조적 로그 (1Hz 스로틀) — 사후 수치분석용 블랙박스 ──
            # "그때 후보가 뭐였고 belief가 얼마였나"를 파일에 남긴다(_dlog→_diaglog).
            # 2↔3 오인 순간 두 후보의 belief 차이·잠금이 옆 버튼으로 점프했는지가
            # 한 줄로 보이게. 파일 쓰기는 _dlog가 state_lock 밖에서 수행(인식 스레드=백그라운드).
            if time.time() - getattr(self, "_ocrlog_ts", 0) > 1.0:
                self._ocrlog_ts = time.time()
                try:
                    with state_lock:                     # 값 읽기만(디스크 I/O 없음)
                        _tgt   = state["target_text"]
                        _tdL   = state["target_dist"]
                        _liftL = state["lift"]
                        _extL  = state["arm_ext"]
                        _lshp  = state.get("lock_shape")
                    # 후보 전부(선택 안 된 동명이인/타 후보 포함): text:belief@(cx,cy)
                    #   ~=coast(비신선) S=모양추론 X=배치모순 강등(suspect)
                    _parts = []
                    for _d in detections:
                        _b = _d.get("box", {})
                        _cx = int((_b.get("x1", 0) + _b.get("x2", 0)) / 2)
                        _cy = int((_b.get("y1", 0) + _b.get("y2", 0)) / 2)
                        _fl = ""
                        if not _d.get("fresh", True): _fl += "~"
                        if _d.get("shape"):           _fl += "S"
                        if _d.get("suspect"):         _fl += "X"
                        _fl = f"[{_fl}]" if _fl else ""
                        _parts.append(f"{_d.get('text','?')}:{_d.get('belief',0):.2f}"
                                      f"@({_cx},{_cy}){_fl}")
                    # 위치 잠금: 잠금이 옆 버튼으로 점프했는지 추적
                    _lk = getattr(self, "_target_lock", None)
                    if _lk:
                        _lks = (f"({int(_lk['cx'])},{int(_lk['cy'])},"
                                f"b{_lk.get('bel', 0):.2f})" + ("S" if _lshp else ""))
                    else:
                        _lks = "none"
                    # 타겟 서보오차 ex/ey (상태창 1214행과 동일 계산식) + dead-zone
                    _exy = ""
                    _oxL, _oyL = _aim_offsets(_tdL)
                    _dzL = _dead_zone_px(_tdL)
                    if _tgt:
                        _dt = next((d for d in detections
                                    if d.get("text") == _tgt and not d.get("suspect")),
                                   None)
                        if _dt:
                            _bt = _dt["box"]
                            _ex = int((_bt["x1"] + _bt["x2"]) / 2 - (CX + _oxL))
                            _ey = int((_bt["y1"] + _bt["y2"]) / 2 - (CY + _oyL))
                            _exy = f" ex={_ex:+d} ey={_ey:+d} dz={_dzL}"
                    _distS = f"{_tdL:.2f}" if _tdL is not None else "?"
                    _liftS = f"{_liftL:.2f}" if _liftL is not None else "?"
                    _extS  = f"{_extL:.2f}" if _extL is not None else "?"
                    self._dlog(f"[OCR] tgt='{_tgt}' dets=[{', '.join(_parts)}] "
                               f"lock={_lks} dist={_distS} lift={_liftS} "
                               f"ext={_extS}{_exy}")
                except Exception:
                    pass

        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            self._processing = False

    @staticmethod
    def _refine_circle(img, bb):
        """버튼 원 테두리를 Hough로 찾아 박스를 원 중심·크기에 스냅.
        img는 640x480 (표시 프레임 또는 확대 OCR 입력 — 둘 다 동일 크기)."""
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
            if d.get("suspect"):
                color = (128, 128, 128)   # 강등 판독은 회색 (잠금 후보 아님)
            # 유지(coast) 중인 박스는 얇게 + 라벨에 ~ 접두사
            cv2.rectangle(vis, (x1,y1), (x2,y2), color,
                          (3 if is_target else 2) if fresh else 1)
            # 모든 인식 박스에 거리 표시 (depth 패치 중앙값 — 비용 무시 가능 수준)
            # → 선택(서보 발동) 없이도 depth 검증 가능
            dist = self._depth_at(bcx, bcy)
            dist_txt = f"{dist:.2f}m" if dist else "?m"
            # 라벨 정리: 목표만 상세(점수/거리), 나머지는 숫자만 작게 — 화면 가림 방지
            # 한글 별칭: cv2는 한글 폰트를 못 그려 '?'로 깨짐 → 오버레이만 영문 표기
            _disp = _OVERLAY_ALIAS.get(d["text"], d["text"])
            if d.get("suspect"):
                _disp = "x" + _disp   # 배치 모순으로 강등된 판독 표시
            elif d.get("shape"):
                _disp = "?" + _disp   # 모양 추론 라벨 (글자 미확인 — 배치 패턴 근거)
            if is_target:
                label, fs, bh = f"{'~' if not fresh else ''}{_disp} {d['score']:.2f} {dist_txt}", 0.5, 20
            else:
                label, fs, bh = f"{'~' if not fresh else ''}{_disp}", 0.45, 15
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
        if not _authority_ok():
            return   # 제어권 없음 — 서보·스캔·자동접근 전부 침묵 (관찰만)
        with state_lock:
            target = state["target_text"]
            tdist  = state["target_dist"]
            _tilt  = state["wall_tilt"]
            _flat  = state["wall_flat"]

        # ── 자동 수직 정렬: 벽 기울기(평면 검증 통과)가 4° 넘으면 그만큼 제자리 회전 ──
        # tilt<0(왼쪽이 멂) → 시계방향("오른쪽으로") / tilt>0 → 반시계. 재측정 반복 수렴.
        _rot_ok = False
        with state_lock:
            _ext_rot = state["arm_ext"]
        if (_flat and _tilt is not None and abs(_tilt) > 4.0
                and not self._nudging
                and not self._is_press_ready()   # 클릭 대기 중엔 회전 금지 (동결)
                # 접근 시작 후(팔 뻗음)에도 회전 금지 — 회전은 팔을 접으므로 접근을
                # 통째로 버리게 됨. READY 직후 CENTERED 한 프레임 깜빡임을 회전이
                # 비집고 들어와 동결을 뚫던 실측 사례(10:26:45) 차단
                and (_ext_rot is None or _ext_rot < 0.05)
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
        det = next((d for d in detections
                    if d["text"] == target and not d.get("suspect")), None)
        # ── [최소 박스 추적] 라벨이 이 프레임에 없을 때만, 잠금 위치에서 가장
        # 가까운 버튼 박스를 그 버튼으로 간주해 조준 공백을 메움 — 라벨 깜빡임
        # (모양 추론 프레임 변동)이 READY를 풀고 재정렬시키던 것 완화.
        # 안전 제약: ①잠금 수명(ts)은 안 건드림 — 신원 유효기간 연장은 라벨만 가능
        # ②반경 = 버튼 폭 이내 (버튼 간격의 절반 미만 → 옆 버튼으로 못 뜀)
        # ③패널 박스가 신선(2.5s)할 때만. 실패 시 기존 동작 그대로.
        if det is None:
            _lk_tr = getattr(self, "_target_lock", None)
            _pbs_tr = getattr(self, "_panel_boxes", [])
            if (_lk_tr and time.time() - _lk_tr["ts"] < 20.0
                    and _pbs_tr
                    and time.time() - getattr(self, "_panel_ts", 0) < 2.5):
                _bw_tr = float(np.median([b["x2"] - b["x1"] for b in _pbs_tr]))
                _nb, _nd = None, 1e9
                for _b in _pbs_tr:
                    _d0 = (abs((_b["x1"] + _b["x2"]) / 2 - _lk_tr["cx"])
                           + abs((_b["y1"] + _b["y2"]) / 2 - _lk_tr["cy"]))
                    if _d0 < _nd:
                        _nb, _nd = _b, _d0
                if _nb is not None and _nd <= max(50.0, _bw_tr):
                    det = {"box": dict(_nb), "text": target,
                           "belief": _lk_tr.get("bel", 0.5),
                           "fresh": True, "track": True}
                    _lk_tr["cx"] = (_nb["x1"] + _nb["x2"]) / 2
                    _lk_tr["cy"] = (_nb["y1"] + _nb["y2"]) / 2
                    if time.time() - getattr(self, "_track_log_ts", 0) > 8.0:
                        self._track_log_ts = time.time()
                        self._dlog(f"[TRACK] '{target}' 라벨 공백 — "
                                   "잠금 박스 연속 추적으로 조준 유지")
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
                lk_fr = getattr(self, "_target_lock", None)
                if lk_fr is not None:
                    lk_fr["ts"] = time.time()   # 동결 중 잠금 수명 연장 —
                    # 로봇 정지 = 버튼 위치 불변. 잠금이 만료되면 다음 모양 정합
                    # 프레임이 엉뚱한 버튼에 새 잠금을 만들어 조준이 점프한다
                    # (2026-07-21 ⑤ 실측: 잠금 재생성 반복 → READY 동결 반복 해제)
                return
            # ── 판독 없는 접근 지속: CENTERED 래치 + 잠금 20초 이내면 남은 접근은
            # depth만으로 진행. 문 심볼처럼 판독이 드문(1분/회 실측) 타겟이
            # "한 스텝 가고 정지"하던 것 해결 — 판독 1회로 READY까지 완주 가능
            with state_lock:
                _cent_hold = state["centered"]
            _lk_c = getattr(self, "_target_lock", None)
            if (_cent_hold and _lk_c and time.time() - _lk_c["ts"] < 20.0
                    and self._goal_done and not self._nudging
                    and time.time() - getattr(self, "_approach_sent", 0) > 1.5):
                _oxc, _oyc = _aim_offsets(tdist)
                _dlc = self._depth_at(CX + _oxc, CY + _oyc)
                # ① 접근 완료 시 READY 래치 — 판독 없는 마지막 구간에서도 개찰구가
                # 열리게 (0.24m 도달하고도 READY 미발동으로 완주 실패하던 것, 16:40 실측).
                # 동결 덕에 조준은 CENTERED 시점 그대로고, 클릭 시 재검증은 유지됨.
                if _dlc and _dlc <= PRESS_READY_DIST:
                    with state_lock:
                        _was_r2 = state.get("press_ready", False)
                        state["press_ready"] = True
                    if not _was_r2:
                        self._dlog(f"[READY] 누르기 활성 ({_dlc:.2f}m, 잠금 유지 접근 완료) — "
                                   "자동 이동 전체 동결, 클릭 대기")
                    return
                if _dlc and _dlc > PRESS_CLOSE_DIST + 0.03:
                    with state_lock:
                        _extc = state["arm_ext"]
                    if _extc is not None and float(_extc) < ARM_EXT_MAX - 0.005:
                        _stepc = max(0.02, min(0.06,
                                     0.35 * (_dlc - PRESS_CLOSE_DIST)))
                        _stepc = min(_stepc, _dlc - PRESS_CLOSE_DIST)
                        self._approach_sent = time.time()
                        self._dlog(f"[APPROACH] 자동 접근(depth, 잠금 유지) "
                                   f"+{_stepc*100:.0f}cm (버튼 {_dlc:.2f}m)")
                        self._send_goal([ARM_JOINT],
                            [max(ARM_EXT_MIN, min(ARM_EXT_MAX,
                                                  float(_extc) + _stepc))])
                        return
            # [진동 방지] 방금(4초 내)까지 그 자리에서 타겟을 봤으면 — 움직이지 말고
            # 재인식을 기다린다. (정렬 위치에서 일시적으로 안 읽힐 때, 군집 추적이
            # 시야를 끌고 내려가 "내려가면 읽히고 올라가면 놓치는" 무한 왕복 방지)
            lk_hold = getattr(self, "_target_lock", None)
            if lk_hold and time.time() - lk_hold["ts"] < 4.0:
                return

            # ── 맵 유도 탐색: 타겟이 배치에 있고 맵 정합이 신선하면, 군집 중심이
            # 아니라 "맵이 예측한 타겟 위치"로 시야를 유도. 가장자리 버튼(문열림 등)
            # 에서 군집 추적과 타겟 서보가 반대로 끌던 줄다리기(예산 2회 소진 실측,
            # 15:55~15:57) 제거 — 두 컨트롤러가 같은 지점을 향하게 함.
            _mc = getattr(self, "_map_cells", None)
            if (_mc and target in _mc
                    # 신선도 12초: 앵커 판독 주기(5~10초 실측)보다 길게 — 3초였을 땐
                    # 판독 사이 공백에 군집 SEEK가 하이재킹해서 문열림을 놓침 (16:29 실측).
                    # 12초 넘게 정합이 끊기면(진짜 실종) 군집 SEEK가 재획득 담당.
                    # 그 사이 이동은 지글(±22px)·정렬 스텝(±35px)뿐이라 ±45px 관용 내.
                    and time.time() - getattr(self, "_map_cells_ts", 0) < 12.0):
                gx, gy = _mc[target]
                sex_m, sey_m = gx - CX, gy - CY
                if abs(sey_m) < 45 and abs(sex_m) < 45:
                    # 예상 위치 도달. 그냥 기다리면 정반사 각도에 얼어붙어 판독이 안
                    # 나옴(1분/회 실측) → 인식 부스트 지글: lift ±1.2cm 교대로 반사
                    # 각도를 계속 바꿔 판독 유도. 탐색 중재자(_may_explore)가
                    # "이동→정지→인식" 리듬을 보장, 폭이 작아 맵 유도와 충돌 없음
                    if time.time() - getattr(self, "_mapwait_log_ts", 0) > 5.0:
                        self._mapwait_log_ts = time.time()
                        self._dlog(f"[SEEK] '{target}' 예상 위치 도달 — 재인식 대기 중")
                    if self._may_explore():
                        self._jiggle_dir = -getattr(self, "_jiggle_dir", -1)
                        _js = 0.012 * self._jiggle_dir
                        self._dlog(f"[SEEK] 인식 부스트 지글 (lift {_js*100:+.1f}cm)")
                        self._send_goal(["joint_lift"],
                                        [max(0.15, min(1.10, float(lift) + _js))])
                    return
                if not self._may_explore():
                    return
                if abs(sey_m) >= 45:
                    self._dlog(f"[SEEK] 맵 예상 위치로 높이 이동 ('{target}', y{sey_m:+.0f}px)")
                    self._send_goal(["joint_lift"],
                                    [max(0.15, min(1.10, float(lift) - KP_LIFT * sey_m))])
                else:
                    with state_lock:
                        _ba_m = state["base_align"]
                    if _ba_m:
                        # 넛지가 거부돼도(이동 중 등) 로그만 반복되던 스팸 방지
                        if time.time() - getattr(self, "_seekx_log_ts", 0) > 4.0:
                            self._seekx_log_ts = time.time()
                            self._dlog(f"[SEEK] 맵 예상 위치로 전후 이동 "
                                       f"('{target}', x{sex_m:+.0f}px)")
                        self._maybe_base_nudge(sex_m, None)
                return

            # 타겟 미인식 → "버튼처럼 생긴 것들"(비숫자 포함) 군집 = 패널 후보 중심으로
            # 시야를 옮김 (집요 추적). 글자를 못 읽는 원거리에서도 작동.
            boxes = list(getattr(self, "_panel_boxes", []))
            if time.time() - getattr(self, "_panel_ts", 0) > 2.5:
                boxes = []

            # ── 패널 시야 가드: 배치 타겟 + 박스 3개 이상 보이면 군집의 "옆으로
            # 끌기"(높이/전후 중심 맞추기) 금지. 판독 가뭄이 아무리 길어도 예상
            # 위치를 지키며 ①멀면 전진(판독 거리 확보) ②가까우면 지글만 한다.
            # 군집 추적은 패널을 정말 잃었을 때(박스<3)만 재획득용으로 발동.
            # (줄다리기로 예산 16cm 소진 반복 실측 — 시간 창 연장으로도 못 막던 것)
            _lay_toks = {tok for r in _layout_rows for tok in r if tok.strip()}
            if target in _lay_toks and len(boxes) >= 3:
                # 탈출구: 정합이 30초+ 죽어 있으면 (판독 가뭄 — 지글도 무성과)
                # "잘못된 자리를 성실히 지키는" 교착이므로, 군집 재센터링을 1회
                # 허용해 앵커가 읽히는 시야로 복귀시킨다 (30초에 1회 스로틀 —
                # 줄다리기 회귀 방지). '5' 선택 후 3분 지글 교착 실측(18:05~08).
                if (time.time() - getattr(self, "_map_cells_ts", 0) > 30.0
                        and time.time() - getattr(self, "_guard_esc_ts", 0) > 30.0):
                    self._guard_esc_ts = time.time()
                    self._dlog("[SEEK] 자리 지킴 무성과 30초 — 군집 재센터링 1회 허용")
                    # return 없이 아래 군집 블록으로 통과 → 재센터링 1회
                else:
                    if time.time() - getattr(self, "_panelwait_log_ts", 0) > 5.0:
                        self._panelwait_log_ts = time.time()
                        self._dlog(f"[SEEK] 패널 시야 내 — '{target}' 자리 지킴 (전진/지글만)")
                    if self._may_explore():
                        _wd = self._depth_at(CX, CY)
                        with state_lock:
                            _ext_g = state["arm_ext"]
                        if (_wd and _wd > 0.35 and not self._nudging
                                and _ext_g is not None
                                and float(_ext_g) < ARM_EXT_MAX - 0.005):
                            _st_g = min(0.04, _wd - 0.30)
                            self._approach_sent = time.time()
                            self._dlog(f"[APPROACH] 판독 거리 확보 전진 +{_st_g*100:.0f}cm "
                                       f"(벽 {_wd:.2f}m)")
                            self._send_goal([ARM_JOINT],
                                [max(ARM_EXT_MIN, min(ARM_EXT_MAX,
                                                      float(_ext_g) + _st_g))])
                        else:
                            self._jiggle_dir = -getattr(self, "_jiggle_dir", -1)
                            _js3 = 0.012 * self._jiggle_dir
                            self._dlog(f"[SEEK] 인식 부스트 지글 (lift {_js3*100:+.1f}cm)")
                            self._send_goal(["joint_lift"],
                                            [max(0.15, min(1.10, float(lift) + _js3))])
                    return

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
                    # 몸체이동 OFF/X정렬 소진이면 어차피 못 움직임 — 헛로그 스팸 방지
                    with state_lock:
                        _ba_seek = state["base_align"]
                    if not _ba_seek:
                        if time.time() - getattr(self, "_seek_off_ts", 0) > 10.0:
                            self._seek_off_ts = time.time()
                            self._dlog("[SEEK] 전후 이동 필요하나 몸체이동 OFF — 생략")
                        return
                    # (SEEK 회수 상한은 제거 — 사용자 결정 2026-07-13. 가드에 거부된
                    # 시도까지 세서 3번 만에 로봇이 굳던 부작용. 과도 이동 방지는
                    # 16cm 마스터 예산 + 타겟당 리셋이 담당)
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
            with state_lock:
                _was_ready = state.get("press_ready", False)
            # 히스테리시스: 진입 0.25 / 유지 0.28 — depth 지터로 동결이 풀렸다
            # 잠기며 READY 후에도 접근이 한 번 더 나가던 것(11:24:10 실측) 방지
            if tdist is not None and \
                    tdist <= PRESS_READY_DIST + (0.03 if _was_ready else 0.0):
                with state_lock:
                    state["press_ready"] = True
                if not _was_ready:
                    self._dlog(f"[READY] 누르기 활성 ({tdist:.2f}m) — "
                               "자동 이동 전체 동결, 클릭 대기")
                return
            if _was_ready:
                with state_lock:
                    state["press_ready"] = False
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
                # 침묵 정체 방지: depth 구멍이면 접근이 조용히 멈추던 것 — 이유를 로그로
                # (문열림 CENTERED 후 20초 무로그 정체 실측, 2026-07-13 16:07)
                if d_live is None and \
                        time.time() - getattr(self, "_dhole_log_ts", 0) > 5.0:
                    self._dhole_log_ts = time.time()
                    self._dlog("[APPROACH] 조준점 depth 측정불가 — 접근 대기 (구멍/반사 의심)")
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
        # READY 동결 중의 "큰" 오차 점프(3×dz 또는 40px+)는 정렬 이탈이 아니라
        # 오인식 — 로봇이 정지해 있는데 버튼이 그만큼 움직일 수 없다 (라벨이
        # 옆 버튼으로 점프한 것). 이 프레임은 무시하고 동결 유지.
        # (2026-07-21 ⑤ 실측: 모양 정합 재배정 → READY↔재정렬 5회 반복, lift 꿈틀)
        # 진짜 어긋난 경우엔 클릭 시 라이브 재검증이 press를 거부하므로 안전.
        if self._is_press_ready() and \
                (abs(ex) > max(3 * dz, 40) or abs(ey) > max(3 * dz, 40)):
            if time.time() - getattr(self, "_jump_log_ts", 0) > 5.0:
                self._jump_log_ts = time.time()
                self._dlog(f"[READY] 라벨 점프 무시 (x{ex:+.0f} y{ey:+.0f}px) — "
                           "동결 유지 (오인식으로 판단)")
            return
        # CENTERED는 래치가 아님 — 히스테리시스(+4px) 넘게 벗어나면 해제 후 재정렬
        with state_lock:
            was_centered = state["centered"]
            if was_centered and (abs(ex) > dz + 4 or abs(ey) > dz + 4):
                state["centered"] = False
                state["press_ready"] = False   # 정조준이 깨지면 READY 래치도 해제
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
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            # ★_last_motion_ts는 '실제로 보낸' 경우에만 갱신 — 실패해도 갱신하면
            # _may_explore가 '안 움직였는데 움직인 걸로' 보고 쿨다운을 먹는다.
            self._dlog(f"[MOVE] ⛔ 액션서버 없음(2.0s) — 관절이동 유실: {joint}")
            return False
        self._last_motion_ts = time.time()   # [A안] 이동 발생 기록 — 전송 성공 경로에서만
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
            # fut.result() 무가드였음 — 예외가 나면 rclpy.spin()이 KeyboardInterrupt만
            # 잡으므로 프로세스가 통째로 죽는다(스핀사망 계열). 반드시 감싼다.
            # 예외 시 ok["v"]는 False 그대로 두고 done만 세워 대기루프를 푼다(=실패 반환).
            try:
                h = fut.result()
            except Exception as e:
                self._dlog(f"[MOVE] ⛔ 응답 예외 — 관절이동 실패 처리: {e!r}")
                done.set()
                return
            if h.accepted:
                h.get_result_async().add_done_callback(on_result)
            else:
                self._dlog(f"[MOVE] ⛔ 거부됨 — 관절이 움직이지 않음: {joint}")
                done.set()

        self.action_client.send_goal_async(goal).add_done_callback(on_resp)
        done.wait(timeout)
        return ok["v"]

    def start_press(self) -> str | None:
        """누르기 시퀀스 시작. 문제 있으면 사유 문자열, 정상 시작이면 None."""
        if not _authority_ok():
            return "제어권 없음 — 대시보드(8080)에서 엘리베이터 제어권을 부여하세요"
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
            if x.get("text") != tgt or x.get("suspect"):
                return False
            if x.get("age", 9.9) < 2.0:
                return True
            return (x.get("age", 9.9) < 8.0 and e_pm is not None
                    and e_pm["ts"] > lm_p + 0.3)
        det = next((x for x in dets if _obs_ok(x)), None)
        # 모양 추론 근거 press는 분석용으로 명시 기록 (차단은 안 함 — 사용자 결정).
        # 특히 '종'(비상벨)은 오배치 시 대가가 커서 반드시 로그에 남긴다
        if det is not None and det.get("shape"):
            self._dlog(f"[PRESS] {'⚠ ' if tgt == '종' else ''}'{tgt}' press 근거가 "
                       "모양 추론 (글자/심볼 미확인) — 화면 십자 위치 확인 권장")
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
        with state_lock:
            _ready_latched = state.get("press_ready", False)
        if not _ready_latched and d > PRESS_READY_DIST:
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
            direct = d <= PRESS_READY_DIST + 0.05   # READY 유지 문턱(0.28)까지 포함
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
                # return이 없으면 아래 접촉감지로 그대로 진입 — 팔이 안 움직였으니
                # actual=누르기 전 위치 → shortfall이 커서 "막힘=닿음"으로 오판하고
                # "✅ 누르기 완료"를 잘못 선언한다(뿌리B). 다른 실패분기와 동일하게 복귀+중단.
                st("❌ 누르기 실패 — 복귀")
                self._dlog("[PRESS] ⛔ 누르기 실패 — 이동 안 됨(사용자 미도달)")
                self._move_joint_wait(ARM_JOINT, start_ext, 4, 12.0)
                return
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
                state["press_ok_ts"] = time.time()   # 여정 단계 순차 해제 근거
                state["target_text"] = None
                state["phase"]       = "SELECT"
                state["centered"]    = False
        finally:
            with state_lock:
                state["pressing"] = False

    def _send_single_joint(self, joint_name: str, position: float, duration_sec: int = 2) -> bool:
        """단일 관절 목표 전송. 전송 성공 True / 액션서버 없어 유실되면 False.
        (_send_goal과 같은 정직화 계약 — 조용한 실패를 호출부가 알 수 있게 bool 반환)"""
        self._last_motion_ts = time.time()   # [A안] 수동 조작도 이동 — 관측 기억 무효화
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self._dlog(f"[GOAL] ⛔ 액션서버 없음(1.0s) — 명령 유실: {joint_name}")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint_name]
        pt = JointTrajectoryPoint()
        pt.positions = [position]
        pt.time_from_start.sec = duration_sec
        goal.trajectory.points = [pt]
        self.action_client.send_goal_async(goal)
        self._dlog(f"{joint_name} -> {position:.2f} rad")
        return True

    def set_wrist_pitch(self, pitch: float) -> bool:
        return self._send_single_joint("joint_wrist_pitch", pitch)

    def set_lift(self, lift: float) -> bool:
        return self._send_single_joint("joint_lift", lift, duration_sec=3)

    def set_gripper(self, aperture_m: float) -> bool:
        return self._send_single_joint(GRIPPER_JOINT, aperture_m, duration_sec=2)

    def set_wrist_yaw(self, yaw: float) -> bool:
        return self._send_single_joint("joint_wrist_yaw", yaw)

    def _send_goal(self, joint_names, positions) -> bool:
        """관절 목표 전송. 전송 성공 True / 액션서버 없어 유실되면 False.
        ★_last_motion_ts는 '실제로 보낸' 경우에만 갱신 — 실패해도 갱신하면
        _may_explore가 '안 움직였는데 움직인 걸로' 보고 쿨다운을 먹는다."""
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self._dlog("[GOAL] ⛔ 액션서버 없음(1.0s) — 명령 유실: "
                       + ", ".join(str(n) for n in joint_names))
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]
        self._goal_done = False
        self._goal_ok   = True   # 응답 콜백이 거부/예외면 False로 내림
        self._last_motion_ts = time.time()   # [A안] 이동 발생 기록 — 전송 성공 경로에서만
        self.action_client.send_goal_async(goal).add_done_callback(self._on_goal_response)
        self._dlog("-> " + ", ".join(
            f"{n}={p:.3f}" for n, p in zip(joint_names, positions)))
        return True

    def _on_goal_response(self, fut):
        # fut.result() 무가드였음 — 예외가 나면 rclpy.spin()이 KeyboardInterrupt만
        # 잡으므로 프로세스가 통째로 죽는다(스핀사망 계열). 반드시 감싼다.
        try:
            h = fut.result()
        except Exception as e:
            self._goal_ok   = False
            self._goal_done = True   # 대기 루프가 영원히 안 풀리는 것 방지
            self._dlog(f"[GOAL] ⛔ 응답 예외 — 명령 실패 처리: {e!r}")
            return
        if h.accepted:
            h.get_result_async().add_done_callback(lambda _: setattr(self,"_goal_done",True))
        else:
            self._goal_ok   = False
            self._goal_done = True
            self._dlog("[GOAL] ⛔ 거부됨 — 로봇이 움직이지 않음")


def main():
    if not start_infer_server():
        print("추론 서버 시작 실패. 종료합니다.")
        return

    # 진단 로거 시작 + 부팅 스냅샷 (제어권 판정 이전 상태를 먼저 남긴다)
    global _diaglog
    if _diag is not None:
        try:
            _diaglog = _diag.DiagLogger("elevator")
            _diaglog.boot_snapshot({"cwd": os.getcwd()})
        except Exception as _le:
            _diaglog = None      # 로거 없어도 본체는 정상 동작해야 함
            print(f"[경고] robot_diag 로거 생성 실패 — 파일 로그 비활성: {_le}", flush=True)

    # 제어권 초기값: fail-closed — 기본은 False(부여 대기), 대시보드가 /authority로
    # 명시적으로 줄 때만 True가 된다. "대시보드 응답 없음 → 나 혼자 권한 보유" 같은
    # 자동판정은 하지 않는다(권한 없이 열리는 경로를 원천 차단). 사람이 명시적으로
    # 켜는 단독 개발 모드만 --standalone 플래그로 보존.
    _standalone = "--standalone" in _sys.argv
    with state_lock:
        state["authority"] = _standalone
        if _standalone:
            # 단독모드 = 대시보드 없음 = 하트비트 없음 → 워치독(리스 deadman) 면제.
            # inf는 /status에 노출 안 되는 내부값이라 JSON 직렬화에 안전.
            state["lease_deadline"] = float("inf")
    if _diaglog:
        _diaglog.log("AUTH", f"제어권 초기화: standalone={_standalone} → authority={_standalone}")
    print(("제어권: 단독 모드 (--standalone, 보유)" if _standalone
           else "제어권: 부여 대기 (8080에서 주세요)"), flush=True)

    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False),
        daemon=True,
    ).start()

    # 브라우저 자동 오픈 — standalone일 때만. 대시보드가 spawn한 경우엔 이미
    # iframe 탭 안에서 보이므로 별도 창을 띄우면 사용자가 원하는 "한 화면"이 깨짐.
    if _standalone:
        threading.Timer(1.5, lambda: webbrowser.open("http://localhost:5000")).start()
    print("대시보드: http://localhost:5000", flush=True)

    rclpy.init()
    node = ElevatorTracker()
    _node_ref[0] = node

    # 진단 계측 부착 (기존 노드에 진단용 구독/타이머만 덧붙임 — 로직 변경 없음)
    if _diag is not None and _diaglog is not None:
        _diag.attach(
            node, _diaglog,
            cameras={
                "grip_rect":  "/camera/gripper_camera/color/image_rect_raw",
                "grip_raw":   "/camera/gripper_camera/color/image_raw",
                "grip_old1":  "/gripper_camera/color/image_raw",
                "grip_old2":  "/gripper_camera/color/image_rect_raw",
                "grip_depth": "/camera/gripper_camera/aligned_depth_to_color/image_raw",
                "body":       "/camera/camera/color/image_raw",
            },
            cmd_vel_topic="/stretch/cmd_vel",
            authority_getter=lambda: state.get("authority"),
            phase_getter=lambda: state.get("phase"),
            own_node_name="elevator_tracker",
        )
        # SIGTERM(VSCode 정지 등)으로 죽여도 finally 정리가 돌게 흘려보냄 →
        # 잔재 감소 + "어떻게 죽었나" 기록
        _diag.install_signal_logging(_diaglog, reraise=True)

    # 리스 deadman — 순수 데몬 스레드(ROS 스핀과 운명 분리, 스핀 죽어도 감시 유지)
    threading.Thread(target=_lease_watchdog, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if _diaglog:
            _diaglog.log("EXIT", "spin 종료 → destroy_node 호출")
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass
        if _diaglog:
            _diaglog.log("EXIT", "destroy_node/shutdown 완료 — 정상 정리")


if __name__ == "__main__":
    main()
