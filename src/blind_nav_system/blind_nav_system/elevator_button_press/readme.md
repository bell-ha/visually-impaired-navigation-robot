# Elevator Button Press

Stretch SE3 로봇의 그리퍼 카메라(D405)로 엘리베이터 버튼을 인식하고,
웹 UI에서 목표 버튼을 클릭하면 로봇 팔이 자동으로 그 버튼을 카메라 중앙에 오도록 상하 이동하는 모듈.

---

## 구조 개요

```
elevator_button_press/
├── main.py               # ★ 메인 실행 파일 (Flask 웹 UI + ROS2 노드 통합)
├── elevator_node.py      # ROS2 노드 (카메라 이미지 → 버튼 인식 결과 퍼블리시)
├── visual_servo_node.py  # Visual servoing 노드
├── ocr_rcnn_server.py    # 영구 추론 서버 (모델 1회 로딩 후 stdin/stdout 통신)
├── ocr_rcnn_infer.py     # 단일 이미지 추론 스크립트 (테스트용)
├── ocr-rcnn-v2/          # OCR-RCNN v2 외부 repo (gitignore 처리)
└── results/              # 테스트 결과 이미지 저장 폴더 (gitignore)
```

> **참고:** 테스트 패널 배치 추론 스크립트(`run_test_panels.py`)는 `tools/` 폴더로 이동됨.

---

## 사용 모델: OCR-RCNN v2

- **역할**: 엘리베이터 버튼 패널에서 버튼의 위치(bounding box)와 숫자/문자를 동시에 인식
- **입력**: 640×480 RGB 이미지
- **출력**: 각 버튼의 `{text, score, belief, box(x1,y1,x2,y2)}`
- **frozen model 파일** (Google Drive에서 다운로드):
  - `detection_graph_640x480.pb` — 버튼 위치 감지 (Faster R-CNN 계열)
  - `ocr_graph.pb` — 버튼 숫자/문자 인식
- **실행 환경**: `blind_nav_system/venv/` 통합 가상환경 (TF 2.13 + `compat.v1` 패치, `--system-site-packages`로 ROS2 상속)

### 추론 속도 최적화

매 프레임마다 subprocess를 새로 띄우면 TF 모델 로딩에 수초가 걸림.
`ocr_rcnn_server.py`를 시작 시 1회만 띄우고, 이미지 경로를 stdin으로 전달 → JSON을 stdout으로 수신하는 방식으로 해결.

- **영상 표시**: 카메라 FPS 그대로 실시간
- **버튼 인식**: 백그라운드 스레드에서 추론, 완료될 때마다 바운딩박스 갱신

---

## 최초 설치 (1회만)

### 통합 venv 생성 및 패키지 설치

```bash
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system

python3 -m venv --system-site-packages venv
source /opt/ros/humble/setup.bash
source venv/bin/activate
pip install -r requirements.txt
```

### frozen model 다운로드 (gdown 사용)

```bash
# venv 활성화 상태에서 실행
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system/elevator_button_press/ocr-rcnn-v2/src/button_recognition/scripts/ocr_rcnn_lib/frozen_model
gdown <detection_graph_640x480.pb Google Drive ID>
gdown <ocr_graph.pb Google Drive ID>
```

---

## 실행 방법

### 터미널 1 — 로봇 드라이버

```bash
ros2 launch stretch_ros2_bridge stretch_robot_process.launch.xml
```

### 터미널 2 — 메인 스크립트

```bash
source /opt/ros/humble/setup.bash
source ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate

python3 ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system/elevator_button_press/main.py
```

시작 시 터미널에 아래가 출력되면 정상:
```
OCR 모델 로딩 중... (최초 1회, 수초 소요)
OCR 모델 준비 완료.
```

### 브라우저

```
http://localhost:5000
```

---

## 웹 UI 사용법

1. 브라우저에서 `http://localhost:5000` 접속
2. **카메라 선택** (상단 드롭다운):
   - `gripper (D405)` — 그리퍼 손목 카메라 (기본값, 버튼 인식용)
   - `body (D435i)` — 몸체 전방 카메라
3. 카메라 피드에 인식된 버튼이 초록 박스로 표시됨
4. 아래 버튼 목록에서 목표 버튼 클릭 → 추적 시작 (파란 배너)
5. 로봇 팔이 상하로 움직여 해당 버튼을 화면 중앙으로 이동
6. 배너가 초록 **CENTERED** 로 바뀌면 완료
7. **Reset (재선택)** 으로 다른 버튼 선택 가능

### 그리퍼 자세 조정 버튼

| 버튼/슬라이더 | 역할 |
|---|---|
| 그리퍼 전방 고정 | wrist_pitch=0.07, wrist_yaw=0.03 으로 복귀 |
| wrist_pitch 슬라이더 | 그리퍼 상하 각도 수동 조정 (-1.57 ~ 0.5 rad) |
| wrist_yaw 슬라이더 | 그리퍼 좌우 각도 수동 조정 (-2.88 ~ 1.67 rad) |

> **고정 자세값** (실측으로 결정): pitch = **0.07 rad**, yaw = **0.03 rad**  
> 이 값에서 그리퍼 D405 카메라가 엘리베이터 패널 정면을 향함.  
> 스크립트 시작 3초 후 자동으로 이 자세로 이동.

---

## 제어 방식 (Visual Servoing)

### 개요

버튼이 카메라 화면 중앙에 올 때까지 로봇 팔의 높이(`joint_lift`)를 반복 조정.

```
[카메라 프레임]
      │
      ▼
 OCR-RCNN 추론 → 버튼 bounding box (x1,y1,x2,y2)
      │
      ▼
 버튼 중심 Y좌표 - 화면 중심 Y(240) = error_y
      │
      ▼
 new_lift = current_lift - KP_LIFT × error_y
      │
      ▼
 /stretch_controller/follow_joint_trajectory (액션 서버)
```

### 제어 대상 관절

| 오차 방향 | 제어 관절 | 게인 | 설명 |
|---|---|---|---|
| Y (상하) | `joint_lift` | 0.0003 m/px | 버튼이 위에 있으면 팔 올림, 아래면 내림 |

- **wrist_pitch, wrist_yaw는 추적 중 절대 변경하지 않음**
- 좌우(X) 오차는 제어하지 않음 — 로봇이 엘리베이터 앞에 정위치된 상태 가정

### Dead Zone

오차가 ±40 px 이내이면 정지하고 CENTERED 상태로 전환.

### 관절 이동 범위 클램프

| 관절 | 최솟값 | 최댓값 |
|---|---|---|
| joint_lift | 0.15 m | 1.10 m |

---

## 의존성 구조

```
터미널 2: python3 main.py  (시스템 Python 3.10 + ROS2)
  ├─ Flask 웹 서버 → http://localhost:5000
  ├─ ROS2 ElevatorTracker 노드
  │    ├─ 구독: /gripper_camera/color/image_raw  (또는 D435i 토픽)
  │    ├─ 구독: /joint_states  (현재 lift 위치 파악)
  │    └─ 액션: /stretch_controller/follow_joint_trajectory
  └─ 영구 subprocess (시작 시 1회 기동)
       └─ blind_nav_system/venv/bin/python3 ocr_rcnn_server.py
            └─ OCR-RCNN v2 (TF 2.13 compat.v1)
                 ├─ detection_graph_640x480.pb
                 └─ ocr_graph.pb
```

---

## 테스트 이미지로 모델 단독 확인

실제 로봇 없이 모델 동작만 확인하려면:

```bash
source ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate

# 배치 테스트 (tools/ 폴더에 위치)
python3 ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system/tools/run_test_panels.py
# 결과 이미지: elevator_button_press/results/ 폴더에 저장
```

단일 이미지 추론:

```bash
python3 ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system/elevator_button_press/ocr_rcnn_infer.py --image /path/to/image.jpg
# JSON 출력 예:
# {"detections": [{"score": 0.98, "text": "3", "belief": 0.95, "box": {...}}]}
```
