# People Tracker — Human Direction Estimation

YOLOv8 + ByteTrack으로 사람을 검출·추적하고, 이동 방향을 추정해 RViz2 맵에 시각화하는 모듈.

---

## 전체 흐름

```
ROS2 머리 카메라 (RGB)
        │
        ▼
  [tracker.py]  YOLOv8n + ByteTrack
  사람 검출 + ID 부여 + 소형 bbox 필터
        │
        ├──────────────────────────────────────┐
        ▼                                      ▼
  [direction.py]                        [ros_publisher.py]
  픽셀 좌표 이력 → EMA 속도 추정        aligned_depth + TF2로
  이미지 기준 방향/각도 계산            픽셀 → 3D → map 좌표 변환
        │                                      │
        ▼                                      ▼
  [flow.py]                             /people_tracker/markers
  군중 전체 흐름(도미넌트 방향) 집계     RViz2 MarkerArray 퍼블리시
        │
        ▼
  [visualization.py]
  OpenCV 창: bbox, 화살표, FPS, 시크바
```

---

## 카메라 구성

| 카메라 | 위치 | /dev 경로 | 접근 방법 |
|--------|------|-----------|-----------|
| Intel RealSense D435 | 머리 | /dev/video0~5 | ROS2 토픽 (직접 열기 불가) |
| Intel RealSense D435 | (두 번째) | /dev/video8~13 | ROS2 토픽 |
| Arducam OV9782 | 그리퍼 | /dev/video6~7 | cv2.VideoCapture(6) 직접 사용 가능 |

**주의:** 머리 RealSense는 `realsense2_camera_node`가 장치를 점유하므로 cv2로 직접 열 수 없음. ROS2 토픽으로만 접근.

### 주요 토픽

| 토픽 | 용도 |
|------|------|
| `/camera/camera/color/image_raw` | RGB 프레임 (YOLO 입력) |
| `/camera/camera/aligned_depth_to_color/image_raw` | depth (color 픽셀과 1:1 정렬) |
| `/camera/camera/color/camera_info` | 카메라 내부 파라미터 (fx, fy, cx, cy) |
| `/people_tracker/markers` | 사람 위치·방향 마커 (출력) |

---

## 실행 방법

### 사전 조건

1. ROS2 Humble 설치
2. `people_tracker` 가상환경 준비 (아래 참고)

### 가상환경 세팅 (최초 1회)

```bash
cd .../people_tracker

# --system-site-packages 필수 (rclpy가 시스템에 있으므로)
python3 -m venv people_tracker --system-site-packages

source /opt/ros/humble/setup.bash
source people_tracker/bin/activate
pip install ultralytics opencv-python numpy
```

### 실행 순서

**터미널 1 — ROS2 스택 (로봇 전체)**
```bash
ros2 launch /home/hello-robot/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system/launch/stretch_robot_process.launch.xml
```

**터미널 2 — RViz2에서 초기 위치 설정**
- `2D Pose Estimate` 버튼으로 로봇 위치 지정 → AMCL 초기화 → `map` 프레임 활성화

**터미널 3 — People Tracker**
```bash
cd .../people_tracker
source /opt/ros/humble/setup.bash
source people_tracker/bin/activate
python3 main.py
```

### 실행 옵션

```bash
python3 main.py                  # 기본: 머리 RealSense (ROS2 토픽)
python3 main.py 6                # 그리퍼 Arducam 카메라 (ROS2 불필요)
python3 main.py video.mp4        # 영상 파일
python3 main.py --conf 0.6       # 신뢰도 임계값 조정 (기본 0.5)
python3 main.py --rotate 0       # 화면 회전 변경 (기본 90도)
python3 main.py --save out.mp4   # 결과 영상 저장
```

### 실행 중 키 조작

| 키 | 동작 |
|----|------|
| `r` | 시계 방향 90도 회전 (토글) |
| `Space` | 일시정지 / 재생 |
| `q` / `ESC` | 종료 |

---

## RViz2 설정

| 항목 | 값 |
|------|----|
| Fixed Frame | `map` |
| Add → MarkerArray | Topic: `/people_tracker/markers` |

마커 종류:
- **파란 실린더** — 사람 위치 (맵 좌표, 실제 미터 단위)
- **주황 화살표** — 이동 방향 (맵 좌표 기반 실제 속도)
- **흰 텍스트** — ID + 카메라까지 거리(m)

---

## 파일별 설명

### `main.py`
진입점. 인자 파싱, 카메라 소스 선택, 메인 루프 실행.

- `source = "ros"` (기본): `RosCapture`로 ROS2 토픽 구독
- `source = "6"` 등: `cv2.VideoCapture`로 직접 열기
- YOLO는 **원본(비회전) 프레임**으로 실행 → depth 좌표계와 일치
- 회전은 **시각화 렌더링 후 마지막**에 적용

```
raw frame → YOLO → direction → flow → visualizer.draw() → rotate → imshow
                            └→ ros_publisher.publish()
```

### `tracker.py`
`PersonTracker` 클래스. YOLOv8n + ByteTrack으로 사람 검출·추적.

- `conf = 0.5` — 신뢰도 임계값 (낮으면 오검출 증가)
- `MIN_BOX_HEIGHT = 80px` — 너무 작은 박스(주먹, 손 등) 필터링
- 반환값: `[(x1, y1, x2, y2, track_id), ...]`

### `direction.py`
`DirectionEstimator` 클래스. 픽셀 좌표 이력으로 이미지 기준 방향 추정.

- 최근 12프레임 발 위치를 선형회귀 → raw velocity
- EMA(α=0.35)로 속도 스무딩
- 각도 급변(60도 초과) 시 이전 각도 유지
- **이미지 픽셀 기준** (맵 좌표 아님) → OpenCV 창 시각화에 사용

### `flow.py`
`CrowdFlowEstimator` 클래스. 군중 전체 흐름 집계.

- 이동 중인 사람들의 평균 velocity 계산
- 최근 30프레임 시간적 스무딩
- 8방향 빈(bin)으로 도미넌트 방향 추출

### `ros_publisher.py`
`PeopleMarkerPublisher` ROS2 노드. depth + TF2로 맵 좌표 계산 후 마커 퍼블리시.

**3D 위치 계산 흐름:**
```
bbox 가슴 픽셀 (u, v)
      ↓ aligned_depth에서 D 샘플링 (7×7 → 15×15 → 25×25 확장 탐색)
카메라 프레임 3D 좌표
  X = (u - cx) * D / fx
  Y = (v - cy) * D / fy
  Z = D
      ↓ TF2: camera_color_optical_frame → map
맵 좌표 (mx, my)
      ↓ 최근 10프레임 이력 선형회귀
실제 이동 속도 벡터 (m/frame)
```

- depth는 **가슴 위치**(bbox 상단 1/3)에서 샘플링 — 바닥(발)은 RealSense에서 depth 누락 많음
- 맵 좌표 기반 velocity이므로 Nav2와 같은 좌표계

### `visualization.py`
`Visualizer` 클래스. OpenCV 창에 결과 렌더링.

- 사람별 색상 구분 (track_id 기반)
- 이동 중이면 방향 화살표, 정지 시 회색 점
- 군중 흐름 화살표 (화면 하단 중앙)
- FPS 표시

### `utils.py`
공통 유틸리티 함수.

- `get_color(track_id)` — 20색 팔레트에서 색상 선택
- `center_of_box`, `bottom_center_of_box` — bbox 좌표 계산
- `angle_degrees(vx, vy)` — 속도 벡터 → 각도 변환
- `draw_text_with_bg` — 배경 있는 텍스트 그리기

---

## 주요 파라미터 튜닝

| 파라미터 | 파일 | 기본값 | 설명 |
|----------|------|--------|------|
| `conf` | tracker.py | 0.5 | 낮추면 더 많이 잡지만 오검출 증가 |
| `MIN_BOX_HEIGHT` | tracker.py | 80px | 높이면 소형 오검출 감소 |
| `HISTORY_LEN` | direction.py | 12 | 길수록 속도 추정 안정, 반응 느려짐 |
| `EMA_ALPHA` | direction.py | 0.35 | 높을수록 최신 속도 반영 빠름 |
| `MIN_SPEED_PX` | direction.py | 0.8 | 정지 판정 임계값 (픽셀/프레임) |
| `TEMPORAL_LEN` | flow.py | 30 | 군중 흐름 시간적 평균 프레임 수 |

---

## 알려진 한계 및 다음 단계

| 한계 | 설명 |
|------|------|
| CPU 전용 추론 | GPU 없음 → YOLOv8n 기준 3~8 FPS |
| `direction.py`는 픽셀 기준 | 맵 좌표 velocity는 `ros_publisher.py`에서만 계산 |
| Nav2 연동 없음 | 현재는 시각화만, 회피 경로 반영 미구현 |
| AMCL 초기화 필요 | 재시작 시 RViz2에서 `2D Pose Estimate` 필요 |

**다음 단계 후보:**
- `/people_tracker/markers` → Nav2 costmap 레이어로 연동
- MediaPipe 추가로 keypoint 기반 depth 샘플링 정밀화
- YOLOv8s 모델로 교체해 검출 정확도 향상
