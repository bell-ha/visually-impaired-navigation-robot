# People Tracker — 사회적 규범 기반 실내 경로 계획

YOLOv8 + ByteTrack으로 사람을 검출·추적하고, 접근자 / 동행자로 분류해
Nav2 경로를 실시간으로 조정하는 모듈.

> **논문 타겟:** ACM CHI  
> **핵심 주장:** 기존 Nav2는 사람을 단순 장애물로 취급하지만, 이 시스템은 이동 방향을 분류해 사회적 규범(오른쪽 양보, 동행자 경로 힌트)을 적용한다.

---

## 전체 파이프라인

```
ROS2 머리 카메라 (RGB + aligned_depth)
        │
        ▼
  [tracker.py]  YOLOv8n + ByteTrack
  카메라 90° 회전 보정 후 YOLO 추론 → bbox 역변환
        │
        ▼
  [ros_publisher.py]
  ├─ depth patch 탐색 (0~6m 정밀) → 실패 시 bbox 높이로 거리 추정 (6~9m)
  ├─ TF2: camera_color_optical_frame → map 좌표 (mx, my)
  │    └─ depth 메시지 타임스탬프 기준 (로봇 이동 중 위치 오차 방지)
  ├─ 속도 벡터 (vx, vy) — 선형회귀 8프레임
  │    └─ bbox 추정 시 4프레임 이상 이력 필요 (노이즈 보정)
  ├─ 로봇 yaw와 비교 → 접근자 / 동행자 / 정지 분류
  ├─ /people_tracker/markers  (RViz: 빨강/초록/파랑 실린더)
  │    └─ bbox 추정 시 반투명, 텍스트에 ~ 접두사
  └─ /people_tracker/people   (JSON: 위치+속도+분류+거리+depth_estimated)
        │
        ▼
  [navigation_modifier.py]
  ├─ 접근자 감지 (7m 이내) → 근거리 costmap으로 좌우 결정
  │    └─ 경유지 = 앞 2m + 선택된 방향 0.6m (복도 한쪽 유지 효과)
  └─ 동행자 감지 (4m 이내) → 속도 방향 블렌딩 → 경유지
        │
        ▼
  [navigation_client.py]
  ├─ 목표 설정 시 + 1초마다 재평가
  ├─ 접근자 있으면 매 주기 경유지 갱신 (다가올수록 회피 방향 재계산)
  ├─ 경유지 있음 → NavigateThroughPoses
  ├─ 경유지 없음 → NavigateToPose (기존)
  └─ /nav/waypoints (RViz: 노란 구체)
```

---

## ✅ 구현 완료

### 1. 카메라 회전 보정 (`tracker.py`)
- 카메라가 물리적으로 90° 회전 장착 → YOLO가 옆으로 누운 사람을 못 잡는 문제
- `update(frame, rotate_deg=90)` — YOLO 입력은 회전, bbox는 원본 좌표로 역변환
- 정면·후면·측면 모두 감지 가능

### 2. 맵 좌표 기반 속도 추정 (`ros_publisher.py`)
- depth patch 7×7 → 15×15 → 25×25 확장 탐색 (0.1~6m 유효)
- TF2: `camera_color_optical_frame → map`, **depth 메시지 타임스탬프** 사용
- 선형회귀 8프레임, 1m 이상 점프 시 이력 리셋
- TF 타임아웃 0.05s (이전 0.3s에서 단축)

### 3. bbox 거리 추정 fallback (`ros_publisher.py`)
- depth 실패(6m 초과·노이즈) 시 bbox 높이로 거리 역산
  ```
  D = (사람 평균 키 1.7m × fy) / bbox_픽셀_높이
  ```
- 유효 범위: 0.5~9m, 15px 이하 bbox는 제외
- 커버리지: 기존 0~6m → **0~9m** 으로 확장

| 구간 | 방식 | 정확도 |
|------|------|--------|
| 0~6m | depth 정밀 측정 | 높음 |
| 6~9m | bbox 높이 추정 | 낮음 (방향 파악 용도) |

- RViz 마커: bbox 추정 시 반투명, 텍스트 `~7.1m` 형태로 표시

### 4. 접근자 / 동행자 분류 (`ros_publisher.py`)
- `|person_angle - robot_yaw| > 90°` → 접근자
- 나머지 → 동행자
- RViz 실린더 색상: 🔴 빨강(접근자) / 🟢 초록(동행자) / 🔵 파랑(정지·미분류)
- 로봇 위치(TF) 없으면 `distance: -1`, 분류는 유지

### 5. `/people_tracker/people` 토픽 퍼블리시 (`ros_publisher.py`)
```json
{
  "people": [
    {
      "id": 1, "x": 1.5, "y": 2.3, "vx": 0.05, "vy": -0.1,
      "classification": "approaching", "distance": 2.1,
      "depth_estimated": false
    }
  ],
  "robot_x": 0.0, "robot_y": 0.0, "robot_yaw": 45.0
}
```
- `depth_estimated: true` — bbox 거리 추정 사용 여부 (신뢰도 낮음 표시)
- `robot_x/y/yaw` — TF 없으면 `null`

### 6. 경유지 계산 (`navigation_modifier.py`)
- **접근자 (7m 이내):**
  - 좌우 결정: 로봇 바로 옆 0.6m 지점의 costmap 비용 비교 (벽 감지 안정)
  - 경유지 위치: 로봇 진행 방향 **2m 앞 + 선택 방향 0.6m**
  - → 복도 한쪽을 유지하며 자연스럽게 통과하는 효과
  - 오른쪽 선호 (사회적 규범), 오른쪽 비용이 20 이상 높으면 왼쪽
- **동행자 (4m 이내):** `목표방향 60% + 동행자속도방향 40%` 블렌딩 → 1.5m 앞 경유지
- 경유지가 점령 영역(costmap > 65)이면 폐기

### 7. 실시간 경로 재계획 (`navigation_client.py`)
- 목표 설정 시 초기 계획 + **1초마다** 재평가 (이전 2초)
- **접근자가 있는 동안 매 주기 경유지 갱신** — 사람이 다가올수록 회피 방향 재계산
- 접근자 없을 때는 상황 변화 시에만 재계획 (잦은 재계획 방지)
- 목표까지 0.8m 이하이면 재계획 생략
- 경유지 있음 → `NavigateThroughPoses`, 없음 → `NavigateToPose`

### 8. RViz 시각화
| 토픽 | 내용 |
|------|------|
| `/people_tracker/markers` | 실린더(빨강/초록/파랑) + 화살표 + ID·거리 텍스트 |
| `/nav/waypoints` | 노란 구체 + "경유지 N" 텍스트 (재계획 시 갱신) |

### 9. 웹 대시보드 On/Off (`main.py`)
- `🧠 사회적 회피 ON/OFF` 버튼
- `/tmp/social_nav_enabled` 파일로 상태 공유
- OFF 시 기존 `NavigateToPose` 직행만 사용 → Baseline 비교 실험 가능

---

## 거리별 커버리지

```
0 ─────────── 2m ──────── 3m ──────── 6m ──────── 9m
 depth+LiDAR   LiDAR만     people_tracker          bbox fallback
 (Nav2 처리)   (Nav2)      depth 정밀              bbox 추정 (반투명)
                           ←── 사회적 회피 전체 범위 ──────────→
```

---

## 실행 방법

### 사전 조건
1. ROS2 Humble + Nav2 실행 중
2. `2D Pose Estimate`로 AMCL 초기화 완료

### 실행 순서

**터미널 1 — ROS2 스택**
```bash
ros2 launch /home/hello-robot/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/launch/stretch_robot_process.launch.xml
```

**터미널 2 — People Tracker**
```bash
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system/people_tracker
bash run.sh
# 기본값: source=ros, rotate=90
```

**터미널 3 — 메인 대시보드 (내비게이션 포함)**
```bash
source /opt/ros/humble/setup.bash
source ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system
python3 main.py
# → http://localhost:8080
```

### 토픽 확인
```bash
# 사람 분류 데이터 (depth_estimated 포함)
ros2 topic echo /people_tracker/people

# 경유지 갱신 여부
ros2 topic echo /nav/waypoints

# 토픽 수신 주파수
ros2 topic hz /people_tracker/markers
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /camera/camera/color/camera_info
```

### RViz 설정
| Fixed Frame | `map` |
|-------------|-------|
| Add → MarkerArray | `/people_tracker/markers` — 사람 실린더 |
| Add → MarkerArray | `/nav/waypoints` — 경유지 (노란 구체) |
| Add → TF | map → base_link → camera_color_optical_frame 체인 확인 |

---

## 파일 구조

| 파일 | 역할 |
|------|------|
| `main.py` | 진입점, ROS2/웹캠/영상 소스 선택, 웹 대시보드 연동 |
| `tracker.py` | YOLOv8n + ByteTrack, 카메라 회전 보정 |
| `ros_publisher.py` | depth+bbox→거리, TF2→맵좌표, 분류, 마커·people 퍼블리시 |
| `direction.py` | 픽셀 기반 방향 추정 (OpenCV 창 화살표 시각화용) |
| `visualization.py` | OpenCV 창 렌더링 |
| `flow.py` | 군중 흐름 추정 (현재 미사용) |
| `utils.py` | 공통 유틸리티 |
| `../navigation_modifier.py` | 경유지 계산 (접근자 회피 / 동행자 힌트) |
| `../navigation_client.py` | Nav2 액션 클라이언트 + 실시간 재계획 |

---

## 주요 파라미터

| 파라미터 | 파일 | 현재값 | 설명 |
|----------|------|--------|------|
| `MIN_LONG_SIDE` | tracker.py | 60px | bbox 긴 변 최소 크기 |
| `HISTORY_LEN` | ros_publisher.py | 8 | 속도 계산 이력 프레임 수 |
| `MIN_HISTORY` | ros_publisher.py | 2 | depth 정밀 시 최소 이력 |
| `MIN_HISTORY_BBOX` | ros_publisher.py | 4 | bbox 추정 시 최소 이력 |
| `MAP_MIN_SPEED` | ros_publisher.py | 0.02 m/f | 정지 판정 임계값 |
| `PERSON_HEIGHT_M` | ros_publisher.py | 1.7 m | bbox 거리 추정용 평균 키 |
| `BBOX_DEPTH_MAX` | ros_publisher.py | 9.0 m | bbox 추정 최대 거리 |
| `APPROACH_DIST_MAX` | navigation_modifier.py | 7.0 m | 접근자 반응 거리 |
| `AVOIDANCE_OFFSET` | navigation_modifier.py | 0.6 m | 좌우 경유지 오프셋 |
| `AVOIDANCE_LOOKAHEAD` | navigation_modifier.py | 2.0 m | 경유지 앞 방향 투영 거리 |
| `COMPANION_DIST_MAX` | navigation_modifier.py | 4.0 m | 동행자 반응 거리 |
| `COMPANION_BLEND` | navigation_modifier.py | 0.4 | 동행자 방향 가중치 |
| `REPLAN_INTERVAL` | navigation_client.py | 1.0 s | 재평가 주기 |
| `REPLAN_MIN_DIST` | navigation_client.py | 0.8 m | 재계획 최소 잔여 거리 |

---

## 논문 실험 설계 (추후)

| 항목 | 설명 |
|------|------|
| **실험 시나리오** | 복도 A→B, 중간에 접근자 1명 / 2명 / 동행자 혼재 |
| **Baseline 비교** | 사회적 회피 OFF vs ON 이동시간 비교 |
| **지표 수집** | 이동시간, 경로 이탈량, 재계획 횟수 로깅 |
| **유저 스터디** | 참가자 설문 — 편안함 / 자연스러움 (CHI 필수) |
