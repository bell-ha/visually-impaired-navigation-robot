# blind_nav_system 핵심 로직

시각장애인 안내 로봇의 핵심 파일들.  
음성 인터페이스, 상태 머신, **사회적 규범 기반 자율주행**, 시각 보조를 담당한다.

---

## 파일 구성

```
blind_nav_system/
├── main.py                  # 진입점 — 웹 대시보드 + 서브프로세스 오케스트레이터
├── interface.py             # 상태 머신 — 음성/버튼 입력 → Nav2 연동
├── navigation_client.py     # Nav2 클라이언트 — 사회적 회피 포함 경로 재계획
├── navigation_modifier.py   # 경유지 계산 — 접근자 회피 / 동행자 경로 힌트
├── vision_assistant.py      # 시각 보조 — 카메라 → GPT-4o Vision → TTS
├── interface-spec.md        # interface.py 상세 구현 명세
├── people_tracker/          # 사람 감지·분류·마커 퍼블리시 (별도 프로세스)
├── obstacle_pusher/         # 물체 장애물 밀기 (구현 예정)
└── tools/                   # 독립 실행 도구 (armleft, mouse_teleop 등)
```

---

## 전체 시스템 구조

```
[아두이노 시리얼]  [웹 브라우저]
       │                │
       └──────┬─────────┘
              ▼
          main.py  ←→  [ROS2 cmd_vel / battery]
         /       \
   stdin/stdout  stdin/stdout
        │               │
  interface.py    vision_assistant.py
        │
  NavigationClient (navigation_client.py)
        │   ↑ 1초마다 재평가
        │   │   접근자 있으면 매 주기 경유지 갱신
        │   └─ NavigationModifier (navigation_modifier.py)
        │         ├─ /people_tracker/people 구독
        │         └─ /local_costmap/costmap 구독
        │
  ┌─────┴──────────────────┐
  │                        │
NavigateThroughPoses   NavigateToPose
(경유지 있을 때)        (직행)

※ people_tracker/ 는 별도 터미널에서 실행 (독립 프로세스)
```

---

## 파일별 역할

### `main.py` — 진입점

```bash
python3 main.py  →  http://localhost:8080
```

- `interface.py`와 `vision_assistant.py`를 서브프로세스로 실행, stdin으로 명령 전달
- 아두이노 시리얼 파싱 (`TRIG,1` → 버튼, `DATA` + 압력 → pull 감지)
- Flask 웹 대시보드 (포트 8080):
  - 실시간 로그 스트리밍 (SSE)
  - 자동 / 수동 모드 전환 + D-pad 수동 조작
  - 로봇 속도 / TTS 속도 런타임 조정
  - 온/오프라인 모드 전환
  - **`🧠 사회적 회피 ON/OFF` 버튼** — `/tmp/social_nav_enabled` 파일로 상태 공유
- ROS2 cmd_vel 구독 → 후진 감지 시 TTS 안내
- 배터리 상태 모니터링

**추가된 Flask 라우트:**

| 라우트 | 설명 |
|--------|------|
| `POST /toggle_social_nav` | 사회적 회피 ON/OFF 토글 |

---

### `navigation_client.py` — Nav2 연동 + 실시간 재계획

interface.py가 동적으로 임포트해서 사용하는 ROS2 노드.

**기존 기능:**
- `location.yaml` → 목적지 좌표 로드
- global/local costmap 클리어 후 이동 시작
- cmd_vel 구독 → 회전 감지 시 "왼쪽/오른쪽" TTS 안내

**추가된 기능:**
- `NavigationModifier` 통합 — 목표 설정 시 경유지 계산
- **1초마다 재평가 타이머** — 이동 중 사람 상황 변화 감지
- **접근자가 있는 동안 매 주기 경유지 갱신** — 다가올수록 회피 방향 재계산
- 접근자 없을 때는 상황 변화 시에만 재계획 (잦은 재계획 방지)
- 경유지 있음 → `NavigateThroughPoses`, 없음 → `NavigateToPose`
- `/tmp/social_nav_enabled` 파일 확인 → OFF면 항상 직행
- `/nav/waypoints` (MarkerArray) — RViz에 노란 구체로 경유지 표시
- 목표 0.8m 이내 → 재계획 생략

```
목표 설정
  └─ compute_waypoints() → _send_to_nav2()
  └─ 1초 타이머 시작

매 1초: _replan_check()
  ├─ 잔여 거리 < 0.8m      → skip
  ├─ 접근자 있음           → 매 주기 경유지 갱신 (다가올수록 재계산)
  ├─ 상황 변화 없음        → skip
  └─ 변화 감지             → cancel → 재전송
```

---

### `navigation_modifier.py` — 경유지 계산

`NavigationClient`에 주입되는 헬퍼 클래스.

**구독 토픽:**
| 토픽 | 용도 |
|------|------|
| `/people_tracker/people` | 접근자/동행자 위치·속도·분류 |
| `/local_costmap/costmap` | 좌우 공간 여유 확인 |

**경유지 계산 로직:**

| 상황 | 동작 |
|------|------|
| 접근자 7m 이내 | 로봇 옆 0.6m 지점 costmap 비용 비교 → 더 열린 쪽 선택 (오른쪽 선호) → 앞 2m + 옆 0.6m 경유지 |
| 동행자만 4m 이내 | 동행자 속도 방향 40% + 목표 방향 60% 블렌딩 → 1.5m 앞 경유지 |
| 아무도 없음 | 경유지 없음 → 직행 |

**회피 경유지 상세:**
- 좌우 결정은 **로봇 바로 옆 0.6m** 지점 costmap 기준 (벽 감지 안정적)
- 경유지 위치는 **진행 방향 2m 앞 + 선택 방향 0.6m** (복도 한쪽 유지 효과)
- 오른쪽 선호 (사회적 규범) — 오른쪽 비용이 20 이상 높을 때만 왼쪽 선택

**주요 파라미터:**

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `APPROACH_DIST_MAX` | 7.0 m | 접근자 반응 거리 |
| `AVOIDANCE_OFFSET` | 0.6 m | 좌우 오프셋 거리 |
| `AVOIDANCE_LOOKAHEAD` | 2.0 m | 경유지 앞 방향 투영 거리 |
| `RIGHT_PREFERENCE_BIAS` | 20 | 오른쪽 선호 마진 (costmap 비용 단위) |
| `COMPANION_DIST_MAX` | 4.0 m | 동행자 반응 거리 |
| `COMPANION_BLEND` | 0.4 | 동행자 방향 가중치 |
| `OCCUPIED_THRESHOLD` | 65 | costmap 점령 기준값 |

---

### `people_tracker/` — 사람 감지 모듈 (별도 프로세스)

YOLOv8n + ByteTrack + depth + TF2로 사람을 맵 좌표에서 추적·분류.  
**`main.py`와 별개로 실행해야 한다.**

**퍼블리시 토픽:**
| 토픽 | 내용 |
|------|------|
| `/people_tracker/markers` | RViz 실린더 (🔴접근자 / 🟢동행자 / 🔵정지) + 화살표 |
| `/people_tracker/people` | JSON: 위치, 속도, 분류, 거리, depth_estimated |

**거리 측정 방식:**

| 구간 | 방식 | 정확도 |
|------|------|--------|
| 0~6m | depth 정밀 측정 | 높음 |
| 6~9m | bbox 높이 추정 (`D = 1.7m × fy / bbox_px`) | 낮음 (방향 파악용) |

- bbox 추정 시 RViz 마커 반투명, 텍스트에 `~` 접두사 표시

> 상세 내용은 `people_tracker/README.md` 참고

---

### `obstacle_pusher/` — 물체 밀기 모듈 (구현 예정)

Nav2 경로 상 물체 장애물에 대해 **우회 비용 vs 밀기 비용** 비교 후 시간 최적 행동 선택.

| 파일 | 역할 | 상태 |
|------|------|------|
| `obstacle_detector.py` | Nav2 경로 위 물체 감지 | ⬜ |
| `object_classifier.py` | YOLO + depth → 밀기 가능 여부 | ⬜ |
| `cost_estimator.py` | 우회 거리 vs 밀기 비용 비교 | ⬜ |
| `push_executor.py` | 밀기 실행 + 성공/실패 감지 | ⬜ |

> 상세 계획은 `obstacle_pusher/README.md` 참고

---

### `vision_assistant.py` — 시각 보조

- "지금 뭐가 보여?" 또는 `/vision` 명령으로 트리거
- ROS2 카메라 → base64 → GPT-4o Vision API → TTS 음성 출력

---

## 실행 순서

**터미널 1 — ROS2 스택 (Nav2 + AMCL + 센서)**
```bash
ros2 launch /home/hello-robot/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/launch/stretch_robot_process.launch.xml
# → RViz에서 2D Pose Estimate로 AMCL 초기화
```

**터미널 2 — People Tracker (별도 프로세스)**
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

---

## RViz 설정 (통합)

| Fixed Frame | `map` |
|-------------|-------|
| `/people_tracker/markers` | 사람 실린더 (빨강/초록/파랑) + 화살표 |
| `/nav/waypoints` | 노란 구체 — 계산된 경유지 |

---

## 의존성

- **ROS2 Humble** — rclpy, nav2_msgs, visualization_msgs 등
- **통합 venv** (`blind_nav_system/venv/`) — flask, vosk, gTTS, ultralytics 등
- **`src/.env`** — `OPENAI_API_KEY` 필수
- **`config/location.yaml`** — 목적지 이름 → Nav2 좌표

---

## 미완료 / 테스트 필요

| 항목 | 설명 |
|------|------|
| NavigateThroughPoses 동작 확인 | Nav2 액션 서버 응답 검증 |
| 실시간 재계획 검증 | 접근자 등장 시 `[NAV] 접근자 경유지 갱신` 로그 확인 |
| `/nav/waypoints` RViz 확인 | 노란 구체 실제 표시 여부 |
| obstacle_pusher 전체 구현 | 4개 파일 미구현 |
