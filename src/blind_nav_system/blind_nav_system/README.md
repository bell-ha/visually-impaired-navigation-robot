# blind_nav_system 핵심 로직

시각장애인 안내 로봇의 핵심 파일들. 음성 인터페이스, 상태 머신, 자율주행 연동, 시각 보조를 담당한다.

---

## 파일 구성

```
blind_nav_system/
├── main.py              # 진입점 — 웹 대시보드 + 서브프로세스 오케스트레이터
├── interface.py         # 상태 머신 — 음성/버튼 입력 → Nav2 연동
├── navigation_client.py # Nav2 액션 클라이언트 — 목적지 이동 + 회전 안내
├── vision_assistant.py  # 시각 보조 — 카메라 → GPT-4o Vision → TTS
├── interface-spec.md    # interface.py 상세 구현 명세 (상태 전이 규칙, GPT 스키마 등)
└── tools/               # 독립 실행·테스트 도구 (armleft, mouse_teleop 등)
```

---

## 파일별 역할

### `main.py` — 진입점

```
python3 main.py  →  http://localhost:8080
```

- `interface.py`와 `vision_assistant.py`를 서브프로세스로 실행하고 stdin으로 명령 전달
- 아두이노 시리얼(`/dev/serial/by-id/usb-FTDI_...`) 파싱
  - `TRIG,1` → interface에 `/button`
  - `TRIG,2` → vision에 `/vision`
  - `DATA` + 압력값 → pull 감지 시 interface에 `/pull`
- Flask 웹 대시보드 (포트 8080):
  - 실시간 로그 스트리밍 (SSE)
  - 자동/수동 모드 전환
  - 수동 조작 D-pad (방향키 지원)
  - 로봇 속도 / TTS 속도 런타임 조정
  - 온/오프라인 모드 전환 (GPT 사용 여부)
- ROS2 cmd_vel 구독 → 후진 감지 시 TTS 안내
- 배터리 상태 모니터링 (`/battery` 토픽)

### `interface.py` — 상태 머신

```
python3 interface.py
python3 interface.py --no-hw   # 하드웨어 없이 실행 (main.py가 이 옵션으로 호출)
```

- 상태: `LOCKED → READY → NAV → PAUSED`
- 음성 인식 (Vosk STT) + TTS (gTTS) + beep
- GPT 호출로 목적지 의도 해석 및 확인 질문 생성
- `NavigationClient`를 동적으로 임포트해 목적지 이동 요청
- 버튼 / pull 입력에 따라 상태 전이
- stdin으로 명령 수신 (`/button`, `/pull`, `/cancel`, `/offline` 등) → main.py와 통신

> 상세 규칙은 `interface-spec.md` 참고 (상태 전이, GPT 스키마, TTS 문구, 파라미터 전체)

### `navigation_client.py` — Nav2 연동

interface.py가 동적으로 임포트해서 사용하는 ROS2 노드.

- `location.yaml`에서 목적지 이름 → 좌표(x, y, w) 로드
- Nav2 `navigate_to_pose` 액션 서버에 goal 전송
- global/local costmap 클리어 후 이동 시작
- `/stretch/cmd_vel` 구독 → `angular.z > 0.4 rad/s` 감지 시 "왼쪽/오른쪽으로 회전" 안내 (5초 간격)
- 도착 시 `is_arrived = True` → interface.py가 LOCKED로 전환
- `cleanup()`: 정지 명령 퍼블리시 + goal cancel + 경로 초기화

### `vision_assistant.py` — 시각 보조

```
python3 vision_assistant.py
python3 vision_assistant.py --mic-index 5  # ReSpeaker 사용 시
```

- "지금 뭐가 보여?" 또는 `/vision` stdin 명령으로 트리거
- ROS2 `/camera/color/image_raw` 또는 RealSense 직접 캡처
- 이미지를 base64로 GPT-4o Vision API에 전송
- 응답을 TTS로 음성 출력
- main.py와 독립적으로 단독 실행 가능

---

## 파일 간 관계

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
        │
   [Nav2 navigate_to_pose]
```

---

## 의존성

- **ROS2 Humble** (rclpy, nav2_msgs, sensor_msgs 등) — 시스템 패키지
- **통합 venv** (`blind_nav_system/venv/`) — pip 패키지 (vosk, gTTS, flask, requests 등)
- **`src/.env`** — `OPENAI_API_KEY` 필수
- **`config/location.yaml`** — 목적지 이름 → Nav2 좌표 매핑
