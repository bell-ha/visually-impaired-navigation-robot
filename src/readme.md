## 📂 소스 구조

### blind_nav_system/blind_nav_system/ (핵심 로직)

| 파일 | 역할 |
|------|------|
| `interface.py` | **메인 실행 파일.** 음성 인식·TTS·버튼 입력·상태 관리·Nav2 연동을 통합한 단독 실행 모듈 |
| `navigation_client.py` | Nav2 액션 클라이언트. 목적지 좌표로 로봇을 실제로 이동시킴 |
| `vision_assistant.py` | **시각 보조 모듈.** Enter 키를 누르면 RealSense로 전방 촬영 후 GPT-4o Vision으로 분석·음성 안내 |
| `audio_web_test.py` | 오디오 진단 웹 툴. 브라우저에서 마이크 레벨·스피커 440Hz 테스트 가능 (`python3 audio_web_test.py`) |
| `mouse_teleop.py` | 마우스 스크롤(전진/후진)·버튼(회전)으로 로봇을 조종하는 텔레오퍼레이션 노드 |
| `armleft.py` | Stretch SE3 팔·리프트 제어 노드 |
| `sensor_monitor.py` | 센서 상태 감시 |
| `main_state_machine.py` | 상태 머신 (interface.py에 통합됨) |

### hardware/

| 파일 | 역할 |
|------|------|
| `input.py` | 아두이노 시리얼 데이터 읽기 및 시각화 (디버그용) |
| `input_bridge.py` | 압력 센서 데이터를 Python 로직으로 전달하는 브릿지 |
| `signal_to_python/signal_to_python.ino` | 아두이노 펌웨어. `DATA,0,0` (idle) / `TRIG,...` (버튼) 포맷으로 시리얼 전송 |

> **아두이노 포트 고정 경로**: `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0`
> (재부팅 후에도 포트 번호 변경 없음)

### config/

- `location.yaml`: 목적지 이름 → Nav2 좌표 매핑 테이블

### launch/

- `stretch_robot_process.launch.xml`: **모든 노드를 한 번에 실행**하는 메인 런치 파일
  - 포함: stretch_driver, RPLidar, RealSense D435i, map_server, AMCL, Nav2

### maps/

- 사전 생성된 실내 지도 파일 (`.pgm` + `.yaml`)

---

### .env 설정 (필수)

`src/.env` 파일에 OpenAI API 키 설정:
```
OPENAI_API_KEY=sk-...
```
