# Visually Impaired Navigation Robot

시각장애인의 실내 이동을 돕기 위한 사람 중심 안내 로봇 프로젝트.
Hello Robot Stretch SE3 위에 ROS2 Nav2 기반 자율주행과 음성 인터페이스, 시각 보조 기능을 결합한다.

## 하드웨어
- 로봇: Hello Robot Stretch SE3
- OS: Ubuntu 22.04 / ROS2 Humble
- LiDAR: RPLidar (자율주행 장애물 감지)
- 카메라: Intel RealSense D435i (RGB + Depth, 시각 보조 및 장애물 감지)
- 마이크: ReSpeaker 4 Mic Array (index 5)
- 스피커: 내장 HDA Intel PCH ALC256 Analog (index 0)
- 버튼/센서: Arduino FTDI (`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0`)

## 하드웨어 성능 (Hello Robot Stretch 3)

### 컴퓨팅
| 항목 | 사양 |
|------|------|
| CPU | Intel Core i5-8259U |
| RAM | 16 GB |
| 저장장치 | 480 GB SSD |

### 외형 및 배터리
| 항목 | 사양 |
|------|------|
| 무게 | 24.5 kg |
| 크기 (W × D × H) | 330 × 340 × 1410 mm |
| 배터리 | 12V SLA × 2개 (합계 18 AH) |
| 동작 시간 | 약 2~5시간 (부하에 따라 다름) |

### 이동 (Base)
| 항목 | 사양 |
|------|------|
| 구동 방식 | 차동 구동 (Differential Drive, 스테퍼 모터 × 2) |
| 최대 속도 | 0.3 m/s |
| 극복 가능 턱 높이 | 1 cm |
| 절벽 감지 센서 | Sharp GP2Y0A51SK0F × 4 (감지 범위 2~15 cm) |
| IMU | Bosch BNO085 9-DOF |

### 리프트 (Lift)
| 항목 | 사양 |
|------|------|
| 수직 이동 범위 | 110 cm |
| 최대 페이로드 | 5 kg |

### 암 (Arm)
| 항목 | 사양 |
|------|------|
| 수평 연장 범위 | 51 cm |
| 최대 페이로드 | 3 kg |
| 구조 | 알루미늄 텔레스코핑 5단 링크 |

### 손목 및 그리퍼
| 항목 | 사양 |
|------|------|
| Yaw 범위 | 330° |
| Pitch 범위 | 150° |
| Roll 범위 | 345° |
| 그리퍼 최대 페이로드 | 2 kg |
| 그리퍼 최대 개구 | 15 cm |

### 헤드 (Pan-Tilt)
| 항목 | 사양 |
|------|------|
| Pan 범위 | 346° (-234° ~ +112°) |
| Tilt 범위 | 115° (-25° ~ +90°) |

### 내장 센서 및 I/O
| 항목 | 사양 |
|------|------|
| 헤드 카메라 | Intel RealSense D435if (최대 인식 거리 10 m) |
| 그리퍼 카메라 | Intel RealSense D405 (최적 범위 7~50 cm) |
| 보조 RGB 카메라 | Arducam OV9782 광각 |
| LiDAR | Slamtec RPLIDAR A1 (범위 0.15~12 m, 각도 해상도 1°) |
| 마이크 어레이 | ReSpeaker v2.0 (4-MEMS 마이크, 음성 인식 최대 5 m) |
| 손목 가속도계 | ADXL343 3축 |
| 개발자 전원 (트렁크) | 12V @ 5A |
| USB 허브 | USB 3.0 4포트 |

> **참고:** 이 프로젝트는 내장 D435if 대신 외장 **Intel RealSense D435i**를 시각 보조용으로 추가 사용하며, RPLIDAR를 자율주행 장애물 감지에 활용한다. 최대 주행 속도 0.3 m/s와 턱 높이 1 cm 제약을 고려해 경로 계획 및 속도 파라미터를 설정해야 한다.

## 빠른 시작 (모든 것을 로봇 자체에서 실행)

### 1. 사전 준비
```bash
# 로봇 초기화 (처음 한 번)
stretch_robot_battery_check.py
stretch_free_robot_process.py
stretch_robot_home.py
stretch_robot_stow.py
```

### 2. 네비게이션 스택 실행
```bash
ros2 launch /home/hello-robot/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/launch/stretch_robot_process.launch.xml
```

### 3. RViz2 실행 (초기 위치 지정용)
```bash
ros2 run rviz2 rviz2
# RViz에서 2D Pose Estimate로 로봇 초기 위치 지정
```

### 4. 인터페이스 실행
```bash
source /opt/ros/humble/setup.bash
source ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system
python3 interface.py
```

### 5. (선택) 시각 보조 실행
```bash
source /opt/ros/humble/setup.bash
source ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system
python3 vision_assistant.py
# Enter 키를 누르면 카메라로 전방을 분석해 음성으로 안내
```

## 소스 구조

### blind_nav_system/blind_nav_system/ (핵심 로직)

| 파일 | 역할 |
|------|------|
| `interface.py` | **메인 실행 파일.** 음성 인식·TTS·버튼 입력·상태 관리·Nav2 연동을 통합한 단독 실행 모듈 |
| `main.py` | interface.py + vision_assistant.py를 서브프로세스로 실행하는 오케스트레이터 |
| `navigation_client.py` | Nav2 액션 클라이언트. 목적지 좌표로 로봇을 실제로 이동시킴 |
| `vision_assistant.py` | **시각 보조 모듈.** Enter 키를 누르면 RealSense로 전방 촬영 후 GPT-4o Vision으로 분석·음성 안내 |

### blind_nav_system/blind_nav_system/tools/ (독립 실행·테스트 도구)

| 파일 | 역할 |
|------|------|
| `audio_web_test.py` | 오디오 진단 웹 툴. 브라우저에서 마이크 레벨·스피커 440Hz 테스트 가능 |
| `mouse_teleop.py` | 마우스 스크롤(전진/후진)·버튼(회전)으로 로봇을 조종하는 텔레오퍼레이션 노드 |
| `armleft.py` | Stretch SE3 팔·리프트 제어 노드 |
| `run_test_panels.py` | 엘리베이터 버튼 패널 배치 추론 및 결과 이미지 저장 |

### blind_nav_system/blind_nav_system/tools/hardware/ (아두이노 연동)

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

- `stretch_robot_process.launch.xml`: 모든 노드를 한 번에 실행하는 메인 런치 파일
  - 포함: stretch_driver, RPLidar, RealSense D435i, map_server, AMCL, Nav2

### maps/

- 사전 생성된 실내 지도 파일 (`.pgm` + `.yaml`)

### .env 설정 (필수)

`src/.env` 파일에 OpenAI API 키 설정:
```
OPENAI_API_KEY=sk-...
```

## 문서
- [Navigation & SLAM 가이드](navigation-and-slam-guide.md)
