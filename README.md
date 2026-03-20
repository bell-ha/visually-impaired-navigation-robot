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
ros2 launch /home/hello-robot/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system/launch/stretch_robot_process.launch.xml
```

### 3. RViz2 실행 (초기 위치 지정용)
```bash
ros2 run rviz2 rviz2
# RViz에서 2D Pose Estimate로 로봇 초기 위치 지정
```

### 4. 인터페이스 실행
```bash
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system
python3 interface.py
```

### 5. (선택) 시각 보조 실행
```bash
python3 vision_assistant.py
# Enter 키를 누르면 카메라로 전방을 분석해 음성으로 안내
```

## 문서
- [시스템 설계](docs/design.md)
- [Navigation & SLAM 가이드](docs/navigation-and-slam-guide.md)
- [소스 구조](src/readme.md)
