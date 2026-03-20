# Stretch 3 내비게이션 및 SLAM 가이드

## 환경

- 로봇: Hello Robot Stretch SE3
- OS: Ubuntu 22.04 / ROS2 Humble
- 워크스페이스: `~/ament_ws`
- **모든 작업을 로봇 자체에서 직접 실행** (SSH 불필요)

---

## 1. 부팅 후 사전 점검

```bash
stretch_robot_battery_check.py     # 배터리 확인 (24V 이하 시 충전)
stretch_free_robot_process.py      # 기존 프로세스 정리
stretch_robot_home.py              # 로봇 원점 설정
stretch_robot_stow.py              # 팔 수납
```

---

## 2. 지도 생성 (SLAM)

### 2.1 실행

```bash
# 1단계: 드라이버
ros2 launch stretch_core stretch_driver.launch.py mode:=navigation broadcast_odom_tf:=True

# 2단계: LiDAR
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/hello-lrf \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser

# 3단계: SLAM Toolbox
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p odom_frame:=odom -p base_frame:=base_link \
  -p scan_topic:=/scan -p mode:=mapping \
  -p use_sim_time:=false \
  -p min_laser_range:=0.8 -p max_laser_range:=15.0 \
  -p minimum_travel_distance:=0.1 \
  -p minimum_travel_heading:=0.1 \
  -p map_update_interval:=0.5 \
  -p transform_timeout:=0.2

# 4단계: RViz2 (시각화)
ros2 run rviz2 rviz2

# 5단계: 키보드 조종으로 공간 탐색
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/stretch/cmd_vel
```

### 2.2 지도 저장

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/maps/맵이름
```

---

## 3. 내비게이션 실행

### 3.1 전체 스택 한 번에 실행 (권장)

```bash
ros2 launch /home/hello-robot/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system/launch/stretch_robot_process.launch.xml
```

포함 내용: stretch_driver, RPLidar, RealSense D435i, map_server, AMCL, Nav2

> 사용할 지도와 파라미터 파일은 런치 파일 내 `map`, `params_file` 인자로 변경

### 3.2 RViz2 실행 및 초기 위치 설정

```bash
ros2 run rviz2 rviz2
```

> **rviz2 실행 오류 시** (snap libpthread 충돌): `~/.bashrc`에 아래 내용이 있는지 확인
> ```bash
> export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ":" "\n" | grep -v "/snap/" | tr "\n" ":" | sed "s/:$//")
> export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
> ```

RViz 설정:
- Fixed Frame: `map`
- Map: Durability `Transient Local`, Reliability `Reliable`
- **2D Pose Estimate**로 실제 로봇 위치 지정 (필수 - AMCL 활성화)

---

## 4. 인터페이스 실행

```bash
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system
python3 interface.py
```

버튼을 누르면 음성으로 목적지를 입력받고 자율 이동 시작.

### 목적지 추가

`config/location.yaml`에 이름과 좌표 추가:

```bash
# RViz의 Publish Point로 좌표 확인
ros2 topic echo /clicked_point
```

---

## 5. 시각 보조 모듈

```bash
python3 vision_assistant.py
```

Enter 키를 누르면 RealSense 카메라로 전방을 촬영 후 GPT-4o Vision이 분석 결과를 음성으로 안내.
강의실 번호판, 문 개폐 상태, 장애물 등을 인식한다.

---

## 6. Nav2 파라미터 주요 설정

파일 위치: `/home/hello-robot/ament_ws/src/stretch_ros2/stretch_nav2/config/nav2_params_human.yaml`

| 항목 | 값 | 설명 |
|------|----|------|
| footprint | 5각형, 뒤 -0.85m | 뒤에 서 있는 사람을 footprint 안으로 포함 |
| inflation_radius | 0.45m | 벽에서 45cm 이상 거리 유지 |
| cost_scaling_factor | 2.5 | 복도 중앙으로 경로 유도 |
| max_vel_theta | 0.5 rad/s | 부드러운 회전 |
| min_vel_x | 0.0 | 후진 완전 차단 |
| use_astar | true | A* 경로 계획 |

### 런타임 속도 조절 (선택)

```bash
ros2 param set /controller_server FollowPath.max_vel_x 0.2
ros2 param set /controller_server FollowPath.max_vel_theta 0.3
```

---

## 7. 배터리 관리

- 24V 이하 → 충전 필요
- LED 노란색(2초 주기) → 저전압 경고
- 충전기: NOCO Genius10 (기본 충전: 12V AGM 모드)
