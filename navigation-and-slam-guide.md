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
```

---

## 2. 지도 생성 (SLAM)

### 2.1 실행

```bash
# 1단계: 드라이버
ros2 launch stretch_core stretch_driver.launch.py mode:=navigation broadcast_odom_tf:=True

# 2단계: LiDAR  ★ angle_compensate 필수 (아래 "빔 수" 항목)
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/hello-lrf \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=true

# 3단계: SLAM Toolbox
#   min_laser_range는 용도에 따라 갈라 쓴다 (아래 "min_laser_range" 항목)
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
ros2 run rviz2 rviz2 -d src/blind_nav_system/config/mapping/mapview.rviz

# 5단계: 조종으로 공간 탐색 — 키보드 또는 게임패드(아래 2.1.4)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/stretch/cmd_vel
```

#### 2.1.1 ★ `angle_compensate:=true` — 없으면 맵이 통째로 안 만들어진다

이걸 빼면 스캔마다 빔 수가 **950~956개로 들쭉날쭉**해지고, slam_toolbox가

```
contains N range readings, expected 942
```

를 내며 **모든 스캔을 거부한다.** 첫 장만 들어가고 그 뒤로 한 장도 안 쌓인다.
켜면 **1080개로 고정**된다(2026-09-02 실측, 40샘플 전수 확인).

**증상이 "맵이 안 커진다"로 나와서 원인을 찾기 어렵다** — 노드는 전부 정상으로
보이고 에러도 로그 안쪽에만 찍힌다. 맵이 안 자라면 이것부터 의심하라.

#### 2.1.2 `min_laser_range` — 좁은 공간에서는 낮춰야 한다

| 용도 | 값 | 이유 |
|---|---|---|
| 복도·건물 전체 | `0.8` | 로봇 자기 몸·부속물을 걸러낸다 |
| 엘리베이터 캐빈 등 좁은 공간 | `0.3` | 벽이 0.7~1.0m라 0.8이면 **벽이 통째로 버려진다** |

좁은 공간을 찍을 때 0.8 그대로 두면 캐빈 안에서 아무것도 안 찍힌다.

#### 2.1.3 운영자 배제 필터 — 뒤따르는 사람이 벽으로 찍히는 것 막기

사람이 로봇을 따라다니며 맵핑하면 그 사람이 그대로 벽이 된다. 로봇 뒤 ±60°를
잘라낸다.

```bash
ros2 run laser_filters scan_to_scan_filter_chain --ros-args \
  --params-file src/blind_nav_system/config/mapping/laser_rear_cut.yaml
# → SLAM 3단계의 scan_topic을 바꾼다:  -p scan_topic:=/scan_rear_cut
```

⚠ **`replace_with_nan: true`가 필수다.** 기본값(false)은 잘라낸 빔을
`range_max + 1 = 13.0`으로 채우는데, SLAM의 `max_laser_range=15.0`이 그 값을
**13m 거리의 벽으로 읽어 로봇 뒤에 가짜 원형 벽**을 만든다. 운영자를 지우려다
훨씬 큰 가짜 구조물을 얻는다.

⚠ **각도는 라이다 프레임 기준이다.** TF 실측 `base_link → laser = 180°`라
**라이다 0° = 로봇 뒤**다. 부호를 뒤집으면 앞쪽을 지우고 **정확히 운영자만 남긴다.**

#### 2.1.4 게임패드 조종

키보드보다 훨씬 부드럽고, 맵핑 품질이 눈에 띄게 좋다.

```bash
ros2 run joy joy_node
ros2 run teleop_twist_joy teleop_node --ros-args \
  --params-file src/blind_nav_system/config/mapping/gamepad_xbox.yaml \
  -r /cmd_vel:=/stretch/cmd_vel
```

LB(버튼 4)를 누르고 있어야 움직이는 데드맨이고, 속도는 0.25 m/s · 0.25 rad/s로
낮춰 두었다(빠르면 스캔이 번진다).

⛔ **`~/.local/bin/stretch_gamepad_teleop.py`는 절대 쓰지 마라.** stretch_body를
직접 잡아 **로봇 바디 락과 시리얼을 물어서 런치의 stretch_driver를 죽인다.**
오토스타트(`~/.config/autostart/hello_robot_gamepad_teleop.desktop`)에도 등록돼
있으니, 맵핑 전에 떠 있지 않은지 확인하라.

#### 2.1.5 기존 지도에 이어 찍기 — 지금은 "새로 찍고 이미지 정합"이다

기존 `all.pgm`은 `.posegraph`가 없어 **slam_toolbox 재개가 불가능하다.** 그래서
지금 방법은 새로 SLAM을 돌린 뒤 이미지 수준에서 겹치는 것이다.

2026-09-02에 엘베 캐빈을 붙인 절차:

1. 게임패드로 대상 구역을 새로 SLAM (`cabin_run.pgm`)
2. 기존 맵과 **FFT 상관으로 회전을 전수 탐색**해 정합 (그날 값: 회전 314.0°,
   오프셋 414/216)
3. RViz `Publish Point`로 병합 **경계선을 지정**하고 그 안쪽만 병합
4. 필요하면 GIMP로 손질 (그날은 엘베 문 벽을 지워 홀↔캐빈을 연결)
5. **`all.pgm`을 갈아끼우고 옛것을 `all_backup_YYYYMMDD.pgm`으로 남긴다**

5번이 중요하다. `all.pgm`을 가리키는 참조가 `all.yaml`·`floor1~4.yaml` **다섯
곳**이라, 참조를 고치는 방식은 한 곳만 빠뜨려도 조용히 어긋난다. 파일을 갈아끼우면
고칠 참조가 0개가 된다.

**앞으로 SLAM을 할 때는 `SerializePoseGraph` 서비스로 포즈그래프도 같이 저장하라.**

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '<경로>/맵이름'}"
```

그러면 다음부터는 이미지 정합이 아니라 **진짜 이어찍기**(`mode:=localization` +
`map_file_name`)가 가능해진다. 오늘 겪은 정합 작업 전체가 없어진다.

### 2.2 지도 저장

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/maps/맵이름
```

---

## 3. 내비게이션 실행

### 3.1 전체 스택 한 번에 실행 (권장)

```bash
ros2 launch /home/hello-robot/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/launch/stretch_robot_process.launch.xml
```

포함 내용: stretch_driver, RPLidar, RealSense D435i, map_server, AMCL, Nav2

> 사용할 지도와 파라미터 파일은 런치 파일 내 `map`, `params_file` 인자로 변경

### 3.2 RViz2 실행 및 초기 위치 설정

```bash
# 권장: 사전 구성된 설정으로 실행 (Map TL·RobotModel·LaserScan·도구 포함 — 수동 설정 불필요)
rviz2 -d ~/robot_view.rviz

# (맨 RViz로 열 경우 — 아래 수동 설정 필요)
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

### 3.3 현재 로봇 위치(map 좌표) 확인

```bash
# AMCL 추정 좌표 1회 출력 — location.yaml의 x/y/w 값을 딸 때 사용
# (pose.pose.position.x / .y + pose.pose.orientation.w)
ros2 topic echo /amcl_pose --once

# 대안: TF에서 직접 확인
ros2 run tf2_ros tf2_echo map base_link
```

> 2D Pose Estimate로 초기 위치를 지정하기 전에는 /amcl_pose가 발행되지 않음.

---

## 4. Nav2 파라미터 주요 설정

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

## 5. 배터리 관리

- 24V 이하 → 충전 필요
- LED 노란색(2초 주기) → 저전압 경고
- 충전기: NOCO Genius10 (기본 충전: 12V AGM 모드)

---

## 6. 기타

```bash
# 화면 밝기 조절
xrandr --output HDMI-1 --brightness 1.5
```
