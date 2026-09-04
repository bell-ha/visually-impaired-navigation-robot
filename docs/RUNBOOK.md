# 실행 매뉴얼 (RUNBOOK)

로봇 앞에서 바로 보는 문서. 모든 실행 명령어를 여기에 모았다.
하드웨어 사양과 알려진 이슈는 [HARDWARE.md](HARDWARE.md) 참조.

---

## 0. 경로 약어

```bash
REPO=~/GitHub/visually-impaired-navigation-robot
PKG=$REPO/src/blind_nav_system
APP=$PKG/blind_nav_system
```

---

## 1. 로봇 초기화 (처음 한 번)

```bash
stretch_robot_battery_check.py
stretch_free_robot_process.py
stretch_robot_home.py
stretch_robot_stow.py
```

---

## 2. 가상환경 · 의존성

```bash
# 활성화
source /home/hello-robot/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate

# 의존성 설치 (활성화 후)
pip install -r /home/hello-robot/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/requirements.txt
```

`.env` 설정 (필수) — `src/.env`
```
OPENAI_API_KEY=sk-...
```

---

## 3. 네비게이션 스택 실행

**런치는 한 번 켜고 유지한다.** 잦은 재시작은 리얼센스를 얼린다.

```bash
ros2 launch /home/hello-robot/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/launch/stretch_robot_process.launch.xml
```

포함 노드: `stretch_driver` · RPLidar · RealSense D435i · `map_server` · AMCL · Nav2

패키지 경로로 실행할 때:
```bash
ros2 launch blind_nav_system stretch_robot_process.launch.xml
```

---

## 4. RViz2 — 초기 위치 지정

```bash
ros2 run rviz2 rviz2
# RViz에서 2D Pose Estimate로 로봇 초기 위치 지정
```

---

## 5. 음성 인터페이스 (메인)

```bash
source /opt/ros/humble/setup.bash
source ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system
python3 interface.py
```

---

## 6. 관제 대시보드 (오케스트레이터)

`interface.py` + `vision_assistant.py`를 서브프로세스로 띄우고 엘리베이터 자동 여정을 관리한다.

```bash
source /opt/ros/humble/setup.bash
source $PKG/venv/bin/activate
cd $APP
python3 main.py
```

브라우저: `http://localhost:8080`
현장(핫스팟)에서는 `http://<로봇IP>:8080`

---

## 7. 엘리베이터 버튼 조작 모듈

**런치는 유지하고 이 앱만 재시작하면 된다.**

```bash
source /opt/ros/humble/setup.bash
source ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate
python3 ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system/elevator_button_press/main.py
```

브라우저: `http://localhost:5000` (자동으로 열림)
현장에서는 `http://<로봇IP>:5000`

---

## 8. 시각 보조 (선택)

```bash
source /opt/ros/humble/setup.bash
source ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system
python3 vision_assistant.py
# Enter 키를 누르면 카메라로 전방을 분석해 음성으로 안내
```

---

## 9. 사람 추적 (people_tracker)

별도 프로세스로 실행한다.

```bash
cd $APP/people_tracker
./run.sh
```

발행 토픽
- `/people_tracker/markers` — RViz 시각화 (빨강 접근자 / 초록 동행자 / 파랑 정지)
- `/people_tracker/people` — JSON (위치 · 속도 · 분류 · 거리 · depth_estimated)

---

## 10. 엘리베이터 캐빈 매핑

**런치는 켜둔 채로 둔다.** AMCL이 살아 있어야 앵커를 얻는다.
`cabin_capture.py`는 `cmd_vel`을 발행하지 않는다 — 이동은 100% 사람(게임패드)이 한다.

```bash
# 게임패드
ros2 run joy joy_node
ros2 run teleop_twist_joy teleop_node \
    --ros-args --params-file <xbox.config.yaml> \
    -r /cmd_vel:=/stretch/cmd_vel

# 캡처 (기록 전용)
cd $APP/cabin_mapping
python3 cabin_capture.py

# 렌더 (오프라인, 몇 번이든 재실행 가능)
python3 cabin_render.py
```

---

## 11. 진단 · 디버그 도구

| 명령 | 용도 |
|---|---|
| `python3 $APP/robot_diag.py` | 로봇 상태 진단 |
| `python3 $APP/robot_diag_nav.py` | 주행 관련 진단 (`/rosout` 실패 이력) |
| `python3 $APP/tools/audio_web_test.py` | 오디오 진단 웹 툴 (마이크 레벨 · 스피커 440Hz) |
| `python3 $APP/tools/mouse_teleop.py` | 마우스 텔레오퍼레이션 (스크롤 전후진 · 버튼 회전) |
| `python3 $APP/tools/armleft.py` | 팔 · 리프트 제어 |
| `python3 $APP/tools/run_test_panels.py` | 엘리베이터 버튼 패널 배치 추론 · 결과 이미지 저장 |
| `python3 $APP/tools/hardware/input.py` | 아두이노 시리얼 읽기 · 시각화 |
| `python3 $APP/tools/hardware/input_bridge.py` | 압력 센서 → Python 브릿지 |

---

## 12. 설정 파일

| 파일 | 내용 |
|---|---|
| `$PKG/config/location.yaml` | 목적지 이름 → Nav2 좌표 매핑 |
| `$PKG/maps/*.pgm` + `*.yaml` | 사전 생성 실내 지도 (`all` · `itrc` · `coex` · `floor3` · `floor4` · `center` · `cabin_run`) |
| `$APP/elevator_button_press/button_layout.json` | 버튼 패널 배치 |
| `$APP/elevator_button_press/aim_trim.json` | 조준 보정값 |
| `~/.ros/fastdds_no_shm.xml` | FastDDS SHM 비활성 (UDP 강제) |

---

## 13. 자주 겪는 문제

| 증상 | 조치 |
|---|---|
| 카메라가 USB에서 사라짐 | 웜 리부트 무효. **종료 후 메인 전원 30초 차단(콜드 리셋)** |
| 리얼센스가 얼음 | 런치 재시작을 줄이고 **앱만** 재시작 |
| body/gripper 카메라가 뒤바뀜 | 런치에서 `serial_no` 고정 확인 ([HARDWARE.md](HARDWARE.md)) |
| 유령 참가자 노드 | `~/.ros/fastdds_no_shm.xml` 적용 확인 |
| 배터리 `pct` NaN | voltage 기준으로 판단 |
| 주행이 자꾸 멈춤 | 뎁스 포인트클라우드 UDP 부하 → transform 캐시 드롭 확인 |
| 아두이노 포트 못 찾음 | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0` |

---

## 14. 관련 문서

- [HARDWARE.md](HARDWARE.md) — 하드웨어 사양 · 카메라 시리얼 · 물리 제약
- [../navigation-and-slam-guide.md](../navigation-and-slam-guide.md) — Navigation & SLAM 가이드
- [../src/blind_nav_system/blind_nav_system/elevator_button_press/readme.md](../src/blind_nav_system/blind_nav_system/elevator_button_press/readme.md)
- [../src/blind_nav_system/blind_nav_system/people_tracker/README.md](../src/blind_nav_system/blind_nav_system/people_tracker/README.md)
- [../src/blind_nav_system/blind_nav_system/cabin_mapping/README.md](../src/blind_nav_system/blind_nav_system/cabin_mapping/README.md)
- [../src/blind_nav_system/blind_nav_system/interface-spec.md](../src/blind_nav_system/blind_nav_system/interface-spec.md)
