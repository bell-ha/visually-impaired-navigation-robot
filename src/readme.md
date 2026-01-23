

### 📂 시각장애인 보행 보조 시스템 구조
* **`urdf/`**: 로봇의 물리적 변화를 담당합니다.
* **`human_safety_zone.xacro`**: 사용자가 서 있는 공간을 로봇의 일부(충돌 영역)로 등록하여 안전 거리를 확보합니다.


* **`blind_nav_system/`**: 로봇의 지능과 동작을 담당하는 ROS 2 패키지입니다.
* **`blind_nav_system/` (내부)**: 실제 파이썬 로직들이 들어있습니다.
* `main_state_machine.py`: 전체 시나리오(인사-목적지-이동)를 총괄하는 뇌 역할을 합니다.
* `voice_interface.py`: 사용자의 음성을 듣고 한국어로 대답하는 입과 귀 역할을 합니다.
* `navigation_client.py`: 목적지 좌표로 로봇을 실제로 움직이는 다리 역할을 합니다.
* `sensor_monitor.py`: 버튼 입력이나 센서 상태를 감시하는 감각 역할을 합니다.


* **`config/`**: 좌표값(`location.yaml`)과 파라미터(`params.yaml`)를 관리합니다.
* **`launch/`**: `start_guide.launch.py`를 통해 위 노드들을 한 번에 실행합니다.
* **`maps/`**: 로봇이 길을 찾을 때 사용하는 지도 파일들이 저장되어 있습니다.



---

### 🛠 현재 구조에서 주의할 점

1. **Xacro 경로 연결**:
`stretch_main.xacro`에서 `include` 하실 때, 경로를 반드시 현재 위치인 `/home/hello-robot/GitHub/visually-impaired-navigation-robot/src/urdf/human_safety_zone.xacro`로 정확히 지정해야 합니다. (`src/`가 중간에 포함됨)
2. **Git 관리**:
현재 `build/`, `install/`, `log/` 폴더들이 소스 폴더 안에 생성되어 있습니다. 이 폴더들은 용량이 크고 실행 환경마다 다르므로, `git add` 하기 전에 `.gitignore` 파일에 추가하거나 해당 폴더들을 제외하고 커밋하는 것이 좋습니다. 만약 실수가 생기면 아까 배운 **`git stash`**로 작업을 되돌릴 수 있습니다.
3. **빌드 및 반영**:
수정된 구조를 적용하려면 로봇 본체에서 아래 명령어를 실행해야 합니다.
```bash
colcon build --packages-select blind_nav_system
source install/setup.bash

```

