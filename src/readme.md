

### 📂 시각장애인 보행 보조 시스템 구조
* **`urdf/`**: 로봇의 물리적 변화를 담당합니다.
* **`human_safety_zone.xacro`**: 사용자가 서 있는 공간을 로봇의 일부(충돌 영역)로 등록하여 안전 거리를 확보합니다.

* **`../files/`**: 프로젝트 관련 발표 자료가 저장되어 있습니다.
  * `시각장애인안내로봇.pptx`: 시스템 소개 발표 자료 (이전 PDF에서 PPTX로 교체됨)


* **`blind_nav_system/`**: 로봇의 지능과 동작을 담당하는 ROS 2 패키지입니다.
* **`blind_nav_system/` (내부)**: 실제 파이썬 로직들이 들어있습니다.
  * `main_state_machine.py`: 전체 시나리오(인사-목적지-이동)를 총괄하는 뇌 역할을 합니다.
  * `main_state_machine_backup.py`: 상태 머신 백업본입니다.
  * `interface.py`: 음성 인식·합성, 사용자 안내 등 인터페이스 전반을 담당합니다. (v3, ROS 없이 단독 테스트 가능) ← 기존 `voice_interface.py` 대체
  * `navigation_client.py`: 목적지 좌표로 로봇을 실제로 움직이는 다리 역할을 합니다.
  * `sensor_monitor.py`: 버튼 입력이나 센서 상태를 감시하는 감각 역할을 합니다.
  * `armleft.py`: Stretch SE3 로봇의 팔(arm)과 리프트(lift)를 목표 위치로 이동·고정하는 제어 노드입니다.
  * `interface.md`: 인터페이스 기능 명세서입니다.
  * **`hardware/`**: 물리적 입력 장치(압력 센서 등) 관련 코드가 들어있습니다.
    * `input.py`: 시리얼(/dev/ttyUSB0)로 압력 센서 데이터를 읽어 당김/잡음을 감지하고 실시간 시각화합니다.
    * `input_bridge.py`: 압력 센서 시리얼 데이터를 읽어 Python 로직으로 전달하는 브릿지입니다.
    * `signal_to_python/signal_to_python.ino`: 센서 신호를 시리얼로 전송하는 아두이노 스케치입니다.


* **`config/`**: 좌표값(`location.yaml`, `location_backup.yaml`)과 파라미터(`params.yaml`)를 관리합니다.
* **`launch/`**: `start_guide.launch.py`를 통해 위 노드들을 한 번에 실행합니다.
* **`maps/`**: 로봇이 길을 찾을 때 사용하는 지도 파일들이 저장되어 있습니다.



---

### � 향후 연구 방향 (To-Do)

* **RT-2 (Robotics Transformer 2)**: 비전-언어-액션 모델 적용 가능성 조사
* **VLM (Vision Language Model)**: 시각 정보 기반 자연어 안내 기능 연구

> 상세 항목은 [`docs/todo.md`](../docs/todo.md)에서 관리됩니다.

---

