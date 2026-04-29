# Elevator Button Detection

Stretch3 카메라로 엘리베이터 버튼을 인식하는 모듈.
OCR-RCNN (frozen graph) → TF 2.x compat.v1 → ROS2 토픽 publish 구조.

## 파일 구조

```
elevator_button_press/
├── setup.sh              # 최초 1회 실행 (venv + 모델 다운로드)
├── requirements.txt      # Python 의존성
├── ocr_rcnn_infer.py     # 추론 스크립트 (venv 내 Python으로 직접 실행 가능)
├── elevator_node.py      # ROS2 노드 (카메라 구독 → 추론 → publish)
└── ocr-rcnn-v2/          # 클론된 외부 repo (gitignore 처리)
```

## 최초 설치 (1회만)

```bash
cd ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/blind_nav_system/elevator_button_press
bash setup.sh
```

설치 내용:
- `~/venv_ocr` 가상환경 생성 (Python 3.10, TF 2.13)
- `ocr-rcnn-v2` 레포 클론
- Google Drive에서 frozen model 다운로드

## 추론 단독 테스트 (ROS2 없이)

```bash
~/venv_ocr/bin/python ocr_rcnn_infer.py --image /path/to/image.jpg
```

출력 예시:
```json
{
  "detections": [
    {"score": 0.923, "class": 1, "box": {"y1": 120, "x1": 80, "y2": 200, "x2": 160}}
  ],
  "image_size": {"width": 640, "height": 480}
}
```

## ROS2 노드 실행

```bash
# 터미널 1: Stretch3 카메라 토픽 확인
ros2 topic list | grep camera

# 터미널 2: 노드 실행
ros2 run blind_nav_system elevator_node
```

구독 토픽: `/camera/color/image_raw`
발행 토픽: `/elevator/button_detections` (JSON String)

결과 확인:
```bash
ros2 topic echo /elevator/button_detections
```

## 의존성 구조

```
ROS2 노드 (Python 3.10, 시스템)
  └─ subprocess 호출
       └─ ~/venv_ocr/bin/python (TF 2.13, 격리된 venv)
            └─ ocr_rcnn_infer.py
```

시스템 Python / ROS2 / PyTorch 환경과 완전히 격리됨.
