#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$HOME/venv_ocr"
OCR_DIR="$SCRIPT_DIR/ocr-rcnn-v2"

echo "=== [1/4] 가상환경 생성: $VENV_DIR ==="
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
else
    echo "venv already exists"
fi

echo "=== [2/4] 패키지 설치 ==="
"$VENV_DIR/bin/pip" install --upgrade pip
"$VENV_DIR/bin/pip" install -r "$SCRIPT_DIR/requirements.txt"

echo "=== [3/4] ocr-rcnn-v2 클론 ==="
if [ ! -d "$OCR_DIR" ]; then
    git clone https://github.com/zhudelong/ocr-rcnn-v2.git "$OCR_DIR"
else
    echo "ocr-rcnn-v2 already cloned"
fi

echo "=== [4/4] 모델 다운로드 ==="
MODEL_DIR="$OCR_DIR/src/button_recognition/scripts/ocr_rcnn_lib"
mkdir -p "$MODEL_DIR"
if [ ! -f "$MODEL_DIR/frozen_inference_graph.pb" ]; then
    cd "$SCRIPT_DIR"
    "$VENV_DIR/bin/python" -m gdown "https://drive.google.com/file/d/1FVXI-G-EsCrkKbknhHL-9Y1pBshY7JCv/view?usp=sharing"
    unzip frozen_model.zip -d "$MODEL_DIR/"
    rm frozen_model.zip
else
    echo "Model already downloaded"
fi

echo ""
echo "=== 설치 완료 ==="
echo "추론 테스트: $VENV_DIR/bin/python $SCRIPT_DIR/ocr_rcnn_infer.py --image <이미지경로>"
echo "ROS2 노드:   ros2 run blind_nav_system elevator_node"
