"""
Elevator button inference script.
Called as a subprocess by elevator_node.py; prints JSON to stdout.

Patches sys.modules so that ocr_rcnn_lib's internal "import tensorflow as tf"
gets tensorflow.compat.v1 (TF 2.x backward-compat shim).
"""
import sys
import os
import json
import argparse

# Patch tensorflow → compat.v1 before any ocr_rcnn_lib import
import tensorflow.compat.v1 as tf
tf.disable_eager_execution()
sys.modules['tensorflow'] = tf

import numpy as np
import cv2

OCR_LIB_PATH = os.path.join(
    os.path.dirname(__file__),
    "ocr-rcnn-v2/src/button_recognition/scripts",
)
OCR_RCNN_LIB_PATH = os.path.join(OCR_LIB_PATH, "ocr_rcnn_lib")
sys.path.insert(0, OCR_LIB_PATH)
sys.path.insert(0, OCR_RCNN_LIB_PATH)

from ocr_rcnn_lib.button_recognition import ButtonRecognizer

FROZEN_MODEL_DIR = os.path.join(OCR_LIB_PATH, "ocr_rcnn_lib/frozen_model")
RCNN_PATH = os.path.join(FROZEN_MODEL_DIR, "detection_graph_640x480.pb")
OCR_PATH  = os.path.join(FROZEN_MODEL_DIR, "ocr_graph.pb")

_recognizer = None


def get_recognizer():
    global _recognizer
    if _recognizer is None:
        _recognizer = ButtonRecognizer(rcnn_path=RCNN_PATH, ocr_path=OCR_PATH)
    return _recognizer


def run_inference(image_path):
    image = cv2.imread(image_path)
    if image is None:
        return {"error": f"Cannot read image: {image_path}"}

    # Model requires exactly 640x480
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if image_rgb.shape[:2] != (480, 640):
        image_rgb = cv2.resize(image_rgb, (640, 480))

    recognitions = get_recognizer().predict(image_rgb)

    detections = []
    for box, score, text, belief in recognitions:
        detections.append({
            "score": round(float(score), 3),
            "text": text,
            "belief": round(float(belief), 3),
            "box": {
                "y1": round(float(box[0]) * 480),
                "x1": round(float(box[1]) * 640),
                "y2": round(float(box[2]) * 480),
                "x2": round(float(box[3]) * 640),
            },
        })

    return {"detections": detections}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    args = parser.parse_args()
    print(json.dumps(run_inference(args.image)))


if __name__ == "__main__":
    main()
 