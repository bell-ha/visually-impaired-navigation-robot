"""
Persistent OCR-RCNN inference server.
Loads the model once, then reads image paths from stdin and writes JSON to stdout.
Spawned once at startup by main.py — eliminates per-frame model loading overhead.
"""
import sys, os, json
import tensorflow.compat.v1 as tf
tf.disable_eager_execution()
sys.modules['tensorflow'] = tf

import cv2

OCR_LIB_PATH      = os.path.join(os.path.dirname(__file__),
                                  "ocr-rcnn-v2/src/button_recognition/scripts")
OCR_RCNN_LIB_PATH = os.path.join(OCR_LIB_PATH, "ocr_rcnn_lib")
sys.path.insert(0, OCR_LIB_PATH)
sys.path.insert(0, OCR_RCNN_LIB_PATH)

from ocr_rcnn_lib.button_recognition import ButtonRecognizer

FROZEN_MODEL_DIR = os.path.join(OCR_RCNN_LIB_PATH, "frozen_model")
recognizer = ButtonRecognizer(
    rcnn_path=os.path.join(FROZEN_MODEL_DIR, "detection_graph_640x480.pb"),
    ocr_path =os.path.join(FROZEN_MODEL_DIR, "ocr_graph.pb"),
)

# Signal that the model is loaded and ready
print("READY", flush=True)

for line in sys.stdin:
    path = line.strip()
    if not path:
        continue
    try:
        img = cv2.imread(path)
        if img is None:
            print(json.dumps({"detections": []}), flush=True)
            continue
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if rgb.shape[:2] != (480, 640):
            rgb = cv2.resize(rgb, (640, 480))
        detections = []
        for box, score, text, belief in recognizer.predict(rgb):
            detections.append({
                "score":  round(float(score), 3),
                "text":   text,
                "belief": round(float(belief), 3),
                "box": {
                    "y1": round(float(box[0]) * 480),
                    "x1": round(float(box[1]) * 640),
                    "y2": round(float(box[2]) * 480),
                    "x2": round(float(box[3]) * 640),
                },
            })
        print(json.dumps({"detections": detections}), flush=True)
    except Exception as e:
        print(json.dumps({"detections": [], "error": str(e)}), flush=True)
