"""
Run OCR-RCNN inference on all test panel images and save visualized results.
"""
import sys
import os
import time
import json

import numpy as np
import cv2
import tensorflow.compat.v1 as tf
tf.disable_eager_execution()
sys.modules['tensorflow'] = tf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OCR_LIB_PATH = os.path.join(SCRIPT_DIR, "ocr-rcnn-v2/src/button_recognition/scripts")
OCR_RCNN_LIB_PATH = os.path.join(OCR_LIB_PATH, "ocr_rcnn_lib")
sys.path.insert(0, OCR_LIB_PATH)
sys.path.insert(0, OCR_RCNN_LIB_PATH)

from ocr_rcnn_lib.button_recognition import ButtonRecognizer

FROZEN_MODEL_DIR = os.path.join(OCR_RCNN_LIB_PATH, "frozen_model")
TEST_PANELS_DIR  = os.path.join(OCR_RCNN_LIB_PATH, "test_panels")
RESULTS_DIR      = os.path.join(SCRIPT_DIR, "results")

os.makedirs(RESULTS_DIR, exist_ok=True)


def draw_results(image_rgb, recognitions):
    vis = image_rgb.copy()
    for box, score, text, belief in recognitions:
        y1 = int(box[0] * 480)
        x1 = int(box[1] * 640)
        y2 = int(box[2] * 480)
        x2 = int(box[3] * 640)

        cv2.rectangle(vis, (x1, y1), (x2, y2), (50, 220, 100), 2)

        label = f"{text} ({score:.2f})"
        font_scale = max(0.4, min(0.7, (x2 - x1) / 80))
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)

        # 텍스트 배경
        bg_y1 = max(y1 - th - 6, 0)
        cv2.rectangle(vis, (x1, bg_y1), (x1 + tw + 4, y1), (50, 220, 100), -1)
        cv2.putText(vis, label, (x1 + 2, y1 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), 1, cv2.LINE_AA)
    return vis


def main():
    print("모델 로딩 중...")
    t_load = time.time()
    recognizer = ButtonRecognizer(
        rcnn_path=os.path.join(FROZEN_MODEL_DIR, "detection_graph_640x480.pb"),
        ocr_path=os.path.join(FROZEN_MODEL_DIR, "ocr_graph.pb"),
    )
    load_time = time.time() - t_load
    print(f"모델 로드 완료: {load_time:.2f}s\n")

    images = sorted([
        f for f in os.listdir(TEST_PANELS_DIR)
        if f.lower().endswith(('.jpg', '.png'))
    ])

    total_times = []
    summary = []

    for fname in images:
        img_path = os.path.join(TEST_PANELS_DIR, fname)
        image = cv2.imread(img_path)
        if image is None:
            continue

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if image_rgb.shape[:2] != (480, 640):
            image_rgb = cv2.resize(image_rgb, (640, 480))

        t0 = time.time()
        recognitions = recognizer.predict(image_rgb)
        elapsed = time.time() - t0
        total_times.append(elapsed)

        vis = draw_results(image_rgb, recognitions)
        vis_bgr = cv2.cvtColor(vis, cv2.COLOR_RGB2BGR)

        # 우측 상단에 타이밍 정보
        timing_label = f"{elapsed*1000:.0f}ms | {len(recognitions)} buttons"
        cv2.rectangle(vis_bgr, (0, 0), (260, 28), (30, 30, 30), -1)
        cv2.putText(vis_bgr, timing_label, (6, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 100), 1, cv2.LINE_AA)

        out_name = os.path.splitext(fname)[0] + "_result.jpg"
        out_path = os.path.join(RESULTS_DIR, out_name)
        cv2.imwrite(out_path, vis_bgr)

        texts = [r[2] for r in recognitions]
        summary.append((fname, len(recognitions), elapsed, texts))
        print(f"  {fname:15s} | {len(recognitions):3d}개 | {elapsed*1000:6.1f}ms | {texts}")

    print(f"\n{'='*60}")
    print(f"총 {len(images)}장 처리 완료")
    print(f"평균 추론 시간: {sum(total_times)/len(total_times)*1000:.1f}ms")
    print(f"최소: {min(total_times)*1000:.1f}ms  최대: {max(total_times)*1000:.1f}ms")
    print(f"결과 저장 위치: {RESULTS_DIR}")


if __name__ == "__main__":
    main()
