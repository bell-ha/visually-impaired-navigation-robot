"""
Elevator button inference script using OCR-RCNN.
Uses TF 2.x compat.v1 mode to run the TF 1.x frozen graph.
Called as a subprocess by elevator_node.py; prints JSON to stdout.
"""
import sys
import os
import json
import argparse

import numpy as np
import cv2
import tensorflow.compat.v1 as tf

tf.disable_eager_execution()

OCR_LIB_PATH = os.path.join(os.path.dirname(__file__), "ocr-rcnn-v2/src/button_recognition/scripts")
sys.path.insert(0, OCR_LIB_PATH)

DEFAULT_MODEL = os.path.join(
    os.path.dirname(__file__),
    "ocr-rcnn-v2/src/button_recognition/scripts/ocr_rcnn_lib/frozen_inference_graph.pb",
)


def load_graph(frozen_graph_path):
    graph = tf.Graph()
    with graph.as_default():
        graph_def = tf.GraphDef()
        with tf.gfile.GFile(frozen_graph_path, "rb") as f:
            graph_def.ParseFromString(f.read())
            tf.import_graph_def(graph_def, name="")
    return graph


def run_inference(image_path, frozen_graph_path):
    image = cv2.imread(image_path)
    if image is None:
        return {"error": f"Cannot read image: {image_path}"}

    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    h, w = image_rgb.shape[:2]

    graph = load_graph(frozen_graph_path)
    with graph.as_default():
        with tf.Session(graph=graph) as sess:
            image_tensor = graph.get_tensor_by_name("image_tensor:0")
            boxes = graph.get_tensor_by_name("detection_boxes:0")
            scores = graph.get_tensor_by_name("detection_scores:0")
            classes = graph.get_tensor_by_name("detection_classes:0")
            num_detections = graph.get_tensor_by_name("num_detections:0")

            out_boxes, out_scores, out_classes, out_num = sess.run(
                [boxes, scores, classes, num_detections],
                feed_dict={image_tensor: np.expand_dims(image_rgb, axis=0)},
            )

    detections = []
    for i in range(int(out_num[0])):
        score = float(out_scores[0][i])
        if score < 0.5:
            continue
        box = out_boxes[0][i]
        detections.append({
            "score": round(score, 3),
            "class": int(out_classes[0][i]),
            "box": {
                "y1": float(box[0]) * h,
                "x1": float(box[1]) * w,
                "y2": float(box[2]) * h,
                "x2": float(box[3]) * w,
            },
        })

    return {"detections": detections, "image_size": {"width": w, "height": h}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    args = parser.parse_args()

    result = run_inference(args.image, args.model)
    print(json.dumps(result))


if __name__ == "__main__":
    main()
