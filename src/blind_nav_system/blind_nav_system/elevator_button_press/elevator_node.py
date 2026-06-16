import os
import json
import subprocess
import tempfile

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

VENV_PYTHON = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../venv/bin/python3"))
INFER_SCRIPT = os.path.join(os.path.dirname(__file__), "ocr_rcnn_infer.py")


class ElevatorButtonNode(Node):
    def __init__(self):
        super().__init__("elevator_button_node")
        self.bridge = CvBridge()
        self.result_pub = self.create_publisher(String, "/elevator/button_detections", 10)
        # Stretch3 wrist camera (D405)
        self.sub = self.create_subscription(
            Image, "/gripper_camera/color/image_raw", self.image_callback, 10
        )
        self._processing = False
        self.get_logger().info("ElevatorButtonNode started — subscribing to /gripper_camera/color/image_raw")

    def image_callback(self, msg):
        if self._processing:
            return
        self._processing = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as f:
                tmp_path = f.name
            cv2.imwrite(tmp_path, cv_image)

            result = subprocess.run(
                [VENV_PYTHON, INFER_SCRIPT, "--image", tmp_path],
                capture_output=True,
                text=True,
                timeout=10,
            )
            os.unlink(tmp_path)

            if result.returncode == 0:
                out_msg = String()
                out_msg.data = result.stdout.strip()
                self.result_pub.publish(out_msg)
                detections = json.loads(result.stdout).get("detections", [])
                for d in detections:
                    self.get_logger().info(f"Button: '{d['text']}' score={d['score']}")
            else:
                self.get_logger().error(f"Inference error: {result.stderr[:200]}")
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
        finally:
            self._processing = False


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorButtonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
