#!/usr/bin/env python3
"""
mouse_teleop.py
- 스크롤 위   : 전진
- 스크롤 아래 : 후진
- 좌클릭 누름 : 좌회전
- 우클릭 누름 : 우회전
- 중간 클릭   : 정지

실행:
  python3 mouse_teleop.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import mouse

LINEAR_SPEED  = 0.3   # m/s
ANGULAR_SPEED = 0.5   # rad/s
DECAY         = 0.7   # 스크롤 후 속도 감쇠 (0.1s마다 곱함)


class MouseTeleop(Node):
    def __init__(self):
        super().__init__('mouse_teleop')
        self.pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self._publish)

        self.linear_x  = 0.0
        self.left_held  = False
        self.right_held = False

        self._listener = mouse.Listener(
            on_scroll=self._on_scroll,
            on_click=self._on_click,
        )
        self._listener.start()

        self.get_logger().info(
            'Mouse Teleop 시작!\n'
            '  스크롤 위/아래 : 전진/후진\n'
            '  좌클릭(누름)   : 좌회전\n'
            '  우클릭(누름)   : 우회전\n'
            '  중간 클릭      : 정지\n'
            '  Ctrl+C         : 종료'
        )

    def _on_scroll(self, x, y, dx, dy):
        if dy > 0:
            self.linear_x = LINEAR_SPEED
        elif dy < 0:
            self.linear_x = -LINEAR_SPEED

    def _on_click(self, x, y, button, pressed):
        if button == mouse.Button.left:
            self.left_held = pressed
        elif button == mouse.Button.right:
            self.right_held = pressed
        elif button == mouse.Button.middle and pressed:
            self.linear_x  = 0.0
            self.left_held  = False
            self.right_held = False

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.linear_x

        if self.left_held:
            msg.angular.z = ANGULAR_SPEED
        elif self.right_held:
            msg.angular.z = -ANGULAR_SPEED

        self.pub.publish(msg)

        # 스크롤 입력은 한 번만 트리거되므로 서서히 감속
        self.linear_x *= DECAY
        if abs(self.linear_x) < 0.01:
            self.linear_x = 0.0


def main():
    rclpy.init()
    node = MouseTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._listener.stop()
        # 정지 명령 전송
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
