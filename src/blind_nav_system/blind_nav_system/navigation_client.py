import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import yaml
import os

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        # 목적지 전송 및 정지 명령용 퍼블리셔
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.stop_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.yaml_path = os.path.expanduser('~/Desktop/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/config/location.yaml')

    def load_location(self, name):
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            return data['locations'].get(name)
        except:
            return None

    def send_goal(self, location_name):
        loc = self.load_location(location_name)
        if not loc: return False

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(loc['x'])
        goal_msg.pose.position.y = float(loc['y'])
        goal_msg.pose.orientation.w = float(loc.get('w', 1.0))

        self.publisher.publish(goal_msg)
        return True

    def stop_robot(self, current_pose=None):
        """속도를 0으로 만들고, 현재 위치를 목적지로 덮어씌워 주행 취소"""
        # 1. 물리적 속도 0 전송
        stop_msg = Twist()
        for _ in range(10):
            self.stop_publisher.publish(stop_msg)
            
        # 2. 현재 위치를 목적지로 전송 (가장 확실한 중단 방법)
        if current_pose:
            cancel_msg = PoseStamped()
            cancel_msg.header.frame_id = "map"
            cancel_msg.header.stamp = self.get_clock().now().to_msg()
            cancel_msg.pose = current_pose
            self.publisher.publish(cancel_msg)