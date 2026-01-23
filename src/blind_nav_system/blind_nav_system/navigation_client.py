import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
import yaml
import os
import time

class NavigationClient(Node):
    def __init__(self, target_key):
        # 고유한 노드 이름 생성
        super().__init__(f'nav_client_{int(time.time())}')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.stop_publisher = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, '/plan', 10)
        
        self.target_key = target_key
        self.goal_handle = None
        self.is_arrived = False
        self.yaml_path = os.path.expanduser('~/Desktop/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/config/location.yaml')

    def load_location(self):
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            return data['locations'].get(self.target_key)
        except: return None

    def start_navigation(self):
        loc = self.load_location()
        if not loc: return False

        # 1. 코스트맵 초기화
        for srv in ['/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap']:
            cli = self.create_client(Trigger, srv)
            if cli.wait_for_service(timeout_sec=0.5):
                cli.call_async(Trigger.Request())

        # 2. 액션 서버 연결
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(loc['x'])
        goal_msg.pose.pose.position.y = float(loc['y'])
        goal_msg.pose.pose.orientation.w = float(loc.get('w', 1.0))

        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            self.goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.is_arrived = True

    def cleanup(self):
        """삭제 전 초기화"""
        try:
            # 경로 삭제
            empty_path = Path()
            empty_path.header.frame_id = "map"
            self.path_publisher.publish(empty_path)
            # 물리적 정지
            stop_msg = Twist()
            for _ in range(5):
                self.stop_publisher.publish(stop_msg)
            # 액션 취소
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
        except: pass