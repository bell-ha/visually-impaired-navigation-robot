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
import math

class NavigationClient(Node):
    def __init__(self, target_key):
        super().__init__(f'nav_client_{int(time.time())}')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.stop_publisher = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, '/plan', 10)
        
        # [수정] 경로 대신 실제 속도 명령(Twist)을 구독하여 회전 감지
        self.vel_sub = self.create_subscription(Twist, '/stretch/cmd_vel', self.vel_callback, 10)
        
        self.last_turn_announcement = 0 
        self.turn_threshold = 0.4  # 회전 감지 임계값 (rad/s), 약 23도/s 이상의 회전일 때 알림
        
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
        for srv in ['/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap']:
            cli = self.create_client(Trigger, srv)
            if cli.wait_for_service(timeout_sec=0.5):
                cli.call_async(Trigger.Request())
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

    # === [핵심] 실제 회전 속도 기반 감지 로직 ===
    def vel_callback(self, msg):
        """로봇이 실제로 회전 명령을 받았을 때 즉시 안내"""
        current_time = time.time()
        
        # angular.z는 초당 회전 각도(라디안)입니다.
        angular_z = msg.angular.z 
        
        # 설정한 임계값보다 빠르게 회전 중인지 확인
        if abs(angular_z) > self.turn_threshold:
            # 5초 간격으로 중복 안내 방지
            if current_time - self.last_turn_announcement > 5.0:
                # ROS 표준: +는 왼쪽(시계반대방향), -는 오른쪽(시계방향)
                direction = "왼쪽" if angular_z > 0 else "오른쪽"
                
                # 라디안을 각도로 변환하여 로그 출력
                deg_per_sec = abs(math.degrees(angular_z))
                self.get_logger().info(f"🔄 [동작 안내] {direction}으로 회전 중입니다. (속도: {deg_per_sec:.1f}°/s)")
                self.last_turn_announcement = current_time

    def cleanup(self):
        try:
            empty_path = Path()
            empty_path.header.frame_id = "map"
            self.path_publisher.publish(empty_path)
            stop_msg = Twist()
            for _ in range(5):
                self.stop_publisher.publish(stop_msg)
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
        except: pass