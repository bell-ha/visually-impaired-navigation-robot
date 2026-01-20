import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory
import yaml
import os

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 1. YAML 파일 경로 설정
        # 패키지가 빌드/설치된 경우 'share' 폴더에서 찾고, 
        # 실패하면 현재 Desktop의 소스 경로에서 직접 찾습니다.
        try:
            package_share_dir = get_package_share_directory('blind_nav_system')
            self.yaml_path = os.path.join(package_share_dir, 'config', 'location.yaml')
            if not os.path.exists(self.yaml_path):
                raise Exception("Share directory file not found")
        except Exception:
            # 빌드 전이나 경로가 다를 경우를 대비한 하드코딩된 백업 경로
            self.yaml_path = os.path.expanduser('~/Desktop/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/config/location.yaml')

        self.get_logger().info(f'설정 파일 경로: {self.yaml_path}')

    def load_location(self, name):
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            return data['locations'].get(name)
        except FileNotFoundError:
            self.get_logger().error(f'파일을 찾을 수 없습니다: {self.yaml_path}')
            return None

    def send_goal(self, location_name):
        loc = self.load_location(location_name)
        if not loc:
            self.get_logger().error(f'"{location_name}" 위치를 찾을 수 없습니다!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # YAML에서 가져온 좌표 설정
        goal_msg.pose.pose.position.x = float(loc['x'])
        goal_msg.pose.pose.position.y = float(loc['y'])
        goal_msg.pose.pose.orientation.w = float(loc.get('w', 1.0))

        self.get_logger().info(f'목표 지점 [{location_name}]으로 이동을 요청합니다...')
        
        # Nav2 서버가 켜질 때까지 대기
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action 서버를 찾을 수 없습니다! (navigation.launch.py를 켰는지 확인하세요)')
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('목표가 서버에 의해 거부되었습니다.')
            return
        self.get_logger().info('목표가 수락되었습니다. 이동을 시작합니다!')

def main(args=None):
    rclpy.init(args=args)
    nav_client = NavigationClient()
    
    # 실행 시 바로 B 지점으로 이동 테스트
    # 나중에는 main_state_machine에서 이 함수를 호출하게 됩니다.
    nav_client.send_goal('point_b')
    
    try:
        rclpy.spin(nav_client)
    except KeyboardInterrupt:
        nav_client.get_logger().info('노드를 종료합니다.')
    finally:
        nav_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()