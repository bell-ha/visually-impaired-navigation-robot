import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import threading
import time
import math
import sys
import select
from nav_msgs.msg import Odometry
from navigation_client import NavigationClient

class MainManager(Node):
    def __init__(self, executor):
        super().__init__('main_manager')
        self.current_pose = None
        self.nav_node = None 
        self.executor = executor # 외부에서 주입받은 실행기 사용
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.executor.add_node(self)
        
        # 백그라운드 스레드에서만 ROS 처리
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def check_interrupt(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            sys.stdin.readline()
            return True
        return False

    def run(self):
        try:
            while rclpy.ok():
                print("\n" + "="*45)
                print("[상태: 대기] 엔터를 누르면 시작합니다.")
                input(">> ")
                
                target = input("[입력] 목적지 (a/b): ").lower().strip()
                if target not in ['a', 'b']: continue

                # === [주행 객체 생성 및 할당] ===
                self.nav_node = NavigationClient(f"point_{target}")
                self.executor.add_node(self.nav_node) 
                
                if self.nav_node.start_navigation():
                    self.manage_navigation(f"point_{target}")
                
                # === [주행 객체 삭제 및 해제] ===
                self.stop_and_destroy_nav()

        except KeyboardInterrupt: pass
        finally: self.stop_and_destroy_nav()

    def manage_navigation(self, target_key):
        print(f"\n>>> [주행 상태] {target_key} 이동 중 (중단: ENTER)")
        time.sleep(1.0)
        while self.check_interrupt(): pass
        
        while rclpy.ok():
            if self.check_interrupt():
                print("\n[알림] 사용자가 중단했습니다. 리셋 중...")
                break

            if self.nav_node.is_arrived:
                print("\n[완료] 목적지에 도착했습니다.")
                break

            if self.current_pose:
                loc = self.nav_node.load_location()
                dist = math.sqrt((self.current_pose.position.x - loc['x'])**2 + 
                                 (self.current_pose.position.y - loc['y'])**2)
                print(f"\r남은 거리: {dist:.2f}m", end="", flush=True)
                if dist < 0.2: break

            time.sleep(0.1)

    def stop_and_destroy_nav(self):
        if self.nav_node:
            self.nav_node.cleanup()
            self.executor.remove_node(self.nav_node)
            self.nav_node.destroy_node()
            self.nav_node = None
            print("[시스템] 주행 모듈 완전 리셋 완료.")

def main(args=None):
    rclpy.init(args=args)
    # 실행기를 먼저 만들고 주입함 (NoneType 에러 방지)
    exec_obj = SingleThreadedExecutor()
    manager = MainManager(exec_obj)
    manager.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()