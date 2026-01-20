import rclpy
from rclpy.node import Node
import threading
import time
import math
import sys
import select
from nav_msgs.msg import Odometry
from voice_interface import VoiceInterface
from navigation_client import NavigationClient

class MainStateMachine(Node):
    def __init__(self):
        super().__init__('main_state_machine')
        
        self.voice = VoiceInterface()
        self.nav = NavigationClient()
        
        self.current_pose = None
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.nav_thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        self.nav_thread.start()
        
        self.destinations = {"point_a": "에이", "point_b": "비"}
        self.main_control_logic()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def check_interrupt(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return True
        return False

    def main_control_logic(self):
        while rclpy.ok():
            print("\n" + "="*40)
            input("[대기] 엔터를 누르면 시작합니다...")
            
            self.voice.speak("어디로 안내해 드릴까요?")
            choice = input("[선택] 목적지 입력 (a/b): ").lower().strip()
            
            if choice in ['a', 'b']:
                display_name = self.destinations[f"point_{choice}"]
                self.voice.speak(f"{display_name} 지점으로 갈까요? 다시 {choice}를 입력하세요.")
                confirm = input(f"[확인] {choice} 입력 시 출발: ").lower().strip()
                
                if confirm == choice:
                    self.run_navigation(f"point_{choice}", display_name)
                else:
                    self.voice.speak("취소되었습니다.")

    def run_navigation(self, target_key, display_name):
        loc = self.nav.load_location(target_key)
        self.nav.send_goal(target_key)
        self.voice.speak(f"{display_name}로 출발합니다. 멈추려면 아무 키나 누르고 엔터를 치세요.")
        
        print(f"\n>>> {display_name} 주행 중... (멈추려면 엔터!)")

        while rclpy.ok():
            # 1. 중단 체크 (현 위치 전송 방식 적용)
            if self.check_interrupt():
                self.nav.stop_robot(self.current_pose) 
                self.voice.speak("안내를 중단합니다.")
                print("\n[중단] 사용자 요청으로 정지함")
                break

            # 2. 도착 감지 (범위 1.2m)
            if self.current_pose and loc:
                dist = math.sqrt((self.current_pose.position.x - loc['x'])**2 + 
                                 (self.current_pose.position.y - loc['y'])**2)
                print(f"\r남은 거리: {dist:.2f}m", end="", flush=True)

                if dist < 1.2: 
                    print("\n[도착] 목적지에 도달했습니다.")
                    self.voice.speak(f"목적지인 {display_name}에 도착했습니다.")
                    # 도착 시에도 확실히 멈추도록 현 위치 전송
                    self.nav.stop_robot(self.current_pose)
                    break
            
            time.sleep(0.2)

def main(args=None):
    rclpy.init(args=args)
    try:
        MainStateMachine()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()