import rclpy
from rclpy.node import Node
from .voice_interface import VoiceInterface
import threading
import sys
import select
import time

class MainStateMachine(Node):
    def __init__(self):
        super().__init__('main_state_machine')
        self.voice = VoiceInterface()
        
        # 상태 제어 변수
        self.is_running = True
        self.emergency_stop = False
        
        # 목적지 및 긍정어 키워드
        self.keywords_A = ["A", "에이", "1번", "첫번째"]
        self.keywords_B = ["B", "비", "2번", "두번째"]
        self.yes_words = ["맞아", "그래", "네", "응", "어", "출발", "가자", "확인"]

        # 키보드(스페이스바) 입력 감시 쓰레드
        self.input_thread = threading.Thread(target=self.check_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info("=== 안내 로봇 시스템 가동 (스페이스바: 시작/정지) ===")
        self.main_control_logic()

    def check_keyboard_input(self):
        """비동기적으로 키보드 입력을 감시하여 emergency_stop 플래그를 관리"""
        while self.is_running:
            # stdin에 데이터가 있는지 0.1초 간격으로 확인
            if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                line = sys.stdin.readline()
                # 스페이스바(공백 포함 입력) 혹은 엔터 감지
                self.get_logger().warn("!!! 버튼(스페이스바) 입력이 감지되었습니다 !!!")
                self.emergency_stop = True

    def main_control_logic(self):
        """설계 문서 시나리오에 따른 메인 루프"""
        while rclpy.ok():
            self.emergency_stop = False
            self.voice.speak("안녕하세요. 안내를 시작하시려면 버튼을 눌러주세요.")
            
            # [대기 상태] 스페이스바를 누를 때까지 대기
            while not self.emergency_stop:
                time.sleep(0.1)
            
            self.emergency_stop = False # 플래그 초기화
            self.voice.speak("어디로 안내해 드릴까요?")
            
            user_input = self.voice.listen()
            if not user_input:
                self.voice.speak("음성을 듣지 못했습니다. 다시 버튼을 눌러주세요.")
                continue

            # 목적지 판별
            target = None
            if any(w in user_input.upper() for w in self.keywords_A): target = "A"
            elif any(w in user_input.upper() for w in self.keywords_B): target = "B"
            
            if target:
                self.voice.speak(f"{target} 지점으로 안내할까요?")
                confirm = self.voice.listen()
                
                # 음성 긍정어 혹은 버튼 재입력 시 확인으로 간주
                if any(w in confirm for w in self.yes_words) or self.emergency_stop:
                    self.emergency_stop = False
                    self.run_navigation(target)
                else:
                    self.voice.speak("취소되었습니다.")
            else:
                self.voice.speak("등록되지 않은 장소입니다.")

    def run_navigation(self, target):
        """실제 주행 시뮬레이션 및 모니터링"""
        self.voice.speak(f"{target} 지점으로 안내를 시작합니다. 손잡이를 잡아주세요.")
        self.voice.speak("앞으로 직진합니다.")
        
        # 주행 모니터링 루프 (총 10단계 시뮬레이션)
        for i in range(1, 11):
            # 주행 중 언제라도 스페이스바를 누르면 정지
            if self.emergency_stop:
                self.get_logger().error("!!! 비상 정지 발생 !!!")
                self.voice.speak("안내를 중단하고 즉시 정지합니다.")
                return

            # 시나리오: 특정 지점(5단계)에서 45도 이상 회전 안내
            if i == 5:
                self.get_logger().info("회전 구간 진입")
                self.voice.speak("잠시 후 왼쪽으로 크게 회전합니다. 주의하세요.")
                time.sleep(1.5) # 회전하는 시간 시뮬레이션

            print(f"[{target} 방향 주행 중...] {i*10}% 완료", end='\r')
            time.sleep(1.0) # 1초당 1단계 진행

        self.voice.speak(f"목적지인 {target} 지점에 도착했습니다. 안내를 종료합니다.")
        self.get_logger().info("도착 완료 및 대기 상태 전환")

def main(args=None):
    rclpy.init(args=args)
    node = MainStateMachine()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()