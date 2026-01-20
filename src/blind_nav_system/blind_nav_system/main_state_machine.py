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
        
        # 목적지 정의
        self.keywords_A = ["A", "에이", "1번", "첫번째"]
        self.keywords_B = ["B", "비", "2번", "두번째"]
        
        # 긍정어 기본 키워드
        self.yes_words = ["맞아", "그래", "네", "응", "어", "출발", "가자", "확인"]

        # 키보드(스페이스바) 입력 감시 쓰레드
        self.input_thread = threading.Thread(target=self.check_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info("=== 시스템 대기 중 (스페이스바를 누르면 시작합니다) ===")
        self.main_control_logic()

    def check_keyboard_input(self):
        while self.is_running:
            if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                line = sys.stdin.readline()
                self.emergency_stop = True

    def main_control_logic(self):
        while rclpy.ok():
            self.emergency_stop = False
            print("\n[대기 모드] 안내를 시작하려면 스페이스바를 누르세요...", end='', flush=True)
            
            while not self.emergency_stop:
                time.sleep(0.1)
            
            self.emergency_stop = False 
            self.voice.speak("안녕하세요. 어디로 안내해 드릴까요?")
            
            user_input = self.voice.listen()
            if not user_input:
                self.voice.speak("음성을 듣지 못했습니다. 다시 버튼을 눌러주세요.")
                continue

            # 1. 목적지 판별 및 해당 목적지의 키워드 리스트 보관
            target = None
            target_keywords = []

            if any(w in user_input.upper() for w in self.keywords_A):
                target = "A"
                target_keywords = self.keywords_A
            elif any(w in user_input.upper() for w in self.keywords_B):
                target = "B"
                target_keywords = self.keywords_B
            
            if target:
                self.voice.speak(f"{target} 지점으로 안내할까요?")
                confirm = self.voice.listen()
                
                # [개선된 긍정 판단 로직]
                # 1. 기본 긍정어(네, 그래 등)가 포함되어 있거나
                # 2. 이전에 말했던 목적지 키워드(A, 에이 등)를 다시 말하거나
                # 3. 스페이스바를 다시 눌렀을 때
                is_confirmed = (
                    any(w in confirm for w in self.yes_words) or 
                    any(w in confirm.upper() for w in target_keywords) or
                    self.emergency_stop
                )

                if is_confirmed:
                    self.emergency_stop = False
                    self.run_navigation(target)
                else:
                    self.voice.speak("취소되었습니다. 다시 시작하려면 버튼을 눌러주세요.")
            else:
                self.voice.speak("등록되지 않은 장소입니다. 다시 시작하려면 버튼을 눌러주세요.")

    def run_navigation(self, target):
        self.voice.speak(f"{target} 지점으로 안내를 시작합니다. 손잡이를 잡아주세요.")
        self.voice.speak("앞으로 직진합니다.")
        
        for i in range(1, 11):
            if self.emergency_stop:
                self.voice.speak("버튼 입력이 돼서 안내를 중단하고 즉시 정지합니다.")
                return

            if i == 5:
                self.voice.speak("잠시 후 왼쪽으로 크게 회전합니다.")
                time.sleep(1.0)

            print(f"[{target} 주행 중...] {i*10}%", end='\r')
            time.sleep(1.0)

        self.voice.speak(f"목적지에 도착했습니다. 안내를 종료합니다.")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MainStateMachine()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()