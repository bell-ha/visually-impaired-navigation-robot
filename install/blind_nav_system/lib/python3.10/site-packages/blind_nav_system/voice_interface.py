import rclpy
from rclpy.node import Node
import speech_recognition as sr
from gtts import gTTS
import pygame
import os
import time

class VoiceInterface(Node):
    def __init__(self):
        super().__init__('voice_interface')
        pygame.mixer.init()
        self.r = sr.Recognizer()
        # [개선 1] 주변 소음 적응 수치를 고정하여 대기 시간 삭제
        self.r.energy_threshold = 300 
        self.get_logger().info("=== 음성 인터페이스 준비 완료 ===")

    def speak(self, text):
        try:
            self.get_logger().info(f"말하기: {text}")
            tts = gTTS(text=text, lang='ko')
            tts.save("temp.mp3")
            pygame.mixer.music.load("temp.mp3")
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                time.sleep(0.05) # [개선 2] 체크 주기를 좁혀 더 빠르게 반응
            pygame.mixer.music.unload()
            os.remove("temp.mp3")
        except Exception as e:
            self.get_logger().error(f"말하기 오류: {e}")

    def listen(self):
        with sr.Microphone() as source:
            # [개선 3] 적응 시간을 1.0 -> 0.3초로 대폭 단축
            self.r.adjust_for_ambient_noise(source, duration=0.3)
            print("\n[듣는 중...] ", end='', flush=True)
            try:
                # phrase_time_limit을 주어 너무 길게 기다리지 않게 설정
                audio = self.r.listen(source, timeout=5, phrase_time_limit=4)
                text = self.r.recognize_google(audio, language='ko-KR')
                self.get_logger().info(f"인식 결과: {text}")
                return text
            except:
                return ""

def main(args=None):
    rclpy.init(args=args)
    node = VoiceInterface()
    
    # 이제 훨씬 매끄럽게 대화가 이어질 것입니다.
    node.speak("목적지를 말씀해 주세요.")
    result = node.listen()
    
    if result:
        node.speak(f"{result}로 이동할까요?")
    
    node.destroy_node()
    rclpy.shutdown()