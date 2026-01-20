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
        self.r.energy_threshold = 300 
        self.get_logger().info("=== 음성 인터페이스 준비 완료 ===")

    def speak(self, text):
        try:
            tts = gTTS(text=text, lang='ko')
            tts.save("temp.mp3")
            pygame.mixer.music.load("temp.mp3")
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                time.sleep(0.05)
            pygame.mixer.music.unload()
            os.remove("temp.mp3")
        except Exception as e:
            self.get_logger().error(f"말하기 오류: {e}")

    def listen(self):
        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(source, duration=0.3)
            print("\n[음성 듣는 중...] ", end='', flush=True)
            try:
                audio = self.r.listen(source, timeout=5, phrase_time_limit=4)
                text = self.r.recognize_google(audio, language='ko-KR')
                return text
            except:
                return ""