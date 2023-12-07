import sys
import threading

# Speech to text package
import speech_recognition
# Text to speech
import pyttsx3
# Json utils
import json
from constant import *
import LidarModule.main as LidarModule

class Assistant:
    def __init__(self):
        # Speech recognizer
        self.recognizer = speech_recognition.Recognizer()
        # Voice response instant
        self.speaker = pyttsx3.init()
        # Set tốc đô nói (words per minute)
        self.speaker.setProperty("rate", 120)
        # Set giọng nói
        voices = self.speaker.getProperty("voices")
        self.speaker.setProperty("voice", voices[1].id)
        self.intent_data = self.load_json(INTENT_LOCATION)["intents"]

        self.lidar_system = LidarModule.LidarDetection("COM5")

        self.run_assistant()

    def run_assistant(self):
        print("Start, you can say")
        while (True):
            try:
                # Use Microphone resource to listen
                with speech_recognition.Microphone() as mic:
                    # Tự điều chỉnh độ ồn của mic
                    self.recognizer.adjust_for_ambient_noise(mic, duration=0.2)
                    # Lắng nghe tiếng nói
                    text = self.get_text_from_speech(mic)

                    # Cmd khởi động
                    if WAKE_WORD in text:
                        self.speaker.say(RESPONSE_WELCOME)
                        self.speaker.runAndWait()

                        text = self.get_text_from_speech(mic)
                        print(text)

                        if "khởi động" in text:
                            self.speaker.say("Khởi động hệ thống")
                            self.speaker.runAndWait()

                            self.main_operation(mic)
                            # self.speaker.stop()
                            # self.root.destroy()
                            # sys.exit(0)
                        else:
                            if text is not None:
                                response = self.query_response(text)
                                if response is not None:
                                    self.speaker.say(response)
                                    self.speaker.runAndWait()
                                else:
                                    self.speaker.say(NOT_UNDERSTAND)
                                    self.speaker.runAndWait()

            except speech_recognition.UnknownValueError:
                print("UnknownValueError")
                continue

    def load_json(self, path):
        with open(path, encoding="utf-8") as file:
            data = json.load(file)
        return data

    def query_response(self, text):
        for intent in self.intent_data:
            for pattern in intent["patterns"]:
                if text in pattern:
                    return intent
        return None

    def get_text_from_speech(self, mic):
        audio = self.recognizer.listen(mic, timeout=6, phrase_time_limit=5)
        text = self.recognizer.recognize_google(audio, language="vi-VN", show_all=False)
        text = text.lower()
        return text

    def main_operation(self, mic):
        while (True):
            try:
                text = self.get_text_from_speech(mic)
                print(text)
                if text is not None:
                    if "kết thúc" in text:
                        print("Back to waiting mode")
                        self.speaker.say("Dừng hệ thống")
                        self.speaker.runAndWait()
                        break
                    else:
                        intent = self.query_response(text)
                        if intent["response"] != "":
                            self.speaker.say(intent["response"])
                            self.speaker.runAndWait()
                        if intent["action"] != "":
                            getattr(self, intent["action"])()

            except speech_recognition.UnknownValueError:
                continue

    def test(self):
        self.speaker.say("xin cảm ơn")
        self.speaker.runAndWait()

    def run_lidar(self):
        self.lidar_system.run_system()


def main():
    assistant = Assistant()

if __name__ == "__main__":
    main()