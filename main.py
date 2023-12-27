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

from zalo_tts import ZaloTTS

class Assistant:
    def __init__(self):
        # Speech recognizer
        self.recognizer = speech_recognition.Recognizer()
        # Voice response instant
        #self.speaker = pyttsx3.init()
        # Set tốc đô nói (words per minute)
        #self.speaker.setProperty("rate", 120)
        # Set giọng nói
        #voices = self.speaker.getProperty("voices")
        #self.speaker.setProperty("voice", voices[1].id)
        self.intent_data = self.load_json(INTENT_LOCATION)["intents"]

        self.lidar_system = LidarModule.LidarDetection(LIDAR_PORT_JET)

        # Engine text-to-speech by Zalo API
        self.ZALO_API_KEY = "ooGxzbgmfnuqLQLAPdkE2B3tGOMvM168"
        self.tts = ZaloTTS(speaker=ZaloTTS.NORTHERN_MEN, api_key=self.ZALO_API_KEY)

        self.run_assistant()

    def run_assistant(self):
        print("Start, you can say")
        while True:
            # Use Microphone resource to listen
            with speech_recognition.Microphone() as mic:
                # Tự điều chỉnh độ ồn của mic
                self.recognizer.adjust_for_ambient_noise(mic, duration=0.2)
                # Lắng nghe tiếng nói
                text = self.get_text_from_speech(mic)

                # Cmd khởi động
                if text is not None and WAKE_WORD in text:
                    self.speaker_say(RESPONSE_WELCOME)

                    text = self.get_text_from_speech(mic)

                    if text is not None:
                        if "khởi động" in text:
                            self.speaker_say(
                                "Khởi động hệ thống, hãy ra lệnh ít nhất 2 từ"
                            )

                            self.main_operation(mic)
                            # self.speaker.stop()
                            # self.root.destroy()
                            # sys.exit(0)
                        else:
                            response = self.query_response(text)
                            if response is not None:
                                self.speaker_say(response)
                            else:
                                self.speaker_say(NOT_UNDERSTAND)

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
        print("Listening...")
        try:
            audio = self.recognizer.listen(mic, timeout=6, phrase_time_limit=5)
            text = self.recognizer.recognize_google(
                audio, language="vi-VN", show_all=False
            )
            text = text.lower()
            print(text)
            return text
        except (
            speech_recognition.UnknownValueError,
            speech_recognition.WaitTimeoutError,
        ) as e:
            print("UnknownValueError")
            return None

    def main_operation(self, mic):
        while True:
            text = self.get_text_from_speech(mic)
            if text is not None:
                if "kết thúc" in text:
                    print("Back to waiting mode")
                    self.speaker_say("Dừng hệ thống")
                    break
                else:
                    intent = self.query_response(text)
                    if intent is not None:
                        if intent["response"] != "":
                            self.speaker_say(intent["response"])
                        if intent["action"] != "":
                            getattr(self, intent["action"])(mic=mic)
                    else:
                        self.speaker_say(NOT_UNDERSTAND)

    def run_lidar(self, **kwargs):
        """Run lidar system"""
        # Hỏi xem có bật hai bên trái phải không
        self.speaker_say("có bật hai bên trái phải không?")

        text = self.get_text_from_speech(kwargs["mic"])

        if text is not None and "không" in text:
            print("Không bật")
            self.speaker_say("xác nhận không")
            self.lidar_system.check_side = False
        else:
            print("Có bật")
            self.speaker_say("xác nhận có")
            self.lidar_system.check_side = True

        # Hỏi xem có nhận diện vật thể không
        self.speaker_say("có bật nhận diện vật thể không?")
        text = self.get_text_from_speech(kwargs["mic"])

        self.lidar_system.system_thread()
        while True:
            text = self.get_text_from_speech(kwargs["mic"])

            if text is not None and "dừng" in text:
                self.speaker_say("tắt phát hiện vật cản")
                self.lidar_system.system_cancel()
                break

    def speaker_say(self, text):
        """Speak text"""
        # Use pyttsx3
        #self.speaker.say(text)
        #self.speaker.runAndWait()

        # Use Zalo API
        self.tts.text_to_speech(text)


def main():
    assistant = Assistant()


if __name__ == "__main__":
    main()
