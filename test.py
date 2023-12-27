import speech_recognition

recognizer = speech_recognition.Recognizer()

try:
    with speech_recognition.Microphone() as mic:
        recognizer.adjust_for_ambient_noise(mic, duration=0.2)
        audio = recognizer.listen(mic, timeout=6, phrase_time_limit=5)
        text = recognizer.recognize_google(audio, language="vi-VN", show_all=False)
        text = text.lower()
        print(text)
except (speech_recognition.UnknownValueError, speech_recognition.WaitTimeoutError) as e:
    print("UnknownValueError")

# import threading
# import time

# def my_function():
#     print("Timer function executed.")

# # Create a timer that will run my_function after 5 seconds
# my_timer = threading.Timer(5, my_function)

# # Start the timer
# my_timer.start()

# # Simulate some other work
# time.sleep(2)

# # Cancel the timer before it executes the function
# my_timer.cancel()

# my_timer.join()
# # Confirm whether the timer was canceled
# if my_timer.is_alive():
#     print("Timer was not canceled.")
# else:
#     print("Timer was canceled.")

#ZALO_API_KEY = "ooGxzbgmfnuqLQLAPdkE2B3tGOMvM168"

#from zalo_tts import ZaloTTS

#tts = ZaloTTS(speaker=ZaloTTS.NORTHERN_MEN, api_key=ZALO_API_KEY)
#tts.text_to_speech("Dạ em nghe.")
