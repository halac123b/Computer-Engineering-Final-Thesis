# import playsound
# import threading

# def play_sound(audio_file):
#     playsound.playsound(audio_file)

# # Replace 'your_audio_file.mp3' with the path to your audio file
# audio_file = "front.mp3"

# # Create a thread to play the sound
# sound_thread = threading.Thread(target=play_sound, args=("front.mp3",))

# # Start the thread
# #sound_thread.start()

# play_sound("front.mp3")
# # Code here will continue to execute immediately without waiting for the sound to finish
# print("This will be printed while the sound is playing.")

# #playsound.playsound(audio_file)


import threading
import time

def my_function():
    print("Thread function")
    time.sleep(2)

# Creating and starting the thread
my_thread = threading.Thread(target=my_function)
my_thread.start()

# Check if the thread is started
if my_thread.is_alive():
    print("Thread is already started")
    my_thread.join()  # Wait for the thread to finish
else:
    print("Thread is not started yet")

# Attempting to start the thread again will raise a RuntimeError
my_thread.start()  # Raises RuntimeError: threads can only be started once
