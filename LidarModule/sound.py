import playsound
import threading

def play_sound(audio_file):
    playsound.playsound(audio_file)

# Replace 'your_audio_file.mp3' with the path to your audio file
#audio_file = "front.mp3"

audio_step = f"./step/{1}step.mp3"
# Create a thread to play the sound
sound_thread = threading.Thread(target=play_sound, args=(audio_step, ))

# Start the thread
#sound_thread.start()

play_sound(audio_step)
# Code here will continue to execute immediately without waiting for the sound to finish
print("This will be printed while the sound is playing.")

#playsound.playsound(audio_file)


