import playsound
import threading

class SoundThread:
  def __init__(self):
    pass

  ''' Function to play audio '''
  def play_sound(self, audio_file, step_file):
      playsound.playsound(step_file)
      playsound.playsound(audio_file)

  def start_thread(self, step):
    audio_step = f"./step/{step}step.mp3"
    self.sound_thread = threading.Thread(target=self.play_sound, args=("./step/front.mp3", audio_step))
    self.sound_thread.start()

  def cancel_thread(self):
     self.sound_thread.join()