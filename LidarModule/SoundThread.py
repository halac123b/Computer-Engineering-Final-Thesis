import playsound
import threading

class SoundThread:
  def __init__(self):
    self.front_audio = "./step/front.mp3"
    self.left_audio = "./step/left.mp3"
    self.right_audio = "./step/right.mp3"

  def play_sound(self, step_file, step_left, step_right):
      ''' Function to play audio '''
      if (step_file != ""):
        playsound.playsound(self.front_audio)
        playsound.playsound(step_file)
      if (step_left != ""):
        playsound.playsound(self.left_audio)
        playsound.playsound(step_left)
      if (step_right != ""):
        playsound.playsound(self.right_audio)
        playsound.playsound(step_right)

  def start_thread(self, step, step_left, step_right):
    if (step > 0):
      audio_step = f"./step/{step}step.mp3"
    else:
      audio_step = ""
    if (step_left > 0):
      audio_left = f"./step/{step_left}step.mp3"
    else:
      audio_left = ""
    if (step_right > 0):
      audio_right = f"./step/{step_right}step.mp3"
    else:
      audio_right = ""

    self.sound_thread = threading.Thread(target=self.play_sound, args=(audio_step, audio_left, audio_right))
    self.sound_thread.start()

  def cancel_thread(self):
     self.sound_thread.join()