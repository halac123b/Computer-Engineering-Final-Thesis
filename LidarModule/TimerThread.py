import threading

class TimerThread:
    def __init__(self, lidar_system):
        self.lidar = lidar_system
        # Thread to run software_timer
        self.timer_thread = threading.Timer(0.1, self.software_timer)

    def software_timer(self):
        if (self.lidar.timer_flag == False):
          print("Updating timer_flag to True")
          self.lidar.timer_flag = True

        # Schedule the timer to run again after 5 seconds
        timer = threading.Timer(5.0, self.software_timer)
        timer.start()

    def start_thread(self):
        self.timer_thread.start()

    def cancel_thread(self):
        self.timer_thread.cancel()
        self.timer_thread.join()