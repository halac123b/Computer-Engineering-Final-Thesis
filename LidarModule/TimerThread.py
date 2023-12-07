import threading

INTERVAL_TIME = 7

class TimerThread:
    def __init__(self, lidar_system):
        self.lidar = lidar_system
        # Thread to run software_timer

        self.stop_event = threading.Event()
        self.interval_time = INTERVAL_TIME

    def software_timer(self):
        if (self.lidar.timer_flag == False):
          print("Updating timer_flag to True")
          self.lidar.timer_flag = True
        print(111)

        if (self.stop_event.is_set()):
            return
        # Schedule the timer to run again after 5 seconds
        timer = threading.Timer(self.interval_time, self.software_timer)
        timer.start()

    def start_thread(self):
        self.timer_thread = threading.Timer(0.1, self.software_timer)
        self.timer_thread.start()

    def cancel_thread(self):
        self.stop_event.set()
        self.timer_thread.cancel()
        self.timer_thread.join()