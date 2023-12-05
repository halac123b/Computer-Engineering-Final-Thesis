import serial
from CalcLidarData import CalcLidarData
import matplotlib.pyplot as plt
import math

import playsound
import threading
import TimerThread

''' Function to play audio '''
def play_sound(audio_file):
    playsound.playsound(audio_file)


# Create a thread to play the sound
sound_thread = threading.Thread(target=play_sound, args=("front.mp3", ))

class LidarDetection:
    def __init__(self, com_port):
        # Tạo 1 figure với pyplot của matplotlib
        # Figure có thể hiểu là 1 canvas, trên đó ta có thể vẽ nhiều biểu đồ
        self.fig = plt.figure(figsize=(8,8))

        # Tạo 1 biểu đồ trên Figure
            # Tại tọa độ 111, tức (1, 1) và mang index = 1 trên figure
            # Hệ tọa độ polar, hình tròn, thường dùng trong các bản đồ radar
        self.ax = self.fig.add_subplot(111, projection='polar')

        # Tạo kết nối Serial
        self.ser = serial.Serial(port=com_port,
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)
        # Control to alert at least each 5s
        self.timer_flag = False
        self.timer_thread = TimerThread.TimerThread(self)


    def init_pyplot(self):
        # Title cho biểu đồ
        self.ax.set_title('Lidar LD19 (exit: Key E)',fontsize=18)
        # Tạo 1 event cho pyplot
            # 'key_press_event': event nhấn 1 key
            # 1 hàm đc trigger cùng event
            # Press E to exit
        plt.connect('key_press_event', lambda event: exit(1) if event.key == 'e' else None)

    # Main function to run system object
    def run_system(self):
        self.init_pyplot()

        tmpString = ""
        lines = list()
        angles = list()
        distances = list()

        # Ngưỡng để xác nhận có vật cản trước mặt trên tổng số khoảng 78 tia
        FRONT_THRESHOLD = 70

        i = 0
        front = 0
        # Check that sound thread has been called before
        sound_check = False

        self.timer_thread.start_thread()

        try:
            while True:
                loopFlag = True
                flag2c = False

                if (i % 40 == 39):
                    if ('line' in locals()):
                        line.remove()

                    print(self.timer_flag)
                    if (front > FRONT_THRESHOLD and self.timer_flag):
                        # Start the thread
                        if (sound_check):
                            print("Lock")
                            sound_thread.join()
                            sound_check = False
                        sound_thread = threading.Thread(target=play_sound, args=("front.mp3", ))
                        sound_thread.start()
                        sound_check = True
                        self.timer_flag = False

                    front = 0

                    # Vẽ biểu đồ scatter (biểu đồ dạng điểm)
                        # Thường biểu diễn tương quan giữa 2 giá trị, ở đây là góc + khoảng cách
                        # c: color, s: size of points
                    line = self.ax.scatter(angles, distances, c="blue", s=5)
                    # Set offset cho vị trí của góc 0 độ trong hệ tọa độ polar
                        # Với hệ tọa độ của Lidar, góc 0 độ ứng với trục 0y nên cần set offset pi / 2
                    self.ax.set_theta_offset(math.pi / 2)
                    # Update Figure, hoặc delay 1 khoảng thời gian
                    plt.pause(0.01)
                    # Clear tập giá trị
                    angles.clear()
                    distances.clear()

                    i = 0


                while loopFlag:
                    # Đọc data từ Serial
                    b = self.ser.read()
                    # Convert int từ byte đọc được
                        # big: byte order của chuỗi bit, các bit quan trọng nhất nằm ở đầu
                    tmpInt = int.from_bytes(b, 'big')

                    # 0x54, indicating the beginning of the data packet (LD19 document)
                    if (tmpInt == 0x54):
                        tmpString += b.hex() + " "
                        flag2c = True
                        continue

                    # 0x2c: fixed value of VerLen (LD19 document)
                    elif (tmpInt == 0x2c and flag2c):
                        tmpString += b.hex()


                        if (not len(tmpString[0:-5].replace(' ','')) == 90):
                            tmpString = ""
                            loopFlag = False
                            flag2c = False
                            continue

                        # Sau khi đọc full 1 gói data Lidar sẽ có kích thước = 90, lấy string và đưa vào hàm CalcLidarData()
                        lidarData = CalcLidarData(tmpString[0:-5])

                        if (self.timer_flag):
                            if (330 <= lidarData.Degree_angle[0] <=360 or 0 <= lidarData.Degree_angle[0] <= 30):
                                check = False
                                for index in range(len(lidarData.Degree_angle)):
                                    if (330 <= lidarData.Degree_angle[index] <=360 or 0 <= lidarData.Degree_angle[index] <=30):
                                        check = True
                                        if (lidarData.Distance_i[index] < 1):
                                            front += 1
                                    elif check:
                                        break

                        # Get giá trị của góc và distance
                        angles.extend(lidarData.Angle_i)
                        distances.extend(lidarData.Distance_i)

                        # print(round(sum(lidarData.Distance_i) / len(lidarData.Distance_i), 2))
                        tmpString = ""
                        loopFlag = False
                    else:
                        tmpString += b.hex()+ " "

                    flag2c = False

                i += 1

        except KeyboardInterrupt:
            # Stop the timer thread before exiting
            self.ser.close()
            plt.close()
            self.timer_thread.cancel_thread()
            print("Program terminated.")

def main():
    lidar_system = LidarDetection("COM5")
    lidar_system.run_system()

if __name__ == "__main__":
    main()