import sys
import serial

from LidarModule.CalcLidarData import CalcLidarData
#import matplotlib.pyplot as plt
#import math

import LidarModule.TimerThread as TimerThread
import LidarModule.SoundThread as SoundThread

import playsound
import threading

class LidarDetection:
    def __init__(self, com_port):
        # Tạo 1 figure với pyplot của matplotlib
        # Figure có thể hiểu là 1 canvas, trên đó ta có thể vẽ nhiều biểu đồ
        # self.fig = plt.figure(figsize=(8,8))

        # Tạo 1 biểu đồ trên Figure
            # Tại tọa độ 111, tức (1, 1) và mang index = 1 trên figure
            # Hệ tọa độ polar, hình tròn, thường dùng trong các bản đồ radar
        #self.ax = self.fig.add_subplot(111, projection='polar')

        # Tạo kết nối Serial
        self.ser = serial.Serial(port=com_port,
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)
        # Control to alert at least each 5s
        self.timer_flag = False

        self.sound_thread = SoundThread.SoundThread()

        # Check whether check left and right direction or not
        self.check_side = True

        self.stop = False


    def init_pyplot(self):
        # Title cho biểu đồ
        self.ax.set_title('Lidar LD19 (exit: Key E)',fontsize=18)
        # Tạo 1 event cho pyplot
            # 'key_press_event': event nhấn 1 key
            # 1 hàm đc trigger cùng event
            # Press E to exit
        #plt.connect('key_press_event', lambda event: exit(1) if event.key == 'e' else None)

    # Main function to run system object
    def run_system(self):
        playsound.playsound("./LidarModule/step/lidar_start.mp3")
        #self.init_pyplot()

        tmpString = ""
        #lines = list()
        angles = list()
        distances = list()

        # Ngưỡng để xác nhận có vật cản trước mặt trên tổng số khoảng 78 tia
        FRONT_THRESHOLD = 50
        # Ngưỡng 2 phía left, right trên 39 tia
        SIDE_THRESHOLD = 33

        i = 0
        # 3 variable to detect avoidance at front, left, right
        front = 0
        left = 0
        right = 0
        # Check that sound thread has been called before
        sound_check = False

        self.timer_thread.start_thread()
        # Dictionary to find nearest avoidance
        avoid_dict = {}
        avoid_left = {}
        avoid_right = {}
        min_distance = 5
        min_distance_left = 3
        min_distance_right = 3

        while not self.stop:
            loopFlag = True
            flag2c = False

            if (i % 40 == 39):
                # if ('line' in locals()):
                #     line.remove()

                #print(self.timer_flag)
                #print(front, left, right)
                if (self.timer_flag):
                    print(front, left, right)
                    if (front > FRONT_THRESHOLD):
                        if (min_distance == 0):
                            min_distance = 0.5

                    else:
                        min_distance = -1

                    if (self.check_side):
                        if (left > SIDE_THRESHOLD):
                            if (min_distance_left == 0):
                                min_distance_left = 0.5
                        else:
                            min_distance_left = -1
                        if (right > SIDE_THRESHOLD):
                            if (min_distance_right == 0):
                                min_distance_right = 0.5
                        else:
                            min_distance_right = -1
                    else:
                        min_distance_left = -1
                        min_distance_right = -1

                    # Start the thread
                    if (sound_check):
                        self.sound_thread.cancel_thread()

                    self.sound_thread.start_thread(int(min_distance * 2), int(min_distance_left * 2), int(min_distance_right * 2))
                    sound_check = True
                    self.timer_flag = False

                avoid_dict.clear()
                avoid_left.clear()
                avoid_right.clear()
                front = 0
                left = 0
                right = 0
                min_distance = 5
                min_distance_left = min_distance_right = 3

                # Vẽ biểu đồ scatter (biểu đồ dạng điểm)
                    # Thường biểu diễn tương quan giữa 2 giá trị, ở đây là góc + khoảng cách
                    # c: color, s: size of points
                #line = self.ax.scatter(angles, distances, c="blue", s=5)
                # Set offset cho vị trí của góc 0 độ trong hệ tọa độ polar
                    # Với hệ tọa độ của Lidar, góc 0 độ ứng với trục 0y nên cần set offset pi / 2
                #self.ax.set_theta_offset(math.pi / 2)
                # Update Figure, hoặc delay 1 khoảng thời gian
                #plt.pause(0.01)
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

                    # if (self.timer_flag):
                    if (320 <= lidarData.Degree_angle[0] <= 360 or 0 <= lidarData.Degree_angle[0] <= 30):
                        check = False
                        for index in range(len(lidarData.Degree_angle)):
                            #print("check")
                            if (330 <= lidarData.Degree_angle[index] <=360 or 0 <= lidarData.Degree_angle[index] <=30):
                                check = True
                                # Khoảng cách để xác nhận vật cản là 5m
                                #print(lidarData.Distance_i[index])
                                if (lidarData.Distance_i[index] < 5):
                                    #print(lidarData.Distance_i[index])
                                    front += 1

                                    round_number = round(lidarData.Distance_i[index] * 2) / 2
                                    if (round_number < min_distance):
                                        if (round_number in avoid_dict):
                                            avoid_dict[round_number] += 1
                                            if (avoid_dict[round_number] >= 15):
                                                min_distance = round_number
                                        else:
                                            avoid_dict[round_number] = 1
                            elif check:
                                break
                    elif (self.check_side):
                        if (260 <= lidarData.Degree_angle[0] <= 300):
                            check = False
                            for index in range(len(lidarData.Degree_angle)):
                                if (270 <= lidarData.Degree_angle[0] <= 300):
                                    check = True
                                    if (lidarData.Distance_i[index] < 3):
                                        right += 1

                                    round_number = round(lidarData.Distance_i[index] * 2) / 2
                                    if (round_number < min_distance_right):
                                        if (round_number in avoid_right):
                                            avoid_right[round_number] += 1
                                            if (avoid_right[round_number] >= 8):
                                                min_distance_right = round_number
                                        else:
                                            avoid_right[round_number] = 1

                        elif (50 <= lidarData.Degree_angle[0] <= 90):
                            check = False
                            for index in range(len(lidarData.Degree_angle)):
                                if (60 <= lidarData.Degree_angle[0] <= 90):
                                    check = True
                                    if (lidarData.Distance_i[index] < 3):
                                        left += 1

                                    round_number = round(lidarData.Distance_i[index] * 2) / 2
                                    if (round_number < min_distance_left):
                                        if (round_number in avoid_left):
                                            avoid_left[round_number] += 1
                                            if (avoid_left[round_number] >= 8):
                                                min_distance_left = round_number
                                        else:
                                            avoid_left[round_number] = 1


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

    def system_thread(self):
        '''Function to run system in another thread'''
        lidar_thread = threading.Thread(target=self.run_system, args=())
        self.timer_thread = TimerThread.TimerThread(self)
        self.stop = False

        lidar_thread.start()

    def system_cancel(self):
        '''Function to cancel system thread'''
        # Stop the timer thread before exiting
        #self.ser.close()
        #plt.close()
        self.stop = True
        self.timer_thread.cancel_thread()
        print("Program terminated.")
        # self.lidar_thread.join()


def main():
    lidar_system = LidarDetection("COM5")
    lidar_system.run_system()

if __name__ == "__main__":
    main()