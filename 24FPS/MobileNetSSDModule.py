import jetson_inference
import jetson_utils
import cv2
import time

class MobileNetSSD():
    def __init__(self, path, threshold):
        self.path = path
        self.threshold = threshold
        self.net = jetson_inference.detectNet(self.path, self.threshold)

    def detect(self, img, display=False):
        # Convert image from Numpy format of CV2 to Cuda format for Detection model can work with
        imgCuda = jetson_utils.cudaFromNumpy(img)
        detections = self.net.Detect(imgCuda, overlay="OVERLAY_NONE")

        #img = jetson_utils.cudaToNumpy(imgCuda)

        objects = []
        for d in detections:
            className = self.net.GetClassDesc(d.ClassID)

            objects.append([className, d])

            # Option có hiển thị kết quả hình ảnh nhận diện ra hay không
            if display:
                #print(d)
                x1, y1, x2, y2 = int(d.Left), int(d.Top), int(d.Right), int(d.Bottom)

                cx, cy = int(d.Center[0]), int(d.Center[1])
                cv2.line(img, (x1, cy), (x2, cy), (255, 0, 255), 1)
                cv2.line(img, (cx, cy), (cx, y2), (255, 0, 255), 1)

                cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)

                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
                cv2.putText(img, className, (x1 + 5, y1 + 15), cv2.FONT_HERSHEY_DUPLEX, 0.75, (255, 0, 255), 2)
                # fps = f'FPS: {int(self.net.GetNetworkFPS())}'
                # cv2.putText(img, fps, (30, 30), cv2.FONT_HERSHEY_DUPLEX, 0.75, (255, 0, 0), 2)
                print(int(self.net.GetNetworkFPS()))

                self.calculator((x1, y1, x2, y2))
            return objects

    def calculator(self, data):
        global lastDirection
        global count
        global ser
        if ser == None:
            ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.01)
            ser.readline()
            if len(data) == 0 :
                if lastDirection != 'straight' :
                    count = 0
                    lastDirection = 'straight'
                count+=1
                if count == 3 :
                    count = 0
                    ser.write(bytes('w\n','utf-8'))
                    print("di thang")
            for data_ob in data:
                if data_ob[0] > 320:
                    if lastDirection != 'left':
                        count = 0
                        lastDirection = 'left'
                    count += 1
                    if count == 3 :
                        count = 0
                        ser.write(bytes('a\n','utf-8'))
                        print("re trai")
                elif data_ob[0] <= 320 :
                    if lastDirection != 'right' :
                        count = 0
                        lastDirection = 'right'
                    count+=1
                    if count == 3 :
                        count = 0
                        ser.write(bytes('d\n','utf-8'))
                        print("re phai")
                    print("in count: ", count)
                time.sleep(1)
                break
            return



def main():

    cap = cv2.VideoCapture(0)

    cap.set (cv2.CAP_PROP_FRAME_WIDTH,640)
    cap.set (cv2.CAP_PROP_FRAME_HEIGHT,480)

    myModel = MobileNetSSD("ssd-mobilenet-v2", 0.5)

    while True:
        success, img = cap.read()

        objects = myModel.detect(img, True)

        cv2.imshow("Image", img)
        cv2.waitKey(1)

if __name__ == "__main__":
    main()

