import jetson_inference
import jetson_utils
import cv2

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

            # Option co hien thi ket qua hinh anh nhan dien ra hay khong
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
                #print(int(self.net.GetNetworkFPS()))

                # self.calculator((x1, y1, x2, y2))
            return objects

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
