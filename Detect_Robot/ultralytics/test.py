from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor
import cv2
import numpy
import numpy as np
from imutils.video import VideoStream
import time
import serial

model = YOLO('yolov8n.pt')
# model.info()  # display model information
# points = []
#with serial.Serial('/dev/ttyUSB0', 9600, timeout=10) as ser:
results = model.predict(source = "0", show =True ,save_txt = True  )
	#while True:                
                        #led_on = input()
		#if 1 == 1:
                        #ser.write(bytes('w\n','utf-8'))
                        #print(ser)

#ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=2) 
#led_on = input("press ")

#print(ser)
#ser.readline()
#ser.write(bytes('w\n','utf-8'))
#print("bla", ser.readline().encode())

    


#model.serial()




