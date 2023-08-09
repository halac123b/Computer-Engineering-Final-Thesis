import time
import serial
import struct

check = True
with serial.Serial('COM3', 9600, timeout=10) as ser:
    while True:
        cc = input()
        if cc == "a" and check:
            ser.write(bytes('a\n','utf-8'))
            check = False

        receive = bytearray(ser.read())[0]

        print(receive)

        receive = bytearray(ser.read())[0]

        print(receive)
