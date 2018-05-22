#!/usr/bin/env python3
import re
import serial

ser = serial.Serial('/dev/ttyACM0')
ser.baudrate = 115200

while(1):
    line = ser.readline().decode('UTF-8')
    print(line)
