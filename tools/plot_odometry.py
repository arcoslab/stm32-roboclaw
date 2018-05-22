#!/usr/bin/env python3
import re
import serial
import sys
import select

def enter(ser):
    ser.write("\r".encode('utf-8'))

def process_line(ser, pattern):
    line = ser.readline().decode('utf-8')
    match = re.match(pattern, line)
    if match:
        print(match.group('vel'))

def write_char(char,ser):
    ser.write(char.encode('utf-8'))
    enter(ser)

ser = serial.Serial('/dev/ttyACM0')
ser.baudrate = 115200
pattern = re.compile(r'.*Past\s*Pos:\s*(?P<ppos> \d+)\s*\|\s*Current\s*Pos:\s*(?P<cpos> \d+)\s*\|\s*TICKS\s*TIME:\s*\d+\.\d+\s*\|\s*Counter:\s*(?P<counter> \d+)\s*\|\s*Current\s*Vel:\s*(?P<vel> -?\d+\.\d+).*')

enter(ser)

while(1):
    process_line(ser, pattern)
    while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        char = sys.stdin.read(1)
        if(char):
            write_char(char, ser)
            break;
        break;
