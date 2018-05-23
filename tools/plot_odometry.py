#!/usr/bin/env python3
import re
import serial
import sys
import select
import matplotlib.pyplot as plt
import collections

def enter(ser):
    ser.write("\r".encode('utf-8'))

def process_line(ser, pattern, line1, fig):
    line = ser.readline().decode('utf-8')
    match = re.match(pattern, line)
    if match:
        print(match.group('cpos'))
        fig_update(float(match.group('vel')), line1, fig)

def write_char(char,ser):
    ser.write(char.encode('utf-8'))
    enter(ser)

def fig_update(vel, line1, fig):
    vels.append(vel)
    line1.set_ydata(list(vels))
    fig.canvas.draw()
    fig.canvas.flush_events()

plt.ion()
vels = collections.deque([0 for number in range(50)], maxlen=50)
fig = plt.figure()
ax = fig.add_subplot(111)
x = collections.deque(range(50), maxlen=50)
vels.append(1700) # this sets the graph size
line1, = ax.plot(list(x), list(vels), 'b-')
ser = serial.Serial('/dev/ttyACM0')
ser.baudrate = 115200
pattern = re.compile(r'.*Past\s*Pos:\s*(?P<ppos> \d+)\s*\|\s*Current\s*Pos:\s*(?P<cpos> \d+)\s*\|\s*TICKS\s*TIME:\s*\d+\.\d+\s*\|\s*Counter:\s*(?P<counter> \d+)\s*\|\s*Current\s*Vel:\s*(?P<vel> -?\d+\.\d+).*')
enter(ser)

while(1):
    process_line(ser, pattern, line1, fig)
    while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:# this is like a poll to stdin. flush with enter
        char = sys.stdin.read(1)
        if(char):
            write_char(char, ser)
            break;
        break;
