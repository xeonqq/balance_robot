#!/usr/bin/env python

import serial, sys
import subprocess
import time

serialPort = "/dev/ttyACM0"
print "reset port: ", serialPort

ser = serial.Serial(
        port=serialPort,
        baudrate=1200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
        )


ser.isOpen()
ser.setDTR(True)
time.sleep(0.022)
ser.setDTR(False)
ser.close()             # close port



time.sleep(3)
print subprocess.Popen('cd build && make upload', shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT).stdout.read()
