import serial
import glob, os

from math import ceil
from collections import deque
import string
import numpy as np
from pylab import *


if __name__ == "__main__":
    while True:
        os.chdir("/dev/")
        files = glob.glob("ttyACM[0-9]")
        if len(files) == 0:
            print "Can't not find serial port"
        else:
            break
    try:
        ser = serial.Serial('/dev/'+files[0], 38400, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False) #Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
    except IOError, msg:
        print msg
    ser.flushInput()
    ser.flushOutput()

    datas = []
    ser.write("hello motor")
    print "start"
    while True:
        data_raw = ser.readline()
        data = string.split(data_raw)
        print data
        if len(data) != 3:
            continue
        datas.append([float(x) for x in data])
            
        if data[0] == '255':
            break
    
    datas = np.asarray(datas)
    print datas
    np.savetxt('/home/qq/sketchbook/balance_robot/analysis/motor_data.txt', datas, delimiter=',')
    plot(datas[:,0], datas[:,1], label='motor1')
    plot(datas[:,0], datas[:,2], label='motor2')
    legend()
    show()

