import serial
import glob, os

from math import ceil
from collections import deque
import string
import numpy as np

class PlotHandel():
    def __init__(self, ax, line_data, data):
        self._lines = line_data
        self._datas = data
        self._ax = ax

    def append_data(self, d):
        if d is not None:
            #if len(self._lines) > 1:
            for i in xrange(len(self._lines)):
                self._datas[i].append(d[i])
                self._lines[i].set_ydata(self._datas[i])
            #else:
            #    self._datas[0].append(d)
            #    self._lines[0].set_ydata(self._datas[0])

            self._ax.relim()
            self._ax.autoscale_view()
    
        
class RTPlot():
    def __init__(self, capacity=4, col=1, xaxis_lim=500, size=(16, 9)):
        from matplotlib import pyplot as plt
        self.capacity = capacity
        self.plt = plt
        self.plt.ion()
        self.fig = self.plt.figure(figsize=size)
        self.plot_handels = {}
        self.deque_lim = xaxis_lim
        self.col = col

    def add_plot(self, name, data_dim, labels):
        deque_lim = self.deque_lim
        cap = self.capacity
        num = len(self.plot_handels)
        columns = self.col
        rows = ceil(cap/columns)
        ax = self.fig.add_subplot(rows, columns, num+1)
        data = [deque([float('NaN')] * deque_lim, deque_lim) for i in range(data_dim)]
        lines = []
        for i in xrange(data_dim):
            line_data, = ax.plot(data[i], label=labels[i])
            lines.append(line_data)
        ax.set_title(name)
        ax.legend(loc="upper right")
        ph = PlotHandel(ax, lines, data)
        self.plot_handels[name] = ph
        return ph
    
    def append_data(self, handel_name, data):
        if self.plot_handels.has_key(handel_name):
            handel = self.plot_handels[handel_name]
            handel.append_data(data)
        else:
            raise NameError('can not find subplot '+handel_name)

    def draw(self):
        self.plt.draw()


if __name__ == "__main__":
    os.chdir("/dev/")
    files = glob.glob("ttyACM[0-9]")
    if len(files) == 0:
        raise Exception("Can't not find serial port");
    try:
        ser = serial.Serial('/dev/'+files[0], 38400, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False) #Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
    except IOError, msg:
        print msg
    ser.flushInput()
    ser.flushOutput()

    rtplot = RTPlot(capacity=1)
    data_raw = ser.readline()
    data = string.split(data_raw)
    data = np.asarray(data)
    num_data = len(data)/2 
    rtplot.add_plot(ser.name, num_data, data[::2])
    while True:
        data_raw = ser.readline()
        data = string.split(data_raw)
        print data
        if len(data) != num_data*2:
            continue
        for i in xrange(num_data):
            w = [float(x) for x in data[1::2]]
            rtplot.append_data(ser.name, w)
        rtplot.draw()
            
        
        
        #print(data_raw)
