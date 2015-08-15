##Demo

The robot can balance itself on two wheels and can handle small disturbance. Further work shall be done on remote controlling to move it.

![robot][robot_pic]

[robot_pic]: https://github.com/xeonqq/balance_robot/blob/kalman/robot_selfie.jpg "Robot Selfie"

Video:
<a href="https://youtu.be/sWVQpQ8RNGM
" target="_blank"><img src="https://i.ytimg.com/vi/sWVQpQ8RNGM/2.jpg?time=1439640081480" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

##Hardwares
1. [Arduino Leonardo Board](https://www.arduino.cc/en/Main/ArduinoBoardLeonardo)

2. [2 DC Motors: JGA25-371 with encoders](http://world.taobao.com/item/40496339515.htm?fromSite=main&spm=a1z0d.6639537.1997196601.413.U9SqEj)

3. [Arduino Motor Driver Shield](http://world.taobao.com/item/20695931042.htm?fromSite=main&spm=a1z0d.6639537.1997196601.4.U9SqEj)

4. [12V-5V-3.3V Power Convertion Board](http://item.taobao.com/item.htm?spm=a312a.7700846.9.323.7gA2vL&id=35296225045&_u=f3e5nn585ef)

5. [ZIPPY Flightmax 2200mAh 3S1P 25C Battery](http://www.hobbyking.com/hobbyking/store/__38109__ZIPPY_Flightmax_2200mAh_3S1P_25C_EU_Warehouse_.html)

6. [HC-06 BlueTooth Module](http://item.taobao.com/item.htm?spm=a312a.7700846.9.121.rql2Wm&id=19087365613&_u=f3e5nn58d41)

7. [MPU-6050 6DOF IMU](https://detail.tmall.com/item.htm?id=18635718636&toSite=main)

##Software Key Components
1. *Kalman filter*: to fuse the data from accerometer and gyro and output angle. The implementation is referenced from TKJElectronics's [Balanduino](https://github.com/TKJElectronics/KalmanFilter).

2. *PID control*: "velocity" PID which can prevent integral windup (Further theory refer to this [link](http://lorien.ncl.ac.uk/ming/digicont/digimath/dpid1.htm)). Target is to stablize the robot around angle 0. 

3. *Tune PID via Bluetooth*: I downloaded [BlueTerm](https://play.google.com/store/apps/details?id=es.pymasde.blueterm&hl=en) to pair and communicate with arduino. Can Tune the PID without wire.

4. *Motor control*: control use PWM signal, a bit tuning is done to make both motors run at same RPM given same PWM value. And the motor has a minimum PWM to be applied, because under certain minimum PWM value the motor does not move at all.

5. *Python Debug*: python script analysis/serial_plot.py can plot the data from Serial port in real time. But the message has to follow the format *"name\t value\t name\t value\t...\n"*

##Problem encountered
This motor can not supply enough torque when the robot deviate its angle too much, so the robot can still fall down when angle error is big. 
