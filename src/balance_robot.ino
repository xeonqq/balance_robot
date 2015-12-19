/* Copyright (C) 2015 Qian Qian. All rights reserved.
 E-mail   :  xeonqq@gmail.com
 */

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EnableInterrupt.h>
#include "PID.h"
#include "Kalman.h"

#include "motor.h"
#include "balance_robot.h"
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include "utils.h"

#include "LowPassFilter.h"
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

float G_Dt;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float rot_y, rot_z; 

float filtered_rot_z;

int16_t yaw_du = 0;


PID pid;
PID pid_yaw;

Kalman kalman;

LowPassFilterFloat lpf_rot_z;
float m1_rpm = 0;
float m2_rpm = 0;

#define  VALID 0xABCD

typedef struct config_t
{
	int valid;
	float Kp;
	float Ki;
	float Kd;
} pid_config;


pid_config balance_pid_values;

pid_config yaw_rate_pid_values;

float u; //pid control output

//in degree
float target_angle = -0.5f;

//in degree/s
float target_yaw_rate = 0.0f;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();

	balance_pid_values.Kp = 0.4f;
	balance_pid_values.Ki = 20.0f;
	balance_pid_values.Kd = 0.0f;

	yaw_rate_pid_values.Kp = 2.0f;
	yaw_rate_pid_values.Ki = 1.0f;
	yaw_rate_pid_values.Kd = 0.0f;

	// initialize serial communication
	// (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
	// it's really up to you depending on your project)
	Serial.begin(38400);
	Serial1.begin(115200);  //bluetooth
	inputString.reserve(20);
	Serial1.println(F("\nPress 'q' to see current PID settings"));
	Serial1.println(F("Press 'p' ,then type a number to change p value"));
	Serial1.println(F("Press 'i' ,then type a number to change i value"));
	Serial1.println(F("Press 'd' ,then type a number to change d value"));
	//Serial1.println(F("Press 'w' to save pid to EEPROM"));
	Serial1.println(F("Press 'Enter' to confirm\n"));

	Serial.println("Initializing I2C devices...");
	mpu.initialize();

	// verify connection
	Serial.println("Testing device connections...");
	Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	pid = PID(balance_pid_values.Kp, balance_pid_values.Ki, balance_pid_values.Kd);
	
	pid_yaw = PID(yaw_rate_pid_values.Kp, yaw_rate_pid_values.Ki, yaw_rate_pid_values.Kd);

	initMotors();

	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	float angleByAcc = atan2(ax, az);
	kalman.setAngle(angleByAcc);

	//4 hz cutoff freq for angular velocity in z
	lpf_rot_z.set_cutoff_frequency(2);

	//calibrate();

	kalmanTimer = millis();
	pidTimer = kalmanTimer;
	imuTimer = millis();
	encoderTimer = imuTimer;
	reportTimer = imuTimer;


}

void balance(float angle_sensed)
{

	//u = pid.control(target_angle, angle_sensed, dt);
	float w_sensed_unbias = kalman.getRate();
	u = pid.cascade_control(target_angle, angle_sensed, w_sensed_unbias, G_Dt);

	//yaw_du = yaw_rate_pid_values.Kp*(target_yaw_rate- rot_z);

	/*
	Serial.print(target_angle); Serial.print('\t');
	Serial.print(angle_sensed); Serial.print('\t');
	Serial.print(w_sensed_unbias); Serial.print('\t');
	Serial.print(G_Dt); Serial.print('\t');
	Serial.println();
	*/

	//Serial.println(pwm);

	//motors_control_direct(u);
}

void yaw_control()
{
	if (target_yaw_rate == 0.0f)
		yaw_du = 0;
	else
		yaw_du = pid_yaw.control(target_yaw_rate, filtered_rot_z, G_Dt);
}

void processBluetooth()
{
	while (Serial1.available()) {
		// get the new byte:
		char inChar = (char)Serial1.read();
		// add it to the inputString:
		inputString += inChar;
		// Serial.println(inChar);
		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\n' || inChar == '\r') {
			stringComplete = true;
		}
	}
	if (stringComplete) {

		if (inputString.startsWith("p") || inputString.startsWith("P"))
		{
			balance_pid_values.Kp = inputString.substring(1).toFloat();
			Serial.print("P in pid set to:\t"); Serial.println(balance_pid_values.Kp);
			Serial1.print("P in pid set to:\t"); Serial1.println(balance_pid_values.Kp);
			pid = PID(balance_pid_values.Kp, balance_pid_values.Ki, balance_pid_values.Kd);
		}
		if (inputString.startsWith("i") || inputString.startsWith("I"))
		{
			balance_pid_values.Ki = inputString.substring(1).toFloat();
			Serial.print("I in pid set to:\t"); Serial.println(balance_pid_values.Ki);

			Serial1.print("I in pid set to:\t"); Serial1.println(balance_pid_values.Ki);
			pid = PID(balance_pid_values.Kp, balance_pid_values.Ki, balance_pid_values.Kd);
		}
		if (inputString.startsWith("d") || inputString.startsWith("D"))
		{
			balance_pid_values.Kd = inputString.substring(1).toFloat();
			Serial.print("D in pid set to:\t"); Serial.println(balance_pid_values.Kd, 5);
			Serial1.print("D in pid set to:\t"); Serial1.println(balance_pid_values.Kd, 5);
			pid = PID(balance_pid_values.Kp, balance_pid_values.Ki, balance_pid_values.Kd);
		}
		if (inputString.startsWith("q") || inputString.startsWith("Q"))
		{
			Serial.print("P:"); Serial.print(balance_pid_values.Kp);Serial.print("\tI:"); Serial.print(balance_pid_values.Ki); Serial.print("\tD:");Serial.println(balance_pid_values.Kd, 5);
			Serial1.print("P:"); Serial1.print(balance_pid_values.Kp);Serial1.print("\tI:"); Serial1.print(balance_pid_values.Ki); Serial1.print("\tD:");Serial1.println(balance_pid_values.Kd, 5);
		}
		if (inputString.startsWith("r") || inputString.startsWith("R"))
		{
			target_yaw_rate = inputString.substring(1).toFloat();
			Serial1.print("yaw_rate set to:\t"); Serial1.println(target_yaw_rate);

		}

		if (inputString.substring(0,2) == "yp")
		{
			yaw_rate_pid_values.Kp = inputString.substring(2).toFloat();
			Serial1.print("P in yaw pid set to:\t"); Serial1.println(yaw_rate_pid_values.Kp);
			pid_yaw = PID(yaw_rate_pid_values.Kp, yaw_rate_pid_values.Ki, yaw_rate_pid_values.Kd);
		}
		if (inputString.substring(0,2) == "yi")
		{
			yaw_rate_pid_values.Ki = inputString.substring(2).toFloat();
			Serial1.print("I in yaw pid set to:\t"); Serial1.println(yaw_rate_pid_values.Ki);
			pid_yaw = PID(yaw_rate_pid_values.Kp, yaw_rate_pid_values.Ki, yaw_rate_pid_values.Kd);
		}
		if (inputString.substring(0,2) == "yd")
		{
			yaw_rate_pid_values.Kd = inputString.substring(2).toFloat();
			Serial1.print("D in yaw pid set to:\t"); Serial1.println(yaw_rate_pid_values.Kd);
			pid_yaw = PID(yaw_rate_pid_values.Kp, yaw_rate_pid_values.Ki, yaw_rate_pid_values.Kd);
		}

		if (inputString.startsWith("f") || inputString.startsWith("f"))
		{
			target_angle = inputString.substring(1).toFloat();
			Serial1.print("target_angle set to:\t"); Serial1.println(target_angle);

		}
		/*
		if (inputString.startsWith("w") || inputString.startsWith("W"))
		{
			balance_pid_values.valid = VALID;
			EEPROM_writeAnything(0, balance_pid_values);
			Serial1.print(F("saved in EEPROM\n"));
		}*/

		inputString = "";
		stringComplete = false;
	}
}
void stopAndReset()
{
  u = 0;
  yaw_du = 0;
  motors_stop();
  pid.reset();
}

void loop()
{
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	//calculate pitch by kalman
	rot_y = -(float)gy / GYRO_SENS; //to degree

	//roation rate at yaw
	rot_z = -(float)gz / GYRO_SENS; //to degree
	//Serial.println(rot_z);
	
	float angleByAcc = atan2(ax, az);

	currentTime = millis(); 

	G_Dt = (currentTime - kalmanTimer);
	//kalman filter is running at MCU's full speed
	float pitch = kalman.getAngle(angleByAcc*Rad2Deg, rot_y, G_Dt/1000.0f);
	kalmanTimer = currentTime;


	if (currentTime - pidTimer > TASK_100HZ)
	{
		G_Dt = (currentTime - pidTimer)/1000.0f;

		filtered_rot_z = lpf_rot_z.apply(rot_z, G_Dt);
		//Serial.print(rot_z); Serial.print('\t');
		//Serial.print(filtered_rot_z); Serial.println();

		//if error less than 35 degree, try to balance
		if (abs(pitch) < 35){	
			balance(pitch);	
			yaw_control();
			motors_control_sep(u-yaw_du, u+yaw_du);
		}
		else{

			stopAndReset();
		}
		pidTimer = currentTime;
	}


	if (currentTime - reportTimer >= TASK_1HZ)  
	{
		processBluetooth();

		//Serial1.print(target_yaw_rate); Serial1.print('\t');
		//Serial1.print(rot_z); Serial1.print('\t');
		//Serial1.print(filtered_rot_z); Serial1.print('\t');
		//Serial1.println(yaw_du);
		reportTimer = currentTime;
	}

}














