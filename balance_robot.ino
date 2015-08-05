
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
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;


int16_t ax, ay, az;
int16_t gx, gy, gz;


PID pid;
PID pid_rpm;
Kalman kalman;

double m1_rpm = 0;
double m2_rpm = 0;

#define  VALID 0xABCD
typedef struct config_t
{
	int valid;
	double Kp;
	double Ki;
	double Kd;
} pid_config;


pid_config pid_values;

double u; //pid control output

double target_angle = 1.7;


String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();

	EEPROM_readAnything(0, pid_values); 

	if (pid_values.valid != VALID){
		pid_values.Kp = 4.5;
		pid_values.Ki = 1;
		pid_values.Kd = 0.002;
	}
	// initialize serial communication
	// (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
	// it's really up to you depending on your project)
	Serial.begin(38400);
	Serial1.begin(9600);  //bluetooth
	inputString.reserve(20);
	Serial1.println(F("\nPress 'q' to see current PID settings"));
	Serial1.println(F("Press 'p' ,then type a number to change p value"));
	Serial1.println(F("Press 'i' ,then type a number to change i value"));
	Serial1.println(F("Press 'd' ,then type a number to change d value"));
	Serial1.println(F("Press 'w' to save pid to EEPROM"));
	Serial1.println(F("Press 'Enter' to confirm\n"));

	Serial.println("Initializing I2C devices...");
	mpu.initialize();

	// verify connection
	Serial.println("Testing device connections...");
	Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	pid = PID(pid_values.Kp, pid_values.Ki, pid_values.Kd);

	initMotors();

	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	double angleByAcc = atan2(ax, az);
	kalman.setAngle(angleByAcc);

	//calibrate();

	kalmanTimer = micros();
	pidTimer = kalmanTimer;
	imuTimer = millis();
	encoderTimer = imuTimer;
	reportTimer = imuTimer;

}

void balance(double angle_sensed, double dt)
{

	u = pid.control(target_angle, angle_sensed, dt);

	motors_control(u);
}

//pitch +=  rot_y*ms_5;
//pitch = angleByAcc*Rad2Deg*0.98+pitch*0.02;
//Serial.print("\t acc_A\t"); Serial.print(angleByAcc*Rad2Deg);
//Serial.print("\t comlementary\t"); Serial.print(pitch);

//rpm(&m1_rpm,&m2_rpm, 1);
//Serial.print("\trpm1:\t"); Serial.print(m1_rpm);
//Serial.print("\trpm2:\t"); Serial.print(m2_rpm);

void processBluetooth()
{
	while (Serial1.available()) {
		// get the new byte:
		char inChar = (char)Serial1.read();
		// add it to the inputString:
		inputString += inChar;
		//Serial.println(inChar);
		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\n' || inChar == '\r') {
			stringComplete = true;
		}
	}
	if (stringComplete) {

		if (inputString.startsWith("p") || inputString.startsWith("P"))
		{
			pid_values.Kp = inputString.substring(1).toFloat();
			Serial.print("P in pid set to:\t"); Serial.println(pid_values.Kp);
			Serial1.print("P in pid set to:\t"); Serial1.println(pid_values.Kp);
			pid = PID(pid_values.Kp, pid_values.Ki, pid_values.Kd);
		}
		if (inputString.startsWith("i") || inputString.startsWith("I"))
		{
			pid_values.Ki = inputString.substring(1).toFloat();
			Serial.print("I in pid set to:\t"); Serial.println(pid_values.Ki);
			Serial1.print("I in pid set to:\t"); Serial1.println(pid_values.Ki);
			pid = PID(pid_values.Kp, pid_values.Ki, pid_values.Kd);
		}
		if (inputString.startsWith("d") || inputString.startsWith("D"))
		{
			pid_values.Kd = inputString.substring(1).toFloat();
			Serial.print("D in pid set to:\t"); Serial.println(pid_values.Kd, 5);
			Serial1.print("D in pid set to:\t"); Serial1.println(pid_values.Kd, 5);
			pid = PID(pid_values.Kp, pid_values.Ki, pid_values.Kd);
		}
		if (inputString.startsWith("q") || inputString.startsWith("Q"))
		{
			Serial.print("P:"); Serial.print(pid_values.Kp);Serial.print("\tI:"); Serial.print(pid_values.Ki); Serial.print("\tD:");Serial.println(pid_values.Kd, 5);
			Serial1.print("P:"); Serial1.print(pid_values.Kp);Serial1.print("\tI:"); Serial1.print(pid_values.Ki); Serial1.print("\tD:");Serial1.println(pid_values.Kd, 5);
		}
		if (inputString.startsWith("w") || inputString.startsWith("W"))
		{
			pid_values.valid = VALID;
			EEPROM_writeAnything(0, pid_values);
			Serial1.print(F("saved in EEPROM\n"));
		}

		inputString = "";
		stringComplete = false;
	}
}
void stopAndReset()
{
  u = 0;
  motors_stop();
  pid.reset();
}

void loop()
{
	double dt;

	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


	//calculate pitch by kalman
	double rot_y = -(double)gy / GYRO_SENS; //to degree
	double angleByAcc = atan2(ax, az);

	currentTime = micros(); 
	dt = (currentTime - kalmanTimer) / 1000000.0f;
	double pitch = kalman.getAngle(angleByAcc*Rad2Deg, rot_y, dt);
	kalmanTimer = currentTime;



	//if error less than 35 degree, try to balance
  currentTime = micros(); 
	if (abs(pitch) < 35){	
		dt = (currentTime - pidTimer) / 1000000.0f;
		balance(pitch, dt);	
	}
	else{
		stopAndReset();
	}
  pidTimer = currentTime;

	currentTime = millis();
	if (currentTime - reportTimer >= 100)  //100 ms
	{
		Serial.print("\t angle\t"); Serial.print(pitch);
		Serial.print("\t control\t"); Serial.print(u);
    //Serial.print("\t acc_A\t"); Serial.print(angleByAcc*Rad2Deg);
		Serial.print("\n"); 
		if (frameCounter % 10 == 0){ //every 1 sec
			processBluetooth();
			frameCounter = 0;
		}
		frameCounter++;
		reportTimer = currentTime;
	}

}














