

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>  // for type definitions

#define REDUCTION_RATIO 21.3
#define ENCODER_RES 334
//pin 2 and pin 3 are (SDA) and (SCL) for MPU6050  // 3 (interrupt 0), 2 (interrupt 1),
//
unsigned char E1 = 5;  //PWMA

unsigned char M1 = 4;  //DIRA

unsigned char E2 = 6;  //PWMB             

unsigned char M2 = 7;  //DIRB

unsigned char INT_M1 = 10;  //encoder interrupt pin for motor1         

unsigned char INT_M2 = 11;  //encoder interrupt pin for motor2


unsigned long encoder_M1_cnt = 0;
unsigned long encoder_M2_cnt = 0;
unsigned long last_encoder_M1_cnt=0;
unsigned long last_encoder_M2_cnt=0;

unsigned int motor1_initiate_pwm=0;
unsigned int motor2_initiate_pwm=0;


//get from motor calibration
int forward_min_pwm_m1 = 42;
int forward_min_pwm_m2 = 55 +10;
int backward_min_pwm_m1 = -34 -10;
int backward_min_pwm_m2 = -39 -10;

void encoder_M1()
{
	encoder_M1_cnt+=1;
}

void encoder_M2()
{
	encoder_M2_cnt+=1;
}

//supposed to be run in a loop
void rpm(double *rpm1, double *rpm2, double dt)  //dt = interval of measurement
{
	*rpm1 = (double)(encoder_M1_cnt - last_encoder_M1_cnt)/(ENCODER_RES*REDUCTION_RATIO)*60/dt;
	*rpm2 = (double)(encoder_M2_cnt - last_encoder_M2_cnt)/(ENCODER_RES*REDUCTION_RATIO)*60/dt;
	last_encoder_M1_cnt = encoder_M1_cnt;
	last_encoder_M2_cnt = encoder_M2_cnt;
}



void initMotors()
{
	pinMode(M1, OUTPUT); 
	pinMode(M2, OUTPUT);
	pinMode(INT_M1, INPUT); 
	pinMode(INT_M2, INPUT);
	enableInterrupt(INT_M1, encoder_M1 , FALLING);
	enableInterrupt(INT_M2, encoder_M2 , FALLING);
}

void motors_stop()
{
	digitalWrite(M1,HIGH);	
	analogWrite(E1, 0);   //0~255
	digitalWrite(M2,HIGH);	
	analogWrite(E2, 0);   //0~255
}

void motor1_pwm(int u) //-255 ~ 255
{
	if (u > 0){  //  FORWARD
		digitalWrite(M1,HIGH);	
		analogWrite(E1, u);   //0~255
	}
	else{	// BACKWARD
		digitalWrite(M1,LOW);         
		analogWrite(E1, -u);   //0~255
	}
}

void motor2_pwm(int u)
{
	if (u > 0){  //  FORWARD 
		digitalWrite(M2, LOW);
		analogWrite(E2, u);   //0~255
	}
	else{	// BACKWARD
		digitalWrite(M2, HIGH);
		analogWrite(E2, -u);   //0~255
	}
}

void motors_control(int u) // control signal from pid
{
	u = min(255, max(u, -255));  //constrain value between -255 to 255
	//Serial.print(u);
	int pwm1, pwm2;
	if (u>0){
		pwm1 = map(u, 0, 255, forward_min_pwm_m1, 255);  
		//if (pwm1 < forward_min_pwm_m2)
		//	pwm2 = forward_min_pwm_m2;
		//else
		pwm2 = min(1.022*pwm1  - 0.8889, 255);

		digitalWrite(M1,HIGH);	
		analogWrite(E1, pwm1);   //0~255
		digitalWrite(M2, LOW);
		analogWrite(E2, pwm2);   //0~255
	}
	else if (u<0){
		pwm1 = map(u, -255, 0, -255, -forward_min_pwm_m1);//backward_min_pwm_m1);  
		//pwm1 = backward_min_pwm_m1 + u;
		pwm2 = max(1.0299*pwm1  + 0.5467, -255);
		//pwm1 = min(pwm1, -255);
		//pwm2 = min(pwm2, -255);
		digitalWrite(M1,LOW);         
		analogWrite(E1, -pwm1);   //0~255
		digitalWrite(M2, HIGH);
		analogWrite(E2, -pwm2);   //0~255
	}
	else
		motors_stop();

	//Serial.print("\t");
	//Serial.print(pwm1);
	//Serial.print("\t");
	//Serial.print(pwm2);
}

void motor1_move(int u) //-255 ~ 255
{
	int real_speed = map(abs(u), 0, 255, forward_min_pwm_m1, 255);  
	if (u > 0){  //  FORWARD
		digitalWrite(M1,HIGH);	
	}
	else{	// BACKWARD
		digitalWrite(M1,LOW);         
	}
	analogWrite(E1, real_speed);   //0~255
}

void motor2_move(int u)
{
  int real_speed = map(abs(u), 0, 255, forward_min_pwm_m2, 255);  
	if (u > 0){  //  FORWARD 
		digitalWrite(M2, LOW);
	}
	else{	// BACKWARD
		digitalWrite(M2, HIGH);
	}
	analogWrite(E2, real_speed);   //0~255
}



void calibrate()  // use with python script on pc, send (pwm, rpm1, rpm2) to pc
{
	int i;
	double rpm1, rpm2;
	bool m1_finish;
	bool m2_finish;
	
	String inputString = "";         // a string to hold incoming data
	while(inputString != "hello motor")
	{
		while (Serial.available())
		{
			char inChar = (char)Serial1.read();
			// add it to the inputString:
			inputString += inChar;
			Serial.println(inputString);
		}
	}
	rpm(&rpm1, &rpm2, 1);
	for (i=-255;i < 256; i++)
	{
		motor1_pwm(i);
		motor2_pwm(i);
		delay(1000);
		rpm(&rpm1, &rpm2, 1);
		Serial.print(i);
		Serial.print("\t");
		Serial.print(rpm1);
		Serial.print("\t");
		Serial.print(rpm2);
		Serial.print("\n");
	}    
}




#endif










