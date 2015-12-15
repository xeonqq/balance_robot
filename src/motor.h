

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


volatile unsigned long encoder_M1_cnt = 0;
volatile unsigned long encoder_M2_cnt = 0;
unsigned long last_encoder_M1_cnt=0;
unsigned long last_encoder_M2_cnt=0;

unsigned int motor1_initiate_pwm=0;
unsigned int motor2_initiate_pwm=0;

const uint16_t MOTOR_INIT_PWM = 45;
uint16_t pwm;


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


void motors_control_direct(int16_t u) // control signal from pid
{
	 //constrain value between -255 to 255
	//Serial.print(u);
	if (u>0){
		pwm = MOTOR_INIT_PWM + u;
		pwm = constrain(pwm, 0, 255);
		digitalWrite(M1,HIGH);	
		analogWrite(E1, pwm);   //0~255
		digitalWrite(M2, LOW);
		analogWrite(E2, pwm);   //0~255
	}
	else if (u<0){
		pwm = -(-MOTOR_INIT_PWM + u);
		pwm = constrain(pwm, 0, 255);
		digitalWrite(M1,LOW);         
		analogWrite(E1, pwm);   //0~255
		digitalWrite(M2, HIGH);
		analogWrite(E2, pwm);   //0~255
	}
	else{
		motors_stop();
	}
}

void motors_control_sep(int16_t m1, int16_t m2) // control signal from pid
{
	 //constrain value between -255 to 255
	//Serial.print(u);
	if (m1>0){
		m1 = MOTOR_INIT_PWM + m1;
		m1 = constrain(m1, 0, 255);
		digitalWrite(M1,HIGH);	
		analogWrite(E1, pwm);   //0~255
	}
	else if(m1<0){
		m1 = -(-MOTOR_INIT_PWM + m1);
		m1 = constrain(m1, 0, 255);
		digitalWrite(M1,LOW);         
		analogWrite(E1, m1);   //0~255
	}
	else{ //stop
		digitalWrite(M1,HIGH);	
		analogWrite(E1, 0);   //0~255
	}

	if (m2>0){
		m2 = MOTOR_INIT_PWM + m2;
		m2 = constrain(m2, 0, 255);
		digitalWrite(M2, LOW);
		analogWrite(E2, m2);   //0~255
	}
	else if (m2<0){
		m2 = -(-MOTOR_INIT_PWM + m2);
		m2 = constrain(m2, 0, 255);
		digitalWrite(M2, HIGH);
		analogWrite(E2, m2);   //0~255
	}
	else{ //stop
		digitalWrite(M2,HIGH);	
		analogWrite(E2, 0);   //0~255
	}
}

//To find the function f(pwm)=rpm, and then try to make both motor run at same rpm when pwm are same
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











