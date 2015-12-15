/* Copyright (C) 2015 Qian Qian. All rights reserved.
 E-mail   :  xeonqq@gmail.com
 */

#ifndef PID_H
#define PID_H


class PID
{
	public:
		PID(float Kp, float Ki, float Kd): 
			Kp(Kp), Ki(Ki), Kd(Kd), u(0), e_(0), e__(0){}
		PID(){};
		~PID(){};
		float control(float target, float sense, float dt);
		float cascade_control(float target_angle, float sensed_angle, float sensed_w, float dt);
    void reset();
	private:
		float Kp;
		float Ki;
		float Kd;
		float u;
		float e_, e__; // error at t-1 and t-2
		static const float Kp_w = 6.0f;
		static const float MAX_BALANCE_CTRL_VAR = 210.0f;
};

float PID::control(float target, float sense, float dt)  //"velocity" PID
{
	float e = target - sense;
	u += (Kp + Ki*dt + Kd/dt)*e - (Kp + 2*Kd/dt)*e_ + Kd/dt*e__;

	u = constrain(u, -MAX_BALANCE_CTRL_VAR, MAX_BALANCE_CTRL_VAR);

	e_ = e;
	e__ = e_;
	return u;
}

//angle in degrees, sensed_w in degrees/sec
float PID::cascade_control(float target_angle, float sensed_angle, float sensed_w, float dt)  //"velocity" PID
{
	//stage 1
	//the expected angular velocity, given the current error in angle
	//larger the Kp_w, larger the control value
	float target_w = (target_angle - sensed_angle)*Kp_w;

	//stage 2
	float e_w = target_w - sensed_w;
	u += (Kp + Ki*dt + Kd/dt)*e_w - (Kp + 2*Kd/dt)*e_ + Kd/dt*e__;
	
	u = constrain(u, -MAX_BALANCE_CTRL_VAR, MAX_BALANCE_CTRL_VAR);

	e_ = e_w;
	e__ = e_w;
	return u;
}

void PID::reset()
{
  e_ = 0;
  e__= 0;
  u = 0;
}

#endif //PID_H

















