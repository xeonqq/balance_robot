/* Copyright (C) 2015 Qian Qian. All rights reserved.
 E-mail   :  xeonqq@gmail.com
 */

#ifndef PID_H
#define PID_H


class PID
{
	public:
		PID(double Kp, double Ki, double Kd): 
			Kp(Kp), Ki(Ki), Kd(Kd), u(0), e_(0), e__(0){}
		PID(){};
		~PID(){};
		double control(double target, double sense, double dt);
    void reset();
	private:
		double Kp;
		double Ki;
		double Kd;
		double u;
		double e_, e__; // error at t-1 and t-2
};

double PID::control(double target, double sense, double dt)  //"velocity" PID
{
	double e = target - sense;
	u += (Kp + Ki*dt + Kd/dt)*e - (Kp + 2*Kd/dt)*e_ + Kd/dt*e__;

	e_ = e;
	e__ = e_;
	return u;
}

void PID::reset()
{
  e_ = 0;
  e__= 0;
}

#endif //PID_H

















