#include "EKF.h"

template<class TYPE>
TYPE normalizeAngle(TYPE angle) {
    const TYPE pi2 = M_PI * 2;
    while (angle > M_PI) angle -= pi2;
    while (angle < -M_PI) angle += pi2;

    return angle;
}

void EKF::init(double angle, double var)
{
	angle = angle;
	var = var;
}

void EKF::predict(double w, double noiseQ, double dt)
{
	angle += w*dt;
	var += noiseQ;
}

void EKF::correct(double acc, double noiseR)
{
	double inno = acc-sin(angle)*g;
	double H = g*cos(angle);
	double S = sqr(H) * var + noiseR;  
	double K = var * H / S;
	angle = normalizeAngle(angle + K*inno);
	var =  (1-K*H)*var;	
}














