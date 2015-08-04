/* Copyright (C) 2015 Qian Qian. All rights reserved.
 E-mail   :  xeonqq@gmail.com
 */

#ifndef EKF_H
#define EKF_H

#include <math.h>

#define g 9.8
#define sqr(x) (x*x)
class EKF  // for 1 axis gyro and accerometer
{
public:
    EKF(){}

    void init(double angle, double var);
    void predict(double w, double noiseQ, double dt); // angular velocity (rad/s) and noise variance
    void correct(double acc, double noiseR); // acceleration (m/s^2) and noise variance
	
    double getAngle() const {return angle;}
private:
    double angle;  // in rad
    double var;
};
#endif //EKF_H
















