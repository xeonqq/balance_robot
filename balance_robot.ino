#include "PID.h"

PID pid;

void setup()
{
	double Kp = 0.2;
 	double Ki = 0.4;
	double Kd = 0.1;
	double dt = 20; //ms
	pid = PID(Kp, Ki, Kd, dt);
}

void loop()
{

}

