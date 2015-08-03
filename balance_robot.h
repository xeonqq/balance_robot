#define TASK_100HZ 2
#define TASK_50HZ 4
#define TASK_10HZ 20
#define TASK_1HZ 200

#define ms_10 0.01  //unit second 

#define ms_5 0.005  //unit second 

// main loop time variable
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

unsigned long frameCounter = 0; // main loop executive frame counter
/*
The output scale for any setting is [-32768, +32767] for each of the six axes. The default setting in the I2Cdevlib class is +/- 2g for the accel and +/- 250 deg/sec for the gyro.
so 
*/
#define GYRO_DEGREE

#ifdef GYRO_DEGREE
#define GYRO_SENS 131   //=1 degree/s
#endif

#ifdef GYRO_RAD
#define GYRO_SENS 7505.7471145    //  (131*57.2957795)   // =1 rad/s
#endif
//#define ACC_SENS 16384  //=1g
//#define GYRO_SENS 131   //=1 degree/s
#define ACC_SENS 1671.8367346938774   //  (16384/9.8)  // =1 m/s^2


double gyro_random_walk_noise = 4.0e-06;

//taken from datasheet
double gyro_white_noise_density = 0.005;  //   degree/s/√Hz

//taken from datasheet
double accelerometer_noise_density = 0.00392;  //400 mug/√Hz -> convert to m/s^2/√Hz









