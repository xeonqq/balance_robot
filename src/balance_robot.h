/* Used for timing */
uint32_t currentTime;
static uint32_t kalmanTimer; // Timer used for the Kalman filter
static uint32_t previous200HzTimer; // 200hz Timer used for the balance loop
static uint32_t previous100HzTimer; // 100hz Timer used for the yaw control loop
static uint32_t imuTimer; // This is used to set a delay between sending IMU values
static uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
static uint32_t reportTimer; // Timer used used to determine when to update the encoder values

//5 ms
const uint32_t TASK_200HZ = 5;
//10 ms
const uint32_t TASK_100HZ = 10;
//50 ms
const uint32_t TASK_20HZ = 50;
//100 ms
const uint32_t TASK_10HZ = 100;
//1000 ms
const uint32_t TASK_1HZ = 1000;

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


#define Bluetooth Serial1 

//in degree
const float MAX_PITCH = 4.5f;

//in degree/s
const float MAX_YAW_RATE = 200.0f;





#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }


