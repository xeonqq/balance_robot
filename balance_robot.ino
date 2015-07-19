
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#include "PID.h"
#include "EKF.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

/*
The output scale for any setting is [-32768, +32767] for each of the six axes. The default setting in the I2Cdevlib class is +/- 2g for the accel and +/- 250 deg/sec for the gyro.
so 
*/
//#define ACC_SENS 16384  //=1g
//#define GYRO_SENS 131   //=1 degree/s
#define ACC_SENS 1671.8367346938774   //  (16384/9.8)  // =1 m/s^2
#define GYRO_SENS 7505.7471145    //  (131*57.2957795)   // =1 rad/s

int16_t ax, ay, az;
int16_t gx, gy, gz;


PID pid;
EKF ekf;

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  double Kp = 0.2;
  double Ki = 0.4;
  double Kd = 0.1;
  double dt = 20; //ms
  pid = PID(Kp, Ki, Kd, dt);
  ekf.init(0, 0.274);  //initial variance 30 deg

}

unsigned long lastUpdateTime=millis();
double random_walk_noise = 4.0e-06;
double gyro_white_noise = 0.005;

void loop()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double acc_y = (double)ay/ACC_SENS;
    double rot_y = (double)gy/GYRO_SENS;
        //Serial.print("gy:\t"); Serial.print(gy);
    //Serial.print("w_y:\t"); Serial.print(rot_y);
    //Serial.print("acc_y:\t"); Serial.print(acc_y);
    unsigned long t = millis();	
    unsigned long dt = t - lastUpdateTime;
    lastUpdateTime = t;
    ekf.predict(rot_y, random_walk_noise*rot_y+gyro_white_noise, 0.01);
    
    ekf.correct(acc_y, 0.01);
    Serial.print("angle:\t"); Serial.print(ekf.getAngle()); 
    Serial.print("\n");
    delay(10);
}



