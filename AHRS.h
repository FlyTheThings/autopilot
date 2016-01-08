/*
 * AHRS.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef AHRS_H_
#define AHRS_H_

#define ATTITUDE_UPDATE_FREQ 100

#define Kp 0.3f			// proportional gain governs rate of convergence to accelerometer/magnetometer, sekunnissa virheestä.
#define Ki 0.000f		// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.005f	// half the sample period

#include "autopilot.h"
#include <math.h>

extern int gx, gy, gz, ax, ay, az;

extern float gxf, gyf, gzf, prevCrs, tempCrs, deltaCrs;

extern float q0;
extern float q1;
extern float q2;
extern float q3;
extern float exInt;	// scaled integral error
extern float eyInt;
extern float ezInt;

char mpu9250SpiRead(char reg);
void mpu9250SpiWrite(char reg, char val);
void MPU9250Init(void);
void calibrateAttitude(void);
void attitudeUpdate(void);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);

#endif /* AHRS_H_ */
