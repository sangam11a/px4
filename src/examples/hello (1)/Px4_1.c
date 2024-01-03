/*
 * Px4.c
 *
 *  Created on: Dec 22, 2023
 *      Author: User
 */
#include "Px4_1.h"
#include "math.h"

float phiHat_deg_ = 0.0f;
float thetaHat_deg_ = 0.0f;

void IMU_ReadAccelerometerComplete(Px4 *imu) {

	// /* Pre-compute accelerometer conversion constant (raw to m/s^2) */
	// imu->accConversion = 1 / 16384.0f;

	// /* Pre-compute gyroscope conversion constant (raw to rad/s) */
	// imu->gyrConversion = 0.01745329251f / 131.0f; /* Datasheet page 39 */

	// /* Form signed 16-bit integers */
	// int16_t accX = (int16_t) ((imu->accRxBuf[0] << 8) | imu->accRxBuf[1]);
	// int16_t accY = (int16_t) ((imu->accRxBuf[2] << 8) | imu->accRxBuf[3]);
	// int16_t accZ = (int16_t) ((imu->accRxBuf[4] << 8) | imu->accRxBuf[5]);

	// /* Convert to m/s^2 */
	// imu->acc_mps2[0] = imu->accConversion * accX;
	// imu->acc_mps2[1] = imu->accConversion * accY;
	// imu->acc_mps2[2] = imu->accConversion * accZ;

	// imu->acc_mps2[0] = ;
	// imu->acc_mps2[`] = ;
	// imu->acc_mps2[0] = ;

	/* Filter accelerometer data */
	RCFilter_Update(&lpfAcc[0], imu->acc_mps2[0]);
	RCFilter_Update(&lpfAcc[1], imu->acc_mps2[1]);
	RCFilter_Update(&lpfAcc[2], imu->acc_mps2[2]);

	//Filtered accelerometer measurement
	float ax_mps2 = lpfAcc[0].out[0];
	float ay_mps2 = lpfAcc[1].out[0];
	float az_mps2 = lpfAcc[2].out[0];

	/*Calculate roll (phi) and pitch(theta) angle estimates using filtered accelerometer readings*/
	phiHat_deg_ = atanf(ay_mps2 / az_mps2) * RAD_TO_DEG;
	thetaHat_deg_ = asinf(ax_mps2 / G_MPS2) * RAD_TO_DEG;

}

