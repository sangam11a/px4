/*
 * Px4.h
 *
 *  Created on: Dec 22, 2023
 *      Author: User
 */

#ifndef SRC_PX4_H_
#define SRC_PX4_H_

#include "main.h"
#include "RCFilter.h"

RCFilter lpfAcc[3];
RCFilter lpfGyr[3];


#define RAD_TO_DEG 		57.2957795131f
#define G_MPS2 9.8100000000f

float phiHat_deg_ = 0.0f;
float thetaHat_deg_ = 0.0f;

float phiHat_rad_ = 0.0f;
float thetaHat_rad_ = 0.0f;

float KALMAN_P_INIT = 0.1f;
float KALMAN_Q = 0.001f;
float KALMAN_R = 0.011f;

typedef struct {


	uint8_t readingAcc;
	uint8_t readingGyr;
	uint8_t accTxBuf[6];
	uint8_t gyrTxBuf[7];
	volatile uint8_t accRxBuf[6];
	volatile uint8_t gyrRxBuf[7];

	/* Conversion constants (raw to m/s^2 and raw to rad/s) */
	float accConversion;
	float gyrConversion;

	/* x-y-z measurements */
	float acc_mps2[3];
	float gyr_rps[3];

} Px4;


//IMU
void IMU_ReadAccelerometerDMA_Complete(Px4 *imu);
#endif /* SRC_PX4_H_ */
