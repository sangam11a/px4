/*
 * Px4.h
 *
 *  Created on: Dec 22, 2023
 *      Author: User
 */

#ifndef SRC_PX4_H_
#define SRC_PX4_H_

// #include "main.h"
#include "RCFilter.h"
#include "EKF.h"

RCFilter lpfAcc[3];
RCFilter lpfGyr[3];

#define RAD_TO_DEG 		57.2957795131f
#define G_MPS2 9.8100000000f

#define SAMPLE_TIME_MS_USB_ 1000

#define IMU_ACC_RAW_TO_MPS2 0.00059875482f
#define IMU_GYR_RAW_TO_RPS 0.00013323124

#define COMP_FILT_ALPHA 0.0500000000f

#define KALMAN_PREDICT_PERIOD_MS 10
#define KALMAN_UPDATE_PERIOD_MS 100

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
void IMU_ReadGyroscopeComplete(Px4 *imu);
void Comlementary_Filter(Px4 *imu);
void EKF_AccelGyro(Px4 *imu, EKF *ekf);
void IMU_ReadAccelerometerComplete(Px4 *imu);
#endif /* SRC_PX4_H_ */
