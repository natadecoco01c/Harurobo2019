/*
 * Odometry.cpp
 *
 *  Created on: Feb 14, 2018
 *      Author: yusaku
 */

#include "Odometry.h"
#include "MadgwickAHRS.h"
//#include "stm32f10x_conf.h"
//#include "Timer.h"

#include <cmath>

Madgwick MDGF;

Odometry::Odometry(void) {

	this->x = 0.0f;
	this->y = 0.0f;
	this->yaw = 0.0f;

	//this->mpu9250 = new MPU9250(SPI_MPU9250, GPIOC, GPIO_PIN_0);
	this->mpu9250 = new MPU9250(SPI_MPU9250, GPIO_MPU9250, PIN_MPU9250);
}

void Odometry::GetBias(float * const avg, float * const stdev) const {
	static constexpr int NumOfTrial = 256;

	float _avg[6] = { };
	float _stdev[6] = { };

	for (int i = 0; i < NumOfTrial; i++) {
		float reading[6] = { };
		reading[0] = (int16_t) mpu9250->WriteWord(
		READ_FLAG | MPUREG_GYRO_XOUT_H, 0x0000) * 1000.0f
				/ GyroSensitivityScaleFactor;
		reading[1] = (int16_t) mpu9250->WriteWord(
		READ_FLAG | MPUREG_GYRO_YOUT_H, 0x0000) * 1000.0f
				/ GyroSensitivityScaleFactor;
		reading[2] = (int16_t) mpu9250->WriteWord(
		READ_FLAG | MPUREG_GYRO_ZOUT_H, 0x0000) * 1000.0f
				/ GyroSensitivityScaleFactor;
		reading[3] = (int16_t) mpu9250->WriteWord(
		READ_FLAG | MPUREG_ACCEL_XOUT_H, 0x0000) * 1000.0f
				/ AccSensitivityScaleFactor;
		reading[4] = (int16_t) mpu9250->WriteWord(
		READ_FLAG | MPUREG_ACCEL_YOUT_H, 0x0000) * 1000.0f
				/ AccSensitivityScaleFactor;
		reading[5] = (int16_t) mpu9250->WriteWord(
		READ_FLAG | MPUREG_ACCEL_ZOUT_H, 0x0000) * 1000.0f
				/ AccSensitivityScaleFactor;
		for (int j = 0; j < 6; j++) {
			_avg[j] += reading[j];
			_stdev[j] += reading[j] * reading[j];
		}
		//Timer::sleep(5);
		HAL_Delay(4); //６つ分やるとすごい時間になるので短く 5->1
	}

	for (int k = 0; k < 6; k++) {
		_avg[k] /= NumOfTrial;

		_stdev[k] -= NumOfTrial * _avg[k] * _avg[k];
		_stdev[k] /= NumOfTrial - 1;
		_stdev[k] = sqrtf(_stdev[k]);

		avg[k] = _avg[k];
		stdev[k] = _stdev[k];
	}
}

bool Odometry::InitGyro(void) {
	uint8_t whoami = mpu9250->WriteByte(READ_FLAG | MPUREG_WHOAMI, 0x00);

	if (whoami != 0x71) {
		delete mpu9250;
		return false;
	}

	// get stable time source
	mpu9250->WriteByte(MPUREG_PWR_MGMT_1, 0x03); // Set clock source to be PLL with z-axis gyroscope reference, bits 2:0 = 011

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 4000 and 250 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 000; this sets the sample rate at 8 kHz for both
	// Maximum delay is 0.97 ms which is just over a 1 kHz maximum rate
	//mpu9250->WriteByte(MPUREG_CONFIG, 0x00);
	mpu9250->WriteByte(MPUREG_CONFIG, 0x03);	//1khz

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	//mpu9250->WriteByte(MPUREG_SMPLRT_DIV, 0x07);  // Use a 1 kHz rate; the same rate set in CONFIG above
	//mpu9250->WriteByte(MPUREG_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
	mpu9250->WriteByte(MPUREG_SMPLRT_DIV, 0x00); //1khzでサンプリング

	mpu9250->WriteByte(MPUREG_GYRO_CONFIG, BITS_FS_1000DPS);
	mpu9250->WriteByte(MPUREG_ACCEL_CONFIG, 0b11000);

	//Timer::sleep(100);
	HAL_Delay(100);

	float avg[6] = { };
	float stdev[6] = { 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0 };

	for (int i = 0; i < 10; i++) {
		this->GetBias(avg, stdev);

		if (stdev[0] < 700 && stdev[1] < 700 && stdev[2] < 700 && stdev[3] < 700
				&& stdev[4] < 700 && stdev[5] < 700) {
			movavg[0] = (int32_t) avg[0];
			movavg[1] = (int32_t) avg[1];
			movavg[2] = (int32_t) avg[2];
			movavg[3] = (int32_t) avg[3];
			movavg[4] = (int32_t) avg[4];
			movavg[5] = (int32_t) avg[5];

			MDGF.begin(this->SamplingFrequency);

			return true;
		}
	}

	// gyro unit is not in desirable state (not stabilized)
	return false;
}

void Odometry::ReadEncoder(void) {
	volatile int16_t _p1 = static_cast<int16_t>(TIM3->CNT);
	TIM3->CNT = 0;

	volatile int16_t _p2 = (-1)*static_cast<int16_t>(TIM4->CNT);
	TIM4->CNT = 0;

	// just a simple rotation matrix
	// translate encoder rates to velocity on x-y plane
	float _yaw = yaw; //- ((float) M_PI / 4.0f); //いじるとしたらこの辺　ジャイロの付け方に依る
	float _cos = cosf(_yaw);
	float _sin = sinf(_yaw);

	x += ((_p1 * _cos) - (_p2 * _sin)) * MPerPulse;
	y += ((_p1 * _sin) + (_p2 * _cos)) * MPerPulse;
}

void Odometry::ReadAccGyro(void) {
	static constexpr int32_t ang_movband = 250;
//	static constexpr int32_t acc_movband = 150;
//	static constexpr float RadPerMilliDeg = M_PI / 180000.0;
//	static constexpr float RadPerMilliDegPerSec = RadPerMilliDeg
//			/ SamplingFrequency;
	static constexpr float ang_w = 0.01f; //追従の強さ
//	static constexpr float acc_w = 0.1f;
//	static const float offset_G = movavg[5] / 1000.0f; //Z軸重力の分,フィルタで消えてしまうので足す

	int raw[6];
	float data[6];

//	static uint32_t lasttime=0;
//	static uint16_t dt=0;

	raw[0] = (((int16_t) mpu9250->WriteWord(READ_FLAG | MPUREG_GYRO_XOUT_H,
			0x0000)) * 1000 / GyroSensitivityScaleFactor) + 0.5f;
	raw[1] = (((int16_t) mpu9250->WriteWord(READ_FLAG | MPUREG_GYRO_YOUT_H,
			0x0000)) * 1000 / GyroSensitivityScaleFactor) + 0.5f;
	raw[2] = (((int16_t) mpu9250->WriteWord(READ_FLAG | MPUREG_GYRO_ZOUT_H,
			0x0000)) * 1000 / GyroSensitivityScaleFactor) + 0.5f;
	raw[3] = (((int16_t) mpu9250->WriteWord(READ_FLAG | MPUREG_ACCEL_XOUT_H,
			0x0000)) * 1000 / AccSensitivityScaleFactor) + 0.5f;
	raw[4] = (((int16_t) mpu9250->WriteWord(READ_FLAG | MPUREG_ACCEL_YOUT_H,
			0x0000)) * 1000 / AccSensitivityScaleFactor) + 0.5f;
	raw[5] = (((int16_t) mpu9250->WriteWord(READ_FLAG | MPUREG_ACCEL_ZOUT_H,
			0x0000)) * 1000 / AccSensitivityScaleFactor) + 0.5f;

	//static constexpr float halfPi = M_PI / 2.0;

//	int dy_raw_mdps = (((int16_t) mpu9250->WriteWord(
//	READ_FLAG | MPUREG_GYRO_ZOUT_H, 0x0000)) * 1000 / SensitivityScaleFactor)
//			+ 0.5f;
	//temp = mpu9250->WriteWord(READ_FLAG | MPUREG_TEMP_OUT_H, 0x0000);

	for (int i = 0; i < 3; i++) {
		biased[i] = raw[i] - movavg[i];

		if (biased[i] < -ang_movband || ang_movband < biased[i]) {
			// yaw is in radian, so, convert from mdps to radian.
//		yaw += (float) dy_biased_mdps * RadPerMilliDegPerSec;
//
//		if (yaw > (float) M_PI) {
//			yaw -= (2.0f * (float) M_PI);
//		} else if (yaw < -(float) M_PI) {
//			yaw += (2.0f * (float) M_PI);
//		}

			data[i] = biased[i] / 1000.0f;
		} else {
			data[i] = 0.0f;
			movavg[i] = (int) ((((float) movavg[i] * (1 - ang_w))
					+ ((float) raw[i] * ang_w)) + 0.5f);
		}
	}
//	for (int i = 3; i < 5; i++) {
//		biased[i] = raw[i] - movavg[i];
//
//		if (biased[i] < -acc_movband || acc_movband < biased[i]) {
//			data[i] = biased[i] / 1000.0f;
//		} else {
//			data[i] = 0.0f;
//			movavg[i] = (int) ((((float) movavg[i] * (1 - acc_w))
//					+ ((float) raw[i] * acc_w)) + 0.5f);
//		}
//	}
//
//	biased[5] = raw[5] - movavg[5];
//
//	if (biased[5] < -acc_movband || acc_movband < biased[5]) {
//		data[5] = (biased[5] / 1000.0f) + offset_G;
//	} else {
//		data[5] = offset_G;
//		movavg[5] = (int) ((((float) movavg[5] * (1 - acc_w))
//				+ ((float) raw[5] * acc_w)) + 0.5f);
//	}
	for(int i=3;i<6;i++){ //多分やんなくてもいい
		data[i]=raw[i]/1000.0f;
	}

	MDGF.updateIMU(data[0], data[1], data[2], data[3], data[4], data[5]);
//	dt = HAL_GetTick()-lasttime;
//	lasttime=HAL_GetTick();
	this->yaw = MDGF.getYawRadians();
}

bool Odometry::Initialize(void) {
	return this->InitGyro();
}

void Odometry::Sample(void) {
	this->ReadEncoder();
	this->ReadAccGyro();
}

void Odometry::SetPose(const float x, const float y, const float yaw) {
	this->x = x;
	this->y = y;
	this->yaw = yaw;
}

void Odometry::GetPose(float * const x, float * const y, float * const yaw) {
	*x = this->x;
	*y = this->y;
	*yaw = this->yaw;
}

