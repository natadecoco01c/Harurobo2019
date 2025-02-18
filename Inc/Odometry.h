/*
 * Odometry.h
 *
 *  Created on: Feb 14, 2018
 *      Author: yusaku
 */
#pragma once

#include "MPU9250.h"
#include <cmath>

#define SPI_MPU9250  SPI2
#define GPIO_MPU9250 GPIOB
#define PIN_MPU9250  GPIO_PIN_12

class Odometry
{
private:
//	float x; //main.cppからアクセスするためにpublicへ移動
//	float y;
//	float yaw;

	MPU9250 *mpu9250 = nullptr;

	// diameter of wheels in metre
	static constexpr float WheelDiameter = 0.060; //タイヤの直径によって変更
	// pulse/rev
	static constexpr float PulsePerRevolution = 100.0 * 4; //パルス 春ロボ100パルスで行くの？
	/// Kpd = 2_pi_r[mm/rev] / Kp[pulse/rev]
	static constexpr float MPerPulse = M_PI * WheelDiameter / PulsePerRevolution;

	// moving average in milli-degree-per-second
	int movavg;

	void GetGyroBias(float * const avg, float * const stdev) const;
	bool InitGyro(void);

	void ReadEncoder(void);
	void ReadGyro(void);

public:
	float x;
	float y;
	float yaw;
	static constexpr float margin = 0.176848f; //オドメトリの座標原点と機体中心を合わせるためのもの　単位はm
	Odometry(void);

	bool Initialize(void);

	void Sample(void);
	void SetPose(float x, float y, float yaw);
	void GetPose(float *x, float *y, float *yaw);

	static constexpr int32_t SamplingFrequency = 200; //TIM1の割り込み周波数と一致
};



















