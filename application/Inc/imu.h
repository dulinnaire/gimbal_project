//
// Created by 81301 on 2024/12/11.
//

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "BMI088.h"

void imu_init();

void imu_get_gyro(float* rate_x, float* rate_y, float* rate_z);
void imu_get_accel(float* accel_x, float* accel_y, float* accel_z);

void imu_update();

float imu_get_pitch_angle();

#ifdef __cplusplus
}
#endif

#endif //IMU_H
