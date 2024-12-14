//
// Created by 81301 on 2024/12/11.
//
#include "imu.h"

#include "math.h"

#define RAD2DEG (180 / 3.1415926)

float imu_pitch_accel = 0, imu_pitch_gyro = 0;
float imu_pitch_angle = 0;
float alpha = 0.9;
float sample_period = 0.001; // 1ms

float rate_x = 0, rate_y = 0, rate_z = 0;
float accel_x = 0, accel_y = 0, accel_z = 0;

void imu_get_gyro(float* rate_x, float* rate_y, float* rate_z);
void imu_get_accel(float* accel_x, float* accel_y, float* accel_z);
float calculate_pitch_accel(float accel_x, float accel_y, float accel_z);
float calculate_pitch_gyro(float pitch_gyro, float rate_x, float rate_y, float rate_z);

void imu_init() {
    BMI088_init();

    imu_get_accel(&accel_x, &accel_y, &accel_z);
    imu_pitch_accel = calculate_pitch_accel(accel_x, accel_y, accel_z);
    imu_pitch_angle = imu_pitch_accel;

    imu_pitch_gyro = imu_pitch_angle;
}

void imu_get_gyro(float* rate_x, float* rate_y, float* rate_z) {
    BMI088_get_gyro_data(rate_x, rate_y, rate_z);
}

void imu_get_accel(float* accel_x, float* accel_y, float* accel_z) {
    BMI088_get_accel_data(accel_x, accel_y, accel_z);
}

float calculate_pitch_accel(float accel_x, float accel_y, float accel_z) {
    return -atan(accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RAD2DEG;
}

float calculate_pitch_gyro(float pitch_gyro, float rate_x, float rate_y, float rate_z) {
    return pitch_gyro + rate_x * sample_period;
}

void imu_update() {
    imu_get_accel(&accel_x, &accel_y, &accel_z);
    imu_get_gyro(&rate_x, &rate_y, &rate_z);

    imu_pitch_accel = calculate_pitch_accel(accel_x, accel_y, accel_z);
    imu_pitch_gyro = calculate_pitch_gyro(imu_pitch_gyro, rate_x, rate_y, rate_z);

    imu_pitch_angle = alpha * imu_pitch_gyro + (1 - alpha) * imu_pitch_accel;

    imu_pitch_gyro = imu_pitch_angle;
}

float imu_get_pitch_angle() {
    return imu_pitch_angle;
}
