//
// Created by 81301 on 2024/11/2.
//

#ifndef BMI088_H
#define BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"
#include "spi.h"
#include "stm32f4xx.h"

#define BMI088_TIMEOUT 1000
#define DUMMY_BYTE 0xFF

// register addresses
// accel
#define ACC_CHIP_ID 0x00
#define ACC_DATA 0x12
#define ACC_CONF 0x40
#define ACC_RANGE 0x41
#define ACC_PWR_CONF 0x7C
#define ACC_PWR_CTRL 0x7D

// gyro
#define GYRO_CHIP_ID 0x00
#define RATE_DATA 0x02
#define GYRO_BANDWIDTH 0x10
#define GYRO_RANGE 0x0F

// configure IMU
void BMI088_init();

// get data
void BMI088_get_accel_data_int(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z);
void BMI088_get_gyro_data_int(int16_t* rate_x, int16_t* rate_y, int16_t* rate_z);

// unit conversion
void BMI088_get_accel_data(float* accel_x, float* accel_y, float* accel_z);
void BMI088_get_gyro_data(float* accel_x, float* accel_y, float* accel_z);

#ifdef __cplusplus
}
#endif

#endif //BMI088_H
