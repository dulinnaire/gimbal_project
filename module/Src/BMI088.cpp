//
// Created by 81301 on 2024/11/2.
//
#include "BMI088.h"

// configure IMU
void BMI088_init();

void BMI088_cs_accel_h();
void BMI088_cs_accel_l();
void BMI088_cs_gyro_h();
void BMI088_cs_gyro_l();

uint8_t BMI088_exchange_byte(uint8_t txdata);

void BMI088_accel_write_reg(uint8_t reg, uint8_t data);
void BMI088_gyro_write_reg(uint8_t reg, uint8_t data);

uint8_t BMI088_accel_read_reg(uint8_t reg);
uint8_t BMI088_gyro_read_reg(uint8_t reg);

void BMI088_accel_burst_read(uint8_t reg, uint8_t rxdata[], uint8_t size);
void BMI088_gyro_burst_read(uint8_t reg, uint8_t rxdata[], uint8_t size);

void BMI088_get_accel_data_int(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z);
void BMI088_get_gyro_data_int(int16_t* rate_x, int16_t* rate_y, int16_t* rate_z);
void BMI088_get_accel_data(float* accel_x, float* accel_y, float* accel_z);
void BMI088_get_gyro_data(float* accel_x, float* accel_y, float* accel_z);

void BMI088_init() {
    // switch accelerometer to SPI mode
    // dummy read ACC_CHIP_ID
    BMI088_accel_read_reg(ACC_CHIP_ID);
    // enter normal mode
    //BMI088_accel_write_reg(ACC_PWR_CONF, 0x00);
    HAL_Delay(1);
    BMI088_accel_write_reg(ACC_PWR_CTRL, 0x04);
    HAL_Delay(450);

    // configure output data rate
    BMI088_accel_write_reg(ACC_CONF, 0xAB); // 800Hz
    BMI088_gyro_write_reg(GYRO_BANDWIDTH, 0x02); // 1000Hz

    // configure measurement range
    BMI088_accel_write_reg(ACC_RANGE, 0x02); // -12g to 12g
    BMI088_gyro_write_reg(GYRO_RANGE, 0x00); // -2000 deg/s to 2000 deg/s
}

// Chip Select
// LOW to select
void BMI088_cs_accel_h() {
    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
}
void BMI088_cs_accel_l() {
    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
}

void BMI088_cs_gyro_h() {
    HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);
}
void BMI088_cs_gyro_l() {
    HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_RESET);
}

// exchange byte
uint8_t BMI088_exchange_byte(uint8_t txdata) {
    uint8_t rxdata;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rxdata, 1, BMI088_TIMEOUT);
    return rxdata;
}

// write
void BMI088_accel_write_reg(uint8_t reg, uint8_t data) {
    BMI088_cs_accel_l();
    // send reg address
    BMI088_exchange_byte(reg & 0x7F);
    // send data
    BMI088_exchange_byte(data);
    BMI088_cs_accel_h();
}

void BMI088_gyro_write_reg(uint8_t reg, uint8_t data) {
    BMI088_cs_gyro_l();
    // send reg address
    BMI088_exchange_byte(reg & 0x7F);
    // send data
    BMI088_exchange_byte(data);
    BMI088_cs_gyro_h();
}

// read
uint8_t BMI088_accel_read_reg(uint8_t reg) {
    BMI088_cs_accel_l();
    // send reg address
    BMI088_exchange_byte(reg | 0x80);
    // read dummy byte
    BMI088_exchange_byte(DUMMY_BYTE);
    // read data
    uint8_t rxdata = BMI088_exchange_byte(DUMMY_BYTE);
    BMI088_cs_accel_h();
    return rxdata;
}

uint8_t BMI088_gyro_read_reg(uint8_t reg) {
    BMI088_cs_gyro_l();
    // send reg address
    BMI088_exchange_byte(reg | 0x80);
    // read data
    uint8_t rxdata = BMI088_exchange_byte(DUMMY_BYTE);
    BMI088_cs_gyro_h();
    return rxdata;
}

// burst read
void BMI088_accel_burst_read(uint8_t reg, uint8_t rxdata[], uint8_t size) {
    BMI088_cs_accel_l();
    // send reg address
    BMI088_exchange_byte(reg | 0x80);
    // read dummy byte
    BMI088_exchange_byte(DUMMY_BYTE);
    // read data
    for (int i = 0; i < size; i++) {
        rxdata[i] = BMI088_exchange_byte(DUMMY_BYTE);
    }
    BMI088_cs_accel_h();
}

void BMI088_gyro_burst_read(uint8_t reg, uint8_t rxdata[], uint8_t size) {
    BMI088_cs_gyro_l();
    // send reg address
    BMI088_exchange_byte(reg | 0x80);
    // read data
    for (int i = 0; i < size; i++) {
        rxdata[i] = BMI088_exchange_byte(DUMMY_BYTE);
    }
    BMI088_cs_gyro_h();
}

// get data
void BMI088_get_accel_data_int(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z) {
    uint8_t rx_buffer[6];
    BMI088_accel_burst_read(ACC_DATA, rx_buffer, 6);
    *accel_x = rx_buffer[1] << 8 | rx_buffer[0];
    *accel_y = rx_buffer[3] << 8 | rx_buffer[2];
    *accel_z = rx_buffer[5] << 8 | rx_buffer[4];
}

void BMI088_get_gyro_data_int(int16_t* rate_x, int16_t* rate_y, int16_t* rate_z) {
    uint8_t rx_buffer[6];
    BMI088_gyro_burst_read(RATE_DATA, rx_buffer, 6);
    *rate_x = rx_buffer[1] << 8 | rx_buffer[0];
    *rate_y = rx_buffer[3] << 8 | rx_buffer[2];
    *rate_z = rx_buffer[5] << 8 | rx_buffer[4];
}

static float linear_mapping(float input, float i_min, float i_max, float o_min, float o_max) {
    return (input - i_min) * (o_max - o_min) / (i_max - i_min) + o_min;
}

// unit: g
void BMI088_get_accel_data(float* accel_x, float* accel_y, float* accel_z) {
    int16_t x, y, z;
    BMI088_get_accel_data_int(&x, &y, &z);
    *accel_x = linear_mapping(x, -32768, 32767, -12, 12);
    *accel_y = linear_mapping(y, -32768, 32767, -12, 12);
    *accel_z = linear_mapping(z, -32768, 32767, -12, 12);
}

// unit: deg/s
void BMI088_get_gyro_data(float* rate_x, float* rate_y, float* rate_z) {
    int16_t x, y, z;
    BMI088_get_gyro_data_int(&x, &y, &z);
    *rate_x = linear_mapping(x, -32768, 32767, -2000, 2000);
    *rate_y = linear_mapping(y, -32768, 32767, -2000, 2000);
    *rate_z = linear_mapping(z, -32768, 32767, -2000, 2000);
}
