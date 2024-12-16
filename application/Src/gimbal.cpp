//
// Created by 81301 on 2024/11/30.
//

#include "gimbal.h"

// Motor Define
#define PITCH_MOTOR_TYPE GM6020
#define YAW_MOTOR_TYPE GM6020

#define PITCH_MOTOR_ID MOTOR_ID_1
#define YAW_MOTOR_ID MOTOR_ID_3

/*
Range:
pitch GM6020 encoder_angle 0x1150-0x1650

pitch上电后的初始ref不能是0

*/

Gimbal::Gimbal() {
    // PID
    PIDInitStruct pid_init_struct;

    pid_init_struct._kp = 185;
    pid_init_struct._ki = 0;
    pid_init_struct._kd = 0;
    pid_init_struct._i_max = 0;
    pid_init_struct._out_max = 10000;
    PID pitch_speed_pid(pid_init_struct);

    pid_init_struct._kp = 0.14; //0.41;
    pid_init_struct._ki = 0.018; //0.043;
    pid_init_struct._kd = 0.6;
    pid_init_struct._i_max = 165;
    pid_init_struct._out_max = 180;
    PID pitch_angle_pid(pid_init_struct);

    pid_init_struct._kp = 150;
    pid_init_struct._ki = 0;
    pid_init_struct._kd = 0;
    pid_init_struct._i_max = 0;
    pid_init_struct._out_max = 10000;
    PID yaw_speed_pid(pid_init_struct);

    pid_init_struct._kp = 0.3;
    pid_init_struct._ki = 0.028;
    pid_init_struct._kd = 0;
    pid_init_struct._i_max = 120;
    pid_init_struct._out_max = 120;
    PID yaw_angle_pid(pid_init_struct);
    // PID 摩擦轮_speed_pid

    // Motors
    // Pitch 6020; Yaw 6020; 摩擦轮两个3508屁股
    pitch_motor = DJIMotor(
        PITCH_MOTOR_TYPE,
        PITCH_MOTOR_ID,
        pitch_speed_pid,
        pitch_angle_pid,
        DOUBLE_ANGLE,
        0x1650,
        FF_ENABLE
    );

    yaw_motor = DJIMotor(
        YAW_MOTOR_TYPE,
        YAW_MOTOR_ID,
        yaw_speed_pid,
        yaw_angle_pid,
        DOUBLE_ANGLE,
        0,
        FF_DISABLE
    );
    // DJIMotor 摩擦轮_left
    // DJIMotor 摩擦轮_right

    angle_ref.pitch_ref = 0x1650;
    angle_ref.yaw_ref = 0;
}

void Gimbal::set_pitch_angle(float angle) {
    angle_ref.pitch_ref = angle;
}

void Gimbal::set_yaw_angle(float angle) {
    angle_ref.yaw_ref = angle;
}

uint16_t Gimbal::pitch_rx_id() {
    return pitch_motor.rx_id();
}

uint16_t Gimbal::yaw_rx_id() {
    return yaw_motor.rx_id();
}

void Gimbal::pitch_data_process(uint8_t data[8]) {
    pitch_motor.data_process(data);
}

void Gimbal::yaw_data_process(uint8_t data[8]) {
    yaw_motor.data_process(data);
}

void Gimbal::stop() {
    pitch_motor.stop();
    yaw_motor.stop();
}

volatile float pitch_angle_mon = 0;
void Gimbal::handle() {
    pitch_motor.set_angle(angle_ref.pitch_ref); // 360
    yaw_motor.set_angle(angle_ref.yaw_ref); // 360

    pitch_angle_mon = pitch_motor.get_total_angle() - (5185 - 13) / 22.75 + 0.4;

    if (true) {
        // pitch_motor.handle(rate_x * 9.55, imu_pitch_angle * 8192.0 / 360);
        // yaw_motor.handle(yaw_motor.get_rotate_speed(), yaw_motor.get_total_angle() * 8192.0 / 360);
        // rate_x 是imu转速，不是电机转速
        pitch_motor.handle(pitch_motor.get_rotate_speed(), imu_pitch_angle * 8192 / 360);
        yaw_motor.handle(yaw_motor.get_rotate_speed(), yaw_motor.get_total_angle() * 8192.0 / 360);

    } else {
        pitch_motor.handle(pitch_motor.get_rotate_speed(), pitch_motor.get_total_angle());
        yaw_motor.handle(yaw_motor.get_rotate_speed(), yaw_motor.get_total_angle());
    }
}

