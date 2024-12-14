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

    pid_init_struct._kp = 145;
    pid_init_struct._ki = 0;
    pid_init_struct._kd = 0;
    pid_init_struct._i_max = 0;
    pid_init_struct._out_max = 10000;
    PID pitch_speed_pid(pid_init_struct);

    pid_init_struct._kp = 0.41;
    pid_init_struct._ki = 0.043;
    pid_init_struct._kd = 0;
    pid_init_struct._i_max = 125;
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

void Gimbal::handle() {
    pitch_motor.set_angle(angle_ref.pitch_ref);
    yaw_motor.set_angle(angle_ref.yaw_ref);

    pitch_motor.handle();
    yaw_motor.handle();
}

