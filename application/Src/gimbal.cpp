//
// Created by 81301 on 2024/11/30.
//

#include "gimbal.h"

// Motor Define
#define PITCH_MOTOR_TYPE GM6020
#define YAW_MOTOR_TYPE GM6020

#define PITCH_MOTOR_ID MOTOR_ID_1
#define YAW_MOTOR_ID MOTOR_ID_2

Gimbal::Gimbal() {
    // PID
    PIDInitStruct pid_init_struct;

    pid_init_struct._kp = 0;
    pid_init_struct._ki = 0;
    pid_init_struct._kd = 0;
    pid_init_struct._i_max = 0;
    pid_init_struct._out_max = 0;
    PID pitch_speed_pid(pid_init_struct);

    pid_init_struct._kp = 0;
    pid_init_struct._ki = 0;
    pid_init_struct._kd = 0;
    pid_init_struct._i_max = 0;
    pid_init_struct._out_max = 0;
    PID pitch_angle_pid(pid_init_struct);

    pid_init_struct._kp = 0;
    pid_init_struct._ki = 0;
    pid_init_struct._kd = 0;
    pid_init_struct._i_max = 0;
    pid_init_struct._out_max = 0;
    PID yaw_speed_pid(pid_init_struct);

    pid_init_struct._kp = 0;
    pid_init_struct._ki = 0;
    pid_init_struct._kd = 0;
    pid_init_struct._i_max = 0;
    pid_init_struct._out_max = 0;
    PID yaw_angle_pid(pid_init_struct);
    // PID 摩擦轮_speed_pid

    // Motors
    // Pitch 6020; Yaw 6020; 摩擦轮两个3508屁股
    pitch_motor =
        DJIMotor(PITCH_MOTOR_TYPE, PITCH_MOTOR_ID, pitch_speed_pid, pitch_angle_pid, DOUBLE_ANGLE);

    yaw_motor = DJIMotor(YAW_MOTOR_TYPE, YAW_MOTOR_ID, yaw_speed_pid, yaw_angle_pid, DOUBLE_ANGLE);
    // DJIMotor 摩擦轮_left
    // DJIMotor 摩擦轮_right

    angle_ref.pitch_ref = 0;
    angle_ref.yaw_ref = 0;
}

void Gimbal::set_pitch_angle(float angle) {
    angle_ref.pitch_ref = angle;
}

void Gimbal::set_yaw_angle(float angle) {
    angle_ref.yaw_ref = angle;
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

