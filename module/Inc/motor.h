//
// Created by 81301 on 2024/11/13.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "can.h"
#include "pid.h"
#include "stm32f4xx.h"

enum DJIMotorType { MOTOR_TYPE_NONE = 0, M3508, M2006, GM6020 };
enum MotorPIDType { PID_TYPE_NONE = 0, SINGLE_SPEED, DOUBLE_ANGLE };
enum MotorID {
    MOTOR_ID_NONE = -1,
    MOTOR_ID_1 = 0,
    MOTOR_ID_2,
    MOTOR_ID_3,
    MOTOR_ID_4,
    MOTOR_ID_5,
    MOTOR_ID_6,
    MOTOR_ID_7,
    MOTOR_ID_8
};

class DJIMotor {
public:
    DJIMotor();
    DJIMotor(
        DJIMotorType DJI_motor_type,
        MotorID id,
        PID speed_pid,
        PID angle_pid,
        MotorPIDType motor_pid_type,
        float ratio
    );

    void data_process(uint8_t data[8]);

    void set_speed(uint16_t speed);

    void set_angle(float angle);

    void handle();

private:
    DJIMotorType motor_type;
    MotorID motor_id;

    MotorPIDType pid_type;

    PID motor_speed_pid; // 速度环PID

    PID motor_angle_pid; // 角度环PID

    struct {
        uint16_t speed_ref;
        uint16_t angle_ref;
    } pid_ref;

    float reduction_ratio; // 电机减速比

    uint16_t encoder_angle; // 当前电机编码器角度 range:[0,8191]
    uint16_t last_encoder_angle; // 上次电机编码器角度 range:[0,8191]
    int16_t delta_encoder_angle; // 编码器端新转动的角度
    int32_t total_encoder_angle; // 编码器转过的总角度
    int32_t round_cnt; // 转过的总圈数

    float rotate_speed; // rpm 反馈转子转速
    float current; // A 反馈转矩电流
    float temp; // °C 反馈电机温度

    CAN_TxHeaderTypeDef tx_header;
};



#endif //MOTOR_H
