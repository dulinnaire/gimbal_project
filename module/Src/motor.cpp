//
// Created by 81301 on 2024/11/13.
//

#include "motor.h"

static float linear_mapping(float in, float in_min, float in_max, float out_min, float out_max) {
    return (in - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}

DJIMotor::DJIMotor() {
    motor_status = STOP;

    motor_type = MOTOR_TYPE_NONE;
    motor_id = MOTOR_ID_NONE;

    pid_type = PID_TYPE_NONE;

    reduction_ratio = 1; // 电机减速比

    motor_speed_pid = PID();
    motor_angle_pid = PID();

    pid_ref.angle_ref = 0;
    pid_ref.speed_ref = 0;

    encoder_angle = 0; // 当前电机编码器角度 range:[0,8191]
    last_encoder_angle = 0; // 上次电机编码器角度 range:[0,8191]
    delta_encoder_angle = 0; // 编码器端新转动的角度
    total_encoder_angle = 0; // 编码器转过的总角度
    round_cnt = 0; // 转过的总圈数

    rotate_speed = 0; // rpm 反馈转子转速
    current = 0; // A 反馈转矩电流
    temp = 0; // °C 反馈电机温度
}

DJIMotor::DJIMotor(
    DJIMotorType DJI_motor_type,
    MotorID id,
    PID speed_pid,
    PID angle_pid,
    MotorPIDType motor_pid_type,
    float init_angle,
    FeedforwardStatus ff_status
) {
    motor_status = STOP;

    feedforward_status = ff_status;

    motor_type = DJI_motor_type;
    motor_id = id;

    pid_type = motor_pid_type;

    switch (motor_type) {
        case M2006:
            reduction_ratio = 36;
            break;
        case M3508:
            reduction_ratio = 3591 / 187;
            break;
        case GM6020:
            reduction_ratio = 1;
            break;
        default:
            reduction_ratio = 0;
            break;
    }

    motor_speed_pid = speed_pid;
    motor_angle_pid = angle_pid;

    pid_ref.angle_ref = 0;
    pid_ref.speed_ref = 0;

    encoder_angle = init_angle; // 当前电机编码器角度 range:[0,8191]
    last_encoder_angle = 0; // 上次电机编码器角度 range:[0,8191]
    delta_encoder_angle = 0; // 编码器端新转动的角度
    total_encoder_angle = 0; // 编码器转过的总角度
    round_cnt = 0; // 转过的总圈数

    rotate_speed = 0; // rpm 反馈转子转速
    current = 0; // A 反馈转矩电流
    temp = 0; // °C 反馈电机温度

    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    switch (motor_type) {
        case M3508:
            if (motor_id <= MOTOR_ID_4) {
                tx_header.StdId = 0x0200;
            } else {
                tx_header.StdId = 0x01FF;
            }
            rx_message_id = 0x200 + motor_id;
            break;
        case M2006:
            if (motor_id <= MOTOR_ID_4) {
                tx_header.StdId = 0x0200;
            } else {
                tx_header.StdId = 0x01FF;
            }
            rx_message_id = 0x200 + motor_id;
            break;
        case GM6020:
            if (motor_id <= MOTOR_ID_4) {
                tx_header.StdId = 0x01FF;
            } else {
                tx_header.StdId = 0x02FF;
            }
            rx_message_id = 0x204 + motor_id;
            break;
        default:
            break;
    }
}

uint16_t DJIMotor::rx_id() const {
    return rx_message_id;
}

float DJIMotor::get_rotate_speed() const {
    return rotate_speed;
}

float DJIMotor::get_total_angle() const {
    return total_encoder_angle / 8192.0 * 360;
}

void DJIMotor::data_process(uint8_t data[8]) {
    last_encoder_angle = encoder_angle;
    encoder_angle = (uint16_t)((data[0] << 8) | data[1]);
    delta_encoder_angle = encoder_angle - last_encoder_angle;

    if (delta_encoder_angle > 4096) {
        round_cnt--;
    } else if (delta_encoder_angle < -4096) {
        round_cnt++;
    }
    total_encoder_angle = 8192 * round_cnt + encoder_angle;

    rotate_speed = (int16_t)((data[2] << 8) | data[3]);
    current = linear_mapping((int16_t)((data[4] << 8) | data[5]), -16384, 16384, -3, 3);
    temp = data[6];
}

// speed: rpm
void DJIMotor::set_speed(uint16_t speed) {
    motor_status = RUNNING;
    pid_ref.speed_ref = pid_type == SINGLE_SPEED ? speed : 0;
}

// angle: degree
void DJIMotor::set_angle(float angle) {
    motor_status = RUNNING;
    pid_ref.angle_ref = pid_type == DOUBLE_ANGLE ? angle / 360 * 8192 * reduction_ratio : 0; // 8192
}

void DJIMotor::stop() {
    motor_status = STOP;
}

/*
计算PID并发送CAN报文
*/
uint8_t can1_tx_data[8] = { 0 };

void DJIMotor::handle(float speed_feedback, float angle_feedback) {
    uint32_t tx_mailbox;
    int16_t feedback_component = 0, feedforward_component = 0;
    int16_t control_value = 0;

    // 计算PID
    switch (pid_type) {
        case SINGLE_SPEED:
            // rpm rpm
            feedback_component = motor_speed_pid.calculate(pid_ref.speed_ref, speed_feedback);
            break;
        case DOUBLE_ANGLE:
            // 8192 8192 rpm
            feedback_component = motor_speed_pid.calculate(
                motor_angle_pid.calculate(pid_ref.angle_ref, angle_feedback),
                speed_feedback
            );
            break;
        default:
            feedback_component = 0;
            break;
    }

    // 计算前馈
    if (feedforward_status == FF_ENABLE) {
        // feedforward_component = linear_mapping(-1.88 * pid_ref.angle_ref * 0.0001 + 0.894, -3, 3, -16384, 16384);
        feedforward_component = linear_mapping(
            -1.88 * (pid_ref.angle_ref * 1 + 5185) * 0.0001 + 0.894,
            -3,
            3,
            -16384,
            16384
        );

    } else {
        feedforward_component = 0;
    }

    control_value = feedback_component + feedforward_component;

    if (motor_status == STOP) {
        control_value = 0;
    }

    can1_tx_data[2 * ((motor_id - 1) % 4)] = control_value >> 8; // 高八位
    can1_tx_data[2 * ((motor_id - 1) % 4) + 1] = control_value & 0xFF; // 低八位
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, can1_tx_data, &tx_mailbox);
}
