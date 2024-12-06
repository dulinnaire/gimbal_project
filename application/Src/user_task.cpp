//
// Created by 81301 on 2024/11/13.
//

#include "can.h"
#include "control_task.h"
#include "gimbal.h"
#include "remote_control.h"
#include "stm32f4xx.h"
#include "tim.h"
#include "watchdog.h"

// TODO: user_task中不应该出现HAL库

Gimbal gimbal;
RC remote_control;

float pitch_angle = 5200; // [0:8191]
float yaw_angle = 0; // [0:360]
volatile float yaw_angle_mon = 0;

float linear_mapping(const float input, float i_min, float i_max, float o_min, float o_max) {
    return (input - i_min) * (o_max - o_min) * 1.0 / (i_max - i_min) + o_min;
}

// The main loop of the program
void loop() {
    remote_control.handle();

    if (pitch_angle > 0x1650) {
        pitch_angle = 0x1650;
    } else if (pitch_angle < 0x1150) {
        pitch_angle = 0x1150;
    }

    gimbal.set_pitch_angle(pitch_angle / 8192 * 360);
    gimbal.set_yaw_angle(yaw_angle);
    yaw_angle_mon = yaw_angle / 360 * 8192;

    // control_handle();
    gimbal.handle();

    feed_watchdog();
}

// execute loop() per 1ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim10) {
        loop();
    }
}

// receive remote control message
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart3) {
        remote_control.rc_data_process();
    }
}

// receive motor message
uint8_t can_rx_buff[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    static CAN_RxHeaderTypeDef can_rx_header;
    if (hcan == &hcan1) {
        // 接收到电机报文，进行处理
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_buff) == HAL_OK) {
            // motor3508.data_process(can_rx_buff);
            // 根据电机反馈报文id获取不同电机反馈
            if (can_rx_header.StdId == gimbal.pitch_rx_id()) {
                gimbal.pitch_data_process(can_rx_buff);
            } else if (can_rx_header.StdId == gimbal.yaw_rx_id()) {
                gimbal.yaw_data_process(can_rx_buff);
            }
        }
    }
}
