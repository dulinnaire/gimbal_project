//
// Created by 81301 on 2024/11/13.
//

#include "can.h"
#include "motor.h"
#include "remote_control.h"
#include "stm32f4xx.h"
#include "tim.h"

int cnt = 0;
RC remote_control;
float angle = 0;

PID motor2006_speed_pid(26, 0, 0, 2500, 1000);
PID motor2006_angle_pid(2.5, 0.11, 0, 100, 100);
DJIMotor motor2006(M2006, MOTOR_ID_1, motor2006_speed_pid, motor2006_angle_pid, DOUBLE_ANGLE, 36);

// The main loop of the program
void loop() {
    // cnt++;
    remote_control.rc_rx_enable();
    motor2006.set_angle(angle);
    motor2006.handle();
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
CAN_RxHeaderTypeDef can_rx_header;
// extern std::map<uint16_t, DJIMotor*> motor_list;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    static uint8_t can_rx_buff[8];
    if (hcan == &hcan1) {
        // 接收到报文，进行处理
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_buff) == HAL_OK) {
            motor2006.data_process(can_rx_buff);
        }
    }
}
