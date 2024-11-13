//
// Created by 81301 on 2024/11/13.
//

#include "remote_control.h"
#include "stm32f4xx.h"
#include "tim.h"

int cnt = 0;
RC remote_control;

// The main loop of the program
void loop() {
    // cnt++;
    remote_control.rc_rx_enable();
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
