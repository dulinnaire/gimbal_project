//
// Created by 81301 on 2024/11/30.
//
#include "dbus.h"

void DBUS_handle(uint8_t data_buffer[]) {
    HAL_UART_Receive_DMA(&huart3, data_buffer, FRAME_SIZE);
}
