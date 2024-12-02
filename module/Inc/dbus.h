//
// Created by 81301 on 2024/11/30.
//

#ifndef DBUS_H
#define DBUS_H

#include "stm32f4xx.h"
#include "usart.h"

#define FRAME_SIZE ((uint8_t)18)

void DBUS_handle(uint8_t data_buffer[]);

#endif //DBUS_H
