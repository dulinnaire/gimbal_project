//
// Created by 81301 on 2024/10/9.
//

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "gpio.h"
#include "stm32f4xx.h"
#include "usart.h"

#define RC_FRAME_SIZE ((uint8_t)18)
#define RC_CH_VAL_MIN ((uint16_t)364)
#define RC_CH_VAL_MAX ((uint16_t)1684)
#define RC_CH_VAL_MID ((uint16_t)1024)

enum RCSwitchStatus { UP = 1, DOWN = 2, MID = 3 };

class RC {
public:
    RC();

    struct RCChannel {
        float right_row;
        float right_col;
        float left_row;
        float left_col;
    } channel_;

    struct RCSwitch {
        RCSwitchStatus left_switch;
        RCSwitchStatus right_switch;
    } switch_;

    void rc_rx_enable();

    void rc_data_process();

private:
    uint8_t rc_buff[RC_FRAME_SIZE];
    uint8_t rc_data[RC_FRAME_SIZE];

    void rc_transfer();
};

#endif //REMOTE_CONTROL_H
