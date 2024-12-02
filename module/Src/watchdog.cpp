//
// Created by 81301 on 2024/12/2.
//

#include "watchdog.h"

void feed_watchdog() {
    HAL_IWDG_Refresh(&hiwdg);
}
