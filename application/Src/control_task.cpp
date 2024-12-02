//
// Created by 81301 on 2024/12/2.
//
#include "control_task.h"

void control_handle() {
    switch (remote_control.switch_.right_switch) {
        case UP: // 右一 pitch
            break;
        case MID: // 右二 yaw
            break;
        case DOWN: // 右三 急停
            break;
        default:
            break;
    }
}
