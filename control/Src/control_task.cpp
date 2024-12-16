//
// Created by 81301 on 2024/12/2.
//
#include "control_task.h"

extern float pitch_angle;

void control_handle() {
    switch (remote_control.switch_.right_switch) {
        case UP: // 右一 pitch
            gimbal.set_pitch_angle(remote_control.channel_.left_col * -30);
            gimbal.set_yaw_angle(remote_control.channel_.left_row * -40);
            break;
        case MID: // 右二 yaw
            gimbal.stop();
            break;
        case DOWN: // 右三 急停
            gimbal.stop();
            break;
        default:
            gimbal.stop();
            break;
    }
}
