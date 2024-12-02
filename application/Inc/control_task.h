//
// Created by 81301 on 2024/12/2.
//

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "gimbal.h"
#include "remote_control.h"

extern RC remote_control;
extern Gimbal gimbal;

void control_handle();

#endif //CONTROL_TASK_H
