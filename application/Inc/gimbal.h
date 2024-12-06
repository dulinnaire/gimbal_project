//
// Created by 81301 on 2024/11/30.
//

#ifndef GIMBAL_H
#define GIMBAL_H

#include "motor.h"
#include "pid.h"

class Gimbal {
public:
    Gimbal();

    void set_pitch_angle(float angle);
    void set_yaw_angle(float angle);

    uint16_t pitch_rx_id();
    uint16_t yaw_rx_id();

    void pitch_data_process(uint8_t data[8]);
    void yaw_data_process(uint8_t data[8]);

    void stop();

    void handle();

private:
    DJIMotor pitch_motor, yaw_motor;
    struct {
        float pitch_ref;
        float yaw_ref;
    } angle_ref;
};

#endif //GIMBAL_H
