//
// Created by 81301 on 2024/10/26.
//

#ifndef PID_H
#define PID_H
class PID {
public:
    PID();
    PID(float kp, float ki, float kd, float out_max, float i_max);
    float calc(float ref, float fdb);

private:
    float in_range(float val, float min, float max);

    float error_[2]; // 0 for current, 1 for last
    float error_sum_;
    float ref_, fdb_;
    float kp_, ki_, kd_;
    float p_out_, i_out_, d_out_;
    float i_max_;
    float out_max_;
    float output_;
};

#endif //PID_H
