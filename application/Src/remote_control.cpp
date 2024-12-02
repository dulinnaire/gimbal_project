#include "remote_control.h"

static float linear_mapping(const float input, float i_min, float i_max, float o_min, float o_max) {
    return (input - i_min) * (o_max - o_min) * 1.0 / (i_max - i_min) + o_min;
}

static float channel_mapping(const float input) {
    return linear_mapping(input, RC_CH_VAL_MIN, RC_CH_VAL_MAX, -1, 1);
}

RC::RC() {
    for (int i = 0; i < RC_FRAME_SIZE; i++) {
        this->rc_buff[i] = 0;
        this->rc_data[i] = 0;
    }
    channel_.left_col = 0;
    channel_.left_row = 0;
    channel_.right_col = 0;
    channel_.right_row = 0;

    switch_.left_switch = DOWN;
    switch_.right_switch = DOWN;
}

void RC::handle() {
    DBUS_handle(this->rc_buff);
}

void RC::rc_transfer() {
    for (int i = 0; i < RC_FRAME_SIZE; i++) {
        rc_data[i] = rc_buff[i];
    }
}

void RC::rc_data_process() {
    this->rc_transfer();
    // 11bits channel
    channel_.right_row = channel_mapping((rc_data[0] | rc_data[1] << 8) & 0x07FF);
    channel_.right_col = channel_mapping((rc_data[1] >> 3 | rc_data[2] << 5) & 0x07FF);
    channel_.left_row =
        channel_mapping((rc_data[2] >> 6 | rc_data[3] << 2 | rc_data[4] << 10) & 0x7FF);
    channel_.left_col = channel_mapping((rc_data[4] >> 1 | rc_data[5] << 7) & 0x7FF);

    // 2bits switch
    switch_.left_switch = (RCSwitchStatus)(rc_data[5] >> 4 & 0x03);
    switch_.right_switch = (RCSwitchStatus)(rc_data[5] >> 6 & 0x03);
}
