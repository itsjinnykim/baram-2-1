#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>

typedef struct {
    float roll_error;
    float pitch_error;
    float roll_cmd;
    float pitch_cmd;
} Balance_Output;

extern volatile float balance_base_roll;
extern volatile float balance_base_pitch;
extern volatile float balance_base_yaw;
extern volatile float balance_roll_error;
extern volatile float balance_pitch_error;
extern volatile float balance_roll_rate;
extern volatile float balance_pitch_rate;
extern volatile float balance_roll_cmd;
extern volatile float balance_pitch_cmd;
extern volatile uint8_t balance_calibrated;
extern volatile uint32_t balance_update_count;

uint8_t Balance_CalibrateIMU(uint16_t sample_count, uint32_t sample_interval_ms, uint32_t timeout_ms);
uint8_t Balance_Update(Balance_Output *out);

#endif /* BALANCE_H */
