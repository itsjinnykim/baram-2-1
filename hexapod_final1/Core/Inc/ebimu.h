#ifndef EBIMU_H
#define EBIMU_H

#include <stdint.h>

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EBIMU_Data;

extern volatile float imu_roll;
extern volatile float imu_pitch;
extern volatile float imu_yaw;
extern volatile uint8_t imu_frame_ready;
extern volatile uint8_t imu_parse_ok;
extern volatile uint32_t imu_rx_count;
extern volatile uint32_t imu_frame_count;
extern volatile uint32_t imu_parse_fail_count;
extern volatile uint32_t imu_last_update_ms;

void EBIMU_StartReceive(void);
void EBIMU_ProcessFrame(void);

#endif /* EBIMU_H */
