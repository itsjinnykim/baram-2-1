#ifndef APP_H
#define APP_H

#include <stdint.h>

void App_Init(void);
void App_Loop(void);
extern volatile float imu_test_roll_raw;
extern volatile float imu_test_pitch_raw;
extern volatile float imu_test_yaw_raw;
extern volatile float imu_test_roll_delta;
extern volatile float imu_test_pitch_delta;
extern volatile int8_t imu_test_roll_sign;
extern volatile int8_t imu_test_pitch_sign;
extern volatile uint32_t imu_test_update_count;
extern volatile uint8_t joint_test_id;
extern volatile uint16_t joint_test_home_position;
extern volatile uint16_t joint_test_goal_position;
extern volatile int16_t joint_test_offset_tick;
extern volatile uint8_t joint_test_phase;
extern volatile uint8_t joint_test_sequence_index;
extern volatile uint8_t joint_test_valid;
extern volatile uint32_t joint_test_send_count;
#endif /* APP_H */


