#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>

#define DXL_MIN_POS       0
#define DXL_MAX_POS       1023
#define DXL_CENTER_POS    512

extern volatile uint8_t dxl_current_id;
extern volatile uint16_t dxl_goal_position;
extern volatile uint16_t dxl_present_position;
extern volatile uint8_t dxl_last_error;
extern volatile uint8_t dxl_last_comm_ok;
extern volatile uint8_t dxl_last_diag;
extern volatile uint8_t dxl_last_rx0;
extern volatile uint8_t dxl_last_rx1;
extern volatile uint8_t dxl_last_rx2;
extern volatile uint8_t dxl_last_rx3;
extern volatile uint8_t dxl_last_rx4;
extern volatile uint8_t dxl_last_rx5;
extern volatile uint8_t dxl_last_rx6;
extern volatile uint8_t dxl_last_rx7;
extern volatile uint32_t dxl_loop_count;
extern volatile uint32_t dxl_comm_success_count;
extern volatile uint32_t dxl_comm_fail_count;
extern volatile uint8_t dxl_last_success_id;
extern volatile uint16_t dxl_measured_home[18];
extern volatile uint8_t dxl_home_read_ok[18];
extern volatile uint32_t dxl_home_read_fail_count;
extern volatile uint8_t dxl_home_capture_done;

void DXL_Init(void);
void DXL_TorqueEnableAll(void);
void DXL_TorqueDisableAll(void);
void DXL_CaptureHomePosition(void);
void DXL_MoveHomePosition(void);
void DXL_RunReferenceMotionTest(void);
uint8_t DXL_WriteGoalPosition(uint8_t id, uint16_t position);
uint8_t DXL_WriteSafeGoalPosition(uint8_t id, uint16_t position);
uint8_t DXL_SyncWriteSafeGoalPositions(const uint8_t *ids, const uint16_t *positions, uint8_t count);
uint8_t DXL_ReadPresentPosition(uint8_t id, uint16_t *out_value);
uint8_t DXL_IsMovingID(uint8_t id);
uint16_t DXL_ClampPosition(int32_t position);

#endif /* DYNAMIXEL_H */
