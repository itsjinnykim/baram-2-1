#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

#define LEG_COUNT          6
#define JOINTS_PER_LEG     3

typedef enum {
    JOINT_COXA = 0,
    JOINT_FEMUR = 1,
    JOINT_TIBIA = 2
} JointIndex;

typedef struct {
    float x;
    float y;
    float z;
} Vec3f;

typedef struct {
    float coxa_deg;
    float femur_deg;
    float tibia_deg;
} LegJointAngles;

extern volatile float leg_target_x[LEG_COUNT];
extern volatile float leg_target_y[LEG_COUNT];
extern volatile float leg_target_z[LEG_COUNT];
extern volatile float leg_home_angle_deg[LEG_COUNT][JOINTS_PER_LEG];
extern volatile float leg_joint_angle_deg[LEG_COUNT][JOINTS_PER_LEG];
extern volatile uint16_t leg_goal_position[LEG_COUNT][JOINTS_PER_LEG];
extern volatile uint8_t leg_joint_id[LEG_COUNT][JOINTS_PER_LEG];
extern volatile uint8_t leg_ik_ok[LEG_COUNT];
extern volatile float leg_roll_z_offset[LEG_COUNT];
extern volatile float leg_pitch_z_offset[LEG_COUNT];
extern volatile float leg_balance_z_offset[LEG_COUNT];
extern volatile uint32_t kinematics_update_count;

void Kinematics_Init(void);
void Kinematics_UpdateFromBalance(float roll_cmd, float pitch_cmd);
uint8_t Kinematics_LegIK(uint8_t leg, const Vec3f *target, LegJointAngles *out);
uint16_t Kinematics_AngleToDynamixel(uint8_t leg, uint8_t joint, float angle_deg);
uint8_t Kinematics_GetJointID(uint8_t leg, uint8_t joint);

#endif /* KINEMATICS_H */
