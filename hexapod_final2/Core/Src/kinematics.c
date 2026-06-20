#include "kinematics.h"
#include "dynamixel.h"

#include <math.h>

#define PI_F                    3.1415926535f
#define RAD_TO_DEG              (180.0f / PI_F)

#define HEXAPOD_COXA_LEN_MM     50.0f
#define HEXAPOD_FEMUR_LEN_MM    70.0f
#define HEXAPOD_TIBIA_LEN_MM    150.0f

#define BALANCE_ROLL_Z_GAIN     1.0f
#define BALANCE_PITCH_Z_GAIN    1.0f
#define JOINT_MAX_DELTA_TICK    30

static const Vec3f leg_home_foot[LEG_COUNT] = {
    { 260.0f,    0.0f, -160.0f },
    { 130.0f,  225.2f, -160.0f },
    {-130.0f,  225.2f, -160.0f },
    {-260.0f,    0.0f, -160.0f },
    {-130.0f, -225.2f, -160.0f },
    { 130.0f, -225.2f, -160.0f }
};

static const Vec3f leg_base_position[LEG_COUNT] = {
    { 140.0f,    0.0f, 0.0f },
    {  70.0f,  121.2f, 0.0f },
    { -70.0f,  121.2f, 0.0f },
    {-140.0f,    0.0f, 0.0f },
    { -70.0f, -121.2f, 0.0f },
    {  70.0f, -121.2f, 0.0f }
};

static const uint8_t joint_dxl_id[LEG_COUNT][JOINTS_PER_LEG] = {
    {  0,  1,  2 },
    {  3,  4,  5 },
    {  6,  7,  8 },
    {  9, 10, 11 },
    { 12, 13, 14 },
    { 15, 16, 17 }
};

static const int8_t joint_direction[LEG_COUNT][JOINTS_PER_LEG] = {
    {  1,  1,  1 },
    {  1,  1,  1 },
    {  1,  1,  1 },
    { -1, -1, -1 },
    { -1, -1, -1 },
    { -1, -1, -1 }
};

static const uint16_t joint_center_position[LEG_COUNT][JOINTS_PER_LEG] = {
    { 444, 466, 166 },
    { 183, 456, 155 },
    { 203, 447, 166 },
    { 201, 490, 138 },
    { 190, 465, 145 },
    { 200, 472, 137 }
};

volatile float leg_target_x[LEG_COUNT] = {0.0f};
volatile float leg_target_y[LEG_COUNT] = {0.0f};
volatile float leg_target_z[LEG_COUNT] = {0.0f};
volatile float leg_home_angle_deg[LEG_COUNT][JOINTS_PER_LEG] = {{0.0f}};
volatile float leg_joint_angle_deg[LEG_COUNT][JOINTS_PER_LEG] = {{0.0f}};
volatile uint16_t leg_goal_position[LEG_COUNT][JOINTS_PER_LEG] = {{0}};
volatile uint8_t leg_joint_id[LEG_COUNT][JOINTS_PER_LEG] = {{0}};
volatile uint8_t leg_ik_ok[LEG_COUNT] = {0};
volatile uint32_t kinematics_update_count = 0;

static float clamp_float(float value, float min_value, float max_value);

void Kinematics_Init(void)
{
    for (uint8_t leg = 0; leg < LEG_COUNT; leg++)
    {
        LegJointAngles home_angles;

        leg_target_x[leg] = leg_home_foot[leg].x;
        leg_target_y[leg] = leg_home_foot[leg].y;
        leg_target_z[leg] = leg_home_foot[leg].z;

        if (Kinematics_LegIK(leg, &leg_home_foot[leg], &home_angles))
        {
            leg_home_angle_deg[leg][JOINT_COXA] = home_angles.coxa_deg;
            leg_home_angle_deg[leg][JOINT_FEMUR] = home_angles.femur_deg;
            leg_home_angle_deg[leg][JOINT_TIBIA] = home_angles.tibia_deg;
            leg_ik_ok[leg] = 1;
        }
        else
        {
            leg_home_angle_deg[leg][JOINT_COXA] = 0.0f;
            leg_home_angle_deg[leg][JOINT_FEMUR] = 0.0f;
            leg_home_angle_deg[leg][JOINT_TIBIA] = 0.0f;
            leg_ik_ok[leg] = 0;
        }

        for (uint8_t joint = 0; joint < JOINTS_PER_LEG; joint++)
        {
            leg_joint_id[leg][joint] = joint_dxl_id[leg][joint];
            leg_joint_angle_deg[leg][joint] = leg_home_angle_deg[leg][joint];
            leg_goal_position[leg][joint] = joint_center_position[leg][joint];
        }
    }

    kinematics_update_count = 0;
}

void Kinematics_UpdateFromBalance(float roll_cmd, float pitch_cmd)
{
    for (uint8_t leg = 0; leg < LEG_COUNT; leg++)
    {
        Vec3f target = leg_home_foot[leg];
        LegJointAngles angles;
        float side_sign = (target.y >= 0.0f) ? 1.0f : -1.0f;
        float front_sign = (target.x >= 0.0f) ? 1.0f : -1.0f;

        target.z += (roll_cmd * side_sign * BALANCE_ROLL_Z_GAIN);
        target.z += (pitch_cmd * front_sign * BALANCE_PITCH_Z_GAIN);

        leg_target_x[leg] = target.x;
        leg_target_y[leg] = target.y;
        leg_target_z[leg] = target.z;

        leg_ik_ok[leg] = Kinematics_LegIK(leg, &target, &angles);
        if (leg_ik_ok[leg])
        {
            leg_joint_angle_deg[leg][JOINT_COXA] = angles.coxa_deg;
            leg_joint_angle_deg[leg][JOINT_FEMUR] = angles.femur_deg;
            leg_joint_angle_deg[leg][JOINT_TIBIA] = angles.tibia_deg;

            leg_goal_position[leg][JOINT_COXA] =
                Kinematics_AngleToDynamixel(leg, JOINT_COXA, angles.coxa_deg);
            leg_goal_position[leg][JOINT_FEMUR] =
                Kinematics_AngleToDynamixel(leg, JOINT_FEMUR, angles.femur_deg);
            leg_goal_position[leg][JOINT_TIBIA] =
                Kinematics_AngleToDynamixel(leg, JOINT_TIBIA, angles.tibia_deg);
        }
    }

    kinematics_update_count++;
}

uint8_t Kinematics_LegIK(uint8_t leg, const Vec3f *target, LegJointAngles *out)
{
    float coxa_rad;
    float local_x;
    float local_y;
    float local_z;
    float horizontal;
    float planar_x;
    float planar_z;
    float distance_sq;
    float distance;
    float femur_cos;
    float tibia_cos;
    float base_angle;
    float femur_offset;

    if (leg >= LEG_COUNT || target == 0 || out == 0)
    {
        return 0;
    }

    local_x = target->x - leg_base_position[leg].x;
    local_y = target->y - leg_base_position[leg].y;
    local_z = target->z - leg_base_position[leg].z;

    coxa_rad = atan2f(local_y, local_x);
    horizontal = sqrtf((local_x * local_x) + (local_y * local_y));
    planar_x = horizontal - HEXAPOD_COXA_LEN_MM;
    planar_z = -local_z;

    distance_sq = (planar_x * planar_x) + (planar_z * planar_z);
    distance = sqrtf(distance_sq);
    if (distance <= 0.001f)
    {
        return 0;
    }

    if (distance > (HEXAPOD_FEMUR_LEN_MM + HEXAPOD_TIBIA_LEN_MM))
    {
        return 0;
    }

    if (distance < fabsf(HEXAPOD_FEMUR_LEN_MM - HEXAPOD_TIBIA_LEN_MM))
    {
        return 0;
    }

    femur_cos = (distance_sq + (HEXAPOD_FEMUR_LEN_MM * HEXAPOD_FEMUR_LEN_MM) -
                 (HEXAPOD_TIBIA_LEN_MM * HEXAPOD_TIBIA_LEN_MM)) /
                (2.0f * distance * HEXAPOD_FEMUR_LEN_MM);
    tibia_cos = ((HEXAPOD_FEMUR_LEN_MM * HEXAPOD_FEMUR_LEN_MM) +
                 (HEXAPOD_TIBIA_LEN_MM * HEXAPOD_TIBIA_LEN_MM) - distance_sq) /
                (2.0f * HEXAPOD_FEMUR_LEN_MM * HEXAPOD_TIBIA_LEN_MM);

    femur_cos = clamp_float(femur_cos, -1.0f, 1.0f);
    tibia_cos = clamp_float(tibia_cos, -1.0f, 1.0f);

    base_angle = atan2f(planar_z, planar_x);
    femur_offset = acosf(femur_cos);

    out->coxa_deg = coxa_rad * RAD_TO_DEG;
    out->femur_deg = (base_angle + femur_offset) * RAD_TO_DEG;
    out->tibia_deg = (PI_F - acosf(tibia_cos)) * RAD_TO_DEG;

    return 1;
}

uint16_t Kinematics_AngleToDynamixel(uint8_t leg, uint8_t joint, float angle_deg)
{
    int32_t position;
    int32_t min_position;
    int32_t max_position;
    float delta_deg;
    float ticks;

    if (leg >= LEG_COUNT || joint >= JOINTS_PER_LEG)
    {
        return DXL_CENTER_POS;
    }

    delta_deg = angle_deg - leg_home_angle_deg[leg][joint];
    ticks = delta_deg * (1023.0f / 300.0f);
    position = (int32_t)joint_center_position[leg][joint] +
               ((int32_t)joint_direction[leg][joint] * (int32_t)ticks);
    min_position = (int32_t)joint_center_position[leg][joint] - JOINT_MAX_DELTA_TICK;
    max_position = (int32_t)joint_center_position[leg][joint] + JOINT_MAX_DELTA_TICK;

    if (position < min_position)
    {
        position = min_position;
    }

    if (position > max_position)
    {
        position = max_position;
    }

    return DXL_ClampPosition(position);
}

uint8_t Kinematics_GetJointID(uint8_t leg, uint8_t joint)
{
    if (leg >= LEG_COUNT || joint >= JOINTS_PER_LEG)
    {
        return 0xFF;
    }

    return joint_dxl_id[leg][joint];
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}
