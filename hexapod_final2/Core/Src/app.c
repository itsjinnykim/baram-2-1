#include "main.h"
#include "app.h"
#include "balance.h"
#include "dynamixel.h"
#include "ebimu.h"
#include "kinematics.h"

#define APP_HOME_CAPTURE_MODE    0
#define APP_IMU_TEST_MODE        0
#define APP_BALANCE_TEST_MODE    0
#define APP_DXL_TEST_MODE        0
#define APP_JOINT_DIRECTION_TEST_MODE 0
#define APP_STARTUP_WAVE_MODE    1
#define APP_SEND_GOAL_MODE       1

#define APP_GOAL_SEND_INTERVAL_MS 20
#define APP_GOAL_SEND_DELTA_TICK  1
#define APP_GOAL_SMOOTH_NUM      3
#define APP_GOAL_SMOOTH_DEN      10
#define APP_JOINT_TEST_DEFAULT_ID 1
#define APP_JOINT_TEST_DELTA_TICK 10
#define APP_JOINT_TEST_INTERVAL_MS 1500
#define APP_WAVE_LEG_FEMUR_ID     1
#define APP_WAVE_LEG_TIBIA_ID     2
#define APP_WAVE_LIFT_TICK        40
#define APP_WAVE_BEND_TICK       340
#define APP_WAVE_NOD_TICK        -20
#define APP_WAVE_STEP_INTERVAL_MS 600
#define APP_WAVE_START_DELAY_MS  2000
#define APP_WAVE_TOGGLE_COUNT     8

volatile float imu_test_roll_raw = 0.0f;
volatile float imu_test_pitch_raw = 0.0f;
volatile float imu_test_yaw_raw = 0.0f;
volatile float imu_test_roll_delta = 0.0f;
volatile float imu_test_pitch_delta = 0.0f;
volatile int8_t imu_test_roll_sign = 0;
volatile int8_t imu_test_pitch_sign = 0;
volatile uint32_t imu_test_update_count = 0;

#if APP_JOINT_DIRECTION_TEST_MODE || APP_STARTUP_WAVE_MODE
static const uint16_t app_home_position[18] = {
    509, 465, 165,
    203, 457, 154,
    200, 448, 162,
    198, 491, 139,
    190, 474, 145,
    199, 484, 128
};
#endif

#if APP_JOINT_DIRECTION_TEST_MODE
static const uint8_t app_joint_test_sequence[] = {
    1, 2,
    4, 5,
    7, 8,
    10, 11,
    13, 14,
    16, 17
};
#endif

volatile uint8_t joint_test_id = APP_JOINT_TEST_DEFAULT_ID;
volatile uint16_t joint_test_home_position = 0;
volatile uint16_t joint_test_goal_position = 0;
volatile int16_t joint_test_offset_tick = 0;
volatile uint8_t joint_test_phase = 0;
volatile uint8_t joint_test_sequence_index = 0;
volatile uint8_t joint_test_valid = 0;
volatile uint32_t joint_test_send_count = 0;

#if APP_STARTUP_WAVE_MODE
static uint8_t app_startup_wave_done = 0;
static uint8_t app_balance_ready = 0;
#endif

#if APP_IMU_TEST_MODE
static void App_UpdateIMUTestDebug(void);
static int8_t App_SignOf(float value);
#endif
#if APP_JOINT_DIRECTION_TEST_MODE
static void App_RunJointDirectionTest(void);
#endif
#if APP_STARTUP_WAVE_MODE
static uint8_t App_RunStartupWave(void);
static void App_SendWaveFemur(int16_t femur_offset);
static void App_SendWaveTibia(int16_t tibia_offset);
#endif

void App_Init(void)
{
    EBIMU_StartReceive();
#if !APP_IMU_TEST_MODE
    DXL_Init();
#endif
    Kinematics_Init();

#if APP_IMU_TEST_MODE
    (void)Balance_CalibrateIMU(100, 10, 5000);
#else
#if APP_HOME_CAPTURE_MODE
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    DXL_CaptureHomePosition();

    while (1)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(500);
    }
#else
#if APP_JOINT_DIRECTION_TEST_MODE
    DXL_TorqueEnableAll();
    DXL_MoveHomePosition();
#elif APP_DXL_TEST_MODE
    DXL_TorqueEnableAll();
#elif APP_STARTUP_WAVE_MODE
    DXL_TorqueEnableAll();
    DXL_MoveHomePosition();
#elif APP_BALANCE_TEST_MODE
    DXL_TorqueDisableAll();
    (void)Balance_CalibrateIMU(100, 10, 5000);
#else
    DXL_TorqueEnableAll();
    DXL_MoveHomePosition();
    (void)Balance_CalibrateIMU(100, 10, 5000);
#endif
#endif
#endif
}

void App_Loop(void)
{
#if APP_SEND_GOAL_MODE && !APP_IMU_TEST_MODE && !APP_DXL_TEST_MODE && !APP_JOINT_DIRECTION_TEST_MODE
    static uint16_t last_sent_position[18] = {
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF
    };
    static uint16_t smoothed_position[18] = {
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF
    };
    static uint32_t last_goal_send_ms = 0;
#endif
#if (!APP_DXL_TEST_MODE && !APP_JOINT_DIRECTION_TEST_MODE) || APP_IMU_TEST_MODE
    Balance_Output balance;
#endif

    EBIMU_ProcessFrame();

#if APP_IMU_TEST_MODE
    (void)Balance_Update(&balance);
    App_UpdateIMUTestDebug();
#elif APP_JOINT_DIRECTION_TEST_MODE
    App_RunJointDirectionTest();
#elif APP_DXL_TEST_MODE
    DXL_RunReferenceMotionTest();
#else
#if APP_STARTUP_WAVE_MODE
    if (app_startup_wave_done == 0)
    {
        if (App_RunStartupWave())
        {
            app_startup_wave_done = 1;
            DXL_MoveHomePosition();
            HAL_Delay(500);
            (void)Balance_CalibrateIMU(100, 10, 5000);
            app_balance_ready = 1;
        }
        return;
    }

    if (app_balance_ready == 0)
    {
        (void)Balance_CalibrateIMU(100, 10, 5000);
        app_balance_ready = 1;
    }
#endif

    if (Balance_Update(&balance))
    {
        Kinematics_UpdateFromBalance(balance.roll_cmd, balance.pitch_cmd);
#if APP_SEND_GOAL_MODE && !APP_IMU_TEST_MODE
        uint32_t now_ms = HAL_GetTick();

        if ((now_ms - last_goal_send_ms) >= APP_GOAL_SEND_INTERVAL_MS)
        {
            uint8_t ids[18];
            uint16_t positions[18];
            uint8_t count = 0;

            for (uint8_t leg = 0; leg < LEG_COUNT; leg++)
            {
                for (uint8_t joint = 0; joint < JOINTS_PER_LEG; joint++)
                {
                    uint8_t id = Kinematics_GetJointID(leg, joint);
                    uint16_t goal = leg_goal_position[leg][joint];
                    uint16_t smooth_goal;
                    uint16_t previous;
                    uint16_t delta;

                    if (id == 0xFF || !DXL_IsMovingID(id))
                    {
                        continue;
                    }

                    if (smoothed_position[id] == 0xFFFF)
                    {
                        smoothed_position[id] = goal;
                    }
                    else
                    {
                        int32_t smooth = (int32_t)smoothed_position[id];
                        int32_t diff = (int32_t)goal - smooth;

                        smooth += (diff * APP_GOAL_SMOOTH_NUM) / APP_GOAL_SMOOTH_DEN;

                        if (diff > 0 && smooth == (int32_t)smoothed_position[id])
                        {
                            smooth++;
                        }
                        else if (diff < 0 && smooth == (int32_t)smoothed_position[id])
                        {
                            smooth--;
                        }

                        smoothed_position[id] = (uint16_t)smooth;
                    }

                    smooth_goal = smoothed_position[id];
                    previous = last_sent_position[id];
                    delta = (smooth_goal > previous) ? (smooth_goal - previous) : (previous - smooth_goal);

                    if (previous != 0xFFFF && delta < APP_GOAL_SEND_DELTA_TICK)
                    {
                        continue;
                    }

                    ids[count] = id;
                    positions[count] = smooth_goal;
                    count++;
                    last_sent_position[id] = smooth_goal;
                }
            }

            if (count > 0)
            {
                (void)DXL_SyncWriteSafeGoalPositions(ids, positions, count);
                last_goal_send_ms = now_ms;
            }
        }
#endif
    }
#endif
}

#if APP_IMU_TEST_MODE
static void App_UpdateIMUTestDebug(void)
{
    imu_test_roll_raw = imu_roll;
    imu_test_pitch_raw = imu_pitch;
    imu_test_yaw_raw = imu_yaw;
    imu_test_roll_delta = imu_roll - balance_base_roll;
    imu_test_pitch_delta = imu_pitch - balance_base_pitch;
    imu_test_roll_sign = App_SignOf(imu_test_roll_delta);
    imu_test_pitch_sign = App_SignOf(imu_test_pitch_delta);
    imu_test_update_count = imu_valid_frame_count;
}

static int8_t App_SignOf(float value)
{
    if (value > 0.2f)
    {
        return 1;
    }

    if (value < -0.2f)
    {
        return -1;
    }

    return 0;
}
#endif

#if APP_STARTUP_WAVE_MODE
static uint8_t App_RunStartupWave(void)
{
    static uint32_t last_step_ms = 0;
    static uint8_t femur_high_phase = 0;
    static uint8_t start_delay_done = 0;
    static uint8_t tibia_lift_done = 0;
    static uint8_t toggle_count = 0;
    uint32_t now_ms = HAL_GetTick();

    if (start_delay_done == 0)
    {
        if (last_step_ms == 0)
        {
            last_step_ms = now_ms;
            return 0;
        }

        if ((now_ms - last_step_ms) < APP_WAVE_START_DELAY_MS)
        {
            return 0;
        }

        start_delay_done = 1;
        last_step_ms = now_ms;
    }

    if (tibia_lift_done == 0)
    {
        App_SendWaveTibia(APP_WAVE_BEND_TICK);
        HAL_Delay(20);
        App_SendWaveFemur(APP_WAVE_LIFT_TICK);
        tibia_lift_done = 1;
        femur_high_phase = 0;
        last_step_ms = now_ms;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        return 0;
    }

    if ((now_ms - last_step_ms) < APP_WAVE_STEP_INTERVAL_MS)
    {
        return 0;
    }

    last_step_ms = now_ms;

    if (toggle_count >= APP_WAVE_TOGGLE_COUNT)
    {
        App_SendWaveFemur(0);
        HAL_Delay(20);
        App_SendWaveTibia(0);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        return 1;
    }

    if (femur_high_phase == 0)
    {
        App_SendWaveFemur(APP_WAVE_LIFT_TICK + APP_WAVE_NOD_TICK);
        femur_high_phase = 1;
    }
    else
    {
        App_SendWaveFemur(APP_WAVE_LIFT_TICK);
        femur_high_phase = 0;
    }

    toggle_count++;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    return 0;
}

static void App_SendWaveFemur(int16_t femur_offset)
{
    uint16_t position;

    position = DXL_ClampPosition((int32_t)app_home_position[APP_WAVE_LEG_FEMUR_ID] + femur_offset);
    (void)DXL_WriteGoalPosition(APP_WAVE_LEG_FEMUR_ID, position);
}

static void App_SendWaveTibia(int16_t tibia_offset)
{
    uint16_t position;

    position = DXL_ClampPosition((int32_t)app_home_position[APP_WAVE_LEG_TIBIA_ID] + tibia_offset);
    (void)DXL_WriteGoalPosition(APP_WAVE_LEG_TIBIA_ID, position);
}
#endif

#if APP_JOINT_DIRECTION_TEST_MODE
static void App_RunJointDirectionTest(void)
{
    static uint32_t last_send_ms = 0;
    static uint8_t return_home_phase = 0;
    uint32_t now_ms = HAL_GetTick();
    uint8_t sequence_count = (uint8_t)(sizeof(app_joint_test_sequence) / sizeof(app_joint_test_sequence[0]));
    uint8_t id;
    int32_t goal;

    if (joint_test_sequence_index >= sequence_count)
    {
        joint_test_sequence_index = 0;
    }

    id = app_joint_test_sequence[joint_test_sequence_index];
    joint_test_id = id;

    if (id >= 18 || !DXL_IsMovingID(id))
    {
        joint_test_valid = 0;
        joint_test_home_position = 0;
        joint_test_goal_position = 0;
        joint_test_offset_tick = 0;
        joint_test_sequence_index++;
        return_home_phase = 0;
        last_send_ms = now_ms;
        return;
    }

    joint_test_valid = 1;
    joint_test_home_position = app_home_position[id];

    if ((now_ms - last_send_ms) < APP_JOINT_TEST_INTERVAL_MS)
    {
        return;
    }

    if (return_home_phase == 0)
    {
        joint_test_offset_tick = APP_JOINT_TEST_DELTA_TICK;
        joint_test_phase = 1;
        return_home_phase = 1;
    }
    else
    {
        joint_test_offset_tick = 0;
        joint_test_phase = 0;
        return_home_phase = 0;
        joint_test_sequence_index++;
        if (joint_test_sequence_index >= sequence_count)
        {
            joint_test_sequence_index = 0;
        }
    }

    goal = (int32_t)joint_test_home_position + (int32_t)joint_test_offset_tick;
    joint_test_goal_position = DXL_ClampPosition(goal);
    (void)DXL_WriteSafeGoalPosition(id, joint_test_goal_position);
    last_send_ms = now_ms;
    joint_test_send_count++;
}
#endif