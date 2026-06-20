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
#define APP_SEND_GOAL_MODE       1
#define APP_GOAL_SEND_INTERVAL_MS 20
#define APP_GOAL_SEND_DELTA_TICK  2

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
#if APP_DXL_TEST_MODE
    DXL_TorqueEnableAll();
#else
#if APP_BALANCE_TEST_MODE
    DXL_TorqueDisableAll();
    (void)Balance_CalibrateIMU(100, 10, 5000);
#else
    DXL_TorqueEnableAll();
    DXL_MoveHomePosition();
    (void)Balance_CalibrateIMU(100, 10, 5000);
#endif
#endif
#endif
#endif
}

void App_Loop(void)
{
#if APP_SEND_GOAL_MODE && !APP_DXL_TEST_MODE
    static uint16_t last_sent_position[18] = {
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF
    };
    static uint32_t last_goal_send_ms = 0;
#endif
#if !APP_DXL_TEST_MODE
    Balance_Output balance;
#endif

    EBIMU_ProcessFrame();
#if APP_DXL_TEST_MODE
    DXL_RunReferenceMotionTest();
#else
    if (Balance_Update(&balance))
    {
        Kinematics_UpdateFromBalance(balance.roll_cmd, balance.pitch_cmd);
#if APP_SEND_GOAL_MODE
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
                    uint16_t previous;
                    uint16_t delta;

                    if (id == 0xFF || !DXL_IsMovingID(id))
                    {
                        continue;
                    }

                    previous = last_sent_position[id];
                    delta = (goal > previous) ? (goal - previous) : (previous - goal);

                    if (previous != 0xFFFF && delta < APP_GOAL_SEND_DELTA_TICK)
                    {
                        continue;
                    }

                    ids[count] = id;
                    positions[count] = goal;
                    count++;
                    last_sent_position[id] = goal;
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
