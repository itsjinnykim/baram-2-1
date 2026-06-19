#include "main.h"
#include "app.h"
#include "balance.h"
#include "dynamixel.h"
#include "ebimu.h"

#define APP_HOME_CAPTURE_MODE    0
#define APP_BALANCE_TEST_MODE    1

void App_Init(void)
{
    EBIMU_StartReceive();
    DXL_Init();

#if APP_HOME_CAPTURE_MODE
    DXL_TorqueDisableAll();
    HAL_Delay(500);
    DXL_CaptureHomePosition();

    while (1)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(500);
    }
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
}

void App_Loop(void)
{
    Balance_Output balance;

    EBIMU_ProcessFrame();
    (void)Balance_Update(&balance);
}
