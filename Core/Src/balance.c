#include "main.h"
#include "balance.h"
#include "ebimu.h"

#define BALANCE_KP_ROLL     1.0f
#define BALANCE_KD_ROLL     0.05f
#define BALANCE_KP_PITCH    1.0f
#define BALANCE_KD_PITCH    0.05f

volatile float balance_base_roll = 0.0f;
volatile float balance_base_pitch = 0.0f;
volatile float balance_base_yaw = 0.0f;
volatile float balance_roll_error = 0.0f;
volatile float balance_pitch_error = 0.0f;
volatile float balance_roll_rate = 0.0f;
volatile float balance_pitch_rate = 0.0f;
volatile float balance_roll_cmd = 0.0f;
volatile float balance_pitch_cmd = 0.0f;
volatile uint8_t balance_calibrated = 0;
volatile uint32_t balance_update_count = 0;

static float prev_roll_error = 0.0f;
static float prev_pitch_error = 0.0f;
static uint32_t prev_update_ms = 0;

uint8_t Balance_CalibrateIMU(uint16_t sample_count, uint32_t sample_interval_ms, uint32_t timeout_ms)
{
    float roll_sum = 0.0f;
    float pitch_sum = 0.0f;
    float yaw_sum = 0.0f;
    uint16_t samples = 0;
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_sample_ms = 0;

    balance_calibrated = 0;

    while (samples < sample_count)
    {
        EBIMU_ProcessFrame();

        if ((HAL_GetTick() - start_ms) > timeout_ms)
        {
            return 0;
        }

        if (!imu_parse_ok)
        {
            continue;
        }

        if ((HAL_GetTick() - last_sample_ms) < sample_interval_ms)
        {
            continue;
        }

        last_sample_ms = HAL_GetTick();
        roll_sum += imu_roll;
        pitch_sum += imu_pitch;
        yaw_sum += imu_yaw;
        samples++;
    }

    balance_base_roll = roll_sum / (float)samples;
    balance_base_pitch = pitch_sum / (float)samples;
    balance_base_yaw = yaw_sum / (float)samples;

    prev_roll_error = 0.0f;
    prev_pitch_error = 0.0f;
    prev_update_ms = HAL_GetTick();
    balance_roll_error = 0.0f;
    balance_pitch_error = 0.0f;
    balance_roll_rate = 0.0f;
    balance_pitch_rate = 0.0f;
    balance_roll_cmd = 0.0f;
    balance_pitch_cmd = 0.0f;
    balance_calibrated = 1;

    return 1;
}

uint8_t Balance_Update(Balance_Output *out)
{
    uint32_t now_ms;
    float dt;

    if (!balance_calibrated || !imu_parse_ok)
    {
        return 0;
    }

    now_ms = HAL_GetTick();
    dt = (float)(now_ms - prev_update_ms) * 0.001f;
    if (dt <= 0.0f)
    {
        dt = 0.001f;
    }

    balance_roll_error = imu_roll - balance_base_roll;
    balance_pitch_error = imu_pitch - balance_base_pitch;
    balance_roll_rate = (balance_roll_error - prev_roll_error) / dt;
    balance_pitch_rate = (balance_pitch_error - prev_pitch_error) / dt;

    balance_roll_cmd = (BALANCE_KP_ROLL * balance_roll_error) +
                       (BALANCE_KD_ROLL * balance_roll_rate);
    balance_pitch_cmd = (BALANCE_KP_PITCH * balance_pitch_error) +
                        (BALANCE_KD_PITCH * balance_pitch_rate);

    prev_roll_error = balance_roll_error;
    prev_pitch_error = balance_pitch_error;
    prev_update_ms = now_ms;
    balance_update_count++;

    if (out != 0)
    {
        out->roll_error = balance_roll_error;
        out->pitch_error = balance_pitch_error;
        out->roll_cmd = balance_roll_cmd;
        out->pitch_cmd = balance_pitch_cmd;
    }

    return 1;
}
