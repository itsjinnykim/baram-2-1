#include "main.h"
#include "ebimu.h"

#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

#define IMU_UART        huart1
#define IMU_LINE_MAX    96

volatile float imu_roll = 0.0f;
volatile float imu_pitch = 0.0f;
volatile float imu_yaw = 0.0f;
volatile uint8_t imu_frame_ready = 0;
volatile uint8_t imu_parse_ok = 0;
volatile uint32_t imu_rx_count = 0;
volatile uint32_t imu_frame_count = 0;
volatile uint32_t imu_parse_fail_count = 0;
volatile uint32_t imu_last_update_ms = 0;

static uint8_t imu_rx_byte;
static char imu_rx_line[IMU_LINE_MAX];
static char imu_ready_line[IMU_LINE_MAX];
static volatile uint8_t imu_rx_line_len = 0;
static char imu_parse_line[IMU_LINE_MAX];

static void EBIMU_PushByte(uint8_t byte);
static uint8_t EBIMU_ParseLine(char *line, EBIMU_Data *out);

void EBIMU_StartReceive(void)
{
    (void)HAL_UART_Receive_IT(&IMU_UART, &imu_rx_byte, 1);
}

void EBIMU_ProcessFrame(void)
{
    EBIMU_Data data;

    if (!imu_frame_ready)
    {
        return;
    }

    __disable_irq();
    strncpy(imu_parse_line, imu_ready_line, sizeof(imu_parse_line));
    imu_parse_line[sizeof(imu_parse_line) - 1] = '\0';
    imu_frame_ready = 0;
    __enable_irq();

    if (EBIMU_ParseLine(imu_parse_line, &data))
    {
        imu_roll = data.roll;
        imu_pitch = data.pitch;
        imu_yaw = data.yaw;
        imu_parse_ok = 1;
        imu_last_update_ms = HAL_GetTick();
    }
    else
    {
        imu_parse_ok = 0;
        imu_parse_fail_count++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        EBIMU_PushByte(imu_rx_byte);
        EBIMU_StartReceive();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        EBIMU_StartReceive();
    }
}

static void EBIMU_PushByte(uint8_t byte)
{
    imu_rx_count++;

    if (byte == '\r' || byte == '\n')
    {
        if (imu_rx_line_len > 0 && !imu_frame_ready)
        {
            imu_rx_line[imu_rx_line_len] = '\0';
            memcpy(imu_ready_line, imu_rx_line, imu_rx_line_len + 1);
            imu_frame_ready = 1;
            imu_frame_count++;
        }
        imu_rx_line_len = 0;
        return;
    }

    if (imu_rx_line_len < (IMU_LINE_MAX - 1))
    {
        imu_rx_line[imu_rx_line_len++] = (char)byte;
    }
    else
    {
        imu_rx_line_len = 0;
    }
}

static uint8_t EBIMU_ParseLine(char *line, EBIMU_Data *out)
{
    char *p = line;
    char *end;
    float values[3];

    while (*p != '\0' && ((*p < '0' || *p > '9') && *p != '-' && *p != '+'))
    {
        p++;
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        values[i] = strtof(p, &end);
        if (end == p)
        {
            return 0;
        }

        p = end;
        while (*p == ',' || *p == ' ' || *p == '\t' || *p == ';')
        {
            p++;
        }
    }

    out->roll = values[0];
    out->pitch = values[1];
    out->yaw = values[2];

    return 1;
}
