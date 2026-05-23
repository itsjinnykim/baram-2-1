/*
 * app.c
 *
 * Dynamixel AX-12A test with STM32 Nucleo-F401RE
 * PA11 = USART6_TX
 * PA12 = USART6_RX
 * PB0  = 74HC126 OE/G
 *
 * ID 0~17 used
 * 0, 3, 6, 9, 12, 15 are never moved
 */

#include "main.h"
#include "app.h"

#include <stdint.h>
#include <string.h>

extern UART_HandleTypeDef huart6;

#define DXL_UART        huart6

#define DXL_DIR_PORT    GPIOB
#define DXL_DIR_PIN     GPIO_PIN_0

#define DXL_TX_MODE()   HAL_GPIO_WritePin(DXL_DIR_PORT, DXL_DIR_PIN, GPIO_PIN_SET)
#define DXL_RX_MODE()   HAL_GPIO_WritePin(DXL_DIR_PORT, DXL_DIR_PIN, GPIO_PIN_RESET)

#define DXL_ADDR_TORQUE_ENABLE      24
#define DXL_ADDR_GOAL_POSITION      30
#define DXL_ADDR_PRESENT_POSITION   36

#define DXL_INST_READ       0x02
#define DXL_INST_WRITE      0x03

#define POS_A   480
#define POS_B   544

static const uint8_t dxl_ids[] = {
    0, 1, 2,
    3, 4, 5,
    6, 7, 8,
    9, 10, 11,
    12, 13, 14,
    15, 16, 17
};

volatile uint8_t  dxl_current_id = 0;
volatile uint16_t dxl_goal_position = 0;
volatile uint16_t dxl_present_position = 0;
volatile uint8_t  dxl_last_error = 0;
volatile uint8_t  dxl_last_comm_ok = 0;
volatile uint32_t dxl_loop_count = 0;
volatile uint32_t dxl_comm_success_count = 0;
volatile uint32_t dxl_comm_fail_count = 0;
volatile uint8_t  dxl_last_success_id = 0xFF;
volatile uint8_t  dxl_id1_comm_ok = 0;
volatile uint16_t dxl_id1_present_position = 0;

static void DXL_GPIO_Init(void);
static uint8_t DXL_Checksum(uint8_t *packet, uint8_t length);
static HAL_StatusTypeDef DXL_SendPacket(uint8_t *packet, uint8_t length);
static HAL_StatusTypeDef DXL_ReadStatusPacket(uint8_t *rx, uint8_t expected_len, uint32_t timeout);

static uint8_t DXL_Write1Byte(uint8_t id, uint8_t address, uint8_t data);
static uint8_t DXL_Write2Byte(uint8_t id, uint8_t address, uint16_t data);
static uint8_t DXL_Read2Byte(uint8_t id, uint8_t address, uint16_t *out_value);

static uint8_t IsMovingID(uint8_t id);
static void DXL_RecordCommResult(uint8_t id, uint8_t ok);

void App_Init(void)
{
    DXL_GPIO_Init();
    DXL_RX_MODE();

    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    for (int i = 0; i < sizeof(dxl_ids) / sizeof(dxl_ids[0]); i++)
    {
        uint8_t id = dxl_ids[i];
        dxl_current_id = id;

        if (IsMovingID(id))
        {
            uint8_t ok = DXL_Write1Byte(id, DXL_ADDR_TORQUE_ENABLE, 1);
            DXL_RecordCommResult(id, ok);
            HAL_Delay(20);
        }
        else
        {
            uint8_t ok = DXL_Write1Byte(id, DXL_ADDR_TORQUE_ENABLE, 0);
            DXL_RecordCommResult(id, ok);
            HAL_Delay(20);
        }

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
}

void App_Loop(void)
{
    static uint8_t toggle = 0;
    uint16_t target_pos;

    if (toggle == 0)
    {
        target_pos = POS_A;
        toggle = 1;
    }
    else
    {
        target_pos = POS_B;
        toggle = 0;
    }

    dxl_goal_position = target_pos;

    for (int i = 0; i < sizeof(dxl_ids) / sizeof(dxl_ids[0]); i++)
    {
        uint8_t id = dxl_ids[i];
        dxl_current_id = id;

        if (IsMovingID(id))
        {
            dxl_last_comm_ok = DXL_Write2Byte(id, DXL_ADDR_GOAL_POSITION, target_pos);
            DXL_RecordCommResult(id, dxl_last_comm_ok);
            HAL_Delay(50);

            uint16_t present = 0;
            dxl_last_comm_ok = DXL_Read2Byte(id, DXL_ADDR_PRESENT_POSITION, &present);
            DXL_RecordCommResult(id, dxl_last_comm_ok);

            if (dxl_last_comm_ok)
            {
                dxl_present_position = present;
                if (id == 1)
                {
                    dxl_id1_present_position = present;
                }
            }

            HAL_Delay(100);
        }
        else
        {
            uint16_t present = 0;
            dxl_last_comm_ok = DXL_Read2Byte(id, DXL_ADDR_PRESENT_POSITION, &present);
            DXL_RecordCommResult(id, dxl_last_comm_ok);

            if (dxl_last_comm_ok)
            {
                dxl_present_position = present;
                if (id == 1)
                {
                    dxl_id1_present_position = present;
                }
            }

            HAL_Delay(50);
        }

        dxl_loop_count++;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

    HAL_Delay(1000);
}

static void DXL_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = DXL_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(DXL_DIR_PORT, &GPIO_InitStruct);

    DXL_RX_MODE();
}

static uint8_t IsMovingID(uint8_t id)
{
    if (id == 0 || id == 3 || id == 6 ||
        id == 9 || id == 12 || id == 15)
    {
        return 0;
    }

    return 1;
}

static void DXL_RecordCommResult(uint8_t id, uint8_t ok)
{
    if (ok)
    {
        dxl_comm_success_count++;
        dxl_last_success_id = id;

        if (id == 1)
        {
            dxl_id1_comm_ok = 1;
        }
    }
    else
    {
        dxl_comm_fail_count++;

        if (id == 1)
        {
            dxl_id1_comm_ok = 0;
        }
    }
}

static uint8_t DXL_Checksum(uint8_t *packet, uint8_t length)
{
    uint16_t sum = 0;

    for (uint8_t i = 2; i < length - 1; i++)
    {
        sum += packet[i];
    }

    return (uint8_t)(~sum);
}

static HAL_StatusTypeDef DXL_SendPacket(uint8_t *packet, uint8_t length)
{
    HAL_StatusTypeDef result;

    DXL_TX_MODE();
    HAL_Delay(1);

    result = HAL_UART_Transmit(&DXL_UART, packet, length, 100);

    uint32_t tickstart = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(&DXL_UART, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - tickstart) > 100)
        {
            DXL_RX_MODE();
            return HAL_TIMEOUT;
        }
    }

    DXL_RX_MODE();

    return result;
}

static HAL_StatusTypeDef DXL_ReadStatusPacket(uint8_t *rx, uint8_t expected_len, uint32_t timeout)
{
    memset(rx, 0, expected_len);

    return HAL_UART_Receive(&DXL_UART, rx, expected_len, timeout);
}

static uint8_t DXL_Write1Byte(uint8_t id, uint8_t address, uint8_t data)
{
    uint8_t packet[8];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 4;
    packet[4] = DXL_INST_WRITE;
    packet[5] = address;
    packet[6] = data;
    packet[7] = DXL_Checksum(packet, 8);

    if (DXL_SendPacket(packet, 8) != HAL_OK)
    {
        return 0;
    }

    uint8_t rx[6];

    if (DXL_ReadStatusPacket(rx, 6, 100) != HAL_OK)
    {
        return 0;
    }

    if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id)
    {
        return 0;
    }

    dxl_last_error = rx[4];

    return rx[4] == 0;
}

static uint8_t DXL_Write2Byte(uint8_t id, uint8_t address, uint16_t data)
{
    uint8_t packet[9];

    uint8_t data_l = data & 0xFF;
    uint8_t data_h = (data >> 8) & 0xFF;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 5;
    packet[4] = DXL_INST_WRITE;
    packet[5] = address;
    packet[6] = data_l;
    packet[7] = data_h;
    packet[8] = DXL_Checksum(packet, 9);

    if (DXL_SendPacket(packet, 9) != HAL_OK)
    {
        return 0;
    }

    uint8_t rx[6];

    if (DXL_ReadStatusPacket(rx, 6, 100) != HAL_OK)
    {
        return 0;
    }

    if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id)
    {
        return 0;
    }

    dxl_last_error = rx[4];

    return rx[4] == 0;
}

static uint8_t DXL_Read2Byte(uint8_t id, uint8_t address, uint16_t *out_value)
{
    uint8_t packet[8];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 4;
    packet[4] = DXL_INST_READ;
    packet[5] = address;
    packet[6] = 2;
    packet[7] = DXL_Checksum(packet, 8);

    if (DXL_SendPacket(packet, 8) != HAL_OK)
    {
        return 0;
    }

    uint8_t rx[8];

    if (DXL_ReadStatusPacket(rx, 8, 100) != HAL_OK)
    {
        return 0;
    }

    if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id)
    {
        return 0;
    }

    dxl_last_error = rx[4];

    if (rx[4] != 0)
    {
        return 0;
    }

    uint8_t low = rx[5];
    uint8_t high = rx[6];

    *out_value = (uint16_t)(low | (high << 8));

    return 1;
}
