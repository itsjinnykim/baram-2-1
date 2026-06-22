#include "main.h"
#include "dynamixel.h"

#include <string.h>

extern UART_HandleTypeDef huart6;

#define DXL_UART        huart6

#define DXL_TX_MODE()   HAL_GPIO_WritePin(DXL_DIR_GPIO_Port, DXL_DIR_Pin, GPIO_PIN_SET)
#define DXL_RX_MODE()   HAL_GPIO_WritePin(DXL_DIR_GPIO_Port, DXL_DIR_Pin, GPIO_PIN_RESET)

#define DXL_ADDR_TORQUE_ENABLE      24
#define DXL_ADDR_GOAL_POSITION      30
#define DXL_ADDR_MOVING_SPEED       32
#define DXL_ADDR_PRESENT_POSITION   36

#define DXL_INST_READ       0x02
#define DXL_INST_WRITE      0x03
#define DXL_INST_SYNC_WRITE 0x83
#define DXL_BROADCAST_ID    0xFE

#define DXL_TEST_POS_A      480
#define DXL_TEST_POS_B      544
#define DXL_SAFE_DELTA_TICK 40
#define DXL_BALANCE_SPEED   55

#define DXL_DIAG_OK             0
#define DXL_DIAG_TX_FAIL        1
#define DXL_DIAG_RX_TIMEOUT     2
#define DXL_DIAG_BAD_HEADER     3
#define DXL_DIAG_BAD_ID         4
#define DXL_DIAG_STATUS_ERROR   5

static const uint8_t dxl_ids[] = {
    0, 1, 2,
    3, 4, 5,
    6, 7, 8,
    9, 10, 11,
    12, 13, 14,
    15, 16, 17
};

static const uint16_t dxl_home_position[18] = {
    509, 465, 165,
    203, 457, 154,
    200, 448, 162,
    198, 491, 139,
    190, 474, 145,
    199, 484, 128
};

volatile uint8_t dxl_current_id = 0;
volatile uint16_t dxl_goal_position = 0;
volatile uint16_t dxl_present_position = 0;
volatile uint8_t dxl_last_error = 0;
volatile uint8_t dxl_last_comm_ok = 0;
volatile uint8_t dxl_last_diag = DXL_DIAG_OK;

volatile uint8_t dxl_last_rx0 = 0;
volatile uint8_t dxl_last_rx1 = 0;
volatile uint8_t dxl_last_rx2 = 0;
volatile uint8_t dxl_last_rx3 = 0;
volatile uint8_t dxl_last_rx4 = 0;
volatile uint8_t dxl_last_rx5 = 0;
volatile uint8_t dxl_last_rx6 = 0;
volatile uint8_t dxl_last_rx7 = 0;

volatile uint32_t dxl_loop_count = 0;
volatile uint32_t dxl_comm_success_count = 0;
volatile uint32_t dxl_comm_fail_count = 0;
volatile uint8_t dxl_last_success_id = 0xFF;
volatile uint16_t dxl_measured_home[18] = {0};
volatile uint8_t dxl_home_read_ok[18] = {0};
volatile uint32_t dxl_home_read_fail_count = 0;
volatile uint8_t dxl_home_capture_done = 0;

static void DXL_GPIO_Init(void);
static uint8_t DXL_Checksum(uint8_t *packet, uint8_t length);
static HAL_StatusTypeDef DXL_SendPacket(uint8_t *packet, uint8_t length);
static HAL_StatusTypeDef DXL_ReadStatusPacket(uint8_t id, uint8_t *rx, uint8_t expected_len, uint32_t timeout);
static void DXL_FlushRx(void);
static uint8_t DXL_Write1Byte(uint8_t id, uint8_t address, uint8_t data);
static uint8_t DXL_Write2Byte(uint8_t id, uint8_t address, uint16_t data);
static uint8_t DXL_Read2Byte(uint8_t id, uint8_t address, uint16_t *out_value);
static uint16_t DXL_GetSafePosition(uint8_t id, uint16_t position);
static void DXL_RecordCommResult(uint8_t id, uint8_t ok);
static void DXL_SaveLastRx(uint8_t *rx, uint8_t length);

void DXL_Init(void)
{
    DXL_GPIO_Init();
    DXL_RX_MODE();
}

void DXL_TorqueEnableAll(void)
{
    for (uint32_t i = 0; i < sizeof(dxl_ids) / sizeof(dxl_ids[0]); i++)
    {
        uint8_t id = dxl_ids[i];
        uint8_t ok;

        dxl_current_id = id;
        ok = DXL_Write1Byte(id, DXL_ADDR_TORQUE_ENABLE, 1);
        dxl_last_comm_ok = ok;
        DXL_RecordCommResult(id, ok);
        HAL_Delay(20);

        ok = DXL_Write2Byte(id, DXL_ADDR_MOVING_SPEED, DXL_BALANCE_SPEED);
        dxl_last_comm_ok = ok;
        DXL_RecordCommResult(id, ok);
        HAL_Delay(20);

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
}
void DXL_TorqueDisableAll(void)
{
    for (uint32_t i = 0; i < sizeof(dxl_ids) / sizeof(dxl_ids[0]); i++)
    {
        uint8_t id = dxl_ids[i];
        uint8_t ok;

        dxl_current_id = id;
        ok = DXL_Write1Byte(id, DXL_ADDR_TORQUE_ENABLE, 0);
        dxl_last_comm_ok = ok;
        DXL_RecordCommResult(id, ok);

        HAL_Delay(20);
    }
}

void DXL_CaptureHomePosition(void)
{
    dxl_home_capture_done = 0;
    dxl_home_read_fail_count = 0;

    for (uint32_t i = 0; i < sizeof(dxl_ids) / sizeof(dxl_ids[0]); i++)
    {
        uint8_t id = dxl_ids[i];
        uint16_t present = 0;

        dxl_measured_home[id] = 0xFFFF;
        dxl_home_read_ok[id] = 0;

        if (DXL_ReadPresentPosition(id, &present))
        {
            dxl_measured_home[id] = present;
            dxl_home_read_ok[id] = 1;
        }
        else
        {
            dxl_home_read_fail_count++;
        }

        HAL_Delay(20);
    }

    dxl_home_capture_done = 1;
}

void DXL_MoveHomePosition(void)
{
    for (uint32_t i = 0; i < sizeof(dxl_ids) / sizeof(dxl_ids[0]); i++)
    {
        uint8_t id = dxl_ids[i];

        (void)DXL_WriteGoalPosition(id, dxl_home_position[id]);
        HAL_Delay(20);
    }
}
void DXL_RunReferenceMotionTest(void)
{
    static uint8_t toggle = 0;
    uint16_t target_pos;

    if (toggle == 0)
    {
        target_pos = DXL_TEST_POS_A;
        toggle = 1;
    }
    else
    {
        target_pos = DXL_TEST_POS_B;
        toggle = 0;
    }

    dxl_goal_position = target_pos;

    for (uint32_t i = 0; i < sizeof(dxl_ids) / sizeof(dxl_ids[0]); i++)
    {
        uint8_t id = dxl_ids[i];
        uint16_t present = 0;

        dxl_current_id = id;

        if (DXL_IsMovingID(id))
        {
            (void)DXL_WriteGoalPosition(id, target_pos);
            HAL_Delay(50);
        }

        (void)DXL_ReadPresentPosition(id, &present);
        dxl_loop_count++;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

        if (DXL_IsMovingID(id))
        {
            HAL_Delay(100);
        }
        else
        {
            HAL_Delay(50);
        }
    }

    HAL_Delay(1000);
}

uint8_t DXL_WriteGoalPosition(uint8_t id, uint16_t position)
{
    uint8_t ok;

    dxl_current_id = id;
    dxl_goal_position = position;
    ok = DXL_Write2Byte(id, DXL_ADDR_GOAL_POSITION, position);
    dxl_last_comm_ok = ok;
    DXL_RecordCommResult(id, ok);

    return ok;
}

uint8_t DXL_WriteSafeGoalPosition(uint8_t id, uint16_t position)
{
    if (id >= (sizeof(dxl_home_position) / sizeof(dxl_home_position[0])))
    {
        return 0;
    }

    return DXL_WriteGoalPosition(id, DXL_GetSafePosition(id, position));
}

uint8_t DXL_SyncWriteSafeGoalPositions(const uint8_t *ids, const uint16_t *positions, uint8_t count)
{
    uint8_t packet[64];
    uint8_t index = 0;

    if (ids == 0 || positions == 0 || count == 0 || count > 18)
    {
        return 0;
    }

    packet[index++] = 0xFF;
    packet[index++] = 0xFF;
    packet[index++] = DXL_BROADCAST_ID;
    packet[index++] = (uint8_t)((count * 3U) + 4U);
    packet[index++] = DXL_INST_SYNC_WRITE;
    packet[index++] = DXL_ADDR_GOAL_POSITION;
    packet[index++] = 2;

    for (uint8_t i = 0; i < count; i++)
    {
        uint8_t id = ids[i];
        uint16_t safe_position;

        if (id >= (sizeof(dxl_home_position) / sizeof(dxl_home_position[0])) || !DXL_IsMovingID(id))
        {
            return 0;
        }

        safe_position = DXL_GetSafePosition(id, positions[i]);
        packet[index++] = id;
        packet[index++] = safe_position & 0xFF;
        packet[index++] = (safe_position >> 8) & 0xFF;
    }

    packet[index] = DXL_Checksum(packet, index + 1U);
    index++;

    if (DXL_SendPacket(packet, index) != HAL_OK)
    {
        dxl_last_diag = DXL_DIAG_TX_FAIL;
        dxl_last_comm_ok = 0;
        dxl_comm_fail_count++;
        return 0;
    }

    dxl_last_diag = DXL_DIAG_OK;
    dxl_last_comm_ok = 1;
    dxl_comm_success_count++;
    dxl_last_success_id = DXL_BROADCAST_ID;

    return 1;
}

uint8_t DXL_ReadPresentPosition(uint8_t id, uint16_t *out_value)
{
    uint8_t ok;

    dxl_current_id = id;
    ok = DXL_Read2Byte(id, DXL_ADDR_PRESENT_POSITION, out_value);
    dxl_last_comm_ok = ok;
    DXL_RecordCommResult(id, ok);

    if (ok)
    {
        dxl_present_position = *out_value;
    }

    return ok;
}

uint8_t DXL_IsMovingID(uint8_t id)
{
    if (id == 0 || id == 3 || id == 6 ||
        id == 9 || id == 12 || id == 15)
    {
        return 0;
    }

    return 1;
}

uint16_t DXL_ClampPosition(int32_t position)
{
    if (position < DXL_MIN_POS)
    {
        return DXL_MIN_POS;
    }

    if (position > DXL_MAX_POS)
    {
        return DXL_MAX_POS;
    }

    return (uint16_t)position;
}

static uint16_t DXL_GetSafePosition(uint8_t id, uint16_t position)
{
    int32_t min_position;
    int32_t max_position;
    int32_t safe_position;

    min_position = (int32_t)dxl_home_position[id] - DXL_SAFE_DELTA_TICK;
    max_position = (int32_t)dxl_home_position[id] + DXL_SAFE_DELTA_TICK;
    safe_position = (int32_t)position;

    if (safe_position < min_position)
    {
        safe_position = min_position;
    }

    if (safe_position > max_position)
    {
        safe_position = max_position;
    }

    return DXL_ClampPosition(safe_position);
}
static void DXL_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = DXL_DIR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DXL_DIR_GPIO_Port, &GPIO_InitStruct);
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
    uint32_t tickstart;

    DXL_FlushRx();
    DXL_TX_MODE();
    HAL_Delay(1);

    result = HAL_UART_Transmit(&DXL_UART, packet, length, 100);

    tickstart = HAL_GetTick();
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

static HAL_StatusTypeDef DXL_ReadStatusPacket(uint8_t id, uint8_t *rx, uint8_t expected_len, uint32_t timeout)
{
    uint8_t byte;
    uint8_t index = 0;
    uint32_t start_ms = HAL_GetTick();

    memset(rx, 0, expected_len);

    while ((HAL_GetTick() - start_ms) <= timeout)
    {
        if (HAL_UART_Receive(&DXL_UART, &byte, 1, 1) != HAL_OK)
        {
            continue;
        }

        if (index == 0)
        {
            if (byte == 0xFF)
            {
                rx[index++] = byte;
            }
            continue;
        }

        if (index == 1)
        {
            if (byte == 0xFF)
            {
                rx[index++] = byte;
            }
            else
            {
                index = (byte == 0xFF) ? 1 : 0;
            }
            continue;
        }

        if (index == 2 && byte == 0xFF)
        {
            rx[0] = 0xFF;
            rx[1] = 0xFF;
            continue;
        }

        rx[index++] = byte;
        if (index >= expected_len)
        {
            DXL_SaveLastRx(rx, expected_len);

            if (rx[2] == id)
            {
                return HAL_OK;
            }

            dxl_last_diag = DXL_DIAG_BAD_ID;
            index = 0;
            memset(rx, 0, expected_len);
        }
    }

    return HAL_TIMEOUT;
}

static void DXL_FlushRx(void)
{
    __HAL_UART_CLEAR_OREFLAG(&DXL_UART);

    while (__HAL_UART_GET_FLAG(&DXL_UART, UART_FLAG_RXNE) != RESET)
    {
        (void)DXL_UART.Instance->DR;
    }
}

static uint8_t DXL_Write1Byte(uint8_t id, uint8_t address, uint8_t data)
{
    uint8_t packet[8];
    uint8_t rx[6];

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
        dxl_last_diag = DXL_DIAG_TX_FAIL;
        return 0;
    }

    if (DXL_ReadStatusPacket(id, rx, 6, 100) != HAL_OK)
    {
        if (dxl_last_diag != DXL_DIAG_BAD_ID)
        {
            dxl_last_diag = DXL_DIAG_RX_TIMEOUT;
        }
        return 0;
    }

    if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id)
    {
        dxl_last_diag = (rx[0] != 0xFF || rx[1] != 0xFF) ? DXL_DIAG_BAD_HEADER : DXL_DIAG_BAD_ID;
        return 0;
    }

    dxl_last_error = rx[4];
    dxl_last_diag = (rx[4] == 0) ? DXL_DIAG_OK : DXL_DIAG_STATUS_ERROR;

    return rx[4] == 0;
}

static uint8_t DXL_Write2Byte(uint8_t id, uint8_t address, uint16_t data)
{
    uint8_t packet[9];
    uint8_t rx[6];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 5;
    packet[4] = DXL_INST_WRITE;
    packet[5] = address;
    packet[6] = data & 0xFF;
    packet[7] = (data >> 8) & 0xFF;
    packet[8] = DXL_Checksum(packet, 9);

    if (DXL_SendPacket(packet, 9) != HAL_OK)
    {
        dxl_last_diag = DXL_DIAG_TX_FAIL;
        return 0;
    }

    if (DXL_ReadStatusPacket(id, rx, 6, 100) != HAL_OK)
    {
        if (dxl_last_diag != DXL_DIAG_BAD_ID)
        {
            dxl_last_diag = DXL_DIAG_RX_TIMEOUT;
        }
        return 0;
    }

    if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id)
    {
        dxl_last_diag = (rx[0] != 0xFF || rx[1] != 0xFF) ? DXL_DIAG_BAD_HEADER : DXL_DIAG_BAD_ID;
        return 0;
    }

    dxl_last_error = rx[4];
    dxl_last_diag = (rx[4] == 0) ? DXL_DIAG_OK : DXL_DIAG_STATUS_ERROR;

    return rx[4] == 0;
}

static uint8_t DXL_Read2Byte(uint8_t id, uint8_t address, uint16_t *out_value)
{
    uint8_t packet[8];
    uint8_t rx[8];

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
        dxl_last_diag = DXL_DIAG_TX_FAIL;
        return 0;
    }

    if (DXL_ReadStatusPacket(id, rx, 8, 100) != HAL_OK)
    {
        if (dxl_last_diag != DXL_DIAG_BAD_ID)
        {
            dxl_last_diag = DXL_DIAG_RX_TIMEOUT;
        }
        return 0;
    }

    if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id)
    {
        dxl_last_diag = (rx[0] != 0xFF || rx[1] != 0xFF) ? DXL_DIAG_BAD_HEADER : DXL_DIAG_BAD_ID;
        return 0;
    }

    dxl_last_error = rx[4];

    if (rx[4] != 0)
    {
        dxl_last_diag = DXL_DIAG_STATUS_ERROR;
        return 0;
    }

    *out_value = (uint16_t)(rx[5] | ((uint16_t)rx[6] << 8));
    dxl_last_diag = DXL_DIAG_OK;

    return 1;
}

static void DXL_RecordCommResult(uint8_t id, uint8_t ok)
{
    if (ok)
    {
        dxl_comm_success_count++;
        dxl_last_success_id = id;
    }
    else
    {
        dxl_comm_fail_count++;
    }
}

static void DXL_SaveLastRx(uint8_t *rx, uint8_t length)
{
    dxl_last_rx0 = (length > 0) ? rx[0] : 0;
    dxl_last_rx1 = (length > 1) ? rx[1] : 0;
    dxl_last_rx2 = (length > 2) ? rx[2] : 0;
    dxl_last_rx3 = (length > 3) ? rx[3] : 0;
    dxl_last_rx4 = (length > 4) ? rx[4] : 0;
    dxl_last_rx5 = (length > 5) ? rx[5] : 0;
    dxl_last_rx6 = (length > 6) ? rx[6] : 0;
    dxl_last_rx7 = (length > 7) ? rx[7] : 0;
}





