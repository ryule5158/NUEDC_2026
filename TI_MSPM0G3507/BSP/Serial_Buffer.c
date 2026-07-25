#include "Serial_Buffer.h"

#define COMMAND_HEADER       0xAAU /* 指令帧头。 */
#define COMMAND_MIN_LENGTH      4U /* 合法指令最小长度。 */
#define BUFFER_SIZE           128U /* 循环缓冲区物理容量。 */

static uint8_t s_buffer[BUFFER_SIZE]; /* 串口指令循环缓冲区。 */
static uint8_t s_read_index;          /* 下一待读取位置。 */
static uint8_t s_write_index;         /* 下一待写入位置。 */

/* 增加循环缓冲区读索引。 */
static void Command_AddReadIndex(uint8_t length)
{
    s_read_index = (uint8_t)((s_read_index + length) % BUFFER_SIZE);
}

/* 读取循环缓冲区指定偏移的数据。 */
static uint8_t Command_Read(uint8_t index)
{
    return s_buffer[index % BUFFER_SIZE];
}

/* 返回循环缓冲区内有效数据长度。 */
static uint8_t Command_GetLength(void)
{
    return (uint8_t)((s_write_index + BUFFER_SIZE - s_read_index) %
                     BUFFER_SIZE);
}

/* 返回循环缓冲区可写长度，保留一格区分满和空。 */
static uint8_t Command_GetRemain(void)
{
    return (uint8_t)(BUFFER_SIZE - 1U - Command_GetLength());
}

uint8_t Command_Write(const uint8_t *data, uint8_t length)
{
    uint8_t first_length;

    if ((data == NULL) || (length == 0U) ||
        (Command_GetRemain() < length)) {
        return 0U;
    }

    first_length = (uint8_t)(BUFFER_SIZE - s_write_index);
    if (first_length > length) {
        first_length = length;
    }
    memcpy(&s_buffer[s_write_index], data, first_length);
    memcpy(s_buffer, &data[first_length], length - first_length);
    s_write_index =
        (uint8_t)((s_write_index + length) % BUFFER_SIZE);
    return length;
}

uint8_t Command_GetCommand(uint8_t *command)
{
    uint8_t length;
    uint8_t sum;
    uint8_t index;

    if (command == NULL) {
        return 0U;
    }

    while (Command_GetLength() >= COMMAND_MIN_LENGTH) {
        if (Command_Read(s_read_index) != COMMAND_HEADER) {
            Command_AddReadIndex(1U);
            continue;
        }

        length = Command_Read((uint8_t)(s_read_index + 1U));
        if ((length < COMMAND_MIN_LENGTH) || (length >= BUFFER_SIZE)) {
            Command_AddReadIndex(1U);
            continue;
        }
        if (Command_GetLength() < length) {
            return 0U;
        }

        sum = 0U;
        for (index = 0U; index < (uint8_t)(length - 1U); index++) {
            sum = (uint8_t)(sum +
                Command_Read((uint8_t)(s_read_index + index)));
        }
        if (sum !=
            Command_Read((uint8_t)(s_read_index + length - 1U))) {
            Command_AddReadIndex(1U);
            continue;
        }

        for (index = 0U; index < length; index++) {
            command[index] =
                Command_Read((uint8_t)(s_read_index + index));
        }
        Command_AddReadIndex(length);
        return length;
    }

    return 0U;
}
