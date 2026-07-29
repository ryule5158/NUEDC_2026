#ifndef __SERIAL_BUFFER_H
#define __SERIAL_BUFFER_H

#include "main.h"
#include <string.h>

/* 向串口指令循环缓冲区写入数据，返回实际写入长度。 */
uint8_t Command_Write(const uint8_t *data, uint8_t length);

/* 从循环缓冲区提取一条完整指令，返回指令长度。 */
uint8_t Command_GetCommand(uint8_t *command);

#endif /* __SERIAL_BUFFER_H */
