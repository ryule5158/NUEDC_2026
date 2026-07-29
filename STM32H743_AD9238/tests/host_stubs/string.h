/* 主机回归测试所需的最小 string.h 声明。 */
#ifndef FRACTIONAL_DELAY_HOST_STRING_H
#define FRACTIONAL_DELAY_HOST_STRING_H

#include <stddef.h>

/* 将 memset 重定向到无 C 运行库依赖的测试实现。 */
#define memset FractionalDelay_HostMemset

/* 按字节填充一段内存。 */
void *FractionalDelay_HostMemset(void *destination, int value, size_t length);

#endif
