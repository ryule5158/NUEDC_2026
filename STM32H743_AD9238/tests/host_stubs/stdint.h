/* 主机回归测试所需的最小 stdint.h 替身。 */
#ifndef FRACTIONAL_DELAY_HOST_STDINT_H
#define FRACTIONAL_DELAY_HOST_STDINT_H

typedef signed char int8_t;           /* 8 位有符号整数。 */
typedef unsigned char uint8_t;        /* 8 位无符号整数。 */
typedef signed short int16_t;         /* 16 位有符号整数。 */
typedef unsigned short uint16_t;      /* 16 位无符号整数。 */
typedef signed int int32_t;           /* 32 位有符号整数。 */
typedef unsigned int uint32_t;        /* 32 位无符号整数。 */
typedef signed __int64 int64_t;       /* 64 位有符号整数。 */
typedef unsigned __int64 uint64_t;    /* 64 位无符号整数。 */

#endif
