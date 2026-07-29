/* 主机回归测试所需的最小 math.h 替身。 */
#ifndef FRACTIONAL_DELAY_HOST_MATH_H
#define FRACTIONAL_DELAY_HOST_MATH_H

#include <float.h>

/* 判断单精度数是否为有限值。 */
#define isfinite(x) ((x) == (x) && (x) <= FLT_MAX && (x) >= -FLT_MAX)

/* 将 floorf 重定向到无 C 运行库依赖的测试实现。 */
#define floorf FractionalDelay_HostFloorf

/* 返回不大于输入值的最大整数单精度数。 */
static __inline float FractionalDelay_HostFloorf(float x)
{
    long long integer_part = (long long)x;
    if ((float)integer_part > x) --integer_part;
    return (float)integer_part;
}

#endif
