#ifndef __DSP_HEADER_H__
#define __DSP_HEADER_H__ /* DSP接口汇总头文件包含保护。 */

#include "Filter.h"     /* FIR、IIR、中值、滑动平均和抽取滤波接口。 */
#include "FilterEx.h"   /* 流式FIR、SOS、巴特沃斯和平滑滤波接口。 */
#include "IQ.h"         /* 已知频率IQ幅度和相位提取接口。 */
#include "SoftPll.h"    /* 通用相位和频率软件锁相接口。 */
#include "Measure.h"    /* 时域、THD5/H1~H16、单周期波形和信号质量测量接口。 */
#include "DSP_ProMax.h" /* 一次获得常用时域、频域、谐波和信号质量参数。 */
#include "Demod.h"      /* AM、FM、BPSK解调和载波恢复接口。 */
#include "Correlate.h"  /* 互相关、自相关和基频估计接口。 */
#include "Fit.h"        /* 最小二乘直线和多项式拟合接口。 */
#include "Adaptive.h"   /* LMS自适应滤波和正弦分量分离接口。 */
#include "ModelFit.h"   /* 未知RLC网络二阶模型拟合接口。 */

#include "dual_response_analyzer.h"

#endif /* __DSP_HEADER_H__ */
