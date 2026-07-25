#ifndef __DSP_HEADER_H__
#define __DSP_HEADER_H__

#include "ADC_FFT.h"    /* ADC采样数据FFT分析接口，兼容BSP/adc_app电压缓冲区。 */
#include "Filter.h"     /* FIR/IIR, median, moving-average, and decimation filters. */
#include "IQ.h"         /* Known-frequency IQ amplitude/phase extraction. */
#include "SoftPll.h"    /* Generic phase/frequency software PLL controller. */
#include "Measure.h"    /* RMS/Vpp/frequency/THD5/H1~H16/SNR/SINAD/ENOB and waveform metrics. */
#include "Demod.h"      /* AM/FM demodulation and simple carrier-tracking PLL. */
#include "Correlate.h"  /* Cross-correlation and autocorrelation tools. */
#include "Fit.h"        /* Least-squares line/polynomial fitting. */
#include "Adaptive.h"   /* LMS adaptive filters and sine separation. */
#include "ModelFit.h"   /* Second-order model fitting for unknown RLC networks. */

#endif /* __DSP_HEADER_H__ */
