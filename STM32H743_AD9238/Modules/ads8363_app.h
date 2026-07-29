#ifndef __ADS8363_APP_H
#define __ADS8363_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ADS8363.h"

#define ADS8363_APP_CHANNEL_0             0U      /* ADS8363 A侧差分通道0, PCB丝印ADC-A1减ADC-A0 */
#define ADS8363_APP_CHANNEL_1             1U      /* ADS8363 A侧差分通道1, PCB丝印A3仅偏置减ADC-A2 */

/*
 * ADS8363应用层说明:
 * 当前APP只读取SDOA的A侧数据, SDOB/B侧未启用, 所以PCB上B侧输入暂不由本APP打印。
 * vref_mv只用于把ADC码值换算成mV; 测试板U3的REFIO1/REFIO2接VREF2V5_1,
 * 由REF5025提供约2500mV参考, 调试时可用实测参考电压替代2500.0f提高精度。
 * sample_rate_hz是APP层软件轮询采样节拍, 过高会受SPI速度、printf和主循环占用限制。
 */

/************************************************************
 * Function :       ADS8363_AppInit
 * Comment  :       初始化ADS8363应用层, 主函数先调用一次
 * Parameter:       channel: 0为ADC-A1-ADC-A0, 1为A3仅偏置-ADC-A2
 *                  vref_mv: 参考电压, 单位mV, 测试板REF5025常用2500.0f
 *                  sample_rate_hz: APP软件采样率, 单位Hz
 *                  print_interval_ms: 打印间隔, 单位ms, 0表示每次采样都打印
 * Return   :       ADS8363状态
 * Date     :       2026-06-13 V1
************************************************************/
ADS8363_StatusTypeDef ADS8363_AppInit(uint8_t channel,
                                      float vref_mv,
                                      float sample_rate_hz,
                                      uint32_t print_interval_ms);

/************************************************************
 * Function :       ADS8363_AppProcess
 * Comment  :       周期读取ADS8363采样并通过printf打印结果
 * Parameter:       null
 * Return   :       null
 * Date     :       2026-06-13 V1
************************************************************/
void ADS8363_AppProcess(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADS8363_APP_H */
