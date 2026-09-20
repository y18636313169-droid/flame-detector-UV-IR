/**
  ******************************************************************************
  * @file    ap_adc.h
  * @brief   AP 层 ADC 管理 — DMA 启动 + 原始值读取
  *
  *          BSP 层 3 通道 DMA 连续转换，本层仅提供 Init 和最新值读取。
  *          滑动滤波/信号处理由各模块自行实现（如 ap_ir 的 128 点FFT窗口）。
  ******************************************************************************
  */
#ifndef __AP_ADC_H__
#define __AP_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_adc.h"
#include <stdint.h>

int      AP_ADC_Init(void);
uint16_t AP_ADC_GetLatest(uint32_t ch);

#define AP_ADC_FAULT_DATA_STUCK    (1U << 0)
#define AP_ADC_FAULT_BIAS          (1U << 1)
#define AP_ADC_FAULT_DMA           (1U << 2)

/**
  * @brief  更新三通道ADC及DMA健康状态监视。
  * @param  now 当前HAL毫秒计时。
  * @param  fire_active 最终火警锁存状态；为1时暂停直流偏置故障判定。
  * @retval 任一ADC采样链故障确认后返回1，否则返回0。
  * @note   由主循环持续调用；这里只使用DMA原始值，不参与红外火焰算法。
  */
uint8_t  AP_ADC_FaultMonitorTask(uint32_t now, uint8_t fire_active);

/** @brief 查询当前是否存在已确认的ADC采样链故障。 */
uint8_t  AP_ADC_IsFault(void);

/** @brief 返回三通道近电源轨故障掩码，bit0/1/2对应3.8/4.5/5.0um。 */
uint8_t  AP_ADC_GetRailFaultMask(void);

/** @brief 返回数据冻结、偏置和DMA异常对应的AP_ADC_FAULT_*汇总标志。 */
uint8_t  AP_ADC_GetFaultFlags(void);

#ifdef __cplusplus
}
#endif

#endif /* __AP_ADC_H__ */
