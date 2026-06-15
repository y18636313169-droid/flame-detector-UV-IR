/**
  ******************************************************************************
  * @file    ap_adc.h
  * @brief   AP 层 ADC 采样管理 — 3 通道独立滑动滤波
  *
  *          基于 BSP 层 3 通道 DMA 连续转换，周期性读取各通道 ADC 原始值，
  *          分别做滑动窗口平均滤波，通过 DBG 串口输出调试信息。
  ******************************************************************************
  */
#ifndef __AP_ADC_H__
#define __AP_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_adc.h"
#include <stdint.h>

#define AP_ADC_WIN_SIZE          (5U)
#define AP_ADC_PRINT_INTERVAL    (50U)

void     AP_ADC_Init(void);
void     AP_ADC_Update(void);
uint16_t AP_ADC_GetFiltered(uint32_t ch);
uint16_t AP_ADC_GetLatest(uint32_t ch);

/** @brief  检查是否需要打印调试信息（由 Update 在 ISR 中置位）
  * @retval 1: 主循环应调用 AP_ADC_PrintDebug()
  * @note   内部自动清除标志，重复调用仅第一次返回 1 */
uint8_t  AP_ADC_IsPrintPending(void);

void     AP_ADC_PrintDebug(void);

#ifdef __cplusplus
}
#endif

#endif /* __AP_ADC_H__ */
