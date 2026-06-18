/**
  ******************************************************************************
  * @file    ap_adc.h
  * @brief   AP 层 ADC 管理 — DMA 启动 + 原始值读取
  *
  *          BSP 层 3 通道 DMA 连续转换，本层仅提供 Init 和最新值读取。
  *          滑动滤波/信号处理由各模块自行实现（如 ap_ir 的 50 点窗口）。
  ******************************************************************************
  */
#ifndef __AP_ADC_H__
#define __AP_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_adc.h"
#include <stdint.h>

void     AP_ADC_Init(void);
uint16_t AP_ADC_GetLatest(uint32_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __AP_ADC_H__ */
