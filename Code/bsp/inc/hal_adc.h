/**
  ******************************************************************************
  * @file    hal_adc.h
  * @brief   BSP ADC 抽象层头文件
  *
  *          CubeMX 配置：ADC1, 三通道扫描, 连续转换, DMA1_CH1 循环
  *            - ADC_IN10(PC0) — 热电堆传感器 1
  *            - ADC_IN11(PC1) — 热电堆传感器 2
  *            - ADC_IN12(PC2) — 热电堆传感器 3
  *
  *          BSP 层管理 3 通道 DMA 缓冲，支持按通道读取最新原始值。
  *          滑动滤波归 ap_adc 管理。
  ******************************************************************************
  */
#ifndef __BSP_HAL_ADC_H__
#define __BSP_HAL_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define BSP_ADC_OK              (0)
#define BSP_ADC_ERROR           (-1)

#define BSP_ADC_NUM_CHANNELS    (3U)          /* 扫描通道数 */
#define BSP_ADC_VREF_MV         (3300UL)
#define BSP_ADC_RESOLUTION      (4096UL)
#define BSP_ADC_RAW_TO_MV(raw)  (((uint32_t)(raw) * BSP_ADC_VREF_MV) / BSP_ADC_RESOLUTION)

/* Exported functions --------------------------------------------------------*/

int  BSP_ADC_Init(void);
int  BSP_ADC_StartDMA(void);
void BSP_ADC_StopDMA(void);

/**
  * @brief  读取指定通道的最新 ADC 原始值
  * @param  ch: 通道索引（0 ~ BSP_ADC_NUM_CHANNELS-1）
  * @param  raw: 输出指针
  * @retval 0: 成功
  * @retval -1: DMA 未启动 或 ch 无效
  */
int  BSP_ADC_ReadValue(uint32_t ch, uint16_t *raw);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_ADC_H__ */
