/**
  ******************************************************************************
  * @file    ap_adc.c
  * @brief   AP 层 ADC 管理 — DMA 启动 + 原始值读取
  ******************************************************************************
  */

#include "ap_adc.h"
#include <string.h>

void AP_ADC_Init(void)
{
    BSP_ADC_StartDMA();
}

uint16_t AP_ADC_GetLatest(uint32_t ch)
{
    uint16_t raw = 0;
    BSP_ADC_ReadValue(ch, &raw);
    return raw;
}
