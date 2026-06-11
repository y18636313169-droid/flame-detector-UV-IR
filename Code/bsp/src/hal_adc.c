/**
  ******************************************************************************
  * @file    hal_adc.c
  * @brief   BSP ADC 抽象层实现 — 单通道 DMA 连续转换
  *
  *          CubeMX 配置：
  *            - ADC1, 扫描模式(1通道 IN11/PC1), 连续转换
  *            - DMA1_CH1, CIRCULAR, 半字传输
  ******************************************************************************
  */

#include "hal_adc.h"
#include "adc.h"

/* Private variables ---------------------------------------------------------*/

static uint16_t adc_dma_buf[1];
static uint8_t  adc_dma_running = 0;
static uint8_t  adc_initialized = 0;

/* ---------------------------------------------------------------------------*/

int BSP_ADC_Init(void)
{
    if (adc_initialized) return BSP_ADC_OK;
    if (hadc.Instance == NULL) return BSP_ADC_ERROR;
    adc_initialized = 1;
    return BSP_ADC_OK;
}

int BSP_ADC_StartDMA(void)
{
    if (!adc_initialized) return BSP_ADC_ERROR;
    if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_dma_buf, 1) != HAL_OK) {
        return BSP_ADC_ERROR;
    }
    adc_dma_running = 1;
    return BSP_ADC_OK;
}

void BSP_ADC_StopDMA(void)
{
    if (!adc_dma_running) return;
    HAL_ADC_Stop_DMA(&hadc);
    adc_dma_running = 0;
}

int BSP_ADC_ReadValue(uint16_t *raw)
{
    if (!adc_dma_running || raw == NULL) return -1;
    *raw = adc_dma_buf[0];
    return 0;
}
