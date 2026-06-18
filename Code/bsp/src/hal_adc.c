/**
  ******************************************************************************
  * @file    hal_adc.c
  * @brief   BSP ADC 抽象层实现 — 3 通道 DMA 连续转换
  *
  *          CubeMX 配置：
  *            - ADC1, 三通道扫描(IN10/PC0, IN11/PC1, IN12/PC2), 连续转换
  *            - DMA1_CH1, CIRCULAR, 半字传输, 内存递增
  ******************************************************************************
  */

#include "hal_adc.h"
#include "adc.h"
#include "string.h"
/* Private variables ---------------------------------------------------------*/

static uint16_t adc_dma_buf[BSP_ADC_NUM_CHANNELS];                 /* 3 通道 DMA 缓冲 */
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
    /* 清空 DMA 缓冲，防止上电后第一次读取未定义数据 */
    memset(adc_dma_buf, 0, sizeof(adc_dma_buf));
    if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_dma_buf, BSP_ADC_NUM_CHANNELS) != HAL_OK) {
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

int BSP_ADC_ReadValue(uint32_t ch, uint16_t *raw)
{
    if (!adc_dma_running || raw == NULL) return -1;
    if (ch >= BSP_ADC_NUM_CHANNELS) return -1;
    *raw = adc_dma_buf[ch];
    return 0;
}
