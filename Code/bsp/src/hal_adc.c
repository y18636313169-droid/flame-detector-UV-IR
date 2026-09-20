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
/* Private variables ---------------------------------------------------------*/

/* DMA异步更新该数组，volatile防止CPU复用过期的寄存器缓存值。 */
static volatile uint16_t adc_dma_buf[BSP_ADC_NUM_CHANNELS];
static volatile uint32_t adc_dma_update_count = 0U;
static volatile uint32_t adc_error_count = 0U;
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
    /* volatile缓冲逐项清零，避免memset丢失DMA共享内存语义。 */
    for (uint32_t ch = 0; ch < BSP_ADC_NUM_CHANNELS; ch++) {
        adc_dma_buf[ch] = 0;
    }
    adc_dma_update_count = 0U;
    adc_error_count = 0U;
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

void BSP_ADC_ConvCpltHandler(ADC_HandleTypeDef *hadc_handle)
{
    if (hadc_handle != NULL && hadc_handle->Instance == ADC1) {
        /* ISR只提供进度证据，故障确认和GPIO更新全部留在主循环。 */
        adc_dma_update_count++;
    }
}

void BSP_ADC_ErrorHandler(ADC_HandleTypeDef *hadc_handle)
{
    if (hadc_handle != NULL && hadc_handle->Instance == ADC1) {
        /* 使用单调计数而非读后清零标志，避免主循环清零时漏掉新错误。 */
        adc_error_count++;
    }
}

uint32_t BSP_ADC_GetUpdateCount(void)
{
    return adc_dma_update_count;
}

uint32_t BSP_ADC_GetErrorCount(void)
{
    return adc_error_count;
}

uint8_t BSP_ADC_IsRunning(void)
{
    return adc_dma_running;
}
