/**
  ******************************************************************************
  * @file    ap_adc.c
  * @brief   AP 层 ADC 采样管理 — 3 通道独立滑动滤波
  ******************************************************************************
  */

#include "ap_adc.h"
#include "hal_adc.h"
#include "hal_uart.h"
#include <string.h>

/* Private types -------------------------------------------------------------*/

typedef struct {
    uint16_t    buf[AP_ADC_WIN_SIZE];
    uint16_t    idx;
    uint16_t    sum;
    uint16_t    latest;
    uint16_t    filtered;
    uint8_t     filled;
} AP_ADC_SlideFilter_t;

/* Private variables ---------------------------------------------------------*/

static AP_ADC_SlideFilter_t ap_filter[BSP_ADC_NUM_CHANNELS];
static volatile uint8_t ap_print_flag;  /* 由 Update 置位，主循环消费后清零 */

/* ---------------------------------------------------------------------------*/

static void filter_push(uint32_t ch, uint16_t raw)
{
    AP_ADC_SlideFilter_t *f = &ap_filter[ch];
    f->latest = raw;

    if (f->filled) f->sum -= f->buf[f->idx];
    f->buf[f->idx] = raw;
    f->sum += raw;

    f->idx = (f->idx + 1) % AP_ADC_WIN_SIZE;
    if (!f->filled && f->idx == 0) f->filled = 1;

    uint16_t count = f->filled ? AP_ADC_WIN_SIZE : f->idx;
    f->filtered = (count > 0) ? (f->sum / count) : 0;
}

/* ---------------------------------------------------------------------------*/

void AP_ADC_Init(void)
{
    memset(ap_filter, 0, sizeof(ap_filter));
    BSP_ADC_StartDMA();
    ap_print_flag = 0;
}

void AP_ADC_Update(void)
{
    uint16_t raw;
    static uint16_t cnt = 0;

    for (uint32_t ch = 0; ch < BSP_ADC_NUM_CHANNELS; ch++) {
        if (BSP_ADC_ReadValue(ch, &raw) == 0) {
            filter_push(ch, raw);
        }
    }

    cnt++;
    if (cnt >= AP_ADC_PRINT_INTERVAL) {
        cnt = 0;
        ap_print_flag = 1;
    }
}

uint16_t AP_ADC_GetFiltered(uint32_t ch)
{
    if (ch >= BSP_ADC_NUM_CHANNELS) return 0;
    return ap_filter[ch].filtered;
}

uint16_t AP_ADC_GetLatest(uint32_t ch)
{
    if (ch >= BSP_ADC_NUM_CHANNELS) return 0;
    return ap_filter[ch].latest;
}

uint8_t AP_ADC_IsPrintPending(void)
{
    uint8_t ret;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ret = ap_print_flag;
    ap_print_flag = 0;
    __set_PRIMASK(primask);
    return ret;
}

void AP_ADC_PrintDebug(void)
{
    for (uint32_t ch = 0; ch < BSP_ADC_NUM_CHANNELS; ch++) {
        BSP_UART_Printf("[ADC%u] raw=%4u(%4dmV) filtered=%4u\r\n",
            (unsigned)ch, ap_filter[ch].latest,
            BSP_ADC_RAW_TO_MV(ap_filter[ch].latest),
            ap_filter[ch].filtered);
    }
}
