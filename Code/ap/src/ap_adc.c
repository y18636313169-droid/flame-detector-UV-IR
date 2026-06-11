/**
  ******************************************************************************
  * @file    ap_adc.c
  * @brief   AP 层 ADC 采样管理 — 单通道滑动滤波
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

static AP_ADC_SlideFilter_t ap_filter;
static volatile uint8_t ap_print_flag;  /* 由 Update 置位，主循环消费后清零 */

/* ---------------------------------------------------------------------------*/

static void filter_push(uint16_t raw)
{
    AP_ADC_SlideFilter_t *f = &ap_filter;
    f->latest = raw;

    if (f->filled) f->sum -= f->buf[f->idx];    // 满 减去要覆盖的idx数据
    f->buf[f->idx] = raw;
    f->sum += raw;

    f->idx = (f->idx + 1) % AP_ADC_WIN_SIZE;
    if (!f->filled && f->idx == 0) f->filled = 1;   // 写指针回环 置位已满

    uint16_t count = f->filled ? AP_ADC_WIN_SIZE : f->idx;  // 取元素个数
    f->filtered = (count > 0) ? (f->sum / count) : 0;
}

/* ---------------------------------------------------------------------------*/

void AP_ADC_Init(void)
{
    memset(&ap_filter, 0, sizeof(ap_filter));
    BSP_ADC_StartDMA();
    ap_print_flag = 0;
}

void AP_ADC_Update(void)
{
    uint16_t raw;
    static uint16_t cnt = 0;
    if (BSP_ADC_ReadValue(&raw) != 0) return;

    filter_push(raw);

    cnt++;
    if (cnt >= AP_ADC_PRINT_INTERVAL) {
        cnt = 0;
        ap_print_flag = 1;
    }
}

uint16_t AP_ADC_GetFiltered(void)
{
    return ap_filter.filtered;
}

uint16_t AP_ADC_GetLatest(void)
{
    return ap_filter.latest;
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
    BSP_UART_Printf("[ADC] raw=%4u(%4dmV) filtered=%4u\r\n",
        ap_filter.latest,
        BSP_ADC_RAW_TO_MV(ap_filter.latest),
        ap_filter.filtered);
}
