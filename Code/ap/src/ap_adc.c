/**
  ******************************************************************************
  * @file    ap_adc.c
  * @brief   AP 层 ADC 管理 — DMA 启动 + 原始值读取
  ******************************************************************************
  */

#include "ap_adc.h"
#include <string.h>

#define ADC_FAULT_SAMPLE_INTERVAL_MS    (10U)    /* 100Hz故障监控，不依赖主循环执行频率 */
#define ADC_RAIL_FAULT_CONFIRM_MS        (20000U) /* 单通道连续轨到轨20秒才确认硬件故障 */
#define ADC_RAIL_RECOVER_CONFIRM_MS      (2000U)  /* 三通道连续正常2秒后解除故障 */
#define ADC_12BIT_FULL_SCALE             (4095U)

typedef struct {
    uint32_t rail_start_ms[BSP_ADC_NUM_CHANNELS]; /* 各通道本次连续轨到轨的起点 */
    uint32_t recover_start_ms;                    /* 三通道同时恢复正常的起点 */
    uint32_t last_sample_ms;                      /* 故障监控100Hz节拍基准 */
    uint8_t  rail_active[BSP_ADC_NUM_CHANNELS];   /* 避免tick=0被误作未开始 */
    uint8_t  recover_active;
    uint8_t  fault_active;
} AP_ADC_FaultMonitor_t;

static AP_ADC_FaultMonitor_t s_adc_fault;

void AP_ADC_Init(void)
{
    memset(&s_adc_fault, 0, sizeof(s_adc_fault));
    s_adc_fault.last_sample_ms = HAL_GetTick();
    BSP_ADC_StartDMA();
}

uint16_t AP_ADC_GetLatest(uint32_t ch)
{
    uint16_t raw = 0;
    BSP_ADC_ReadValue(ch, &raw);
    return raw;
}

/**
  * @brief  Detect an ADC input held exactly at either conversion rail.
  *
  * A stable optical background is allowed to produce an unchanged ADC value;
  * only exact 0/full-scale samples are considered abnormal. Each channel is
  * timed independently, and any one channel remaining at a rail for 10 seconds
  * asserts the module fault. Once asserted, all three channels must remain in
  * 1..4094 for 2 seconds before the fault is cleared. Alternating between 0 and
  * 4095 is still a rail fault because the channel never returns to a valid code.
  */
uint8_t AP_ADC_FaultMonitorTask(uint32_t now)
{
    uint8_t all_channels_valid = 1U;

    if ((now - s_adc_fault.last_sample_ms) < ADC_FAULT_SAMPLE_INTERVAL_MS) {
        return s_adc_fault.fault_active;
    }
    s_adc_fault.last_sample_ms = now;

    for (uint32_t ch = 0U; ch < BSP_ADC_NUM_CHANNELS; ch++) {
        uint16_t raw = AP_ADC_GetLatest(ch);
        uint8_t at_rail = (raw == 0U || raw == ADC_12BIT_FULL_SCALE) ? 1U : 0U;

        if (at_rail != 0U) {
            all_channels_valid = 0U;
            if (s_adc_fault.rail_active[ch] == 0U) {
                s_adc_fault.rail_active[ch] = 1U;
                s_adc_fault.rail_start_ms[ch] = now;
            } else if ((now - s_adc_fault.rail_start_ms[ch]) >=
                       ADC_RAIL_FAULT_CONFIRM_MS) {
                s_adc_fault.fault_active = 1U;
            }
        } else {
            s_adc_fault.rail_active[ch] = 0U;
            s_adc_fault.rail_start_ms[ch] = 0U;
        }
    }

    if (s_adc_fault.fault_active == 0U) {
        s_adc_fault.recover_active = 0U;
        s_adc_fault.recover_start_ms = 0U;
        return 0U;
    }

    if (all_channels_valid == 0U) {
        s_adc_fault.recover_active = 0U;
        s_adc_fault.recover_start_ms = 0U;
    } else if (s_adc_fault.recover_active == 0U) {
        s_adc_fault.recover_active = 1U;
        s_adc_fault.recover_start_ms = now;
    } else if ((now - s_adc_fault.recover_start_ms) >=
               ADC_RAIL_RECOVER_CONFIRM_MS) {
        s_adc_fault.fault_active = 0U;
        s_adc_fault.recover_active = 0U;
        s_adc_fault.recover_start_ms = 0U;
    }

    return s_adc_fault.fault_active;
}

uint8_t AP_ADC_IsFault(void)
{
    return s_adc_fault.fault_active;
}
