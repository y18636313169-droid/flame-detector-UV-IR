/**
  ******************************************************************************
  * @file    ap_adc.c
  * @brief   ADC DMA启动、原始值读取和三通道采样链故障监视
  ******************************************************************************
  */

#include "ap_adc.h"
#include <string.h>

/*
 * 故障监视采用长时间确认，避免火焰冲击、插拔毛刺和单次DMA抖动误报。
 * 这些阈值只用于硬件健康判断，不参与红外火焰识别算法。
 */
#define ADC_FAULT_SAMPLE_INTERVAL_MS      (10U)
#define ADC_NEAR_RAIL_LOW                 (16U)
#define ADC_NEAR_RAIL_HIGH                (4079U)
#define ADC_RAIL_FAULT_CONFIRM_MS         (20000U)
#define ADC_RAIL_RECOVER_CONFIRM_MS       (2000U)
#define ADC_STUCK_FAULT_CONFIRM_MS        (600000U)
#define ADC_STUCK_DEADBAND                (5U)
#define ADC_STUCK_RECOVER_CONFIRM_MS      (2000U)
#define ADC_STUCK_RECOVER_MIN_CHANGES     (10U)
#define ADC_STUCK_RECOVER_GAP_MS          (200U)
#define ADC_BIAS_LOW                      (1024U)
#define ADC_BIAS_HIGH                     (3071U)
#define ADC_BIAS_AVERAGE_SAMPLES          (100U)
#define ADC_BIAS_FAULT_CONFIRM_MS         (60000U)
#define ADC_BIAS_RECOVER_CONFIRM_MS       (5000U)
#define ADC_DMA_STALL_CONFIRM_MS          (1000U)
#define ADC_DMA_RECOVER_CONFIRM_MS        (2000U)

typedef struct {
    /* 每通道独立计时，避免一路异常影响另外两路的确认和恢复。 */
    uint32_t rail_start_ms[BSP_ADC_NUM_CHANNELS];
    uint32_t rail_recover_ms[BSP_ADC_NUM_CHANNELS];
    uint32_t bias_start_ms[BSP_ADC_NUM_CHANNELS];
    uint32_t bias_recover_ms[BSP_ADC_NUM_CHANNELS];
    uint32_t bias_sum[BSP_ADC_NUM_CHANNELS];
    uint32_t last_change_ms[BSP_ADC_NUM_CHANNELS];
    uint32_t stuck_recover_ms[BSP_ADC_NUM_CHANNELS];
    uint16_t last_raw[BSP_ADC_NUM_CHANNELS];
    uint16_t bias_sample_count[BSP_ADC_NUM_CHANNELS];
    uint8_t stuck_recover_changes[BSP_ADC_NUM_CHANNELS];
    /* 三个bit按ADC通道索引保存正在计时或已经确认的故障。 */
    uint8_t rail_timing_mask;
    uint8_t rail_fault_mask;
    uint8_t bias_timing_mask;
    uint8_t bias_fault_mask;
    uint8_t stuck_fault_mask;
    uint8_t sample_initialized;

    /* DMA计数由ISR递增，主循环只比较进度和错误计数，不读取DMA中间状态。 */
    uint32_t last_sample_ms;
    uint32_t last_dma_progress_ms;
    uint32_t last_dma_update_count;
    uint32_t last_dma_error_count;
    uint32_t dma_recover_ms;
    uint8_t dma_fault;
    uint8_t dma_recover_active;
    uint8_t fault_flags;
} AP_ADC_FaultMonitor_t;

static AP_ADC_FaultMonitor_t s_adc_fault;

static uint8_t channel_bit(uint32_t ch)
{
    return (uint8_t)(1U << ch);
}

int AP_ADC_Init(void)
{
    int result;
    uint32_t now = HAL_GetTick();

    memset(&s_adc_fault, 0, sizeof(s_adc_fault));
    s_adc_fault.last_sample_ms = now;
    s_adc_fault.last_dma_progress_ms = now;
    result = BSP_ADC_StartDMA();
    s_adc_fault.last_dma_update_count = BSP_ADC_GetUpdateCount();
    s_adc_fault.last_dma_error_count = BSP_ADC_GetErrorCount();
    return result;
}

uint16_t AP_ADC_GetLatest(uint32_t ch)
{
    uint16_t raw = 0U;
    (void)BSP_ADC_ReadValue(ch, &raw);
    return raw;
}

/** @brief 对单通道近电源轨状态进行延时确认和延时恢复。 */
static void monitor_rail(uint32_t ch, uint16_t raw, uint32_t now)
{
    uint8_t bit = channel_bit(ch);
    uint8_t abnormal = (raw <= ADC_NEAR_RAIL_LOW ||
                        raw >= ADC_NEAR_RAIL_HIGH) ? 1U : 0U;

    if (abnormal != 0U) {
        s_adc_fault.rail_recover_ms[ch] = 0U;
        if ((s_adc_fault.rail_timing_mask & bit) == 0U) {
            s_adc_fault.rail_timing_mask |= bit;
            s_adc_fault.rail_start_ms[ch] = now;
        } else if ((now - s_adc_fault.rail_start_ms[ch]) >=
                   ADC_RAIL_FAULT_CONFIRM_MS) {
            s_adc_fault.rail_fault_mask |= bit;
        }
        return;
    }

    s_adc_fault.rail_timing_mask &= (uint8_t)~bit;
    s_adc_fault.rail_start_ms[ch] = 0U;
    if ((s_adc_fault.rail_fault_mask & bit) == 0U) {
        s_adc_fault.rail_recover_ms[ch] = 0U;
    } else if (s_adc_fault.rail_recover_ms[ch] == 0U) {
        s_adc_fault.rail_recover_ms[ch] = now;
    } else if ((now - s_adc_fault.rail_recover_ms[ch]) >=
               ADC_RAIL_RECOVER_CONFIRM_MS) {
        s_adc_fault.rail_fault_mask &= (uint8_t)~bit;
        s_adc_fault.rail_recover_ms[ch] = 0U;
    }
}

/**
  * @brief 确认通道是否长期冻结，并对恢复过程去抖。
  * @note  原始值相对冻结基准的偏差不超过5码时仍视为未变化，避免ADC末位噪声
  *        持续刷新计时；只有越过死区的变化才更新基准并作为恢复证据。
  */
static void monitor_stuck(uint32_t ch, uint16_t raw, uint32_t now)
{
    uint8_t bit = channel_bit(ch);
    uint16_t delta = (raw >= s_adc_fault.last_raw[ch])
                   ? (uint16_t)(raw - s_adc_fault.last_raw[ch])
                   : (uint16_t)(s_adc_fault.last_raw[ch] - raw);

    if (delta > ADC_STUCK_DEADBAND) {
        s_adc_fault.last_raw[ch] = raw;
        s_adc_fault.last_change_ms[ch] = now;
        if ((s_adc_fault.stuck_fault_mask & bit) != 0U) {
            if (s_adc_fault.stuck_recover_ms[ch] == 0U) {
                s_adc_fault.stuck_recover_ms[ch] = now;
                s_adc_fault.stuck_recover_changes[ch] = 1U;
            } else if (s_adc_fault.stuck_recover_changes[ch] < 0xFFU) {
                s_adc_fault.stuck_recover_changes[ch]++;
            }
            if ((now - s_adc_fault.stuck_recover_ms[ch]) >=
                    ADC_STUCK_RECOVER_CONFIRM_MS &&
                s_adc_fault.stuck_recover_changes[ch] >=
                    ADC_STUCK_RECOVER_MIN_CHANGES) {
                s_adc_fault.stuck_fault_mask &= (uint8_t)~bit;
                s_adc_fault.stuck_recover_ms[ch] = 0U;
                s_adc_fault.stuck_recover_changes[ch] = 0U;
            }
        }
        return;
    }

    if ((s_adc_fault.stuck_fault_mask & bit) == 0U) {
        if ((now - s_adc_fault.last_change_ms[ch]) >=
            ADC_STUCK_FAULT_CONFIRM_MS) {
            s_adc_fault.stuck_fault_mask |= bit;
        }
    } else if (s_adc_fault.stuck_recover_ms[ch] != 0U &&
               (now - s_adc_fault.last_change_ms[ch]) >=
               ADC_STUCK_RECOVER_GAP_MS) {
        /* 恢复期再次长时间不变化，必须重新累计有效变化。 */
        s_adc_fault.stuck_recover_ms[ch] = 0U;
        s_adc_fault.stuck_recover_changes[ch] = 0U;
    }
}

/** @brief 使用1秒原始数据均值确认直流偏置是否长期明显偏离标称1.65V中点。 */
static void monitor_bias(uint32_t ch, uint16_t mean, uint32_t now)
{
    uint8_t bit = channel_bit(ch);
    uint8_t abnormal = (mean < ADC_BIAS_LOW || mean > ADC_BIAS_HIGH) ? 1U : 0U;

    if (abnormal != 0U) {
        s_adc_fault.bias_recover_ms[ch] = 0U;
        if ((s_adc_fault.bias_timing_mask & bit) == 0U) {
            s_adc_fault.bias_timing_mask |= bit;
            s_adc_fault.bias_start_ms[ch] = now;
        } else if ((now - s_adc_fault.bias_start_ms[ch]) >=
                   ADC_BIAS_FAULT_CONFIRM_MS) {
            s_adc_fault.bias_fault_mask |= bit;
        }
        return;
    }

    s_adc_fault.bias_timing_mask &= (uint8_t)~bit;
    s_adc_fault.bias_start_ms[ch] = 0U;
    if ((s_adc_fault.bias_fault_mask & bit) == 0U) {
        s_adc_fault.bias_recover_ms[ch] = 0U;
    } else if (s_adc_fault.bias_recover_ms[ch] == 0U) {
        s_adc_fault.bias_recover_ms[ch] = now;
    } else if ((now - s_adc_fault.bias_recover_ms[ch]) >=
               ADC_BIAS_RECOVER_CONFIRM_MS) {
        s_adc_fault.bias_fault_mask &= (uint8_t)~bit;
        s_adc_fault.bias_recover_ms[ch] = 0U;
    }
}

/**
  * @brief 累计单通道偏置均值，满1秒样本后更新一次偏置故障状态。
  * @note  火焰交流分量会使瞬时ADC值大幅摆动，使用均值可提取前端直流工作点，
  *        避免把正常火焰波动误判为偏置电压异常。
  */
static void accumulate_bias(uint32_t ch, uint16_t raw, uint32_t now)
{
    s_adc_fault.bias_sum[ch] += raw;
    s_adc_fault.bias_sample_count[ch]++;
    if (s_adc_fault.bias_sample_count[ch] >= ADC_BIAS_AVERAGE_SAMPLES) {
        uint16_t mean = (uint16_t)(s_adc_fault.bias_sum[ch] /
                                   s_adc_fault.bias_sample_count[ch]);

        s_adc_fault.bias_sum[ch] = 0U;
        s_adc_fault.bias_sample_count[ch] = 0U;
        monitor_bias(ch, mean, now);
    }
}

/**
  * @brief 暂停火警期间的偏置检测并丢弃未完成窗口。
  * @note  已确认故障保持不变；仅取消未完成的确认或恢复计时，待消警后重新采集
  *        完整均值窗口，防止火警期间的数据参与硬件静态偏置判断。
  */
static void pause_bias_monitor(void)
{
    memset(s_adc_fault.bias_sum, 0, sizeof(s_adc_fault.bias_sum));
    memset(s_adc_fault.bias_sample_count, 0,
           sizeof(s_adc_fault.bias_sample_count));
    memset(s_adc_fault.bias_start_ms, 0, sizeof(s_adc_fault.bias_start_ms));
    memset(s_adc_fault.bias_recover_ms, 0,
           sizeof(s_adc_fault.bias_recover_ms));
    s_adc_fault.bias_timing_mask = 0U;
}

/** @brief 使用DMA完成计数和错误计数判断采集缓冲是否停止更新。 */
static void monitor_dma(uint32_t now)
{
    uint32_t updates = BSP_ADC_GetUpdateCount();
    uint32_t errors = BSP_ADC_GetErrorCount();
    uint8_t progressed = (updates != s_adc_fault.last_dma_update_count) ? 1U : 0U;
    uint8_t new_error = (errors != s_adc_fault.last_dma_error_count) ? 1U : 0U;

    s_adc_fault.last_dma_update_count = updates;
    s_adc_fault.last_dma_error_count = errors;
    if (progressed != 0U) {
        s_adc_fault.last_dma_progress_ms = now;
    }

    if (BSP_ADC_IsRunning() == 0U || new_error != 0U ||
        (now - s_adc_fault.last_dma_progress_ms) >= ADC_DMA_STALL_CONFIRM_MS) {
        s_adc_fault.dma_fault = 1U;
        s_adc_fault.dma_recover_active = 0U;
        s_adc_fault.dma_recover_ms = 0U;
        return;
    }

    if (s_adc_fault.dma_fault == 0U || progressed == 0U) {
        return;
    }
    if (s_adc_fault.dma_recover_active == 0U) {
        s_adc_fault.dma_recover_active = 1U;
        s_adc_fault.dma_recover_ms = now;
    } else if ((now - s_adc_fault.dma_recover_ms) >=
               ADC_DMA_RECOVER_CONFIRM_MS) {
        s_adc_fault.dma_fault = 0U;
        s_adc_fault.dma_recover_active = 0U;
        s_adc_fault.dma_recover_ms = 0U;
    }
}

uint8_t AP_ADC_FaultMonitorTask(uint32_t now, uint8_t fire_active)
{
    uint8_t read_ok = 1U;

    if ((now - s_adc_fault.last_sample_ms) < ADC_FAULT_SAMPLE_INTERVAL_MS) {
        return (s_adc_fault.rail_fault_mask != 0U ||
                s_adc_fault.fault_flags != 0U) ? 1U : 0U;
    }
    s_adc_fault.last_sample_ms = now;
    monitor_dma(now);

    for (uint32_t ch = 0U; ch < BSP_ADC_NUM_CHANNELS; ch++) {
        uint16_t raw;

        if (BSP_ADC_ReadValue(ch, &raw) != BSP_ADC_OK) {
            read_ok = 0U;
            continue;
        }
        if (s_adc_fault.sample_initialized == 0U) {
            s_adc_fault.last_raw[ch] = raw;
            s_adc_fault.last_change_ms[ch] = now;
        }
        monitor_rail(ch, raw, now);
        monitor_stuck(ch, raw, now);
        if (fire_active == 0U) {
            accumulate_bias(ch, raw, now);
        }
    }
    s_adc_fault.sample_initialized = 1U;

    if (fire_active != 0U) {
        pause_bias_monitor();
    }

    /* DMA已报告运行但读取失败同样属于采集链异常，不能只依赖进度计数。 */
    if (read_ok == 0U) {
        s_adc_fault.dma_fault = 1U;
    }

    s_adc_fault.fault_flags = 0U;
    if (s_adc_fault.stuck_fault_mask != 0U) {
        s_adc_fault.fault_flags |= AP_ADC_FAULT_DATA_STUCK;
    }
    if (s_adc_fault.bias_fault_mask != 0U) {
        s_adc_fault.fault_flags |= AP_ADC_FAULT_BIAS;
    }
    if (s_adc_fault.dma_fault != 0U) {
        s_adc_fault.fault_flags |= AP_ADC_FAULT_DMA;
    }
    return (s_adc_fault.rail_fault_mask != 0U ||
            s_adc_fault.fault_flags != 0U) ? 1U : 0U;
}

uint8_t AP_ADC_IsFault(void)
{
    return (s_adc_fault.rail_fault_mask != 0U ||
            s_adc_fault.fault_flags != 0U) ? 1U : 0U;
}

uint8_t AP_ADC_GetRailFaultMask(void)
{
    return s_adc_fault.rail_fault_mask;
}

uint8_t AP_ADC_GetFaultFlags(void)
{
    return s_adc_fault.fault_flags;
}
