/**
  ******************************************************************************
  * @file    ap_fault.h
  * @brief   MCU故障位图、复位原因及树莓派通信链路监视
  ******************************************************************************
  */
#ifndef __AP_FAULT_H__
#define __AP_FAULT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define AP_FAULT_COMM_QUERY_PERIOD_MS       (30000UL)
#define AP_FAULT_COMM_MISSED_LIMIT          (3U)

typedef enum {
    AP_FAULT_EEPROM_CRC_ERROR       = (1UL << 0),
    AP_FAULT_EEPROM_RANGE_ERROR     = (1UL << 1),
    AP_FAULT_EEPROM_VERSION_ERROR   = (1UL << 2),
    AP_FAULT_EEPROM_WRITE_ERROR     = (1UL << 3),
    AP_FAULT_ADC_CH38_RANGE_ERROR   = (1UL << 4),
    AP_FAULT_ADC_CH45_RANGE_ERROR   = (1UL << 5),
    AP_FAULT_ADC_CH50_RANGE_ERROR   = (1UL << 6),
    AP_FAULT_ADC_DATA_STUCK         = (1UL << 7),
    AP_FAULT_ADC_BIAS_ERROR         = (1UL << 8),
    AP_FAULT_ADC_INIT_ERROR         = (1UL << 9),
    AP_FAULT_ADC_DMA_ERROR          = (1UL << 10),
    AP_FAULT_VREF_ERROR             = (1UL << 11),
    AP_FAULT_UV_INTERFACE_ERROR     = (1UL << 12),
    AP_FAULT_COMM_LINK_ERROR        = (1UL << 13),
    AP_FAULT_WATCHDOG_RESET         = (1UL << 14),
    AP_FAULT_ADC_TASK_TIMEOUT       = (1UL << 15),
    AP_FAULT_ALGORITHM_TASK_TIMEOUT = (1UL << 16),
    AP_FAULT_MAIN_LOOP_TIMEOUT      = (1UL << 17),
} AP_FaultBit_t;

typedef enum {
    AP_CONFIG_GROUP_SYSTEM = (1U << 0),
    AP_CONFIG_GROUP_ADC    = (1U << 1),
    AP_CONFIG_GROUP_UV     = (1U << 2),
    AP_CONFIG_GROUP_IR     = (1U << 3),
} AP_ConfigGroupBit_t;

typedef enum {
    AP_RESET_REASON_UNKNOWN = 0,
    AP_RESET_REASON_POWER_ON,
    AP_RESET_REASON_PIN,
    AP_RESET_REASON_SOFTWARE,
    AP_RESET_REASON_WATCHDOG,
    AP_RESET_REASON_BROWNOUT,
} AP_ResetReason_t;

/** @brief 保存并清除复位标志，初始化通信监视和低有效BUG输出。 */
void AP_Fault_Init(uint32_t now_ms);

/** @brief 更新30秒树莓派查询监视，并根据总故障位维护BUG输出。 */
void AP_Fault_Task(uint32_t now_ms);

/** @brief 设置、清除或按掩码更新一个或多个当前故障位。 */
void AP_Fault_Set(uint32_t mask);
void AP_Fault_Clear(uint32_t mask);
void AP_Fault_Update(uint32_t mask, uint32_t active_bits);

/** @brief 记录树莓派发来的合法运行状态查询，并恢复通信链路状态。 */
void AP_Fault_NotifyOperationalQuery(uint32_t now_ms);

/** @brief 设置或清除需要树莓派重新配置的参数组。 */
void AP_Fault_SetConfigGroups(uint16_t groups);
void AP_Fault_ClearConfigGroups(uint16_t groups);

/** @brief 标记当前是否正在把异常参数组恢复为编译时默认值。 */
void AP_Fault_SetDefaultsRecoveryActive(uint8_t active);

/** @brief 获取当前故障位、待重配参数组及通信监视状态。 */
uint32_t AP_Fault_GetBits(void);
uint16_t AP_Fault_GetConfigGroups(void);
uint8_t AP_Fault_GetConfigStatus(void);
uint8_t AP_Fault_GetMissedPeriods(void);
AP_ResetReason_t AP_Fault_GetResetReason(void);

#ifdef __cplusplus
}
#endif

#endif /* __AP_FAULT_H__ */
