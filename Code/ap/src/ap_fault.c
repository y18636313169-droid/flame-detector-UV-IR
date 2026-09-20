/**
  ******************************************************************************
  * @file    ap_fault.c
  * @brief   集中故障状态及低电平有效BUG输出管理
  *
  *          各故障源只更新位图，由本模块唯一管理BUG引脚，避免一个监视器
  *          在清除自身故障时误清除其他模块仍然存在的故障输出。
  ******************************************************************************
  */

#include "ap_fault.h"

#include "hal_alarm.h"
#include "main.h"

#define AP_CONFIG_STATUS_NEED_CONFIG       (1U << 0)
#define AP_CONFIG_STATUS_DEFAULTS_ACTIVE    (1U << 1)

typedef struct {
    uint32_t bits;                      /* 当前全部故障位 */
    uint32_t last_operational_query_ms; /* 最近合法运行查询时间 */
    uint16_t config_groups;             /* 需要重新配置的参数组 */
    uint8_t config_status;              /* 待配置/默认值恢复状态 */
    uint8_t missed_periods;             /* 连续缺失的30秒查询周期 */
    uint8_t bug_asserted;               /* 当前BUG命令状态，避免重复写GPIO */
    AP_ResetReason_t reset_reason;      /* 本次启动捕获的最近复位原因 */
} AP_FaultState_t;

static AP_FaultState_t s_fault;

/** @brief 启动时读取一次RCC复位标志，转换为协议复位原因后统一清除。 */
static AP_ResetReason_t capture_reset_reason(void)
{
    AP_ResetReason_t reason = AP_RESET_REASON_UNKNOWN;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
        reason = AP_RESET_REASON_WATCHDOG;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) {
        reason = AP_RESET_REASON_SOFTWARE;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET) {
        /* STM32L1只暴露POR/PDR组合标志，无法可靠细分上电与欠压。 */
        reason = AP_RESET_REASON_POWER_ON;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) {
        reason = AP_RESET_REASON_PIN;
    }

    __HAL_RCC_CLEAR_RESET_FLAGS();
    return reason;
}

/** @brief 根据总故障位图更新BUG，仅电平发生变化时操作GPIO。 */
static void update_bug_output(void)
{
    uint8_t asserted = (s_fault.bits != 0U) ? 1U : 0U;

    if (asserted == s_fault.bug_asserted) {
        return;
    }
    if (asserted != 0U) {
        BSP_FAULT_Set();
    } else {
        BSP_FAULT_Reset();
    }
    s_fault.bug_asserted = asserted;
}

void AP_Fault_Init(uint32_t now_ms)
{
    s_fault.bits = 0U;
    s_fault.last_operational_query_ms = now_ms;
    s_fault.config_groups = 0U;
    s_fault.config_status = 0U;
    s_fault.missed_periods = 0U;
    s_fault.bug_asserted = 0U;
    s_fault.reset_reason = capture_reset_reason();

    /* 看门狗复位作为最近一次异常复位记录保留，并同步置当前故障位。 */
    if (s_fault.reset_reason == AP_RESET_REASON_WATCHDOG) {
        s_fault.bits |= AP_FAULT_WATCHDOG_RESET;
    }
    update_bug_output();
}

void AP_Fault_Task(uint32_t now_ms)
{
    uint32_t elapsed = now_ms - s_fault.last_operational_query_ms;
    uint32_t missed = elapsed / AP_FAULT_COMM_QUERY_PERIOD_MS;

    if (missed > AP_FAULT_COMM_MISSED_LIMIT) {
        missed = AP_FAULT_COMM_MISSED_LIMIT;
    }
    s_fault.missed_periods = (uint8_t)missed;
    if (missed >= AP_FAULT_COMM_MISSED_LIMIT) {
        s_fault.bits |= AP_FAULT_COMM_LINK_ERROR;
    }
    update_bug_output();
}

void AP_Fault_Set(uint32_t mask)
{
    s_fault.bits |= mask;
    update_bug_output();
}

void AP_Fault_Clear(uint32_t mask)
{
    s_fault.bits &= ~mask;
    update_bug_output();
}

void AP_Fault_Update(uint32_t mask, uint32_t active_bits)
{
    s_fault.bits = (s_fault.bits & ~mask) | (active_bits & mask);
    update_bug_output();
}

void AP_Fault_NotifyOperationalQuery(uint32_t now_ms)
{
    s_fault.last_operational_query_ms = now_ms;
    s_fault.missed_periods = 0U;
    s_fault.bits &= ~AP_FAULT_COMM_LINK_ERROR;
    update_bug_output();
}

void AP_Fault_SetConfigGroups(uint16_t groups)
{
    s_fault.config_groups |= groups;
    if (s_fault.config_groups != 0U) {
        s_fault.config_status |= AP_CONFIG_STATUS_NEED_CONFIG;
    }
}

void AP_Fault_ClearConfigGroups(uint16_t groups)
{
    s_fault.config_groups &= (uint16_t)~groups;
    if (s_fault.config_groups == 0U) {
        s_fault.config_status &= (uint8_t)~AP_CONFIG_STATUS_NEED_CONFIG;
        s_fault.bits &= ~(AP_FAULT_EEPROM_CRC_ERROR |
                          AP_FAULT_EEPROM_RANGE_ERROR |
                          AP_FAULT_EEPROM_VERSION_ERROR |
                          AP_FAULT_EEPROM_WRITE_ERROR);
        update_bug_output();
    }
}

void AP_Fault_SetDefaultsRecoveryActive(uint8_t active)
{
    if (active != 0U) {
        s_fault.config_status |= AP_CONFIG_STATUS_DEFAULTS_ACTIVE;
    } else {
        s_fault.config_status &= (uint8_t)~AP_CONFIG_STATUS_DEFAULTS_ACTIVE;
    }
}

uint32_t AP_Fault_GetBits(void) { return s_fault.bits; }
uint16_t AP_Fault_GetConfigGroups(void) { return s_fault.config_groups; }
uint8_t AP_Fault_GetConfigStatus(void) { return s_fault.config_status; }
uint8_t AP_Fault_GetMissedPeriods(void) { return s_fault.missed_periods; }
AP_ResetReason_t AP_Fault_GetResetReason(void) { return s_fault.reset_reason; }
