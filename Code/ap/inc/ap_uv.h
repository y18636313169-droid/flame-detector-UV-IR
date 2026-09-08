/**
  ******************************************************************************
  * @file    ap_uv.h
  * @brief   AP 层紫外火焰检测模块
  *
  *          基于 TIM3 捕获的 C10807 UV 脉冲，利用时间窗口计数和状态机
  *          决策火焰报警。
  *
  *          参数通过 min/max 范围 + 灵敏度等级线性插值计算：
  *            level=0 → 使用各参数的 min 值，最灵敏
  *            level=9 → 使用各参数的 max 值，最迟钝
  *            中间等级线性插值
  *
  *          == 使用示例 ==
  *          AP_UV_Init(on_fire_alarm);
  *          AP_UV_SetConfig(5,24, 1000,3500, 0,3500);
  *          AP_UV_SetLevel(5);  // 自动计算各参数
  *          while (1) {
  *              AP_UV_Process(HAL_GetTick());
  *          }
  ******************************************************************************
  */
#ifndef __AP_UV_H__
#define __AP_UV_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                          常量宏                                             */
/* ========================================================================== */

#define UV_SENS_LEVELS          (10U)
#define UV_MAX_HISTORY          (64U)
#define UV_DROP_OFF_PERCENT     (80U)    /* WARNING掉线下限占当前阈值的比例 */
#define UV_DROPOUT_MS           (2000U)  /* 连续低于掉线下限的允许时间 */

/* ========================================================================== */
/*                          状态枚举                                           */
/* ========================================================================== */

typedef enum {
    UV_STATE_IDLE = 0,
    UV_STATE_WARNING,
    UV_STATE_FIRE
} UV_DetectorState_t;

/* ========================================================================== */
/*                          API 声明                                           */
/* ========================================================================== */

void AP_UV_Init(void (*on_fire)(void));
void AP_UV_Feed(void);
void AP_UV_Process(uint32_t now);
void AP_UV_Task(void);

/**
  * @brief  只清除底层捕获队列和上层脉冲历史，不修改当前报警状态。
  * @note   修改脉宽过滤范围后调用，防止旧范围接收的脉冲参与新窗口判定。
  */
void AP_UV_ClearPulseHistory(void);

/**
  * @brief  设置 3 项检测参数的 min/max 范围（即等级 0 和等级 9 的值）
  *         供初始化或命令行修改 min/max 后调用。
  *         设置后需调用 AP_UV_SetLevel 才能生效。
  */
void AP_UV_SetConfig(uint32_t thr_min, uint32_t thr_max,
                     uint32_t win_min, uint32_t win_max,
                     uint32_t cfm_min, uint32_t cfm_max);

/**
  * @brief  获取当前配置的 min/max 范围
  */
void AP_UV_GetConfig(uint32_t *thr_min, uint32_t *thr_max,
                     uint32_t *win_min, uint32_t *win_max,
                     uint32_t *cfm_min, uint32_t *cfm_max);

/**
  * @brief  设置灵敏度等级并自动线性插值计算各检测参数
  * @param  level: 0~9，0最灵敏，9最迟钝
  */
void AP_UV_SetLevel(uint8_t level);

void AP_UV_SetPrintWindow(uint32_t ms);
uint32_t AP_UV_GetPrintWindow(void);

/**
  * @brief  测试模式：打印过去 print_window_ms 内的 UV 脉冲
  *         (脉宽列表，超时的脉冲会被自动移除)
  * @param  now: HAL_GetTick()
  */
void AP_UV_PrintData(uint32_t now);

void AP_UV_Process_Reset(void);
UV_DetectorState_t AP_UV_GetState(void);

/**
  * @brief  获取当前运行的检测参数
  */
void AP_UV_GetParams(uint32_t *threshold, uint32_t *window_ms,
                     uint32_t *confirm_ms, uint32_t *fire_timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __AP_UV_H__ */
