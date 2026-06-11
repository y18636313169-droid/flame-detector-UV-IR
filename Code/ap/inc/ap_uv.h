/**
  ******************************************************************************
  * @file    ap_uv.h
  * @brief   AP 层紫外火焰检测模块
  *
  *          基于 TIM3 捕获的 C10807 UV 脉冲，利用时间窗口计数和状态机
  *          决策火焰报警。支持 10 级灵敏度动态可调。
  *
  *          实例在模块内部静态定义，外部直接调用 API 即可。
  *
  *          == 使用示例 ==
  *          AP_UV_Init(5, on_fire_alarm);     // 中灵敏度，注册回调
  *          while (1) {
  *              AP_UV_Process(HAL_GetTick());  // 内部自动 Feed
  *              HAL_Delay(20);
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
/*                          灵敏度宏定义                                       */
/* ========================================================================== */

#define UV_SENS_LEVELS          (10U)

#define UV_WINDOW_MS_0          (3500U)
#define UV_WINDOW_MS_9          (1000U)
#define UV_THRESHOLD_0          (35U)
#define UV_THRESHOLD_9          (5U)
#define UV_CONFIRM_MS_0         (3500U)
#define UV_CONFIRM_MS_9         (0U)
#define UV_CLEAR_MS_0           (10000U)
#define UV_CLEAR_MS_9           (3000U)

#define UV_MAX_HISTORY          (20U)

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

void AP_UV_Init(uint8_t level, void (*on_fire)(void));
void AP_UV_Feed(void);
void AP_UV_Process(uint32_t now);
void AP_UV_SetLevel(uint8_t level);
void AP_UV_Reset(void);
UV_DetectorState_t AP_UV_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __AP_UV_H__ */
