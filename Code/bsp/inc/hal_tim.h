/**
  ******************************************************************************
  * @file    hal_tim.h
  * @brief   BSP 定时器抽象层头文件 — 双通道独立捕获 + 环形缓冲输出
  *
  *          使用 TIM3 双通道分别捕获 C10807 UV TRON 不规则脉冲的上升沿和
  *          下降沿（无从模式复位，定时器自由运行）：
  *            - CH1 (PA6, 上升沿)：记录上升沿 CNT 值
  *            - CH2 (PA6, 下降沿)：记录下降沿 CNT 值并配对计算脉宽
  *
  *          == 测量原理 ==
  *          1. 上升沿 ISR：CCR1 → last_rising_cnt
  *          2. 下降沿 ISR：CCR2 与 last_rising_cnt 差值（处理 16-bit 回绕）→ 脉宽
  *          3. 有效脉宽（6~14ms）入环形缓冲
  *
  *          == ISR 调用链 ==
  *          TIM3_IRQHandler → HAL_TIM_IRQHandler
  *            ├── HAL_TIM_IC_CaptureCallback(htim) → BSP_TIM_IC_CaptureHandler(htim)
  *            │     ├── CH1 → 存 last_rising_cnt
  *            │     └── CH2 → 算脉宽入环
  *            └── HAL_TIM_PeriodElapsedCallback(htim) → BSP_TIM_IC_PeriodHandler(htim)
  *
  *          == 主循环调用示例 ==
  *          BSP_TIM_IC_Init(BSP_TIM_UV);
  *          // Start 由 Init 内部自动调用
  *          while (1) {
  *              BSP_TIM_PulseData_t d;
  *              if (BSP_TIM_IC_ReadPulse(BSP_TIM_UV, &d) == 0) {
  *                  // d.pulse_width_us = 高电平时长(μs)
  *                  // d.timestamp_ms   = 上升沿 HAL_GetTick() 时间戳
  *              }
  *          }
  ******************************************************************************
  */
#ifndef __BSP_HAL_TIM_H__
#define __BSP_HAL_TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ring_buffer.h"
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

#define BSP_TIM_OK          (0)
#define BSP_TIM_ERROR       (-1)
#define BSP_TIM_BUSY        (-2)

/**
  * @brief  UV 脉冲有效判定门限默认值
  *         C10807 典型脉宽 ~10ms，有效窗口 6~14ms
  *         可通过 BSP_TIM_IC_SetPulseRange 运行时修改，
  *         或通过 EEPROM 配置（参数重启后生效）。
  */
#define BSP_TIM_UV_PW_MIN_US_DEFAULT    (6000U)
#define BSP_TIM_UV_PW_MAX_US_DEFAULT    (14000U)

/**
  * @brief  环形缓冲池大小（元素个数）
  *         C10807 最大输出 ~100Hz，应用处理一个脉冲约 10ms，
  *         20 个槽位足够缓冲瞬态峰值。
  */
#define BSP_TIM_UV_PULSE_POOL_SIZE  (20U)

/* ========================================================================== */
/*                     TIM 实例映射                                            */
/* ========================================================================== */

#define BSP_TIM_UV_INST         TIM3
#define BSP_TIM_UV_CLOCK_HZ     (32000000UL)

/* Exported types ------------------------------------------------------------*/

typedef enum {
    BSP_TIM_UV = 0,
    BSP_TIM_NUM
} BSP_TIM_Id_t;

/**
  * @brief  脉冲数据 — 每次下降沿捕获产生一条
  */
typedef struct {
    uint32_t    pulse_width_us;     /* 高电平脉宽（微秒），已包含 16-bit 回绕补偿 */
    uint32_t    timestamp_ms;       /* 上升沿捕获时的系统滴答（ms）               */
} BSP_TIM_PulseData_t;

/* Exported functions --------------------------------------------------------*/

int  BSP_TIM_IC_Init(BSP_TIM_Id_t id);
int  BSP_TIM_IC_Start(BSP_TIM_Id_t id);
int  BSP_TIM_IC_Stop(BSP_TIM_Id_t id);

/**
  * @brief  读取一个脉冲数据（非阻塞）
  * @param  id:    TIM 端口 ID
  * @param  pulse: 输出脉冲数据指针
  * @retval 0: 读取成功
  * @retval -1: 环形缓冲区空
  */
int  BSP_TIM_IC_ReadPulse(BSP_TIM_Id_t id, BSP_TIM_PulseData_t *pulse);

/**
  * @brief  清空 TIM 脉冲环形缓冲区（丢弃所有未读脉冲）
  *         用于状态复位时防止旧脉冲数据影响新判定周期。
  * @param  id: TIM 端口 ID
  * @retval 0: 成功
  * @retval -1: 无效 ID 或未初始化
  */
int  BSP_TIM_IC_ClearAllPulse(BSP_TIM_Id_t id);

/**
  * @brief  批量读取所有脉冲数据（非阻塞）
  * @param  id:       TIM 端口 ID
  * @param  pulses:   输出数组
  * @param  max:    数组最大容量
  * @retval 实际读取到的脉冲数量（0 = 缓冲空）
  */
uint16_t BSP_TIM_IC_ReadAllPulse(BSP_TIM_Id_t id, BSP_TIM_PulseData_t *pulses, uint16_t max);

/**
  * @brief  设置脉冲宽度有效范围
  *         ISR 将只把在此范围内的脉冲写入环形缓冲
  * @param  min_us: 最小脉宽(µs)
  * @param  max_us: 最大脉宽(µs)
  */
void BSP_TIM_IC_SetPulseRange(uint16_t min_us, uint16_t max_us);

/**
  * @brief  读取窗口内的脉冲（peek 不 pop）+ 清理超时脉冲
  *         在 window_ms 范围内的脉冲拷贝到输出数组（不移除），
  *         超过 window_ms 的脉冲从环形缓冲中移除。
  * @param  id:        TIM 端口 ID
  * @param  now_ms:    当前时间戳 (HAL_GetTick)
  * @param  window_ms: 时间窗口(ms)
  * @param  pulses:    输出数组
  * @param  max:       数组最大容量
  * @retval 窗口内脉冲数量（0 = 无有效脉冲）
  */
uint16_t BSP_TIM_IC_ReadWindow(BSP_TIM_Id_t id, uint32_t now_ms,
                                uint32_t window_ms, BSP_TIM_PulseData_t *pulses,
                                uint16_t max);


/* ========================================================================== */
/*     内部转发接口 — 由 main.c 中的 HAL 弱回调调用                            */
/* ========================================================================== */

/**
  * @brief  TIM 输入捕获事件处理（由 HAL_TIM_IC_CaptureCallback 转发）
  *         根据 htim->Channel 区分 CH1（记录上升沿）和 CH2（配对计算脉宽），
  *         被 main.c 调用。
  * @param  htim: HAL TIM 句柄
  */
void BSP_TIM_IC_CaptureHandler(TIM_HandleTypeDef *htim);

/**
  * @brief  TIM 周期更新事件处理（由 HAL_TIM_PeriodElapsedCallback 转发）
  *         内含 UV 脉冲溢出计数（仅扩展用途）。
  *         TIM5/TIM6 的更新事件已在 main.c 中直接分发，此函数仅处理 TIM3 溢出。
  * @param  htim: HAL TIM 句柄
  */
void BSP_TIM_IC_PeriodHandler(TIM_HandleTypeDef *htim);


// /**
//   * @brief  捕获状态机枚举
//   */
// typedef enum {
//     CAPTURE_IDLE = 0,               /* 等待上升沿                    */
//     CAPTURE_RISING_EDGE,            /* 已捕获上升沿，等待下降沿      */
//     CAPTURE_COMPLETE                /* 下降沿已捕获，数据已入缓冲    */
// } BSP_TIM_CaptureState_t;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_TIM_H__ */
