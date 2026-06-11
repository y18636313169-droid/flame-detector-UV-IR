/**
  ******************************************************************************
  * @file    hal_tim.h
  * @brief   BSP 定时器抽象层头文件 — 单通道脉宽捕获 + 环形缓冲输出
  *
  *          使用 TIM 单通道 + 手动极性切换测量 UV TRON 脉冲高电平宽度。
  *          捕获完成的数据通过环形缓冲区传递给主循环，ISR 只写不读。
  *
  *          == 测量原理 ==
  *          1. CH1 上升沿捕获 → 停止 TIM，清零 CNT，切为下降沿，重新启动
  *          2. 期间每溢出一次 overflow_cnt++
  *          3. CH1 下降沿捕获 → 计算总时长 → 写入环形缓冲区 → 切回上升沿
  *          4. 主循环轮询读取缓冲区
  *
  *          == 主循环调用示例 ==
  *
  *          BSP_TIM_IC_Init(BSP_TIM_UV);
  *          BSP_TIM_IC_Start(BSP_TIM_UV);
  *
  *          while (1) {
  *              BSP_TIM_PulseData_t pulse;
  *              if (BSP_TIM_IC_ReadPulse(BSP_TIM_UV, &pulse) == 0) {
  *                  if (pulse.pulse_width_us >= BSP_TIM_UV_PW_MIN_US &&
  *                      pulse.pulse_width_us <= BSP_TIM_UV_PW_MAX_US) {
  *                      // C10807 有效火焰脉冲
  *                  }
  *              }
  *          }
  *
  *          == 全部 API ==
  *          BSP_TIM_IC_Init(id)          — 初始化（绑定 HAL 句柄）
  *          BSP_TIM_IC_Start(id)         — 启动捕获
  *          BSP_TIM_IC_Stop(id)          — 停止捕获
  *          BSP_TIM_IC_ReadPulse(id, &d) — 读取脉冲数据（轮询）
  *          BSP_TIM_IC_GetState(id)      — 查询状态机（调试）
  *          BSP_TIM_IC_GetTickUs(id)     — 每 tick 微秒数
  *          BSP_TIM_IC_GetHandle(id)     — 底层 HAL 句柄
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
  * @brief  UV 脉冲有效判定门限
  *         C10807 典型脉宽 ~10ms，有效窗口 6~14ms
  *         电焊/闪电干扰脉冲通常 <1ms
  */
#define BSP_TIM_UV_PW_MIN_US    (6000U)
#define BSP_TIM_UV_PW_MAX_US    (14000U)

/**
  * @brief  环形缓冲池大小（元素个数）
  *         C10807 最大输出 ~100Hz，应用处理一个脉冲约 10ms，
  *         20 个槽位足够缓冲瞬态峰值。
  */
#define BSP_TIM_UV_PULSE_POOL_SIZE  (20U)

/* ========================================================================== */
/*                     TIM 实例映射                                            */
/* ========================================================================== */

#define BSP_TIM_UV_INST         TIM3      /* CubeMX 重新生成后改为 TIM3 (PA6/PA7) */
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
    uint32_t    pulse_width_us;     /* 高电平脉宽（微秒），已包含溢出展开 */
    uint32_t    timestamp_ms;       /* 捕获完成时的系统滴答（ms）        */
} BSP_TIM_PulseData_t;

/**
  * @brief  捕获状态机枚举
  */
typedef enum {
    CAPTURE_IDLE = 0,               /* 等待上升沿                    */
    CAPTURE_RISING_EDGE,            /* 已捕获上升沿，等待下降沿      */
    CAPTURE_COMPLETE                /* 下降沿已捕获，数据已入缓冲    */
} BSP_TIM_CaptureState_t;

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
  * @brief  批量读取所有脉冲数据（非阻塞）
  * @param  id:       TIM 端口 ID
  * @param  pulses:   输出数组
  * @param  max:      数组最大容量
  * @retval 实际读取到的脉冲数量（0 = 缓冲空）
  */
uint16_t BSP_TIM_IC_ReadAllPulse(BSP_TIM_Id_t id, BSP_TIM_PulseData_t *pulses, uint16_t max);
/*
BSP_TIM_CaptureState_t BSP_TIM_IC_GetState(BSP_TIM_Id_t id);
float BSP_TIM_IC_GetTickUs(BSP_TIM_Id_t id);
void *BSP_TIM_IC_GetHandle(BSP_TIM_Id_t id);*/

/* ========================================================================== */
/*     内部转发接口 — 由 main.c 中的 HAL 弱回调调用                            */
/* ========================================================================== */

/**
  * @brief  TIM 输入捕获事件处理（由 HAL_TIM_IC_CaptureCallback 转发）
  *         内含 UV 脉冲测量状态机，被 main.c 调用。
  * @param  htim: HAL TIM 句柄
  */
void BSP_TIM_IC_CaptureHandler(TIM_HandleTypeDef *htim);

/**
  * @brief  TIM 周期更新事件处理（由 HAL_TIM_PeriodElapsedCallback 转发）
  *         内含 UV 脉冲溢出计数，被 main.c 调用。
  *         TIM5 的更新事件已在 main.c 中直接分发给 BSP_LED_TickHandler，
  *         此函数仅处理 TIM3 等非 LED 定时器的溢出。
  * @param  htim: HAL TIM 句柄
  */
void BSP_TIM_IC_PeriodHandler(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_TIM_H__ */
