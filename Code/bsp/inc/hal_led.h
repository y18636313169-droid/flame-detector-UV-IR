/**
  ******************************************************************************
  * @file    hal_led.h
  * @brief   BSP LED 驱动头文件
  *
  *          三种模式（互斥，最后调用的生效）：
  *            Blink : 闪烁 N 次后自动停止
  *            Work  : 一直以固定间隔翻转（心跳灯）
  *            Stop  : 停止所有闪烁，熄灭 LED
  *
  *          底层由 TIM5 更新中断驱动，非阻塞。
  *          需要在 main 开头调用 MX_TIM5_Init() 完成定时器初始化。
  ******************************************************************************
  */
#ifndef __BSP_HAL_LED_H__
#define __BSP_HAL_LED_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

/** @brief  系统指示灯引脚（CubeMX 定义在 main.h 中） */
#define BSP_LED_PIN         LED_Pin             /* PC6      */
#define BSP_LED_PORT        LED_GPIO_Port       /* GPIOC    */
#define BSP_LED_ON_LEVEL    GPIO_PIN_RESET      /* 低电平点亮      */
#define BSP_LED_OFF_LEVEL   GPIO_PIN_SET        /* 高电平熄灭      */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  点亮系统 LED
  */
static inline void BSP_LED_On(void)
{
    HAL_GPIO_WritePin(BSP_LED_PORT, BSP_LED_PIN, BSP_LED_ON_LEVEL);
}

/**
  * @brief  熄灭系统 LED
  */
static inline void BSP_LED_Off(void)
{
    HAL_GPIO_WritePin(BSP_LED_PORT, BSP_LED_PIN, BSP_LED_OFF_LEVEL);
}

/**
  * @brief  翻转系统 LED
  */
static inline void BSP_LED_Toggle(void)
{
    HAL_GPIO_TogglePin(BSP_LED_PORT, BSP_LED_PIN);
}

/**
  * @brief  LED 闪烁 N 次（非阻塞）
  * @param  count:       闪烁次数
  * @param  interval_ms: 闪烁间隔（ms），最小 10ms
  * @note   启动后立即返回，TIM5 ISR 完成后台翻转。
  *         闪烁期间调用 BSP_LED_Work() 或再次调用 Blink 会覆盖当前模式。
  */
void BSP_LED_Blink(uint32_t count, uint32_t interval_ms);

/**
  * @brief  LED 持续工作（心跳灯模式，非阻塞）
  * @param  interval_ms: 翻转间隔（ms），最小 10ms
  * @note   一直以固定间隔翻转，直至调用 BSP_LED_Stop()。
  *         调用 BSP_LED_Blink() 会覆盖此模式。
  */
void BSP_LED_Work(uint32_t interval_ms);

/**
  * @brief  停止所有 LED 闪烁/工作，熄灭 LED
  */
void BSP_LED_Stop(void);

/**
  * @brief  LED 滴答处理函数 — 由 HAL_TIM_PeriodElapsedCallback 分发调用
  * @note   ISR 上下文，必须极短。自动根据当前模式决定行为。
  */
void BSP_LED_TickHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_LED_H__ */
