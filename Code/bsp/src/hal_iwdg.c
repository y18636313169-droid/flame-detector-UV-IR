/**
  ******************************************************************************
  * @file    hal_iwdg.c
  * @brief   BSP 独立看门狗抽象层实现
  *
  *          使用 volatile 标志位实现 ISR→主循环的喂狗请求传递：
  *            - ISR 调用 BSP_IWDG_RequestFeed() 置位
  *            - 主循环调用 BSP_IWDG_CheckAndRefresh() 消费
  *
  *          使用前提：
  *              在 main.c 中先调用 MX_IWDG_Init() 完成 IWDG 初始化，
  *              之后方可正常工作。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hal_iwdg.h"
#include "iwdg.h"               /* extern hiwdg */

/* Private variables ---------------------------------------------------------*/

static volatile uint8_t s_feed_request = 0;    /* ISR→主循环喂狗请求标志 */

/* ---------------------------------------------------------------------------*/
/*                        公有 API 实现                                        */
/* ---------------------------------------------------------------------------*/

void BSP_IWDG_RequestFeed(void)
{
    s_feed_request = 1;
}

void BSP_IWDG_CheckAndRefresh(void)
{
    if (s_feed_request) {
        s_feed_request = 0;
        HAL_IWDG_Refresh(&hiwdg);
    }
}
