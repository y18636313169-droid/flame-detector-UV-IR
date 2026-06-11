/**
  ******************************************************************************
  * @file    hal_iwdg.h
  * @brief   BSP 独立看门狗（IWDG）抽象层
  *
  *          采用 ISR 置标志 + 主循环消费模式：
  *            1. 定时器 ISR 周期性调用 BSP_IWDG_RequestFeed() 置位请求标志
  *            2. 主循环调用 BSP_IWDG_CheckAndRefresh() 检查标志并喂狗
  *
  *          优势：即使主循环偶发阻塞，也能维持固定的喂狗间隔；
  *               若 ISR 或主循环任一挂死，看门狗超时复位。
  *
  *          当前配置（iwdg.c）：
  *              Prescaler = 256, Reload = 1563
  *              LSI ≈ 37kHz → 超时 ≈ 10.8s
  *              喂狗周期建议设置在超时的 1/5 ~ 1/3（约 2~3s）
  *
  *          == 使用示例 ==
  *          MX_IWDG_Init();              // CubeMX 初始化（main.c）
  *
  *          // TIM6 ISR 中每 2000ms：
  *          BSP_IWDG_RequestFeed();
  *
  *          // 主循环中：
  *          BSP_IWDG_CheckAndRefresh();
  ******************************************************************************
  */
#ifndef __BSP_HAL_IWDG_H__
#define __BSP_HAL_IWDG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  ISR 中调用 — 请求喂狗
  *         设置喂狗请求标志，由主循环的 CheckAndRefresh 消费。
  *         TIM6 等定时 ISR 中每 2s 调用一次。
  */
void BSP_IWDG_RequestFeed(void);

/**
  * @brief  主循环中调用 — 检查标志位并喂狗
  *         若 BSP_IWDG_RequestFeed 已置位，则喂狗并清标志。
  *         建议放在主循环末尾，每次迭代调用一次。
  */
void BSP_IWDG_CheckAndRefresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_IWDG_H__ */
