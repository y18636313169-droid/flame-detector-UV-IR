/**
  ******************************************************************************
  * @file    hal_alarm.h
  * @brief   BSP 报警输出驱动头文件
  *
  *          控制 ALM1(PA1)、ALM2(PA0) 输出引脚。
  *          报警输出低电平有效（内部上拉，默认高电平无报警）。
  *
  *          == 使用示例 ==
  *          BSP_ALARM_Init();     // 初始化，默认无报警
  *          BSP_ALARM_Set();      // 火警 → ALM1+ALM2 输出低
  *          BSP_ALARM_Reset();    // 消警 → ALM1+ALM2 恢复高
  *          BSP_ALARM_SetSingle(1);   // 仅 ALM1 报警
  *          BSP_ALARM_ResetSingle(2); // 仅 ALM2 消警
  ******************************************************************************
  */
#ifndef __BSP_HAL_ALARM_H__
#define __BSP_HAL_ALARM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

/* GPIO 引脚映射（宏来自 CubeMX main.h） */
#define BSP_ALM1_PIN        ALM1_Pin          /* PA1 */
#define BSP_ALM1_PORT       ALM1_GPIO_Port    /* GPIOA */
#define BSP_ALM2_PIN        ALM2_Pin          /* PA0 */
#define BSP_ALM2_PORT       ALM2_GPIO_Port    /* GPIOA */

/** @brief  报警输出电平（低电平有效） */
#define BSP_ALARM_ON_LEVEL      GPIO_PIN_RESET
#define BSP_ALARM_OFF_LEVEL     GPIO_PIN_SET

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化报警引脚
  *         CubeMX 已配置 GPIO 模式/速度/上拉，
  *         本函数将 ALM1/ALM2 置为高电平（无报警状态）。
  */
void BSP_ALARM_Init(void);

/**
  * @brief  置报警 — ALM1 + ALM2 同时输出低电平
  */
void BSP_ALARM_Set(void);

/**
  * @brief  清报警 — ALM1 + ALM2 同时恢复高电平
  */
void BSP_ALARM_Reset(void);

/**
  * @brief  单路置报警
  * @param  id: 1 = ALM1, 2 = ALM2
  */
void BSP_ALARM_SetSingle(uint8_t id);

/**
  * @brief  单路清报警
  * @param  id: 1 = ALM1, 2 = ALM2
  */
void BSP_ALARM_ResetSingle(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_ALARM_H__ */
