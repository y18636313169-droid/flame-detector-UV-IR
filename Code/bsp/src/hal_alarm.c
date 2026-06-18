/**
  ******************************************************************************
  * @file    hal_alarm.c
  * @brief   BSP 报警输出驱动实现
  *
  *          CubeMX 已将 ALM1(PA1)、ALM2(PA0) 配置为推挽输出、内部上拉。
  *          上电初始态为低电平，调用 BSP_ALARM_Init() 后置高（无报警）。
  *
  *          == 电平约定 ==
  *            低电平 = 报警有效
  *            高电平 = 无报警（内部上拉维持）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hal_alarm.h"

/* ---------------------------------------------------------------------------*/
/*                        公有 API 实现                                        */
/* ---------------------------------------------------------------------------*/

void BSP_ALARM_Init(void)
{
    /* 上电后置高（无报警），CubeMX 默认初始化值可能为低 */
    HAL_GPIO_WritePin(BSP_ALM1_PORT, BSP_ALM1_PIN, BSP_ALARM_OFF_LEVEL);
    HAL_GPIO_WritePin(BSP_ALM2_PORT, BSP_ALM2_PIN, BSP_ALARM_OFF_LEVEL);
}

void BSP_ALARM_Set(void)
{
    HAL_GPIO_WritePin(BSP_ALM1_PORT, BSP_ALM1_PIN, BSP_ALARM_ON_LEVEL);
    HAL_GPIO_WritePin(BSP_ALM2_PORT, BSP_ALM2_PIN, BSP_ALARM_ON_LEVEL);
}

void BSP_ALARM_Reset(void)
{
    HAL_GPIO_WritePin(BSP_ALM1_PORT, BSP_ALM1_PIN, BSP_ALARM_OFF_LEVEL);
    HAL_GPIO_WritePin(BSP_ALM2_PORT, BSP_ALM2_PIN, BSP_ALARM_OFF_LEVEL);
}

void BSP_ALARM_SetSingle(uint8_t id)
{
    GPIO_TypeDef *port;
    uint16_t      pin;

    if (id == 1) {
        port = BSP_ALM1_PORT; pin = BSP_ALM1_PIN;
    } else if (id == 2) {
        port = BSP_ALM2_PORT; pin = BSP_ALM2_PIN;
    } else {
        return;
    }
    HAL_GPIO_WritePin(port, pin, BSP_ALARM_ON_LEVEL);
}

void BSP_ALARM_ResetSingle(uint8_t id)
{
    GPIO_TypeDef *port;
    uint16_t      pin;

    if (id == 1) {
        port = BSP_ALM1_PORT; pin = BSP_ALM1_PIN;
    } else if (id == 2) {
        port = BSP_ALM2_PORT; pin = BSP_ALM2_PIN;
    } else {
        return;
    }
    HAL_GPIO_WritePin(port, pin, BSP_ALARM_OFF_LEVEL);
}
