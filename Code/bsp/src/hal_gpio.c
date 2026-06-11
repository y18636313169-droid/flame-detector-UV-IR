/**
  ******************************************************************************
  * @file    hal_gpio.c
  * @brief   BSP GPIO 抽象层实现 — 纯 GPIO I/O 操作
  *          底层使用 STM32 HAL GPIO 驱动。
  *          LED 控制请见 hal_led.c。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hal_gpio.h"
#include "gpio.h"

/* ---------------------------------------------------------------------------*/
/*                        公有 API 实现                                        */
/* ---------------------------------------------------------------------------*/

int BSP_GPIO_Init(void)
{
    /* CubeMX 的 MX_GPIO_Init() 已在 main 开头完成全部 GPIO 配置 */
    return BSP_GPIO_OK;
}

void BSP_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, BSP_GPIO_Level_t level)
{
    if (port == NULL) return;
    HAL_GPIO_WritePin(port, pin, (GPIO_PinState)level);
}

BSP_GPIO_Level_t BSP_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == NULL) return BSP_GPIO_LOW;
    return (BSP_GPIO_Level_t)HAL_GPIO_ReadPin(port, pin);
}

void BSP_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == NULL) return;
    HAL_GPIO_TogglePin(port, pin);
}
