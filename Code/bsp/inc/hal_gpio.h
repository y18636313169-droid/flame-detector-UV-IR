/**
  ******************************************************************************
  * @file    hal_gpio.h
  * @brief   BSP GPIO 抽象层头文件
  *          封装 GPIO 数字 I/O 控制，提供统一的板级引脚操作接口。
  *          不依赖特定引脚用途，纯 GPIO 读写。
  *          LED 控制请包含 hal_led.h。
  ******************************************************************************
  */
#ifndef __BSP_HAL_GPIO_H__
#define __BSP_HAL_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

#define BSP_GPIO_OK         (0)
#define BSP_GPIO_ERROR      (-1)

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  GPIO 电平状态
  */
typedef enum {
    BSP_GPIO_LOW  = 0,
    BSP_GPIO_HIGH = 1
} BSP_GPIO_Level_t;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化板级 GPIO 补充配置
  *         CubeMX 的 MX_GPIO_Init() 已完成基本配置，
  *         此函数做 BSP 层的补充初始化（若无则直接返回 OK）。
  * @retval BSP_GPIO_OK     成功
  */
int BSP_GPIO_Init(void);

/**
  * @brief  设置指定引脚输出电平
  * @param  port:  GPIO 端口（如 GPIOA、GPIOC）
  * @param  pin:   引脚号（如 GPIO_PIN_9）
  * @param  level: BSP_GPIO_HIGH 或 BSP_GPIO_LOW
  */
void BSP_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, BSP_GPIO_Level_t level);

/**
  * @brief  读取指定引脚输入电平
  * @param  port: GPIO 端口
  * @param  pin:  引脚号
  * @retval BSP_GPIO_HIGH 或 BSP_GPIO_LOW
  */
BSP_GPIO_Level_t BSP_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

/**
  * @brief  翻转指定引脚电平
  * @param  port: GPIO 端口
  * @param  pin:  引脚号
  */
void BSP_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_GPIO_H__ */
