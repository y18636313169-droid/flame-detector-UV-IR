/**
  ******************************************************************************
  * @file    hal_board.c
  * @brief   BSP 板级初始化聚合实现
  *
  *          在 CubeMX 的 MX_*_Init() 全部执行完毕后调用 BSP_BoardInit()，
  *          完成 BSP 各模块的初始化和硬件绑定。
  *
  *          注意事项：BSP_UART_Init 依赖 CubeMX 先初始化对应的 UART 句柄和 DMA，
  *          因此必须在 MX_USART1_UART_Init() / MX_USART2_UART_Init() 之后调用。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hal_board.h"
#include "hal_gpio.h"
#include "hal_adc.h"
#include "hal_tim.h"
#include "hal_uart.h"
#include "hal_alarm.h"
#include "hal_led.h"

/* ---------------------------------------------------------------------------*/
/*                        公有 API 实现                                        */
/* ---------------------------------------------------------------------------*/

int BSP_BoardInit(void)
{
    /* GPIO 补充初始化（确保 LED 初始熄灭等） */
    if (BSP_GPIO_Init() != BSP_GPIO_OK) {
        return BSP_BOARD_ERROR;
    }

    /* 500ms闪烁灯 */
    BSP_LED_Work(500);

    /* 报警输出初始化（MX_GPIO_Init 后立即执行，防止启动期间误触发） */
    BSP_ALARM_Init();

    /* ADC 初始化 */
    if (BSP_ADC_Init() != 0) {
        return BSP_BOARD_ERROR;
    }

    /* TIM3 输入捕获初始化 */
    if (BSP_TIM_IC_Init(BSP_TIM_UV) != BSP_TIM_OK) {
        return BSP_BOARD_ERROR;
    }

    /* 三路串口初始化 */
    for (BSP_UART_Id_t i = 0; i < BSP_UART_NUM; i++) {
        if (BSP_UART_Init(i) != BSP_UART_OK) {
            return BSP_BOARD_ERROR;
        }
    }

    return BSP_BOARD_OK;
}
