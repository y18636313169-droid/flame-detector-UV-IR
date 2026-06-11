/**
  ******************************************************************************
  * @file    hal_led.c
  * @brief   BSP LED 驱动 — TIM5 中断驱动，非阻塞
  *
  *          == 两种工作模式（互斥）==
  *          BLINK: 闪烁 N 次后自动停止
  *          WORK : 持续心跳灯，直到调用 BSP_LED_Stop()
  *
  *          TIM5 配置（CubeMX 生成，参见 tim.c）：
  *            PSC = 32000-1 → 1kHz (1ms/tick)
  *            ARR = interval_ms - 1（每次调用时动态设置）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hal_led.h"
#include "tim.h"            /* 引用 htim5 */

/* Private define ------------------------------------------------------------*/

#define LED_MIN_INTERVAL_MS     (10U)

/* Private types -------------------------------------------------------------*/

typedef enum {
    LED_MODE_IDLE  = 0,
    LED_MODE_BLINK,
    LED_MODE_WORK,
} LED_Mode_t;

typedef struct {
    LED_Mode_t          mode;               /* 当前模式                  */
    volatile uint16_t   toggle_remaining;   /* 剩余翻转次数（BLINK用）  */
} BSP_LED_Ctrl_t;

/* Private variables ---------------------------------------------------------*/

static BSP_LED_Ctrl_t led_ctrl = { LED_MODE_IDLE, 0 };

/* ---------------------------------------------------------------------------*/
/*                        内部辅助                                             */
/* ---------------------------------------------------------------------------*/

/**
  * @brief  启动 TIM5（设置 ARR，清除残留 UIF 后启动）
  */
static void led_start_timer(uint32_t interval_ms)
{
    if (interval_ms < LED_MIN_INTERVAL_MS) {
        interval_ms = LED_MIN_INTERVAL_MS;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    __HAL_TIM_SET_AUTORELOAD(&htim5, interval_ms - 1U);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);

    __set_PRIMASK(primask);

    HAL_TIM_Base_Start_IT(&htim5);
}

/**
  * @brief  停止 TIM5
  */
static void led_stop_timer(void)
{
    HAL_TIM_Base_Stop_IT(&htim5);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    led_ctrl.mode = LED_MODE_IDLE;
    __set_PRIMASK(primask);
}

/* ---------------------------------------------------------------------------*/
/*                        公有 API 实现                                        */
/* ---------------------------------------------------------------------------*/

void BSP_LED_Blink(uint32_t count, uint32_t interval_ms)
{
    if (count == 0) return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    led_ctrl.mode             = LED_MODE_BLINK;
    led_ctrl.toggle_remaining = (uint16_t)(count * 2U);

    __set_PRIMASK(primask);

    led_start_timer(interval_ms);
}

void BSP_LED_Work(uint32_t interval_ms)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    led_ctrl.mode = LED_MODE_WORK;
    __set_PRIMASK(primask);

    led_start_timer(interval_ms);
}

void BSP_LED_Stop(void)
{
    led_stop_timer();
    BSP_LED_Off();
}

void BSP_LED_TickHandler(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    LED_Mode_t mode = led_ctrl.mode;
    __set_PRIMASK(primask);

    switch (mode) {

        case LED_MODE_BLINK:
            BSP_LED_Toggle();

            __disable_irq();
            if (led_ctrl.toggle_remaining > 0) {
                led_ctrl.toggle_remaining--;
            }
            if (led_ctrl.toggle_remaining == 0) {
                __set_PRIMASK(primask);
                led_stop_timer();
                BSP_LED_Off();
            } else {
                __set_PRIMASK(primask);
            }
            break;

        case LED_MODE_WORK:
            BSP_LED_Toggle();
            break;

        default:    /* IDLE — 保护性停止 */
            led_stop_timer();
            break;
    }
}
