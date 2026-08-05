/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hal_alarm.h"
#include "hal_board.h"
#include "hal_led.h"
#include "hal_tim.h"
#include "hal_uart.h"
#include "hal_iwdg.h"

#include "ap_adc.h"
#include "ap_eeprom.h"
#include "ap_ir.h"
#include "ap_uart_protocol.h"
#include "ap_uv.h"
#include "cmd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ISR → 主循环标志 (ISR 只置位, 主循环消费) */
#if defined(IR_TEST_MODE)
static volatile uint8_t  test_print_pending;    /* 10ms 测试打印标志 */
static uint8_t           ir_print_enabled;      /* IR ADC 打印开关 */
static uint8_t           uv_print_enabled;      /* UV 脉冲打印开关 */
#else
static uint8_t           show_mode_enabled;     /* 1: 演示模式仅用UV报警，由EEPROM恢复 */
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if defined(IR_TEST_MODE)

/**
  * @brief  100Hz: ADC→去直流→均方值
  */
static void test_print_data(void)
{
    AP_IR_TestPrint(HAL_GetTick());
}


static void test_print_marker(const char *msg)
{
    BSP_UART_Printf("\r\n=== MARKER");
    if (msg != NULL && msg[0] != '\0') {
        BSP_UART_Printf(" [%s]", msg);
    }
    BSP_UART_Printf(" @ t=%lums ===\r\n\r\n", HAL_GetTick());
}

/* ---- 对外接口 (供 cmd.c 调用) ---- */
void TEST_SetIrEnabled(uint8_t en) { ir_print_enabled = en; }
uint8_t TEST_GetIrEnabled(void) { return ir_print_enabled; }
void TEST_SetUvEnabled(uint8_t en) { uv_print_enabled = en; }
uint8_t TEST_GetUvEnabled(void) { return uv_print_enabled; }

void TEST_InsertMarker(const char *msg) { test_print_marker(msg); }

#else

int APP_SetShowMode(uint8_t en)
{
    uint8_t next_mode = (en != 0U) ? 1U : 0U;

    if (show_mode_enabled == next_mode) {
        return 0;
    }

    AP_EEPROM_System_Param_t config = *AP_EEPROM_System_Get();
    config.show_mode = next_mode;
    /*
     * 命令切换属于低频操作，允许在主循环写DATA EEPROM。
     * 必须先保存成功再切换运行状态，防止RAM模式与掉电配置不一致。
     */
    if (AP_EEPROM_System_Save(&config) != 0) {
        return -1;
    }

    show_mode_enabled = next_mode;
    /*
     * 演示模式暂停IR任务。进入和退出时均清空IR状态、滤波器及历史窗口，
     * 防止恢复双传感器判断后使用切换前的旧FIRE状态或过期特征。
     */
    AP_IR_Reset();
    return 0;
}

uint8_t APP_GetShowMode(void)
{
    return show_mode_enabled;
}

#endif /* IR_TEST_MODE */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  BSP_BoardInit();

  AP_EEPROM_Init();    // 加载 EEPROM 参数到内存
  AP_IR_Init();        // 从 EEPROM 加载参数并初始化红外检测
  AP_UV_Init(NULL);    // 从 EEPROM 读取参数并初始化紫外检测
  AP_ADC_Init();
  AP_UART_ProtocolInit();

  /* ISR → 主循环标志初始化 */
#if defined(IR_TEST_MODE)
  test_print_pending = 0;
  ir_print_enabled = 0;
  uv_print_enabled = 0;   /* 默认关闭 */
  BSP_UART_Printf("[TEST] IR_TEST_MODE enabled — type 'debug on' to start\r\n");
#else
  /* EEPROM已完成CRC和取值校验，上电直接恢复断电前的演示/正常模式。 */
  show_mode_enabled = (uint8_t)AP_EEPROM_System_Get()->show_mode;
  BSP_UART_Printf("NORMAL START! SHOW_MODE=%s\r\n",
                  show_mode_enabled ? "on" : "off");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if !defined(IR_TEST_MODE) /* 测试模式下不进行串口通信 使用命令行指令测试通信串口收发 */
    // AP_UART_TxTask(); // 串口通信TX任务
    // AP_UART_RxTask(); // 串口通信RX任务
#endif /* IR_TEST_MODE */

    cmd_parser_task(); // 命令行解析

#if defined(IR_TEST_MODE)
    /* 测试模式: IR 打印由 test_print_pending 触发 */
    if (test_print_pending) {
        test_print_pending = 0;
        if (ir_print_enabled) {
            test_print_data();
        }
        if (uv_print_enabled) {
            static uint32_t uv_cnt = 0;
            uv_cnt++;
            if (uv_cnt >= 50) {
                uv_cnt = 0;
                AP_UV_PrintData(HAL_GetTick());
            }
        }
    }
#else
    /* ================================================================ */
    /*  正常模式: 紫外检测 + 红外检测 + 双重确认                          */
    /* ================================================================ */
    AP_UV_Task();
    /*
     * 演示模式仅推进UV状态机以缩短演示报警时间；IR定时中断仍只置单bit标志，
     * 不会形成采样积压，退出演示模式时由APP_SetShowMode()重新清空IR上下文。
     */
    if (!APP_GetShowMode()) {
        AP_IR_Task();
    }

    /* 正常模式要求UV&&IR；演示模式只要求UV，演示开关不改变各传感器内部算法。 */
    {
        static uint8_t last_fire = 0;
        uint8_t show_mode = APP_GetShowMode();
        uint8_t now_fire = (AP_UV_GetState() == UV_STATE_FIRE)
                        && (show_mode || (AP_IR_GetState() == IR_STATE_FIRE));
        if (now_fire && !last_fire) {
            BSP_ALARM_Set();              // 硬件报警输出 (ALM1+ALM2 低)
            // uint8_t data = 1;
            // AP_UART_Send(AP_FCODE_FIRE_ALARM, &data, 1);
#if defined(AP_ALGO_DEBUG_ENABLE)
            /* 仅记录最终组合报警沿，便于区分单传感器FIRE与实际输出报警。 */
            BSP_UART_Printf("[ALARM] T%lu ON MODE=%s UV=%u IR=%u\r\n",
                            (unsigned long)HAL_GetTick(),
                            show_mode ? "SHOW" : "NORMAL",
                            (unsigned int)AP_UV_GetState(),
                            (unsigned int)AP_IR_GetState());
#endif
        } else if (!now_fire && last_fire) {
            BSP_ALARM_Reset();            // 硬件报警解除 (ALM1+ALM2 高)
            // uint8_t data = 0;
            // AP_UART_Send(AP_FCODE_FIRE_ALARM, &data, 1);
#if defined(AP_ALGO_DEBUG_ENABLE)
            BSP_UART_Printf("[ALARM] T%lu OFF MODE=%s UV=%u IR=%u\r\n",
                            (unsigned long)HAL_GetTick(),
                            show_mode ? "SHOW" : "NORMAL",
                            (unsigned int)AP_UV_GetState(),
                            (unsigned int)AP_IR_GetState());
#endif
        }
        last_fire = now_fire;
    }
#endif /* IR_TEST_MODE */

    BSP_IWDG_CheckAndRefresh();   // 检查标志位并喂狗
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* ========================================================================== */
/*          HAL 定时器回调覆盖 — 由 stm32l1xx_it.c 中的 IRQHandler 触发        */
/* ========================================================================== */

/**
  * @brief  输入捕获回调 — 转发给 BSP_TIM_IC_CaptureHandler（TIM3）
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    BSP_TIM_IC_CaptureHandler(htim);
}

void task_10ms(void)
{
  AP_IR_FeedIsr();        // 仅置位 volatile 标志 (极轻量)
#if defined(IR_TEST_MODE)
  test_print_pending = 1; // 主循环消费, 执行 test_print_data()
#endif
}

/**
  * @brief  更新事件回调 — TIM5 驱动 LED，TIM2 给 TIM 溢出计数
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint32_t cnt_task = 0;
    if (htim->Instance == TIM5) {
        BSP_LED_TickHandler();
        return;
    }
    if (htim->Instance == TIM6) {
        cnt_task ++;
        // AP_UART_CheckTimeout();
        if (cnt_task % 10 == 0)
        {
          task_10ms();
        }
        if (cnt_task % 2000 == 0)          // 每 2000ms 请求喂狗
        {
          BSP_IWDG_RequestFeed();
        }
        return;
    }

    BSP_TIM_IC_PeriodHandler(htim);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
