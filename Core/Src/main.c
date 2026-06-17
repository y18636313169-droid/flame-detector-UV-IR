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
static volatile uint8_t  ap_adc_pending;        /* 10ms ADC 采样标志 */
#if defined(IR_TEST_MODE)
static volatile uint8_t  test_print_pending;    /* 10ms 测试打印标志 */
static uint8_t           test_print_enabled;    /* CLI on/off 控制 */
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  ADC 采样任务 (主循环调用)
  *         消费 ap_adc_pending 标志, 执行实际 ADC 数据读取和滑动滤波
  */
void adc_sample_task(void)
{
    if (!ap_adc_pending) return;
    ap_adc_pending = 0;
    AP_ADC_Update();
}

#if defined(IR_TEST_MODE)

/**
  * @brief  测试模式: 打印 ADC + UV 原始数据
  *         100Hz 紧凑格式, 每行 T<ms> 前缀, 方便 PC 解析
  */
static void test_print_data(void)
{
    uint32_t tick = HAL_GetTick();

    /* ADC 3 通道 */
    BSP_UART_Printf("T%lu ADC %u %u %u\r\n",
        (unsigned long)tick,
        AP_ADC_GetLatest(0), AP_ADC_GetLatest(1), AP_ADC_GetLatest(2));

    /* UV 脉冲: 脉宽列表 */
    BSP_TIM_PulseData_t pulses[20];
    uint16_t n = BSP_TIM_IC_ReadAllPulse(BSP_TIM_UV, pulses, 20);
    BSP_UART_Printf("T%lu UV %u", (unsigned long)tick, (unsigned)n);
    for (uint16_t i = 0; i < n; i++) {
        BSP_UART_Printf(" %u", pulses[i].pulse_width_us);
    }
    BSP_UART_Printf("\r\n");
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
void TEST_SetPrintEnabled(uint8_t en) { test_print_enabled = en; }
uint8_t TEST_GetPrintEnabled(void) { return test_print_enabled; }
void TEST_InsertMarker(const char *msg) { test_print_marker(msg); }

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
  ap_adc_pending = 0;
#if defined(IR_TEST_MODE)
  test_print_pending = 0;
  test_print_enabled = 0;   // 默认关闭, 通过 debug on 开启
  BSP_UART_Printf("[TEST] IR_TEST_MODE enabled — type 'debug on' to start\r\n");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    AP_UART_TxTask(); // 串口通信TX任务
    AP_UART_RxTask(); // 串口通信RX任务

    cmd_parser_task(); // 命令行解析

    adc_sample_task(); // 消费 ap_adc_pending → AP_ADC_Update()

#if defined(IR_TEST_MODE)
    /* ================================================================ */
    /*  测试模式: 100Hz 原始数据打印 (CLI: debug on/off, mark)          */
    /* ================================================================ */
    if (test_print_pending && test_print_enabled) {
        test_print_pending = 0;
        test_print_data();
    }
#else
    /* ================================================================ */
    /*  正常模式: 紫外检测 + 红外检测 + 双重确认                          */
    /* ================================================================ */
    AP_UV_Process(HAL_GetTick());
    AP_IR_Task();

    if (AP_ADC_IsPrintPending()) {
        AP_ADC_PrintDebug();
    }

    /* 双重确认: UV && IR 同时触发 → 火警上报 */
    {
        static uint8_t last_fire = 0;
        uint8_t now_fire = (AP_UV_GetState() == UV_STATE_FIRE)
                        && (AP_IR_GetState() == IR_STATE_FIRE);
        if (now_fire && !last_fire) {
            uint8_t data = 1;
            AP_UART_Send(AP_FCODE_FIRE_ALARM, &data, 1);
        } else if (!now_fire && last_fire) {
            uint8_t data = 0;
            AP_UART_Send(AP_FCODE_FIRE_ALARM, &data, 1);
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
  ap_adc_pending = 1;     // 主循环消费, 执行 AP_ADC_Update()
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
        AP_UART_CheckTimeout();
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
