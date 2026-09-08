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
static volatile uint8_t  test_print_pending;    /* 10ms 测试打印标志 */
static uint8_t           ir_print_enabled;      /* IR ADC 打印开关 */
static uint8_t           uv_print_enabled;      /* UV 脉冲打印开关 */
static uint8_t           show_mode_enabled;     /* 1: 演示模式仅用UV报警，由EEPROM恢复 */
static uint8_t           test_mode_enabled;     /* 1: 运行时测试模式，不执行报警算法 */
static uint8_t           alarm_output_active;   /* 最终ALM输出沿状态，模式切换时同步清零 */
static uint8_t           adc_fault_output_active; /* BUG输出沿状态，仅由ADC轨到轨故障驱动 */
static volatile uint8_t  recover_irq_pending;   /* EXTI只锁存事件，主循环执行完整恢复 */
static uint8_t           recover_low_latched;   /* Continuous low level triggers only once */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  100Hz: ADC→去直流→均方值
  */
static void test_print_data(void)
{
    AP_IR_TestPrint(HAL_GetTick());
}

/**
  * @brief  Consume an active-low RECOVER edge and clear all alarm states once.
  * @note   EXTI latches even a pulse that has ended before the main loop runs.
  *         A continuous low level triggers only once; it must return high
  *         before another recovery is accepted. Algorithm and history reset
  *         deliberately remain outside interrupt context.
  */
static void recover_input_task(void)
{
    if (recover_irq_pending == 0U) {
        /* Returning high rearms the next falling edge after the current event. */
        if (HAL_GPIO_ReadPin(UV_RECOVER_GPIO_Port, UV_RECOVER_Pin) != GPIO_PIN_RESET) {
            recover_low_latched = 0U;
        }
        return;
    }

    recover_irq_pending = 0U;
    if (recover_low_latched != 0U) {
        return;
    }

    {
        uint32_t now = HAL_GetTick();

        /* Manual recovery clears sensor states and the final alarm output together. */
        AP_IR_Reset();
        AP_UV_Process_Reset();
        BSP_ALARM_Reset();
        alarm_output_active = 0U;
        recover_low_latched = 1U;

#if defined(AP_ALGO_DEBUG_ENABLE)
        BSP_UART_Printf("[RECOVER] T%lu alarm states reset\r\n", (unsigned long)now);
#else
        (void)now;
#endif
    }
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

int APP_SetTestMode(uint8_t en)
{
    uint8_t next_mode = (en != 0U) ? 1U : 0U;
    if (test_mode_enabled == next_mode) return 0;

    AP_EEPROM_System_Param_t config = *AP_EEPROM_System_Get();
    config.test_mode = next_mode;
    /* 与演示模式一致，必须先持久化成功再改变主循环执行路径。 */
    if (AP_EEPROM_System_Save(&config) != 0) return -1;

    test_mode_enabled = next_mode;
    if (next_mode == 0U) {
        /* 离开测试模式时强制恢复USART2正式协议，防止诊断模式遗留导致失联。 */
        AP_UART_SetDiagnosticMode(false);
    }
    /*
     * 测试/应用算法使用不同的数据推进方式。切换时清空两套检测上下文和
     * 硬件报警，防止测试历史、旧FIRE状态或脉冲队列跨模式继续生效。
     */
    AP_IR_Reset();
    AP_UV_Process_Reset();
    BSP_ALARM_Reset();
    alarm_output_active = 0U;
    test_print_pending = 0U;
    return 0;
}

uint8_t APP_GetTestMode(void)
{
    return test_mode_enabled;
}

int APP_SetIrProfileEnabled(uint8_t en)
{
    uint8_t next = (en != 0U) ? 1U : 0U;
    if (AP_IR_GetProfileEnabled() == next) return 0;

    AP_EEPROM_System_Param_t config = *AP_EEPROM_System_Get();
    config.ir_profile_enabled = next;
    if (AP_EEPROM_System_Save(&config) != 0) return -1;

    /* EEPROM成功后再清理并切换包络分类器，保持掉电配置与当前行为一致。 */
    AP_IR_SetProfileEnabled(next);
    return 0;
}

uint8_t APP_GetIrProfileEnabled(void)
{
    return AP_IR_GetProfileEnabled();
}

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
  if (BSP_BoardInit() != BSP_BOARD_OK) {
    Error_Handler();
  }

  AP_EEPROM_Init();    // 加载 EEPROM 参数到内存
  AP_IR_Init();        // 从 EEPROM 加载参数并初始化红外检测
  AP_UV_Init(NULL);    // 从 EEPROM 读取参数并初始化紫外检测
  AP_ADC_Init();
  AP_UART_ProtocolInit();

  /* EEPROM已完成CRC/范围校验，运行模式和包络开关均恢复断电前配置。 */
  const AP_EEPROM_System_Param_t *system_config = AP_EEPROM_System_Get();
  test_print_pending = 0U;
  ir_print_enabled = 0U;
  uv_print_enabled = 0U;
  alarm_output_active = 0U;
  adc_fault_output_active = 0U;
  recover_irq_pending = 0U;
  recover_low_latched = 0U;
  show_mode_enabled = (uint8_t)system_config->show_mode;
  test_mode_enabled = (uint8_t)system_config->test_mode;
  AP_IR_SetProfileEnabled((uint8_t)system_config->ir_profile_enabled);
  BSP_UART_Printf("START MODE=%s SHOW=%s PROFILE=%s\r\n",
                  test_mode_enabled ? "TEST" : "APP",
                  show_mode_enabled ? "on" : "off",
                  APP_GetIrProfileEnabled() ? "on" : "off");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    recover_input_task(); /* RECOVER low clears IR, UV and the final ALM state. */

    /*
     * USART2配置协议只在主循环执行：先消费已到达的DMA数据，再检查会话超时，
     * 避免10s边界上已经到达的合法请求被误判为未建联。RX仍限制每轮32字节。
     */
    AP_UART_RxTask();
    AP_UART_TxTask();
    AP_UART_CheckTimeout();

    /*
     * BUG只表示红外ADC硬件级轨到轨故障，不承载初始化、通信或算法未通过状态。
     * 监控在所有运行模式下执行，但不直接改变IR/UV火警状态。
     */
    {
      uint8_t adc_fault = AP_ADC_FaultMonitorTask(HAL_GetTick());
      if (adc_fault != adc_fault_output_active) {
        if (adc_fault != 0U) {
          BSP_FAULT_Set();       /* BUG低电平：任一ADC通道连续0/4095满10秒 */
        } else {
          BSP_FAULT_Reset();     /* BUG高电平：三通道连续恢复有效值满2秒 */
        }
        adc_fault_output_active = adc_fault;
      }
    }
    cmd_parser_task(); // 命令行解析

    if (APP_GetTestMode()) {
      /* 运行时测试模式: 仅定时采集/打印，不推进UV/IR报警状态机。 */
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
    } else {
      /* ================================================================ */
      /*  应用模式: 紫外检测 + 红外检测 + 最终组合确认                     */
      /* ================================================================ */
      AP_UV_Task();
      /*
       * 演示模式仅推进UV状态机以缩短演示报警时间；IR定时中断仍只置单bit标志，
       * 不会形成采样积压，退出演示模式时由APP_SetShowMode()重新清空IR上下文。
       */
      if (!APP_GetShowMode()) {
          AP_IR_Task();
      }

      /* 正常模式要求UV&&IR；演示模式只要求UV，演示开关不改变传感器内部算法。 */
      {
        uint8_t show_mode = APP_GetShowMode();
        uint8_t now_fire = (AP_UV_GetState() == UV_STATE_FIRE)
                        && (show_mode || (AP_IR_GetState() == IR_STATE_FIRE));
        if (now_fire && !alarm_output_active) {
            BSP_ALARM_Set();              // Single ALM output: low means alarm
#if defined(AP_ALGO_DEBUG_ENABLE)
            /* 仅记录最终组合报警沿，便于区分单传感器FIRE与实际输出报警。 */
            BSP_UART_Printf("[ALARM] T%lu ON MODE=%s UV=%u IR=%u\r\n",
                            (unsigned long)HAL_GetTick(),
                            show_mode ? "SHOW" : "NORMAL",
                            (unsigned int)AP_UV_GetState(),
                            (unsigned int)AP_IR_GetState());
#endif
        } else if (!now_fire && alarm_output_active) {
            BSP_ALARM_Reset();            // Return single ALM output high (idle)
#if defined(AP_ALGO_DEBUG_ENABLE)
            BSP_UART_Printf("[ALARM] T%lu OFF MODE=%s UV=%u IR=%u\r\n",
                            (unsigned long)HAL_GetTick(),
                            show_mode ? "SHOW" : "NORMAL",
                            (unsigned int)AP_UV_GetState(),
                            (unsigned int)AP_IR_GetState());
#endif
        }
        alarm_output_active = now_fire;
      }
    }

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

/**
  * @brief  Latch the active-low RECOVER falling edge for the main loop.
  * @note   Do not reset detector histories or print from EXTI context.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == UV_RECOVER_Pin) {
    recover_irq_pending = 1U;
  }
}

void task_10ms(void)
{
  AP_IR_FeedIsr();        // 仅置位 volatile 标志 (极轻量)
  test_print_pending = 1; // 主循环消费, 执行 test_print_data()
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
  /* Fatal MCU/software errors are not mapped to BUG; BUG is reserved for ADC rail faults. */
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
