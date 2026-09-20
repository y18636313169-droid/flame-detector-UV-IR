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
#include "ap_fault.h"
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
static uint8_t           alarm_output_active;   /* 最终ALM锁存状态，仅RECOVER/复位清除 */
static volatile uint8_t  recover_irq_pending;   /* EXTI只锁存事件，主循环执行完整恢复 */
static uint8_t           recover_low_latched;   /* 持续低电平只允许触发一次恢复 */

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
  * @brief  消费一次低有效RECOVER事件并清除全部火警状态。
  * @note   即使脉冲在主循环执行前已经结束，EXTI锁存标志仍可保留该事件。
  *         持续低电平只触发一次，输入恢复高电平后才允许下一次恢复；
  *         算法状态和历史窗口清零必须留在主循环，禁止在中断中执行。
  */
static void recover_input_task(void)
{
    if (recover_irq_pending == 0U) {
        /* 输入恢复高电平后，重新允许下一次下降沿恢复事件。 */
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

        /* 手动恢复同时清除两个探测器状态和最终锁存报警输出。 */
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
     * 测试/应用算法使用不同的数据推进方式，切换时清空检测上下文；
     * 已锁存的整机火警只能由RECOVER/复位清除，模式命令不得隐式消警。
     */
    AP_IR_Reset();
    AP_UV_Process_Reset();
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

/**
  * @brief 更新集中故障状态，并把ADC模块故障映射到整机故障位图。
  * @note  最终火警锁存期间仍监视电源轨、冻结和DMA，但暂停静态偏置判断，
  *        防止火焰信号改变直流均值后产生无意义的偏置故障。
  */
static void app_fault_task(void)
{
    uint32_t adc_fault_bits = 0U;
    uint8_t adc_flags;
    uint8_t rail_mask;

    AP_Fault_Task(HAL_GetTick());
    (void)AP_ADC_FaultMonitorTask(HAL_GetTick(), alarm_output_active);
    rail_mask = AP_ADC_GetRailFaultMask();
    adc_flags = AP_ADC_GetFaultFlags();

    if ((rail_mask & (1U << 0)) != 0U) adc_fault_bits |= AP_FAULT_ADC_CH38_RANGE_ERROR;
    if ((rail_mask & (1U << 1)) != 0U) adc_fault_bits |= AP_FAULT_ADC_CH45_RANGE_ERROR;
    if ((rail_mask & (1U << 2)) != 0U) adc_fault_bits |= AP_FAULT_ADC_CH50_RANGE_ERROR;
    if ((adc_flags & AP_ADC_FAULT_DATA_STUCK) != 0U) adc_fault_bits |= AP_FAULT_ADC_DATA_STUCK;
    if ((adc_flags & AP_ADC_FAULT_BIAS) != 0U) adc_fault_bits |= AP_FAULT_ADC_BIAS_ERROR;
    if ((adc_flags & AP_ADC_FAULT_DMA) != 0U) adc_fault_bits |= AP_FAULT_ADC_DMA_ERROR;

    AP_Fault_Update(AP_FAULT_ADC_CH38_RANGE_ERROR |
                    AP_FAULT_ADC_CH45_RANGE_ERROR |
                    AP_FAULT_ADC_CH50_RANGE_ERROR |
                    AP_FAULT_ADC_DATA_STUCK |
                    AP_FAULT_ADC_BIAS_ERROR |
                    AP_FAULT_ADC_DMA_ERROR,
                    adc_fault_bits);
}

uint8_t APP_GetAlarmActive(void)
{
    return alarm_output_active;
}

/**
  * @brief 应用已经校验的SYSTEM EEPROM副本，不重复写入EEPROM。
  * @note  重新配置会清空探测器历史，但不得清除已锁存的最终火警；
  *        最终火警只允许通过RECOVER输入或MCU复位清除。
  */
void APP_ApplySystemConfig(void)
{
    const AP_EEPROM_System_Param_t *system = AP_EEPROM_System_Get();

    show_mode_enabled = (uint8_t)system->show_mode;
    test_mode_enabled = (uint8_t)system->test_mode;
    AP_IR_SetProfileEnabled((uint8_t)system->ir_profile_enabled);
    AP_IR_Reset();
    AP_UV_Process_Reset();
}

/**
  * @brief  恢复全部可配置EEPROM参数组，并应用校验通过的RAM副本。
  * @retval 全部成功返回0；存在整机火警或任一参数组失败时返回-1。
  * @note   仅操作当前四个配置扇区，不触碰设备身份及后续生产/标定扇区。
  */
int APP_FactoryReset(void)
{
    const AP_EEPROM_UV_Param_t *uv;
    const AP_EEPROM_IR_Param_t *ir;
    uint16_t failed_groups;

    if (alarm_output_active != 0U) {
        return -1;
    }

    failed_groups = AP_EEPROM_ResetAll();
    uv = AP_EEPROM_UV_Get();
    ir = AP_EEPROM_IR_Get();

    /* 每组都应用其校验后的RAM副本；写失败的组继续使用原有效配置。 */
    APP_ApplySystemConfig();
    AP_UV_SetConfig(uv->thr_min, uv->thr_max, uv->win_min, uv->win_max,
                    uv->cfm_min, uv->cfm_max);
    AP_UV_SetLevel((uint8_t)uv->sensitivity);
    AP_UV_SetPrintWindow(uv->print_window_ms);
    BSP_TIM_IC_SetPulseRange((uint16_t)uv->pw_min_us,
                             (uint16_t)uv->pw_max_us);
    AP_IR_SetConfig(ir->power_min, ir->power_max,
                    ir->r38_threshold, ir->r50_threshold,
                    ir->freq_low_x10, ir->freq_high_x10,
                    ir->cfm_min, ir->cfm_max);
    AP_IR_SetLevel((uint8_t)ir->sensitivity);

    /* 配置整体替换后，清空两套探测器历史，禁止旧窗口继续参与判断。 */
    AP_IR_Reset();
    AP_UV_Process_Reset();
    return (failed_groups == 0U) ? 0 : -1;
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

  AP_Fault_Init(HAL_GetTick()); /* 统一故障位先接管BUG，并保存最近复位原因。 */
  (void)AP_EEPROM_Init(); /* 逐组错误已由EEPROM模块写入集中故障位。 */
  AP_IR_Init();        // 从 EEPROM 加载参数并初始化红外检测
  AP_UV_Init(NULL);    // 从 EEPROM 读取参数并初始化紫外检测
  if (AP_ADC_Init() != BSP_ADC_OK) {
    AP_Fault_Set(AP_FAULT_ADC_INIT_ERROR);
  }
  AP_UART_ProtocolInit();

  /* EEPROM已完成CRC/范围校验，运行模式和包络开关均恢复断电前配置。 */
  const AP_EEPROM_System_Param_t *system_config = AP_EEPROM_System_Get();
  test_print_pending = 0U;
  ir_print_enabled = 0U;
  uv_print_enabled = 0U;
  alarm_output_active = 0U;
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
    recover_input_task(); /* RECOVER低脉冲清除IR、UV及最终ALM锁存状态。 */

    /*
     * USART2配置协议只在主循环执行：先消费已到达的DMA数据，再检查会话超时，
     * 避免10s边界上已经到达的合法请求被误判为未建联。RX仍限制每轮32字节。
     */
    AP_UART_RxTask();
    AP_UART_TxTask();
    AP_UART_CheckTimeout();

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
            BSP_ALARM_Set();              // 单路ALM输出：低电平表示火警
            alarm_output_active = 1U;
#if defined(AP_ALGO_DEBUG_ENABLE)
            /* 仅记录最终组合报警沿，便于区分单传感器FIRE与实际输出报警。 */
            BSP_UART_Printf("[ALARM] T%lu ON MODE=%s UV=%u IR=%u\r\n",
                            (unsigned long)HAL_GetTick(),
                            show_mode ? "SHOW" : "NORMAL",
                            (unsigned int)AP_UV_GetState(),
                            (unsigned int)AP_IR_GetState());
#endif
        }
        /* IR/UV后续掉出不自动消警，最终ALM保持到RECOVER低脉冲或MCU复位。 */
      }
    }

    /* 集中故障任务放在探测任务之后，使本轮刚锁存的火警立即暂停偏置检测。 */
    app_fault_task();
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

/** @brief 记录一次完整三通道ADC DMA扫描，作为采集健康监视依据。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc_handle)
{
  BSP_ADC_ConvCpltHandler(hadc_handle);
}

/** @brief 记录ADC/HAL错误事件，故障确认仍由主循环完成。 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc_handle)
{
  BSP_ADC_ErrorHandler(hadc_handle);
}

/**
  * @brief  锁存低有效RECOVER下降沿，交由主循环处理。
  * @note   EXTI中禁止清空探测历史或执行串口打印。
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
  /* 致命卡死由IWDG复位恢复；可监视的当前故障统一由AP_Fault聚合。 */
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
