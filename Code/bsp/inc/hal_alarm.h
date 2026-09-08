/**
  ******************************************************************************
  * @file    hal_alarm.h
  * @brief   Alarm and fault output driver.
  *
  *          ALM(PA0) is active low: low=alarm, high=idle.
  *          BUG(PA1) is active low: low=fault, high=normal.
  ******************************************************************************
  */
#ifndef __BSP_HAL_ALARM_H__
#define __BSP_HAL_ALARM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BSP_ALARM_PIN          ALM_Pin
#define BSP_ALARM_PORT         ALM_GPIO_Port
#define BSP_FAULT_PIN          BUG_Pin
#define BSP_FAULT_PORT         BUG_GPIO_Port

#define BSP_ALARM_ON_LEVEL     GPIO_PIN_RESET
#define BSP_ALARM_OFF_LEVEL    GPIO_PIN_SET
#define BSP_FAULT_ON_LEVEL     GPIO_PIN_RESET
#define BSP_FAULT_OFF_LEVEL    GPIO_PIN_SET

/** Initialize ALM and BUG to their inactive levels. */
void BSP_ALARM_Init(void);

/** Assert or clear the single fire alarm output. */
void BSP_ALARM_Set(void);
void BSP_ALARM_Reset(void);

/** Assert or clear the hardware fault indication output. */
void BSP_FAULT_Set(void);
void BSP_FAULT_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_ALARM_H__ */
