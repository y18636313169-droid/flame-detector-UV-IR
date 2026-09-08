/**
  ******************************************************************************
  * @file    hal_alarm.c
  * @brief   Single alarm and fault output driver implementation.
  ******************************************************************************
  */

#include "hal_alarm.h"

void BSP_ALARM_Init(void)
{
    /* Keep both externally visible outputs inactive until a real event occurs. */
    BSP_ALARM_Reset();
    BSP_FAULT_Reset();
}

void BSP_ALARM_Set(void)
{
    HAL_GPIO_WritePin(BSP_ALARM_PORT, BSP_ALARM_PIN, BSP_ALARM_ON_LEVEL);
}

void BSP_ALARM_Reset(void)
{
    HAL_GPIO_WritePin(BSP_ALARM_PORT, BSP_ALARM_PIN, BSP_ALARM_OFF_LEVEL);
}

void BSP_FAULT_Set(void)
{
    HAL_GPIO_WritePin(BSP_FAULT_PORT, BSP_FAULT_PIN, BSP_FAULT_ON_LEVEL);
}

void BSP_FAULT_Reset(void)
{
    HAL_GPIO_WritePin(BSP_FAULT_PORT, BSP_FAULT_PIN, BSP_FAULT_OFF_LEVEL);
}
