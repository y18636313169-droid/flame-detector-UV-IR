/**
  ******************************************************************************
  * @file    hal_board.h
  * @brief   BSP 板级初始化聚合
  *
  *          将 BSP 各模块的 Init 集中到一处调用。
  *          CubeMX 生成的 MX_*_Init() 不在本文件内管理，
  *          它们由 main.c 按顺序调用。
  *
  *          推荐 main.c 调用顺序：
  *              HAL_Init();
  *              SystemClock_Config();
  *              MX_GPIO_Init();   MX_DMA_Init();   ...  (CubeMX)
  *              BSP_BoardInit();                      (BSP)
  ******************************************************************************
  */
#ifndef __BSP_HAL_BOARD_H__
#define __BSP_HAL_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

#define BSP_BOARD_OK         (0)
#define BSP_BOARD_ERROR      (-1)

/* Exported functions --------------------------------------------------------*/

int BSP_BoardInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_BOARD_H__ */
