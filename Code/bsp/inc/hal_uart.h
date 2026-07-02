/**
  ******************************************************************************
  * @file    hal_uart.h
  * @brief   BSP UART 抽象层 — TX DMA + RX DMA CIRCULAR 环形缓冲驱动
  *
  *          TX: 应用写入环形缓冲 → DMA 自动搬运。发送完成中断自动链式发送下一段。
  *          RX: DMA CIRCULAR 持续填充 → 应用通过 Read/Peek/Consume 读取。
  *
  *          串口资源：
  *            DBG (调试) = USART1 — PA9(TX)/PA10(RX), DMA1_CH4(TX)/CH5(RX)
  *            COM (通信) = USART2 — PA2(TX)/PA3(RX),  DMA1_CH7(TX)/CH6(RX)
  ******************************************************************************
  */
#ifndef __BSP_HAL_UART_H__
#define __BSP_HAL_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

/* Exported defines ----------------------------------------------------------*/

#define BSP_UART_TX_BUF_SIZE        (256U)
#define BSP_UART_RX_BUF_SIZE        (256U)   /* 环形缓冲，须为 2^n */
#define BSP_UART_PRINTF_BUF_SIZE    (256U)

#define BSP_UART_OK                 (0)
#define BSP_UART_ERROR              (-1)
#define BSP_UART_TIMEOUT            (-2)

/* ========================================================================== */
/*                     UART 端口角色映射                                       */
/* ========================================================================== */

#define BSP_UART_COM_INST           USART2    /* 通信串口（外部协议交互） */
#define BSP_UART_DBG_INST           USART1    /* 调试串口（日志打印）     */

#define BSP_UART_COM_BAUDRATE       115200U
#define BSP_UART_DBG_BAUDRATE       115200U

/* Exported types ------------------------------------------------------------*/

typedef enum {
    BSP_UART_COM = 0,
    BSP_UART_DBG,
    BSP_UART_NUM
} BSP_UART_Id_t;

/* Exported functions --------------------------------------------------------*/

/* ---- 初始化/反初始化 ---------------------------------------------------- */
int  BSP_UART_Init(BSP_UART_Id_t id);
int  BSP_UART_DeInit(BSP_UART_Id_t id);

/* ---- 发送（非阻塞 + 阻塞） ---------------------------------------------- */
uint16_t BSP_UART_Write(BSP_UART_Id_t id, const uint8_t *data, uint16_t len);
int      BSP_UART_WriteBlock(BSP_UART_Id_t id, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
bool     BSP_UART_IsTxComplete(BSP_UART_Id_t id);

/* ---- 接收（从 RX 中断环形缓冲读取）--------------------------------------- */
uint16_t BSP_UART_Read(BSP_UART_Id_t id, uint8_t *buf, uint16_t len);
uint16_t BSP_UART_GetRxDataLen(BSP_UART_Id_t id);

/* ---- Peek / Consume ----------------------------------------------------- */
bool     BSP_UART_Peek(BSP_UART_Id_t id, void *out, uint16_t offset);
uint16_t BSP_UART_PeekPacket(BSP_UART_Id_t id, void *out, uint16_t offset, uint16_t len);
void     BSP_UART_Consume(BSP_UART_Id_t id, uint16_t len);

/* ---- 缓冲管理 ----------------------------------------------------------- */
void     BSP_UART_ClearRxBuf(BSP_UART_Id_t id);
void     BSP_UART_ClearTxBuf(BSP_UART_Id_t id);

/* ---- Printf ------------------------------------------------------------- */
int BSP_UART_Printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_UART_H__ */
