/**
  ******************************************************************************
  * @file    ap_uart_protocol.h
  * @brief   AP 层串口通信协议模块
  *
  *          帧格式（宏定义，待定可改）：
  *  ┌──────┬──────┬──────┬──────┬──────────┬──────┬──────┐
  *  │ STX  │ SEQ  │ LEN  │ FCODE│  DATA    │ CRC16│ ETX  │
  *  │ 1B   │ 1B   │ 1B   │ 1B   │  nB      │ 2B   │ 1B   │
  *  └──────┴──────┴──────┴──────┴──────────┴──────┴──────┘
  *
  *          LEN = FCODE(1B) + DATA(nB) + CRC16(2B) = n + 3
  *          CRC 范围：从 LEN 到 DATA 末尾，共(LEN-1)字节
  *          SEQ 在 AP_UART_Send 内部统一自增，外部不涉足。
  *
  *          所有发送默认需要应答（ACK），应答超时在 TIM6 滴答中轮询。
  ******************************************************************************
  */
#ifndef __AP_UART_PROTOCOL_H__
#define __AP_UART_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                          协议帧宏定义                                       */
/* ========================================================================== */

#define AP_UART_STX             (0x02U)
#define AP_UART_ETX             (0x03U)

/** @brief  LEN 最小值（LEN 不包含自身，最小 = FCODE + CRC16 = 3） */
#define AP_UART_LEN_MIN         (3U)

/** @brief  DATA 区最大长度（约 124B） */
#define AP_UART_DATA_MAX        (121U)

/** @brief  整包最大长度 */
#define AP_UART_FRAME_MAX       (1 + 1 + 1 + 1 + AP_UART_DATA_MAX + 2 + 1)

/** @brief  TX 事件缓冲大小（环形缓冲，存放待发送的原始数据） */
#define AP_UART_TX_BUF_SIZE     (512U)

/** @brief  ACK 超时时间（ms） */
#define AP_UART_ACK_TIMEOUT_MS      (200U)

/** @brief  AP_UART_CheckTimeout 被调用的间隔（ms）
 *          当前 TIM6=1ms，每中断调用一次，此值为 1
 *          若改为每 10ms 调一次，设此值为 10 */
#define AP_UART_TIMEOUT_TICK_MS     (1U)

/** @brief  最大重传次数 */
#define AP_UART_RETRANS_MAX     (3U)

/* ========================================================================== */
/*                         偏移量宏（用于帧内寻址）                             */
/* ========================================================================== */

#define AP_UART_OFFSET_STX      (0)
#define AP_UART_OFFSET_SEQ      (1)
#define AP_UART_OFFSET_LEN      (2)
#define AP_UART_OFFSET_FCODE    (3)
#define AP_UART_OFFSET_DATA     (4)

/* ========================================================================== */
/*                          FCode 功能码枚举                                   */
/* ========================================================================== */

typedef enum {
    /* 系统类 */
    AP_FCODE_HEARTBEAT       = 0x01,

    /* 数据上报 */
    AP_FCODE_ADC_DATA        = 0x10,
    AP_FCODE_UV_PULSE        = 0x11,
    AP_FCODE_FIRE_ALARM      = 0x12,

    /* 控制指令 */
    AP_FCODE_SET_PARAM       = 0x20,
    AP_FCODE_GET_PARAM       = 0x21,

    /* 调试 */
    AP_FCODE_DBG_MSG         = 0xF0,
} AP_UART_FCode_t;

/* ========================================================================== */
/*                         数据结构定义                                        */
/* ========================================================================== */

/**
  * @brief  解析状态机步进
  */
typedef enum {
    AP_UART_STEP_STX = 0,
    AP_UART_STEP_LEN,
    AP_UART_STEP_DATA,
} AP_UART_ParseStep_t;

/**
  * @brief  TX 状态机状态
  */
typedef enum {
    AP_UART_TX_IDLE = 0,
    AP_UART_TX_SENDING,
    AP_UART_TX_WAIT_ACK,
} AP_UART_TxState_t;

/**
  * @brief  ACK 回复数据结构 — 由各 FCode 回调函数填充
  */
typedef struct {
    uint8_t   fcode;                  /* 回复的 FCODE（跟随原帧或使用 ACK） */
    uint8_t   data[AP_UART_DATA_MAX]; /* 回复数据区                        */
    uint16_t  data_len;               /* 回复数据长度（0=不回复）           */
} AP_UART_Ack_t;

/**
  * @brief  FCode 处理回调函数类型
  * @param  fcode:   收到的 FCODE
  * @param  data:    收到的 DATA 指针
  * @param  len:     收到的数据长度
  * @param  seq:     收到的 SEQ
  * @param  ack:     填充此结构体以回复；data_len=0 则不回复
  */
typedef void (*ap_uart_rx_handler_t)(uint8_t fcode, const uint8_t *data,
                                      uint16_t len, uint8_t seq,
                                      AP_UART_Ack_t *ack);

/* ========================================================================== */
/*                         外部 API                                           */
/* ========================================================================== */

/**
  * @brief  初始化串口协议模块
  *         清空 TX 缓冲，复位解析状态机
  */
void AP_UART_ProtocolInit(void);

/**
  * @brief  发送一帧数据（默认需要应答，SEQ 内部自增）
  * @param  fcode: 功能码
  * @param  data:  数据区指针（为 NULL 且 len=0 表示无数据）
  * @param  len:   数据区长度
  * @retval 0: 成功（已写入 TX 缓冲）
  * @retval -1: 参数错误或缓冲满
  */
int  AP_UART_Send(AP_UART_FCode_t fcode, const uint8_t *data, uint16_t len);

/**
  * @brief  TX 状态机（主循环周期调用）
  *         从 TX 缓冲取数据 → 组包 → 发送 → 等待 ACK
  */
void AP_UART_TxTask(void);

/**
  * @brief  RX 解析状态机（主循环周期调用）
  *         逐字节解析 STX→LEN→DATA，CRC 校验后分发
  */
void AP_UART_RxTask(void);

/**
  * @brief  ACK 超时检查（由 TIM6 的 task_10ms 调用）
  *         每 10ms 检查一次，超时则重传
  */
void AP_UART_CheckTimeout(void);

/**
  * @brief  查询 TX/RX 是否均空闲
  * @retval true: 空闲（无待发送/待应答帧，解析状态机在 STEP_STX）
  */
bool AP_UART_IsIdle(void);

#ifdef __cplusplus
}
#endif

#endif /* __AP_UART_PROTOCOL_H__ */
