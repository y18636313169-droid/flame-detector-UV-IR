/**
  ******************************************************************************
  * @file    ap_uart_protocol.c
  * @brief   AP 层串口通信协议实现
  *
  *          TX：Send 写入事件缓冲 → TxTask 组包发送 → 等待 ACK → 超时重传
  *          RX：RxTask 三步状态机解析 → CRC 校验 → FCode 分发
  *
  *          TX 事件缓冲格式（环形）：
  *            [entry_len(1B), fcode(1B), data0..dataN]
  *            entry_len = 1 + len(data)  （含 fcode 自身）
  *
  *          RX 帧绝对偏移（rx_frame[0] = STX）：
  *            [0]STX [1]SEQ [2]LEN [3]FCODE [4..]DATA [LEN+1]CRC_HI [LEN+2]CRC_LO [LEN+3]ETX
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ap_uart_protocol.h"
#include "hal_uart.h"
#include "crc16.h"
#include <string.h>

/* ========================================================================== */
/*                          私有宏                                             */
/* ========================================================================== */

/** @brief  调试打印开关 — 注释此行关闭所有协议调试输出 */
#define AP_UART_DEBUG_ENABLE

#ifdef AP_UART_DEBUG_ENABLE
#define DBG(fmt, ...)   BSP_UART_Printf("[UART] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

#define TX_MASK             (AP_UART_TX_BUF_SIZE - 1U)

/* 从绝对帧偏移计算 CRC/ETX 位置（基于 LEN 值，CRC 范围从 LEN 起算 len-1 字节） */
#define CRC_OFFSET_HI(len)  ((len) + 1)
#define CRC_OFFSET_LO(len)  ((len) + 2)
#define ETX_OFFSET(len)     ((len) + 3)

/* ========================================================================== */
/*                          私有数据                                           */
/* ========================================================================== */

static uint8_t           tx_buf[AP_UART_TX_BUF_SIZE];
static volatile uint16_t tx_write;
static volatile uint16_t tx_read;

static volatile AP_UART_TxState_t  tx_state = AP_UART_TX_IDLE;
static uint8_t                     tx_seq_counter;
static volatile uint16_t           tx_timeout_cnt;
static volatile uint8_t            tx_ack_flag;   /* WAIT_ACK 中收到匹配 ACK */

typedef struct {
    uint8_t   fcode;
    uint8_t   data[AP_UART_DATA_MAX];
    uint16_t  data_len;
    uint8_t   seq;
    volatile uint8_t  retrans_cnt;   /* ISR 中递增 */
} TxPending_t;
static TxPending_t  tx_pending;

static AP_UART_ParseStep_t rx_step = AP_UART_STEP_STX;
static uint8_t  rx_frame[AP_UART_FRAME_MAX];
static uint16_t rx_frame_len;

/* ========================================================================== */
/*                         内部函数声明                                        */
/* ========================================================================== */

static int      tx_buf_push(uint8_t fcode, const uint8_t *data, uint16_t data_len);
static int      tx_buf_check(void);                    /* 仅检查缓冲非空+长度合法 */
static int      tx_buf_pop(uint8_t *fcode, uint8_t *data, uint16_t *data_len);
static void     tx_buf_consume(void);
static void     tx_send_pending(void);
static void     tx_complete(void);
static void     process_rx_frame(void);

/* ========================================================================== */
/*                          CRC16 Modbus                                       */
/*          使用 modules/crc/crc16.h 中的共享 CRC16_Modbus                     */
/* ========================================================================== */

/* ========================================================================== */
/*                     TX 事件缓冲管理                                         */
/* ========================================================================== */

static int tx_buf_push(uint8_t fcode, const uint8_t *data, uint16_t data_len)
{
    uint16_t entry_len = AP_UART_LEN_MIN + data_len;   /* LEN_MIN(fcode+crc) + data */
    uint16_t total     = 1 + entry_len;                /* entry_len + payload */

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t used = (tx_write - tx_read) & TX_MASK;
    uint16_t free = AP_UART_TX_BUF_SIZE - used;
    if (free < total) {
        __set_PRIMASK(primask);
        return -1;
    }

    tx_buf[tx_write] = entry_len;
    tx_write = (tx_write + 1) & TX_MASK;

    tx_buf[tx_write] = fcode;
    tx_write = (tx_write + 1) & TX_MASK;

    if (data_len > 0 && data != NULL) {
        uint16_t l = data_len;
        uint16_t first = AP_UART_TX_BUF_SIZE - tx_write;
        if (first > l) first = l;
        memcpy(&tx_buf[tx_write], data, first);
        if (l > first) {
            memcpy(tx_buf, data + first, l - first);
        }
        tx_write = (tx_write + l) & TX_MASK;
    }

    __set_PRIMASK(primask);
    return 0;
}

/**
  * @brief  检查 TX 缓冲非空，且读指针指向的 entry_len 合法
  * @retval 0: 缓冲非空且数据合法
  * @retval -1: 缓冲空 / 长度超范围
  */
static int tx_buf_check(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (tx_write == tx_read) {  // 空
        __set_PRIMASK(primask);
        return -1;
    }

    uint8_t entry_len = tx_buf[tx_read];
    __set_PRIMASK(primask);

    if (entry_len < AP_UART_LEN_MIN || entry_len > AP_UART_LEN_MIN + AP_UART_DATA_MAX) {    // 长度小于最小值或最大值
        return -1;
    }
    return 0;
}

static int tx_buf_pop(uint8_t *fcode, uint8_t *data, uint16_t *data_len)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (tx_write == tx_read) {
        __set_PRIMASK(primask);
        return -1;
    }

    uint16_t pos = tx_read;
    uint16_t entry_len = tx_buf[pos];
    pos = (pos + 1) & TX_MASK;

    *fcode = tx_buf[pos];
    pos = (pos + 1) & TX_MASK;

    *data_len = entry_len - AP_UART_LEN_MIN;    
    /* 仅读取数据，不更新ptr 防止重传数据丢失 */
    if (*data_len > 0 && data != NULL) {
        uint16_t l = *data_len;
        uint16_t first = AP_UART_TX_BUF_SIZE - pos;
        if (first > l) first = l;
        memcpy(data, &tx_buf[pos], first);
        if (l > first) {
            memcpy(data + first, tx_buf, l - first);
        }
    }

    __set_PRIMASK(primask);
    return 0;
}

static void tx_buf_consume(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (tx_read != tx_write) {
        uint16_t entry_len = tx_buf[tx_read];
        tx_read = (tx_read + 1 + entry_len) & TX_MASK;  // 更新读指针，数据长度+1(len自身的1字节)
    }

    __set_PRIMASK(primask);
}

/* ========================================================================== */
/*                       TX 状态机核心                                        */
/* ========================================================================== */

static void tx_send_pending(void)
{
    uint8_t  frame[AP_UART_FRAME_MAX];
    uint16_t pos = 0;

    uint8_t  frame_len = AP_UART_LEN_MIN + tx_pending.data_len;

    frame[pos++] = AP_UART_STX;
    frame[pos++] = tx_pending.seq;
    frame[pos++] = frame_len;                /* LEN */
    frame[pos++] = tx_pending.fcode;         /* FCODE */
    if (tx_pending.data_len > 0) {
        memcpy(&frame[pos], tx_pending.data, tx_pending.data_len);
        pos += tx_pending.data_len;
    }
    uint16_t crc = CRC16_Modbus(&frame[AP_UART_OFFSET_LEN], frame_len - 1);
    frame[pos++] = (uint8_t)(crc >> 8);
    frame[pos++] = (uint8_t)(crc);
    frame[pos++] = AP_UART_ETX;

    BSP_UART_Write(BSP_UART_COM, frame, pos);
    DBG("TX send  fcode=0x%02X seq=%u len=%u",
        tx_pending.fcode, tx_pending.seq, pos);
}

static void tx_complete(void)
{
    DBG("TX done  fcode=0x%02X seq=%u", tx_pending.fcode, tx_pending.seq);
    tx_buf_consume();
    memset(&tx_pending, 0, sizeof(tx_pending));
    tx_state = AP_UART_TX_IDLE;
}

void AP_UART_TxTask(void)
{
    switch (tx_state) {

        /* ================================================================ */
        /*  IDLE: 仅检查缓冲非空且数据长度合法，不 pop                      */
        /* ================================================================ */
        case AP_UART_TX_IDLE:
            if (tx_buf_check() != 0) break;
            tx_state = AP_UART_TX_SENDING;
            break;

        /* ================================================================ */
        /*  SENDING: pop → SEQ++ → 组帧发送 → WAIT_ACK                     */
        /* ================================================================ */
        case AP_UART_TX_SENDING: {
            uint8_t  fcode;
            uint8_t  data[AP_UART_DATA_MAX];
            uint16_t data_len;

            if (tx_buf_pop(&fcode, data, &data_len) != 0) {     // 仅peek 不更新读指针 等发送完成收到ack后再更新
                tx_state = AP_UART_TX_IDLE;

                break;
            }

            tx_pending.fcode     = fcode;
            tx_pending.data_len  = data_len;
            tx_pending.retrans_cnt = 0;
            if (data_len > 0) {
                memcpy(tx_pending.data, data, data_len);
            }

            tx_seq_counter = (tx_seq_counter == 0xFF) ? 1 : tx_seq_counter + 1;     // 更新seq
            tx_pending.seq = tx_seq_counter;

            tx_send_pending();

            tx_ack_flag   = 0;
            tx_timeout_cnt = AP_UART_ACK_TIMEOUT_MS / AP_UART_TIMEOUT_TICK_MS;
            tx_state = AP_UART_TX_WAIT_ACK;
            break;
        }

        /* ================================================================ */
        /*  WAIT_ACK: check ack flag / 超时由 CheckTimeout 处理             */
        /* ================================================================ */
        case AP_UART_TX_WAIT_ACK:
            if (tx_ack_flag) {  // 由rx的接收解析置位
                tx_complete();
            }
            break;

        default:
            tx_state = AP_UART_TX_IDLE;
            break;
    }
}

/* ========================================================================== */
/*                       ACK 超时检查（TIM6 周期调用）                        */
/* ========================================================================== */

void AP_UART_CheckTimeout(void)
{
    if (tx_state != AP_UART_TX_WAIT_ACK) return;

    if (tx_timeout_cnt > 0) {
        tx_timeout_cnt--;
        return;
    }

    tx_pending.retrans_cnt++;
    DBG("TX timeout fcode=0x%02X seq=%u retrans=%u/%u",
        tx_pending.fcode, tx_pending.seq,
        tx_pending.retrans_cnt, AP_UART_RETRANS_MAX);
    if (tx_pending.retrans_cnt <= AP_UART_RETRANS_MAX) {
        tx_send_pending();
        tx_timeout_cnt = AP_UART_ACK_TIMEOUT_MS / AP_UART_TIMEOUT_TICK_MS;
    } else {
        tx_complete();
    }
}

/* ========================================================================== */
/*                       RX 解析状态机                                        */
/* ========================================================================== */
/*
 * RX 解析策略：不提前 consume 任何字节，
 * 整帧校验通过后一次性 consume，保证偏移从 STX 算起。
 *
 * BSP_UART 缓冲中的帧布局：
 *   [0]STX [1]SEQ [2]LEN [3]FCODE [4..]DATA [...LEN..LEN+1]CRC [LEN+2]ETX
 */

void AP_UART_RxTask(void)
{
    uint8_t byte;

    switch (rx_step) {

        /* ================================================================ */
        /*  STEP_STX: 等待帧头 0x02 — 不 consume，找到后直接切 LEN          */
        /* ================================================================ */
        case AP_UART_STEP_STX:
            if (BSP_UART_Peek(BSP_UART_COM, &byte, 0)) {
                if (byte == AP_UART_STX) {
                    rx_frame_len = 0;
                    memset(rx_frame, 0, sizeof(rx_frame));
                    rx_step = AP_UART_STEP_LEN;    /* 不 consume，保留 STX */
                } else {
                    DBG("RX drop  0x%02X (expect 0x02)", byte);
                    BSP_UART_Consume(BSP_UART_COM, 1);  // 找下一个头帧
                }
            }
            break;

        /* ================================================================ */
        /*  STEP_LEN: 等 STX+SEQ+LEN 到齐，读 offset=2 的 LEN 字段         */
        /*  仍不 consume，保留整帧在缓冲中                                   */
        /* ================================================================ */
        case AP_UART_STEP_LEN:
            if (BSP_UART_GetRxDataLen(BSP_UART_COM) < 3) {
                break;  /* STX + SEQ + LEN 未到齐 */
            }

            /* Peek offset 2 获取 LEN 字节（0=STX, 1=SEQ, 2=LEN） */
            BSP_UART_Peek(BSP_UART_COM, &byte, 2);
            if (byte < AP_UART_LEN_MIN || byte > (AP_UART_LEN_MIN + AP_UART_DATA_MAX)) {
                DBG("RX badlen %u (valid %u~%u)", byte, AP_UART_LEN_MIN,
                    AP_UART_LEN_MIN + AP_UART_DATA_MAX);
                BSP_UART_Consume(BSP_UART_COM, 1);
                rx_step = AP_UART_STEP_STX;
                break;
            }

            rx_frame_len = byte;
            rx_step = AP_UART_STEP_DATA;
            break;

        /* ================================================================ */
        /*  STEP_DATA: 等整帧到齐 → PeekPacket → CRC → ETX → Consume        */
        /* ================================================================ */
        case AP_UART_STEP_DATA: {
            /* 整帧长度 = STX(1)+SEQ(1)+LEN(1)+FCODE(1)+DATA(n)+CRC(2)+ETX(1) = rx_frame_len+4 */
            uint16_t total = rx_frame_len + 4U;

            if (BSP_UART_GetRxDataLen(BSP_UART_COM) < total) {
                break;
            }

            BSP_UART_PeekPacket(BSP_UART_COM, rx_frame, 0, total);

            /* CRC 范围：从 LEN 起，LEN+FCODE+DATA = 共 len-1 字节 */
            uint16_t crc_calc = CRC16_Modbus(&rx_frame[AP_UART_OFFSET_LEN],
                                              rx_frame_len - 1);
            uint16_t crc_recv = (uint16_t)rx_frame[CRC_OFFSET_HI(rx_frame_len)] << 8
                              | (uint16_t)rx_frame[CRC_OFFSET_LO(rx_frame_len)];

            if (crc_calc != crc_recv) {
                DBG("RX badcrc calc=0x%04X recv=0x%04X", crc_calc, crc_recv);
                BSP_UART_Consume(BSP_UART_COM, 1);  /* 丢 STX 重同步 */
                rx_step = AP_UART_STEP_STX;
                break;
            }

            /* ETX 检查 */
            if (rx_frame[ETX_OFFSET(rx_frame_len)] != AP_UART_ETX) {
                DBG("RX badetx got=0x%02X", rx_frame[ETX_OFFSET(rx_frame_len)]);
                BSP_UART_Consume(BSP_UART_COM, 1);
                rx_step = AP_UART_STEP_STX;
                break;
            }

            /* 校验全部通过，一次性 consume 整帧 */
            BSP_UART_Consume(BSP_UART_COM, total);
            DBG("RX recv  fcode=0x%02X seq=%u len=%u",
                rx_frame[AP_UART_OFFSET_FCODE],
                rx_frame[AP_UART_OFFSET_SEQ],
                total);

            process_rx_frame();

            rx_step = AP_UART_STEP_STX;
            break;
        }

        default:
            rx_step = AP_UART_STEP_STX;
            break;
    }
}

/* ========================================================================== */
/*                  ACK 回复发送（直接发送，不经过 TX 缓冲）                   */
/* ========================================================================== */

static void send_ack_frame(uint8_t peer_seq, uint8_t fcode,
                           const uint8_t *ack_data, uint16_t ack_len)
{
    uint8_t  frame[AP_UART_FRAME_MAX];
    uint16_t pos = 0;
    uint8_t  frame_len = AP_UART_LEN_MIN + ack_len;

    frame[pos++] = AP_UART_STX;
    frame[pos++] = peer_seq;
    frame[pos++] = frame_len;
    frame[pos++] = fcode;
    if (ack_len > 0 && ack_data != NULL) {
        memcpy(&frame[pos], ack_data, ack_len);
        pos += ack_len;
    }
    uint16_t crc = CRC16_Modbus(&frame[AP_UART_OFFSET_LEN], frame_len - 1);
    frame[pos++] = (uint8_t)(crc >> 8);
    frame[pos++] = (uint8_t)(crc);
    frame[pos++] = AP_UART_ETX;

    BSP_UART_Write(BSP_UART_COM, frame, pos);
}

/* ========================================================================== */
/*                 FCode 回调函数实现                                          */
/* ========================================================================== */

static void rx_get_param(uint8_t fcode, const uint8_t *data, uint16_t len,
                         uint8_t seq, AP_UART_Ack_t *ack)
{
    (void)fcode;
    (void)seq;
    if (len < 1) return;
    uint8_t  param_id = data[0];
    uint16_t value    = 0;                    /* TODO: 读取参数值 */

    ack->data[0]  = param_id;
    ack->data[1]  = (uint8_t)(value >> 8);
    ack->data[2]  = (uint8_t)(value);
    ack->data_len = 3;
}

static void rx_set_param(uint8_t fcode, const uint8_t *data, uint16_t len,
                         uint8_t seq, AP_UART_Ack_t *ack)
{
    (void)fcode;
    (void)seq;
    if (len < 1) return;
    /* TODO: 写入参数 */
    ack->data[0]  = data[0];                  /* 参数 ID */
    ack->data[1]  = 0;                        /* 0 = 成功 */
    ack->data_len = 2;
}

static void rx_heartbeat(uint8_t fcode, const uint8_t *data, uint16_t len,
                         uint8_t seq, AP_UART_Ack_t *ack)
{
    (void)fcode; (void)data; (void)len; (void)seq;
    ack->data_len = 0;                        /* 仅帧头+CRC，无数据 */
}

/* ========================================================================== */
/*                       FCode 分发表                                         */
/* ========================================================================== */

typedef struct {
    uint8_t              fcode;
    ap_uart_rx_handler_t handler;
} AP_UART_RxEntry_t;

static const AP_UART_RxEntry_t rx_table[] = {
    { AP_FCODE_GET_PARAM,  rx_get_param  },
    { AP_FCODE_SET_PARAM,  rx_set_param  },
    { AP_FCODE_HEARTBEAT,  rx_heartbeat  },
};
#define RX_TABLE_SIZE  (sizeof(rx_table) / sizeof(rx_table[0]))

/* ========================================================================== */
/*                       RX 帧处理                                           */
/* ========================================================================== */

static void process_rx_frame(void)
{
    /* rx_frame[0]=STX, [1]=SEQ, [2]=LEN, [3]=FCODE, [4..]=DATA */
    uint8_t  peer_seq = rx_frame[AP_UART_OFFSET_SEQ];
    uint8_t  len_val  = rx_frame[AP_UART_OFFSET_LEN];
    uint16_t data_len = len_val - AP_UART_LEN_MIN;
    uint8_t  fcode    = rx_frame[AP_UART_OFFSET_FCODE];
    uint8_t *data     = &rx_frame[AP_UART_OFFSET_DATA];

    /* ================================================================ */
    /*  ACK 帧 — 按 SEQ 匹配，由 TxTask 在 WAIT_ACK 消费                */
    /* ================================================================ */
    // if (fcode == AP_FCODE_ACK) {
        if (tx_state == AP_UART_TX_WAIT_ACK
            && peer_seq == tx_pending.seq) {
            DBG("TX ack   fcode=0x%02X seq=%u", fcode, peer_seq);
            tx_ack_flag = 1;
            return;
        }
    // }

    /* ================================================================ */
    /*  非 ACK 帧 — 查 FCode 表分发，默认回复 data[0]=0                 */
    /* ================================================================ */

    AP_UART_Ack_t ack;
    ack.fcode    = fcode;
    ack.data[0]  = 1;                        /* 未注册 FCODE 默认回复 1 */
    ack.data_len = 1;

    for (uint8_t i = 0; i < RX_TABLE_SIZE; i++) {
        if (rx_table[i].fcode == fcode) {
            ack.data[0] = 0;                 /* 已注册 FCODE，先置 0 由 handler 覆盖 */
            rx_table[i].handler(fcode, data, data_len, peer_seq, &ack);
            break;
        }
    }

    DBG("RX reply fcode=0x%02X seq=%u data_len=%u",
        ack.fcode, peer_seq, ack.data_len);
    send_ack_frame(peer_seq, ack.fcode, ack.data, ack.data_len);
}

/* ========================================================================== */
/*                        公有 API                                            */
/* ========================================================================== */

void AP_UART_ProtocolInit(void)
{
    tx_write     = 0;
    tx_read      = 0;
    tx_state     = AP_UART_TX_IDLE;
    tx_seq_counter = 0;
    tx_ack_flag  = 0;
    memset(&tx_pending, 0, sizeof(tx_pending));

    rx_step      = AP_UART_STEP_STX;
    rx_frame_len = 0;
}

int AP_UART_Send(AP_UART_FCode_t fcode, const uint8_t *data, uint16_t len)
{
    if (len > AP_UART_DATA_MAX) {
        return -1;
    }
    return tx_buf_push((uint8_t)fcode, data, len);
}

bool AP_UART_IsIdle(void)
{
    bool tx_idle = (tx_state == AP_UART_TX_IDLE) && (tx_write == tx_read);
    bool rx_idle = (rx_step == AP_UART_STEP_STX);
    return tx_idle && rx_idle;
}
