/**
  ******************************************************************************
  * @file    hal_uart.c
  * @brief   BSP UART 抽象层 — TX DMA + RX DMA CIRCULAR 环形缓冲驱动
  *
  *          TX (COM): 应用写入环形缓冲 → DMA 自动搬运。
  *          TX (DBG): BSP_UART_Printf 直接轮询发送，不依赖 DMA。
  *          RX: DMA CIRCULAR 模式持续填充 → 应用通过 Read/Peek/Consume 读取。
  *
  *          串口资源：
  *            DBG (USART1) — PA9/PA10, DMA1_CH4(TX)/CH5(RX)
  *            COM (USART2) — PA2/PA3,  DMA1_CH7(TX)/CH6(RX)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hal_uart.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* 引用 CubeMX 生成的 UART/DMA 句柄（定义在 usart.c） */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* Private define ------------------------------------------------------------*/

#if (BSP_UART_TX_BUF_SIZE & (BSP_UART_TX_BUF_SIZE - 1U)) != 0
#error "BSP_UART_TX_BUF_SIZE must be a power of 2!"
#endif
#if (BSP_UART_RX_BUF_SIZE & (BSP_UART_RX_BUF_SIZE - 1U)) != 0
#error "BSP_UART_RX_BUF_SIZE must be a power of 2!"
#endif

#define TX_MASK                 (BSP_UART_TX_BUF_SIZE - 1U)
#define RX_MASK                 (BSP_UART_RX_BUF_SIZE - 1U)

#ifndef UART_MIN
#define UART_MIN(a, b)          ((a) < (b) ? (a) : (b))
#endif

#define PRINTF_BUF_SIZE         (256U)

/* ========================================================================== */
/*                      UART 端口控制块                                        */
/* ========================================================================== */

typedef struct {
    UART_HandleTypeDef  *handle;
    DMA_HandleTypeDef   *hdma_tx;
    DMA_HandleTypeDef   *hdma_rx;

    uint8_t             tx_buf[BSP_UART_TX_BUF_SIZE];
    volatile uint16_t   tx_in;
    volatile uint16_t   tx_out;
    volatile bool       tx_busy;
    volatile uint16_t   tx_dma_len;       /* Length owned by the active TX DMA. */
    volatile bool       tx_fault;

    /* RX DMA CIRCULAR 环形缓冲 */
    uint8_t             rx_buf[BSP_UART_RX_BUF_SIZE];
    volatile uint32_t   rx_rd_total;       /* Monotonic byte count consumed by main. */
    volatile uint32_t   rx_wrap_count;     /* Full DMA rounds, updated by RX TC ISR. */
    volatile uint32_t   rx_last_total;
    volatile bool       rx_overflow;
    volatile bool       rx_restart_needed;

    uint8_t             initialized;
} BSP_UART_Ctrl_t;

static BSP_UART_Ctrl_t uart_ctrl[BSP_UART_NUM] = {
    [BSP_UART_COM] = { .handle = NULL },
    [BSP_UART_DBG] = { .handle = NULL },
};

/* ========================================================================== */
/*                     内部辅助函数                                            */
/* ========================================================================== */

static inline bool is_valid_id(BSP_UART_Id_t id)
{
    return (id >= 0 && id < BSP_UART_NUM);
}

static inline BSP_UART_Ctrl_t *get_ctrl(BSP_UART_Id_t id)
{
    return &uart_ctrl[id];
}

/**
 * @brief Return the monotonic RX DMA producer count.
 * @note  The TC callback counts complete DMA rounds. The last-total correction
 *        covers the short interval after NDTR reloads but before the TC ISR runs.
 */
static uint32_t rx_dma_produced(BSP_UART_Ctrl_t *ctrl)
{
    uint32_t wraps_before;
    uint32_t wraps_after;
    uint32_t total;
    uint16_t ndtr;
    uint16_t pos;

    if (ctrl->hdma_rx == NULL) return 0U;
    do {
        wraps_before = ctrl->rx_wrap_count;
        ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(ctrl->hdma_rx);
        wraps_after = ctrl->rx_wrap_count;
    } while (wraps_before != wraps_after);

    pos = (uint16_t)(BSP_UART_RX_BUF_SIZE - ndtr);
    if (pos >= BSP_UART_RX_BUF_SIZE) pos = 0U;
    total = wraps_after * BSP_UART_RX_BUF_SIZE + pos;
    if (total < ctrl->rx_last_total) {
        total += BSP_UART_RX_BUF_SIZE;
    }
    ctrl->rx_last_total = total;
    return total;
}

static uint16_t rx_data_avail(BSP_UART_Ctrl_t *ctrl)
{
    uint32_t produced = rx_dma_produced(ctrl);
    uint32_t avail = produced - ctrl->rx_rd_total;

    /* DMA has lapped the consumer, so discard the overwritten byte stream. */
    if (avail >= BSP_UART_RX_BUF_SIZE) {
        ctrl->rx_rd_total = produced;
        ctrl->rx_overflow = true;
        return 0U;
    }
    return (uint16_t)avail;
}

static inline uint8_t *rx_buf_at(BSP_UART_Ctrl_t *ctrl, uint16_t offset)
{
    return &ctrl->rx_buf[(ctrl->rx_rd_total + offset) & RX_MASK];
}

static void start_tx_dma(BSP_UART_Ctrl_t *ctrl)
{
    uint16_t avail = (ctrl->tx_in - ctrl->tx_out) & TX_MASK;
    if (avail == 0) {
        ctrl->tx_busy = false;
        return;
    }
    uint16_t chunk = UART_MIN(avail, BSP_UART_TX_BUF_SIZE - ctrl->tx_out);
    ctrl->tx_dma_len = chunk;
    ctrl->tx_busy = true;
    if (HAL_UART_Transmit_DMA(ctrl->handle, ctrl->tx_buf + ctrl->tx_out, chunk) != HAL_OK) {
        /* Keep queued bytes intact; the caller observes tx_fault and aborts. */
        ctrl->tx_dma_len = 0U;
        ctrl->tx_busy = false;
        ctrl->tx_fault = true;
        return;
    }
    __HAL_UART_DISABLE_IT(ctrl->handle, UART_IT_TC);
}

static BSP_UART_Ctrl_t *find_ctrl_by_handle(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < BSP_UART_NUM; i++) {
        if (uart_ctrl[i].handle == huart) {
            return &uart_ctrl[i];
        }
    }
    return NULL;
}

/* ========================================================================== */
/*                         公有 API 实现                                       */
/* ========================================================================== */

int BSP_UART_Init(BSP_UART_Id_t id)
{
    if (!is_valid_id(id)) return BSP_UART_ERROR;
    BSP_UART_Ctrl_t *ctrl = get_ctrl(id);
    if (ctrl->initialized) return BSP_UART_OK;

    if (id == BSP_UART_DBG) {
        ctrl->handle  = &huart1;
        ctrl->hdma_tx = &hdma_usart1_tx;
        ctrl->hdma_rx = &hdma_usart1_rx;
    } else {
        ctrl->handle  = &huart2;
        ctrl->hdma_tx = &hdma_usart2_tx;
        ctrl->hdma_rx = &hdma_usart2_rx;
    }

    if (ctrl->handle == NULL || ctrl->hdma_rx == NULL) {
        return BSP_UART_ERROR;
    }

    ctrl->tx_in     = 0;
    ctrl->tx_out    = 0;
    ctrl->tx_busy   = false;
    ctrl->tx_dma_len = 0U;
    ctrl->tx_fault = false;
    ctrl->rx_rd_total = 0U;
    ctrl->rx_wrap_count = 0U;
    ctrl->rx_last_total = 0U;
    ctrl->rx_overflow = false;
    ctrl->rx_restart_needed = false;

    if (HAL_UART_Receive_DMA(ctrl->handle, ctrl->rx_buf, BSP_UART_RX_BUF_SIZE) != HAL_OK) {
        return BSP_UART_ERROR;
    }

    ctrl->initialized = 1;
    return BSP_UART_OK;
}

int BSP_UART_DeInit(BSP_UART_Id_t id)
{
    if (!is_valid_id(id) || !get_ctrl(id)->initialized) return BSP_UART_ERROR;
    BSP_UART_Ctrl_t *ctrl = get_ctrl(id);
    HAL_UART_DMAStop(ctrl->handle);
    HAL_UART_DeInit(ctrl->handle);
    ctrl->initialized = 0;
    return BSP_UART_OK;
}

uint16_t BSP_UART_Write(BSP_UART_Id_t id, const uint8_t *data, uint16_t len)
{
    if (!is_valid_id(id) || !get_ctrl(id)->initialized || data == NULL || len == 0) return 0;
    BSP_UART_Ctrl_t *ctrl = get_ctrl(id);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t used = (ctrl->tx_in - ctrl->tx_out) & TX_MASK;
    /* Keep one slot unused so equal indices unambiguously mean empty. */
    uint16_t free = (BSP_UART_TX_BUF_SIZE - 1U) - used;
    if (free < len) len = free;
    if (len == 0) { __set_PRIMASK(primask); return 0; }

    uint16_t l = UART_MIN(len, BSP_UART_TX_BUF_SIZE - ctrl->tx_in);
    memcpy(ctrl->tx_buf + ctrl->tx_in, data, l);
    if (len > l) memcpy(ctrl->tx_buf, data + l, len - l);
    ctrl->tx_in = (ctrl->tx_in + len) & TX_MASK;

    bool need_start = !ctrl->tx_busy;
    if (need_start) ctrl->tx_busy = true;
    __set_PRIMASK(primask);

    if (need_start) start_tx_dma(ctrl);
    return len;
}

int BSP_UART_WriteBlock(BSP_UART_Id_t id, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    uint16_t offset = 0U;
    uint32_t tick_start;

    if (!is_valid_id(id) || !get_ctrl(id)->initialized || data == NULL || len == 0) return BSP_UART_ERROR;
    tick_start = HAL_GetTick();
    while (offset < len) {
        offset = (uint16_t)(offset + BSP_UART_Write(id, data + offset,
                                                   (uint16_t)(len - offset)));
        if (BSP_UART_TxFaulted(id)) {
            BSP_UART_ClearTxBuf(id);
            return BSP_UART_ERROR;
        }
        if (timeout_ms > 0U && (uint32_t)(HAL_GetTick() - tick_start) >= timeout_ms) {
            BSP_UART_ClearTxBuf(id);
            return BSP_UART_TIMEOUT;
        }
    }
    while (!BSP_UART_IsTxComplete(id)) {
        if (BSP_UART_TxFaulted(id)) {
            BSP_UART_ClearTxBuf(id);
            return BSP_UART_ERROR;
        }
        if (timeout_ms > 0U && (uint32_t)(HAL_GetTick() - tick_start) >= timeout_ms) {
            BSP_UART_ClearTxBuf(id);
            return BSP_UART_TIMEOUT;
        }
    }
    return BSP_UART_OK;
}

bool BSP_UART_IsTxComplete(BSP_UART_Id_t id)
{
    if (!is_valid_id(id) || !get_ctrl(id)->initialized) return false;
    BSP_UART_Ctrl_t *ctrl = get_ctrl(id);
    return (!ctrl->tx_busy) && (ctrl->tx_in == ctrl->tx_out);
}

bool BSP_UART_TxFaulted(BSP_UART_Id_t id)
{
    bool fault;
    uint32_t primask;

    if (!is_valid_id(id) || !get_ctrl(id)->initialized) return true;
    primask = __get_PRIMASK();
    __disable_irq();
    fault = get_ctrl(id)->tx_fault;
    get_ctrl(id)->tx_fault = false;
    __set_PRIMASK(primask);
    return fault;
}

uint16_t BSP_UART_Read(BSP_UART_Id_t id, uint8_t *buf, uint16_t len)
{
    if (!is_valid_id(id) || !get_ctrl(id)->initialized || buf == NULL || len == 0) return 0;
    BSP_UART_Ctrl_t *ctrl = get_ctrl(id);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t avail = rx_data_avail(ctrl);
    if (avail == 0) { __set_PRIMASK(primask); return 0; }
    if (len > avail) len = avail;

    uint16_t rd_idx = (uint16_t)(ctrl->rx_rd_total & RX_MASK);
    uint16_t l = UART_MIN(len, BSP_UART_RX_BUF_SIZE - rd_idx);
    memcpy(buf, ctrl->rx_buf + rd_idx, l);
    if (len > l) memcpy(buf + l, ctrl->rx_buf, len - l);
    ctrl->rx_rd_total += len;

    __set_PRIMASK(primask);
    return len;
}

uint16_t BSP_UART_GetRxDataLen(BSP_UART_Id_t id)
{
    if (!is_valid_id(id) || !get_ctrl(id)->initialized) return 0;
    return rx_data_avail(get_ctrl(id));
}

bool BSP_UART_RxOverflowed(BSP_UART_Id_t id)
{
    bool overflow;
    uint32_t primask;
    BSP_UART_Ctrl_t *ctrl;

    if (!is_valid_id(id) || !get_ctrl(id)->initialized) return true;
    ctrl = get_ctrl(id);

    /* If error recovery could not restart RX in ISR, retry from the main loop. */
    if (ctrl->rx_restart_needed) {
        ctrl->rx_rd_total = 0U;
        ctrl->rx_wrap_count = 0U;
        ctrl->rx_last_total = 0U;
        if (HAL_UART_Receive_DMA(ctrl->handle, ctrl->rx_buf,
                                 BSP_UART_RX_BUF_SIZE) == HAL_OK) {
            ctrl->rx_restart_needed = false;
        } else {
            ctrl->rx_overflow = true;
        }
    }
    primask = __get_PRIMASK();
    __disable_irq();
    (void)rx_data_avail(ctrl);
    overflow = ctrl->rx_overflow;
    ctrl->rx_overflow = false;
    __set_PRIMASK(primask);
    return overflow;
}

bool BSP_UART_Peek(BSP_UART_Id_t id, void *out, uint16_t offset)
{
    if (!is_valid_id(id) || !get_ctrl(id)->initialized || out == NULL) return false;
    BSP_UART_Ctrl_t *ctrl = get_ctrl(id);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t avail = rx_data_avail(ctrl);
    if (offset >= avail) { __set_PRIMASK(primask); return false; }

    *(uint8_t *)out = *rx_buf_at(ctrl, offset);
    __set_PRIMASK(primask);
    return true;
}

uint16_t BSP_UART_PeekPacket(BSP_UART_Id_t id, void *out, uint16_t offset, uint16_t len)
{
    if (!is_valid_id(id) || !get_ctrl(id)->initialized || out == NULL || len == 0) return 0;
    BSP_UART_Ctrl_t *ctrl = get_ctrl(id);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t avail = rx_data_avail(ctrl);
    if (offset + len > avail) {
        if (offset >= avail) { __set_PRIMASK(primask); return 0; }
        len = avail - offset;
    }

    uint16_t start = (uint16_t)((ctrl->rx_rd_total + offset) & RX_MASK);
    uint16_t l = UART_MIN(len, BSP_UART_RX_BUF_SIZE - start);
    memcpy(out, ctrl->rx_buf + start, l);
    if (len > l) memcpy((uint8_t *)out + l, ctrl->rx_buf, len - l);

    __set_PRIMASK(primask);
    return len;
}

void BSP_UART_Consume(BSP_UART_Id_t id, uint16_t len)
{
    if (!is_valid_id(id) || !get_ctrl(id)->initialized || len == 0) return;
    BSP_UART_Ctrl_t *ctrl = get_ctrl(id);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t avail = rx_data_avail(ctrl);
    if (len > avail) len = avail;
    ctrl->rx_rd_total += len;

    __set_PRIMASK(primask);
}

void BSP_UART_ClearRxBuf(BSP_UART_Id_t id)
{
    if (!is_valid_id(id) || !get_ctrl(id)->initialized) return;
    BSP_UART_Ctrl_t *ctrl = get_ctrl(id);
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* RX DMA owns rx_buf continuously; clearing only advances the consumer. */
    ctrl->rx_rd_total = rx_dma_produced(ctrl);
    __set_PRIMASK(primask);
}

void BSP_UART_ClearTxBuf(BSP_UART_Id_t id)
{
    if (!is_valid_id(id) || !get_ctrl(id)->initialized) return;
    BSP_UART_Ctrl_t *ctrl = get_ctrl(id);
    uint32_t primask;

    /* Abort first because DMA may still be reading from tx_buf. */
    (void)HAL_UART_AbortTransmit(ctrl->handle);
    primask = __get_PRIMASK();
    __disable_irq();
    ctrl->tx_out = ctrl->tx_in;
    ctrl->tx_busy = false;
    ctrl->tx_dma_len = 0U;
    memset(ctrl->tx_buf, 0, BSP_UART_TX_BUF_SIZE);
    __set_PRIMASK(primask);
}

/* ========================================================================== */
/*                      Printf — DBG 串口轮询发送                             */
/*          使用 HAL_UART_Transmit 阻塞发送，不依赖 DMA，防止 TX DMA 卡死     */
/* ========================================================================== */

int BSP_UART_Printf(const char *fmt, ...)
{
    char buf[PRINTF_BUF_SIZE];
    int  ret;
    va_list args;

    va_start(args, fmt);
    ret = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (ret > 0) {
        uint16_t len = (ret >= (int)sizeof(buf)) ? (uint16_t)(sizeof(buf) - 1U) : (uint16_t)ret;
        /* 直接轮询发送 DBG 口（USART1），不依赖 DMA */
        HAL_UART_Transmit(uart_ctrl[BSP_UART_DBG].handle, (uint8_t *)buf, len, HAL_MAX_DELAY);
    }
    return ret;
}

/* ========================================================================== */
/*                     HAL 回调覆盖                                            */
/* ========================================================================== */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    BSP_UART_Ctrl_t *ctrl = find_ctrl_by_handle(huart);
    if (ctrl == NULL) { __set_PRIMASK(primask); return; }

    uint16_t done = ctrl->tx_dma_len;
    if (done == 0) { ctrl->tx_busy = false; __set_PRIMASK(primask); return; }

    /* Advance exactly this DMA transfer; later queued bytes belong to next DMA. */
    ctrl->tx_dma_len = 0U;
    ctrl->tx_out = (ctrl->tx_out + done) & TX_MASK;
    bool more = ((ctrl->tx_in - ctrl->tx_out) & TX_MASK) > 0;
    if (!more) ctrl->tx_busy = false;
    __set_PRIMASK(primask);

    if (more) start_tx_dma(ctrl);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_Ctrl_t *ctrl = find_ctrl_by_handle(huart);
    if (ctrl != NULL) {
        ctrl->rx_wrap_count++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_Ctrl_t *ctrl = find_ctrl_by_handle(huart);
    if (ctrl == NULL) return;

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    (void)HAL_UART_DMAStop(huart);

    /* A UART error invalidates a partial TX frame and the current RX stream. */
    /* Preserve an earlier unconsumed TX fault if another UART error follows. */
    ctrl->tx_fault = ctrl->tx_fault || ctrl->tx_busy ||
                     (ctrl->tx_in != ctrl->tx_out);
    ctrl->tx_out = ctrl->tx_in;
    ctrl->tx_busy = false;
    ctrl->tx_dma_len = 0U;
    ctrl->rx_rd_total = 0U;
    ctrl->rx_wrap_count = 0U;
    ctrl->rx_last_total = 0U;
    ctrl->rx_overflow = true;

    ctrl->rx_restart_needed =
        (HAL_UART_Receive_DMA(huart, ctrl->rx_buf, BSP_UART_RX_BUF_SIZE) != HAL_OK);
}
