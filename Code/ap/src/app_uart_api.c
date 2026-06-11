#include "app_uart_api.h"
#include "ht32l6/inc/hal_uart.h"
#include "app_main.h"

typedef enum
{
  USART_TX_STEP_IDLE,
  USART_TX_STEP_MEDIATOR,
  USART_TX_STEP_WAITACK,
  USART_TX_STEP_RTNS,
} USART_TX_EVT_STEP_E;

static USART_TX_EVT_STEP_E usart_tx_step = USART_TX_STEP_IDLE;

static bool wait_timeout = false;

bool hub_log_enable = false;

void XW_hub_api_usart_tx_event_task(void)
{
  switch (usart_tx_step)
  {
  case USART_TX_STEP_IDLE:
    if (XW_hub_api_is_usart_tx_buf_notempty() && XW_hub_api_get_usart_tx_event() == 0)  // 非空且长度合法
    {
      usart_tx_step = USART_TX_STEP_MEDIATOR;
    }
    break;
  case USART_TX_STEP_MEDIATOR:
    if (XW_hub_api_usart_tx_event_mediator() == 0)
    {
      start_wait_ack_Timer(SWTMR_MS_TO_CNT(200), &wait_timeout);
      usart_tx_step = USART_TX_STEP_WAITACK;
    }
    else
    {
      usart_tx_step = USART_TX_STEP_IDLE;
      XW_hub_api_update_read_ptr();
    }
    break;
  case USART_TX_STEP_WAITACK:
    if (wait_timeout)
    {
      usart_tx_step = USART_TX_STEP_RTNS; // 查找应答报文超时
      // APP_PRINTF("wait ack timeout\r\n");
      // XW_hub_api_update_read_ptr();
    }
    else
    {
      if (check_usart_active_evt_is_finish((HUB_API_USART_MSG_FCODE_E)*XW_hub_api_pop_usart_buf_adr(STEP_FCODE - STEP_LEN)))  // fcode收到ack消息 finish
      { // TODO查找应答报文 更新tx缓存读指针
        stop_wait_ack_Timer(&wait_timeout);
        XW_hub_api_update_read_ptr();
        usart_tx_step = USART_TX_STEP_IDLE;
      }
    }
    break;
  case USART_TX_STEP_RTNS:
    // TODO 重传
    if (check_usart_evt_retrans_end((HUB_API_USART_MSG_FCODE_E)*XW_hub_api_pop_usart_buf_adr(STEP_FCODE - STEP_LEN))) // TODO: || 内存不足时 无需重传，直接更新读指针
    {
      clean_retrans_ack_flag((HUB_API_USART_MSG_FCODE_E)*XW_hub_api_pop_usart_buf_adr(STEP_FCODE - STEP_LEN));
      // usart_tx_step = USART_TX_STEP_IDLE;
      XW_hub_api_update_read_ptr();
    }
    // XW_hub_api_usart_retry_usart_tx_event(); // 未更新指针 切换IDLE重新发送视为重传
    usart_tx_step = USART_TX_STEP_IDLE;
    break;
  default:
    usart_tx_step = USART_TX_STEP_IDLE;
    break;
  }
  // if (state_last != usart_tx_step)
  // {
  //   APP_PRINTF("t s:%d->%d\r\n", state_last, usart_tx_step);
  //   state_last = usart_tx_step;
  // }
  // XW_hub_api_get_usart_tx_event();
}

void XW_hub_api_task(void)
{
  XW_hub_api_usart_tx_event_task();
  XW_hub_api_usart_listen();
}

uint8_t XW_hub_tx_IsIdle(void)
{
//  APP_PRINTF("step:%d,idle:%d\r\n", (usart_tx_step == USART_TX_STEP_IDLE));
  return (usart_tx_step == USART_TX_STEP_IDLE);
}

uint8_t XW_hub_IsIdle(void)
{
  bool isTxIdle = XW_hub_tx_IsIdle() && !XW_hub_api_is_usart_tx_buf_notempty();

  bool isRxIdle = XW_hub_rx_IsIdle() && hal_uart_is_rx_buf_empty();

//  static bool isTxIdle_last = 1,isRxIdle_last = 1;
//  if (isTxIdle_last != isTxIdle || isRxIdle_last != isRxIdle)
//  {
//    APP_PRINTF("HUB:tx%d,rx%d\r\n", isTxIdle, isRxIdle);
//  }
//  isTxIdle_last = isTxIdle;
//  isRxIdle_last = isRxIdle;
  // 如果两个状态机都空闲，返回 true
  return (isTxIdle && isRxIdle);
}


