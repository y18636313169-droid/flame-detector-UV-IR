#include "app_uart_api_sub.h"
#include "device.h"


bool check_hub_event_need_rtns(void)//TODO 新增入参fcode需要判断
{
  bool bret = false;
  unsigned char rtns = 0;
  unsigned char flag = 0;
  unsigned char enable_retrans = 0;
  getSinkApiParam(SINKAPIEvtA0TNS, (unsigned char *)&rtns);
  getSinkApiParam(SINKAPIEvtA0AckFlag, (unsigned char *)&flag);
  getSinkApiParam(SINKAPIEnableRetrans, (unsigned char *)&enable_retrans);
  if((rtns < 4 && flag) && (enable_retrans)){
      bret = true;
  }
  return bret;
}

bool check_hub_event_is_finsih(void)
{
  bool bret = false;
  unsigned char flag = 0;
  getSinkApiParam(SINKAPIEvtA0AckFlag, (unsigned char *)&flag);
  if(!flag){
      bret = true;
  }
  return bret;
}

bool check_local_event_is_finsih(void)
{
  bool bret = false;
  unsigned char flag = 0;
  getSinkApiParam(SINKAPIEvtA0AckFlag, (unsigned char *)&flag);
  if(!flag){
      bret = true;
  }
  return bret;
}

static MultiTimer wait_ack_Timer = {.Handle = 0x01};    // Tx
static MultiTimer usart_idle_Timer = {.Handle = 0x02};  // Rx

static void wait_ack_TimerCallback(MultiTimer *timer, void *userData)
{
    (void)timer;
    bool timeout = true;
    if (userData)
    {
        memcpy(userData, &timeout, 1);
    }
}

void start_wait_ack_Timer(int time, void *userData)
{
    bool timeout = false;
    if (userData)
    {
        memcpy(userData, &timeout, 1);
    }
    softwareMultiTimerStart(EN_RTC_TIMER, &wait_ack_Timer, time, wait_ack_TimerCallback, userData, 0);
}

void stop_wait_ack_Timer(void *userData)
{
    bool timeout = false;
    if (userData)
    {
        memcpy(userData, &timeout, 1);
    }
    softwareMultiTimerStop(EN_RTC_TIMER, &wait_ack_Timer, 0);
}

static void usart_idle_TimerCallback(MultiTimer *timer, void *userData)
{
    (void)timer;
    (void)userData;
    extern void XW_hub_api_usart_msg_parse_init(void);
    XW_hub_api_usart_msg_parse_init();
}

void start_usart_idle_Timer(int time, void *userData)
{
    (void)userData;
    softwareMultiTimerStart(EN_RTC_TIMER, &usart_idle_Timer, time, usart_idle_TimerCallback, userData, 0);
}

void stop_usart_idle_Timer(void *userData)
{
    (void)userData;
    softwareMultiTimerStop(EN_RTC_TIMER, &usart_idle_Timer, 0);
}

void restart_usart_idle_Timer(int time, void *userData)
{
    if (softwareMultiTimerFind(EN_RTC_TIMER, &usart_idle_Timer) >= 0)
    {
        stop_usart_idle_Timer(userData);
    }
    start_usart_idle_Timer(time, userData);
}

