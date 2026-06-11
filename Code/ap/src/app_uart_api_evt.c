#include "app_uart_api_evt.h"
#include "ht32l6/inc/hal_uart.h"
#include "device.h"
#include "app_main.h"
#include "app_state.h"
#define HUB_API_WEAK __attribute__((weak))
#define USART_IDLE_TIMEOUT_MS (SWTMR_MS_TO_CNT(50)) // 50MS

static PARSE_STEP_E g_parseStep = STEP_STX;

static unsigned char packet_add_seq_byte(void);
static int pack_passive_usart_packet(HUB_API_USART_MSG_FCODE_E fcode, unsigned char error_code,
                                     unsigned char *msg, int msg_len,
                                     hub_api_usart_msg_t *pkt);
// static unsigned char ack_msg[HUB_API_USART_MSG_DATA_MAX_LEN] = {0};
// static int ack_msg_len = 0;

unsigned char usart_tx_buffer[UART_BUF_SIZE] = {0};
static hub_api_usart_msg_t pop_pkt = {0};
volatile uint16_t tx_write = 0, tx_read = 0;

// static int usart_tx_len = 0;

// static long pkt_rx_count = 0;
// static long pkt_tx_count = 0;

#if XDEBUG_RF
int rx_cmd_parse(uint8_t *data, hub_api_usart_msg_t *ptr_ack_pkt);
#endif

HUB_API_WEAK int rx_tx_event(uint8_t *data,
                                hub_api_usart_msg_t *ptr_ack_pkt)
{
    unsigned char tns = 0;
    unsigned char flag = 0;

    setSinkApiParam(SINKAPIEvtA0AckFlag, (unsigned char *)&flag);
    setSinkApiParam(SINKAPIEvtA0TNS, (unsigned char *)&tns);
    return 0;
}

HUB_API_WEAK int rx_event_hub_event(uint8_t *data,
                                    hub_api_usart_msg_t *ptr_ack_pkt)
{
    unsigned char err_code = 0;
    // TODO 获取相较于首地址的偏移，决定如何获取数据
    uint8_t u8data = data[STEP_DATA];
    switch (u8data)
    {
    case SLAVE_TO_MASTER_LOCAL_EVENT_CONFIG_REQ:
        APP_StatePostGetInitConfig();
        break;
    case SLAVE_TO_MASTER_LOCAL_EVENT_PAIR_SUCC:
        if (APP_GetStateInfoRam()->localOption.isPairing == 1)
        {
            APP_StateSetSelfCheckSL(eSystemOptionPairSuccess);
            APP_GetStateInfoRam()->localOption.isPairing = 0;
        }
        break;
    case SLAVE_TO_MASTER_LOCAL_EVENT_PAIR_FAIL:
        if (APP_GetStateInfoRam()->localOption.isPairing == 1)
        {
            APP_StateSetSelfCheckSL(eSystemOptionPairFail);
            APP_GetStateInfoRam()->localOption.isPairing = 0;
        }
        break;
    case SLAVE_TO_MASTER_LOCAL_EVENT_FINDME:

        APP_StateSetSelfCheckSL(eSystemOptionFindMe);
        break;
    case SLAVE_TO_MASTER_LOCAL_EVENT_SELF_TEST:
        StateInfo_SetField(APP_GetStateInfoRam(), FIELD_BUTTON_EVT, BUTTON_SELF_CHECK);
        break;
    case SLAVE_TO_MASTER_LOCAL_EVENT_GET_STATE:
        APP_StatePostGetState();
        break;
    case SLAVE_TO_MASTER_LOCAL_EVENT_IAP_START:
        AP_Reset(1); // 调用复位函数 不用返回ack 进入boot后发送的消息视为ack
        break;
    default:
        break;
    }
    err_code = 0;
    goto ack;
ack:
    pack_passive_usart_packet((HUB_API_USART_MSG_FCODE_E)data[STEP_FCODE], err_code, (unsigned char *)&err_code, 1, ptr_ack_pkt);
    XW_hub_api_creat_passive_usart_ack(ptr_ack_pkt);
    return 0;
}

static const hub_api_usart_rx_entry_t ds_hub_api_rx_fcode_table[] = {
    {FROM_M_FCODE_ALARM_STATE_REPORT, rx_tx_event},
    {FROM_M_FCODE_ABNORMAL_STATE_REPORT, rx_tx_event},
    {FROM_M_FCODE_SYS_STATE_REPORT, rx_tx_event},
    {FROM_M_FCODE_SYS_LOCAL_OPERATE, rx_tx_event},
    {TO_M_EVENT_LOCAL_EVENT, rx_event_hub_event},
#if XDEBUG_RF
    {TO_M_FCODE_FROM_M_FCODE_DBG_MSG, rx_cmd_parse},
#endif
};

#define usart_rx_cmd_num (sizeof(ds_hub_api_rx_fcode_table) / sizeof(ds_hub_api_rx_fcode_table[0]))

static int pack_passive_usart_packet(HUB_API_USART_MSG_FCODE_E fcode, unsigned char error_code,
                                     unsigned char *msg, int msg_len,
                                     hub_api_usart_msg_t *pkt)
{
    static uint16_t cnt = 0;
    cnt++;
    unsigned short crc_value;
    memset((char *)pkt, 0, sizeof(hub_api_usart_msg_t));
    pkt->stx = 0x02;
    pkt->seq = packet_add_seq_byte();
    // pkt->len = msg_len;
    // pkt->statu = passive_packet_add_statu_byte(error_code);
    pkt->fcode = fcode;
    memcpy(&pkt->data, msg, msg_len);
    pkt->len = HUB_API_USART_MSG_LEN_MIN_VALUE + msg_len;
    crc_value = B_L_SWAP_16(get_msg_crc16(&pkt->len, pkt->len + 1 - 2));
    pkt->crc[0] = (crc_value >> 8) & 0xff;
    pkt->crc[1] = (crc_value) & 0xff;
    pkt->etx = 0x03;

    // printf_pkt(pkt);

    return 0;
}

static int pack_err_packrt_usart_packet(HUB_API_USART_MSG_FCODE_E fcode, unsigned char error_code,
                                        hub_api_usart_msg_t *pkt)
{
    unsigned short crc_value;
    memset((char *)pkt, 0, sizeof(hub_api_usart_msg_t));
    pkt->stx = 0x02;

    pkt->len = 0;
    pkt->seq = packet_add_seq_byte();
    // pkt->statu = passive_packet_add_statu_byte(error_code);
    pkt->fcode = fcode;
    pkt->data[0] = error_code;
    pkt->len = 5;
    crc_value = B_L_SWAP_16(get_msg_crc16(&pkt->len, pkt->len + 1 - 2));
    pkt->crc[0] = (crc_value >> 8) & 0xff;
    pkt->crc[1] = (crc_value) & 0xff;
    pkt->etx = 0x03;

    //    printf_pkt(pkt);

    return 0;
}

#if XDEBUG_RF
int rx_cmd_parse(uint8_t *data, hub_api_usart_msg_t *ptr_ack_pkt)
{
    int err_code = 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t cmd_len = (data[STEP_LEN] > HUB_API_USART_EXPECT_MSG_LEN) ? (data[STEP_LEN] - HUB_API_USART_EXPECT_MSG_LEN) : 0; // 实际cmd长度
    for (int i = STEP_DATA; i < STEP_DATA + cmd_len; i++)
    {
        cmd_parser_task(data[i]);
    }
    cmd_table_end();

    err_code = 0;
    goto ack;
ack:
    pack_passive_usart_packet((HUB_API_USART_MSG_FCODE_E)data[STEP_FCODE], err_code, (unsigned char *)&err_code, 1, ptr_ack_pkt);
    XW_hub_api_creat_passive_usart_ack(ptr_ack_pkt);
    __set_PRIMASK(primask);
    return err_code;
}
#endif


static hub_api_usart_msg_t usart_ack_pkt = {0};
// 轮询函数
void XW_hub_api_usart_listen(void)
{
    static uint8_t s_len;
    static uint8_t start_cmd = 0;
    static uint32_t last_rx_time  = 0;
    switch (g_parseStep)
    {
    case STEP_STX:
    {
        uint8_t data_stx = 0;
        if (hal_uart_receive(UART_DEV, &data_stx, 1))
        {
            rcv_cnt_inc();
#if !XDEBUG_RF
            last_rx_time = hal_rtc_get_cnt();
            if (data_stx == 0x02 && !cmd_isInTransMode())
            {
                restart_usart_idle_Timer(USART_IDLE_TIMEOUT_MS, NULL);
                g_parseStep = STEP_LEN;
                //            APP_PRINTF("rxdmaout:%d\r\n", hal_uart_get_buf_out_prt(UART_DEV, UXART_FN_RX));
            }
#ifdef SUPPORT_CMD_LINE_FUNCTION
            else
            {
               if(start_cmd)
               {
                    cmd_parser_task(data_stx);
               }
               else
               {
                   if (data_stx != 0x00)
                   {
                       start_cmd = 1;         // 开始正式接收命令
                       cmd_parser_task(data_stx);
                   }
               }
            }
#endif
#else
            if (data_stx == 0x02)
            {
                restart_usart_idle_Timer(USART_IDLE_TIMEOUT_MS, NULL);
                g_parseStep = STEP_LEN;
            }
#endif
        }
#if !XDEBUG_RF
        else
        {
#ifdef SUPPORT_CMD_LINE_FUNCTION    // 每次进来 做nMs时长判断 然后发送
            uint32_t elapsed = (hal_rtc_get_cnt() - last_rx_time) & 0xFFFFFFu;
            if (elapsed > USART_IDLE_TIMEOUT_MS)
            {
                // 真正超时后再 flush 并清 start_cmd
                cmd_parser_flush();
                start_cmd = 0;
            }
#endif
        }
#endif
        break;
    }
    case STEP_LEN:
    {
        uint8_t data_len[2] = {0};
        if (hal_uart_peekPacket(UART_DEV, UXART_FN_RX, data_len, 0, 2))
        {
            if (data_len[1] >= HUB_API_USART_MSG_LEN_MIN_VALUE && data_len[1] <= HUB_API_USART_MSG_LEN_MAX_VALUE)
            {
                g_parseStep = STEP_DATA;
                s_len = data_len[1] + 2; // 预期的长度，除ETX外，包括SEQ LEN
            }
            else
            {
                g_parseStep = STEP_STX;
                APP_PRINTF("lE len%d\r\n", data_len[1]);
                break;
            }
        }
        break;
    }
    case STEP_DATA:
    {
        uint8_t data_etx = 0;
        uint8_t data[64] = {0};
        if (hal_uart_peek(UART_DEV, UXART_FN_RX, &data_etx, s_len)) // 偏移s_len为etx位置
        {
            stop_usart_idle_Timer(NULL);
            if (data_etx == 0x03)
            {
                data[STEP_STX] = 0x02;
                hal_uart_peekPacket(UART_DEV, UXART_FN_RX, &data[STEP_SEQ], 0, s_len + 1); // 跳过stx 加上etx

                if ((data[s_len - 1] << 8 | data[s_len]) == B_L_SWAP_16(get_msg_crc16(&data[STEP_LEN], s_len + 1 - 2 - 2))) // crc len1+data-crc长度2 crc校验通过 s_len+1为总长度-2crc16 -2seq+etx
                {
                    g_parseStep = STEP_STX;
                    XW_hub_api_usart_rx_event_mediator(data, &usart_ack_pkt);
                }
                else
                {
                    g_parseStep = STEP_STX;
                    APP_PRINTF("lE CRC\r\n");
                }
            }
            else
            {
                g_parseStep = STEP_STX;
                APP_PRINTF("lErr etx\n");
            }
        }
        break;
    }
    default:
    {
        g_parseStep = STEP_STX;
        break;
    }
    }
}

uint8_t XW_hub_rx_IsIdle(void)
{
//    APP_PRINTF("Stepidle:%d\r\n", (g_parseStep == STEP_STX));
    return (g_parseStep == STEP_STX);
}

// static void XW_hub_api_pkt_clean(hub_api_usart_msg_t *ptr_pkt)
//{
//     memset(ptr_pkt, 0, sizeof(hub_api_usart_msg_t));
// }

void XW_hub_api_usart_msg_parse_init(void)
{
    // APP_PRINTF("l tm out\r\n");
    g_parseStep = STEP_STX;
}

int XW_hub_api_usart_rx_event_mediator(uint8_t *data,
                                       hub_api_usart_msg_t *ptr_ack_pkt)
{
    if (!data)
    {
        return 1;
    }
    unsigned int i = 0;
    for (i = 0; i < usart_rx_cmd_num; i++)
    {
        if (ds_hub_api_rx_fcode_table[i].fcode == data[STEP_FCODE])
        {
            break;
        }
    }

    if (i == usart_rx_cmd_num)
    {
        hal_uart_update_outptr(UART_DEV, UXART_FN_RX, data[STEP_LEN] + HUB_API_USART_EXPECT_MSG_LEN - 1); // stx seq len etx
        return 2;
    }
    hal_uart_update_outptr(UART_DEV, UXART_FN_RX, data[STEP_LEN] + HUB_API_USART_EXPECT_MSG_LEN - 1);
    
//    APP_PRINTF("success!\r\n");
    ds_hub_api_rx_fcode_table[i].fun(data, ptr_ack_pkt);

    return 0;
}

void XW_hub_api_creat_msg(HUB_API_USART_MSG_FCODE_E fcode, unsigned char *data, int len, bool is_NeedAck)
{
    hub_api_usart_msg_t pkt = {0};
    if (len > (HUB_API_USART_MSG_DATA_MAX_LEN - 1))
    {
        return;
    }
    if (len)
    {
        if (!data)
        {
            return;
        }
    }
    else
    {
        // 没有参数，data可以为NULL
    }
    if (is_NeedAck)
    {
        uint8_t msg[HUB_API_USART_MSG_LEN_MAX_VALUE];
        memset(msg, 0, HUB_API_USART_MSG_LEN_MAX_VALUE);
        int pos = 0;
        msg[pos++] = len + HUB_API_USART_MSG_LEN_MIN_VALUE; // len
        msg[pos++] = 0x00;  // status
        msg[pos++] = fcode;
        memcpy(&msg[pos], data, len);
        pos += len;
        pos += 2;   // crc
        XW_hub_api_push_usart_tx_event((unsigned char *)msg, pos);  // 需要应答 放入缓冲队列
    }
    else
    {
        hub_api_usart_msg_t *ptr_pkt = &pkt;
        ptr_pkt->stx = 0x02;
        ptr_pkt->seq = packet_add_seq_byte();
        ptr_pkt->statu = 0;
        ptr_pkt->fcode = fcode;
        ptr_pkt->len = len + HUB_API_USART_MSG_LEN_MIN_VALUE;
        memcpy(ptr_pkt->data, data, len);
        uint16_t crc_value = B_L_SWAP_16(get_msg_crc16(&ptr_pkt->len, ptr_pkt->len + 1 - 2));
        ptr_pkt->data[ptr_pkt->len -HUB_API_USART_MSG_LEN_MIN_VALUE] = (crc_value >> 8) & 0xff;
        ptr_pkt->data[ptr_pkt->len -HUB_API_USART_MSG_LEN_MIN_VALUE+1] = (crc_value) & 0xff;
        ptr_pkt->data[ptr_pkt->len -HUB_API_USART_MSG_LEN_MIN_VALUE+2] = 0x03;
        usart_tx_msg((unsigned char *)ptr_pkt, ptr_pkt->len + HUB_API_USART_EXPECT_MSG_LEN);    // 不需要应答 直接放入底层串口发送区
        // while (!hal_uart_is_tx_complete(UART_SUBG_BOOTLOADER));
    }
}

int XW_hub_api_push_usart_tx_event(uint8_t *data, uint16_t len)
{
    if (!data || len == 0 || len > HUB_API_USART_MSG_LEN_MAX_VALUE)
        return -1;
    
    uint16_t used = (tx_write - tx_read) & (UART_BUF_SIZE - 1);
    uint16_t free_space = UART_BUF_SIZE - used - 1;
    if (free_space < (UART_BUF_SIZE >> 3)) // 1/8空间
    {
        unsigned char flag = 0;
        setSinkApiParam(SINKAPIEnableRetrans, (unsigned char *)&flag); // 失能重传
    }
    if (free_space < len)
        return -2;
    
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    uint16_t first_part = UART_BUF_SIZE - tx_write;
    if (first_part > len)
        first_part = len;

    // 分段cpy
    memcpy(&usart_tx_buffer[tx_write], data, first_part);
    // printbuff(&usart_tx_buffer[tx_write], first_part, "%02x ", "msg:");
    // while(!hal_uart_is_tx_complete(UART_SUBG_BOOTLOADER));

    if (len > first_part)
    {
        memcpy(usart_tx_buffer, data + first_part, len - first_part);
        // printbuff(usart_tx_buffer, len - first_part, "%x ", "\r\n:");
        // while(!hal_uart_is_tx_complete(UART_SUBG_BOOTLOADER));
    }

    tx_write = (tx_write + len) & (UART_BUF_SIZE - 1); // 更新写指针
    __set_PRIMASK(primask);
    // APP_PRINTF("tx w:%d\r\n", tx_write);
    return 0;
}

void XW_hub_api_creat_passive_usart_ack(const hub_api_usart_msg_t *ptr_pkt)
{
    if (ptr_pkt->len < HUB_API_USART_MSG_LEN_MIN_VALUE || ptr_pkt->len + HUB_API_USART_EXPECT_MSG_LEN > HUB_API_USART_MSG_MAX_LEN) // 长度超限
    {
        return;
    }

    unsigned char msg[HUB_API_USART_ACK_MSG_MAX_LEN] = {0};
    int len = 0;
    int i = 0;

    msg[len++] = ptr_pkt->stx;
    msg[len++] = ptr_pkt->seq;
    msg[len++] = ptr_pkt->len;
    msg[len++] = ptr_pkt->statu;
    msg[len++] = ptr_pkt->fcode;
    for (i = 0; i < (ptr_pkt->len - HUB_API_USART_MSG_LEN_MIN_VALUE); i++)
    {
        msg[len++] = ptr_pkt->data[i];
    }
    msg[len++] = ptr_pkt->crc[0];
    msg[len++] = ptr_pkt->crc[1];
    msg[len++] = ptr_pkt->etx;

    hal_uart_write(UART_DEV, msg, len);
}

int XW_hub_api_is_usart_tx_buf_notempty(void)
{
    return !(tx_write == tx_read);
}

int XW_hub_api_check_usart_tx_event(void)
{
    uint16_t available = (tx_write - tx_read) & (UART_BUF_SIZE - 1); // 可用数据
    if (available < (HUB_API_USART_MSG_LEN_MIN_VALUE))               // 过短
        return -1;
    // APP_PRINTF("tx r:%d\r\n", tx_read);
    uint8_t pkt_len = usart_tx_buffer[(tx_read) & (UART_BUF_SIZE - 1)]; // 读取len字段  压入队列时先写len 再写data；

    if (pkt_len > HUB_API_USART_MSG_LEN_MAX_VALUE) // 超范围
        return -2;
    // if (available < pkt_len + HUB_API_USART_EXPECT_MSG_LEN) // 过短 异常长度
    //     return -3;
    // APP_PRINTF("pkt_len:%d\r\n", pkt_len);
    return 0;
}

uint8_t *XW_hub_api_pop_usart_buf_adr(uint16_t offset)
{
    return &usart_tx_buffer[(tx_read + offset) & (UART_BUF_SIZE - 1)];
}

int XW_hub_api_pop_usart_tx_event(uint8_t *buf, uint16_t *len)
{
    if (!buf || !len)
        return -1;
    uint8_t pkt_len = usart_tx_buffer[(tx_read) & (UART_BUF_SIZE - 1)]; // 读取len字段

    uint16_t start = tx_read;
    uint16_t first_part = UART_BUF_SIZE - start;
    *len = pkt_len;
    // APP_PRINTF("len:%d\r\n", *len);
    // while(!hal_uart_is_tx_complete(UART_SUBG_BOOTLOADER));
    if (*len == 0) return -2;
    memcpy(buf, &usart_tx_buffer[start], ((first_part < pkt_len) ? first_part : pkt_len)); // memcpy全长/末尾第一段
    if (pkt_len > first_part)
    {
        memcpy((uint8_t *)buf + first_part, usart_tx_buffer, pkt_len - first_part); // 剩余部分
    }
    // printbuff(buf, pkt_len + 1, "%02x ", "msgpop:");
    // while(!hal_uart_is_tx_complete(UART_SUBG_BOOTLOADER));
    // tx_read = (tx_read + pkt_len) & (UART_BUF_SIZE - 1);

    return 0;
}

int XW_hub_api_get_usart_tx_event(void)
{
    if (XW_hub_api_check_usart_tx_event() != 0)
        return -1;
    return 0;
}

int XW_hub_api_usart_tx_event_mediator(void)
{
    // unsigned char data[HUB_API_USART_MSG_MAX_LEN] = {0};
    hub_api_usart_msg_t *ptr_pkt = &pop_pkt;
    if (XW_hub_api_pop_usart_tx_event(&ptr_pkt->len, (uint16_t *)&ptr_pkt->len) != 0)
    {
        return 1;
    }

    unsigned char flag = 1;
    ptr_pkt->stx = 0x02;
    ptr_pkt->seq = packet_add_seq_byte();
    ptr_pkt->statu = 0;
    uint16_t crc_value = B_L_SWAP_16(get_msg_crc16(&ptr_pkt->len, ptr_pkt->len + 1 - 2));

    ptr_pkt->data[ptr_pkt->len -HUB_API_USART_MSG_LEN_MIN_VALUE] = (crc_value >> 8) & 0xff;
    ptr_pkt->data[ptr_pkt->len -HUB_API_USART_MSG_LEN_MIN_VALUE+1] = (crc_value) & 0xff;
    ptr_pkt->data[ptr_pkt->len -HUB_API_USART_MSG_LEN_MIN_VALUE+2] = 0x03;
    usart_tx_msg((uint8_t *)ptr_pkt, ptr_pkt->len + HUB_API_USART_EXPECT_MSG_LEN);
    setSinkApiParam(SINKAPIEvtA0AckFlag, (unsigned char *)&flag);
    return 0;
}

uint16_t usart_tx_msg(uint8_t *data, uint16_t data_len)
{
    if (!data || data_len > HUB_API_USART_MSG_MAX_LEN)
        return 0;
    if (hal_uart_write(UART_DEV, data, data_len))
    {
        return data_len;
    }
    else
    {
        return 0;
    }
}

uint8_t XW_hub_api_update_read_ptr(void)
{
    // APP_PRINTF("%s\r\n", __func__);
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (((tx_read - tx_write) & (UART_BUF_SIZE - 1)) > (UART_BUF_SIZE >> 1))
    {
        unsigned char flag = 1;
        setSinkApiParam(SINKAPIEnableRetrans, (unsigned char *)&flag); // 使能重传
    }
    uint8_t len = usart_tx_buffer[(tx_read) & (UART_BUF_SIZE - 1)]; // 读取len字段
    tx_read = (tx_read + len + 1) & (UART_BUF_SIZE - 1);
    __set_PRIMASK(primask);
    return 0;
}

bool check_usart_active_evt_is_finish(HUB_API_USART_MSG_FCODE_E fcode)
{
    bool bret = true;
    switch (fcode)
    {
    case FROM_M_FCODE_ALARM_STATE_REPORT:
    case FROM_M_FCODE_ABNORMAL_STATE_REPORT:
    case FROM_M_FCODE_SYS_STATE_REPORT:
    case FROM_M_FCODE_SYS_LOCAL_OPERATE:
    case FROM_M_FCODE_INIT_CONIFG:
        // bret = check_hub_event_is_finsih();
        // break;
    case TO_M_EVENT_LOCAL_EVENT:
        bret = check_local_event_is_finsih();
        break;
    default:
        break;
    }

    return bret;
}

static uint8_t fcode_cnt[5] = {0};
bool check_usart_evt_retrans_end(HUB_API_USART_MSG_FCODE_E fcode)
{
    bool bret = false;
//    uint8_t cnt = 0;
    switch (fcode)
    {
    case FROM_M_FCODE_ALARM_STATE_REPORT:
        fcode_cnt[0]++;
        bret = (fcode_cnt[0] >= 3) ? true : false;
        break;
    case FROM_M_FCODE_ABNORMAL_STATE_REPORT:
        fcode_cnt[1]++;
        bret = (fcode_cnt[1] >= 3) ? true : false;
        break;
    case FROM_M_FCODE_SYS_STATE_REPORT:
        fcode_cnt[2]++;
        bret = (fcode_cnt[2] >= 3) ? true : false;
        break;
    case FROM_M_FCODE_SYS_LOCAL_OPERATE:
        fcode_cnt[3]++;
        bret = (fcode_cnt[3] >= 3) ? true : false;
        // getSinkApiParam(SINKAPIEvtA1TNS, &cnt);
        // bret = (cnt >= 3) ? true : false;
        // cnt++;
        // setSinkApiParam(SINKAPIEvtA1TNS, &cnt);
        break;
    case FROM_M_FCODE_INIT_CONIFG:
        fcode_cnt[4]++;
        bret = (fcode_cnt[4] >= 3) ? true : false;
        // getSinkApiParam(SINKAPIEvtA0TNS, &cnt);
        // bret = (cnt >= 3) ? true : false;
        // cnt++;
        // setSinkApiParam(SINKAPIEvtA0TNS, &cnt);
        break;
    default:
        break;
    }
    return bret;
}

static unsigned char packet_add_seq_byte(void)
{
    uint8_t ret_seq = 0;
    uint8_t hub_seq = 0;
    getSinkApiParam(SINKAPIHubUsartSeq, &hub_seq);
    ret_seq = hub_seq;
    hub_seq++;
    hub_seq %= 0xff;
    if (hub_seq == 0)
    {
        hub_seq = 1;
    }
    setSinkApiParam(SINKAPIHubUsartSeq, &hub_seq);
    return ret_seq;
}

void clean_retrans_ack_flag(HUB_API_USART_MSG_FCODE_E fcode)
{
    switch (fcode)
    {
    case FROM_M_FCODE_ALARM_STATE_REPORT:
        fcode_cnt[0] = 0;
        break;
    case FROM_M_FCODE_ABNORMAL_STATE_REPORT:
        fcode_cnt[1] = 0;
        break;
    case FROM_M_FCODE_SYS_STATE_REPORT:
        fcode_cnt[2] = 0;
        break;
    case FROM_M_FCODE_SYS_LOCAL_OPERATE:
        fcode_cnt[3] = 0;
        break;
    case FROM_M_FCODE_INIT_CONIFG:
        fcode_cnt[4] = 0;
        break;
    default:
        break;
    }
}
