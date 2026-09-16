/**
  ******************************************************************************
  * @file    ap_uart_protocol.c
  * @brief   树莓派配置协议：流式收包、会话、去重、EEPROM配置和DMA应答
  *
  *          本模块运行在裸机主循环中。ISR只维护USART DMA，本文件不在中断
  *          内解析协议或写EEPROM。每次RX任务限制读取32字节且最多处理一帧，
  *          避免连续串口数据长期占用主循环、影响100Hz红外采样任务。
  ******************************************************************************
  */

#include "ap_uart_protocol.h"

#include "ap_eeprom.h"
#include "ap_ir.h"
#include "ap_uv.h"
#include "crc16.h"
#include "hal_tim.h"
#include "hal_uart.h"
#include "main.h"

#include <string.h>

#define AP_UART_RESPONSE_FRAME_COUNT_MAX  (2U)
#define AP_UART_F0_DATA_LEN                (6U)
#define AP_UART_CONFIG_SNAPSHOT_LEN        (84U)
#define AP_UART_CONFIG_RESPONSE_LEN        (AP_UART_CONFIG_SNAPSHOT_LEN)

/*
 * 128字节是包含帧头、长度、CRC在内的单帧线长上限；固定开销9字节后，
 * 业务Data最多119字节。编译期锁定关系，防止后续修改常量造成收发越界。
 */
typedef char AP_UART_FrameLimitMustRemain128Bytes[
    (AP_UART_FRAME_MAX == 128U) ? 1 : -1];
typedef char AP_UART_DataLimitMustRemain119Bytes[
    (AP_UART_DATA_MAX == 119U) ? 1 : -1];
typedef char AP_UART_FrameLengthRelationMustMatch[
    ((6U + AP_UART_PAYLOAD_LEN_MAX) == AP_UART_FRAME_MAX) ? 1 : -1];
typedef char AP_UART_ConfigLayoutMustRemain84Bytes[
    ((4U + AP_UART_CONFIG_FIELD_COUNT * 4U) == AP_UART_CONFIG_SNAPSHOT_LEN) ? 1 : -1];
typedef char AP_UART_ConfigMustFitData[
    (AP_UART_CONFIG_RESPONSE_LEN <= AP_UART_DATA_MAX) ? 1 : -1];

typedef enum {
    RX_PARSE_SOF0 = 0,
    RX_PARSE_SOF1,
    RX_PARSE_FRAME,
} RxParseState_t;

/** @brief 最多两帧的响应事务；GET_CONFIG使用配置帧加F0，其余请求只使用F0。 */
typedef struct {
    uint8_t  active;
    uint8_t  frame_count;
    uint8_t  frame_index;
    uint16_t offset;
    uint32_t last_progress_ms;
    uint16_t length[AP_UART_RESPONSE_FRAME_COUNT_MAX];
    uint8_t  frame[AP_UART_RESPONSE_FRAME_COUNT_MAX][AP_UART_FRAME_MAX];
} TxBundle_t;

/**
 * @brief 最近一次已执行请求及其完整响应缓存。
 * @note  树莓派严格停等，因此缓存一条即可覆盖应答丢失后的同帧重发。
 *        GET_CONFIG包含0x81配置数据帧+F0两帧，必须整体缓存和整体重放。
 */
typedef struct {
    uint8_t  valid;
    uint16_t request_sequence;
    uint8_t  request_message_id;
    uint8_t  request_data_len;
    uint8_t  request_data[AP_UART_DATA_MAX];
    uint8_t  response_count;
    uint16_t response_length[AP_UART_RESPONSE_FRAME_COUNT_MAX];
    uint8_t  response_frame[AP_UART_RESPONSE_FRAME_COUNT_MAX][AP_UART_FRAME_MAX];
} RequestCache_t;

/** @brief 当前配置会话及请求序号基准；新握手或链路异常会整体清零。 */
typedef struct {
    uint8_t  active;
    uint32_t nonce;
    uint32_t last_rx_ms;
    uint8_t  latest_sequence_valid;
    uint16_t latest_sequence;
} SessionState_t;

static RxParseState_t s_rx_state;
static uint8_t  s_rx_frame[AP_UART_FRAME_MAX];
static uint16_t s_rx_pos;
static uint16_t s_rx_expected;
static uint32_t s_rx_start_ms;

static TxBundle_t    s_tx;
static RequestCache_t s_cache;
static SessionState_t s_session;
static uint16_t       s_tx_sequence;
static uint8_t        s_diagnostic_mode;

static void put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
}

static void put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
}

static uint16_t get_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static uint32_t get_u32_le(const uint8_t *src)
{
    return (uint32_t)src[0]
         | ((uint32_t)src[1] << 8)
         | ((uint32_t)src[2] << 16)
         | ((uint32_t)src[3] << 24);
}

static uint16_t next_tx_sequence(void)
{
    s_tx_sequence++;
    if (s_tx_sequence == 0U) {
        s_tx_sequence = 1U;
    }
    return s_tx_sequence;
}

/**
 * @brief  将一条逻辑消息编码为线帧，并分配非零发送序号。
 * @return 完整帧长度；参数或Data长度非法时返回0。
 */
static uint16_t build_frame(uint8_t message_id, const uint8_t *data,
                            uint16_t data_len, uint8_t *frame)
{
    uint16_t payload_len;
    uint16_t crc_offset;
    uint16_t crc;

    if (frame == NULL || data_len > AP_UART_DATA_MAX ||
        (data_len > 0U && data == NULL)) {
        return 0U;
    }

    payload_len = AP_UART_BODY_FIXED_LEN + data_len;
    frame[0] = AP_UART_SOF_0;
    frame[1] = AP_UART_SOF_1;
    frame[2] = AP_UART_PROTOCOL_VERSION;
    frame[3] = (uint8_t)payload_len;
    put_u16_le(&frame[4], next_tx_sequence());
    frame[6] = message_id;
    if (data_len > 0U) {
        memcpy(&frame[7], data, data_len);
    }

    crc_offset = 4U + payload_len;
    crc = CRC16_CCITT(&frame[4], payload_len);
    put_u16_le(&frame[crc_offset], crc);
    return (uint16_t)(crc_offset + 2U);
}

/** @brief 开始新的响应事务，同时启动500ms发送进展计时。 */
static void tx_bundle_begin(void)
{
    memset(&s_tx, 0, sizeof(s_tx));
    s_tx.active = 1U;
    s_tx.last_progress_ms = HAL_GetTick();
}

/** @brief 按顺序向当前响应事务追加一帧，容量或组帧失败时返回0。 */
static uint8_t tx_bundle_add(uint8_t message_id, const uint8_t *data,
                             uint16_t data_len)
{
    uint8_t index = s_tx.frame_count;
    uint16_t length;

    if (index >= AP_UART_RESPONSE_FRAME_COUNT_MAX) {
        return 0U;
    }

    length = build_frame(message_id, data, data_len, s_tx.frame[index]);
    if (length == 0U) {
        return 0U;
    }
    s_tx.length[index] = length;
    s_tx.frame_count++;
    return 1U;
}

/** @brief 生成固定6字节Data的F0结果帧，并在USART1记录业务结果。 */
static void append_f0(uint16_t request_sequence, uint8_t request_message_id,
                      AP_UART_Result_t result)
{
    uint8_t data[AP_UART_F0_DATA_LEN];

    put_u16_le(&data[0], request_sequence);
    data[2] = request_message_id;
    data[3] = (uint8_t)result;
    data[4] = 0U; /* error_code按简化协议保留为0。 */
    data[5] = 0U;
    (void)tx_bundle_add(AP_MSG_COMMAND_RESPONSE, data, sizeof(data));

    /*
     * 配置请求频率很低，允许在主循环通过USART1输出最终处理结果。
     * 该位置位于EEPROM写入、读回及运行参数更新之后，SUCCESS表示配置已真正生效；
     * 日志不得走USART2，避免文本字节破坏树莓派二进制协议流。
     */
    BSP_UART_Printf("[PROTO] RESP REQ_SEQ=%u ID=0x%02X RESULT=%s\r\n",
                    (unsigned int)request_sequence,
                    (unsigned int)request_message_id,
                    (result == AP_UART_RESULT_SUCCESS) ? "SUCCESS" : "FAILED");
}

/** @brief 缓存请求及全部响应，保证重复SET不再擦写EEPROM。 */
static void cache_current_response(uint16_t sequence, uint8_t message_id,
                                   const uint8_t *data, uint8_t data_len)
{
    memset(&s_cache, 0, sizeof(s_cache));
    s_cache.valid = 1U;
    s_cache.request_sequence = sequence;
    s_cache.request_message_id = message_id;
    s_cache.request_data_len = data_len;
    if (data_len > 0U) {
        memcpy(s_cache.request_data, data, data_len);
    }
    s_cache.response_count = s_tx.frame_count;
    for (uint8_t i = 0U; i < s_tx.frame_count; i++) {
        s_cache.response_length[i] = s_tx.length[i];
        memcpy(s_cache.response_frame[i], s_tx.frame[i], s_tx.length[i]);
    }
}

/** @brief 比较请求序号、ID、长度和Data，判断是否为最近请求的原样重发。 */
static uint8_t cached_request_matches(uint16_t sequence, uint8_t message_id,
                                      const uint8_t *data, uint8_t data_len)
{
    if (s_cache.valid == 0U || s_cache.request_sequence != sequence ||
        s_cache.request_message_id != message_id ||
        s_cache.request_data_len != data_len) {
        return 0U;
    }
    if (data_len == 0U) {
        return 1U;
    }
    return (memcmp(s_cache.request_data, data, data_len) == 0) ? 1U : 0U;
}

/** @brief 原样重放缓存响应；保留旧响应帧序号，不重新执行请求。 */
static void replay_cached_response(void)
{
    memset(&s_tx, 0, sizeof(s_tx));
    s_tx.active = 1U;
    /* 重放也属于一次新发送，不能沿用清零后的时间戳触发500ms立即超时。 */
    s_tx.last_progress_ms = HAL_GetTick();
    s_tx.frame_count = s_cache.response_count;
    for (uint8_t i = 0U; i < s_cache.response_count; i++) {
        s_tx.length[i] = s_cache.response_length[i];
        memcpy(s_tx.frame[i], s_cache.response_frame[i], s_cache.response_length[i]);
    }
}

/** @brief 丢弃当前候选帧并回到SOF搜索状态。 */
static void parser_reset(void)
{
    s_rx_state = RX_PARSE_SOF0;
    s_rx_pos = 0U;
    s_rx_expected = 0U;
    s_rx_start_ms = 0U;
}

/** @brief 结束配置会话，同时清除序号基准和最近请求缓存。 */
static void session_reset(void)
{
    memset(&s_session, 0, sizeof(s_session));
    memset(&s_cache, 0, sizeof(s_cache));
}

/** @brief 16位自然回环序号比较；半区间之前的值均视为旧请求。 */
static uint8_t sequence_is_newer(uint16_t sequence, uint16_t previous)
{
    uint16_t delta = (uint16_t)(sequence - previous);
    return (delta != 0U && delta < 0x8000U) ? 1U : 0U;
}

/** @brief 将已经持久化的UV配置同步到检测器和脉宽捕获驱动。 */
static void apply_uv_config(const AP_EEPROM_UV_Param_t *p)
{
    AP_UV_SetConfig(p->thr_min, p->thr_max, p->win_min, p->win_max,
                    p->cfm_min, p->cfm_max);
    AP_UV_SetLevel((uint8_t)p->sensitivity);
    BSP_TIM_IC_SetPulseRange((uint16_t)p->pw_min_us, (uint16_t)p->pw_max_us);
}

/**
 * @brief 终止失去同步的收发事务并要求重新握手。
 * @note  UART错误、DMA覆盖或发送超时后无法确认线上已有多少字节，残帧不可复用。
 */
static void abort_transport(void)
{
    BSP_UART_ClearTxBuf(BSP_UART_COM);
    BSP_UART_ClearRxBuf(BSP_UART_COM);
    memset(&s_tx, 0, sizeof(s_tx));
    parser_reset();
    session_reset();
}

/** @brief 将已经持久化的IR配置同步到检测器当前运行参数。 */
static void apply_ir_config(const AP_EEPROM_IR_Param_t *p)
{
    AP_IR_SetConfig(p->power_min, p->power_max,
                    p->r38_threshold, p->r50_threshold,
                    p->freq_low_x10, p->freq_high_x10,
                    p->cfm_min, p->cfm_max);
    AP_IR_SetLevel((uint8_t)p->sensitivity);
}

/**
 * @brief 修改一个SYSTEM配置。
 * @note  复用现有APP接口完成“先落盘、后生效”；相同值由APP接口直接成功返回，
 *        不会产生重复EEPROM写入。协议不新增任何报警清除行为。
 */
static uint8_t set_system_value(uint8_t message_id, uint32_t value)
{
    if (message_id == AP_MSG_SET_SHOW_MODE) {
        if (value > 1U) return 0U;
        return (APP_SetShowMode((uint8_t)value) == 0) ? 1U : 0U;
    }
    if (message_id == AP_MSG_SET_IR_PROFILE_ENABLED) {
        if (value > 1U) return 0U;
        return (APP_SetIrProfileEnabled((uint8_t)value) == 0) ? 1U : 0U;
    }
    return 0U;
}

/**
 * @brief 事务式修改一个UV字段：候选组校验、落盘读回后再更新运行参数。
 * @return 1表示已生效或原值相同，0表示ID、参数或存储失败。
 */
static uint8_t set_uv_value(uint8_t message_id, uint32_t value)
{
    AP_EEPROM_UV_Param_t candidate = *AP_EEPROM_UV_Get();
    uint32_t *field = NULL;

    switch (message_id) {
        case AP_MSG_SET_UV_SENSITIVITY: field = &candidate.sensitivity; break;
        case AP_MSG_SET_UV_THR_MIN:     field = &candidate.thr_min; break;
        case AP_MSG_SET_UV_THR_MAX:     field = &candidate.thr_max; break;
        case AP_MSG_SET_UV_WIN_MIN_MS:  field = &candidate.win_min; break;
        case AP_MSG_SET_UV_WIN_MAX_MS:  field = &candidate.win_max; break;
        case AP_MSG_SET_UV_CFM_MIN_MS:  field = &candidate.cfm_min; break;
        case AP_MSG_SET_UV_CFM_MAX_MS:  field = &candidate.cfm_max; break;
        case AP_MSG_SET_UV_PW_MIN_US:   field = &candidate.pw_min_us; break;
        case AP_MSG_SET_UV_PW_MAX_US:   field = &candidate.pw_max_us; break;
        default: return 0U;
    }

    /* 幂等SET不写EEPROM，避免树莓派同步或重发相同配置消耗写入寿命。 */
    if (*field == value) {
        return 1U;
    }
    *field = value;
    if (AP_EEPROM_UV_Save(&candidate) != 0) {
        return 0U;
    }
    apply_uv_config(AP_EEPROM_UV_Get());
    if (message_id == AP_MSG_SET_UV_PW_MIN_US ||
        message_id == AP_MSG_SET_UV_PW_MAX_US) {
        /* Apply the new range without retaining pulses accepted by the old range. */
        AP_UV_ClearPulseHistory();
    }
    return 1U;
}

/**
 * @brief 事务式修改一个IR字段：候选组校验、落盘读回后再更新运行参数。
 * @return 1表示已生效或原值相同，0表示ID、参数或存储失败。
 */
static uint8_t set_ir_value(uint8_t message_id, uint32_t value)
{
    AP_EEPROM_IR_Param_t candidate = *AP_EEPROM_IR_Get();
    uint32_t *field = NULL;

    switch (message_id) {
        case AP_MSG_SET_IR_SENSITIVITY:  field = &candidate.sensitivity; break;
        case AP_MSG_SET_IR_POWER_MIN:    field = &candidate.power_min; break;
        case AP_MSG_SET_IR_POWER_MAX:    field = &candidate.power_max; break;
        case AP_MSG_SET_IR_R38_X1000:    field = &candidate.r38_threshold; break;
        case AP_MSG_SET_IR_R50_X1000:    field = &candidate.r50_threshold; break;
        case AP_MSG_SET_IR_FREQ_LOW_X10: field = &candidate.freq_low_x10; break;
        case AP_MSG_SET_IR_FREQ_HIGH_X10:field = &candidate.freq_high_x10; break;
        case AP_MSG_SET_IR_CFM_MIN_MS:   field = &candidate.cfm_min; break;
        case AP_MSG_SET_IR_CFM_MAX_MS:   field = &candidate.cfm_max; break;
        default: return 0U;
    }

    if (*field == value) {
        return 1U;
    }
    *field = value;
    if (AP_EEPROM_IR_Save(&candidate) != 0) {
        return 0U;
    }
    apply_ir_config(AP_EEPROM_IR_Get());
    return 1U;
}

/** @brief 校验SET的4字节Data，并按MessageID分发到SYSTEM、UV或IR参数组。 */
static uint8_t set_config_value(uint8_t message_id, const uint8_t *data,
                                uint8_t data_len)
{
    uint32_t value;

    if (data_len != 4U) {
        return 0U;
    }
    value = get_u32_le(data);

    if (message_id >= AP_MSG_SET_SHOW_MODE &&
        message_id <= AP_MSG_SET_IR_PROFILE_ENABLED) {
        return set_system_value(message_id, value);
    }
    if (message_id >= AP_MSG_SET_UV_SENSITIVITY &&
        message_id <= AP_MSG_SET_UV_PW_MAX_US) {
        return set_uv_value(message_id, value);
    }
    if (message_id >= AP_MSG_SET_IR_SENSITIVITY &&
        message_id <= AP_MSG_SET_IR_CFM_MAX_MS) {
        return set_ir_value(message_id, value);
    }
    return 0U;
}

/** @brief 按协议固定偏移编码20项配置，禁止直接发送编译器结构体布局。 */
static void build_config_snapshot(uint8_t *snapshot)
{
    const AP_EEPROM_System_Param_t *system = AP_EEPROM_System_Get();
    const AP_EEPROM_UV_Param_t *uv = AP_EEPROM_UV_Get();
    const AP_EEPROM_IR_Param_t *ir = AP_EEPROM_IR_Get();
    uint16_t offset = 0U;

    put_u16_le(&snapshot[offset], AP_UART_CONFIG_SCHEMA_VERSION); offset += 2U;
    put_u16_le(&snapshot[offset], AP_UART_CONFIG_FIELD_COUNT); offset += 2U;

#define PUT_CONFIG_U32(value_) \
    do { put_u32_le(&snapshot[offset], (value_)); offset += 4U; } while (0)
    PUT_CONFIG_U32(system->show_mode);
    PUT_CONFIG_U32(system->ir_profile_enabled);
    PUT_CONFIG_U32(uv->sensitivity);
    PUT_CONFIG_U32(uv->thr_min);
    PUT_CONFIG_U32(uv->thr_max);
    PUT_CONFIG_U32(uv->win_min);
    PUT_CONFIG_U32(uv->win_max);
    PUT_CONFIG_U32(uv->cfm_min);
    PUT_CONFIG_U32(uv->cfm_max);
    PUT_CONFIG_U32(uv->pw_min_us);
    PUT_CONFIG_U32(uv->pw_max_us);
    PUT_CONFIG_U32(ir->sensitivity);
    PUT_CONFIG_U32(ir->power_min);
    PUT_CONFIG_U32(ir->power_max);
    PUT_CONFIG_U32(ir->r38_threshold);
    PUT_CONFIG_U32(ir->r50_threshold);
    PUT_CONFIG_U32(ir->freq_low_x10);
    PUT_CONFIG_U32(ir->freq_high_x10);
    PUT_CONFIG_U32(ir->cfm_min);
    PUT_CONFIG_U32(ir->cfm_max);
#undef PUT_CONFIG_U32

    (void)offset; /* 长度由编译期常量和上方固定字段表共同约束。 */
}

/** @brief 完成新请求：追加F0、保存去重响应，并推进会话序号基准。 */
static void finish_request(uint16_t sequence, uint8_t message_id,
                           const uint8_t *data, uint8_t data_len,
                           AP_UART_Result_t result)
{
    append_f0(sequence, message_id, result);
    cache_current_response(sequence, message_id, data, data_len);
    s_session.latest_sequence = sequence;
    s_session.latest_sequence_valid = 1U;
}

/** @brief 校验非零nonce并建立新会话；完全重复的握手只重放缓存响应。 */
static void handle_handshake(uint16_t sequence, const uint8_t *data,
                             uint8_t data_len, uint32_t now_ms)
{
    uint32_t nonce;

    if (data_len != 4U || (nonce = get_u32_le(data)) == 0U) {
        tx_bundle_begin();
        append_f0(sequence, AP_MSG_HANDSHAKE, AP_UART_RESULT_FAILED);
        return;
    }

    if (s_session.active != 0U && s_session.nonce == nonce &&
        cached_request_matches(sequence, AP_MSG_HANDSHAKE, data, data_len)) {
        s_session.last_rx_ms = now_ms;
        replay_cached_response();
        return;
    }

    /* 新握手建立新的序号域；旧请求和旧应答缓存不得跨会话继续生效。 */
    session_reset();
    s_session.active = 1U;
    s_session.nonce = nonce;
    s_session.last_rx_ms = now_ms;
    tx_bundle_begin();
    finish_request(sequence, AP_MSG_HANDSHAKE, data, data_len,
                   AP_UART_RESULT_SUCCESS);
}

/** @brief 生成固定84字节配置快照，并按“0x81数据帧、F0”顺序应答。 */
static void handle_get_config(uint16_t sequence, const uint8_t *data,
                              uint8_t data_len)
{
    uint8_t response[AP_UART_CONFIG_RESPONSE_LEN];

    tx_bundle_begin();
    if (data_len != 0U) {
        finish_request(sequence, AP_MSG_GET_CONFIG, data, data_len,
                       AP_UART_RESULT_FAILED);
        return;
    }

    /*
     * 成功GET固定先发0x81配置数据、后发F0成功。0x81 Data只包含84字节
     * 配置快照，不混入请求序号；严格停等关系及随后F0负责请求匹配。
     * 任一帧丢失时
     * 树莓派使用原请求重发，从机将按缓存重放完全相同的两帧。
     */
    build_config_snapshot(response);
    (void)tx_bundle_add(AP_MSG_GET_CONFIG, response, sizeof(response));
    finish_request(sequence, AP_MSG_GET_CONFIG, data, data_len,
                   AP_UART_RESULT_SUCCESS);
}

/**
 * @brief 处理已通过基础帧校验的请求，执行建联、去重、新旧序号和业务分发。
 * @note  F0为单向响应，收到后静默丢弃且不刷新会话。
 */
static void process_request(uint16_t sequence, uint8_t message_id,
                            const uint8_t *data, uint8_t data_len,
                            uint32_t now_ms)
{
    uint16_t delta;
    uint8_t success;

    /*
     * F0是从机发送方向的统一应答。若因线路回环或对端误发被本板收到，
     * 只消费不应答，避免把响应再解释成请求并形成无意义的往返消息。
     */
    if (message_id == AP_MSG_COMMAND_RESPONSE) {
        /* F0不是树莓派业务请求，只静默丢弃，不能用于延长配置会话。 */
        return;
    }

    /*
     * 能进入本函数说明版本、长度、CRC和非零Sequence均已校验通过。
     * SET和HANDSHAKE均为4字节小端值，直接打印解析值便于核对树莓派组包；
     * GET_CONFIG等无Data请求只打印长度。业务是否执行成功由后续RESP日志给出。
     */
    if (data_len == 4U) {
        BSP_UART_Printf("[PROTO] RX OK SEQ=%u ID=0x%02X LEN=%u VALUE=%lu\r\n",
                        (unsigned int)sequence,
                        (unsigned int)message_id,
                        (unsigned int)data_len,
                        (unsigned long)get_u32_le(data));
    } else {
        BSP_UART_Printf("[PROTO] RX OK SEQ=%u ID=0x%02X LEN=%u\r\n",
                        (unsigned int)sequence,
                        (unsigned int)message_id,
                        (unsigned int)data_len);
    }

    if (message_id == AP_MSG_HANDSHAKE) {
        handle_handshake(sequence, data, data_len, now_ms);
        return;
    }

    tx_bundle_begin();
    if (s_session.active == 0U) {
        append_f0(sequence, message_id, AP_UART_RESULT_FAILED);
        return;
    }
    s_session.last_rx_ms = now_ms;

    if (s_session.latest_sequence_valid != 0U) {
        delta = (uint16_t)(sequence - s_session.latest_sequence);
        if (delta == 0U) {
            if (cached_request_matches(sequence, message_id, data, data_len)) {
                replay_cached_response();
            } else {
                /* 同序号内容冲突只失败应答，不覆盖最后一次成功缓存。 */
                tx_bundle_begin();
                append_f0(sequence, message_id, AP_UART_RESULT_FAILED);
            }
            return;
        }
        if (sequence_is_newer(sequence, s_session.latest_sequence) == 0U) {
            /* 会话内旧请求拒绝执行，也不推进最新序号。 */
            append_f0(sequence, message_id, AP_UART_RESULT_FAILED);
            return;
        }
    }

    if (message_id == AP_MSG_GET_CONFIG) {
        handle_get_config(sequence, data, data_len);
        return;
    }

    success = set_config_value(message_id, data, data_len);
    finish_request(sequence, message_id, data, data_len,
                   success ? AP_UART_RESULT_SUCCESS : AP_UART_RESULT_FAILED);
}

/** @brief 校验完整候选帧的CRC和Sequence，通过后交给请求处理器。 */
static void validate_complete_frame(uint32_t now_ms)
{
    uint8_t payload_len = s_rx_frame[3];
    uint16_t crc_offset = 4U + payload_len;
    uint16_t received_crc = get_u16_le(&s_rx_frame[crc_offset]);
    uint16_t calculated_crc = CRC16_CCITT(&s_rx_frame[4], payload_len);
    uint16_t sequence;
    uint8_t message_id;
    uint8_t data_len;

    if (received_crc != calculated_crc) {
        return;
    }

    sequence = get_u16_le(&s_rx_frame[4]);
    if (sequence == 0U) {
        return;
    }
    message_id = s_rx_frame[6];
    data_len = (uint8_t)(payload_len - AP_UART_BODY_FIXED_LEN);
    process_request(sequence, message_id, &s_rx_frame[7], data_len, now_ms);
}

/**
 * @brief 逐字节搜索SOF并收集候选帧，支持DMA拆包和粘包。
 * @return 1表示候选帧已完成或已判非法，本轮RX任务应停止取数。
 */
static uint8_t parser_feed(uint8_t byte, uint32_t now_ms)
{
    switch (s_rx_state) {
        case RX_PARSE_SOF0:
            if (byte == AP_UART_SOF_0) {
                s_rx_frame[0] = byte;
                s_rx_pos = 1U;
                s_rx_start_ms = now_ms;
                s_rx_state = RX_PARSE_SOF1;
            }
            break;

        case RX_PARSE_SOF1:
            if (byte == AP_UART_SOF_1) {
                s_rx_frame[s_rx_pos++] = byte;
                s_rx_state = RX_PARSE_FRAME;
            } else if (byte == AP_UART_SOF_0) {
                /* 连续AA时保留最后一个AA作为新候选帧起点。 */
                s_rx_frame[0] = byte;
                s_rx_pos = 1U;
                s_rx_start_ms = now_ms;
            } else {
                parser_reset();
            }
            break;

        case RX_PARSE_FRAME:
            if (s_rx_pos >= AP_UART_FRAME_MAX) {
                parser_reset();
                return 1U;
            }
            s_rx_frame[s_rx_pos++] = byte;
            if (s_rx_pos == 4U) {
                uint8_t payload_len = s_rx_frame[3];
                if (s_rx_frame[2] != AP_UART_PROTOCOL_VERSION ||
                    payload_len < AP_UART_BODY_FIXED_LEN ||
                    payload_len > AP_UART_PAYLOAD_LEN_MAX) {
                    parser_reset();
                    return 1U;
                }
                s_rx_expected = (uint16_t)(6U + payload_len);
            }
            if (s_rx_expected > 0U && s_rx_pos == s_rx_expected) {
                validate_complete_frame(now_ms);
                parser_reset();
                return 1U;
            }
            break;

        default:
            parser_reset();
            break;
    }
    return 0U;
}

/** @brief 初始化解析、响应、会话、去重和诊断模式状态。 */
void AP_UART_ProtocolInit(void)
{
    parser_reset();
    memset(&s_tx, 0, sizeof(s_tx));
    session_reset();
    s_tx_sequence = 0U;
    s_diagnostic_mode = 0U;
}

/** @brief 有界消费USART2 DMA数据；严格停等期间不解析下一请求。 */
void AP_UART_RxTask(void)
{
    uint8_t byte;
    uint8_t transport_fault;
    uint32_t now_ms;

    if (s_diagnostic_mode != 0U) {
        return;
    }

    /* 两个故障标志均为读取后清除，必须每轮分别读取，不能使用短路表达式。 */
    transport_fault = (uint8_t)(BSP_UART_TxFaulted(BSP_UART_COM) ? 1U : 0U);
    transport_fault |= (uint8_t)(BSP_UART_RxOverflowed(BSP_UART_COM) ? 1U : 0U);
    if (transport_fault != 0U) {
        abort_transport();
        return;
    }

    now_ms = HAL_GetTick();
    /* 半帧超过100ms后先丢弃，再从新到达数据中搜索SOF。 */
    if (s_rx_state != RX_PARSE_SOF0 &&
        (uint32_t)(now_ms - s_rx_start_ms) >= AP_UART_FRAME_TIMEOUT_MS) {
        parser_reset();
    }

    /* 响应尚未物理发完时保持严格一问一答，不提前执行下一条配置。 */
    if (s_tx.active != 0U) {
        return;
    }

    for (uint16_t i = 0U; i < AP_UART_RX_BUDGET_PER_TASK; i++) {
        if (BSP_UART_Read(BSP_UART_COM, &byte, 1U) != 1U) {
            break;
        }
        if (parser_feed(byte, now_ms) != 0U) {
            break;
        }
    }
}

/** @brief 非阻塞推进响应入队，并等待USART2 DMA物理发送完成。 */
void AP_UART_TxTask(void)
{
    uint16_t remaining;
    uint16_t accepted;

    if (s_diagnostic_mode != 0U) {
        return;
    }

    if (s_tx.active == 0U) {
        return;
    }

    if (BSP_UART_TxFaulted(BSP_UART_COM)) {
        abort_transport();
        return;
    }

    if (s_tx.frame_index < s_tx.frame_count) {
        remaining = (uint16_t)(s_tx.length[s_tx.frame_index] - s_tx.offset);
        if (remaining > 0U) {
            accepted = BSP_UART_Write(BSP_UART_COM,
                                      &s_tx.frame[s_tx.frame_index][s_tx.offset],
                                      remaining);
            s_tx.offset = (uint16_t)(s_tx.offset + accepted);
            if (accepted > 0U) {
                s_tx.last_progress_ms = HAL_GetTick();
            }
            return;
        }
        s_tx.frame_index++;
        s_tx.offset = 0U;
        return;
    }

    /* 全部字节入队后继续等待DMA物理发送完成，再允许解析下一请求。 */
    if (BSP_UART_IsTxComplete(BSP_UART_COM)) {
        memset(&s_tx, 0, sizeof(s_tx));
    }
}

/** @brief 处理半帧、响应发送和空闲会话三类超时。 */
void AP_UART_CheckTimeout(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (s_diagnostic_mode != 0U) {
        return;
    }

    if (s_rx_state != RX_PARSE_SOF0 &&
        (uint32_t)(now_ms - s_rx_start_ms) >= AP_UART_FRAME_TIMEOUT_MS) {
        parser_reset();
    }

    if (s_tx.active != 0U &&
        (uint32_t)(now_ms - s_tx.last_progress_ms) >= AP_UART_TX_TIMEOUT_MS) {
        /* 正常128字节帧远快于500ms；超时说明发送链路已失去进展。 */
        abort_transport();
        return;
    }

    if (s_session.active != 0U && s_rx_state == RX_PARSE_SOF0 &&
        s_tx.active == 0U && BSP_UART_IsTxComplete(BSP_UART_COM) &&
        BSP_UART_GetRxDataLen(BSP_UART_COM) == 0U &&
        (uint32_t)(now_ms - s_session.last_rx_ms) >= AP_UART_SESSION_TIMEOUT_MS) {
        /* 会话超时同步清除旧序号和去重缓存，下一次必须重新握手。 */
        session_reset();
    }
}

/** @brief 判断解析器、响应事务和底层物理发送是否全部空闲。 */
bool AP_UART_IsIdle(void)
{
    return (s_rx_state == RX_PARSE_SOF0) && (s_tx.active == 0U) &&
           BSP_UART_IsTxComplete(BSP_UART_COM);
}

/** @brief 切换USART2原始诊断独占模式；切换边界会清除协议残帧和会话。 */
void AP_UART_SetDiagnosticMode(bool enabled)
{
    uint8_t next = enabled ? 1U : 0U;

    if (s_diagnostic_mode == next) {
        return;
    }

    /*
     * 原始字节与协议帧不能共用同一数据流。切换边界直接废弃会话和残帧，
     * 恢复正式协议后树莓派必须重新握手，避免把测试数据解释成配置请求。
     */
    abort_transport();
    s_diagnostic_mode = next;
}

/** @brief 返回USART2是否正由本地原始诊断命令独占。 */
bool AP_UART_GetDiagnosticMode(void)
{
    return (s_diagnostic_mode != 0U);
}
