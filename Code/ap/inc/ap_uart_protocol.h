/**
  ******************************************************************************
  * @file    ap_uart_protocol.h
  * @brief   树莓派与火焰探测板 USART2 配置协议
  *
  *          线格式沿用 bot_wheel：
  *          AA 55 | Version | PayloadLen | SequenceLE | MessageID |
  *          Data | CRC16/CCITT-FALSE LE
  *
  *          本板是协议从机，只响应树莓派请求，不主动发送串口状态帧。
  *          树莓派每30秒查询运行状态；ALARM/BUG电平变化后分别查询详情。
  *          COMMAND_RESPONSE(0xF0)固定为6字节，各只读查询先返回数据帧，
  *          再使用F0结束本次严格停等事务。
  ******************************************************************************
  */
#ifndef __AP_UART_PROTOCOL_H__
#define __AP_UART_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* 线协议固定字段。PayloadLen包含Sequence、MessageID和业务Data。 */
#define AP_UART_SOF_0                    (0xAAU)
#define AP_UART_SOF_1                    (0x55U)
#define AP_UART_PROTOCOL_VERSION         (0x10U)
#define AP_UART_BODY_FIXED_LEN           (3U)
#define AP_UART_FRAME_MAX                (128U)
#define AP_UART_FRAME_OVERHEAD           (9U)
#define AP_UART_DATA_MAX                 (AP_UART_FRAME_MAX - AP_UART_FRAME_OVERHEAD)
#define AP_UART_PAYLOAD_LEN_MAX          (AP_UART_BODY_FIXED_LEN + AP_UART_DATA_MAX)

#define AP_UART_FRAME_TIMEOUT_MS         (100U)
#define AP_UART_TX_TIMEOUT_MS            (500U)
#define AP_UART_SESSION_TIMEOUT_MS       (10000U)
#define AP_UART_RX_BUDGET_PER_TASK       (32U)
#define AP_UART_CONFIG_SCHEMA_VERSION    (1U)
#define AP_UART_CONFIG_FIELD_COUNT       (20U)
#define AP_UART_STATUS_SCHEMA_VERSION    (1U)
#define AP_UART_DEVICE_INFO_SCHEMA_VERSION (1U)

/* 固件版本线格式：major.minor.patch.build，各段占8位。 */
#define AP_FW_VERSION_MAJOR              (1U)
#define AP_FW_VERSION_MINOR              (1U)
#define AP_FW_VERSION_PATCH              (0U)
#define AP_FW_VERSION_BUILD              (0U)
#define AP_FW_VERSION_U32 \
    ((AP_FW_VERSION_MAJOR << 24) | (AP_FW_VERSION_MINOR << 16) | \
     (AP_FW_VERSION_PATCH << 8) | AP_FW_VERSION_BUILD)

/* 树莓派请求、板端数据响应及统一结果应答的MessageID。 */
typedef enum {
    AP_MSG_HANDSHAKE                  = 0x80,
    AP_MSG_GET_CONFIG                = 0x81,
    AP_MSG_GET_STATUS                = 0x82,
    AP_MSG_STATUS_DATA               = 0x83,
    AP_MSG_GET_ALARM_DETAIL          = 0x84,
    AP_MSG_ALARM_DETAIL              = 0x85,
    AP_MSG_GET_FAULT_DETAIL          = 0x86,
    AP_MSG_FAULT_DETAIL              = 0x87,
    AP_MSG_GET_DEVICE_INFO           = 0x88,
    AP_MSG_DEVICE_INFO               = 0x89,

    AP_MSG_SET_SHOW_MODE             = 0x90,
    AP_MSG_SET_IR_PROFILE_ENABLED    = 0x91,
    AP_MSG_FACTORY_RESET             = 0x92,

    AP_MSG_SET_UV_SENSITIVITY        = 0xA0,
    AP_MSG_SET_UV_THR_MIN            = 0xA1,
    AP_MSG_SET_UV_THR_MAX            = 0xA2,
    AP_MSG_SET_UV_WIN_MIN_MS         = 0xA3,
    AP_MSG_SET_UV_WIN_MAX_MS         = 0xA4,
    AP_MSG_SET_UV_CFM_MIN_MS         = 0xA5,
    AP_MSG_SET_UV_CFM_MAX_MS         = 0xA6,
    AP_MSG_SET_UV_PW_MIN_US          = 0xA7,
    AP_MSG_SET_UV_PW_MAX_US          = 0xA8,

    AP_MSG_SET_IR_SENSITIVITY        = 0xB0,
    AP_MSG_SET_IR_POWER_MIN          = 0xB1,
    AP_MSG_SET_IR_POWER_MAX          = 0xB2,
    AP_MSG_SET_IR_R38_X1000          = 0xB3,
    AP_MSG_SET_IR_R50_X1000          = 0xB4,
    AP_MSG_SET_IR_FREQ_LOW_X10       = 0xB5,
    AP_MSG_SET_IR_FREQ_HIGH_X10      = 0xB6,
    AP_MSG_SET_IR_CFM_MIN_MS         = 0xB7,
    AP_MSG_SET_IR_CFM_MAX_MS         = 0xB8,

    AP_MSG_COMMAND_RESPONSE          = 0xF0,
} AP_UART_MessageId_t;

/* 协议按需求只暴露成功/失败；error_code固定保留为0以兼容bot_wheel F0布局。 */
typedef enum {
    AP_UART_RESULT_SUCCESS = 0,
    AP_UART_RESULT_FAILED  = 1,
} AP_UART_Result_t;

/** @brief 初始化解析器、发送器、会话和请求去重状态。 */
void AP_UART_ProtocolInit(void);

/** @brief 主循环调用：有界读取USART2 DMA数据并处理最多一条请求。 */
void AP_UART_RxTask(void);

/** @brief 主循环调用：非阻塞地把当前响应帧推进USART2 TX DMA队列。 */
void AP_UART_TxTask(void);

/** @brief 主循环调用：处理100ms半帧、500ms发送和10s会话超时。 */
void AP_UART_CheckTimeout(void);

/** @brief 查询协议解析、响应发送及USART2物理发送是否均为空闲。 */
bool AP_UART_IsIdle(void);

/**
 * @brief  切换USART2原始诊断模式。
 * @note   开启或关闭时都会清除当前协议会话及收发残留；开启后协议任务暂停，
 *         USART2只允许测试命令直接访问。
 */
void AP_UART_SetDiagnosticMode(bool enabled);

/** @brief 查询USART2是否处于原始诊断独占模式。 */
bool AP_UART_GetDiagnosticMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __AP_UART_PROTOCOL_H__ */
