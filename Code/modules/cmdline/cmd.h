#ifndef __CMD_PARSER_H__
#define __CMD_PARSER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 *   命令行解析模块（独立 RX 任务）
 *
 *   自己是独立的 RX 任务，无外部传参，内部从 DBG 串口缓冲读取字节并解析。
 *   检测到结束符 (\r\n) 后解析整行命令，查表分发到对应的处理函数。
 *
 *   使用方式（主循环中调用）：
 *     cmd_parser_task();   // 自动读取 DBG RX 缓冲并解析命令
 *
 *   已有命令：
 *     help          — 列出所有命令及用法
 *     adc           — 读取 ADC 通道值 / 设置参数
 *     uv            — 查询紫外检测状态 / 设置灵敏度
 *     param         — 获取/设置运行参数
 *     state         — 打印系统状态
 *     reset         — 软件复位 MCU
 *     debug         — 开关调试打印
 * ========================================================================== */

void cmd_parser_task(void);     // 主循环调用，自动从 DBG 串口读取字节并解析

#ifdef __cplusplus
}
#endif

#endif /* __CMD_PARSER_H__ */
