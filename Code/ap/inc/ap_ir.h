/**
  ******************************************************************************
  * @file    ap_ir.h
  * @brief   AP 层红外三波段多光谱融合火焰检测模块
  *
  *          基于三路 RPFA913CC 热释电传感器(3.8μm/4.5μm/5.0μm)的
  *          多光谱融合火焰检测。算法流程：
  *            采集→连续20Hz IIR低通→历史缓存(256点@100Hz)
  *            →50点DC/功率+256点FFT频域特征+光谱比→五判据串联
  *            →IR报警状态机
  *          同时，P45首次跨过门槛独立启动点火包络分类：
  *            1.5秒峰值+最多1.5秒OFF以上有效均值→LIGHTER/SUSTAINED
  *          包络分类与WARNING证据积分并行；若证据先满足，则在进入FIRE前
  *          立即使用已采稳定样本提前分类，不要求稳定窗补满。仅PEAK≥10万
  *          且late/peak<50%才判LIGHTER；
  *          LIGHTER阻止FIRE；后续每1秒统计一次有效平均能量，单窗满足
  *          有效占比≥60%后走两条独立恢复路径：恢复至峰值30%连续2窗，
  *          或均值≥5万连续5窗；任一路径完成后单向升级为SUSTAINED。
  *          SUSTAINED禁止反向降级，分类数据不足时按放行处理。
  *          若观察期间从未进入WARNING且P45低于OFF满1秒，则丢弃该短瞬态并
  *          直接重新ARMED，不生成分类终态。
  *          若火焰已存在且持续波动导致BYPASS无法安静布防，仅在当周期P45达到
  *          ON且正常IR光谱/频率判据通过时按热启动接纳为SUSTAINED；
  *          不读取掉线尾段遗留的旧WARNING，也不绕过原五判据和确认积分。
  *
  *          == 参数存储(EEPROM) ==
  *          4.5um进场功率与确认时长以 min/max 范围存储，等级0~9线性插值。
  *          光谱比和频率范围为固定值，不随等级变化：
  *            power:    平均功率进场阈值(去直流信号均方值)
  *            cfm:      确认时长(ms)
  *            r38:      R4.5/3.8 光谱比阈值(×1000)
  *            r50:      R4.5/5.0 光谱比阈值(×1000)
  *            freq_low:  频率下限(×10)
  *            freq_high: 频率上限(×10)
  *
  *          == ISR/主循环分离 ==
  *            AP_IR_FeedIsr():    ISR(TIM6)调用，仅置位 volatile 标志
  *            AP_IR_Task():       主循环调用，检查标志→Feed→Process
  *
  *          == 使用示例 ==
  *          // 初始化(在 AP_EEPROM_Init 之后)
  *          AP_IR_Init();
  *
  *          // TIM6 10ms ISR:
  *          AP_IR_FeedIsr();
  *
  *          // 主循环:
  *          AP_IR_Task();  // 内部调 Feed() + Process()
  *
  *          // 获取状态:
  *          if (AP_IR_GetState() == IR_STATE_FIRE) { ... }
  ******************************************************************************
  */
#ifndef __AP_IR_H__
#define __AP_IR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "ap_util.h"

/* ========================================================================== */
/*                          常量宏                                             */
/* ========================================================================== */

#define IR_SENS_LEVELS          (10U)       /* 灵敏度 0~9, 同步 UV          */
#define IR_CH_NUM               (3U)        /* 3 通道(3.8/4.5/5.0 μm)      */
/* FFT使用2.56秒长窗提高低频分辨率；DC/功率仍保持原0.5秒标定量纲。 */
#define IR_HISTORY_SIZE          (256U)     /* 历史缓存容量: 2.56s @100Hz  */
#define IR_DC_POWER_WINDOW_SIZE   (50U)     /* DC/功率窗口: 0.5s @100Hz   */
#define IR_FFT_WINDOW_SIZE          (256U)     /* FFT窗口: 2.56s，频率分辨率0.390625Hz */

#if (IR_DC_POWER_WINDOW_SIZE > IR_HISTORY_SIZE) || \
    (IR_FFT_WINDOW_SIZE > IR_HISTORY_SIZE)
#error "IR feature window must not exceed IR history capacity"
#endif

#if (IR_FFT_WINDOW_SIZE == 0U) || \
    ((IR_FFT_WINDOW_SIZE & (IR_FFT_WINDOW_SIZE - 1U)) != 0U)
#error "IR FFT window must be a non-zero power of two"
#endif

/* ap_ir.c 的周期 Hann 步进常量按 256 点离线量化；修改窗口时必须同步重算常量。 */
#if (IR_FFT_WINDOW_SIZE != 256U)
#error "IR FFT window requires matching fixed-point Hann constants"
#endif

/* 通道索引 (CubeMX 标签: IR_OUT_3/2/1) */
#define IR_CH_REF_A             (0U)        /* 3.8μm — PC0/IN10 = IR_OUT_3 */
#define IR_CH_MAIN              (1U)        /* 4.5μm — PC1/IN11 = IR_OUT_2 */
#define IR_CH_REF_B             (2U)        /* 5.0μm — PC2/IN12 = IR_OUT_1 */

/* ========================================================================== */
/*                          状态枚举                                           */
/* ========================================================================== */

typedef enum {
    IR_STATE_IDLE = 0,
    IR_STATE_WARNING,
    IR_STATE_FIRE
} IR_DetectorState_t;

/* ========================================================================== */
/*                          API 声明                                           */
/* ========================================================================== */

/* ---- 初始化 & 任务 ------------------------------------------------------ */

/**
  * @brief  初始化红外检测模块
  *         从 EEPROM 加载参数, 清空历史窗口, 状态机置 IDLE
  */
void AP_IR_Init(void);

/**
  * @brief  ISR 中调用 — 置位 feed_pending 标志
  *         由 TIM6 的 task_10ms() 每 10ms 调用一次
  */
void AP_IR_FeedIsr(void);

/**
  * @brief  从 ADC 读最新值，经连续 IIR 后推入 128 点历史窗口（不进状态机）
  */
void AP_IR_Feed(void);

/**
  * @brief  主循环中调用 — 检查标志→Feed→Process
  *         若 feed_pending 置位则:
  *           1. 清标志
  *           2. AP_IR_Feed(): 读最新 ADC 值并更新连续 IIR/历史窗口
  *           3. AP_IR_Process(): 窗口满后执行全流程检测
  */
void AP_IR_Task(void);

/* ---- 灵敏度等级 0~9: 功率与确认时长按 min→max 线性插值 ------------------ */

void AP_IR_SetLevel(uint8_t level);
void AP_IR_GetLevel(uint8_t *level);

/* ---- 参数配置 (由 CLI + EEPROM 调用) ------------------------------------ */

void AP_IR_SetConfig(uint32_t power_min, uint32_t power_max,
                     uint32_t r38_threshold, uint32_t r50_threshold,
                     uint32_t freq_low_x10, uint32_t freq_high_x10,
                     uint32_t cfm_min, uint32_t cfm_max);

void AP_IR_GetConfig(uint32_t *power_min, uint32_t *power_max,
                     uint32_t *r38_threshold, uint32_t *r50_threshold,
                     uint32_t *freq_low_x10, uint32_t *freq_high_x10,
                     uint32_t *cfm_min, uint32_t *cfm_max);

/* ---- 当前运行参数 ------------------------------------------------------- */

void AP_IR_GetParams(uint32_t *pwr, uint32_t *r38, uint32_t *r50,
                     uint32_t *freq_low, uint32_t *freq_high, uint32_t *cfm);

/* ---- 状态管理 ----------------------------------------------------------- */

IR_DetectorState_t AP_IR_GetState(void);
void AP_IR_Reset(void);

/* ---- 调试特征值 --------------------------------------------------------- */

void AP_IR_GetFeatures(uint32_t power[3], float dominant_freq[3],
                       uint32_t *r45_38, uint32_t *r45_50);
/**
  * @brief  测试模式: 执行完整信号处理链，打印中间结果，不进状态机
  *         每 100Hz 调用一次，输出行格式:
  *           T<ms> IRD DC=<c0>,<c1>,<c2> P=<p0>,<p1>,<p2> F10=<f0,f1,f2> R1000=<r38,r50>
  * @param  now: HAL_GetTick()
  */
/**
  * @brief  测试模式: 输出三路 ADC 原始值、均方值、窗口平均能量
  *         输出: RAW=... P=... E=...
  * @param  now: HAL_GetTick()
  */
void AP_IR_TestPrint(uint32_t now);
void AP_IR_TestSetAvgWindowMs(uint32_t ms);
uint32_t AP_IR_TestGetAvgWindowMs(void);

/**
  * @brief  测试模式: 执行完整信号处理链，打印中间结果，不进状态机
  *         每 100Hz 调用一次，输出行格式:
  *           T<ms> IRD DC=<c0>,<c1>,<c2> P=<p0>,<p1>,<p2> F10=<f0,f1,f2> R1000=<r38,r50>
  * @param  now: HAL_GetTick()
  */
void AP_IR_DebugProcess(uint32_t now);

/** @brief 运行时启停点火包络分类；关闭时不再以LIGHTER阻止IR报警。 */
void AP_IR_SetProfileEnabled(uint8_t enabled);
uint8_t AP_IR_GetProfileEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __AP_IR_H__ */
