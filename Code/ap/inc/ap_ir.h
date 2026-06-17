/**
  ******************************************************************************
  * @file    ap_ir.h
  * @brief   AP 层红外三波段多光谱融合火焰检测模块
  *
  *          基于三路 RPFA913CC 热释电传感器(3.8μm/4.5μm/5.0μm)的
  *          多光谱融合火焰检测。算法流程：
  *            采集→历史窗口(50点@100Hz)→预处理(去直流+40Hz IIR低通)
  *            →特征提取(平均功率+过零率+光谱比)→五判据串联→状态机
  *
  *          == 参数存储(EEPROM) ==
  *          4 项核心参数以 min/max 范围存储，灵敏度 0~9 级线性插值：
  *            pwr:    功率阈值(×1000)
  *            r38:    R4.5/3.8 光谱比(×1000)
  *            r50:    R4.5/5.0 光谱比(×1000)
  *            cfm:    确认时长(ms)
  *          频率范围为固定值(火焰频率不随灵敏度变化):
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

/* ========================================================================== */
/*                          常量宏                                             */
/* ========================================================================== */

#define IR_SENS_LEVELS          (10U)       /* 灵敏度 0~9, 同步 UV          */
#define IR_CH_NUM               (3U)        /* 3 通道(3.8/4.5/5.0 μm)      */
#define IR_HISTORY_SIZE         (50U)       /* 50点 = 0.5s @100Hz          */

/* 通道索引 (硬件映射: PC0/IN10, PC1/IN11, PC2/IN12) */
#define IR_CH_REF_A             (0U)        /* 3.8μm 参考通道A(高温热源)    */
#define IR_CH_MAIN              (1U)        /* 4.5μm 主信号通道(火焰特征)   */
#define IR_CH_REF_B             (2U)        /* 5.0μm 参考通道B(背景辐射)    */

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
  * @brief  主循环中调用 — 检查标志→Feed→Process
  *         若 feed_pending 置位则:
  *           1. 清标志
  *           2. AP_IR_Feed(): 读最新 ADC 值推入 50 点历史窗口
  *           3. AP_IR_Process(): 窗口满后执行全流程检测
  */
void AP_IR_Task(void);

/* ---- 灵敏度 0~9 (同步 UV 的线性插值模式) -------------------------------- */

void AP_IR_SetLevel(uint8_t level);
void AP_IR_GetLevel(uint8_t *level);

/* ---- min/max 范围配置 (由 CLI + EEPROM 调用) ---------------------------- */

void AP_IR_SetConfig(uint32_t pwr_min, uint32_t pwr_max,
                     uint32_t r38_min, uint32_t r38_max,
                     uint32_t r50_min, uint32_t r50_max,
                     uint32_t freq_low_x10, uint32_t freq_high_x10,
                     uint32_t cfm_min, uint32_t cfm_max);

void AP_IR_GetConfig(uint32_t *pwr_min, uint32_t *pwr_max,
                     uint32_t *r38_min, uint32_t *r38_max,
                     uint32_t *r50_min, uint32_t *r50_max,
                     uint32_t *freq_low_x10, uint32_t *freq_high_x10,
                     uint32_t *cfm_min, uint32_t *cfm_max);

/* ---- 当前运行参数 ------------------------------------------------------- */

void AP_IR_GetParams(uint32_t *pwr, uint32_t *r38, uint32_t *r50,
                     uint32_t *freq_low, uint32_t *freq_high, uint32_t *cfm);

/* ---- 状态管理 ----------------------------------------------------------- */

IR_DetectorState_t AP_IR_GetState(void);
void AP_IR_Reset(void);

/* ---- 调试特征值 --------------------------------------------------------- */

void AP_IR_GetFeatures(uint32_t power[3], float zcr[3],
                       uint32_t *r45_38, uint32_t *r45_50);

#ifdef __cplusplus
}
#endif

#endif /* __AP_IR_H__ */
