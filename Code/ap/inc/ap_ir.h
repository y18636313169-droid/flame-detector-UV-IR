/**
  ******************************************************************************
  * @file    ap_ir.h
  * @brief   AP 层红外检测模块 — 3 通道热电堆火焰检测
  *
  *          基于 ADC 采样的 RPFA913CC 热释电传感器信号，
  *          对每路做滑动滤波、阈值比较、状态管理。
  *
  *          == 使用（预留框架，待实现）==
  *          // 主循环中
  *          for (int i = 0; i < 3; i++) {
  *              uint16_t raw = AP_ADC_GetLatest(i);
  *              AP_IR_Update(i, raw);
  *          }
  ******************************************************************************
  */
#ifndef __AP_IR_H__
#define __AP_IR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define AP_IR_NUM_CHANNELS      (3U)

/** @brief  每路 IR 检测器状态 */
typedef enum {
    IR_STATE_IDLE = 0,          /* 低于阈值，正常 */
    IR_STATE_ALARM              /* 超阈值，火警 */
} IR_DetectorState_t;

/* ========================================================================== */
/*                         公有 API                                            */
/* ========================================================================== */

/**
  * @brief  初始化 IR 检测器（预留）
  * @param  idx: 通道索引 0~2
  * @param  threshold: ADC 阈值
  * @param  hysteresis: 回滞值
  */
void AP_IR_Init(uint32_t idx, uint32_t threshold, uint32_t hysteresis);

/**
  * @brief  更新单路 IR 检测（预留）
  *         内部做滤波 + 阈值比较 + 状态更新
  * @param  idx:  通道索引 0~2
  * @param  raw:  ADC 最新原始值
  */
void AP_IR_Update(uint32_t idx, uint16_t raw);

/**
  * @brief  获取单路 IR 状态（预留）
  * @param  idx: 通道索引 0~2
  * @return IR_STATE_IDLE / IR_STATE_ALARM
  */
IR_DetectorState_t AP_IR_GetState(uint32_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __AP_IR_H__ */
