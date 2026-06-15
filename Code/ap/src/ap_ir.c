/**
  ******************************************************************************
  * @file    ap_ir.c
  * @brief   AP 层红外检测模块实现（预留框架）
  *
  *          每路 IR 通道独立维护状态，主函数体暂为 TODO。
  ******************************************************************************
  */

#include "ap_ir.h"
#include <string.h>

/* ========================================================================== */
/*                         内部数据结构                                        */
/* ========================================================================== */

typedef struct {
    uint32_t            threshold;
    uint32_t            hysteresis;
    IR_DetectorState_t  state;
} IR_Channel_t;

static IR_Channel_t s_ir_ch[AP_IR_NUM_CHANNELS];
static uint8_t s_initialized = 0;

/* ========================================================================== */
/*                         公有 API 实现                                       */
/* ========================================================================== */

void AP_IR_Init(uint32_t idx, uint32_t threshold, uint32_t hysteresis)
{
    if (idx >= AP_IR_NUM_CHANNELS) return;

    if (!s_initialized) {
        memset(s_ir_ch, 0, sizeof(s_ir_ch));
        s_initialized = 1;
    }

    s_ir_ch[idx].threshold  = threshold;
    s_ir_ch[idx].hysteresis = hysteresis;
    s_ir_ch[idx].state      = IR_STATE_IDLE;
}

void AP_IR_Update(uint32_t idx, uint16_t raw)
{
    if (idx >= AP_IR_NUM_CHANNELS) return;

    IR_Channel_t *ch = &s_ir_ch[idx];

    /* TODO: 滑动滤波 */
    (void)raw;
    (void)ch;

    /* TODO: 阈值比较 + 回滞控制 + 状态迁移 */
    /* if (ch->state == IR_STATE_IDLE && filtered > ch->threshold) */
    /*     ch->state = IR_STATE_ALARM; */
    /* if (ch->state == IR_STATE_ALARM && filtered < ch->threshold - ch->hysteresis) */
    /*     ch->state = IR_STATE_IDLE; */
}

IR_DetectorState_t AP_IR_GetState(uint32_t idx)
{
    if (idx >= AP_IR_NUM_CHANNELS) return IR_STATE_IDLE;
    return s_ir_ch[idx].state;
}
