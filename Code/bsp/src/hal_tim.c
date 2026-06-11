/**
  ******************************************************************************
  * @file    hal_tim.c
  * @brief   BSP 定时器抽象层实现 — 单通道极性切换脉宽捕获 + 环形缓冲输出
  *
  *          使用 TIM2 CH1 单通道，通过软件手动切换捕获极性，配合溢出计数，
  *          测量 C10807 UV TRON 输出脉冲的高电平宽度。
  *
  *          == 架构 ==
  *          ┌──────────────┐     脉冲数据       ┌──────────────┐
  *          │  TIM2 ISR    │ ── push ────────→  │  环形缓冲区   │
  *          │  (stm32l1xx  │                    │  (容量 20)    │
  *          │   _it.c)     │                    └──────┬───────┘
  *          │    ↓ HAL     │                           │ pop
  *          │  main.c     │                           ↓
  *          │  (应用层分发) │                   ┌──────────────┐
  *          │    ↓ BSP     │                   │  主循环       │
  *          │  Capture/    │                   │  (消费者)     │
  *          │  Period      │                   └──────────────┘
  *          │  Handler     │
  *          └──────────────┘
  *
  *          == 状态机 ==
  *          CAPTURE_IDLE ──(上升沿)──→ CAPTURE_RISING_EDGE
  *                                        │  (overflow_cnt++)
  *                                        ↓  (下降沿)
  *          CAPTURE_COMPLETE ──(自动续测)──→ (推入环形缓冲)
  *
  *          == 主循环调用示例 ==
  *          BSP_TIM_IC_Init(BSP_TIM_UV);
  *          BSP_TIM_IC_Start(BSP_TIM_UV);
  *          while (1) {
  *              BSP_TIM_PulseData_t d;
  *              if (BSP_TIM_IC_ReadPulse(BSP_TIM_UV, &d) == 0) {
  *                  // d.pulse_width_us = 高电平时长(μs)
  *                  // d.timestamp_ms   = HAL_GetTick() 时间戳
  *              }
  *          }
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hal_tim.h"
#include "tim.h"            /* 引用 htim3 */

/* ---------------------------------------------------------------------------*/
/*                     私有宏                                                  */
/* ---------------------------------------------------------------------------*/

/** @brief  从计数值换算为微秒 */
#define TICKS_TO_US(ticks, tick_us)  ((uint32_t)((float)(ticks) * (tick_us)))

/* ---------------------------------------------------------------------------*/
/*                     控制块                                                  */
/* ---------------------------------------------------------------------------*/

typedef struct {
    TIM_HandleTypeDef               *handle;        /* HAL 句柄          */
    volatile BSP_TIM_CaptureState_t state;          /* 捕获状态机        */
    volatile uint32_t               overflow_cnt;   /* 高电平期间溢出    */
    float                           tick_us;        /* 每 tick 微秒数    */
    uint8_t                         initialized;

    /* === 环形缓冲区：ISR 写入，主循环读取 === */
    ring_buffer_t                   pulse_rb;
    BSP_TIM_PulseData_t             pulse_pool[BSP_TIM_UV_PULSE_POOL_SIZE];
} BSP_TIM_Ctrl_t;

/* Private variables ---------------------------------------------------------*/

static BSP_TIM_Ctrl_t tim_ctrl[BSP_TIM_NUM] = {
    [BSP_TIM_UV] = { .handle = NULL, .state = CAPTURE_IDLE }
};

/* ---------------------------------------------------------------------------*/
/*                         内部辅助函数                                        */
/* ---------------------------------------------------------------------------*/

static inline int is_valid_id(BSP_TIM_Id_t id)
{
    return (id >= 0 && id < BSP_TIM_NUM);
}

static inline BSP_TIM_Ctrl_t *get_ctrl(BSP_TIM_Id_t id)
{
    return &tim_ctrl[id];
}

static BSP_TIM_Ctrl_t *find_ctrl_by_inst(TIM_TypeDef *inst)
{
    if (inst == BSP_TIM_UV_INST) return &tim_ctrl[BSP_TIM_UV];
    return NULL;
}

/* ---------------------------------------------------------------------------*/
/*                     共用函数：启动一轮新测量                                */
/* ---------------------------------------------------------------------------*/

static void start_new_measurement(BSP_TIM_Ctrl_t *ctrl)
{
    TIM_IC_InitTypeDef ic_cfg;

    __HAL_TIM_DISABLE(ctrl->handle);
    __HAL_TIM_SET_COUNTER(ctrl->handle, 0);

    ic_cfg.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
    ic_cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic_cfg.ICPrescaler = TIM_ICPSC_DIV1;
    ic_cfg.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(ctrl->handle, &ic_cfg, TIM_CHANNEL_1);

    ctrl->overflow_cnt = 0;
    __HAL_TIM_CLEAR_FLAG(ctrl->handle, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE(ctrl->handle);

    ctrl->state = CAPTURE_RISING_EDGE;
}

/* ---------------------------------------------------------------------------*/
/*               BSP_TIM_IC_CaptureHandler — 捕获状态机核心                    */
/*               由 main.c 中的 HAL_TIM_IC_CaptureCallback 转发调用            */
/* ---------------------------------------------------------------------------*/

void BSP_TIM_IC_CaptureHandler(TIM_HandleTypeDef *htim)
{
    BSP_TIM_Ctrl_t *ctrl = find_ctrl_by_inst(htim->Instance);
    if (ctrl == NULL) return;

    switch (ctrl->state) {

        /* ================================================================ */
        /*  上升沿（IDLE 或 COMPLETE）→ 开始一轮新测量                       */
        /* ================================================================ */
        case CAPTURE_IDLE:
        case CAPTURE_COMPLETE:
            start_new_measurement(ctrl);
            break;

        /* ================================================================ */
        /*  下降沿 → 计算脉宽，推入环形缓冲，切回上升沿                       */
        /* ================================================================ */
        case CAPTURE_RISING_EDGE: {
            uint32_t arr   = ctrl->handle->Init.Period;
            uint32_t ccr1  = HAL_TIM_ReadCapturedValue(ctrl->handle, TIM_CHANNEL_1);
            uint32_t total = ccr1 + ctrl->overflow_cnt * (arr + 1U);

            /* 组装脉冲数据 */
            BSP_TIM_PulseData_t pulse;
            pulse.pulse_width_us = TICKS_TO_US(total, ctrl->tick_us);
            pulse.timestamp_ms   = HAL_GetTick();

            /* 满足条件后 写入环形缓冲区（满则覆盖，由 ring_buffer_push_overwrite 内部处理） */
            if (pulse.pulse_width_us >= BSP_TIM_UV_PW_MIN_US &&
                 pulse.pulse_width_us <= BSP_TIM_UV_PW_MAX_US)
            {
                ring_buffer_push_overwrite(&ctrl->pulse_rb, &pulse);
            }

            /* 切换回上升沿极性，准备下一轮 */
            {
                TIM_IC_InitTypeDef ic_cfg;
                ic_cfg.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
                ic_cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
                ic_cfg.ICPrescaler = TIM_ICPSC_DIV1;
                ic_cfg.ICFilter    = 0;
                __HAL_TIM_DISABLE(ctrl->handle);
                HAL_TIM_IC_ConfigChannel(ctrl->handle, &ic_cfg, TIM_CHANNEL_1);
                __HAL_TIM_ENABLE(ctrl->handle);
            }

            ctrl->state = CAPTURE_COMPLETE;
            break;
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*               BSP_TIM_IC_PeriodHandler — 溢出计数                           */
/*               由 main.c 中的 HAL_TIM_PeriodElapsedCallback 转发调用         */
/* ---------------------------------------------------------------------------*/

void BSP_TIM_IC_PeriodHandler(TIM_HandleTypeDef *htim)
{
    /* UV 脉冲捕获溢出计数（仅 RISING_EDGE 期间有效） */
    BSP_TIM_Ctrl_t *ctrl = find_ctrl_by_inst(htim->Instance);
    if (ctrl == NULL) return;
    if (ctrl->state == CAPTURE_RISING_EDGE) {
        ctrl->overflow_cnt++;
    }
}

/* ---------------------------------------------------------------------------*/
/*                        公有 API 实现                                        */
/* ---------------------------------------------------------------------------*/

int BSP_TIM_IC_Init(BSP_TIM_Id_t id)
{
    if (!is_valid_id(id)) {
        return BSP_TIM_ERROR;
    }

    if (tim_ctrl[id].initialized) {
        return BSP_TIM_OK;
    }

    BSP_TIM_Ctrl_t *ctrl = &tim_ctrl[id];

    /* 绑定 HAL 句柄 */
    if (id == BSP_TIM_UV) {
        ctrl->handle = &htim3;
    }
    if (ctrl->handle == NULL) {
        return BSP_TIM_ERROR;
    }

    /* 计算 tick_us */
    ctrl->tick_us = (float)(ctrl->handle->Init.Prescaler + 1U)
                  / (float)(BSP_TIM_UV_CLOCK_HZ / 1000000U);

    ctrl->state        = CAPTURE_IDLE;
    ctrl->overflow_cnt = 0;
    ctrl->initialized  = 1;

    /* 初始化环形缓冲区 */
    ring_buffer_init(&ctrl->pulse_rb, ctrl->pulse_pool,
                     sizeof(BSP_TIM_PulseData_t),
                     BSP_TIM_UV_PULSE_POOL_SIZE);
    /* 开启捕获 */
    BSP_TIM_IC_Start(BSP_TIM_UV);

    return BSP_TIM_OK;
}

int BSP_TIM_IC_Start(BSP_TIM_Id_t id)
{
    if (!is_valid_id(id) || !tim_ctrl[id].initialized) {
        return BSP_TIM_ERROR;
    }

    BSP_TIM_Ctrl_t *ctrl = &tim_ctrl[id];

    ctrl->state        = CAPTURE_IDLE;
    ctrl->overflow_cnt = 0;

    /* 清空环形缓冲区（丢弃启动前残留数据） */
    ring_buffer_clear(&ctrl->pulse_rb);

    /* 确保 CH1 上升沿捕获 */
    {
        TIM_IC_InitTypeDef ic_cfg;
        ic_cfg.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
        ic_cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
        ic_cfg.ICPrescaler = TIM_ICPSC_DIV1;
        ic_cfg.ICFilter    = 0;
        HAL_TIM_IC_ConfigChannel(ctrl->handle, &ic_cfg, TIM_CHANNEL_1);
    }

    HAL_TIM_IC_Start_IT(ctrl->handle, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_IT(ctrl->handle, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(ctrl->handle, 0);

    return BSP_TIM_OK;
}

int BSP_TIM_IC_Stop(BSP_TIM_Id_t id)
{
    if (!is_valid_id(id) || !tim_ctrl[id].initialized) {
        return BSP_TIM_ERROR;
    }

    BSP_TIM_Ctrl_t *ctrl = &tim_ctrl[id];

    __HAL_TIM_DISABLE_IT(ctrl->handle, TIM_IT_UPDATE);
    HAL_TIM_IC_Stop_IT(ctrl->handle, TIM_CHANNEL_1);
    ctrl->state = CAPTURE_IDLE;

    return BSP_TIM_OK;
}

int BSP_TIM_IC_ReadPulse(BSP_TIM_Id_t id, BSP_TIM_PulseData_t *pulse)
{
    if (!is_valid_id(id) || !tim_ctrl[id].initialized || pulse == NULL) {
        return BSP_TIM_ERROR;
    }

    /* 从环形缓冲区弹出一条脉冲数据 */
    return ring_buffer_pop(&tim_ctrl[id].pulse_rb, pulse);
}

uint16_t BSP_TIM_IC_ReadAllPulse(BSP_TIM_Id_t id, BSP_TIM_PulseData_t *pulses, uint16_t max)
{
    if (!is_valid_id(id) || !tim_ctrl[id].initialized || pulses == NULL || max == 0) {
        return 0;
    }

    uint16_t cnt = 0;
    while (cnt < max && ring_buffer_pop(&tim_ctrl[id].pulse_rb, &pulses[cnt]) == 0) {
        cnt++;
    }
    return cnt;
}
/*
BSP_TIM_CaptureState_t BSP_TIM_IC_GetState(BSP_TIM_Id_t id)
{
    if (!is_valid_id(id)) return CAPTURE_IDLE;
    return tim_ctrl[id].state;
}

float BSP_TIM_IC_GetTickUs(BSP_TIM_Id_t id)
{
    if (!is_valid_id(id) || !tim_ctrl[id].initialized) return 0.0f;
    return tim_ctrl[id].tick_us;
}

void *BSP_TIM_IC_GetHandle(BSP_TIM_Id_t id)
{
    if (!is_valid_id(id) || !tim_ctrl[id].initialized) return NULL;
    return (void *)tim_ctrl[id].handle;
}*/
