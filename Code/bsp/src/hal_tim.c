/**
  ******************************************************************************
  * @file    hal_tim.c
  * @brief   BSP 定时器抽象层实现 — 双通道独立捕获 + 环形缓冲输出
  *
  *          使用 TIM3 双通道捕获 C10807 UV TRON 不规则脉冲：
  *            - CH1 (PA6, 上升沿): CNT 软件清零，开始计时
  *            - CH2 (PA6, 下降沿): 直接读 CCR2（从 0 开始计数，即为脉宽）
  *
  *          == 测量原理 ==
  *          C10807 UV 输出是不规则脉冲，无固定周期。
  *          定时器 1µs/tick，CNT 自由运行，ARR=65535。
  *
  *          1. 上升沿 ISR：手动清零 CNT + 清零 overflow_cnt，记录时间戳
  *          2. 下降沿 ISR：CCR2 即从上升沿开始的计数值（CNT 从 0 开始）
  *                        + overflow_cnt × (ARR+1) 展开超出 65ms 的长脉冲
  *          3. 有效脉宽（6~14ms）写入环形缓冲
  *
  *          == ISR 调用链 ==
  *          TIM3_IRQHandler → HAL_TIM_IRQHandler
  *            ├── HAL_TIM_IC_CaptureCallback(htim)
  *            │     ├── CH1 → 清零 CNT + overflow_cnt，存时间戳
  *            │     └── CH2 → 读 CCR2 + overflow_cnt 展开 → 入环形缓冲
  *            └── HAL_TIM_PeriodElapsedCallback(htim)
  *                  └── overflow_cnt++（脉冲期间 CNT 溢出时增加）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hal_tim.h"
#include "tim.h"            /* 引用 htim3 */

/* ---------------------------------------------------------------------------*/
/*                     私有宏                                                  */
/* ---------------------------------------------------------------------------*/

/** @brief  从计数值换算为微秒（此处 tick=1µs，但保留宏便于未来调整） */
#define TICKS_TO_US(ticks, tick_us)  ((uint32_t)((float)(ticks) * (tick_us)))

/* ---------------------------------------------------------------------------*/
/*                     控制块                                                  */
/* ---------------------------------------------------------------------------*/

typedef struct {
    TIM_HandleTypeDef               *handle;        /* HAL 句柄          */
    volatile uint32_t               overflow_cnt;   /* 溢出计数（上升沿清零）*/
    float                           tick_us;        /* 每 tick 微秒数    */
    uint8_t                         initialized;

    /* 上升沿暂存 */
    volatile uint32_t               rising_tick_ms; /* 上升沿的 HAL_GetTick() */

    /* === 环形缓冲区：ISR 写入（CH2 下降沿），主循环读取 === */
    ring_buffer_t                   pulse_rb;
    BSP_TIM_PulseData_t             pulse_pool[BSP_TIM_UV_PULSE_POOL_SIZE];
} BSP_TIM_Ctrl_t;

/* Private variables ---------------------------------------------------------*/

static BSP_TIM_Ctrl_t tim_ctrl[BSP_TIM_NUM] = {
    [BSP_TIM_UV] = { .handle = NULL }
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
/*               BSP_TIM_IC_CaptureHandler — 双通道独立捕获核心                */
/*               由 main.c 中的 HAL_TIM_IC_CaptureCallback 转发调用            */
/* ---------------------------------------------------------------------------*/

void BSP_TIM_IC_CaptureHandler(TIM_HandleTypeDef *htim)
{
    BSP_TIM_Ctrl_t *ctrl = find_ctrl_by_inst(htim->Instance);
    if (ctrl == NULL) return;

    /* ================================================================ */
    /*  上升沿捕获（CH1）— 软件清零 CNT + overflow_cnt，重新开始计时    */
    /* ================================================================ */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        ctrl->overflow_cnt    = 0;
        ctrl->rising_tick_ms  = HAL_GetTick();
        __HAL_TIM_SET_COUNTER(ctrl->handle, 0);
        __HAL_TIM_CLEAR_FLAG(ctrl->handle, TIM_FLAG_UPDATE);
    }
    /* ================================================================ */
    /*  下降沿捕获（CH2）— CCR2 即脉宽（CNT 从 0 开始计数）            */
    /* ================================================================ */
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        uint32_t ccr2    = HAL_TIM_ReadCapturedValue(ctrl->handle, TIM_CHANNEL_2);
        uint32_t arr     = ctrl->handle->Init.Period;
        uint32_t total   = ccr2 + ctrl->overflow_cnt * (arr + 1U);

        BSP_TIM_PulseData_t pulse;
        pulse.pulse_width_us = TICKS_TO_US(total, ctrl->tick_us);
        pulse.timestamp_ms   = ctrl->rising_tick_ms;   /* 以上升沿时间为准 */

        /* 有效脉宽判定后入环形缓冲 */
        if (pulse.pulse_width_us >= BSP_TIM_UV_PW_MIN_US &&
            pulse.pulse_width_us <= BSP_TIM_UV_PW_MAX_US) {
            ring_buffer_push_overwrite(&ctrl->pulse_rb, &pulse);
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*               BSP_TIM_IC_PeriodHandler — 溢出计数（仅扩展用途）             */
/*               由 main.c 中的 HAL_TIM_PeriodElapsedCallback 转发调用         */
/* ---------------------------------------------------------------------------*/

void BSP_TIM_IC_PeriodHandler(TIM_HandleTypeDef *htim)
{
    BSP_TIM_Ctrl_t *ctrl = find_ctrl_by_inst(htim->Instance);
    if (ctrl == NULL) return;
    ctrl->overflow_cnt++;
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

    /* 计算 tick_us：32MHz / 32 = 1MHz → 1 µs/tick */
    ctrl->tick_us = (float)(ctrl->handle->Init.Prescaler + 1U)
                  / (float)(BSP_TIM_UV_CLOCK_HZ / 1000000U);

    ctrl->overflow_cnt    = 0;
    ctrl->rising_tick_ms  = 0;
    ctrl->initialized     = 1;

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

    ctrl->overflow_cnt    = 0;
    ctrl->rising_tick_ms  = 0;

    /* 清空环形缓冲区（丢弃启动前残留数据） */
    ring_buffer_clear(&ctrl->pulse_rb);

    /* 启动 CH1 上升沿捕获 */
    HAL_TIM_IC_Start_IT(ctrl->handle, TIM_CHANNEL_1);
    /* 启动 CH2 下降沿捕获 */
    HAL_TIM_IC_Start_IT(ctrl->handle, TIM_CHANNEL_2);
    /* 使能更新中断（溢出计数） */
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
    HAL_TIM_IC_Stop_IT(ctrl->handle, TIM_CHANNEL_2);

    return BSP_TIM_OK;
}

int BSP_TIM_IC_ReadPulse(BSP_TIM_Id_t id, BSP_TIM_PulseData_t *pulse)
{
    if (!is_valid_id(id) || !tim_ctrl[id].initialized || pulse == NULL) {
        return BSP_TIM_ERROR;
    }

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

/* ========================================================================== */
/*         旧单通道极性翻转捕获实现 — 保留注释以供后续参考                     */
/* ========================================================================== */
#if 0
/**
  * @brief  单通道极性切换脉宽捕获（已废弃，由双通道独立捕获替代）
  *
  *          原方案使用 CH1 单通道，软件在 ISR 中切换捕获极性：
  *            ISR 上升沿 → 切为下降沿，开始计时
  *            ISR 下降沿 → 切为上升沿，计算脉宽，入环
  *
  *          缺点：上升沿到下降沿之间的重配置存在时钟级延迟。已由双通道取代。
  */
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

void BSP_TIM_IC_CaptureHandler(TIM_HandleTypeDef *htim)
{
    BSP_TIM_Ctrl_t *ctrl = find_ctrl_by_inst(htim->Instance);
    if (ctrl == NULL) return;

    switch (ctrl->state) {
        case CAPTURE_IDLE:
        case CAPTURE_COMPLETE:
            start_new_measurement(ctrl);
            break;
        case CAPTURE_RISING_EDGE: {
            uint32_t arr   = ctrl->handle->Init.Period;
            uint32_t ccr1  = HAL_TIM_ReadCapturedValue(ctrl->handle, TIM_CHANNEL_1);
            uint32_t total = ccr1 + ctrl->overflow_cnt * (arr + 1U);
            BSP_TIM_PulseData_t pulse;
            pulse.pulse_width_us = TICKS_TO_US(total, ctrl->tick_us);
            pulse.timestamp_ms   = HAL_GetTick();
            if (pulse.pulse_width_us >= BSP_TIM_UV_PW_MIN_US &&
                 pulse.pulse_width_us <= BSP_TIM_UV_PW_MAX_US) {
                ring_buffer_push_overwrite(&ctrl->pulse_rb, &pulse);
            }
            TIM_IC_InitTypeDef ic_cfg;
            ic_cfg.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
            ic_cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
            ic_cfg.ICPrescaler = TIM_ICPSC_DIV1;
            ic_cfg.ICFilter    = 0;
            __HAL_TIM_DISABLE(ctrl->handle);
            HAL_TIM_IC_ConfigChannel(ctrl->handle, &ic_cfg, TIM_CHANNEL_1);
            __HAL_TIM_ENABLE(ctrl->handle);
            ctrl->state = CAPTURE_COMPLETE;
            break;
        }
    }
}
*/
#endif /* #if 0 */
