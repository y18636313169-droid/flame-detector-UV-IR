/**
  ******************************************************************************
  * @file    ap_uv.c
  * @brief   AP 层紫外火焰检测模块实现
  *
  *          三态状态机：IDLE → WARNING → FIRE
  *            IDLE:    窗口计数 ≥ threshold → WARNING
  *            WARNING: 持续满足且 ≥ confirm_ms → FIRE / 掉线 → IDLE
  *            FIRE:    窗口计数=0 持续超过 clear_ms → 自动消警回 IDLE
  *
  *          实例在文件内部静态定义，外部无传参。
  ******************************************************************************
  */

#include "ap_uv.h"
#include "hal_tim.h"
#include "hal_uart.h"
#include <string.h>

/* ========================================================================== */
/*                          调试打印开关                                       */
/* ========================================================================== */
#define AP_UV_DEBUG_ENABLE

#ifdef AP_UV_DEBUG_ENABLE
#define DBG(fmt, ...)   BSP_UART_Printf("[UV] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

/* ========================================================================== */
/*                        内部数据结构                                         */
/* ========================================================================== */

#define MAX_HISTORY     (20U)

typedef struct {
    uint32_t    timestamp_ms;
    uint16_t    width_us;
} UV_Pulse_t;

typedef struct {
    UV_Pulse_t  history[MAX_HISTORY];
    uint8_t     head;
    uint8_t     count;

    uint8_t     level;
    uint32_t    window_ms;
    uint8_t     threshold;
    uint32_t    confirm_ms;
    uint32_t    clear_ms;

    UV_DetectorState_t state;
    uint32_t    warning_start_ms;
    uint32_t    fire_start_ms;

    void        (*on_fire)(void);
} UV_FlameDetector_t;

static UV_FlameDetector_t uv_det;

/* ========================================================================== */
/*                        内部辅助 — 参数计算                                 */
/* ========================================================================== */

static void uv_calc_params(void)
{
    uint8_t lv = uv_det.level;
    if (lv >= UV_SENS_LEVELS) lv = UV_SENS_LEVELS - 1;

    uv_det.window_ms  = UV_WINDOW_MS_0
                      - (UV_WINDOW_MS_0 - UV_WINDOW_MS_9) * lv / (UV_SENS_LEVELS - 1);

    uv_det.threshold  = UV_THRESHOLD_0
                      - (UV_THRESHOLD_0 - UV_THRESHOLD_9) * lv / (UV_SENS_LEVELS - 1);

    uv_det.confirm_ms = UV_CONFIRM_MS_0
                      - (UV_CONFIRM_MS_0 - UV_CONFIRM_MS_9) * lv / (UV_SENS_LEVELS - 1);

    uv_det.clear_ms   = UV_CLEAR_MS_0
                      - (UV_CLEAR_MS_0 - UV_CLEAR_MS_9) * lv / (UV_SENS_LEVELS - 1);
}

/* ========================================================================== */
/*                        内部辅助 — 状态名                                    */
/* ========================================================================== */

static const char *uv_state_name(UV_DetectorState_t s)
{
    switch (s) {
        case UV_STATE_IDLE:    return "IDLE";
        case UV_STATE_WARNING: return "WARNING";
        case UV_STATE_FIRE:    return "FIRE";
        default:               return "?";
    }
}

/* ========================================================================== */
/*                        公有 API 实现                                        */
/* ========================================================================== */

void AP_UV_Init(uint8_t level, void (*on_fire)(void))
{
    memset(&uv_det, 0, sizeof(uv_det));

    uv_det.level   = (level >= UV_SENS_LEVELS) ? (UV_SENS_LEVELS - 1) : level;
    uv_det.state   = UV_STATE_IDLE;
    uv_det.on_fire = on_fire;

    uv_calc_params();
}

void AP_UV_Feed(void)
{
    BSP_TIM_PulseData_t pulses[MAX_HISTORY];
    uint16_t n = BSP_TIM_IC_ReadAllPulse(BSP_TIM_UV, pulses, MAX_HISTORY);

    if (n > 0) {
        DBG("Feed %u pulse(s)", n);
        for (uint16_t i = 0; i < n; i++) {
            DBG("  #%u width=%uus t=%ums", i,
                pulses[i].pulse_width_us, pulses[i].timestamp_ms);
        }
    }

    for (uint16_t i = 0; i < n; i++) {
        if (uv_det.count >= MAX_HISTORY) {
            uv_det.head = (uv_det.head + 1) % MAX_HISTORY;
            uv_det.count--;
        }

        uint8_t wi = (uv_det.head + uv_det.count) % MAX_HISTORY;
        uv_det.history[wi].timestamp_ms = pulses[i].timestamp_ms;
        uv_det.history[wi].width_us     = (uint16_t)pulses[i].pulse_width_us;
        uv_det.count++;
    }
}

void AP_UV_Process(uint32_t now)
{
    /* ---- 从 BSP 读取新脉冲 ---- */
    AP_UV_Feed();

    /* ---- 统计窗口内脉冲数 ---- */
    uint8_t pulse_cnt = 0;
    for (uint8_t i = 0; i < uv_det.count; i++) {
        uint32_t age = now - uv_det.history[i].timestamp_ms;
        if (age <= uv_det.window_ms) {
            pulse_cnt++;
        }
    }

    /* ---- 状态机 ---- */
    {
        UV_DetectorState_t old = uv_det.state;

        switch (old) {

            case UV_STATE_IDLE:
                if (pulse_cnt >= uv_det.threshold) {
                    uv_det.state = UV_STATE_WARNING;
                    uv_det.warning_start_ms = now;
                }
                break;

            case UV_STATE_WARNING:
                if (pulse_cnt >= uv_det.threshold) {
                    if ((now - uv_det.warning_start_ms) >= uv_det.confirm_ms) {
                        uv_det.state = UV_STATE_FIRE;
                        uv_det.fire_start_ms = now;
                        if (uv_det.on_fire != NULL) {
                            uv_det.on_fire();
                        }
                    }
                } else {
                    uv_det.state = UV_STATE_IDLE;
                }
                break;

            case UV_STATE_FIRE:
                if (pulse_cnt == 0 && (now - uv_det.fire_start_ms) >= uv_det.clear_ms) {
                    uv_det.state = UV_STATE_IDLE;
                }
                break;

            default:
                break;
        }

        if (uv_det.state != old) {
            DBG("%s -> %s  pulse=%u/%u  t=%ums",
                uv_state_name(old), uv_state_name(uv_det.state),
                pulse_cnt, uv_det.threshold, now);
        }
    }
}

void AP_UV_SetLevel(uint8_t level)
{
    uv_det.level = (level >= UV_SENS_LEVELS) ? (UV_SENS_LEVELS - 1) : level;
    uv_calc_params();
    DBG("SetLevel %u: win=%ums thr=%u confirm=%ums clear=%ums",
        uv_det.level, uv_det.window_ms, uv_det.threshold,
        uv_det.confirm_ms, uv_det.clear_ms);
}

void AP_UV_Reset(void)
{
    uv_det.state = UV_STATE_IDLE;
    uv_det.warning_start_ms = 0;
}

UV_DetectorState_t AP_UV_GetState(void)
{
    return uv_det.state;
}
