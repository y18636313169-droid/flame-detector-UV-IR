/**
  ******************************************************************************
  * @file    ap_uv.c
  * @brief   AP 层紫外火焰检测模块实现
  *
  *          三态状态机：IDLE → WARNING → FIRE
  *            IDLE:    窗口计数 ≥ threshold → WARNING
  *            WARNING: 80%迟滞下限以上累计确认；低于下限持续2秒 → IDLE
  *            FIRE:    保持30分钟后自动消警回 IDLE
  *
  *          实例在文件内部静态定义，外部无传参。
  ******************************************************************************
  */

#include "ap_uv.h"
#include "ap_eeprom.h"
#include "ap_util.h"
#include "hal_tim.h"
#include "hal_uart.h"
#include "ap_ir.h"
#include <string.h>

/* ========================================================================== */
/*                          调试打印开关                                       */
/* ========================================================================== */
#if defined(AP_ALGO_DEBUG_ENABLE)
#define DBG(fmt, ...)   BSP_UART_Printf("[UV] " fmt "\r\n", ##__VA_ARGS__)
#define UV_DEBUG_LOG_INTERVAL_MS  (500U)
#else
#define DBG(fmt, ...)
#endif

/* ========================================================================== */
/*                        内部数据结构                                         */
/* ========================================================================== */

#define MAX_HISTORY         UV_MAX_HISTORY
/* 分批搬运降低主循环单次栈占用；未读脉冲继续保留在底层128槽缓冲中。 */
#define UV_FEED_BATCH_SIZE  (64U)

typedef struct {
    uint32_t    timestamp_ms;
    uint16_t    width_us;
} UV_Pulse_t;

typedef struct {
    UV_Pulse_t  history[MAX_HISTORY];
    uint8_t     head;       /* 下一写入位置（环形写指针） */
    uint8_t     count;      /* 有效元素个数 */

    uint8_t     level;              /* 当前等级 0~9 */
    uint32_t    window_ms;          /* 当前实际检测窗口 n ms内满足n thres个脉冲 */
    uint8_t     threshold;          /* 脉冲判断阈值数 */
    uint32_t    confirm_ms;         /* 确认时长 */
    uint32_t    fire_timeout_ms;   /* 固定FIRE保持时间，不参与等级插值 */

    /* min/max 配置（等级0使用min，等级9使用max）*/
    uint32_t    thr_min, thr_max;
    uint32_t    win_min, win_max;
    uint32_t    cfm_min, cfm_max;

    UV_DetectorState_t state;
    uint32_t    warning_update_ms;   /* 有效积分按实际毫秒更新，避免主循环频率影响 */
    uint32_t    valid_accum_ms;
    uint32_t    drop_start_ms;
    uint8_t     drop_active;
    uint32_t    fire_start_ms;     /* 进入FIRE时记录的绝对超时起点 */

    uint32_t    print_window_ms;    /* 测试打印窗口(ms) EEPROM 存储 */
    void        (*on_fire)(void);
} UV_FlameDetector_t;

static UV_FlameDetector_t uv_det;

static uint8_t uv_drop_threshold(void)
{
    /* 向上取整，保证整数脉冲下限不会低于当前阈值的80%。 */
    uint32_t threshold = ((uint32_t)uv_det.threshold * UV_DROP_OFF_PERCENT + 99U) / 100U;
    return (threshold == 0U) ? 1U : (uint8_t)threshold;
}

static void uv_warning_reset(void)
{
    uv_det.state = UV_STATE_IDLE;
    uv_det.warning_update_ms = 0U;
    uv_det.valid_accum_ms = 0U;
    uv_det.drop_start_ms = 0U;
    uv_det.drop_active = 0U;
}

#if defined(AP_ALGO_DEBUG_ENABLE)
/* 汇总高脉冲率下的覆盖情况，避免逐脉冲日志阻塞主循环。 */
static uint32_t s_debug_last_snapshot_ms;
static uint32_t s_debug_overwrite_count;
#endif

/* ========================================================================== */
/*                        内部辅助 — 状态名                                    */
/* ========================================================================== */

#if defined(AP_ALGO_DEBUG_ENABLE)
static const char *uv_state_name(UV_DetectorState_t s)
{
    switch (s) {
        case UV_STATE_IDLE:    return "IDLE";
        case UV_STATE_WARNING: return "WARNING";
        case UV_STATE_FIRE:    return "FIRE";
        default:               return "?";
    }
}

static void debug_log_snapshot(uint32_t now, uint8_t pulse_cnt)
{
    if ((now - s_debug_last_snapshot_ms) < UV_DEBUG_LOG_INTERVAL_MS) return;
    s_debug_last_snapshot_ms = now;

    DBG("T%lu S=%s CNT=%u/%u OFF=%u WIN=%lu ACC=%lu/%lu DROP=%u OVR=%lu",
        (unsigned long)now, uv_state_name(uv_det.state),
        (unsigned int)pulse_cnt, (unsigned int)uv_det.threshold,
        (unsigned int)uv_drop_threshold(),
        (unsigned long)uv_det.window_ms,
        (unsigned long)uv_det.valid_accum_ms,
        (unsigned long)uv_det.confirm_ms,
        (unsigned int)uv_det.drop_active,
        (unsigned long)s_debug_overwrite_count);
    s_debug_overwrite_count = 0U;
}
#endif

/* ========================================================================== */
/*                        公有 API 实现                                        */
/* ========================================================================== */

void AP_UV_Init(void (*on_fire)(void))
{
    memset(&uv_det, 0, sizeof(uv_det));
#if defined(AP_ALGO_DEBUG_ENABLE)
    s_debug_last_snapshot_ms = 0U;
    s_debug_overwrite_count = 0U;
#endif

    uv_det.state   = UV_STATE_IDLE;
    uv_det.on_fire = on_fire;

    /* 从 EEPROM 加载参数并自动计算 */
    const AP_EEPROM_UV_Param_t *p = AP_EEPROM_UV_Get();
    uv_det.level = (p->sensitivity < UV_SENS_LEVELS) ? p->sensitivity : (UV_SENS_LEVELS - 1);
    AP_UV_SetConfig(p->thr_min, p->thr_max,
                     p->win_min, p->win_max,
                     p->cfm_min, p->cfm_max);
    AP_UV_SetLevel(uv_det.level);

    /* 将 EEPROM 中配置的脉宽阈值写入 BSP */
    BSP_TIM_IC_SetPulseRange((uint16_t)p->pw_min_us, (uint16_t)p->pw_max_us);
    uv_det.print_window_ms = p->print_window_ms;
    DBG("Init level=%u pulse=%lu..%luus print_win=%lums",
        (unsigned int)uv_det.level,
        (unsigned long)p->pw_min_us, (unsigned long)p->pw_max_us,
        (unsigned long)uv_det.print_window_ms);
}

void AP_UV_Feed(void)
{
    BSP_TIM_PulseData_t pulses[UV_FEED_BATCH_SIZE];
    uint16_t n = BSP_TIM_IC_ReadAllPulse(BSP_TIM_UV, pulses, UV_FEED_BATCH_SIZE);

    for (uint16_t i = 0; i < n; i++) {
        if (uv_det.count >= MAX_HISTORY) {
            uv_det.head = (uv_det.head + 1) % MAX_HISTORY;
            uv_det.count--;
#if defined(AP_ALGO_DEBUG_ENABLE)
            s_debug_overwrite_count++;
#endif
        }

        uint8_t wi = (uv_det.head + uv_det.count) % MAX_HISTORY;
        uv_det.history[wi].timestamp_ms = pulses[i].timestamp_ms;
        uv_det.history[wi].width_us     = (uint16_t)pulses[i].pulse_width_us;
        uv_det.count++;
    }
}

void AP_UV_Process(uint32_t now)
{
    /* ---- 移除检测窗口外的旧脉冲 ---- */
    while (uv_det.count > 0U) {
        UV_Pulse_t *oldest = &uv_det.history[uv_det.head];
        if ((now - oldest->timestamp_ms) <= uv_det.window_ms) break;    /* 最旧数据已在窗口内 提前break跳出循环 */
        uv_det.head = (uv_det.head + 1U) % MAX_HISTORY;
        uv_det.count--;
    }
    uint8_t pulse_cnt = uv_det.count;

#if defined(AP_ALGO_DEBUG_ENABLE)
    debug_log_snapshot(now, pulse_cnt);
#endif

    /* ---- 状态机 ---- */
    {
        UV_DetectorState_t old = uv_det.state;

        switch (old) {

            case UV_STATE_IDLE:
                if (pulse_cnt >= uv_det.threshold) {    /* 脉冲数满足入场阈值 直接进入warning态进行确认 */
                    uv_det.state = UV_STATE_WARNING;
                    uv_det.warning_update_ms = now;
                    uv_det.valid_accum_ms = 0U;
                    uv_det.drop_active = 0U;
                }
                break;

            case UV_STATE_WARNING: {
                uint32_t elapsed_ms = now - uv_det.warning_update_ms;   /* 获取距离上一次判断的时间 由于主循环执行延时 非固定10ms 阈值满足 + 不满足 -*/
                uint8_t drop_threshold = uv_drop_threshold();   /* 获取掉线阈值 */ 
                uv_det.warning_update_ms = now;

                if (pulse_cnt >= drop_threshold) {  /* 满足阈值条件 */
                    if (uv_det.drop_active != 0U) {     /* 掉线后恢复 */
                        DBG("T%lu DROP recovered CNT=%u OFF=%u ACC=%lu",
                            (unsigned long)now, (unsigned int)pulse_cnt,
                            (unsigned int)drop_threshold,
                            (unsigned long)uv_det.valid_accum_ms);
                    }
                    uv_det.drop_active = 0U;
                    uv_det.drop_start_ms = 0U;
                    /* 达到80%迟滞下限即视为证据持续，与IR的迟滞保持逻辑一致。 */
                    if (uv_det.valid_accum_ms < uv_det.confirm_ms) {    /* 不满足确认时长 增加运行时间 */
                        uint32_t remain = uv_det.confirm_ms - uv_det.valid_accum_ms;
                        uv_det.valid_accum_ms += (elapsed_ms < remain) ? elapsed_ms : remain;
                    }
                    if (uv_det.valid_accum_ms >= uv_det.confirm_ms) {   /* 满足确认时间 报警 */
                        uv_det.state = UV_STATE_FIRE;
                        uv_det.fire_start_ms = now;
                        uv_det.drop_active = 0U;
                        if (uv_det.on_fire != NULL) {
                            uv_det.on_fire();   /* 执行紫外着火cb函数 */
                        }
                    }
                } else {    /* 不满足阈值条件 */
                    /* 掉线期间按实际时间回退积分，连续2秒不足退出WARNING。 */
                    uv_det.valid_accum_ms = (uv_det.valid_accum_ms > elapsed_ms)
                                              ? (uv_det.valid_accum_ms - elapsed_ms) : 0U;
                    if (uv_det.drop_active == 0U) { /* 初次掉线 */
                        uv_det.drop_active = 1U;    
                        uv_det.drop_start_ms = now;
                        DBG("T%lu DROP start CNT=%u < OFF=%u ACC=%lu",
                            (unsigned long)now, (unsigned int)pulse_cnt,
                            (unsigned int)drop_threshold,
                            (unsigned long)uv_det.valid_accum_ms);
                    } else if ((now - uv_det.drop_start_ms) >= UV_DROPOUT_MS) { /* 超出允许的掉线时长，重回IDLE */
                        DBG("T%lu DROP timeout CNT=%u elapsed=%lu -> IDLE",
                            (unsigned long)now, (unsigned int)pulse_cnt,
                            (unsigned long)(now - uv_det.drop_start_ms));
                        uv_warning_reset();
                    }
                }
                break;
            }

            case UV_STATE_FIRE:
                /* FIRE期间暂定不根据脉冲消失清除，只允许固定超时消警。 */
                if ((now - uv_det.fire_start_ms) >= uv_det.fire_timeout_ms && AP_IR_GetState() == IR_STATE_IDLE) {
                    DBG("T%lu FIRE timeout elapsed=%lu -> IDLE",
                        (unsigned long)now,
                        (unsigned long)(now - uv_det.fire_start_ms));
                    /* FIRE超时回到IDLE时同步清除WARNING遗留的积分和掉线状态。 */
                    uv_warning_reset();
                    uv_det.fire_start_ms = 0U;
                }
                break;

            default:
                break;
        }

        if (uv_det.state != old) {
            DBG("T%lu STATE %s -> %s CNT=%u/%u",
                (unsigned long)now,
                uv_state_name(old), uv_state_name(uv_det.state),
                (unsigned int)pulse_cnt, (unsigned int)uv_det.threshold);
        }
    }
}

void AP_UV_Task(void)
{
    AP_UV_Feed();                   /* ---- 从 BSP 读取新脉冲 ---- */
    AP_UV_Process(HAL_GetTick());   /* ---- 流程检测 ---- */
}

/* ========================================================================== */

void AP_UV_SetConfig(uint32_t thr_min, uint32_t thr_max,
                     uint32_t win_min, uint32_t win_max,
                     uint32_t cfm_min, uint32_t cfm_max)
{
    uv_det.thr_min = thr_min;
    uv_det.thr_max = thr_max;
    uv_det.win_min = win_min;
    uv_det.win_max = win_max;
    uv_det.cfm_min = cfm_min;
    uv_det.cfm_max = cfm_max;
}

void AP_UV_GetConfig(uint32_t *thr_min, uint32_t *thr_max,
                     uint32_t *win_min, uint32_t *win_max,
                     uint32_t *cfm_min, uint32_t *cfm_max)
{
    if (thr_min) *thr_min = uv_det.thr_min;
    if (thr_max) *thr_max = uv_det.thr_max;
    if (win_min) *win_min = uv_det.win_min;
    if (win_max) *win_max = uv_det.win_max;
    if (cfm_min) *cfm_min = uv_det.cfm_min;
    if (cfm_max) *cfm_max = uv_det.cfm_max;
}

void AP_UV_SetLevel(uint8_t level)
{
    uv_det.level = (level >= UV_SENS_LEVELS) ? (UV_SENS_LEVELS - 1) : level;
    uint32_t lv = uv_det.level;
    uint32_t threshold = lerp_u32(uv_det.thr_min, uv_det.thr_max, lv, UV_SENS_LEVELS);

    if (threshold > MAX_HISTORY) threshold = MAX_HISTORY;
    if (threshold == 0U) threshold = 1U;
    uv_det.threshold  = (uint8_t)threshold;
    uv_det.window_ms  = lerp_u32(uv_det.win_min, uv_det.win_max, lv, UV_SENS_LEVELS);
    uv_det.confirm_ms = lerp_u32(uv_det.cfm_min, uv_det.cfm_max, lv, UV_SENS_LEVELS);
    /* 火警保持时间由IR/UV共用宏统一管理，避免灵敏度改变锁存时长。 */
    uv_det.fire_timeout_ms = AP_FIRE_AUTO_CLEAR_MS;

    DBG("SetLevel %u: thr=%u off=%u win=%lu cfm=%lu fire_timeout=%lu",
        uv_det.level, uv_det.threshold, uv_drop_threshold(), uv_det.window_ms,
        uv_det.confirm_ms, uv_det.fire_timeout_ms);
}

void AP_UV_Process_Reset(void)
{
    uv_det.state = UV_STATE_IDLE;
    uv_det.warning_update_ms = 0U;
    uv_det.valid_accum_ms = 0U;
    uv_det.drop_start_ms = 0U;
    uv_det.drop_active = 0U;
    uv_det.fire_start_ms = 0;
    uv_det.head = 0;
    uv_det.count = 0;
#if defined(AP_ALGO_DEBUG_ENABLE)
    s_debug_last_snapshot_ms = 0U;
    s_debug_overwrite_count = 0U;
#endif

    /* 清空 TIM3 脉冲环缓冲，防止旧脉冲影响新判定 */
    BSP_TIM_IC_ClearAllPulse(BSP_TIM_UV);
    DBG("Reset -> IDLE, pulse history cleared");
}

UV_DetectorState_t AP_UV_GetState(void)
{
    return uv_det.state;
}

void AP_UV_SetPrintWindow(uint32_t ms)
{
    if (ms < 100) ms = 100;  /* 最小 100ms */
    uv_det.print_window_ms = ms;
}

uint32_t AP_UV_GetPrintWindow(void)
{
    return uv_det.print_window_ms;
}

void AP_UV_GetParams(uint32_t *threshold, uint32_t *window_ms,
                     uint32_t *confirm_ms, uint32_t *fire_timeout_ms)
{
    if (threshold)  *threshold  = uv_det.threshold;
    if (window_ms)  *window_ms  = uv_det.window_ms;
    if (confirm_ms) *confirm_ms = uv_det.confirm_ms;
    if (fire_timeout_ms) *fire_timeout_ms = uv_det.fire_timeout_ms;
}

#if defined(IR_TEST_MODE)
void AP_UV_PrintData(uint32_t now)
{
    BSP_TIM_PulseData_t pulses[128];
    uint16_t n = BSP_TIM_IC_ReadWindow(BSP_TIM_UV, now,
                                        uv_det.print_window_ms, pulses, 128);
    if (n > 0) {
        BSP_UART_Printf("T%lu UV %u", (unsigned long)now, (unsigned)n);
        // for (uint16_t i = 0; i < n; i++) {
        //     BSP_UART_Printf(" %u", pulses[i].pulse_width_us);
        // }
        BSP_UART_Printf("\r\n");
    }
}
#endif /* IR_TEST_MODE */
