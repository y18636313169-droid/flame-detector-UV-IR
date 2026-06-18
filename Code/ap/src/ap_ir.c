/**
  ******************************************************************************
  * @file    ap_ir.c
  * @brief   AP 层红外三波段多光谱融合火焰检测模块实现
  *
  *          算法流程:
  *            AP_IR_Task() → 检查标志 → Feed(推ADC) → Process(检测)
  *
  *          Process 内部:
  *            1. 线性化 50 点历史窗口 → int32_t 工作缓冲
  *            2. 去直流(减均值)
  *            3. 40Hz 二阶 IIR 低通滤波 (Butterworth)
  *            4. 计算平均功率(均方值 ×1000) → uint32_t
  *            5. 计算过零率 → float Hz
  *            6. 计算光谱比 R4.5/3.8 和 R4.5/5.0 (×1000)
  *            7. 五判据串联决策
  *            8. 状态机迁移 (IDLE↔WARNING→FIRE)
  *
  *          参数管理(同步 UV 模式):
  *            4 项参数以 min/max 范围 + 灵敏度 0~9 线性插值:
  *              pwr(功率), r38/r50(光谱比), cfm(确认时长)
  *            2 项频率参数固定值(不随灵敏度改变):
  *              freq_low(下限), freq_high(上限)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ap_ir.h"
#include "ap_adc.h"
#include "ap_eeprom.h"
#include "ap_util.h"
#include "hal_uart.h"
#include <string.h>
#include <math.h>

/* ========================================================================== */
/*                          调试打印开关                                       */
/* ========================================================================== */
#define AP_IR_DEBUG_ENABLE

#ifdef AP_IR_DEBUG_ENABLE
#define DBG(fmt, ...)   BSP_UART_Printf("[IR] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

/* ========================================================================== */
/*                      IIR 低通滤波器系数 (Q15 定点)                          */
/*         二阶 Butterworth Lowpass, fc=40Hz, fs=100Hz                        */
/*         直接 I 型: y = (b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2) >> 15       */
/*         系数缩放: Q15 = round(coeff * 32768)                               */
/* ========================================================================== */

#define IIR_B0_Q15  6770    /*  0.2066 * 32768 */
#define IIR_B1_Q15  13543   /*  0.4133 * 32768 */
#define IIR_B2_Q15  6770    /*  0.2066 * 32768 */
#define IIR_A1_Q15  (-12104) /* -0.3695 * 32768 (差方程中用 -a1*y1, 直接存 a1) */
#define IIR_A2_Q15  6416    /*  0.1958 * 32768 */

/* ========================================================================== */
/*                         内部数据结构                                        */
/* ========================================================================== */

/**
  @brief  单通道历史窗口 (环形缓冲)
          存储原始 ADC 值(0~4095), head 指向下一个写入位置
*/
typedef struct {
    uint16_t    raw[IR_HISTORY_SIZE];
    uint8_t     head;
    uint8_t     count;
} IR_History_t;

/**
  @brief  单通道特征提取结果
*/
typedef struct {
    uint32_t    power_x1000;        /* 平均功率(均方值 ×1000) */
    float       zcr_hz;             /* 过零率(Hz) */
} IR_Features_t;

/**
  @brief  红外检测器主结构体
*/
typedef struct {
    /* 历史窗口 (3 通道) */
    IR_History_t    history[IR_CH_NUM];

    /* 特征提取结果 */
    IR_Features_t   feat[IR_CH_NUM];
    uint32_t        ratio_45_38_x1000;  /* (P_45 / P_38) ×1000 */
    uint32_t        ratio_45_50_x1000;  /* (P_45 / P_50) ×1000 */

    /* 状态机 */
    IR_DetectorState_t  state;
    uint32_t            warning_start_ms;

    /* 灵敏度配置 (min/max 范围, 由 EEPROM 加载) */
    uint8_t     level;          /* 0~9 */
    uint32_t    pwr_min;
    uint32_t    pwr_max;
    uint32_t    r38_min;
    uint32_t    r38_max;
    uint32_t    r50_min;
    uint32_t    r50_max;
    uint32_t    cfm_min;
    uint32_t    cfm_max;

    /* 频率范围固定值 (不随灵敏度改变) */
    uint16_t    freq_low_x10;
    uint16_t    freq_high_x10;

    /* 当前运行参数 (由 SetLevel 基于 min/max 插值计算) */
    uint32_t    power_thr_x1000;    /* 功率阈值 (×1000) */
    uint32_t    ratio_38_thr_x1000; /* 光谱比阈值 R45/38 (×1000) */
    uint32_t    ratio_50_thr_x1000; /* 光谱比阈值 R45/50 (×1000) */
    uint32_t    confirm_ms;         /* 确认时长 (ms) */

    /* ISR → 主循环标志 */
    volatile uint8_t  feed_pending;
} IR_Detector_t;

/* Private variables ---------------------------------------------------------*/

static IR_Detector_t s_ir;

/* ========================================================================== */
/*                        内部辅助 — 状态名                                    */
/* ========================================================================== */

static const char *ir_state_name(IR_DetectorState_t s)
{
    switch (s) {
        case IR_STATE_IDLE:    return "IDLE";
        case IR_STATE_WARNING: return "WARNING";
        case IR_STATE_FIRE:    return "FIRE";
        default:               return "?";
    }
}

/* ========================================================================== */
/*                     历史窗口操作                                            */
/* ========================================================================== */

/**
  @brief  推入一个 ADC 原始值到指定通道的历史窗口
  @param  ch:  通道索引
  @param  raw: ADC 原始值 (0~4095)
*/
static void history_push(uint32_t ch, uint16_t raw)
{
    IR_History_t *h = &s_ir.history[ch];
    h->raw[h->head] = raw;
    h->head = (h->head + 1) % IR_HISTORY_SIZE;
    if (h->count < IR_HISTORY_SIZE) {
        h->count++;
    }
}

/**
  @brief  将历史窗口线性化到 int32_t 工作缓冲(从最旧到最新)
  @param  h:   历史窗口指针
  @param  buf: 输出缓冲(至少 IR_HISTORY_SIZE 个 int32_t)
  @return 有效数据点数
*/
static uint8_t history_to_workbuf(IR_History_t *h, int32_t *buf)
{
    uint8_t cnt = h->count;
    if (cnt == 0) return 0;

    uint8_t start = (h->head + IR_HISTORY_SIZE - cnt) % IR_HISTORY_SIZE;
    for (uint8_t i = 0; i < cnt; i++) {
        buf[i] = (int32_t)h->raw[(start + i) % IR_HISTORY_SIZE];
    }
    return cnt;
}

/* ========================================================================== */
/*                        预处理函数                                           */
/* ========================================================================== */

/**
  @brief  去直流 — 减去均值得到交流分量
  @param  buf: 输入/输出缓冲 (int32_t)
  @param  len: 数据点数
*/
static void remove_dc(int32_t *buf, uint16_t len)
{
    int64_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    int32_t mean = (int32_t)(sum / len);
    for (uint16_t i = 0; i < len; i++) {
        buf[i] -= mean;
    }
}

/**
  @brief  二阶 IIR 低通滤波 (Butterworth 40Hz @100Hz)
          直接 I 型：
            y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
                   - a1*y[n-1] - a2*y[n-2]
          每次从零状态开始 (x[-1]=x[-2]=y[-1]=y[-2]=0)
  @param  buf: 输入/输出缓冲 (in-place 滤波)
  @param  len: 数据点数
*/
static void iir_lowpass_40hz(int32_t *buf, uint16_t len)
{
    int32_t x1 = 0, x2 = 0;
    int32_t y1 = 0, y2 = 0;

    for (uint16_t i = 0; i < len; i++) {
        int32_t x0 = buf[i];
        int32_t y0 = (IIR_B0_Q15 * x0 + IIR_B1_Q15 * x1 + IIR_B2_Q15 * x2
                       - IIR_A1_Q15 * y1 - IIR_A2_Q15 * y2) >> 15;

        buf[i] = y0;

        x2 = x1; x1 = x0;
        y2 = y1; y1 = y0;
    }
}

/* ========================================================================== */
/*                        特征提取函数                                         */
/* ========================================================================== */

#if defined(IR_TEST_MODE)
/**
  @brief  计算均值 (DC 偏置)
  @param  buf: 原始信号 (int32_t)
  @param  len: 数据点数
  @return 均值 (int32_t)
*/
static int32_t calc_mean(const int32_t *buf, uint16_t len)
{
    int64_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return (int32_t)(sum / len);
}
#endif /* IR_TEST_MODE */

/**
  @brief  计算平均功率 (均方值 ×1000)
  @param  buf: 已去直流的信号 (int32_t)
  @param  len: 数据点数
  @return 功率值 ×1000 (uint32_t)
*/
static uint32_t calc_power_x1000(const int32_t *buf, uint16_t len)
{
    uint64_t sum_sq = 0;
    for (uint16_t i = 0; i < len; i++) {
        int64_t val = (int64_t)buf[i];
        sum_sq += val * val;
    }
    /* sum_sq / len × 1000 = sum_sq * 1000 / len */
    return (uint32_t)((sum_sq * 1000ULL) / len);
}

/**
  @brief  计算过零率 (ZCR)
          检测已去直流信号穿过零电平的次数
          ZCR = 过零次数 / (2 × 窗口秒数)
  @param  buf:  已去直流的信号 (int32_t)
  @param  len:  数据点数
  @param  fs:   采样率 (Hz)
  @return 过零率 (Hz)
*/
static float calc_zcr(const int32_t *buf, uint16_t len, float fs)
{
    uint16_t zc_count = 0;
    for (uint16_t i = 1; i < len; i++) {
        /* 符号变化: 正→负 或 负→正 */
        if ((buf[i - 1] >= 0 && buf[i] < 0) ||
            (buf[i - 1] < 0 && buf[i] >= 0)) {
            zc_count++;
        }
    }
    float window_time = (float)len / fs;
    if (window_time <= 0.0f) return 0.0f;
    return (float)zc_count / (2.0f * window_time);
}

/* ========================================================================== */
/*                     五判据串联决策                                           */
/* ========================================================================== */

/**
  @brief  检查五大判据是否全部通过
  @param  d: 检测器指针 (使用当前的 feat 和参数)
  @retval true:  全部通过 → 火焰确认
  @retval false: 任一判据不通过 → 无火
*/
static bool check_all_criteria(IR_Detector_t *d)
{
    /* ① 主通道功率 ≥ 阈值 */
    if (d->feat[IR_CH_MAIN].power_x1000 < d->power_thr_x1000) {
        DBG("FAIL criterion-1: P=%lu < THR=%lu",
            (unsigned long)d->feat[IR_CH_MAIN].power_x1000,
            (unsigned long)d->power_thr_x1000);
        return false;
    }

    /* ② 光谱比 R4.5/3.8 ≥ 阈值 (排除高温热源) */
    if (d->ratio_45_38_x1000 < d->ratio_38_thr_x1000) {
        DBG("FAIL criterion-2: R45/38=%lu < THR=%lu",
            (unsigned long)d->ratio_45_38_x1000,
            (unsigned long)d->ratio_38_thr_x1000);
        return false;
    }

    /* ③ 光谱比 R4.5/5.0 ≥ 阈值 (排除背景辐射) */
    if (d->ratio_45_50_x1000 < d->ratio_50_thr_x1000) {
        DBG("FAIL criterion-3: R45/50=%lu < THR=%lu",
            (unsigned long)d->ratio_45_50_x1000,
            (unsigned long)d->ratio_50_thr_x1000);
        return false;
    }

    /* ④ 过零率在频率范围内 */
    {
        float zcr = d->feat[IR_CH_MAIN].zcr_hz;
        uint16_t zcr_x10 = (uint16_t)(zcr * 10.0f + 0.5f);
        if (zcr_x10 < d->freq_low_x10 || zcr_x10 > d->freq_high_x10) {
            DBG("FAIL criterion-4: ZCR=%lu.%luHz (%lu*10) not in [%lu,%lu]",
                (unsigned long)zcr, (unsigned long)(zcr * 10) % 10,
                (unsigned long)zcr_x10,
                (unsigned long)d->freq_low_x10,
                (unsigned long)d->freq_high_x10);
            return false;
        }
    }

    /* ⑤ 频率一致性 (可选增强) — 各通道过零率应接近 */
    {
        float zcr_main = d->feat[IR_CH_MAIN].zcr_hz;
        float zcr_refa = d->feat[IR_CH_REF_A].zcr_hz;
        float zcr_refb = d->feat[IR_CH_REF_B].zcr_hz;
        float diff_a = (float)fabs((double)(zcr_main - zcr_refa));
        float diff_b = (float)fabs((double)(zcr_main - zcr_refb));
        if (diff_a > 5.0f || diff_b > 5.0f) {
            DBG("FAIL criterion-5: ZCR diff main-38=%.1fHz main-50=%.1fHz",
                diff_a, diff_b);
            return false;
        }
    }

    return true;
}

static void AP_IR_Process(uint32_t now)
{
    /* 窗口未满 50 点 → 跳过 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        if (s_ir.history[ch].count < IR_HISTORY_SIZE) return;
    }

    /* 工作缓冲 (复用, 最大通道点数) */
    int32_t work[IR_HISTORY_SIZE];

    /* ================================================================ */
    /*  逐通道: 线性化 → 去直流 → IIR → 特征提取                        */
    /* ================================================================ */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        history_to_workbuf(&s_ir.history[ch], work);
        remove_dc(work, IR_HISTORY_SIZE);
        iir_lowpass_40hz(work, IR_HISTORY_SIZE);

        s_ir.feat[ch].power_x1000 = calc_power_x1000(work, IR_HISTORY_SIZE);
        s_ir.feat[ch].zcr_hz = calc_zcr(work, IR_HISTORY_SIZE, 100.0f);
    }

    /* ================================================================ */
    /*  计算光谱比 (防除零) 大于0正常计算 否则输出9999                     */
    /* ================================================================ */
    {
        uint32_t p_refa = s_ir.feat[IR_CH_REF_A].power_x1000;
        uint32_t p_main = s_ir.feat[IR_CH_MAIN].power_x1000;
        uint32_t p_refb = s_ir.feat[IR_CH_REF_B].power_x1000;

        s_ir.ratio_45_38_x1000 = (p_refa > 0) ? (p_main * 1000U / p_refa) : 9999U;
        s_ir.ratio_45_50_x1000 = (p_refb > 0) ? (p_main * 1000U / p_refb) : 9999U;
    }

    /* ================================================================ */
    /*  状态机                                                           */
    /* ================================================================ */
    {
        IR_DetectorState_t old = s_ir.state;

        switch (old) {

            case IR_STATE_IDLE:
                if (check_all_criteria(&s_ir)) {
                    s_ir.state = IR_STATE_WARNING;
                    s_ir.warning_start_ms = now;
                }
                break;

            case IR_STATE_WARNING:
                if (check_all_criteria(&s_ir)) {
                    if ((now - s_ir.warning_start_ms) >= s_ir.confirm_ms) {
                        s_ir.state = IR_STATE_FIRE;
                        if (old != IR_STATE_FIRE) {
                            DBG("-> FIRE (confirmed %lums)", s_ir.confirm_ms);
                        }
                    }
                } else {
                    s_ir.state = IR_STATE_IDLE;
                }
                break;

            case IR_STATE_FIRE:
                /* 保持, 等待外部复位 */
                break;

            default:
                s_ir.state = IR_STATE_IDLE;
                break;
        }

        if (s_ir.state != old) {
            DBG("%s -> %s",
                ir_state_name(old), ir_state_name(s_ir.state));
        }
    }
}

/* ========================================================================== */
/*                        公有 API 实现                                        */
/* ========================================================================== */

void AP_IR_Init(void)
{
    memset(&s_ir, 0, sizeof(s_ir));
    s_ir.state = IR_STATE_IDLE;
    s_ir.feed_pending = 0;

    /* 从 EEPROM 加载参数并自动计算 */
    const AP_EEPROM_IR_Param_t *p = AP_EEPROM_IR_Get();
    s_ir.level = (p->sensitivity < IR_SENS_LEVELS) ? (uint8_t)p->sensitivity
                                                    : (uint8_t)(IR_SENS_LEVELS - 1);
    AP_IR_SetConfig(p->pwr_min, p->pwr_max,
                    p->r38_min, p->r38_max,
                    p->r50_min, p->r50_max,
                    (uint32_t)p->freq_low_x10, (uint32_t)p->freq_high_x10,
                    p->cfm_min, p->cfm_max);
    AP_IR_SetLevel(s_ir.level);

    DBG("Init complete, level=%u", s_ir.level);
}

/* ========================================================================== */

void AP_IR_FeedIsr(void)
{
    s_ir.feed_pending = 1;
}

/* ========================================================================== */

void AP_IR_Feed(void)
{
    if (!s_ir.feed_pending) return;
    s_ir.feed_pending = 0;

    /* 读 3 通道最新 ADC 值推入历史窗口（不进状态机） */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        uint16_t raw = AP_ADC_GetLatest(ch);
        history_push(ch, raw);
    }
}

void AP_IR_Task(void)
{
    AP_IR_Feed();                     /* Feed: 推入历史窗口     */
    AP_IR_Process(HAL_GetTick());     /* Process: 全流程检测    */
}

/* ========================================================================== */

/* ========================================================================== */
/*                    灵敏度/参数配置                                          */
/* ========================================================================== */

void AP_IR_SetConfig(uint32_t pwr_min, uint32_t pwr_max,
                     uint32_t r38_min, uint32_t r38_max,
                     uint32_t r50_min, uint32_t r50_max,
                     uint32_t freq_low_x10, uint32_t freq_high_x10,
                     uint32_t cfm_min, uint32_t cfm_max)
{
    s_ir.pwr_min      = pwr_min;
    s_ir.pwr_max      = pwr_max;
    s_ir.r38_min      = r38_min;
    s_ir.r38_max      = r38_max;
    s_ir.r50_min      = r50_min;
    s_ir.r50_max      = r50_max;
    s_ir.freq_low_x10   = (uint16_t)freq_low_x10;
    s_ir.freq_high_x10  = (uint16_t)freq_high_x10;
    s_ir.cfm_min      = cfm_min;
    s_ir.cfm_max      = cfm_max;
}

void AP_IR_GetConfig(uint32_t *pwr_min, uint32_t *pwr_max,
                     uint32_t *r38_min, uint32_t *r38_max,
                     uint32_t *r50_min, uint32_t *r50_max,
                     uint32_t *freq_low_x10, uint32_t *freq_high_x10,
                     uint32_t *cfm_min, uint32_t *cfm_max)
{
    if (pwr_min)      *pwr_min      = s_ir.pwr_min;
    if (pwr_max)      *pwr_max      = s_ir.pwr_max;
    if (r38_min)      *r38_min      = s_ir.r38_min;
    if (r38_max)      *r38_max      = s_ir.r38_max;
    if (r50_min)      *r50_min      = s_ir.r50_min;
    if (r50_max)      *r50_max      = s_ir.r50_max;
    if (freq_low_x10) *freq_low_x10 = s_ir.freq_low_x10;
    if (freq_high_x10)*freq_high_x10= s_ir.freq_high_x10;
    if (cfm_min)      *cfm_min      = s_ir.cfm_min;
    if (cfm_max)      *cfm_max      = s_ir.cfm_max;
}

void AP_IR_SetLevel(uint8_t level)
{
    s_ir.level = (level >= IR_SENS_LEVELS) ? (IR_SENS_LEVELS - 1) : level;
    uint32_t lv = s_ir.level;

    s_ir.power_thr_x1000  = lerp_u32(s_ir.pwr_min,  s_ir.pwr_max,   lv, IR_SENS_LEVELS);
    s_ir.ratio_38_thr_x1000 = lerp_u32(s_ir.r38_min, s_ir.r38_max,   lv, IR_SENS_LEVELS);
    s_ir.ratio_50_thr_x1000 = lerp_u32(s_ir.r50_min, s_ir.r50_max,   lv, IR_SENS_LEVELS);
    s_ir.confirm_ms        = lerp_u32(s_ir.cfm_min, s_ir.cfm_max, lv, IR_SENS_LEVELS);

    DBG("SetLevel %u: pwr=%lu r38=%lu r50=%lu cfm=%lu",
        s_ir.level,
        (unsigned long)s_ir.power_thr_x1000,
        (unsigned long)s_ir.ratio_38_thr_x1000,
        (unsigned long)s_ir.ratio_50_thr_x1000,
        (unsigned long)s_ir.confirm_ms);
}

void AP_IR_GetLevel(uint8_t *level)
{
    if (level) *level = s_ir.level;
}

void AP_IR_GetParams(uint32_t *pwr, uint32_t *r38, uint32_t *r50,
                     uint32_t *freq_low, uint32_t *freq_high, uint32_t *cfm)
{
    if (pwr)      *pwr      = s_ir.power_thr_x1000;
    if (r38)      *r38      = s_ir.ratio_38_thr_x1000;
    if (r50)      *r50      = s_ir.ratio_50_thr_x1000;
    if (freq_low) *freq_low = s_ir.freq_low_x10;
    if (freq_high)*freq_high= s_ir.freq_high_x10;
    if (cfm)      *cfm      = s_ir.confirm_ms;
}

/* ========================================================================== */

IR_DetectorState_t AP_IR_GetState(void)
{
    return s_ir.state;
}

void AP_IR_Reset(void)
{
    s_ir.state = IR_STATE_IDLE;
    s_ir.warning_start_ms = 0;
    /* 清空历史窗口 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        s_ir.history[ch].head  = 0;
        s_ir.history[ch].count = 0;
    }
    DBG("Reset -> IDLE");
}

void AP_IR_GetFeatures(uint32_t power[3], float zcr[3],
                       uint32_t *r45_38, uint32_t *r45_50)
{
    for (uint32_t i = 0; i < IR_CH_NUM; i++) {
        if (power) power[i] = s_ir.feat[i].power_x1000;
        if (zcr)   zcr[i]   = s_ir.feat[i].zcr_hz;
    }
    if (r45_38) *r45_38 = s_ir.ratio_45_38_x1000;
    if (r45_50) *r45_50 = s_ir.ratio_45_50_x1000;
}

/* ========================================================================== */
/*                    测试模式: 信号处理链调试打印                              */
/* ========================================================================== */

#if defined(IR_TEST_MODE)
void AP_IR_DebugProcess(uint32_t now)
{
    /* 窗口未满 50 点 → 跳过 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        if (s_ir.history[ch].count < IR_HISTORY_SIZE) return;
    }

    int32_t work[IR_HISTORY_SIZE];
    int32_t dc_offset[IR_CH_NUM];
    uint32_t power[IR_CH_NUM];
    float    zcr[IR_CH_NUM];

    /* 逐通道: 线性化 → 算DC偏置 → 去直流 → IIR → 特征提取 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        uint8_t n = history_to_workbuf(&s_ir.history[ch], work);
        if (n < IR_HISTORY_SIZE) return;

        dc_offset[ch] = calc_mean(work, IR_HISTORY_SIZE);
        remove_dc(work, IR_HISTORY_SIZE);
        iir_lowpass_40hz(work, IR_HISTORY_SIZE);

        power[ch] = calc_power_x1000(work, IR_HISTORY_SIZE);
        zcr[ch]   = calc_zcr(work, IR_HISTORY_SIZE, 100.0f);
    }

    /* 光谱比 (防除零) 大于0正常计算 否则输出9999 */
    uint32_t r45_38 = (power[IR_CH_REF_A] > 0)
                    ? (power[IR_CH_MAIN] * 1000U / power[IR_CH_REF_A]) : 9999U;
    uint32_t r45_50 = (power[IR_CH_REF_B] > 0)
                    ? (power[IR_CH_MAIN] * 1000U / power[IR_CH_REF_B]) : 9999U;

    /* 紧凑打印: T<ms> IRD DC=... P=... Z=... R=... */
    BSP_UART_Printf("T%lu IRD DC=%ld,%ld,%ld P=%lu,%lu,%lu Z=%.1f,%.1f,%.1f R=%lu,%lu\r\n",
        (unsigned long)now,
        (long)dc_offset[0], (long)dc_offset[1], (long)dc_offset[2],
        (unsigned long)power[0], (unsigned long)power[1], (unsigned long)power[2],
        (double)zcr[0], (double)zcr[1], (double)zcr[2],
        (unsigned long)r45_38, (unsigned long)r45_50);
}
#endif /* IR_TEST_MODE */
