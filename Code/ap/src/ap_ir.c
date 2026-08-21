/**
  ******************************************************************************
  * @file    ap_ir.c
  * @brief   AP 层红外三波段多光谱融合火焰检测模块实现
  *
  *          算法流程:
  *            AP_IR_Task() → 检查标志 → Feed(推ADC) → Process(检测)
  *
  *          Process 内部:
  *            1. 每通道连续执行 20Hz 二阶 IIR 低通滤波
  *            2. 最近 50 点去直流并计算平均功率(均方值) → uint32_t
  *            3. 最近 200 点独立去直流并计算过零率 → float Hz
  *            4. 计算光谱比 R4.5/3.8 和 R4.5/5.0 (×1000)
  *            5. 五判据串联决策
  *            6. 独立点火包络分类(打火机快速衰减/持续燃烧)
  *            7. 状态机迁移 (IDLE↔WARNING→FIRE)
  *
  *          参数管理:
  *            power(4.5um进场功率)与cfm(确认时长)按等级min→max插值
  *            r38/r50(光谱比)与频率范围为固定值
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ap_ir.h"
#include "ap_adc.h"
#include "ap_eeprom.h"
#include "ap_util.h"
#include "ap_uv.h"
#include "hal_uart.h"
#include <string.h>
/* ========================================================================== */
/*                          调试打印开关                                       */
/* ========================================================================== */
#if defined(AP_ALGO_DEBUG_ENABLE)
#define DBG(fmt, ...)   BSP_UART_Printf("[IR] " fmt "\r\n", ##__VA_ARGS__)
#define IR_DEBUG_LOG_INTERVAL_MS      (500U)
#else
#define DBG(fmt, ...)
#endif

/* ========================================================================== */
/*                      IIR 低通滤波器系数 (Q15 定点)                          */
/*         二阶 Butterworth Lowpass, fc=20Hz, fs=100Hz                        */
/*         直接 I 型: y = (b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2) >> 15       */
/*         系数缩放: Q15 = round(coeff * 32768)                               */
/* ========================================================================== */

#define IIR_B0_Q15  6770     /*  0.2066 * 32768 */
#define IIR_B1_Q15  13543    /*  0.4133 * 32768 */
#define IIR_B2_Q15  6770     /*  0.2066 * 32768 */
#define IIR_A1_Q15  (-12104) /* -0.3695 * 32768 */
#define IIR_A2_Q15  6416     /*  0.1958 * 32768 */

#define IR_SAMPLE_RATE_HZ             (100.0f)
#define IR_FREQ_CONSISTENCY_X10       (20U)
#define IR_ZCR_HOLD_MARGIN_X10        (3U)    /* 2秒ZCR窗口一个量化档约0.25Hz */
#define IR_REF_ZCR_RMS_GAIN           (2U)    /* 参考RMS至少达到2倍死区才信任ZCR */
#define IR_POWER_OFF_PERCENT          (40U)   /* 迟滞退出阈值=当前进场阈值的40% */
#define IR_POWER_DROPOUT_MS           (2000U) /* 连续低功率2秒才退出WARNING */
#define IR_CRITERIA_DROPOUT_MS        (2000U) /* 光谱/频率连续失效2秒才退出WARNING */
#define IR_PROCESS_STEP_MS            (10U)   /* 红外的固定处理周期 */

/*
 * 点火包络分类参数。
 *
 * 打火机在点火瞬间会产生很高的4.5um功率峰值，随后稳定功率快速下降；
 * 酒精火盆则通常由较低初始能量逐步进入持续、密集的稳定燃烧。这里不比较
 * 单点ADC毛刺，而是在有效冷启动后由P45第一次跨过当前等级ON阈值立即开始观察：
 * 前1.5秒持续记录0.5秒滚动均方值的最高峰，覆盖打火机约1秒内完成的
 * “0->峰值->快速衰减”过程；随后最多1.5秒独立统计衰减后的稳定能量和迟滞
 * 下限占空比。峰值窗与稳定窗不重叠；若WARNING先达到FIRE确认条件，稳定窗
 * 立即使用已采样本提前收口，不强制等待最大窗口结束。启动捕获
 * 不等待光谱/ZCR，因而不会遗漏真实点火峰值。
 *
 * 本段变量/日志术语：
 *   P45      = 4.5um主通道最近0.5秒去直流信号的均方值；
 *   ON       = 当前等级的power_threshold，P45达到该值视为出现有效火焰能量；
 *   OFF      = power_off_threshold，ON的40%，用于安静和迟滞判断；
 *   PEAK     = 首次达到ON后0~1.5秒内的最大P45；
 *   LATE     = 1.5秒后至FIRE_READY/最大3秒之间，P45>=OFF有效样本的平均值；
 *   R1000    = LATE/PEAK×1000，例如400表示后期保留峰值的40%；
 *   DUTY1000 = 观察段内P45>=OFF的样本占比×1000，例如600表示60%。
 *
 * PROFILE不影响IR进入WARNING及其证据积分，两个过程每10ms并行更新。只有
 * WARNING准备进入FIRE时才读取PROFILE结论：OBS先提前收口，LIGHTER阻止本次
 * FIRE，但可在后续能量持续恢复时单向升级为SUSTAINED；SUSTAINED禁止反向降级。
 * 高灵敏度下分类数据不足则放行。若启动后始终未进入WARNING，
 * P45低于OFF连续1秒即丢弃本次预采集并直接重新ARMED，不提交分类终态。
 */
#define IR_PROFILE_QUIET_MS                (1000U) /* 布防条件：P45连续低于OFF的时间，单位ms */
#define IR_PROFILE_RELEASE_MS  (IR_POWER_DROPOUT_MS) /* 终态释放沿用WARNING连续掉线超时 */
#define IR_PROFILE_PEAK_WINDOW_MS          (1500U) /* 峰值窗固定时长：覆盖打火机启动峰值及快速衰减 */
#define IR_PROFILE_LATE_WINDOW_MAX_MS      (1500U) /* 稳定窗最大时长；FIRE_READY可在中途提前收口 */
#define IR_PROFILE_LATE_START_MS  (IR_PROFILE_PEAK_WINDOW_MS) /* 稳定窗紧接峰值窗 */
#define IR_PROFILE_OBSERVE_END_MS \
    (IR_PROFILE_LATE_START_MS + IR_PROFILE_LATE_WINDOW_MAX_MS) /* 完整观察最大时长 */
#define IR_PROFILE_DECAY_RATIO_X1000        (400U) /* LATE/PEAK门限：400表示40.0% */
#define IR_PROFILE_LIGHTER_PEAK_MIN       (100000U) /* 打火机峰值门槛：低能量启动沿不参与打火机分类 */
#define IR_PROFILE_RECOVERY_WINDOW_MS       (1000U) /* LIGHTER后持续火焰恢复判定窗口，单位ms */
#define IR_PROFILE_RECOVERY_POWER_MIN      (50000U) /* 恢复窗有效均值门槛：5万以上视为真实火焰候选 */
#define IR_PROFILE_RECOVERY_CONFIRM_WINDOWS    (2U) /* 连续通过2个恢复窗才升级，拒绝单窗偶发高值 */
#define IR_PROFILE_RECOVERY_DUTY_X1000       (600U) /* 恢复窗口内P45>=OFF至少占60%，避免单点峰值升级 */

#if (IR_PROFILE_PEAK_WINDOW_MS != IR_PROFILE_LATE_START_MS) || \
    (IR_PROFILE_LATE_WINDOW_MAX_MS == 0U) || \
    ((IR_PROFILE_QUIET_MS % IR_PROCESS_STEP_MS) != 0U) || \
    ((IR_PROFILE_RELEASE_MS % IR_PROCESS_STEP_MS) != 0U) || \
    ((IR_PROFILE_PEAK_WINDOW_MS % IR_PROCESS_STEP_MS) != 0U) || \
    ((IR_PROFILE_LATE_WINDOW_MAX_MS % IR_PROCESS_STEP_MS) != 0U) || \
    ((IR_PROFILE_OBSERVE_END_MS % IR_PROCESS_STEP_MS) != 0U) || \
    ((IR_PROFILE_RECOVERY_WINDOW_MS % IR_PROCESS_STEP_MS) != 0U) || \
    (IR_PROFILE_RECOVERY_POWER_MIN == 0U) || \
    (IR_PROFILE_RECOVERY_CONFIRM_WINDOWS == 0U) || \
    (IR_PROFILE_RECOVERY_CONFIRM_WINDOWS > 255U) || \
    (IR_PROFILE_DECAY_RATIO_X1000 > 1000U) || \
    (IR_PROFILE_RECOVERY_DUTY_X1000 > 1000U)
#error "IR transient profile parameters are invalid"
#endif

/* ========================================================================== */
/*                         内部数据结构                                        */
/* ========================================================================== */

/**
  @brief  单通道历史窗口 (环形缓冲)
          raw保留给测试打印/DC观测，filtered供正式算法提取特征，
          避免为了调试重新执行或改变正式滤波链。
*/
typedef struct {
    uint16_t    raw[IR_HISTORY_SIZE];
    int32_t     filtered[IR_HISTORY_SIZE];
    uint16_t    head;
    uint16_t    count;
} IR_History_t;

/** @brief 每通道独立的连续IIR状态，确保每个ADC样本只参与一次滤波。 */
typedef struct {
    int32_t x1, x2;
    int32_t y1, y2;
    uint8_t initialized;
} IR_IIR_State_t;

/**
  @brief  单通道特征提取结果
*/
typedef struct {
    uint32_t    power_x1000;        /* 去直流信号均方值(保留历史字段名) */
    float       zcr_hz;             /* 过零率(Hz) */
} IR_Features_t;

/**
  @brief  主通道独立时域包络分类状态

  状态迁移：
    BYPASS --P45<OFF持续1秒---------------> ARMED
    ARMED  --P45首次达到ON---------------> OBSERVING
    OBSERVING --已进入WARNING且观察到期/FIRE就绪--> LIGHTER/SUSTAINED
    OBSERVING --从未进入WARNING且P45<OFF持续1秒--> ARMED
    BYPASS --当周期P45>=ON且正常IR判据通过--> SUSTAINED
    LIGHTER --有效能量持续恢复------------> SUSTAINED
    LIGHTER/SUSTAINED --P45<OFF持续2秒-----> BYPASS
    BYPASS --P45<OFF重新持续1秒------------> ARMED

  BYPASS表示尚未观察到完整的“安静背景->点火”过程，此时没有有效包络分类；
  若设备面对已经燃烧且持续波动的火焰，P45可能无法连续低于OFF满1秒。此时正常
  正常IR状态机当周期确认P45>=ON且光谱/频率判据通过后，BYPASS才按热启动
  直接接纳为SUSTAINED，不能使用掉线宽限期内遗留的旧WARNING状态。
  LIGHTER是可恢复的干扰结论，后续出现持续真实火焰时只允许升级为SUSTAINED；
  SUSTAINED是本次火源的最终结论，禁止反向降级为LIGHTER。
  消退后必须先回BYPASS，再重新完成安静布防，下一次ON上穿沿才开始新分类。
  PROFILE不暂停IR确认积分；仅进入过WARNING的事件才提交分类结果。没有进入
  WARNING的短瞬态在安静1秒后直接丢弃并重新ARMED，不产生额外2秒锁存时间。
 */
typedef enum {
    IR_PROFILE_BYPASS = 0, /* 未布防：等待新的完整安静期 */
    IR_PROFILE_ARMED,      /* 已布防：已确认安静背景，等待P45第一次跨过ON */
    IR_PROFILE_OBSERVING,  /* 采集/等待中：记录包络并等待WARNING或安静取消 */
    IR_PROFILE_LIGHTER,    /* 当前识别为打火机干扰，可单向升级为持续火焰 */
    IR_PROFILE_SUSTAINED   /* 本次点火沿分类为持续火焰，锁存到火源消退 */
} IR_ProfileState_t;

/**
  @brief  主通道点火包络分类上下文

  late_sum/late_samples/late_high_samples在OBS稳定段和LIGHTER恢复窗口复用：
    - late_sum只累加P45>=OFF的有效样本，低于OFF的样本不进入能量均值；
    - late_samples记录窗口总样本数，仅用于计算有效占空比；
    - late_high_samples记录有效样本数，也是late_sum计算均值时的分母。
  recovery_pass_windows记录连续满足恢复判据的1秒窗口数量。恢复能量采用或关系：
  有效均值>=5万，或有效均值恢复到启动峰值的40%以上；两条路径都必须同时满足
  有效占比>=60%。任一窗口不通过立即清零，连续2窗通过才允许升级。
  low_power_accum_ms在不同状态下有三种明确语义：
    - BYPASS：重新布防前的连续安静时间；
    - OBSERVING且未见WARNING：判定无效短瞬态的连续安静时间；
    - LIGHTER/SUSTAINED：确认本次火源已经消退的连续低功率时间。
 */
typedef struct {
    IR_ProfileState_t state;             /* 当前包络分类状态 */
    uint32_t phase_start_ms;             /* OBS观察或LIGHTER恢复窗口的起始tick，单位ms */
    uint32_t low_power_accum_ms;         /* 当前阶段P45连续低于OFF的累计时间，单位ms */
    uint32_t early_peak;                 /* 首次跨过ON后0~1.5秒内P45最大值 */
    uint64_t late_sum;                   /* 当前后段窗口内P45>=OFF有效样本的累加和 */
    uint16_t late_samples;               /* 当前后段窗口的全部10ms样本数量 */
    uint16_t late_high_samples;          /* 当前后段窗口内P45>=OFF的有效样本数量 */
    uint8_t warning_seen;                /* 本次观察是否实际进入过IR WARNING */
    uint8_t recovery_pass_windows;       /* LIGHTER连续通过恢复判据的1秒窗口数量 */
} IR_Profile_t;

/**
  @brief  红外检测器主结构体
*/
typedef struct {
    /* 历史窗口 (3 通道) */
    IR_History_t    history[IR_CH_NUM];
    IR_IIR_State_t  iir[IR_CH_NUM];

    /* 特征提取结果 */
    IR_Features_t   feat[IR_CH_NUM];
    uint32_t        ratio_45_38_x1000;  /* (P_45 / P_38) ×1000 */
    uint32_t        ratio_45_50_x1000;  /* (P_45 / P_50) ×1000 */
    uint16_t        zcr_dead_zone[IR_CH_NUM]; /* 从EEPROM加载的各通道固定死区 */
    uint8_t         dead_zone_ready;          /* EEPROM固定死区加载完成标志 */

    /* 状态机 */
    IR_DetectorState_t  state;
    uint32_t            fire_start_ms;       /* FIRE绝对超时起点 */
    uint32_t            valid_accum_ms;       /* WARNING有效证据积分 */
    uint32_t            power_drop_start_ms;  /* 低于迟滞下限的起点 */
    uint32_t            criteria_drop_start_ms; /* 光谱/频率失效起点 */
    uint8_t             power_active;         /* 功率迟滞锁存状态 */
    uint8_t             power_drop_active;    /* 避免tick=0作为无效哨兵 */
    uint8_t             criteria_drop_active; /* WARNING其余判据失效标志 */
    IR_Profile_t        profile;              /* 点火后功率包络及稳定占空比分类器 */
    uint8_t             profile_enabled;      /* 0时跳过包络分类，不阻止正常IR判警 */

    /* 功率/确认时间范围及固定判据，由EEPROM加载。 */
    uint8_t     level;          /* 0~9 */
    uint32_t    power_min;           /* 等级0进场功率，最灵敏 */
    uint32_t    power_max;           /* 等级9进场功率，最迟钝 */
    uint32_t    power_threshold;     /* 当前等级线性插值得到的进场功率 */
    uint32_t    power_off_threshold; /* 当前进场功率的40%，用于迟滞退出 */
    uint32_t    ratio_38_threshold;  /* 固定光谱特征，不随报警等级插值 */
    uint32_t    ratio_50_threshold;  /* 固定光谱特征，不随报警等级插值 */
    uint32_t    cfm_min;
    uint32_t    cfm_max;

    /* 频率范围固定值 (不随灵敏度改变) */
    uint16_t    freq_low_x10;
    uint16_t    freq_high_x10;

    /* 当前运行参数：功率在上方保存，光谱比固定，确认时间按等级插值。 */
    uint32_t    ratio_38_thr_x1000; /* 光谱比阈值 R45/38 (×1000) */
    uint32_t    ratio_50_thr_x1000; /* 光谱比阈值 R45/50 (×1000) */
    uint32_t    confirm_ms;         /* 确认时长 (ms) */

    /* ISR → 主循环标志 */
    volatile uint8_t  feed_pending;
} IR_Detector_t;

/* Private variables ---------------------------------------------------------*/

static IR_Detector_t s_ir;

/* 手动标定独立保存5个窗口结果；标定完成前不改变正式算法正在使用的死区。 */
typedef struct {
    AP_IR_ZcrCalState_t state;
    uint8_t  windows_collected;
    uint32_t phase_start_ms;
    uint32_t next_window_ms;
    uint16_t p90_window[IR_CH_NUM][IR_ZCR_CAL_WINDOW_COUNT];
} IR_ZcrCalibration_t;

static IR_ZcrCalibration_t s_zcr_cal;

static int32_t iir_lowpass_20hz_sample(uint32_t ch, int32_t x0);
static uint16_t zcr_to_x10(float zcr_hz);
static bool reference_zcr_valid(const IR_Detector_t *d, uint32_t ch);
static void process_zcr_calibration(uint32_t now);

#if defined(AP_ALGO_DEBUG_ENABLE)
/* 算法调试日志独立限频，避免判据在100Hz循环中持续占满调试串口。 */
static uint32_t s_debug_last_fail_ms;
static uint32_t s_debug_last_snapshot_ms;
static uint8_t  s_debug_window_ready;
#endif

#define IR_TEST_AVG_WINDOW_MS_DEFAULT  (10000UL)
#define IR_TEST_AVG_WINDOW_MS_MIN      (100UL)
#define IR_TEST_AVG_WINDOW_MS_MAX      (60000UL)
#define IR_TEST_SAMPLE_PERIOD_MS       (10UL)
#define IR_TEST_WINDOW_MARK_VALUE      (4096UL)

typedef struct {
    uint64_t sum[IR_CH_NUM];
    uint32_t samples;
    uint32_t start_ms;
    uint32_t window_ms;
    uint32_t last_avg[IR_CH_NUM];
    uint8_t  avg_valid;
    uint8_t  marker_pending;
} IR_TestStats_t;

static IR_TestStats_t s_ir_test;

static void test_stats_reset(uint32_t now)
{
    memset(s_ir_test.sum, 0, sizeof(s_ir_test.sum));
    s_ir_test.samples = 0;
    s_ir_test.start_ms = now;
}

static uint32_t test_stats_window_samples(void)
{
    uint32_t n = s_ir_test.window_ms / IR_TEST_SAMPLE_PERIOD_MS;
    return (n == 0U) ? 1U : n;
}

static void test_stats_add(const uint32_t pwr[IR_CH_NUM])
{
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        s_ir_test.sum[ch] += pwr[ch];
    }
    s_ir_test.samples++;

    if (s_ir_test.samples >= test_stats_window_samples()) {
        for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
            s_ir_test.last_avg[ch] = (uint32_t)(s_ir_test.sum[ch] / s_ir_test.samples);
        }
        s_ir_test.avg_valid = 1;
        s_ir_test.marker_pending = 1;
        test_stats_reset(HAL_GetTick());
    }
}

static void test_stats_get_energy(uint32_t avg[IR_CH_NUM])
{
    if (s_ir_test.marker_pending) {
        for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
            avg[ch] = IR_TEST_WINDOW_MARK_VALUE;
        }
        s_ir_test.marker_pending = 0;
        return;
    }

    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        avg[ch] = s_ir_test.avg_valid ? s_ir_test.last_avg[ch] : 0U;
    }
}

void AP_IR_TestSetAvgWindowMs(uint32_t ms)
{
    if (ms < IR_TEST_AVG_WINDOW_MS_MIN) ms = IR_TEST_AVG_WINDOW_MS_MIN;
    if (ms > IR_TEST_AVG_WINDOW_MS_MAX) ms = IR_TEST_AVG_WINDOW_MS_MAX;
    s_ir_test.window_ms = ms;
    s_ir_test.avg_valid = 0;
    s_ir_test.marker_pending = 0;
    test_stats_reset(HAL_GetTick());
}

uint32_t AP_IR_TestGetAvgWindowMs(void)
{
    return s_ir_test.window_ms;
}

/* ========================================================================== */
/*                        内部辅助 — 状态名                                    */
/* ========================================================================== */

#if defined(AP_ALGO_DEBUG_ENABLE)
static const char *ir_state_name(IR_DetectorState_t s)
{
    switch (s) {
        case IR_STATE_IDLE:    return "IDLE";
        case IR_STATE_WARNING: return "WARNING";
        case IR_STATE_FIRE:    return "FIRE";
        default:               return "?";
    }
}

static const char *ir_profile_name(IR_ProfileState_t state)
{
    switch (state) {
        case IR_PROFILE_BYPASS:    return "BYPASS";
        case IR_PROFILE_ARMED:     return "ARMED";
        case IR_PROFILE_OBSERVING: return "OBS";
        case IR_PROFILE_LIGHTER:   return "LIGHTER";
        case IR_PROFILE_SUSTAINED: return "SUSTAINED";
        default:                   return "?";
    }
}

static bool debug_log_due(uint32_t *last_ms, uint32_t now)
{
    if ((now - *last_ms) < IR_DEBUG_LOG_INTERVAL_MS) return false;
    *last_ms = now;
    return true;
}

static void debug_log_features(uint32_t now)
{
    if (!debug_log_due(&s_debug_last_snapshot_ms, now)) return;

    /* 单行快照覆盖五判据输入和WARNING积分，便于直接对应状态机行为。 */
    DBG("T%lu S=%s P=%lu,%lu,%lu R1000=%lu,%lu Z10=%u,%u,%u DZ=%u,%u,%u ACC=%lu/%lu DROP=%u REFV=%u,%u PROF=%s",
        (unsigned long)now, ir_state_name(s_ir.state),
        (unsigned long)s_ir.feat[0].power_x1000,
        (unsigned long)s_ir.feat[1].power_x1000,
        (unsigned long)s_ir.feat[2].power_x1000,
        (unsigned long)s_ir.ratio_45_38_x1000,
        (unsigned long)s_ir.ratio_45_50_x1000,
        (unsigned int)zcr_to_x10(s_ir.feat[0].zcr_hz),
        (unsigned int)zcr_to_x10(s_ir.feat[1].zcr_hz),
        (unsigned int)zcr_to_x10(s_ir.feat[2].zcr_hz),
        (unsigned int)s_ir.zcr_dead_zone[0],
        (unsigned int)s_ir.zcr_dead_zone[1],
        (unsigned int)s_ir.zcr_dead_zone[2],
        (unsigned long)s_ir.valid_accum_ms,
        (unsigned long)s_ir.confirm_ms,
        (unsigned int)((s_ir.power_drop_active != 0U) ||
                       (s_ir.criteria_drop_active != 0U)),
        (unsigned int)reference_zcr_valid(&s_ir, IR_CH_REF_A),
        (unsigned int)reference_zcr_valid(&s_ir, IR_CH_REF_B),
        s_ir.profile_enabled ? ir_profile_name(s_ir.profile.state) : "OFF");
}

#define DBG_CRITERION(fmt, ...)                                                \
    do {                                                                       \
        uint32_t debug_now = HAL_GetTick();                                    \
        if (debug_log_due(&s_debug_last_fail_ms, debug_now)) {                 \
            DBG("T%lu FAIL " fmt, (unsigned long)debug_now, ##__VA_ARGS__);    \
        }                                                                      \
    } while (0)
#else
#define DBG_CRITERION(fmt, ...)
#endif

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
    /* 同一采样点同时保存原值和连续滤波值，保证两条数据链时间对齐。 */
    h->raw[h->head] = raw;
    h->filtered[h->head] = iir_lowpass_20hz_sample(ch, raw);
    h->head = (h->head + 1) % IR_HISTORY_SIZE;
    if (h->count < IR_HISTORY_SIZE) {
        h->count++;
    }
}

/**
  @brief  将最近指定数量的滤波历史线性化到工作缓冲(从最旧到最新)
  @param  h:   历史窗口指针
  @param  buf: 输出缓冲(至少 request_count 个 int32_t)
  @param  request_count: 需要的最近样本数
  @return 有效数据点数
*/
static uint16_t history_to_workbuf(const IR_History_t *h, int32_t *buf,
                                   uint16_t request_count)
{
    uint16_t cnt = (h->count < request_count) ? h->count : request_count;
    if (cnt == 0) return 0;

    uint16_t start = (h->head + IR_HISTORY_SIZE - cnt) % IR_HISTORY_SIZE;
    for (uint16_t i = 0; i < cnt; i++) {
        buf[i] = h->filtered[(start + i) % IR_HISTORY_SIZE];
    }
    return cnt;
}

static uint16_t history_raw_to_workbuf(const IR_History_t *h, int32_t *buf,
                                       uint16_t request_count)
{
    uint16_t cnt = (h->count < request_count) ? h->count : request_count;
    if (cnt == 0U) return 0U;

    uint16_t start = (h->head + IR_HISTORY_SIZE - cnt) % IR_HISTORY_SIZE;
    for (uint16_t i = 0; i < cnt; i++) {
        buf[i] = h->raw[(start + i) % IR_HISTORY_SIZE];
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

static uint16_t calculate_zcr_noise_p90(uint32_t ch)
{
    int32_t work[IR_ZCR_CAL_WINDOW_SAMPLES];
    const uint16_t index = (uint16_t)(((IR_ZCR_CAL_WINDOW_SAMPLES *
                                       IR_ZCR_NOISE_PERCENTILE + 99U) / 100U) - 1U);

    history_to_workbuf(&s_ir.history[ch], work, IR_ZCR_CAL_WINDOW_SAMPLES);
    remove_dc(work, IR_ZCR_CAL_WINDOW_SAMPLES);

    /* P90使用绝对交流幅度，忽略最顶部10%的偶发毛刺。 */
    for (uint16_t i = 0U; i < IR_ZCR_CAL_WINDOW_SAMPLES; i++) {
        int64_t sample = work[i];
        uint64_t magnitude = (uint64_t)((sample < 0) ? -sample : sample);
        work[i] = (magnitude > UINT16_MAX) ? UINT16_MAX : (int32_t)magnitude;
    }
    for (uint16_t i = 1U; i < IR_ZCR_CAL_WINDOW_SAMPLES; i++) {
        int32_t value = work[i];
        uint16_t j = i;
        while ((j > 0U) && (work[j - 1U] > value)) {
            work[j] = work[j - 1U];
            j--;
        }
        work[j] = value;
    }

    return (uint16_t)work[index];
}

static uint16_t median_zcr_noise(const uint16_t values[IR_ZCR_CAL_WINDOW_COUNT])
{
    uint16_t sorted[IR_ZCR_CAL_WINDOW_COUNT];
    memcpy(sorted, values, sizeof(sorted));

    /* 5个窗口取中位数，避免某一个窗口的环境扰动永久写入EEPROM。 */
    for (uint32_t i = 1U; i < IR_ZCR_CAL_WINDOW_COUNT; i++) {
        uint16_t value = sorted[i];
        uint32_t j = i;
        while ((j > 0U) && (sorted[j - 1U] > value)) {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = value;
    }
    return sorted[IR_ZCR_CAL_WINDOW_COUNT / 2U];
}

static void finish_zcr_calibration(void)
{
    AP_EEPROM_IR_Param_t candidate = *AP_EEPROM_IR_Get();

    for (uint32_t ch = 0U; ch < IR_CH_NUM; ch++) {
        uint32_t dead_zone = (uint32_t)(((uint64_t)median_zcr_noise(
                                            s_zcr_cal.p90_window[ch]) *
                                        IR_ZCR_NOISE_GAIN_NUM +
                                        IR_ZCR_NOISE_GAIN_DEN - 1U) /
                                       IR_ZCR_NOISE_GAIN_DEN);
        if (dead_zone < IR_ZCR_DEAD_ZONE_MIN) dead_zone = IR_ZCR_DEAD_ZONE_MIN;
        /* 手动标定仍限制在10~30，防止强扰动窗口屏蔽真实火焰过零。 */
        if (dead_zone > IR_ZCR_DEAD_ZONE_MAX) dead_zone = IR_ZCR_DEAD_ZONE_MAX;
        candidate.zcr_dead_zone[ch] = dead_zone;
    }

    /* EEPROM写入成功后才切换运行参数，保证掉电配置与当前判据一致。 */
    if (AP_EEPROM_IR_Save(&candidate) != 0) {
        s_zcr_cal.state = AP_IR_ZCR_CAL_ERROR;
        DBG("ZCR_CAL save failed; keep DZ=%u,%u,%u",
            (unsigned int)s_ir.zcr_dead_zone[0],
            (unsigned int)s_ir.zcr_dead_zone[1],
            (unsigned int)s_ir.zcr_dead_zone[2]);
        return;
    }

    for (uint32_t ch = 0U; ch < IR_CH_NUM; ch++) {
        s_ir.zcr_dead_zone[ch] = (uint16_t)candidate.zcr_dead_zone[ch];
    }
    s_ir.dead_zone_ready = 1U;
    s_zcr_cal.state = AP_IR_ZCR_CAL_DONE;
    DBG("ZCR_CAL saved DZ=%u,%u,%u",
        (unsigned int)s_ir.zcr_dead_zone[0],
        (unsigned int)s_ir.zcr_dead_zone[1],
        (unsigned int)s_ir.zcr_dead_zone[2]);
}

static void process_zcr_calibration(uint32_t now)
{
    if (s_zcr_cal.state == AP_IR_ZCR_CAL_WARMUP) {
        if ((now - s_zcr_cal.phase_start_ms) < IR_ZCR_CAL_WARMUP_MS) return;

        /* 预热结束后再完整等待2秒，确保第一个窗口不包含启动阶段数据。 */
        s_zcr_cal.state = AP_IR_ZCR_CAL_COLLECTING;
        s_zcr_cal.next_window_ms = now +
            (IR_ZCR_CAL_WINDOW_SAMPLES * 1000U / (uint32_t)IR_SAMPLE_RATE_HZ);
        DBG("ZCR_CAL collecting %u windows",
            (unsigned int)IR_ZCR_CAL_WINDOW_COUNT);
        return;
    }

    if (s_zcr_cal.state != AP_IR_ZCR_CAL_COLLECTING) return;

    /* 出现疑似火焰时终止标定，避免把火焰交流幅度保存成本底噪声。 */
    if (s_ir.state != IR_STATE_IDLE) {
        s_zcr_cal.state = AP_IR_ZCR_CAL_ERROR;
        DBG("ZCR_CAL aborted: IR state=%u", (unsigned int)s_ir.state);
        return;
    }
    if ((int32_t)(now - s_zcr_cal.next_window_ms) < 0) return;

    for (uint32_t ch = 0U; ch < IR_CH_NUM; ch++) {
        if (s_ir.history[ch].count < IR_ZCR_CAL_WINDOW_SAMPLES) return;
        s_zcr_cal.p90_window[ch][s_zcr_cal.windows_collected] =
            calculate_zcr_noise_p90(ch);
    }
    DBG("ZCR_CAL window=%u/%u P90=%u,%u,%u",
        (unsigned int)(s_zcr_cal.windows_collected + 1U),
        (unsigned int)IR_ZCR_CAL_WINDOW_COUNT,
        (unsigned int)s_zcr_cal.p90_window[0][s_zcr_cal.windows_collected],
        (unsigned int)s_zcr_cal.p90_window[1][s_zcr_cal.windows_collected],
        (unsigned int)s_zcr_cal.p90_window[2][s_zcr_cal.windows_collected]);

    s_zcr_cal.windows_collected++;
    if (s_zcr_cal.windows_collected >= IR_ZCR_CAL_WINDOW_COUNT) {
        finish_zcr_calibration();
    } else {
        /* 以上次实际取样时刻为起点，确保下一窗口与当前窗口不重叠。 */
        s_zcr_cal.next_window_ms = now +
            (IR_ZCR_CAL_WINDOW_SAMPLES * 1000U / (uint32_t)IR_SAMPLE_RATE_HZ);
    }
}

/**
  @brief  单点二阶 IIR 低通滤波 (Butterworth 20Hz @100Hz)
          直接 I 型：
            y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
                   - a1*y[n-1] - a2*y[n-2]
          每个通道独立维护状态，每个新采样点只处理一次。
*/
static int32_t iir_lowpass_20hz_sample(uint32_t ch, int32_t x0)
{
    IR_IIR_State_t *st = &s_ir.iir[ch];
    if (st->initialized == 0U) {
        /* 用首个ADC值预充状态，避免从0启动产生大幅人工瞬态。 */
        st->x1 = x0;
        st->x2 = x0;
        st->y1 = x0;
        st->y2 = x0;
        st->initialized = 1U;
        return x0;
    }

    int64_t acc = (int64_t)IIR_B0_Q15 * x0
                + (int64_t)IIR_B1_Q15 * st->x1
                + (int64_t)IIR_B2_Q15 * st->x2
                - (int64_t)IIR_A1_Q15 * st->y1
                - (int64_t)IIR_A2_Q15 * st->y2;
    int32_t y0 = (int32_t)(acc >> 15);

    st->x2 = st->x1;
    st->x1 = x0;
    st->y2 = st->y1;
    st->y1 = y0;
    return y0;
}

/* ========================================================================== */
/*                        特征提取函数                                         */
/* ========================================================================== */

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

/**
  @brief  计算平均功率，即去直流信号的均方值
  @param  buf: 已去直流的信号 (int32_t)
  @param  len: 数据点数
  @return 均方值 (uint32_t，超范围时饱和)
*/
static uint32_t calc_mean_square(const int32_t *buf, uint16_t len)
{
    uint64_t sum_sq = 0;
    for (uint16_t i = 0; i < len; i++) {
        int64_t val = (int64_t)buf[i];
        sum_sq += val * val;
    }
    uint64_t mean_square = sum_sq / len;
    return (mean_square > UINT32_MAX) ? UINT32_MAX : (uint32_t)mean_square;
}

static uint32_t calc_ratio_x1000(uint32_t numerator, uint32_t denominator)
{
    /* 主通道有能量而参考通道严格为0时，真实比例趋于无穷，按饱和值通过下限判据。 */
    if (denominator == 0U) return (numerator > 0U) ? UINT32_MAX : 0U;
    uint64_t ratio = (uint64_t)numerator * 1000U / denominator;
    return (ratio > UINT32_MAX) ? UINT32_MAX : (uint32_t)ratio;
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
static float calc_zcr(const int32_t *buf, uint16_t len, float fs,
                      uint16_t dead_zone)
{
    uint16_t zc_count = 0;
    int8_t last_sign = 0;

    for (uint16_t i = 0; i < len; i++) {
        int8_t sign = 0;
        if (buf[i] > (int32_t)dead_zone) {
            sign = 1;
        } else if (buf[i] < -(int32_t)dead_zone) {
            sign = -1;
        }

        if (sign == 0) continue;
        if (last_sign != 0 && sign != last_sign) zc_count++;
        last_sign = sign;
    }
    float window_time = (float)len / fs;
    if (window_time <= 0.0f) return 0.0f;
    return (float)zc_count / (2.0f * window_time);
}

static uint16_t zcr_to_x10(float zcr_hz)
{
    if (zcr_hz <= 0.0f) return 0U;
    if (zcr_hz >= 6553.5f) return UINT16_MAX;
    return (uint16_t)(zcr_hz * 10.0f + 0.5f);
}

/* ========================================================================== */
/*                     光谱与频率硬判据                                        */
/* ========================================================================== */

/**
  @brief  检查两项光谱比判据
  @param  d: 检测器指针 (使用当前的 feat 和参数)
  @retval true:  两项光谱比均通过
  @retval false: 任一光谱比不通过
*/
static bool check_spectral_criteria(IR_Detector_t *d)
{
    /* ② 光谱比 R4.5/3.8 ≥ 阈值 (排除高温热源) */
    if (d->ratio_45_38_x1000 < d->ratio_38_thr_x1000) {
        DBG_CRITERION("criterion-2 R45/38=%lu < THR=%lu P38=%lu",
            (unsigned long)d->ratio_45_38_x1000,
            (unsigned long)d->ratio_38_thr_x1000,
            (unsigned long)d->feat[IR_CH_REF_A].power_x1000);
        return false;
    }

    /* ③ 光谱比 R4.5/5.0 ≥ 阈值 (排除背景辐射) */
    if (d->ratio_45_50_x1000 < d->ratio_50_thr_x1000) {
        DBG_CRITERION("criterion-3 R45/50=%lu < THR=%lu P50=%lu",
            (unsigned long)d->ratio_45_50_x1000,
            (unsigned long)d->ratio_50_thr_x1000,
            (unsigned long)d->feat[IR_CH_REF_B].power_x1000);
        return false;
    }

    return true;
}

/**
  @brief  判断参考通道当前功率是否足以支撑ZCR一致性判定
          均方值与幅度平方同量纲，以2倍标定死区的平方作为有效下限，
          避免刚越过死区的弱参考信号将噪声ZCR当成真实频率。
  @param  d: 检测器实例
  @param  ch: 参考通道索引，只允许传入IR_CH_REF_A或IR_CH_REF_B
  @retval true: 参考通道功率足以参与频率一致性判断
  @retval false: 参考通道过弱，本周期跳过该通道一致性判断
 */
static bool reference_zcr_valid(const IR_Detector_t *d, uint32_t ch)
{
    uint32_t valid_rms = (uint32_t)d->zcr_dead_zone[ch] * IR_REF_ZCR_RMS_GAIN;
    uint32_t valid_power = valid_rms * valid_rms;
    return d->feat[ch].power_x1000 >= valid_power;
}

/**
  @brief  检查ZCR范围和三通道频率一致性
          频率来自2秒滚动窗口，同时用于IDLE入态和WARNING证据维持。
  @param  d: 检测器实例，读取当前ZCR、功率、死区和频率配置
  @param  warning_hold: false使用严格入态频带；true将上下限各放宽一个ZCR量化档
  @retval true: 主通道在频带内，且所有有效参考通道满足一致性
  @retval false: 主频率越界，或任一有效参考通道与主通道差值超过上限
 */
static bool check_frequency_criteria(IR_Detector_t *d, bool warning_hold)
{
    /* ④ 过零率在频率范围内 */
    {
        uint16_t zcr_x10 = zcr_to_x10(d->feat[IR_CH_MAIN].zcr_hz);
        uint16_t low_x10 = d->freq_low_x10;
        uint16_t high_x10 = d->freq_high_x10;
        if (warning_hold) {
            low_x10 = (low_x10 > IR_ZCR_HOLD_MARGIN_X10)
                        ? (uint16_t)(low_x10 - IR_ZCR_HOLD_MARGIN_X10) : 0U;
            high_x10 = (high_x10 <= UINT16_MAX - IR_ZCR_HOLD_MARGIN_X10)
                         ? (uint16_t)(high_x10 + IR_ZCR_HOLD_MARGIN_X10) : UINT16_MAX;
        }
        if (zcr_x10 < low_x10 || zcr_x10 > high_x10) {
            DBG_CRITERION("criterion-4 Z10=%lu not in [%lu,%lu] hold=%u",
                (unsigned long)zcr_x10,
                (unsigned long)low_x10,
                (unsigned long)high_x10,
                (unsigned int)warning_hold);
            return false;
        }
    }

    /* ⑤ 频率一致性：仅有效参考通道参与，弱参考信号的噪声ZCR不得否决火焰。 */
    {
        uint16_t zcr_main = zcr_to_x10(d->feat[IR_CH_MAIN].zcr_hz);
        uint16_t zcr_refa = zcr_to_x10(d->feat[IR_CH_REF_A].zcr_hz);
        uint16_t zcr_refb = zcr_to_x10(d->feat[IR_CH_REF_B].zcr_hz);
        uint16_t diff_a = (zcr_main >= zcr_refa) ? (zcr_main - zcr_refa)
                                                  : (zcr_refa - zcr_main);
        uint16_t diff_b = (zcr_main >= zcr_refb) ? (zcr_main - zcr_refb)
                                                  : (zcr_refb - zcr_main);
        bool refa_valid = reference_zcr_valid(d, IR_CH_REF_A);
        bool refb_valid = reference_zcr_valid(d, IR_CH_REF_B);
        /* 差值上限本身允许通过，仅超过2.0Hz才视为不一致。 */
        if ((refa_valid && diff_a > IR_FREQ_CONSISTENCY_X10) ||
            (refb_valid && diff_b > IR_FREQ_CONSISTENCY_X10)) {
            DBG_CRITERION("criterion-5 dZ10 main-38=%u main-50=%u REFV=%u,%u limit=%u",
                          (unsigned int)diff_a, (unsigned int)diff_b,
                          (unsigned int)refa_valid, (unsigned int)refb_valid,
                          (unsigned int)IR_FREQ_CONSISTENCY_X10);
            return false;
        }
    }

    return true;
}

static bool check_entry_criteria(IR_Detector_t *d)
{
    /* 入态要求光谱、主ZCR及所有有效参考通道的频率一致性通过。 */
    return check_spectral_criteria(d) && check_frequency_criteria(d, false);
}

/**
  @brief  清除点火包络分类上下文
  @param  d: 检测器实例
  @note   安静累计和启动沿布防状态都会清零，下一次只有主通道重新低于OFF
          满1秒，才会被视为可用于区分打火机/持续火焰的完整点火过程。
 */
static void ignition_profile_clear(IR_Detector_t *d)
{
    memset(&d->profile, 0, sizeof(d->profile));
    d->profile.state = IR_PROFILE_BYPASS;
}

/**
  @brief  BYPASS状态下累计点火前安静时间并完成新启动沿布防
  @param  d: 检测器实例
  @param  power: 当前4.5um的0.5秒滚动均方值
  @note   本函数只由BYPASS调用，不检查IR报警状态。P45低于OFF满1秒后锁存
          ARMED；随后即使先经过OFF~ON区间也不会丢失下一次ON上穿沿。
 */
static void ignition_profile_arm_if_quiet(IR_Detector_t *d, uint32_t power)
{
    if (power < d->power_off_threshold) {
        if (d->profile.low_power_accum_ms < IR_PROFILE_QUIET_MS) {  /* 小于阈值 小于布防条件 +10ms */
            uint32_t remain = IR_PROFILE_QUIET_MS -
                              d->profile.low_power_accum_ms;
            d->profile.low_power_accum_ms +=
                (remain < IR_PROCESS_STEP_MS) ? remain : IR_PROCESS_STEP_MS;
        }
        if (d->profile.low_power_accum_ms >= IR_PROFILE_QUIET_MS) { /* 大于布防条件，从bypass->armed */
            memset(&d->profile, 0, sizeof(d->profile));
            d->profile.state = IR_PROFILE_ARMED;
            DBG("T%lu PROFILE armed P=%lu OFF=%lu",
                (unsigned long)HAL_GetTick(), (unsigned long)power,
                (unsigned long)d->power_off_threshold);
        }
    } else {
        d->profile.low_power_accum_ms = 0U;
    }
}

/**
  @brief  LIGHTER/SUSTAINED终态下等待本次火源完全消退
  @param  d: 检测器实例
  @param  now: 当前毫秒tick
  @param  power: 当前4.5um的0.5秒滚动均方值
  @note   LIGHTER可由独立恢复函数单向升级为SUSTAINED，SUSTAINED不会反向降级。
          两种状态都只有在P45连续低于OFF满2秒后才确认本次火源结束并回到BYPASS；
          期间只要恢复到OFF以上就清除
          掉线计时。BYPASS还需重新累计1秒安静时间才能进入ARMED，因此旧火源
          的短暂低谷或稳定尾段不会被当成新的点火沿。
 */
static void ignition_profile_release_if_quiet(IR_Detector_t *d, uint32_t now,
                                               uint32_t power)
{
    IR_Profile_t *profile = &d->profile;

    if (power >= d->power_off_threshold) {
        profile->low_power_accum_ms = 0U;
        return;
    }

    if (profile->low_power_accum_ms < IR_PROFILE_RELEASE_MS) {
        uint32_t remain = IR_PROFILE_RELEASE_MS -
                          profile->low_power_accum_ms;
        profile->low_power_accum_ms +=
            (remain < IR_PROCESS_STEP_MS) ? remain : IR_PROCESS_STEP_MS;
    }

    if (profile->low_power_accum_ms >= IR_PROFILE_RELEASE_MS) {
#if defined(AP_ALGO_DEBUG_ENABLE)
        IR_ProfileState_t released_state = profile->state;
#endif
        memset(profile, 0, sizeof(*profile));
        profile->state = IR_PROFILE_BYPASS;
        DBG("T%lu PROFILE released %s P=%lu OFF=%lu -> BYPASS",
            (unsigned long)now, ir_profile_name(released_state),
            (unsigned long)power, (unsigned long)d->power_off_threshold);
    }
}

/**
  @brief  P45在布防后第一次跨过ON时立即启动点火包络观察
  @param  d: 检测器实例
  @param  now: 当前毫秒tick
  @param  power: 首次达到ON的4.5um滚动均方值

  该函数由独立PROFILE监视器调用，不依赖光谱、ZCR或IR状态机是否已进入
  WARNING。这样可在五判据尚未满足时捕获真实点火峰值，避免日志中点火峰值
  已经衰减数秒后才以WARNING入口作为错误起点。
 */
static void ignition_profile_start(IR_Detector_t *d, uint32_t now,
                                   uint32_t power)
{
    memset(&d->profile, 0, sizeof(d->profile));
    d->profile.phase_start_ms = now;
    d->profile.early_peak = power;
    d->profile.state = IR_PROFILE_OBSERVING;    /* 布防->观察 */

    DBG("T%lu PROFILE onset P=%lu ON=%lu -> OBS",
        (unsigned long)now, (unsigned long)power,
        (unsigned long)d->power_threshold);
}

/**
  @brief  丢弃没有进入WARNING且已经安静的短瞬态观察
  @param  d: 检测器实例
  @param  now: 当前毫秒tick
  @param  power: 当前4.5um滚动均方值
  @return true表示本次观察已取消并直接重新ARMED

  PROFILE必须在P45首次跨过ON时提前启动，才能保存真实启动峰值；但只有后续
  真正进入WARNING，该包络才具有报警过滤价值。如果整个事件没有进入WARNING，
  且P45已经连续低于OFF满1秒，则这1秒本身已经满足下一次点火沿的安静布防
  条件，因此直接OBSERVING->ARMED，不再生成LIGHTER/SUSTAINED，也不再额外
  等待终态2秒释放和BYPASS 1秒布防。
 */
static bool ignition_profile_cancel_no_warning_if_quiet(IR_Detector_t *d,
                                                         uint32_t now,
                                                         uint32_t power)
{
    IR_Profile_t *profile = &d->profile;

    if (profile->state != IR_PROFILE_OBSERVING ||
        profile->warning_seen != 0U) {
        return false;
    }

    if (power >= d->power_off_threshold) {
        profile->low_power_accum_ms = 0U;
        return false;
    }

    if (profile->low_power_accum_ms < IR_PROFILE_QUIET_MS) {
        uint32_t remain = IR_PROFILE_QUIET_MS -
                          profile->low_power_accum_ms;
        profile->low_power_accum_ms +=
            (remain < IR_PROCESS_STEP_MS) ? remain : IR_PROCESS_STEP_MS;
    }

    if (profile->low_power_accum_ms < IR_PROFILE_QUIET_MS) return false;

#if defined(AP_ALGO_DEBUG_ENABLE)
    uint32_t observe_ms = now - profile->phase_start_ms;
    uint32_t peak = profile->early_peak;
#endif
    memset(profile, 0, sizeof(*profile));
    profile->state = IR_PROFILE_ARMED;
    DBG("T%lu PROFILE canceled OBS=%lu PEAK=%lu REASON=NO_WARNING_QUIET P=%lu OFF=%lu -> ARMED",
        (unsigned long)now, (unsigned long)observe_ms,
        (unsigned long)peak, (unsigned long)power,
        (unsigned long)d->power_off_threshold);
    return true;
}

static uint32_t ignition_profile_mean(uint64_t sum, uint16_t samples)
{
    /* sum使用64位，避免高功率场景下连续累加发生32位溢出。 */
    return (samples == 0U) ? 0U : (uint32_t)(sum / samples);
}

static uint32_t ignition_profile_duty_x1000(uint16_t high_samples,
                                            uint16_t samples)
{
    /* 返回千分比整数，例如600表示60.0%，避免在10ms任务中使用浮点运算。 */
    return (samples == 0U) ? 0U
                           : ((uint32_t)high_samples * 1000U) / samples;
}

/**
  @brief  LIGHTER状态下检查后续是否已经转变为持续真实火焰
  @param  d: 检测器实例
  @param  now: 当前毫秒tick
  @param  power: 当前4.5um滚动均方值

  LIGHTER不是永久锁存结论。分类完成后按1秒窗口继续观察主通道，但均值只统计
  P45>=OFF的有效样本，低谷样本仅计入窗口总数以形成DUTY1000。由于30cm酒精盆
  点火瞬间可能出现170万级轰燃峰值，后续真实稳定火焰即使维持6万~几十万，
  相对启动峰值也可能远低于40%，因此必须保留固定5万的绝对恢复路径。同时，
  固定阈值会受距离、镜片透过率和器件增益影响，远距离真实火焰可能达不到5万，
  因此也保留恢复到启动峰值40%以上的相对路径，两条能量路径使用或关系。

  无论通过绝对路径还是相对路径，都要求有效占比>=60%，并连续通过2个1秒窗口
  后才允许LIGHTER单向升级为SUSTAINED。第一次通过只累计确认次数；任一窗口
  不通过便清零，可防止近距离打火机偶发一个窗口超过5万时被误升级。

  同时保留连续低于OFF满2秒的释放规则；因此打火机熄灭会先回BYPASS，而不会
  因为低谷样本被平均进去产生虚假的恢复结论。
  */
static void ignition_profile_recover_sustained(IR_Detector_t *d,
                                                uint32_t now,
                                                uint32_t power)
{
    IR_Profile_t *profile = &d->profile;
    uint32_t valid_mean;
    uint32_t ratio_x1000;
    uint32_t duty_x1000;
    bool absolute_pass;
    bool relative_pass;
    bool window_pass;

    ignition_profile_release_if_quiet(d, now, power);
    if (profile->state != IR_PROFILE_LIGHTER) return;

    profile->late_samples++;
    if (power >= d->power_off_threshold) {
        profile->late_sum += power;
        profile->late_high_samples++;
    }

    if ((now - profile->phase_start_ms) < IR_PROFILE_RECOVERY_WINDOW_MS) {
        return;
    }

    valid_mean = ignition_profile_mean(profile->late_sum,
                                       profile->late_high_samples);
    ratio_x1000 = (profile->early_peak == 0U) ? 0U
        : (uint32_t)(((uint64_t)valid_mean * 1000U) /
                     profile->early_peak);
    duty_x1000 = ignition_profile_duty_x1000(
        profile->late_high_samples, profile->late_samples);

    /*
     * 绝对能量和相对恢复使用或关系，避免任何单一标尺覆盖全部距离和硬件差异：
     *   - absolute_pass处理巨大轰燃峰值后仍稳定在5万以上的真实火焰；
     *   - relative_pass处理整体幅值偏小、但相对启动峰值恢复明显的真实火焰。
     * 两条路径共同受有效样本和占空比约束，再由连续窗口计数过滤偶发越线。
     */
    absolute_pass = valid_mean >= IR_PROFILE_RECOVERY_POWER_MIN;
    relative_pass = ratio_x1000 >= IR_PROFILE_DECAY_RATIO_X1000;
    window_pass = (profile->late_high_samples != 0U) &&
                  (absolute_pass || relative_pass) &&
                  (duty_x1000 >= IR_PROFILE_RECOVERY_DUTY_X1000);
    if (window_pass) {
        if (profile->recovery_pass_windows <
            IR_PROFILE_RECOVERY_CONFIRM_WINDOWS) {
            profile->recovery_pass_windows++;
        }
    } else {
        profile->recovery_pass_windows = 0U;
    }

    if (profile->recovery_pass_windows >=
        IR_PROFILE_RECOVERY_CONFIRM_WINDOWS) {
        profile->state = IR_PROFILE_SUSTAINED;
        profile->low_power_accum_ms = 0U;
        DBG("T%lu PROFILE LIGHTER -> SUSTAINED MEAN=%lu RMIN=%lu R1000=%lu ABS=%u REL=%u DUTY1000=%lu HIT=%u/%u PEAK=%lu",
            (unsigned long)now, (unsigned long)valid_mean,
            (unsigned long)IR_PROFILE_RECOVERY_POWER_MIN,
            (unsigned long)ratio_x1000,
            (unsigned int)absolute_pass, (unsigned int)relative_pass,
            (unsigned long)duty_x1000,
            (unsigned int)profile->recovery_pass_windows,
            (unsigned int)IR_PROFILE_RECOVERY_CONFIRM_WINDOWS,
            (unsigned long)profile->early_peak);
    } else {
        DBG("T%lu PROFILE recovery pending MEAN=%lu RMIN=%lu R1000=%lu ABS=%u REL=%u DUTY1000=%lu HIT=%u/%u VALID=%u/%u",
            (unsigned long)now, (unsigned long)valid_mean,
            (unsigned long)IR_PROFILE_RECOVERY_POWER_MIN,
            (unsigned long)ratio_x1000,
            (unsigned int)absolute_pass, (unsigned int)relative_pass,
            (unsigned long)duty_x1000,
            (unsigned int)profile->recovery_pass_windows,
            (unsigned int)IR_PROFILE_RECOVERY_CONFIRM_WINDOWS,
            (unsigned int)profile->late_high_samples,
            (unsigned int)profile->late_samples);
    }

    /* 每个恢复窗口独立统计，避免旧低谷无限稀释后续真实火焰的能量。 */
    profile->phase_start_ms = now;
    profile->late_sum = 0U;
    profile->late_samples = 0U;
    profile->late_high_samples = 0U;
    if (profile->state == IR_PROFILE_SUSTAINED) {
        profile->recovery_pass_windows = 0U;
    }
}

/**
  @brief  使用当前已收集数据结束本次点火包络观察
  @param  d: 检测器实例
  @param  now: 当前毫秒tick
  @param  fire_ready: 0=峰值窗+最大稳定窗到期，1=WARNING已具备FIRE条件而提前收口

  仅当本次观察已经进入过WARNING时才允许提交分类。正常情况下峰值窗1.5秒加
  稳定窗最多1.5秒；如果WARNING证据先达到confirm_ms，则必须在
  进入FIRE前调用，保证日志和报警顺序始终是PROFILE result在前、FIRE在后。

  提前收口使用0~1.5秒峰值窗及其后已经实际取得的全部稳定段样本，不要求补满
  1.5秒稳定窗。高灵敏度下FIRE_READY可能早于稳定窗，此时late_samples为0，
  无法可靠识别快速衰减，按SUSTAINED放行报警；不会为了分类延迟正常报警。
  */
static void ignition_profile_finish(IR_Detector_t *d, uint32_t now,
                                    uint8_t fire_ready)
{
    IR_Profile_t *profile = &d->profile;
#if defined(AP_ALGO_DEBUG_ENABLE)
    uint32_t observe_ms = now - profile->phase_start_ms;
    uint32_t late_ms = (uint32_t)profile->late_samples *
                       IR_PROCESS_STEP_MS;
#endif
    uint32_t late_mean;
    uint32_t ratio_x1000;
#if defined(AP_ALGO_DEBUG_ENABLE)
    uint32_t duty_x1000;
#endif

    /*
     * 没有进入过WARNING的包络只属于预采集数据，不能提交成LIGHTER或
     * SUSTAINED终态，否则一次无效点火也会制造数秒的分类锁存时间。
     */
    if (profile->state != IR_PROFILE_OBSERVING ||
        profile->warning_seen == 0U) return;

    /*
     * LATE只代表仍高于OFF的有效火焰能量。低于OFF的样本不进入均值，
     * 避免酒精火焰短时低谷把后段均值压成约2000并被误判为快速衰减。
     */
    late_mean = ignition_profile_mean(profile->late_sum,
                                      profile->late_high_samples);
    ratio_x1000 = (profile->early_peak == 0U) ? 1000U
        : (uint32_t)(((uint64_t)late_mean * 1000U) /
                     profile->early_peak);
#if defined(AP_ALGO_DEBUG_ENABLE)
    duty_x1000 = ignition_profile_duty_x1000(
        profile->late_high_samples, profile->late_samples);
#endif

    /*
     * 打火机分类必须同时满足固定峰值门槛（当前10万）、至少一个OFF以上有效
     * 后段样本，以及LATE/PEAK<40%。该门槛不是火焰进场阈值，也不随灵敏度
     * 变化。无有效后段数据时不能用低于OFF的样本证明衰减，
     * 按SUSTAINED放行，优先避免真实火焰漏报。
     */
    if (profile->early_peak >= IR_PROFILE_LIGHTER_PEAK_MIN &&
        profile->late_high_samples != 0U &&
        ratio_x1000 < IR_PROFILE_DECAY_RATIO_X1000) {
        profile->state = IR_PROFILE_LIGHTER;
    } else {
        profile->state = IR_PROFILE_SUSTAINED;
    }

#if defined(AP_ALGO_DEBUG_ENABLE)
    DBG("T%lu PROFILE result=%s PEAK=%lu PMIN=%lu LATE=%lu R1000=%lu DUTY1000=%lu VALID=%u/%u LATE_MS=%lu OBS=%lu MODE=%s",
        (unsigned long)now, ir_profile_name(profile->state),
        (unsigned long)profile->early_peak,
        (unsigned long)IR_PROFILE_LIGHTER_PEAK_MIN,
        (unsigned long)late_mean,
        (unsigned long)ratio_x1000, (unsigned long)duty_x1000,
        (unsigned int)profile->late_high_samples,
        (unsigned int)profile->late_samples,
        (unsigned long)late_ms,
        (unsigned long)observe_ms,
        (fire_ready != 0U) ? "FIRE_READY" : "FULL");
#else
    (void)now;
    (void)fire_ready;
#endif

    /*
     * LIGHTER保留early_peak作为后续恢复基准，并从分类完成时启动新的1秒恢复窗；
     * 它只能升级为SUSTAINED。SUSTAINED不会反向降级，两者仍使用2秒掉线释放。
     */
    profile->low_power_accum_ms = 0U;
    profile->phase_start_ms = now;
    profile->late_sum = 0U;
    profile->late_samples = 0U;
    profile->late_high_samples = 0U;
}

/**
  @brief  独立更新主通道点火包络分类器
  @param  d: 检测器实例
  @param  now: 当前毫秒tick

  初次观察分为互不重叠的两个区间：
    - 0~1.5秒：持续记录从首次跨过ON开始的early_peak，覆盖打火机完整起峰；
    - 1.5~3.0秒：最多累计1.5秒late_mean和P45>=OFF的late_duty。

  P45本身已经是0.5秒滚动均方值，因此early_peak不是ADC单点毛刺。按照现场
  特征，仅当PEAK达到固定门槛、有OFF以上有效后段样本且late/peak<40%时标记为
  LIGHTER，其他情况标记为SUSTAINED。低于OFF的样本不参与late均值，只进入
  late_duty分母。若WARNING先达到确认时间，则在FIRE迁移前使用已有数据提前
  结束稳定窗，采到多少稳定样本就使用多少。LIGHTER后续按1秒窗口检查有效能量，满足恢复幅度与占空比后只允许
  单向升级为SUSTAINED；两种状态均在P45低于OFF满2秒后回BYPASS重新布防。
 */
static void ignition_profile_update(IR_Detector_t *d, uint32_t now)
{
    /* 关闭后不维护观察/恢复状态，红外五判据状态机仍独立运行。 */
    if (d->profile_enabled == 0U) return;

    IR_Profile_t *profile = &d->profile; /* 独立包络状态，不等同于IR报警状态 */
    uint32_t power = d->feat[IR_CH_MAIN].power_x1000; /* 当前P45：0.5秒滚动均方值 */
    uint32_t elapsed = now - profile->phase_start_ms; /* 当前阶段已运行时间，单位ms */

    if (profile->state == IR_PROFILE_ARMED) {
        /* 只认布防后的第一次ON上穿沿，该时刻作为最长3秒观察窗口的t=0。 */
        if (power >= d->power_threshold) {
            ignition_profile_start(d, now, power);  /* 从布防到观察 */
        }
        return;
    }

    if (profile->state == IR_PROFILE_OBSERVING) {
        /*
         * PROFILE在本周期状态机迁移之前运行，因此通常会在进入WARNING后的
         * 下一次10ms任务看到该状态。warning_seen一旦置位便保持到分类完成，
         * 后续WARNING短时抖动不会把有效候选重新当成无效点击。
         */
        if (d->state == IR_STATE_WARNING || d->state == IR_STATE_FIRE) {
            profile->warning_seen = 1U;
        }

        /*
         * 未进入WARNING的短瞬态一旦安静满1秒便直接重新ARMED。该检查必须
         * 早于固定观察阶段处理，否则仍会先输出无意义的分类结果。
         */
        if (ignition_profile_cancel_no_warning_if_quiet(d, now, power)) {
            return;
        }

        /*
         * 阶段1，t=[0,1.5s)：只更新P45最大值。即使P45首次超过ON后继续升高，
         * early_peak也会跟随更新，因此不会把第一次过阈值值误当成峰值。
         */
        if (elapsed < IR_PROFILE_PEAK_WINDOW_MS) {
            if (power > profile->early_peak) {
                profile->early_peak = power;    /* 动态更新峰值 */
            }
            return;
        }

        /*
         * 阶段2，t=[1.5s,3.0s)：峰值已经锁定，最多统计1.5秒后期稳定能量。
         * late_samples记录窗口总样本数；只有P45>=OFF时才写入late_sum并增加
         * late_high_samples。LATE使用有效样本数作分母，低于OFF的低谷不会
         * 参与均值；总样本数仍保留用于输出有效时间占比DUTY1000。
         * WARNING若在本区间内先满足FIRE条件，由ignition_profile_allows_fire()
         * 立即使用当前已采样本收口，不会等待IR_PROFILE_OBSERVE_END_MS。
         */
        if (elapsed >= IR_PROFILE_LATE_START_MS &&
            elapsed < IR_PROFILE_OBSERVE_END_MS) {
            profile->late_samples++;
            if (power >= d->power_off_threshold) {
                profile->late_sum += power;
                profile->late_high_samples++;
            }
            return;
        }

        /*
         * 高功率但尚未进入WARNING时保留已采集摘要，不提前生成终态。如果
         * 后续判据恢复并进入WARNING，可直接使用现有峰值/后段数据；如果先
         * 安静满1秒，则由上方无WARNING取消路径丢弃并重新布防。
         */
        if (elapsed >= IR_PROFILE_OBSERVE_END_MS &&
            profile->warning_seen != 0U) {
            ignition_profile_finish(d, now, 0U);
        }
        return;
    }

    if (profile->state == IR_PROFILE_LIGHTER) {
        /*
         * LIGHTER允许后续真实火焰使能量持续恢复后单向升级；该路径不会清除或
         * 暂停WARNING积分，升级完成的同一周期即可继续检查FIRE迁移。
         */
        ignition_profile_recover_sustained(d, now, power);
        return;
    }

    if (profile->state == IR_PROFILE_SUSTAINED) {
        /* SUSTAINED为单向最终结论，只等待本次火源连续低于OFF满2秒后释放。 */
        ignition_profile_release_if_quiet(d, now, power);
        return;
    }

    /* BYPASS独立等待安静布防，不依赖当前IR处于IDLE、WARNING还是FIRE。 */
    ignition_profile_arm_if_quiet(d, power);
}

/**
  @brief  在正常IR判据当周期有效时接纳缺少冷启动沿的持续火焰
  @param  d: 检测器实例
  @param  now: 当前毫秒tick
  @param  power: 当前4.5um滚动均方值

  设备上电时火焰可能已经存在，或旧事件释放后火焰仍在波动，PROFILE因无法取得
  P45<OFF连续1秒的安静背景而停留在BYPASS。该场景没有可用于打火机分类的完整
  启动沿，因此当正常IR状态机在本周期确认P45>=ON且光谱/频率判据全部通过时，
  直接把PROFILE接纳为SUSTAINED。

  本函数只能从正常IR判据通过的分支调用，不能仅根据上周期遗留的WARNING状态
  调用。这样LIGHTER刚释放而WARNING尚未完成掉线超时时，即使PROFILE已是BYPASS，
  P45很低或当前判据无效也不会误触发热启动。
  */
static void ignition_profile_accept_hot_start(IR_Detector_t *d,
                                               uint32_t now,
                                               uint32_t power)
{
    if (d->profile_enabled == 0U) return;

    IR_Profile_t *profile = &d->profile;

    if (profile->state != IR_PROFILE_BYPASS ||
        power < d->power_threshold) {
        return;
    }

    memset(profile, 0, sizeof(*profile));
    profile->state = IR_PROFILE_SUSTAINED;
    profile->phase_start_ms = now;
    DBG("T%lu PROFILE BYPASS -> SUSTAINED REASON=HOT_START IR=%s P=%lu",
        (unsigned long)now, ir_state_name(d->state),
        (unsigned long)power);
}

/**
  @brief  在IR准备进入FIRE时完成PROFILE分类并检查本次点火沿是否允许报警
  @param  d: 检测器实例
  @param  now: 当前毫秒tick
  @retval true: PROFILE为SUSTAINED，或当前没有可用启动沿分类，允许进入FIRE
  @retval false: PROFILE已判为LIGHTER，本次WARNING保持但不得进入FIRE
  @note   OBS会在本函数内先完成分类，因此不会再出现FIRE日志早于PROFILE result。
          BYPASS/ARMED表示没有足够数据完成分类，按高灵敏度漏报优先策略放行。
 */
static bool ignition_profile_allows_fire(IR_Detector_t *d, uint32_t now)
{
    /* 包络关闭用于允许打火机演示，不能改变正常功率/光谱/频率确认条件。 */
    if (d->profile_enabled == 0U) return true;

    if (d->profile.state == IR_PROFILE_OBSERVING) {
        /*
         * 本函数只会在WARNING证据已满足、准备进入FIRE时调用。显式置位可
         * 覆盖PROFILE先于状态机运行的边界周期，保证最终分类不会因时序漏标。
         */
        d->profile.warning_seen = 1U;
        ignition_profile_finish(d, now, 1U);
    }

    if (d->profile.state == IR_PROFILE_LIGHTER) {
        DBG_CRITERION("PROFILE LIGHTER blocks FIRE ACC=%lu/%lu",
                      (unsigned long)d->valid_accum_ms,
                      (unsigned long)d->confirm_ms);
        return false;
    }

    return true;
}

/**
  @brief  取消本次WARNING并清理所有确认积分、迟滞和drop上下文
  @param  d: 检测器实例
  @param  now: 当前毫秒tick
  @note   不清历史窗口、滤波器和已提取特征。若PROFILE仍在OBS，说明当前报警
          候选在分类完成前已经连续掉线超时，该观察结果已无报警意义，立即取消
          并回BYPASS；已经完成的LIGHTER/SUSTAINED终态仍按2秒掉线规则释放。
 */
static void warning_reset(IR_Detector_t *d, uint32_t now)
{
    if (d->profile.state == IR_PROFILE_OBSERVING) {
        DBG("T%lu PROFILE canceled OBS=%lu REASON=WARNING_RESET -> BYPASS",
            (unsigned long)now,
            (unsigned long)(now - d->profile.phase_start_ms));
        ignition_profile_clear(d);
    }

    d->state = IR_STATE_IDLE;
    d->valid_accum_ms = 0U;
    d->power_drop_start_ms = 0U;
    d->criteria_drop_start_ms = 0U;
    d->power_active = 0U;
    d->power_drop_active = 0U;
    d->criteria_drop_active = 0U;
}

/**
  @brief  在WARNING全部维持判据通过时累计一个10ms有效证据周期
  @param  d: 检测器实例
  @note   积分在confirm_ms处饱和，状态迁移仍由AP_IR_Process统一执行。
 */
static void warning_accumulate(IR_Detector_t *d)
{
    /* 积分只需达到确认时间，饱和可避免长时间WARNING发生无符号溢出。 */
    if (d->valid_accum_ms < d->confirm_ms) {
        uint32_t remain = d->confirm_ms - d->valid_accum_ms;
        d->valid_accum_ms += (remain < IR_PROCESS_STEP_MS) ? remain : IR_PROCESS_STEP_MS;
    }
}

static void warning_decay(IR_Detector_t *d)
{
    d->valid_accum_ms = (d->valid_accum_ms > IR_PROCESS_STEP_MS)
                      ? (d->valid_accum_ms - IR_PROCESS_STEP_MS) : 0U;
}

/**
  @brief  检查WARNING期间4.5um主通道功率是否可继续作为有效证据
  @param  d: 检测器实例
  @param  now: 当前毫秒tick，用于连续低功率drop计时
  @retval true: 功率达到开启阈值，或已锁存后仍高于40%迟滞下限
  @retval false: 功率处于drop宽限期，或连续低功率2秒后已复位到IDLE
  @note   本函数只维护功率迟滞/drop状态，不修改确认积分；宽限期内积分暂停。
 */
static bool warning_power_available(IR_Detector_t *d, uint32_t now)
{
    uint32_t power = d->feat[IR_CH_MAIN].power_x1000;

    if (power >= d->power_threshold) {
#if defined(AP_ALGO_DEBUG_ENABLE)
        if (d->power_drop_active != 0U) {
            DBG("T%lu POWER recovered P=%lu >= ON=%lu ACC=%lu",
                (unsigned long)now, (unsigned long)power,
                (unsigned long)d->power_threshold,
                (unsigned long)d->valid_accum_ms);
        }
#endif
        d->power_active = 1U;
        d->power_drop_active = 0U;
        return true;
    }

    if (d->power_active != 0U && power >= d->power_off_threshold) {
        /* 迟滞区保持有效，避免功率在开启阈值附近轻微波动造成状态抖动。 */
#if defined(AP_ALGO_DEBUG_ENABLE)
        if (d->power_drop_active != 0U) {
            DBG("T%lu POWER recovered in hysteresis P=%lu OFF=%lu ON=%lu ACC=%lu",
                (unsigned long)now, (unsigned long)power,
                (unsigned long)d->power_off_threshold,
                (unsigned long)d->power_threshold,
                (unsigned long)d->valid_accum_ms);
        }
#endif
        d->power_drop_active = 0U;
        return true;
    }

    /* 低于派生迟滞下限时允许最多2秒连续掉线，积分由WARNING统一回退。 */
    if (d->power_drop_active == 0U) {
        d->power_drop_active = 1U;
        d->power_drop_start_ms = now;
        DBG("T%lu POWER dropout P=%lu < OFF=%lu ACC=%lu",
            (unsigned long)now, (unsigned long)power,
            (unsigned long)d->power_off_threshold,
            (unsigned long)d->valid_accum_ms);
    } else if ((now - d->power_drop_start_ms) >= IR_POWER_DROPOUT_MS) {
        DBG("T%lu POWER dropout timeout P=%lu elapsed=%lu -> IDLE",
            (unsigned long)now, (unsigned long)power,
            (unsigned long)(now - d->power_drop_start_ms));
        warning_reset(d, now);
    }
    return false;
}

/**
  @brief  检查WARNING期间光谱比、主ZCR及有效参考通道频率一致性
  @param  d: 检测器实例
  @param  now: 当前毫秒tick，用于连续判据失效drop计时
  @retval true: 本周期全部可用判据通过，并已累计一个10ms确认周期
  @retval false: 判据处于drop宽限期，或连续失效2秒后已复位到IDLE
  @note   短暂失败仅暂停积分，不扣除历史有效证据；恢复后从原积分继续。
 */
static bool warning_criteria_available(IR_Detector_t *d, uint32_t now)
{
    bool criteria_ok = check_spectral_criteria(d) && check_frequency_criteria(d, true);

    if (criteria_ok) {
#if defined(AP_ALGO_DEBUG_ENABLE)
        if (d->criteria_drop_active != 0U) {
            DBG("T%lu CRITERIA recovered ACC=%lu",
                (unsigned long)now, (unsigned long)d->valid_accum_ms);
        }
#endif
        d->criteria_drop_start_ms = 0U;
        d->criteria_drop_active = 0U;
        warning_accumulate(d);
        return true;
    }

    /* 单个2秒滚动窗口异常回退积分，连续失效才撤销WARNING。 */
    warning_decay(d);
    if (d->criteria_drop_active == 0U) {
        d->criteria_drop_active = 1U;
        d->criteria_drop_start_ms = now;
        DBG("T%lu CRITERIA dropout ACC=%lu",
            (unsigned long)now, (unsigned long)d->valid_accum_ms);
    } else if ((now - d->criteria_drop_start_ms) >= IR_CRITERIA_DROPOUT_MS) {
        DBG("T%lu CRITERIA dropout timeout elapsed=%lu -> IDLE",
            (unsigned long)now,
            (unsigned long)(now - d->criteria_drop_start_ms));
        warning_reset(d, now);
    }
    return false;
}

static void AP_IR_Process(uint32_t now)
{
    /* 最终判定依赖2秒ZCR窗口；DC/功率仍只使用其中最近50点。 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        if (s_ir.history[ch].count < IR_ZCR_WINDOW_SIZE) return;
    }
    /* ZCR必须使用EEPROM中经过校验的固定死区。 */
    if (s_ir.dead_zone_ready == 0U) return;

#if defined(AP_ALGO_DEBUG_ENABLE)
    if (s_debug_window_ready == 0U) {
        s_debug_window_ready = 1U;
        DBG("T%lu WINDOW ready DC/P=%u ZCR=%u samples",
            (unsigned long)now,
            (unsigned int)IR_DC_POWER_WINDOW_SIZE,
            (unsigned int)IR_ZCR_WINDOW_SIZE);
    }
#endif

    /* 工作缓冲 (复用, 最大通道点数) */
    int32_t work[IR_HISTORY_SIZE];

    /* ================================================================ */
    /*  逐通道: 50点DC/功率与200点ZCR分别计算，避免长ZCR窗口改变功率标定。 */
    /* ================================================================ */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        history_to_workbuf(&s_ir.history[ch], work, IR_DC_POWER_WINDOW_SIZE);
        remove_dc(work, IR_DC_POWER_WINDOW_SIZE);
        s_ir.feat[ch].power_x1000 = calc_mean_square(work, IR_DC_POWER_WINDOW_SIZE);

        history_to_workbuf(&s_ir.history[ch], work, IR_ZCR_WINDOW_SIZE);
        remove_dc(work, IR_ZCR_WINDOW_SIZE);
        s_ir.feat[ch].zcr_hz = calc_zcr(work, IR_ZCR_WINDOW_SIZE,
                                       IR_SAMPLE_RATE_HZ, s_ir.zcr_dead_zone[ch]);
    }

    /* ================================================================ */
    /*  计算光谱比，参考通道无有效功率时判定为无效                         */
    /* ================================================================ */
    {
        uint32_t p_main = s_ir.feat[IR_CH_MAIN].power_x1000;

        /* 比例直接使用未缩放均方值，避免参考通道因显示缩放截断为假零。 */
        s_ir.ratio_45_38_x1000 = calc_ratio_x1000(
            p_main, s_ir.feat[IR_CH_REF_A].power_x1000);
        s_ir.ratio_45_50_x1000 = calc_ratio_x1000(
            p_main, s_ir.feat[IR_CH_REF_B].power_x1000);
    }

    /*
     * 包络监视必须早于IR状态机执行，并且不能等待五判据全部通过。
     * 它只依赖已经计算完成的P45，可准确捕获第一次跨过当前等级ON后的启动最高峰。
     */
    ignition_profile_update(&s_ir, now);

#if defined(AP_ALGO_DEBUG_ENABLE)
    debug_log_features(now);
#endif

    /* ================================================================ */
    /*  状态机                                                           */
    /* ================================================================ */
    {
        IR_DetectorState_t old = s_ir.state;

        switch (old) {

            case IR_STATE_IDLE: {
                uint32_t power = s_ir.feat[IR_CH_MAIN].power_x1000;

                /* 进入WARNING要求主功率、光谱、主ZCR及有效参考频率一致性通过。 */
                if (power >= s_ir.power_threshold && check_entry_criteria(&s_ir)) {
                    s_ir.state = IR_STATE_WARNING;
                    /*
                     * 若PROFILE因没有安静背景仍在BYPASS，本次新鲜的五判据进场
                     * 证据可将其作为热启动持续火焰接纳，不需要伪造点火包络。
                     */
                    ignition_profile_accept_hot_start(&s_ir, now, power);
                    /*
                     * PROFILE本周期已经先执行，必须在状态迁移点同步标记，
                     * 防止下一周期前的边界处理误把有效候选当成无WARNING瞬态。
                     */
                    if (s_ir.profile.state == IR_PROFILE_OBSERVING) {
                        s_ir.profile.warning_seen = 1U;
                    }
                    s_ir.power_active = 1U;
                    s_ir.valid_accum_ms = 0U;
                    s_ir.power_drop_start_ms = 0U;
                    s_ir.criteria_drop_start_ms = 0U;
                    s_ir.power_drop_active = 0U;
                    s_ir.criteria_drop_active = 0U;
                }
                break;
            }

            case IR_STATE_WARNING:
                if (warning_power_available(&s_ir, now)) {
                    bool criteria_ok = warning_criteria_available(&s_ir, now);

                    /*
                     * IR证据积分只由功率、光谱和频率判据决定。PROFILE可能同时
                     * 处于ARMED/OBS/LIGHTER/SUSTAINED，但不会暂停或清除该积分。
                     * 若旧分类已释放到BYPASS，必须等当前功率重新达到ON且本周期
                     * 判据通过后才接纳热启动，不能读取遗留WARNING状态直接迁移。
                     */
                    if (criteria_ok) {
                        ignition_profile_accept_hot_start(
                            &s_ir, now,
                            s_ir.feat[IR_CH_MAIN].power_x1000);
                    }

                    if (criteria_ok &&
                        s_ir.valid_accum_ms >= s_ir.confirm_ms &&
                        ignition_profile_allows_fire(&s_ir, now)) {
                        s_ir.state = IR_STATE_FIRE;
                        s_ir.fire_start_ms = now;
                        s_ir.power_drop_start_ms = 0U;
                        s_ir.criteria_drop_start_ms = 0U;
                        s_ir.power_drop_active = 0U;
                        s_ir.criteria_drop_active = 0U;
                    }
                } else if (s_ir.state == IR_STATE_WARNING) {    /*  && (!ignition_profile_allows_fire(&s_ir, now)) */
                    /* 功率drop宽限期回退积分，连续低功率2秒仍由功率函数清回IDLE。 */
                    warning_decay(&s_ir);
                    s_ir.criteria_drop_start_ms = 0U;
                    s_ir.criteria_drop_active = 0U;
                }
                break;

            case IR_STATE_FIRE:
                /* FIRE期间不再检查功率/频率等条件，只允许固定超时消警。 */
                if ((now - s_ir.fire_start_ms) >= AP_FIRE_AUTO_CLEAR_MS && AP_UV_GetState() == UV_STATE_IDLE) {
                    DBG("T%lu FIRE timeout elapsed=%lu -> IDLE",
                        (unsigned long)now,
                        (unsigned long)(now - s_ir.fire_start_ms));
                    warning_reset(&s_ir, now);
                    s_ir.fire_start_ms = 0U;
                }
                break;

            default:
                s_ir.state = IR_STATE_IDLE;
                break;
        }

        if (s_ir.state != old) {
            DBG("T%lu STATE %s -> %s P=%lu ACC=%lu/%lu",
                (unsigned long)now,
                ir_state_name(old), ir_state_name(s_ir.state),
                (unsigned long)s_ir.feat[IR_CH_MAIN].power_x1000,
                (unsigned long)s_ir.valid_accum_ms,
                (unsigned long)s_ir.confirm_ms);
        }
    }
}

/* ========================================================================== */
/*                        公有 API 实现                                        */
/* ========================================================================== */

void AP_IR_Init(void)
{
    memset(&s_ir, 0, sizeof(s_ir));
    memset(&s_zcr_cal, 0, sizeof(s_zcr_cal));
#if defined(AP_ALGO_DEBUG_ENABLE)
    s_debug_last_fail_ms = 0U;
    s_debug_last_snapshot_ms = 0U;
    s_debug_window_ready = 0U;
#endif
    memset(&s_ir_test, 0, sizeof(s_ir_test));
    s_ir_test.window_ms = IR_TEST_AVG_WINDOW_MS_DEFAULT;
    test_stats_reset(HAL_GetTick());
    s_ir.state = IR_STATE_IDLE;
    s_ir.feed_pending = 0;
    s_ir.profile_enabled = 1U; /* 默认保持正式算法行为，随后由系统EEPROM配置覆盖。 */
    ignition_profile_clear(&s_ir);

    /* 从EEPROM加载功率范围和固定判据，再计算当前等级运行参数。 */
    const AP_EEPROM_IR_Param_t *p = AP_EEPROM_IR_Get();
    s_ir.level = (p->sensitivity < IR_SENS_LEVELS) ? (uint8_t)p->sensitivity
                                                    : (uint8_t)(IR_SENS_LEVELS - 1);
    AP_IR_SetConfig(p->power_min, p->power_max,
                    p->r38_threshold, p->r50_threshold,
                    (uint32_t)p->freq_low_x10, (uint32_t)p->freq_high_x10,
                    p->cfm_min, p->cfm_max);

    /* 上电直接使用EEPROM固定死区，不再使用启动阶段ADC数据自动标定。 */
    for (uint32_t ch = 0U; ch < IR_CH_NUM; ch++) {
        s_ir.zcr_dead_zone[ch] = (uint16_t)p->zcr_dead_zone[ch];
    }
    s_ir.dead_zone_ready = 1U;

    DBG("Init level=%u DC/P=%u ZCR=%u DZ=%u,%u,%u P_ON=%lu P_OFF=%lu DROP=%u CFM=%lu",
        s_ir.level,
        (unsigned int)IR_DC_POWER_WINDOW_SIZE,
        (unsigned int)IR_ZCR_WINDOW_SIZE,
        (unsigned int)s_ir.zcr_dead_zone[0],
        (unsigned int)s_ir.zcr_dead_zone[1],
        (unsigned int)s_ir.zcr_dead_zone[2],
        (unsigned long)s_ir.power_threshold,
        (unsigned long)s_ir.power_off_threshold,
        (unsigned int)IR_POWER_DROPOUT_MS,
        (unsigned long)s_ir.confirm_ms);
}

/* ========================================================================== */

void AP_IR_FeedIsr(void)
{
    s_ir.feed_pending = 1;
}

/* ========================================================================== */

static bool take_feed_request(void)
{
    uint32_t primask = __get_PRIMASK();
    bool pending;

    /* 原子消费ISR标志，避免读取和清零之间到来的10ms请求被覆盖。 */
    __disable_irq();
    pending = (s_ir.feed_pending != 0U);
    s_ir.feed_pending = 0;
    __set_PRIMASK(primask);

    return pending;
}

static void feed_latest_sample(void)
{
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        history_push(ch, AP_ADC_GetLatest(ch));
    }
}

/* ========================================================================== */

void AP_IR_Feed(void)
{
    if (!take_feed_request()) return;
    feed_latest_sample();
    /* 测试模式可只Feed不运行状态机，手动标定仍由主循环增量推进。 */
    process_zcr_calibration(HAL_GetTick());
}

void AP_IR_Task(void)
{
    if (!take_feed_request()) return;
    feed_latest_sample();
    uint32_t now = HAL_GetTick();
    process_zcr_calibration(now);
    AP_IR_Process(now);
}

/* ========================================================================== */

/* ========================================================================== */
/*                    灵敏度/参数配置                                          */
/* ========================================================================== */

void AP_IR_SetConfig(uint32_t power_min, uint32_t power_max,
                     uint32_t r38_threshold, uint32_t r50_threshold,
                     uint32_t freq_low_x10, uint32_t freq_high_x10,
                     uint32_t cfm_min, uint32_t cfm_max)
{
    s_ir.power_min = power_min;
    s_ir.power_max = power_max;
    s_ir.ratio_38_threshold = r38_threshold;
    s_ir.ratio_50_threshold = r50_threshold;
    s_ir.freq_low_x10   = (uint16_t)freq_low_x10;
    s_ir.freq_high_x10  = (uint16_t)freq_high_x10;
    s_ir.cfm_min      = cfm_min;
    s_ir.cfm_max      = cfm_max;

    /* 配置改变后立即按当前等级重算运行值，避免RAM保留旧阈值。 */
    AP_IR_SetLevel(s_ir.level);
}

void AP_IR_GetConfig(uint32_t *power_min, uint32_t *power_max,
                     uint32_t *r38_threshold, uint32_t *r50_threshold,
                     uint32_t *freq_low_x10, uint32_t *freq_high_x10,
                     uint32_t *cfm_min, uint32_t *cfm_max)
{
    if (power_min) *power_min = s_ir.power_min;
    if (power_max) *power_max = s_ir.power_max;
    if (r38_threshold) *r38_threshold = s_ir.ratio_38_threshold;
    if (r50_threshold) *r50_threshold = s_ir.ratio_50_threshold;
    if (freq_low_x10) *freq_low_x10 = s_ir.freq_low_x10;
    if (freq_high_x10)*freq_high_x10= s_ir.freq_high_x10;
    if (cfm_min)      *cfm_min      = s_ir.cfm_min;
    if (cfm_max)      *cfm_max      = s_ir.cfm_max;
}

void AP_IR_SetLevel(uint8_t level)
{
    s_ir.level = (level >= IR_SENS_LEVELS) ? (IR_SENS_LEVELS - 1) : level;
    uint32_t lv = s_ir.level;

    /*
     * 等级0最灵敏，使用较低进场阈值；等级9最迟钝，使用较高进场阈值。
     * OFF始终从当前ON按40%派生，保证所有等级具有一致的迟滞比例。
     */
    s_ir.power_threshold =
        lerp_u32(s_ir.power_min, s_ir.power_max, lv, IR_SENS_LEVELS);
    s_ir.power_off_threshold =
        (uint32_t)((uint64_t)s_ir.power_threshold *
                   IR_POWER_OFF_PERCENT / 100U);
    if (s_ir.power_off_threshold == 0U) s_ir.power_off_threshold = 1U;

    s_ir.ratio_38_thr_x1000 = s_ir.ratio_38_threshold;
    s_ir.ratio_50_thr_x1000 = s_ir.ratio_50_threshold;
    s_ir.confirm_ms         = lerp_u32(s_ir.cfm_min, s_ir.cfm_max, lv, IR_SENS_LEVELS);

    DBG("SetLevel %u: pwr=%lu off=%lu range=%lu..%lu r38=%lu r50=%lu cfm=%lu",
        s_ir.level,
        (unsigned long)s_ir.power_threshold,
        (unsigned long)s_ir.power_off_threshold,
        (unsigned long)s_ir.power_min,
        (unsigned long)s_ir.power_max,
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
    if (pwr)      *pwr      = s_ir.power_threshold;
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

void AP_IR_SetProfileEnabled(uint8_t enabled)
{
    uint8_t next = (enabled != 0U) ? 1U : 0U;
    if (s_ir.profile_enabled == next) return;

    s_ir.profile_enabled = next;
    /*
     * 动态切换时丢弃旧包络结论。关闭后立即解除LIGHTER阻断；重新开启时
     * 必须重新经过安静布防或热启动接纳，不能复用关闭前的峰值和稳定窗。
     */
    ignition_profile_clear(&s_ir);
    DBG("PROFILE enabled=%u context reset", (unsigned int)next);
}

uint8_t AP_IR_GetProfileEnabled(void)
{
    return s_ir.profile_enabled;
}

void AP_IR_Reset(void)
{
    s_ir.state = IR_STATE_IDLE;
    s_ir.fire_start_ms = 0;
    s_ir.valid_accum_ms = 0;
    s_ir.power_drop_start_ms = 0;
    s_ir.criteria_drop_start_ms = 0;
    s_ir.power_active = 0;
    s_ir.power_drop_active = 0;
    s_ir.criteria_drop_active = 0;
    ignition_profile_clear(&s_ir);
    /* 复位检测状态时取消未完成标定，但保留EEPROM固定死区。 */
    AP_IR_CancelZcrCalibration();
    /* 清空历史窗口 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        s_ir.history[ch].head  = 0;
        s_ir.history[ch].count = 0;
        memset(&s_ir.iir[ch], 0, sizeof(s_ir.iir[ch]));
    }
#if defined(AP_ALGO_DEBUG_ENABLE)
    /* 复位后重新报告窗口就绪，便于确认算法何时恢复有效判定。 */
    s_debug_window_ready = 0U;
    s_debug_last_fail_ms = 0U;
    s_debug_last_snapshot_ms = 0U;
#endif
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

uint8_t AP_IR_GetZcrDeadZone(uint16_t dead_zone[3])
{
    if (dead_zone != NULL) {
        for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
            dead_zone[ch] = s_ir.zcr_dead_zone[ch];
        }
    }
    return s_ir.dead_zone_ready;
}

int AP_IR_StartZcrCalibration(void)
{
    if ((s_zcr_cal.state == AP_IR_ZCR_CAL_WARMUP) ||
        (s_zcr_cal.state == AP_IR_ZCR_CAL_COLLECTING)) {
        return -1;
    }
    /* 标定只允许从无火警状态启动，防止明显火焰数据污染本底。 */
    if (s_ir.state != IR_STATE_IDLE) return -2;

    memset(&s_zcr_cal, 0, sizeof(s_zcr_cal));
    s_zcr_cal.state = AP_IR_ZCR_CAL_WARMUP;
    s_zcr_cal.phase_start_ms = HAL_GetTick();
    DBG("ZCR_CAL started: warmup=%lu ms windows=%u x %u samples",
        (unsigned long)IR_ZCR_CAL_WARMUP_MS,
        (unsigned int)IR_ZCR_CAL_WINDOW_COUNT,
        (unsigned int)IR_ZCR_CAL_WINDOW_SAMPLES);
    return 0;
}

void AP_IR_CancelZcrCalibration(void)
{
    /* 取消只清标定上下文，不改变当前生效死区和EEPROM。 */
    memset(&s_zcr_cal, 0, sizeof(s_zcr_cal));
    s_zcr_cal.state = AP_IR_ZCR_CAL_IDLE;
}

void AP_IR_GetZcrCalibrationStatus(AP_IR_ZcrCalStatus_t *status)
{
    if (status == NULL) return;

    memset(status, 0, sizeof(*status));
    status->state = s_zcr_cal.state;
    status->windows_collected = s_zcr_cal.windows_collected;
    status->windows_total = IR_ZCR_CAL_WINDOW_COUNT;
    for (uint32_t ch = 0U; ch < IR_CH_NUM; ch++) {
        status->dead_zone[ch] = s_ir.zcr_dead_zone[ch];
    }

    /* remaining_ms给出到全部窗口完成的估计时间，便于命令行观察进度。 */
    const uint32_t window_ms = IR_ZCR_CAL_WINDOW_SAMPLES * 1000U /
                               (uint32_t)IR_SAMPLE_RATE_HZ;
    uint32_t now = HAL_GetTick();
    if (s_zcr_cal.state == AP_IR_ZCR_CAL_WARMUP) {
        uint32_t elapsed = now - s_zcr_cal.phase_start_ms;
        uint32_t warmup_left = (elapsed < IR_ZCR_CAL_WARMUP_MS) ?
                               (IR_ZCR_CAL_WARMUP_MS - elapsed) : 0U;
        status->remaining_ms = warmup_left + IR_ZCR_CAL_WINDOW_COUNT * window_ms;
    } else if (s_zcr_cal.state == AP_IR_ZCR_CAL_COLLECTING) {
        uint32_t next_left = ((int32_t)(s_zcr_cal.next_window_ms - now) > 0) ?
                             (s_zcr_cal.next_window_ms - now) : 0U;
        uint32_t windows_after_next = IR_ZCR_CAL_WINDOW_COUNT -
                                      s_zcr_cal.windows_collected - 1U;
        status->remaining_ms = next_left + windows_after_next * window_ms;
    }
}

/* ========================================================================== */
/**
  * @brief  测试模式: 读 ADC → 去直流 → 均方值
  *         按 window_ms 累计输出三路平均能量: E <avg0>,<avg1>,<avg2>
  */
void AP_IR_TestPrint(uint32_t now)
{
    uint16_t raw[IR_CH_NUM] = {0};
    /* Feed: 读 3 通道最新 ADC 值推入历史窗口 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        raw[ch] = AP_ADC_GetLatest(ch);
        history_push(ch, raw[ch]);
    }
    /* 测试打印路径也可执行手动标定，但不会进入正式火焰状态机。 */
    process_zcr_calibration(now);

    /* 测试功率沿用原50点窗口，保证与既有0.5秒测试数据可直接比较。 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        if (s_ir.history[ch].count < IR_DC_POWER_WINDOW_SIZE) return;
    }

    int32_t work[IR_HISTORY_SIZE];
    uint32_t pwr[IR_CH_NUM];
    /* 逐通道: 线性化 → DC偏置 → 去直流 → 均方值 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        uint16_t n = history_to_workbuf(&s_ir.history[ch], work,
                                        IR_DC_POWER_WINDOW_SIZE);
        if (n < IR_DC_POWER_WINDOW_SIZE) return;

        remove_dc(work, IR_DC_POWER_WINDOW_SIZE);

        pwr[ch] = calc_mean_square(work, IR_DC_POWER_WINDOW_SIZE);
    }
    test_stats_add(pwr);

    uint32_t avg[IR_CH_NUM];
    test_stats_get_energy(avg);
    BSP_UART_Printf("%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
        (unsigned long)raw[0], (unsigned long)raw[1], (unsigned long)raw[2],
        (unsigned long)pwr[0], (unsigned long)pwr[1], (unsigned long)pwr[2],
        (unsigned long)avg[0], (unsigned long)avg[1], (unsigned long)avg[2]);
}

/*                    测试模式: 信号处理链调试打印                              */
/* ========================================================================== */

void AP_IR_DebugProcess(uint32_t now)
{
    /* 调试行包含ZCR，因此等待200点；其中DC和Power仍按最近50点计算。 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        if (s_ir.history[ch].count < IR_ZCR_WINDOW_SIZE) return;
    }

    int32_t work[IR_HISTORY_SIZE];
    int32_t dc_offset[IR_CH_NUM];
    uint32_t power[IR_CH_NUM];
    float    zcr[IR_CH_NUM];

    /* DC/Power保持50点测试口径，ZCR独立使用200点长窗口。 */
    for (uint32_t ch = 0; ch < IR_CH_NUM; ch++) {
        uint16_t n = history_raw_to_workbuf(&s_ir.history[ch], work,
                                            IR_DC_POWER_WINDOW_SIZE);
        if (n < IR_DC_POWER_WINDOW_SIZE) return;

        dc_offset[ch] = calc_mean(work, IR_DC_POWER_WINDOW_SIZE);
        history_to_workbuf(&s_ir.history[ch], work, IR_DC_POWER_WINDOW_SIZE);
        remove_dc(work, IR_DC_POWER_WINDOW_SIZE);
        power[ch] = calc_mean_square(work, IR_DC_POWER_WINDOW_SIZE);

        history_to_workbuf(&s_ir.history[ch], work, IR_ZCR_WINDOW_SIZE);
        remove_dc(work, IR_ZCR_WINDOW_SIZE);
        zcr[ch] = calc_zcr(work, IR_ZCR_WINDOW_SIZE,
                           IR_SAMPLE_RATE_HZ, s_ir.zcr_dead_zone[ch]);
    }

    /* 与正式算法一致：参考严格为0时按主通道能量决定比例饱和或归零。 */
    uint32_t r45_38 = calc_ratio_x1000(power[IR_CH_MAIN], power[IR_CH_REF_A]);
    uint32_t r45_50 = calc_ratio_x1000(power[IR_CH_MAIN], power[IR_CH_REF_B]);

    /* 保持既有测试数据列不变；固定死区及手动标定进度通过ir命令查看。 */
    BSP_UART_Printf("T%lu IRD DC=%ld,%ld,%ld P=%lu,%lu,%lu Z10=%u,%u,%u R1000=%lu,%lu\r\n",
        (unsigned long)now,
        (long)dc_offset[0], (long)dc_offset[1], (long)dc_offset[2],
        (unsigned long)power[0], (unsigned long)power[1], (unsigned long)power[2],
        (unsigned int)zcr_to_x10(zcr[0]),
        (unsigned int)zcr_to_x10(zcr[1]),
        (unsigned int)zcr_to_x10(zcr[2]),
        (unsigned long)r45_38, (unsigned long)r45_50);
}
