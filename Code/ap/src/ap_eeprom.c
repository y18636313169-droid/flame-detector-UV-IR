/**
  ******************************************************************************
  * @file    ap_eeprom.c
  * @brief   AP 层参数存储管理实现
  ******************************************************************************
  */

#include "ap_eeprom.h"
#include "ap_fault.h"
#include "crc16.h"
#include <string.h>

/* CRC 偏移辅助 */
#define CRC_OFF(type, field)    ((uint16_t)(uintptr_t)&((type *)0)->field)

/* IR 为 UV 风格多参数结构体，CRC 偏移辅助需单独定义(较长, 用宏内联) */
#define IR_CRC_OFFSET   ((uint16_t)(uintptr_t)&((AP_EEPROM_IR_Param_t *)0)->crc16)

/* 参数合法边界集中定义，EEPROM加载和CLI保存使用同一套约束。 */
#define PARAM_SENS_MAX          (9U)
#define PARAM_UV_HISTORY_MAX    (64U)
#define PARAM_TIME_MIN_MS       (100U)
#define PARAM_TIME_MAX_MS       (60000U)
#define PARAM_IR_FREQ_MIN_X10   (10U)
#define PARAM_IR_FREQ_MAX_X10   (200U)
#define PARAM_POWER_MAX         (10000000U)
#define PARAM_RATIO_MAX_X1000   (100000U)
#define PARAM_IR_DEAD_ZONE_MIN  (10U)
#define PARAM_IR_DEAD_ZONE_MAX  (30U)
#define PARAM_UV_LEGACY_THR_MAX_V1 (27U)
#define PARAM_UV_LEGACY_THR_MAX_V2 (28U)
#define EEPROM_IR_LEGACY_V6_MAGIC   (0x45495236UL)
#define EEPROM_IR_LEGACY_V6_VERSION (6U)
#define EEPROM_IR_VERSION           (7U)
#define EEPROM_SYSTEM_LEGACY_V1_MAGIC   (0x45535931UL)
#define EEPROM_SYSTEM_LEGACY_V1_VERSION (1U)
#define EEPROM_SYSTEM_VERSION           (2U)

/* ========================================================================== */
/*                         内部变量                                            */
/* ========================================================================== */

static AP_EEPROM_ADC_Param_t s_adc;
static AP_EEPROM_UV_Param_t  s_uv;
static AP_EEPROM_IR_Param_t  s_ir;   /* 3 路 IR 检测参数 */
static AP_EEPROM_System_Param_t s_system;

/* 系统参数v1只保存演示模式；升级到v2时必须保留该掉电配置。 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t show_mode;
    uint16_t crc16;
    uint16_t _pad;
} AP_EEPROM_System_V1_Param_t;

/*
 * v6只保存一个固定进场功率。保留旧布局用于一次性迁移，避免升级固件时
 * 丢失现场已经设置的灵敏度、光谱参数和旧ZCR兼容字段。
 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t sensitivity;
    uint32_t power_threshold;
    uint32_t r38_threshold;
    uint32_t r50_threshold;
    uint32_t freq_low_x10;
    uint32_t freq_high_x10;
    uint32_t cfm_min;
    uint32_t cfm_max;
    uint32_t zcr_dead_zone[3];
    uint16_t crc16;
    uint16_t _pad;
} AP_EEPROM_IR_V6_Param_t;

#define IR_V6_CRC_OFFSET \
    ((uint16_t)(uintptr_t)&((AP_EEPROM_IR_V6_Param_t *)0)->crc16)

/* v7恰好占用一个64字节扇区；以下检查防止后续加字段覆盖相邻UV/IR扇区。 */
typedef char IR_V6_LayoutMustRemain60Bytes[
    (sizeof(AP_EEPROM_IR_V6_Param_t) == 60U) ? 1 : -1];
typedef char IR_V7_LayoutMustFitOneSector[
    (sizeof(AP_EEPROM_IR_Param_t) <= EEPROM_SECTOR_SIZE) ? 1 : -1];
typedef char SystemLayoutMustFitOneSector[
    ((sizeof(AP_EEPROM_System_Param_t) <= EEPROM_SECTOR_SIZE) &&
     ((sizeof(AP_EEPROM_System_Param_t) % sizeof(uint32_t)) == 0U)) ? 1 : -1];
typedef char SystemV1LayoutMustRemain20Bytes[
    (sizeof(AP_EEPROM_System_V1_Param_t) == 20U) ? 1 : -1];
typedef char EepromFaultGroupLayoutMustMatch[
    ((AP_EEPROM_GROUP_SYSTEM == AP_CONFIG_GROUP_SYSTEM) &&
     (AP_EEPROM_GROUP_ADC == AP_CONFIG_GROUP_ADC) &&
     (AP_EEPROM_GROUP_UV == AP_CONFIG_GROUP_UV) &&
     (AP_EEPROM_GROUP_IR == AP_CONFIG_GROUP_IR)) ? 1 : -1];

/** @brief 记录加载默认值的原因，避免参数自动恢复后掩盖原始EEPROM故障。 */
static void record_load_result(uint16_t group, int load_result)
{
    if (load_result == BSP_EEPROM_LOAD_VALID) {
        return;
    }
    if (load_result == BSP_EEPROM_LOAD_CRC_DEFAULTED) {
        AP_Fault_Set(AP_FAULT_EEPROM_CRC_ERROR);
    } else if (load_result == BSP_EEPROM_LOAD_MAGIC_DEFAULTED) {
        AP_Fault_Set(AP_FAULT_EEPROM_VERSION_ERROR);
    } else {
        AP_Fault_Set(AP_FAULT_EEPROM_WRITE_ERROR);
    }
    AP_Fault_SetConfigGroups(group);
}

/** @brief 区分结构版本错误与参数范围错误，并标记对应待重配组。 */
static void record_semantic_error(uint16_t group, uint8_t version_error)
{
    AP_Fault_Set(version_error ? AP_FAULT_EEPROM_VERSION_ERROR
                               : AP_FAULT_EEPROM_RANGE_ERROR);
    AP_Fault_SetConfigGroups(group);
}

/** @brief 统一收口保存结果；仅在写入和读回均成功后清除参数组故障。 */
static int finish_storage_operation(uint16_t group, int result)
{
    if (result == 0) {
        AP_Fault_ClearConfigGroups(group);
        return AP_EEPROM_OK;
    }
    AP_Fault_Set(AP_FAULT_EEPROM_WRITE_ERROR);
    AP_Fault_SetConfigGroups(group);
    return AP_EEPROM_ERROR_STORAGE;
}

/* ========================================================================== */
/*                         默认值回调                                          */
/* ========================================================================== */

static void adc_defaults(void *buf)
{
    AP_EEPROM_ADC_Param_t *p = (AP_EEPROM_ADC_Param_t *)buf;
    memset(p, 0, sizeof(*p));
    p->magic     = EEPROM_ADC_MAGIC;
    p->version   = 1;
    p->length    = sizeof(AP_EEPROM_ADC_Param_t);
}

static void uv_defaults(void *buf)
{
    AP_EEPROM_UV_Param_t *p = (AP_EEPROM_UV_Param_t *)buf;
    memset(p, 0, sizeof(*p));
    p->magic       = EEPROM_UV_MAGIC;
    p->version     = 3;
    p->length      = sizeof(AP_EEPROM_UV_Param_t);
    p->sensitivity = AP_EEPROM_UV_DEFAULT_SENS;
    p->thr_min     = AP_EEPROM_UV_DEFAULT_THR_MIN;
    p->thr_max     = AP_EEPROM_UV_DEFAULT_THR_MAX;
    p->win_min     = AP_EEPROM_UV_DEFAULT_WIN_MIN;
    p->win_max     = AP_EEPROM_UV_DEFAULT_WIN_MAX;
    p->cfm_min     = AP_EEPROM_UV_DEFAULT_CFM_MIN;
    p->cfm_max     = AP_EEPROM_UV_DEFAULT_CFM_MAX;
    p->pw_min_us   = AP_EEPROM_UV_DEFAULT_PW_MIN_US;
    p->pw_max_us   = AP_EEPROM_UV_DEFAULT_PW_MAX_US;
    p->print_window_ms = AP_EEPROM_UV_DEFAULT_PRINT_WIN_MS;
}

static void ir_defaults(void *buf)
{
    AP_EEPROM_IR_Param_t *p = (AP_EEPROM_IR_Param_t *)buf;
    memset(p, 0, sizeof(*p));
    p->magic       = EEPROM_IR_MAGIC;
    p->version     = EEPROM_IR_VERSION;
    p->length      = sizeof(AP_EEPROM_IR_Param_t);
    p->sensitivity = AP_EEPROM_IR_DEFAULT_SENS;
    p->power_min   = AP_EEPROM_IR_DEFAULT_POWER_MIN;
    p->power_max   = AP_EEPROM_IR_DEFAULT_POWER_MAX;
    p->r38_threshold = AP_EEPROM_IR_DEFAULT_R38;
    p->r50_threshold = AP_EEPROM_IR_DEFAULT_R50;
    p->freq_low_x10  = AP_EEPROM_IR_DEFAULT_FREQ_LOW;
    p->freq_high_x10 = AP_EEPROM_IR_DEFAULT_FREQ_HIGH;
    p->cfm_min     = AP_EEPROM_IR_DEFAULT_CFM_MIN;
    p->cfm_max     = AP_EEPROM_IR_DEFAULT_CFM_MAX;
    p->zcr_dead_zone[0] = AP_EEPROM_IR_DEFAULT_DZ_38;
    p->zcr_dead_zone[1] = AP_EEPROM_IR_DEFAULT_DZ_45;
    p->zcr_dead_zone[2] = AP_EEPROM_IR_DEFAULT_DZ_50;
}

static void system_defaults(void *buf)
{
    AP_EEPROM_System_Param_t *p = (AP_EEPROM_System_Param_t *)buf;
    memset(p, 0, sizeof(*p));
    p->magic     = EEPROM_SYSTEM_MAGIC;
    p->version   = EEPROM_SYSTEM_VERSION;
    p->length    = sizeof(*p);
    p->show_mode = AP_EEPROM_SYSTEM_DEFAULT_SHOW_MODE;
    p->test_mode = AP_EEPROM_SYSTEM_DEFAULT_TEST_MODE;
    p->ir_profile_enabled = AP_EEPROM_SYSTEM_DEFAULT_IR_PROFILE;
}

/** @brief 校验ADC参数，防止超过12位ADC有效范围。 */
static int adc_params_valid(const AP_EEPROM_ADC_Param_t *p)
{
    if (p->version != 1U || p->length != sizeof(*p)) return 0;
    for (uint32_t ch = 0; ch < 3U; ch++) {
        if (p->threshold[ch] > 4095U) return 0;
    }
    return 1;
}

/** @brief 校验UV窗口、历史容量、脉宽和时间范围的组合合法性。 */
static int uv_params_valid(const AP_EEPROM_UV_Param_t *p)
{
    if (p->version != 3U || p->length != sizeof(*p)) return 0;
    if (p->sensitivity > PARAM_SENS_MAX) return 0;
    if (p->thr_min == 0U || p->thr_min > p->thr_max ||
        p->thr_max > PARAM_UV_HISTORY_MAX) return 0;
    if (p->win_min < PARAM_TIME_MIN_MS || p->win_min > p->win_max ||
        p->win_max > PARAM_TIME_MAX_MS) return 0;
    if (p->cfm_min > p->cfm_max || p->cfm_max > PARAM_TIME_MAX_MS) return 0;
    if (p->pw_min_us == 0U || p->pw_min_us > p->pw_max_us ||
        p->pw_max_us > UINT16_MAX) return 0;
    if (p->print_window_ms < PARAM_TIME_MIN_MS ||
        p->print_window_ms > PARAM_TIME_MAX_MS) return 0;
    return 1;
}

/** @brief 校验IR功率范围、固定光谱比、1~20Hz频带及确认时间范围。 */
static int ir_params_valid(const AP_EEPROM_IR_Param_t *p)
{
    if (p->version != EEPROM_IR_VERSION || p->length != sizeof(*p)) return 0;
    if (p->sensitivity > PARAM_SENS_MAX) return 0;
    /* 等级0必须对应较低进场阈值，避免灵敏度方向与确认时长相反。 */
    if (p->power_min == 0U || p->power_min > p->power_max ||
        p->power_max > PARAM_POWER_MAX) return 0;
    if (p->r38_threshold == 0U || p->r38_threshold > PARAM_RATIO_MAX_X1000 ||
        p->r50_threshold == 0U || p->r50_threshold > PARAM_RATIO_MAX_X1000) return 0;
    if (p->freq_low_x10 < PARAM_IR_FREQ_MIN_X10 ||
        p->freq_low_x10 > p->freq_high_x10 ||
        p->freq_high_x10 > PARAM_IR_FREQ_MAX_X10) return 0;
    if (p->cfm_min > p->cfm_max || p->cfm_max > PARAM_TIME_MAX_MS) return 0;
    /* 该三个字仅用于继续接受旧v7数据，FFT检测链不使用其数值。 */
    for (uint32_t ch = 0; ch < 3U; ch++) {
        if (p->zcr_dead_zone[ch] < PARAM_IR_DEAD_ZONE_MIN ||
            p->zcr_dead_zone[ch] > PARAM_IR_DEAD_ZONE_MAX) return 0;
    }
    return 1;
}

/** @brief 三个系统开关均只允许0/1，拒绝损坏或越界的持久化配置。 */
static int system_params_valid(const AP_EEPROM_System_Param_t *p)
{
    if (p->version != EEPROM_SYSTEM_VERSION || p->length != sizeof(*p)) return 0;
    if (p->show_mode > 1U || p->test_mode > 1U ||
        p->ir_profile_enabled > 1U) return 0;
    return 1;
}

/** @brief 校验旧v1系统参数及CRC，防止把随机EEPROM内容当作迁移源。 */
static int system_v1_params_valid(AP_EEPROM_System_V1_Param_t *p)
{
    if (p->magic != EEPROM_SYSTEM_LEGACY_V1_MAGIC ||
        p->version != EEPROM_SYSTEM_LEGACY_V1_VERSION ||
        p->length != sizeof(*p) || p->show_mode > 1U) return 0;

    uint16_t saved_crc = p->crc16;
    p->crc16 = 0U;
    uint16_t calc_crc = CRC16_CCITT((const uint8_t *)p, sizeof(*p));
    p->crc16 = saved_crc;
    return saved_crc == calc_crc;
}

/**
  @brief  将系统参数v1迁移到v2，保留原演示模式并补充运行时测试/包络开关
  @return 1: 已迁移；0: 不是有效v1；-1: EEPROM保存失败
 */
static int system_try_migrate_v1(void)
{
    AP_EEPROM_System_V1_Param_t old;
    BSP_EEPROM_Read(EEPROM_SECTOR_SYSTEM, &old, sizeof(old));
    if (!system_v1_params_valid(&old)) return 0;

    system_defaults(&s_system);
    s_system.show_mode = old.show_mode;
    return (BSP_EEPROM_SaveSector(
                EEPROM_SECTOR_SYSTEM, &s_system, sizeof(s_system),
                CRC_OFF(AP_EEPROM_System_Param_t, crc16)) == 0) ? 1 : -1;
}

/** @brief 校验旧v6结构及其CRC，只有完整有效的数据才允许迁移。 */
static int ir_v6_params_valid(AP_EEPROM_IR_V6_Param_t *p)
{
    if (p->magic != EEPROM_IR_LEGACY_V6_MAGIC ||
        p->version != EEPROM_IR_LEGACY_V6_VERSION ||
        p->length != sizeof(*p)) return 0;

    uint16_t saved_crc = p->crc16;
    p->crc16 = 0U;
    uint16_t calc_crc = CRC16_CCITT((const uint8_t *)p, sizeof(*p));
    p->crc16 = saved_crc;
    if (saved_crc != calc_crc) return 0;

    if (p->sensitivity > PARAM_SENS_MAX) return 0;
    if (p->power_threshold == 0U ||
        p->power_threshold > PARAM_POWER_MAX) return 0;
    if (p->r38_threshold == 0U ||
        p->r38_threshold > PARAM_RATIO_MAX_X1000 ||
        p->r50_threshold == 0U ||
        p->r50_threshold > PARAM_RATIO_MAX_X1000) return 0;
    if (p->freq_low_x10 < PARAM_IR_FREQ_MIN_X10 ||
        p->freq_low_x10 > p->freq_high_x10 ||
        p->freq_high_x10 > PARAM_IR_FREQ_MAX_X10) return 0;
    if (p->cfm_min > p->cfm_max ||
        p->cfm_max > PARAM_TIME_MAX_MS) return 0;
    for (uint32_t ch = 0U; ch < 3U; ch++) {
        if (p->zcr_dead_zone[ch] < PARAM_IR_DEAD_ZONE_MIN ||
            p->zcr_dead_zone[ch] > PARAM_IR_DEAD_ZONE_MAX) return 0;
    }
    return 1;
}

/**
  @brief  将有效v6 IR参数迁移到v7
  @return 1: 已迁移并保存；0: 当前不是有效v6；-1: 迁移保存失败
  @note   旧固定功率不再沿用，统一替换为实测得到的12000~22000等级范围。
          其余现场参数全部保留。
 */
static int ir_try_migrate_v6(void)
{
    AP_EEPROM_IR_V6_Param_t old;
    BSP_EEPROM_Read(EEPROM_SECTOR_IR, &old, sizeof(old));
    if (!ir_v6_params_valid(&old)) return 0;

    ir_defaults(&s_ir);
    s_ir.sensitivity = old.sensitivity;
    s_ir.r38_threshold = old.r38_threshold;
    s_ir.r50_threshold = old.r50_threshold;
    s_ir.freq_low_x10 = old.freq_low_x10;
    s_ir.freq_high_x10 = old.freq_high_x10;
    s_ir.cfm_min = old.cfm_min;
    s_ir.cfm_max = old.cfm_max;
    for (uint32_t ch = 0U; ch < 3U; ch++) {
        s_ir.zcr_dead_zone[ch] = old.zcr_dead_zone[ch];
    }

    return (BSP_EEPROM_SaveSector(EEPROM_SECTOR_IR, &s_ir, sizeof(s_ir),
                                  IR_CRC_OFFSET) == 0) ? 1 : -1;
}

/* ========================================================================== */
/*                         公有 API                                            */
/* ========================================================================== */

int AP_EEPROM_Init(void)
{
    int ret = 0;
    int load_result;

    load_result = BSP_EEPROM_LoadSector(
        EEPROM_SECTOR_ADC, &s_adc, sizeof(s_adc), EEPROM_ADC_MAGIC,
        adc_defaults, CRC_OFF(AP_EEPROM_ADC_Param_t, crc16));
    record_load_result(AP_EEPROM_GROUP_ADC, load_result);
    if (load_result == BSP_EEPROM_LOAD_WRITE_ERROR) {
        ret = -1;
    }
    /* CRC正确但语义非法时同样恢复默认值，避免合法CRC掩盖错误配置。 */
    if (!adc_params_valid(&s_adc)) {
        uint8_t version_error =
            (s_adc.version != 1U || s_adc.length != sizeof(s_adc)) ? 1U : 0U;
        record_semantic_error(AP_EEPROM_GROUP_ADC, version_error);
        adc_defaults(&s_adc);
        if (BSP_EEPROM_SaveSector(
                EEPROM_SECTOR_ADC, &s_adc, sizeof(s_adc),
                CRC_OFF(AP_EEPROM_ADC_Param_t, crc16)) != 0) {
            AP_Fault_Set(AP_FAULT_EEPROM_WRITE_ERROR);
            ret = -1;
        }
    }

    load_result = BSP_EEPROM_LoadSector(
        EEPROM_SECTOR_UV, &s_uv, sizeof(s_uv), EEPROM_UV_MAGIC,
        uv_defaults, CRC_OFF(AP_EEPROM_UV_Param_t, crc16));
    record_load_result(AP_EEPROM_GROUP_UV, load_result);
    if (load_result == BSP_EEPROM_LOAD_WRITE_ERROR) {
        ret = -1;
    }
    /* v3参数必须同时通过结构版本和业务范围校验。 */
    if (!uv_params_valid(&s_uv)) {
        uint8_t version_error =
            (s_uv.version != 3U || s_uv.length != sizeof(s_uv)) ? 1U : 0U;
        record_semantic_error(AP_EEPROM_GROUP_UV, version_error);
        uv_defaults(&s_uv);
        if (BSP_EEPROM_SaveSector(
                EEPROM_SECTOR_UV, &s_uv, sizeof(s_uv),
                CRC_OFF(AP_EEPROM_UV_Param_t, crc16)) != 0) {
            AP_Fault_Set(AP_FAULT_EEPROM_WRITE_ERROR);
            ret = -1;
        }
    }
    /* 仅迁移历史默认上限27/28到实测进入阈值24，保留其余现场标定参数。 */
    if ((s_uv.thr_max == PARAM_UV_LEGACY_THR_MAX_V1) ||
        (s_uv.thr_max == PARAM_UV_LEGACY_THR_MAX_V2)) {
        s_uv.thr_max = AP_EEPROM_UV_DEFAULT_THR_MAX;
        if (BSP_EEPROM_SaveSector(EEPROM_SECTOR_UV, &s_uv, sizeof(s_uv),
                                  CRC_OFF(AP_EEPROM_UV_Param_t, crc16)) != 0) {
            AP_Fault_Set(AP_FAULT_EEPROM_WRITE_ERROR);
            AP_Fault_SetConfigGroups(AP_EEPROM_GROUP_UV);
            ret = -1;
        }
    }

    int ir_migration = ir_try_migrate_v6();
    if (ir_migration < 0) {
        /* RAM中仍保留完整v7默认值；EEPROM写失败由返回值上报。 */
        AP_Fault_Set(AP_FAULT_EEPROM_WRITE_ERROR);
        AP_Fault_SetConfigGroups(AP_EEPROM_GROUP_IR);
        ret = -1;
    } else if (ir_migration == 0) {
        load_result = BSP_EEPROM_LoadSector(
            EEPROM_SECTOR_IR, &s_ir, sizeof(s_ir), EEPROM_IR_MAGIC,
            ir_defaults, IR_CRC_OFFSET);
        record_load_result(AP_EEPROM_GROUP_IR, load_result);
        if (load_result == BSP_EEPROM_LOAD_WRITE_ERROR) {
            ret = -1;
        }
        /* 非法v7内容由参数校验链恢复为v7默认值。 */
        if (!ir_params_valid(&s_ir)) {
            uint8_t version_error =
                (s_ir.version != EEPROM_IR_VERSION ||
                 s_ir.length != sizeof(s_ir)) ? 1U : 0U;
            record_semantic_error(AP_EEPROM_GROUP_IR, version_error);
            ir_defaults(&s_ir);
            if (BSP_EEPROM_SaveSector(EEPROM_SECTOR_IR, &s_ir, sizeof(s_ir),
                                      IR_CRC_OFFSET) != 0) {
                AP_Fault_Set(AP_FAULT_EEPROM_WRITE_ERROR);
                ret = -1;
            }
        }
    }

    /*
     * 系统模式使用独立扇区，避免修改已有ADC/UV/IR结构导致现场参数失效。
     * v1保留已有SHOW值并迁移；未写过该扇区时建立APP/PROFILE开启默认配置。
     */
    int system_migration = system_try_migrate_v1();
    if (system_migration < 0) {
        AP_Fault_Set(AP_FAULT_EEPROM_WRITE_ERROR);
        AP_Fault_SetConfigGroups(AP_EEPROM_GROUP_SYSTEM);
        ret = -1;
    } else if (system_migration == 0) {
        load_result = BSP_EEPROM_LoadSector(
            EEPROM_SECTOR_SYSTEM, &s_system, sizeof(s_system),
            EEPROM_SYSTEM_MAGIC, system_defaults,
            CRC_OFF(AP_EEPROM_System_Param_t, crc16));
        record_load_result(AP_EEPROM_GROUP_SYSTEM, load_result);
        if (load_result == BSP_EEPROM_LOAD_WRITE_ERROR) {
            ret = -1;
        }
        if (!system_params_valid(&s_system)) {
            uint8_t version_error =
                (s_system.version != EEPROM_SYSTEM_VERSION ||
                 s_system.length != sizeof(s_system)) ? 1U : 0U;
            record_semantic_error(AP_EEPROM_GROUP_SYSTEM, version_error);
            system_defaults(&s_system);
            if (BSP_EEPROM_SaveSector(
                    EEPROM_SECTOR_SYSTEM, &s_system, sizeof(s_system),
                    CRC_OFF(AP_EEPROM_System_Param_t, crc16)) != 0) {
                AP_Fault_Set(AP_FAULT_EEPROM_WRITE_ERROR);
                ret = -1;
            }
        }
    }

    return ret;
}

/* ---- ADC ---- */

const AP_EEPROM_ADC_Param_t *AP_EEPROM_ADC_Get(void)
{
    return &s_adc;
}

int AP_EEPROM_ADC_Save(const AP_EEPROM_ADC_Param_t *p)
{
    if (p == NULL) return AP_EEPROM_ERROR_INVALID;
    /* 候选值先校验并写入成功后再提交到RAM，保存失败不污染运行副本。 */
    AP_EEPROM_ADC_Param_t candidate = *p;
    candidate.magic = EEPROM_ADC_MAGIC;
    candidate.version = 1U;
    candidate.length = sizeof(candidate);
    if (!adc_params_valid(&candidate)) return AP_EEPROM_ERROR_INVALID;
    int ret = BSP_EEPROM_SaveSector(EEPROM_SECTOR_ADC, &candidate, sizeof(candidate),
                                    CRC_OFF(AP_EEPROM_ADC_Param_t, crc16));
    if (ret == 0) {
        s_adc = candidate;
    }
    return finish_storage_operation(AP_EEPROM_GROUP_ADC, ret);
}

int AP_EEPROM_ADC_Reset(void)
{
    AP_EEPROM_ADC_Param_t candidate;
    int ret;

    adc_defaults(&candidate);
    ret = BSP_EEPROM_SaveSector(EEPROM_SECTOR_ADC, &candidate,
                                sizeof(candidate),
                                CRC_OFF(AP_EEPROM_ADC_Param_t, crc16));
    if (ret == 0) s_adc = candidate;
    return finish_storage_operation(AP_EEPROM_GROUP_ADC, ret);
}

/* ---- UV ---- */

const AP_EEPROM_UV_Param_t *AP_EEPROM_UV_Get(void)
{
    return &s_uv;
}

int AP_EEPROM_UV_Save(const AP_EEPROM_UV_Param_t *p)
{
    if (p == NULL) return AP_EEPROM_ERROR_INVALID;
    /* 使用事务式候选副本，非法参数和Flash失败都保留原配置。 */
    AP_EEPROM_UV_Param_t candidate = *p;
    candidate.magic = EEPROM_UV_MAGIC;
    candidate.version = 3U;
    candidate.length = sizeof(candidate);
    if (!uv_params_valid(&candidate)) return AP_EEPROM_ERROR_INVALID;
    int ret = BSP_EEPROM_SaveSector(EEPROM_SECTOR_UV, &candidate, sizeof(candidate),
                                    CRC_OFF(AP_EEPROM_UV_Param_t, crc16));
    if (ret == 0) s_uv = candidate;
    return finish_storage_operation(AP_EEPROM_GROUP_UV, ret);
}

int AP_EEPROM_UV_Reset(void)
{
    AP_EEPROM_UV_Param_t candidate;
    int ret;

    uv_defaults(&candidate);
    ret = BSP_EEPROM_SaveSector(EEPROM_SECTOR_UV, &candidate,
                                sizeof(candidate),
                                CRC_OFF(AP_EEPROM_UV_Param_t, crc16));
    if (ret == 0) s_uv = candidate;
    return finish_storage_operation(AP_EEPROM_GROUP_UV, ret);
}

/* ---- IR ---- */

const AP_EEPROM_IR_Param_t *AP_EEPROM_IR_Get(void)
{
    return &s_ir;
}

int AP_EEPROM_IR_Save(const AP_EEPROM_IR_Param_t *p)
{
    if (p == NULL) return AP_EEPROM_ERROR_INVALID;
    /* 使用事务式候选副本，功率范围/固定判据在落盘前统一校验。 */
    AP_EEPROM_IR_Param_t candidate = *p;
    candidate.magic = EEPROM_IR_MAGIC;
    candidate.version = EEPROM_IR_VERSION;
    candidate.length = sizeof(candidate);
    if (!ir_params_valid(&candidate)) return AP_EEPROM_ERROR_INVALID;
    int ret = BSP_EEPROM_SaveSector(EEPROM_SECTOR_IR, &candidate, sizeof(candidate),
                                    IR_CRC_OFFSET);
    if (ret == 0) s_ir = candidate;
    return finish_storage_operation(AP_EEPROM_GROUP_IR, ret);
}

int AP_EEPROM_IR_Reset(void)
{
    AP_EEPROM_IR_Param_t candidate;
    int ret;

    ir_defaults(&candidate);
    ret = BSP_EEPROM_SaveSector(EEPROM_SECTOR_IR, &candidate,
                                sizeof(candidate), IR_CRC_OFFSET);
    if (ret == 0) s_ir = candidate;
    return finish_storage_operation(AP_EEPROM_GROUP_IR, ret);
}

/* ---- System ---- */

const AP_EEPROM_System_Param_t *AP_EEPROM_System_Get(void)
{
    return &s_system;
}

int AP_EEPROM_System_Save(const AP_EEPROM_System_Param_t *p)
{
    if (p == NULL) return AP_EEPROM_ERROR_INVALID;

    /*
     * 先校验并写入候选副本，Flash写入成功后才提交RAM配置；
     * 这样掉电存储失败不会造成当前模式和持久化模式不一致。
     */
    AP_EEPROM_System_Param_t candidate = *p;
    candidate.magic = EEPROM_SYSTEM_MAGIC;
    candidate.version = EEPROM_SYSTEM_VERSION;
    candidate.length = sizeof(candidate);
    if (!system_params_valid(&candidate)) return AP_EEPROM_ERROR_INVALID;

    int ret = BSP_EEPROM_SaveSector(EEPROM_SECTOR_SYSTEM, &candidate,
                                    sizeof(candidate),
                                    CRC_OFF(AP_EEPROM_System_Param_t, crc16));
    if (ret == 0) s_system = candidate;
    return finish_storage_operation(AP_EEPROM_GROUP_SYSTEM, ret);
}

int AP_EEPROM_System_Reset(void)
{
    AP_EEPROM_System_Param_t candidate;
    int ret;

    system_defaults(&candidate);
    ret = BSP_EEPROM_SaveSector(EEPROM_SECTOR_SYSTEM, &candidate,
                                sizeof(candidate),
                                CRC_OFF(AP_EEPROM_System_Param_t, crc16));
    if (ret == 0) s_system = candidate;
    return finish_storage_operation(AP_EEPROM_GROUP_SYSTEM, ret);
}

uint16_t AP_EEPROM_ResetAll(void)
{
    uint16_t failed = 0U;

    AP_Fault_SetDefaultsRecoveryActive(1U);
    if (AP_EEPROM_System_Reset() != AP_EEPROM_OK) failed |= AP_EEPROM_GROUP_SYSTEM;
    if (AP_EEPROM_ADC_Reset() != AP_EEPROM_OK) failed |= AP_EEPROM_GROUP_ADC;
    if (AP_EEPROM_UV_Reset() != AP_EEPROM_OK) failed |= AP_EEPROM_GROUP_UV;
    if (AP_EEPROM_IR_Reset() != AP_EEPROM_OK) failed |= AP_EEPROM_GROUP_IR;
    AP_Fault_SetDefaultsRecoveryActive(0U);
    return failed;
}
