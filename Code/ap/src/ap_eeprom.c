/**
  ******************************************************************************
  * @file    ap_eeprom.c
  * @brief   AP 层参数存储管理实现
  ******************************************************************************
  */

#include "ap_eeprom.h"
#include <string.h>

/* CRC 偏移辅助 */
#define CRC_OFF(type, field)    ((uint16_t)(uintptr_t)&((type *)0)->field)

/* IR 为 UV 风格多参数结构体，CRC 偏移辅助需单独定义(较长, 用宏内联) */
#define IR_CRC_OFFSET   ((uint16_t)(uintptr_t)&((AP_EEPROM_IR_Param_t *)0)->crc16)

/* ========================================================================== */
/*                         内部变量                                            */
/* ========================================================================== */

static AP_EEPROM_ADC_Param_t s_adc;
static AP_EEPROM_UV_Param_t  s_uv;
static AP_EEPROM_IR_Param_t  s_ir;   /* 3 路 IR 检测参数 */

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
    p->version     = 2;
    p->length      = sizeof(AP_EEPROM_UV_Param_t);
    p->sensitivity = AP_EEPROM_UV_DEFAULT_SENS;
    p->thr_min     = AP_EEPROM_UV_DEFAULT_THR_MIN;
    p->thr_max     = AP_EEPROM_UV_DEFAULT_THR_MAX;
    p->win_min     = AP_EEPROM_UV_DEFAULT_WIN_MIN;
    p->win_max     = AP_EEPROM_UV_DEFAULT_WIN_MAX;
    p->cfm_min     = AP_EEPROM_UV_DEFAULT_CFM_MIN;
    p->cfm_max     = AP_EEPROM_UV_DEFAULT_CFM_MAX;
    p->clr_min     = AP_EEPROM_UV_DEFAULT_CLR_MIN;
    p->clr_max     = AP_EEPROM_UV_DEFAULT_CLR_MAX;
    p->pw_min_us   = AP_EEPROM_UV_DEFAULT_PW_MIN_US;
    p->pw_max_us   = AP_EEPROM_UV_DEFAULT_PW_MAX_US;
    p->print_window_ms = AP_EEPROM_UV_DEFAULT_PRINT_WIN_MS;
}

static void ir_defaults(void *buf)
{
    AP_EEPROM_IR_Param_t *p = (AP_EEPROM_IR_Param_t *)buf;
    memset(p, 0, sizeof(*p));
    p->magic       = EEPROM_IR_MAGIC;
    p->version     = 2;
    p->length      = sizeof(AP_EEPROM_IR_Param_t);
    p->sensitivity = AP_EEPROM_IR_DEFAULT_SENS;
    p->pwr_min     = AP_EEPROM_IR_DEFAULT_PWR_MIN;
    p->pwr_max     = AP_EEPROM_IR_DEFAULT_PWR_MAX;
    p->r38_min     = AP_EEPROM_IR_DEFAULT_R38_MIN;
    p->r38_max     = AP_EEPROM_IR_DEFAULT_R38_MAX;
    p->r50_min     = AP_EEPROM_IR_DEFAULT_R50_MIN;
    p->r50_max     = AP_EEPROM_IR_DEFAULT_R50_MAX;
    p->freq_low_x10  = AP_EEPROM_IR_DEFAULT_FREQ_LOW;
    p->freq_high_x10 = AP_EEPROM_IR_DEFAULT_FREQ_HIGH;
    p->cfm_min     = AP_EEPROM_IR_DEFAULT_CFM_MIN;
    p->cfm_max     = AP_EEPROM_IR_DEFAULT_CFM_MAX;
}

/* ========================================================================== */
/*                         公有 API                                            */
/* ========================================================================== */

int AP_EEPROM_Init(void)
{
    int ret = 0;

    if (BSP_EEPROM_LoadSector(EEPROM_SECTOR_ADC, &s_adc, sizeof(s_adc),
                               EEPROM_ADC_MAGIC, adc_defaults,
                               CRC_OFF(AP_EEPROM_ADC_Param_t, crc16)) != 0) {
        ret = -1;
    }

    if (BSP_EEPROM_LoadSector(EEPROM_SECTOR_UV, &s_uv, sizeof(s_uv),
                               EEPROM_UV_MAGIC, uv_defaults,
                               CRC_OFF(AP_EEPROM_UV_Param_t, crc16)) != 0) {
        ret = -1;
    }

    if (BSP_EEPROM_LoadSector(EEPROM_SECTOR_IR, &s_ir, sizeof(s_ir),
                               EEPROM_IR_MAGIC, ir_defaults,
                               IR_CRC_OFFSET) != 0) {
        ret = -1;
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
    if (p == NULL) return -1;
    memcpy(&s_adc, p, sizeof(s_adc));
    return BSP_EEPROM_SaveSector(EEPROM_SECTOR_ADC, &s_adc, sizeof(s_adc),
                                  CRC_OFF(AP_EEPROM_ADC_Param_t, crc16));
}

int AP_EEPROM_ADC_Reset(void)
{
    adc_defaults(&s_adc);
    return BSP_EEPROM_SaveSector(EEPROM_SECTOR_ADC, &s_adc, sizeof(s_adc),
                                  CRC_OFF(AP_EEPROM_ADC_Param_t, crc16));
}

/* ---- UV ---- */

const AP_EEPROM_UV_Param_t *AP_EEPROM_UV_Get(void)
{
    return &s_uv;
}

int AP_EEPROM_UV_Save(const AP_EEPROM_UV_Param_t *p)
{
    if (p == NULL) return -1;
    memcpy(&s_uv, p, sizeof(s_uv));
    return BSP_EEPROM_SaveSector(EEPROM_SECTOR_UV, &s_uv, sizeof(s_uv),
                                  CRC_OFF(AP_EEPROM_UV_Param_t, crc16));
}

int AP_EEPROM_UV_Reset(void)
{
    uv_defaults(&s_uv);
    return BSP_EEPROM_SaveSector(EEPROM_SECTOR_UV, &s_uv, sizeof(s_uv),
                                  CRC_OFF(AP_EEPROM_UV_Param_t, crc16));
}

/* ---- IR ---- */

const AP_EEPROM_IR_Param_t *AP_EEPROM_IR_Get(void)
{
    return &s_ir;
}

int AP_EEPROM_IR_Save(const AP_EEPROM_IR_Param_t *p)
{
    if (p == NULL) return -1;
    memcpy(&s_ir, p, sizeof(s_ir));
    return BSP_EEPROM_SaveSector(EEPROM_SECTOR_IR, &s_ir, sizeof(s_ir),
                                  IR_CRC_OFFSET);
}

int AP_EEPROM_IR_Reset(void)
{
    ir_defaults(&s_ir);
    return BSP_EEPROM_SaveSector(EEPROM_SECTOR_IR, &s_ir, sizeof(s_ir),
                                  IR_CRC_OFFSET);
}
