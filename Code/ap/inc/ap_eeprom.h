/**
  ******************************************************************************
  * @file    ap_eeprom.h
  * @brief   AP 层参数存储管理 — 模块参数分组 + 默认值 + 保存/加载
  *
  *          基于 BSP 层 EEPROM 扇区接口，封装各功能模块的参数结构体、
  *          默认值和读写操作。应用代码（含 cmd）通过本模块存取参数。
  *
  *          新增参数组的步骤：
  *            1. 在下方定义结构体 + 魔数
  *            2. 分配一个扇区号（EEPROM_SECTOR_xx）
  *            3. 在 ap_eeprom.c 中添加默认值回调 + Init 加载 + Get/Save/Reset
  ******************************************************************************
  */
#ifndef __AP_EEPROM_H__
#define __AP_EEPROM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "hal_eeprom.h"

/* ========================================================================== */
/*                         扇区分配                                            */
/* ========================================================================== */

#define EEPROM_SECTOR_ADC   (EEPROM_BASE + 0 * EEPROM_SECTOR_SIZE)   /* 0x08080000 */
#define EEPROM_SECTOR_UV    (EEPROM_BASE + 1 * EEPROM_SECTOR_SIZE)   /* 0x08080040 */
#define EEPROM_SECTOR_IR    (EEPROM_BASE + 2 * EEPROM_SECTOR_SIZE)   /* 0x08080080 */

/* ========================================================================== */
/*                         1. ADC 参数                                         */
/* ========================================================================== */

#define EEPROM_ADC_MAGIC    0x45414443UL       /* "EADC" */

typedef struct {
    uint32_t    magic;              /* 魔数 */
    uint32_t    version;            /* 版本=1 */
    uint32_t    length;             /* 结构体总字节数 */
    uint32_t    threshold[3];       /* 3 路 ADC 原始值阈值 */
    uint16_t    crc16;              /* CRC16-CCITT */
    uint16_t    _pad;
} AP_EEPROM_ADC_Param_t;

/* ========================================================================== */
/*                         2. UV 检测参数                                      */
/* ========================================================================== */

#define EEPROM_UV_MAGIC     0x45555656UL       /* "EUVV" */

typedef struct {
    uint32_t    magic;
    uint32_t    version;
    uint32_t    length;
    uint32_t    sensitivity;        /* 当前等级 0~9 */
    uint32_t    thr_min;            /* threshold 等级0 */
    uint32_t    thr_max;            /* threshold 等级9 */
    uint32_t    win_min;            /* window 等级0 */
    uint32_t    win_max;            /* window 等级9 */
    uint32_t    cfm_min;            /* confirm 等级0 */
    uint32_t    cfm_max;            /* confirm 等级9 */
    uint32_t    clr_min;            /* clear 等级0 */
    uint32_t    clr_max;            /* clear 等级9 */
    uint16_t    crc16;
    uint16_t    _pad;
} AP_EEPROM_UV_Param_t;

/* ========================================================================== */
/*                         3. IR 检测参数                                      */
/* ========================================================================== */

#define EEPROM_IR_MAGIC     0x45495200UL       /* "EIR\0" */

/** @brief  单路 IR 检测参数 */
typedef struct {
    uint32_t    threshold;          /* ADC 原始值阈值 */
    uint32_t    hysteresis;         /* 回滞值 */
    uint32_t    filter_shift;       /* 滤波系数（右移位数，0=无滤波） */
    uint32_t    _reserved;          /* 预留 */
} AP_EEPROM_IR_ChanParam_t;

typedef struct {
    uint32_t    magic;
    uint32_t    version;
    uint32_t    length;
    AP_EEPROM_IR_ChanParam_t ch[3]; /* 3 路 IR 通道参数 */
    uint16_t    crc16;
    uint16_t    _pad;
} AP_EEPROM_IR_Param_t;

/* ========================================================================== */
/*                        默认值宏                                              */
/* ========================================================================== */

#define AP_EEPROM_UV_DEFAULT_SENS      5U
#define AP_EEPROM_UV_DEFAULT_THR_MIN   5U
#define AP_EEPROM_UV_DEFAULT_THR_MAX   35U
#define AP_EEPROM_UV_DEFAULT_WIN_MIN   1000U
#define AP_EEPROM_UV_DEFAULT_WIN_MAX   3500U
#define AP_EEPROM_UV_DEFAULT_CFM_MIN   0U
#define AP_EEPROM_UV_DEFAULT_CFM_MAX   3500U
#define AP_EEPROM_UV_DEFAULT_CLR_MIN   3000U
#define AP_EEPROM_UV_DEFAULT_CLR_MAX   10000U

#define AP_EEPROM_IR_DEFAULT_THRESHOLD      2000U
#define AP_EEPROM_IR_DEFAULT_HYSTERESIS     100U
#define AP_EEPROM_IR_DEFAULT_FILTER_SHIFT   2U

/* ========================================================================== */
/*                        公有 API                                             */
/* ========================================================================== */

/** @brief 初始化所有参数组（从 EEPROM 加载或写默认值）*/
int  AP_EEPROM_Init(void);

/* ---- ADC ---- */
const AP_EEPROM_ADC_Param_t *AP_EEPROM_ADC_Get(void);
int  AP_EEPROM_ADC_Save(const AP_EEPROM_ADC_Param_t *p);
int  AP_EEPROM_ADC_Reset(void);

/* ---- UV ---- */
const AP_EEPROM_UV_Param_t  *AP_EEPROM_UV_Get(void);
int  AP_EEPROM_UV_Save(const AP_EEPROM_UV_Param_t *p);
int  AP_EEPROM_UV_Reset(void);

/* ---- IR ---- */
const AP_EEPROM_IR_Param_t  *AP_EEPROM_IR_Get(void);
int  AP_EEPROM_IR_Save(const AP_EEPROM_IR_Param_t *p);
int  AP_EEPROM_IR_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __AP_EEPROM_H__ */
