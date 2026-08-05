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
#define EEPROM_SECTOR_SYSTEM (EEPROM_BASE + 3 * EEPROM_SECTOR_SIZE)  /* 0x080800C0 */

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

/* v3移除随等级变化的消警时间；旧v2魔数会在初始化时触发默认参数重建。 */
#define EEPROM_UV_MAGIC     0x45555633UL       /* "EUV3" — version 3 */

typedef struct {
    uint32_t    magic;
    uint32_t    version;            /* 3 */
    uint32_t    length;
    uint32_t    sensitivity;        /* 当前等级：0最灵敏，9最迟钝 */
    uint32_t    thr_min;            /* threshold 等级0 */
    uint32_t    thr_max;            /* threshold 等级9 */
    uint32_t    win_min;            /* window 等级0 */
    uint32_t    win_max;            /* window 等级9 */
    uint32_t    cfm_min;            /* confirm 等级0 */
    uint32_t    cfm_max;            /* confirm 等级9 */
    uint32_t    pw_min_us;          /* UV 最小脉宽(µs)  */
    uint32_t    pw_max_us;          /* UV 最大脉宽(µs)  */
    uint32_t    print_window_ms;    /* 测试打印窗口(ms)  */
    uint16_t    crc16;
    uint16_t    _pad;
} AP_EEPROM_UV_Param_t;

/* ========================================================================== */
/*                         3. IR 检测参数 (多光谱融合算法)                     */
/* ========================================================================== */

/*
 * v7将4.5um进场功率由固定值改为等级0/9两个端点。
 * ap_eeprom.c负责把有效v6参数迁移到v7，并保留现场标定的ZCR死区。
 */
#define EEPROM_IR_MAGIC     0x45495237UL       /* "EIR7" - version 7 */

/**
  @brief  IR 多光谱融合检测参数
          4.5um进场功率和确认时长以 min/max 范围存储，等级0~9线性插值。
          光谱比阈值和频率范围为固定值，不随等级变化。

          缩放约定:
            pwr_xxx = 去直流信号均方值
            r38/r50 = 光谱比 ×1000          (e.g. 1500 = 1.500)
            freq_xxx = 频率 ×10             (e.g. 15 = 1.5Hz)
            cfm = 确认时长 毫秒
*/
typedef struct {
    uint32_t    magic;              /* EEPROM_IR_MAGIC       */
    uint32_t    version;            /* 7                     */
    uint32_t    length;             /* sizeof                 */
    uint32_t    sensitivity;        /* 0最灵敏，9最迟钝       */
    uint32_t    power_min;          /* 4.5um进场阈值，等级0最灵敏 */
    uint32_t    power_max;          /* 4.5um进场阈值，等级9最迟钝 */
    uint32_t    r38_threshold;      /* R4.5/3.8(×1000) 固定   */
    uint32_t    r50_threshold;      /* R4.5/5.0(×1000) 固定   */
    uint32_t    freq_low_x10;       /* 频率下限(×10)   固定  */
    uint32_t    freq_high_x10;      /* 频率上限(×10)   固定  */
    uint32_t    cfm_min;            /* 确认时长(ms)    等级0  */
    uint32_t    cfm_max;            /* 确认时长(ms)    等级9  */
    uint32_t    zcr_dead_zone[3];    /* 3.8/4.5/5.0通道固定死区 */
    uint16_t    crc16;
    uint16_t    _pad;
} AP_EEPROM_IR_Param_t;

/* ========================================================================== */
/*                         4. 系统运行参数                                     */
/* ========================================================================== */

#define EEPROM_SYSTEM_MAGIC 0x45535931UL       /* "ESY1" - version 1 */

typedef struct {
    uint32_t    magic;
    uint32_t    version;            /* 1 */
    uint32_t    length;
    uint32_t    show_mode;          /* 0: UV&&IR正常模式，1: 仅UV演示模式 */
    uint16_t    crc16;
    uint16_t    _pad;
} AP_EEPROM_System_Param_t;

/* ========================================================================== */
/*                        默认值宏                                              */
/* ========================================================================== */

#define AP_EEPROM_SYSTEM_DEFAULT_SHOW_MODE 0U

#define AP_EEPROM_UV_DEFAULT_SENS      5U
#define AP_EEPROM_UV_DEFAULT_THR_MIN   5U
#define AP_EEPROM_UV_DEFAULT_THR_MAX   24U      /* 3.5秒火盆主分布下沿24；80%迟滞下限为20 */
#define AP_EEPROM_UV_DEFAULT_WIN_MIN   1000U
#define AP_EEPROM_UV_DEFAULT_WIN_MAX   3500U
#define AP_EEPROM_UV_DEFAULT_CFM_MIN   0U
#define AP_EEPROM_UV_DEFAULT_CFM_MAX   3500U
#define AP_EEPROM_UV_DEFAULT_PW_MIN_US     6000U
#define AP_EEPROM_UV_DEFAULT_PW_MAX_US     14000U
#define AP_EEPROM_UV_DEFAULT_PRINT_WIN_MS  2000U

/* IR 多光谱融合默认值 (中灵敏度, 对应文档Ⅱ级) */
#define AP_EEPROM_IR_DEFAULT_SENS      5U
#define AP_EEPROM_IR_DEFAULT_POWER_MIN 12000U   /* 等级0：兼顾远距离弱火与背景裕量 */
#define AP_EEPROM_IR_DEFAULT_POWER_MAX 22000U   /* 等级9：不超过实测弱火22600峰值 */
#define AP_EEPROM_IR_DEFAULT_R38       1500U    /* 1.5  ×1000，固定 */
#define AP_EEPROM_IR_DEFAULT_R50       1300U    /* 1.3  ×1000，固定 */
#define AP_EEPROM_IR_DEFAULT_FREQ_LOW  10U      /* 1.0Hz ×10 (固定) */
#define AP_EEPROM_IR_DEFAULT_FREQ_HIGH 200U     /* 20Hz ×10 (固定) */
#define AP_EEPROM_IR_DEFAULT_CFM_MIN   200U     /* 200ms */
#define AP_EEPROM_IR_DEFAULT_CFM_MAX   3000U    /* 3000ms */
#define AP_EEPROM_IR_DEFAULT_DZ_38     15U      /* 3.8um固定ZCR死区 */
#define AP_EEPROM_IR_DEFAULT_DZ_45     15U      /* 4.5um固定ZCR死区 */
#define AP_EEPROM_IR_DEFAULT_DZ_50     10U      /* 5.0um固定ZCR死区 */

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

/* ---- System ---- */
const AP_EEPROM_System_Param_t *AP_EEPROM_System_Get(void);
int  AP_EEPROM_System_Save(const AP_EEPROM_System_Param_t *p);
int  AP_EEPROM_System_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __AP_EEPROM_H__ */
