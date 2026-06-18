#include "cmd.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "hal_uart.h"
#include "hal_led.h"
#include "ap_adc.h"
#include "ap_eeprom.h"
#include "ap_uv.h"
#include "ap_ir.h"
#include "main.h"
#include "ap_util.h"

/* ========================================================================== */
/*                          命令宏定义                                         */
/* ========================================================================== */

#define CMD_BUF_SIZE        128
#define CMD_MAX_ARGC        8
#define CMD_END             "\r\n"
#define CMD_END_LEN         (sizeof(CMD_END) - 1)

#define CMD_PRINTF(...)     BSP_UART_Printf(__VA_ARGS__)

/* ========================================================================== */
/*                          内部变量                                           */
/* ========================================================================== */

static char cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_index = 0;

/* ========================================================================== */
/*                          命令处理函数声明                                    */
/* ========================================================================== */

typedef void (*cmd_handler_t)(int argc, char **argv);

static void cmd_help(int argc, char **argv);
static void cmd_adc(int argc, char **argv);
static void cmd_uv(int argc, char **argv);
static void cmd_ir(int argc, char **argv);
static void cmd_param(int argc, char **argv);
static void cmd_state(int argc, char **argv);
static void cmd_reset(int argc, char **argv);
static void cmd_debug(int argc, char **argv);
static void cmd_mark(int argc, char **argv);
static void cmd_led(int argc, char **argv);
#if defined(IR_TEST_MODE)
static void cmd_uart(int argc, char **argv);
#endif
static void cmd_unknown(int argc, char **argv);

/* ========================================================================== */
/*                          命令表                                             */
/* ========================================================================== */

typedef struct {
    const char *name;
    cmd_handler_t handler;
} cmd_entry_t;

static const cmd_entry_t cmd_table[] = {
    {"help",   cmd_help},
    {"adc",    cmd_adc},
    {"uv",     cmd_uv},
    {"ir",     cmd_ir},
    {"param",  cmd_param},
    {"state",  cmd_state},
    {"reset",  cmd_reset},
    {"debug",  cmd_debug},
    {"mark",   cmd_mark},
    {"led",    cmd_led},
#if defined(IR_TEST_MODE)
    {"uart",   cmd_uart},
#endif /* IR_TEST_MODE */

    {NULL,     cmd_unknown},
};

/* ========================================================================== */
/*                          process_cmd_line — 解析 + 查表分发                 */
/* ========================================================================== */

static void process_cmd_line(const char *line)
{
    char *argv[CMD_MAX_ARGC];
    int argc = 0;
    char cmd_copy[CMD_BUF_SIZE];

    strncpy(cmd_copy, line, sizeof(cmd_copy));
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';

    char *token = strtok(cmd_copy, " ");
    while (token && argc < CMD_MAX_ARGC) {
        argv[argc++] = token;
        token = strtok(NULL, " ");
    }

    if (argc == 0)
        return;

    for (const cmd_entry_t *entry = cmd_table; entry->name; entry++) {
        if (strcmp(argv[0], entry->name) == 0) {
            entry->handler(argc, argv);
            return;
        }
    }

    cmd_unknown(argc, argv);
}

/* ========================================================================== */
/*                         公有 API                                            */
/* ========================================================================== */

void cmd_parser_task(void)
{
    uint8_t ch;

    while (BSP_UART_Read(BSP_UART_COM, &ch, 1) == 1) {
        if (cmd_index + 1 >= CMD_END_LEN) {
            if (memcmp(&cmd_buf[cmd_index + 1 - CMD_END_LEN], CMD_END, CMD_END_LEN - 1) == 0 &&
                ch == CMD_END[CMD_END_LEN - 1]) {
                cmd_buf[cmd_index + 1 - CMD_END_LEN] = '\0';
                process_cmd_line((char *)cmd_buf);
                cmd_index = 0;
                continue;
            }
        }

        if (cmd_index < CMD_BUF_SIZE - 1) {
            cmd_buf[cmd_index++] = ch;
        } else {
            cmd_index = 0;
        }
    }
}

/* ========================================================================== */
/*                          帮助命令                                            */
/* ========================================================================== */

static void cmd_help(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    CMD_PRINTF("Available commands:\r\n");
    CMD_PRINTF("  help                     — print this help\r\n");
    CMD_PRINTF("  param                    — show all module parameters\r\n");
    CMD_PRINTF("  param uv <field> [value] — UV parameter (sensitivity/minmax/...)\r\n");
    CMD_PRINTF("  param ir <field> [value] — IR parameter (sensitivity/pwr/r38/r50/freq/...)\r\n");
    CMD_PRINTF("  adc <ch>                 — read ADC channel raw value\r\n");
    CMD_PRINTF("  adc threshold <ch> <val> — set ADC threshold\r\n");
    CMD_PRINTF("  uv                       — print UV detector state\r\n");
    CMD_PRINTF("  ir                       — print IR state & features\r\n");
    CMD_PRINTF("  state                    — print system state\r\n");
    CMD_PRINTF("  reset                    — software reset MCU\r\n");
    CMD_PRINTF("  debug <on/off>           — toggle 100Hz test data print\r\n");
    CMD_PRINTF("  mark [text]              — insert scenario marker with optional text\r\n");
    CMD_PRINTF("  led work <ms>            — set LED heartbeat interval\r\n");
    CMD_PRINTF("  led blink <n> <ms>       — LED blink N times at interval\r\n");
    CMD_PRINTF("  led stop                 — stop LED, turn off\r\n");
#if defined(IR_TEST_MODE)
    CMD_PRINTF("  uart loop <n>            — COM loopback test, send N bytes\r\n");
    CMD_PRINTF("  uart recv                — print received COM data on DBG\r\n");
#endif
}

/* ========================================================================== */
/*                         cmd_param — 二级模块分发                             */
/*         param uv|ir <field> [value] / param adc threshold <ch> <val>       */
/* ========================================================================== */

static void cmd_param_uv(int argc, char **argv);
static void cmd_param_ir(int argc, char **argv);

static void cmd_param(int argc, char **argv)
{
    if (argc == 1) {
        const AP_EEPROM_UV_Param_t *uv = AP_EEPROM_UV_Get();
        const AP_EEPROM_IR_Param_t  *ir = AP_EEPROM_IR_Get();
        const AP_EEPROM_ADC_Param_t *adc = AP_EEPROM_ADC_Get();

        CMD_PRINTF("--- UV (min→max) ---\r\n");
        CMD_PRINTF("  level=%lu\r\n", (unsigned long)uv->sensitivity);
        CMD_PRINTF("  thr: %lu→%lu\r\n", (unsigned long)uv->thr_min, (unsigned long)uv->thr_max);
        CMD_PRINTF("  win: %lu→%lu\r\n", (unsigned long)uv->win_min, (unsigned long)uv->win_max);
        CMD_PRINTF("  cfm: %lu→%lu\r\n", (unsigned long)uv->cfm_min, (unsigned long)uv->cfm_max);
        CMD_PRINTF("  clr: %lu→%lu\r\n", (unsigned long)uv->clr_min, (unsigned long)uv->clr_max);
        CMD_PRINTF("--- IR (min→max) ---\r\n");
        CMD_PRINTF("  level=%lu\r\n", (unsigned long)ir->sensitivity);
        CMD_PRINTF("  pwr:  %lu→%lu (×1000)\r\n",
            (unsigned long)ir->pwr_min, (unsigned long)ir->pwr_max);
        CMD_PRINTF("  r38:  %lu→%lu (×1000)\r\n",
            (unsigned long)ir->r38_min, (unsigned long)ir->r38_max);
        CMD_PRINTF("  r50:  %lu→%lu (×1000)\r\n",
            (unsigned long)ir->r50_min, (unsigned long)ir->r50_max);
        CMD_PRINTF("  freq: low=%lu high=%lu (×10,固定)\r\n",
            (unsigned long)ir->freq_low_x10, (unsigned long)ir->freq_high_x10);
        CMD_PRINTF("  cfm:  %lu→%lu (ms)\r\n",
            (unsigned long)ir->cfm_min, (unsigned long)ir->cfm_max);
        CMD_PRINTF("--- ADC thresholds ---\r\n");
        CMD_PRINTF("  ch0=%lu ch1=%lu ch2=%lu\r\n",
            (unsigned long)adc->threshold[0],
            (unsigned long)adc->threshold[1],
            (unsigned long)adc->threshold[2]);
        return;
    }

    if (argc < 2) { CMD_PRINTF("Usage: param uv|ir|adc ...\r\n"); return; }

    if (strcmp(argv[1], "uv") == 0) {
        cmd_param_uv(argc - 1, argv + 1);
    } else if (strcmp(argv[1], "ir") == 0) {
        cmd_param_ir(argc - 1, argv + 1);
    } else if (strcmp(argv[1], "adc") == 0) {
        if (argc < 3) { CMD_PRINTF("Usage: param adc threshold <ch> <val>\r\n"); return; }
        CMD_PRINTF("(use 'adc threshold <ch> <val>')\r\n");
    } else {
        CMD_PRINTF("Unknown module '%s' (try uv/ir/adc)\r\n", argv[1]);
    }
}

/* =================================================================== */
/*   param uv <field> [value]                                         */
/* =================================================================== */

// /* 内部辅助：将 AP_UV 实时 min/max 覆盖回待保存的 EEPROM 结构体 */
// static void uv_sync_minmax(AP_EEPROM_UV_Param_t *p)
// {
//     AP_UV_GetConfig(&p->thr_min, &p->thr_max,
//                     &p->win_min, &p->win_max,
//                     &p->cfm_min, &p->cfm_max,
//                     &p->clr_min, &p->clr_max);
// }

static void cmd_param_uv(int argc, char **argv)
{
    const AP_EEPROM_UV_Param_t *uv = AP_EEPROM_UV_Get();

    if (argc == 1) {
        uint32_t thr_min, thr_max, win_min, win_max;
        uint32_t cfm_min, cfm_max, clr_min, clr_max;
        uint32_t thr, win, cfm, clr;
        AP_UV_GetConfig(&thr_min, &thr_max, &win_min, &win_max,
                        &cfm_min, &cfm_max, &clr_min, &clr_max);
        AP_UV_GetParams(&thr, &win, &cfm, &clr);

        CMD_PRINTF("UV level=%lu\r\n", (unsigned long)uv->sensitivity);
        CMD_PRINTF("         min→max    cur\r\n");
        CMD_PRINTF("  thr:   %lu→%-8lu %lu\r\n",
            (unsigned long)thr_min, (unsigned long)thr_max, (unsigned long)thr);
        CMD_PRINTF("  win:   %lu→%-8lu %lu\r\n",
            (unsigned long)win_min, (unsigned long)win_max, (unsigned long)win);
        CMD_PRINTF("  cfm:   %lu→%-8lu %lu\r\n",
            (unsigned long)cfm_min, (unsigned long)cfm_max, (unsigned long)cfm);
        CMD_PRINTF("  clr:   %lu→%-8lu %lu\r\n",
            (unsigned long)clr_min, (unsigned long)clr_max, (unsigned long)clr);
        return;
    }

    if (strcmp(argv[1], "sensitivity") == 0) {
        if (argc == 2) {
            CMD_PRINTF("sensitivity=%lu\r\n", (unsigned long)uv->sensitivity);
        } else {
            uint32_t lv = strtoul(argv[2], NULL, 0);
            if (lv > 9) lv = 9;

            AP_UV_SetLevel((uint8_t)lv);          /* 插值计算所有参数 */

            AP_EEPROM_UV_Param_t p = *uv;
            p.sensitivity = lv;
            if (AP_EEPROM_UV_Save(&p) == 0) {
                CMD_PRINTF("sensitivity=%lu saved\r\n", (unsigned long)p.sensitivity);
            } else { CMD_PRINTF("save failed\r\n"); }
        }
        return;
    }

    if (strcmp(argv[1], "minmax") == 0) {
        uint32_t thr_min, thr_max, win_min, win_max;
        uint32_t cfm_min, cfm_max, clr_min, clr_max;
        AP_UV_GetConfig(&thr_min, &thr_max, &win_min, &win_max,
                        &cfm_min, &cfm_max, &clr_min, &clr_max);
        CMD_PRINTF("thr  %lu %lu\r\n", (unsigned long)thr_min, (unsigned long)thr_max);
        CMD_PRINTF("win  %lu %lu\r\n", (unsigned long)win_min, (unsigned long)win_max);
        CMD_PRINTF("cfm  %lu %lu\r\n", (unsigned long)cfm_min, (unsigned long)cfm_max);
        CMD_PRINTF("clr  %lu %lu\r\n", (unsigned long)clr_min, (unsigned long)clr_max);
        return;
    }

    /* 修改 min/max 值 */
    {
        int found = 1;
        AP_EEPROM_UV_Param_t p = *uv;
        const char *field = argv[1];

        if (strcmp(field, "thr_min") == 0 && argc > 2) { p.thr_min = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "thr_max") == 0 && argc > 2) { p.thr_max = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "win_min") == 0 && argc > 2) { p.win_min = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "win_max") == 0 && argc > 2) { p.win_max = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "cfm_min") == 0 && argc > 2) { p.cfm_min = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "cfm_max") == 0 && argc > 2) { p.cfm_max = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "clr_min") == 0 && argc > 2) { p.clr_min = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "clr_max") == 0 && argc > 2) { p.clr_max = strtoul(argv[2], NULL, 0); }
        else { found = 0; }

        if (found) {
            AP_UV_SetConfig(p.thr_min, p.thr_max, p.win_min, p.win_max,
                            p.cfm_min, p.cfm_max, p.clr_min, p.clr_max);
            AP_UV_SetLevel((uint8_t)p.sensitivity);

            if (AP_EEPROM_UV_Save(&p) == 0) {
                CMD_PRINTF("%s saved (set level to re-calc)\r\n", field);
            } else { CMD_PRINTF("save failed\r\n"); }
            return;
        }
    }

    CMD_PRINTF("Unknown UV field: %s (try sensitivity/minmax/thr_min/thr_max/win_min/...)\r\n", argv[1]);
}

/* =================================================================== */
/*   param ir <field> [value] — 多光谱融合参数                          */
/* =================================================================== */

static void cmd_param_ir(int argc, char **argv)
{
    const AP_EEPROM_IR_Param_t *ir = AP_EEPROM_IR_Get();

    if (argc == 1) {
        uint32_t pwr_min, pwr_max, r38_min, r38_max;
        uint32_t r50_min, r50_max, freq_low, freq_high;
        uint32_t cfm_min, cfm_max;
        uint32_t pwr, r38, r50, cfm;
        AP_IR_GetConfig(&pwr_min, &pwr_max, &r38_min, &r38_max,
                        &r50_min, &r50_max, &freq_low, &freq_high,
                        &cfm_min, &cfm_max);
        AP_IR_GetParams(&pwr, &r38, &r50, &freq_low, &freq_high, &cfm);

        CMD_PRINTF("IR level=%lu\r\n", (unsigned long)ir->sensitivity);
        CMD_PRINTF("         min→max    cur\r\n");
        CMD_PRINTF("  pwr:   %lu→%-8lu %lu\r\n",
            (unsigned long)pwr_min, (unsigned long)pwr_max, (unsigned long)pwr);
        CMD_PRINTF("  r38:   %lu→%-8lu %lu\r\n",
            (unsigned long)r38_min, (unsigned long)r38_max, (unsigned long)r38);
        CMD_PRINTF("  r50:   %lu→%-8lu %lu\r\n",
            (unsigned long)r50_min, (unsigned long)r50_max, (unsigned long)r50);
        CMD_PRINTF("  freq:  low=%lu high=%lu (×10,固定)\r\n",
            (unsigned long)freq_low, (unsigned long)freq_high);
        CMD_PRINTF("  cfm:   %lu→%-8lu %lu\r\n",
            (unsigned long)cfm_min, (unsigned long)cfm_max, (unsigned long)cfm);
        return;
    }

    if (strcmp(argv[1], "sensitivity") == 0) {
        if (argc == 2) {
            CMD_PRINTF("sensitivity=%lu\r\n", (unsigned long)ir->sensitivity);
        } else {
            uint32_t lv = strtoul(argv[2], NULL, 0);
            if (lv > 9) lv = 9;

            AP_IR_SetLevel((uint8_t)lv);

            AP_EEPROM_IR_Param_t p = *ir;
            p.sensitivity = lv;
            if (AP_EEPROM_IR_Save(&p) == 0) {
                CMD_PRINTF("sensitivity=%lu saved\r\n", (unsigned long)p.sensitivity);
            } else { CMD_PRINTF("save failed\r\n"); }
        }
        return;
    }

    if (strcmp(argv[1], "minmax") == 0) {
        uint32_t pwr_min, pwr_max, r38_min, r38_max;
        uint32_t r50_min, r50_max, freq_low, freq_high;
        uint32_t cfm_min, cfm_max;
        AP_IR_GetConfig(&pwr_min, &pwr_max, &r38_min, &r38_max,
                        &r50_min, &r50_max, &freq_low, &freq_high,
                        &cfm_min, &cfm_max);
        CMD_PRINTF("pwr  %lu %lu\r\n", (unsigned long)pwr_min, (unsigned long)pwr_max);
        CMD_PRINTF("r38  %lu %lu\r\n", (unsigned long)r38_min, (unsigned long)r38_max);
        CMD_PRINTF("r50  %lu %lu\r\n", (unsigned long)r50_min, (unsigned long)r50_max);
        CMD_PRINTF("freq  %lu %lu\r\n", (unsigned long)freq_low, (unsigned long)freq_high);
        CMD_PRINTF("cfm  %lu %lu\r\n", (unsigned long)cfm_min, (unsigned long)cfm_max);
        return;
    }

    /* 修改 min/max 值 + 固定频率 */
    {
        int found = 1;
        AP_EEPROM_IR_Param_t p = *ir;
        const char *field = argv[1];

        if (strcmp(field, "pwr_min") == 0 && argc > 2)      { p.pwr_min   = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "pwr_max") == 0 && argc > 2)  { p.pwr_max   = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "r38_min") == 0 && argc > 2)  { p.r38_min   = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "r38_max") == 0 && argc > 2)  { p.r38_max   = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "r50_min") == 0 && argc > 2)  { p.r50_min   = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "r50_max") == 0 && argc > 2)  { p.r50_max   = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "freq_low") == 0 && argc > 2) { p.freq_low_x10  = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "freq_high") == 0 && argc > 2){ p.freq_high_x10 = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "cfm_min") == 0 && argc > 2)  { p.cfm_min   = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "cfm_max") == 0 && argc > 2)  { p.cfm_max   = strtoul(argv[2], NULL, 0); }
        else { found = 0; }

        if (found) {
            AP_IR_SetConfig(p.pwr_min, p.pwr_max, p.r38_min, p.r38_max,
                            p.r50_min, p.r50_max,
                            p.freq_low_x10, p.freq_high_x10,
                            p.cfm_min, p.cfm_max);
            AP_IR_SetLevel((uint8_t)p.sensitivity);

            if (AP_EEPROM_IR_Save(&p) == 0) {
                CMD_PRINTF("%s saved\r\n", field);
            } else { CMD_PRINTF("save failed\r\n"); }
            return;
        }
    }

    CMD_PRINTF("Unknown IR field: %s (try sensitivity/minmax/pwr_min/pwr_max/r38_min/...)\r\n", argv[1]);
}

/* ========================================================================== */
/*                         uv — 紫外检测状态                                    */
/* ========================================================================== */

static void cmd_uv(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    const char *s;
    switch (AP_UV_GetState()) {
        case UV_STATE_IDLE:    s = "IDLE";    break;
        case UV_STATE_WARNING: s = "WARNING"; break;
        case UV_STATE_FIRE:    s = "FIRE";    break;
        default:               s = "?";       break;
    }
    CMD_PRINTF("UV state: %s\r\n", s);
}

/* ========================================================================== */
/*                         ir — 红外三波段检测状态 + 实时特征                   */
/* ========================================================================== */

static void cmd_ir(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    const char *s;
    switch (AP_IR_GetState()) {
        case IR_STATE_IDLE:    s = "IDLE";    break;
        case IR_STATE_WARNING: s = "WARNING"; break;
        case IR_STATE_FIRE:    s = "FIRE";    break;
        default:               s = "?";       break;
    }
    CMD_PRINTF("IR state: %s\r\n", s);

    uint32_t power[3];
    float    zcr[3];
    uint32_t r45_38, r45_50;
    AP_IR_GetFeatures(power, zcr, &r45_38, &r45_50);

    CMD_PRINTF("  P[38]=%lu  P[45]=%lu  P[50]=%lu\r\n",
        (unsigned long)power[0], (unsigned long)power[1], (unsigned long)power[2]);
    CMD_PRINTF("  ZCR[38]=%.1fHz  ZCR[45]=%.1fHz  ZCR[50]=%.1fHz\r\n",
        (double)zcr[0], (double)zcr[1], (double)zcr[2]);
    CMD_PRINTF("  R45/38=%lu.%03lu  R45/50=%lu.%03lu\r\n",
        (unsigned long)(r45_38 / 1000), (unsigned long)(r45_38 % 1000),
        (unsigned long)(r45_50 / 1000), (unsigned long)(r45_50 % 1000));
}

/* ========================================================================== */
/*                         adc — ADC 阈值设置                                   */
/* ========================================================================== */

static void cmd_adc(int argc, char **argv)
{
    const AP_EEPROM_ADC_Param_t *adc = AP_EEPROM_ADC_Get();

    if (argc == 1) {
        for (uint32_t i = 0; i < 3; i++) {
            CMD_PRINTF("ADC%lu threshold: %lu\r\n",
                (unsigned long)i, (unsigned long)adc->threshold[i]);
        }
        return;
    }

    /* adc threshold <ch> <val> */
    if (argc == 4 && strcmp(argv[1], "threshold") == 0) {
        uint32_t ch = (uint32_t)strtoul(argv[2], NULL, 0);
        if (ch > 2) {
            CMD_PRINTF("ch must be 0~2\r\n");
            return;
        }
        AP_EEPROM_ADC_Param_t p = *adc;
        p.threshold[ch] = (uint32_t)strtoul(argv[3], NULL, 0);
        if (AP_EEPROM_ADC_Save(&p) == 0) {
            CMD_PRINTF("ADC%lu threshold=%lu saved\r\n",
                (unsigned long)ch, (unsigned long)p.threshold[ch]);
        } else {
            CMD_PRINTF("save failed\r\n");
        }
        return;
    }

    CMD_PRINTF("Usage: adc threshold <ch=0~2> <value>\r\n");
}

/* ========================================================================== */
/*                         状态命令                                            */
/* ========================================================================== */

static void cmd_state(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    const AP_EEPROM_UV_Param_t *uv = AP_EEPROM_UV_Get();
    const AP_EEPROM_IR_Param_t  *ir = AP_EEPROM_IR_Get();
    const AP_EEPROM_ADC_Param_t *adc = AP_EEPROM_ADC_Get();
    uint32_t tick = HAL_GetTick();

    uint32_t uv_thr, uv_win, uv_cfm, uv_clr;
    AP_UV_GetParams(&uv_thr, &uv_win, &uv_cfm, &uv_clr);

    CMD_PRINTF("System state (uptime=%lums):\r\n", (unsigned long)tick);
    CMD_PRINTF("  UV: st=%u lv=%lu thr=%lu win=%lu cfm=%lu clr=%lu\r\n",
        (unsigned)AP_UV_GetState(),
        (unsigned long)uv->sensitivity, (unsigned long)uv_thr,
        (unsigned long)uv_win, (unsigned long)uv_cfm, (unsigned long)uv_clr);
    uint32_t ir_pwr, ir_r38, ir_r50, ir_flow, ir_fhigh, ir_cfm;
    AP_IR_GetParams(&ir_pwr, &ir_r38, &ir_r50, &ir_flow, &ir_fhigh, &ir_cfm);

    CMD_PRINTF("  IR: st=%u lv=%lu pwr=%lu r38=%lu r50=%lu freq=%lu/%lu cfm=%lu\r\n",
        (unsigned)AP_IR_GetState(),
        (unsigned long)ir->sensitivity, (unsigned long)ir_pwr,
        (unsigned long)ir_r38, (unsigned long)ir_r50,
        (unsigned long)ir_flow, (unsigned long)ir_fhigh, (unsigned long)ir_cfm);
    CMD_PRINTF("  ADC thr: %lu/%lu/%lu\r\n",
        (unsigned long)adc->threshold[0],
        (unsigned long)adc->threshold[1],
        (unsigned long)adc->threshold[2]);
}

/* ========================================================================== */
/*                         reset                                                */
/* ========================================================================== */

static void cmd_reset(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    CMD_PRINTF("resetting...\r\n");
    NVIC_SystemReset();
}

/* ========================================================================== */
/*                         debug                                                */
/* ========================================================================== */

static void cmd_debug(int argc, char **argv)
{
#if defined(IR_TEST_MODE)
    if (argc < 2) {
        CMD_PRINTF("debug=%s\r\n", TEST_GetPrintEnabled() ? "on" : "off");
        return;
    }
    if (strcmp(argv[1], "on") == 0) {
        TEST_SetPrintEnabled(1);
        CMD_PRINTF("debug on\r\n");
    } else if (strcmp(argv[1], "off") == 0) {
        TEST_SetPrintEnabled(0);
        CMD_PRINTF("debug off\r\n");
    } else {
        CMD_PRINTF("Usage: debug on|off\r\n");
    }
#else
    (void)argc;
    (void)argv;
    CMD_PRINTF("debug: only available in IR_TEST_MODE\r\n");
#endif
}

/* ========================================================================== */
/*                         mark — 测试场景分隔标记                              */
/* ========================================================================== */

static void cmd_mark(int argc, char **argv)
{
#if defined(IR_TEST_MODE)
    const char *msg = (argc > 1) ? argv[1] : NULL;
    TEST_InsertMarker(msg);
#else
    (void)argc;
    (void)argv;
    CMD_PRINTF("mark: only available in IR_TEST_MODE\r\n");
#endif
}

/* ========================================================================== */
/*                         led — LED 控制                                      */
/* ========================================================================== */

static void cmd_led(int argc, char **argv)
{
    if (argc < 2) {
        CMD_PRINTF("Usage: led work <ms> | blink <n> <ms> | stop\r\n");
        return;
    }

    if (strcmp(argv[1], "work") == 0 && argc > 2) {
        uint32_t ms = strtoul(argv[2], NULL, 0);
        if (ms < 10) ms = 10;
        BSP_LED_Work(ms);
        CMD_PRINTF("LED work interval=%lums\r\n", (unsigned long)ms);
    } else if (strcmp(argv[1], "blink") == 0 && argc > 3) {
        uint32_t n  = strtoul(argv[2], NULL, 0);
        uint32_t ms = strtoul(argv[3], NULL, 0);
        if (ms < 10) ms = 10;
        if (n > 0) {
            BSP_LED_Blink(n, ms);
            CMD_PRINTF("LED blink %lu times, interval=%lums\r\n",
                (unsigned long)n, (unsigned long)ms);
        }
    } else if (strcmp(argv[1], "stop") == 0) {
        BSP_LED_Stop();
        CMD_PRINTF("LED stopped\r\n");
    } else {
        CMD_PRINTF("Usage: led work <ms> | blink <n> <ms> | stop\r\n");
    }
}

/* ========================================================================== */
/*                         uart — 串口通信测试                                 */
/* ========================================================================== */
#if defined(IR_TEST_MODE)
static void cmd_uart(int argc, char **argv)
{
    if (argc < 2) {
        CMD_PRINTF("Usage: uart loop <n> | recv\r\n");
        return;
    }

    if (strcmp(argv[1], "loop") == 0) {
        uint32_t n = (argc > 2) ? strtoul(argv[2], NULL, 0) : 16;
        if (n > 128) n = 128;
        uint8_t buf[128];
        for (uint32_t i = 0; i < n; i++) buf[i] = (uint8_t)(i & 0xFF);
        BSP_UART_Write(BSP_UART_COM, buf, (uint16_t)n);
        BSP_UART_Printf("[UART] loopback %lu bytes sent on COM\r\n", (unsigned long)n);

    } else if (strcmp(argv[1], "recv") == 0) {
        uint8_t buf[64];
        uint16_t len = BSP_UART_Read(BSP_UART_COM, buf, sizeof(buf));
        if (len > 0) {
            BSP_UART_Printf("[UART] COM recv %u bytes:\r\n  HEX: ", (unsigned)len);
            for (uint16_t i = 0; i < len; i++) {
                BSP_UART_Printf("%02X ", buf[i]);
            }
            BSP_UART_Printf("\r\n  ASC: ");
            for (uint16_t i = 0; i < len; i++) {
                BSP_UART_Printf("%c", (buf[i] >= 0x20 && buf[i] < 0x7F) ? buf[i] : '.');
            }
            BSP_UART_Printf("\r\n");
        } else {
            BSP_UART_Printf("[UART] COM no data\r\n");
        }
    } else {
        CMD_PRINTF("Usage: uart loop <n> | recv\r\n");
    }
}
#endif
/* ========================================================================== */
/*                         未知命令                                             */
/* ========================================================================== */

static void cmd_unknown(int argc, char **argv)
{
    CMD_PRINTF("Unknown command: %s\r\n", argv[0]);
    CMD_PRINTF("Type 'help' for available commands\r\n");
}
