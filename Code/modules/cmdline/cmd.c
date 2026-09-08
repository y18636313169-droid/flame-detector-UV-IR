#include "cmd.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "hal_uart.h"
#include "hal_led.h"
#include "hal_tim.h"
#include "ap_adc.h"
#include "ap_eeprom.h"
#include "ap_uv.h"
#include "ap_ir.h"
#include "ap_uart_protocol.h"
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
static void cmd_gpio(int argc, char **argv);
static void cmd_adc(int argc, char **argv);
static void cmd_uv(int argc, char **argv);
static void cmd_ir(int argc, char **argv);
static void cmd_param(int argc, char **argv);
static void cmd_state(int argc, char **argv);
static void cmd_reset(int argc, char **argv);
static void cmd_debug(int argc, char **argv);
static void cmd_mark(int argc, char **argv);
static void cmd_led(int argc, char **argv);
static void cmd_uart(int argc, char **argv);
static void cmd_show(int argc, char **argv);
static void cmd_test(int argc, char **argv);
static void cmd_profile(int argc, char **argv);
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
    {"gpio",   cmd_gpio},
    {"adc",    cmd_adc},
    {"uv",     cmd_uv},
    {"ir",     cmd_ir},
    {"param",  cmd_param},
    {"state",  cmd_state},
    {"reset",  cmd_reset},
    {"debug",  cmd_debug},
    {"mark",   cmd_mark},
    {"led",    cmd_led},
    {"uart",   cmd_uart},
    {"show",   cmd_show},
    {"test",   cmd_test},
    {"profile", cmd_profile},

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

    while (BSP_UART_Read(BSP_UART_DBG, &ch, 1) == 1) {
        if ((uint32_t)cmd_index + 1U >= (uint32_t)CMD_END_LEN) {
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
    CMD_PRINTF("  param uv <field> [value] — UV parameter (level 0 most sensitive, 9 least)\r\n");
    CMD_PRINTF("  param ir <field> [value] — IR parameter (level 0 most sensitive, 9 least)\r\n");
    CMD_PRINTF("    IR power fields: power_min / power_max\r\n");
    CMD_PRINTF("  adc <ch>                 — read ADC channel raw value\r\n");
    CMD_PRINTF("  adc threshold <ch> <val> — set ADC threshold\r\n");
    CMD_PRINTF("  uv                       — print UV detector state\r\n");
    CMD_PRINTF("  ir                       — print IR state & features\r\n");
    CMD_PRINTF("  state                    — print system state\r\n");
    CMD_PRINTF("  reset                    — software reset MCU\r\n");
    CMD_PRINTF("  debug <on/off>           — toggle test data print\r\n");
    CMD_PRINTF("  debug ir win <sec>       — set IR energy average window\r\n");
    CMD_PRINTF("  mark [text]              — insert scenario marker with optional text\r\n");
    CMD_PRINTF("  led work <ms>            — set LED heartbeat interval\r\n");
    CMD_PRINTF("  led blink <n> <ms>       — LED blink N times at interval\r\n");
    CMD_PRINTF("  led stop                 — stop LED, turn off\r\n");
    CMD_PRINTF("  test mode [on|off]       — persistent runtime test mode\r\n");
    CMD_PRINTF("  show mode [on|off]       — persistent UV-only demonstration mode\r\n");
    CMD_PRINTF("  profile [on|off]         — persistent IR ignition-profile switch\r\n");
    CMD_PRINTF("  uart raw <on/off>        — pause/resume protocol for COM test\r\n");
    CMD_PRINTF("  uart loop <n>            — send N raw bytes on COM\r\n");
    CMD_PRINTF("  uart recv                — print raw COM data on DBG\r\n");
}

/* ========================================================================== */
/*                         cmd_param — 二级模块分发                             */
/*         param uv|ir <field> [value] / param adc threshold <ch> <val>       */
/* ========================================================================== */

static void cmd_param_uv(int argc, char **argv);
static void cmd_param_ir(int argc, char **argv);


/* ========================================================================== */
/*                         gpio — GPIO 读写                                   */
/* ========================================================================== */

static void cmd_gpio(int argc, char **argv)
{
    if (argc < 4) {
        CMD_PRINTF("Usage:\r\n");
        CMD_PRINTF("  gpio write <port> <pin> <0/1>             — set pin\r\n");
        CMD_PRINTF("  gpio read <port> <pin>                    — read pin\r\n");
        CMD_PRINTF("  gpio toggle <port> <pin>                  — toggle pin\r\n");
        CMD_PRINTF("  port: A/B/C/D/E/F/G/H (GPIOA~H)\r\n");
        CMD_PRINTF("  pin:  0-15\r\n");
        return;
    }

    GPIO_TypeDef *port;
    char port_char = argv[2][0] | 0x20; /* to lower */
    if (port_char == 'a') port = GPIOA;
    else if (port_char == 'b') port = GPIOB;
    else if (port_char == 'c') port = GPIOC;
    else if (port_char == 'd') port = GPIOD;
    else if (port_char == 'e') port = GPIOE;
    else if (port_char == 'f') port = GPIOF;
    else if (port_char == 'g') port = GPIOG;
    else if (port_char == 'h') port = GPIOH;
    else { CMD_PRINTF("gpio: invalid port '%s'\r\n", argv[2]); return; }

    uint16_t pin = (uint16_t)(1 << atoi(argv[3]));
    if (pin == 0 || atoi(argv[3]) > 15) {
        CMD_PRINTF("gpio: invalid pin %s\r\n", argv[3]);
        return;
    }

    if (strcmp(argv[1], "write") == 0 && argc >= 5) {
        bool state = (atoi(argv[4]) != 0);
        HAL_GPIO_WritePin(port, pin, state);
        CMD_PRINTF("gpio: %c%d=%s\r\n", port_char - 0x20, atoi(argv[3]),
            state ? "HIGH" : "LOW");
    } else if (strcmp(argv[1], "read") == 0) {
        bool val = HAL_GPIO_ReadPin(port, pin);
        CMD_PRINTF("gpio: %c%d=%s\r\n", port_char - 0x20, atoi(argv[3]),
            val ? "HIGH" : "LOW");
    } else if (strcmp(argv[1], "toggle") == 0) {
        HAL_GPIO_TogglePin(port, pin);
        CMD_PRINTF("gpio: %c%d toggled\r\n", port_char - 0x20, atoi(argv[3]));
    } else {
        CMD_PRINTF("gpio: check 'help gpio'\r\n");
    }
}

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
        CMD_PRINTF("  fire timeout: %lu ms (fixed)\r\n", (unsigned long)AP_FIRE_AUTO_CLEAR_MS);
        CMD_PRINTF("--- IR parameters ---\r\n");
        CMD_PRINTF("  level=%lu\r\n", (unsigned long)ir->sensitivity);
        CMD_PRINTF("  power: %lu->%lu (level 0->9)\r\n",
            (unsigned long)ir->power_min,
            (unsigned long)ir->power_max);
        CMD_PRINTF("  r38:  %lu (×1000, fixed)\r\n",
            (unsigned long)ir->r38_threshold);
        CMD_PRINTF("  r50:  %lu (×1000, fixed)\r\n",
            (unsigned long)ir->r50_threshold);
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
    }
    else if ((strcmp(argv[1], "sensitivity") == 0)) {
        const AP_EEPROM_UV_Param_t *uv = AP_EEPROM_UV_Get();
        const AP_EEPROM_IR_Param_t *ir = AP_EEPROM_IR_Get();
        if (argc == 2)
        {
            CMD_PRINTF("uv sensitivity:%lu\r\n", (unsigned long)uv->sensitivity);
            CMD_PRINTF("ir sensitivity:%lu\r\n", (unsigned long)ir->sensitivity);
        }
        else {
            uint32_t lv = strtoul(argv[2], NULL, 0);
            if (lv > 9U) {
                CMD_PRINTF("sensitivity must be 0~9\r\n");
                return;
            }
            
            AP_EEPROM_UV_Param_t p = *uv;
            p.sensitivity = lv;
            /* 先通过EEPROM统一校验并保存，成功后再切换运行等级。 */
            if (AP_EEPROM_UV_Save(&p) == 0) {
                AP_UV_SetLevel((uint8_t)lv);
                CMD_PRINTF("uv sensitivity=%lu saved\r\n", (unsigned long)p.sensitivity);
            } else { CMD_PRINTF("invalid uv parameter or save failed\r\n"); }

            
            AP_EEPROM_IR_Param_t q = *ir;
            q.sensitivity = lv;
            /* 先保存后应用，避免Flash失败时RAM等级与掉电参数不一致。 */
            if (AP_EEPROM_IR_Save(&q) == 0) {
                AP_IR_SetLevel((uint8_t)lv);
                CMD_PRINTF("ir sensitivity=%lu saved\r\n", (unsigned long)q.sensitivity);
            } else { CMD_PRINTF("invalid ir parameter or save failed\r\n"); }
        }
    } 
    else {
        CMD_PRINTF("Unknown module '%s' (try uv/ir/adc)\r\n", argv[1]);
    }
}

/* =================================================================== */
/*   param uv <field> [value]                                         */
/* =================================================================== */

static void cmd_param_uv(int argc, char **argv)
{
    const AP_EEPROM_UV_Param_t *uv = AP_EEPROM_UV_Get();

    if (argc == 1) {
        uint32_t thr_min, thr_max, win_min, win_max;
        uint32_t cfm_min, cfm_max;
        uint32_t thr, win, cfm, fire_timeout;
        AP_UV_GetConfig(&thr_min, &thr_max, &win_min, &win_max,
                        &cfm_min, &cfm_max);
        AP_UV_GetParams(&thr, &win, &cfm, &fire_timeout);

        CMD_PRINTF("UV level=%lu (0 most sensitive, 9 least)\r\n",
            (unsigned long)uv->sensitivity);
        CMD_PRINTF("         configured values\r\n");
        CMD_PRINTF("  thr:   %lu→%-8lu %lu\r\n",
            (unsigned long)thr_min, (unsigned long)thr_max, (unsigned long)thr);
        CMD_PRINTF("  win:   %lu→%-8lu %lu\r\n",
            (unsigned long)win_min, (unsigned long)win_max, (unsigned long)win);
        CMD_PRINTF("  cfm:   %lu→%-8lu %lu\r\n",
            (unsigned long)cfm_min, (unsigned long)cfm_max, (unsigned long)cfm);
        /* UV掉线下限按当前整数阈值的80%向上取整，与状态机计算一致。 */
        uint32_t drop_threshold = (thr * UV_DROP_OFF_PERCENT + 99U) / 100U;
        CMD_PRINTF("  drop:  off=%lu (%u%%) timeout=%lu ms (fixed)\r\n",
            (unsigned long)drop_threshold, (unsigned int)UV_DROP_OFF_PERCENT,
            (unsigned long)UV_DROPOUT_MS);
        CMD_PRINTF("  fire timeout: %lu ms (fixed)\r\n", (unsigned long)fire_timeout);
        return;
    }

    if (strcmp(argv[1], "sensitivity") == 0) {
        if (argc == 2) {
            CMD_PRINTF("sensitivity=%lu\r\n", (unsigned long)uv->sensitivity);
        } else {
            uint32_t lv = strtoul(argv[2], NULL, 0);
            if (lv > 9U) {
                CMD_PRINTF("sensitivity must be 0~9\r\n");
                return;
            }

            AP_EEPROM_UV_Param_t p = *uv;
            p.sensitivity = lv;
            /* 先通过EEPROM统一校验并保存，成功后再切换运行等级。 */
            if (AP_EEPROM_UV_Save(&p) == 0) {
                AP_UV_SetLevel((uint8_t)lv);
                CMD_PRINTF("sensitivity=%lu saved\r\n", (unsigned long)p.sensitivity);
            } else { CMD_PRINTF("invalid parameter or save failed\r\n"); }
        }
        return;
    }

    if (strcmp(argv[1], "minmax") == 0) {
        uint32_t thr_min, thr_max, win_min, win_max;
        uint32_t cfm_min, cfm_max;
        AP_UV_GetConfig(&thr_min, &thr_max, &win_min, &win_max,
                        &cfm_min, &cfm_max);
        CMD_PRINTF("thr  %lu %lu\r\n", (unsigned long)thr_min, (unsigned long)thr_max);
        CMD_PRINTF("win  %lu %lu\r\n", (unsigned long)win_min, (unsigned long)win_max);
        CMD_PRINTF("cfm  %lu %lu\r\n", (unsigned long)cfm_min, (unsigned long)cfm_max);
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
        else if (strcmp(field, "pw_min") == 0 && argc > 2)       { p.pw_min_us = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "pw_max") == 0 && argc > 2)       { p.pw_max_us = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "print_win") == 0 && argc > 2)    { p.print_window_ms = strtoul(argv[2], NULL, 0); }else { found = 0; }

        if (found) {
            /* 禁止非法CLI参数先进入运行态：保存成功后才刷新UV/BSP配置。 */
            if (AP_EEPROM_UV_Save(&p) == 0) {
                AP_UV_SetConfig(p.thr_min, p.thr_max, p.win_min, p.win_max,
                                p.cfm_min, p.cfm_max);
                AP_UV_SetLevel((uint8_t)p.sensitivity);
                AP_UV_SetPrintWindow(p.print_window_ms);
                BSP_TIM_IC_SetPulseRange((uint16_t)p.pw_min_us, (uint16_t)p.pw_max_us);
                CMD_PRINTF("%s saved\r\n", field);
            } else { CMD_PRINTF("invalid parameter or save failed\r\n"); }
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
        uint32_t power_min, power_max, power, r38, r50, freq_low, freq_high;
        uint32_t cfm_min, cfm_max;
        uint32_t cfm;
        AP_IR_GetConfig(&power_min, &power_max, &r38, &r50,
                        &freq_low, &freq_high,
                        &cfm_min, &cfm_max);
        AP_IR_GetParams(&power, &r38, &r50, &freq_low, &freq_high, &cfm);

        CMD_PRINTF("IR level=%lu (0 most sensitive, 9 least)\r\n",
            (unsigned long)ir->sensitivity);
        CMD_PRINTF("         min→max    cur\r\n");
        CMD_PRINTF("  power: %lu->%-8lu %lu\r\n",
            (unsigned long)power_min, (unsigned long)power_max,
            (unsigned long)power);
        CMD_PRINTF("  r38:   %lu (fixed)\r\n", (unsigned long)r38);
        CMD_PRINTF("  r50:   %lu (fixed)\r\n", (unsigned long)r50);
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
            if (lv > 9U) {
                CMD_PRINTF("sensitivity must be 0~9\r\n");
                return;
            }

            AP_EEPROM_IR_Param_t p = *ir;
            p.sensitivity = lv;
            /* 先保存后应用，避免Flash失败时RAM等级与掉电参数不一致。 */
            if (AP_EEPROM_IR_Save(&p) == 0) {
                AP_IR_SetLevel((uint8_t)lv);
                CMD_PRINTF("sensitivity=%lu saved\r\n", (unsigned long)p.sensitivity);
            } else { CMD_PRINTF("invalid parameter or save failed\r\n"); }
        }
        return;
    }

    if (strcmp(argv[1], "minmax") == 0) {
        uint32_t power_min, power_max, r38, r50, freq_low, freq_high;
        uint32_t cfm_min, cfm_max;
        AP_IR_GetConfig(&power_min, &power_max, &r38, &r50,
                        &freq_low, &freq_high,
                        &cfm_min, &cfm_max);
        CMD_PRINTF("power %lu %lu\r\n",
            (unsigned long)power_min, (unsigned long)power_max);
        CMD_PRINTF("r38  %lu (fixed)\r\n", (unsigned long)r38);
        CMD_PRINTF("r50  %lu (fixed)\r\n", (unsigned long)r50);
        CMD_PRINTF("freq  %lu %lu\r\n", (unsigned long)freq_low, (unsigned long)freq_high);
        CMD_PRINTF("cfm  %lu %lu\r\n", (unsigned long)cfm_min, (unsigned long)cfm_max);
        return;
    }

    /* 修改功率/确认时间范围或其余固定判据。 */
    {
        int found = 1;
        AP_EEPROM_IR_Param_t p = *ir;
        const char *field = argv[1];

        if (strcmp(field, "power_min") == 0 && argc > 2)     { p.power_min = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "power_max") == 0 && argc > 2){ p.power_max = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "r38") == 0 && argc > 2)      { p.r38_threshold = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "r50") == 0 && argc > 2)      { p.r50_threshold = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "freq_low") == 0 && argc > 2) { p.freq_low_x10  = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "freq_high") == 0 && argc > 2){ p.freq_high_x10 = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "cfm_min") == 0 && argc > 2)  { p.cfm_min   = strtoul(argv[2], NULL, 0); }
        else if (strcmp(field, "cfm_max") == 0 && argc > 2)  { p.cfm_max   = strtoul(argv[2], NULL, 0); }
        else { found = 0; }

        if (found) {
            /* EEPROM统一校验min<=max等约束，保存成功后才更新正式算法。 */
            if (AP_EEPROM_IR_Save(&p) == 0) {
                AP_IR_SetConfig(p.power_min, p.power_max,
                                p.r38_threshold, p.r50_threshold,
                                p.freq_low_x10, p.freq_high_x10,
                                p.cfm_min, p.cfm_max);
                CMD_PRINTF("%s saved\r\n", field);
            } else { CMD_PRINTF("invalid parameter or save failed\r\n"); }
            return;
        }
    }

    CMD_PRINTF("Unknown IR field: %s (try sensitivity/minmax/power_min/power_max/...)\r\n", argv[1]);
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
    float    dominant_freq[3];
    uint32_t r45_38, r45_50;
    AP_IR_GetFeatures(power, dominant_freq, &r45_38, &r45_50);
    /* nano printf不保证浮点格式支持，FFT主频按×10定点整数输出。 */
    uint32_t freq_x10[3] = {
        (uint32_t)(dominant_freq[0] * 10.0f + 0.5f),
        (uint32_t)(dominant_freq[1] * 10.0f + 0.5f),
        (uint32_t)(dominant_freq[2] * 10.0f + 0.5f)
    };

    CMD_PRINTF("  P[38]=%lu  P[45]=%lu  P[50]=%lu\r\n",
        (unsigned long)power[0], (unsigned long)power[1], (unsigned long)power[2]);
    CMD_PRINTF("  FFT_Fx10[38]=%lu  FFT_Fx10[45]=%lu  FFT_Fx10[50]=%lu\r\n",
        (unsigned long)freq_x10[0], (unsigned long)freq_x10[1],
        (unsigned long)freq_x10[2]);
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

    if (argc == 2)
    {
        uint32_t ch = (uint32_t)strtoul(argv[1], NULL, 0);
        if (ch <= 2)
        {
            CMD_PRINTF("ADC%lu = %lu\r\n",
                (unsigned long)ch, (unsigned long)AP_ADC_GetLatest(ch));
        }
        else
        {
            for (uint8_t i = 0; i <=2; i++)
            {
                CMD_PRINTF("ADC%lu = %lu ",
                (unsigned long)i, (unsigned long)AP_ADC_GetLatest(i));
            }
            CMD_PRINTF("\r\n");
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
/*                         show — 演示模式                                     */
/* ========================================================================== */

static void cmd_show(int argc, char **argv)
{
    if ((argc == 1) || ((argc == 2) && (strcmp(argv[1], "mode") == 0))) {
        CMD_PRINTF("show mode=%s (%s)\r\n",
            APP_GetShowMode() ? "on" : "off",
            APP_GetShowMode() ? "UV only" : "UV && IR");
        return;
    }

    if ((argc == 3) && (strcmp(argv[1], "mode") == 0)) {
        if (strcmp(argv[2], "on") == 0) {
            if (APP_SetShowMode(1U) != 0) {
                CMD_PRINTF("show mode save failed\r\n");
                return;
            }
            CMD_PRINTF("show mode=on (UV only, saved)\r\n");
            return;
        }
        if (strcmp(argv[2], "off") == 0) {
            if (APP_SetShowMode(0U) != 0) {
                CMD_PRINTF("show mode save failed\r\n");
                return;
            }
            CMD_PRINTF("show mode=off (UV && IR, saved)\r\n");
            return;
        }
    }

    CMD_PRINTF("Usage: show mode [on|off]\r\n");
}

/* ========================================================================== */
/*                         test — 运行时测试模式                               */
/* ========================================================================== */

static void cmd_test(int argc, char **argv)
{
    if ((argc == 1) || ((argc == 2) && (strcmp(argv[1], "mode") == 0))) {
        CMD_PRINTF("test mode=%s\r\n", APP_GetTestMode() ? "on" : "off");
        return;
    }

    if ((argc == 3) && (strcmp(argv[1], "mode") == 0)) {
        uint8_t enable;
        if (strcmp(argv[2], "on") == 0) {
            enable = 1U;
        } else if (strcmp(argv[2], "off") == 0) {
            enable = 0U;
        } else {
            CMD_PRINTF("Usage: test mode [on|off]\r\n");
            return;
        }

        if (APP_SetTestMode(enable) != 0) {
            CMD_PRINTF("test mode save failed\r\n");
            return;
        }
        CMD_PRINTF("test mode=%s (saved)%s\r\n",
            enable ? "on" : "off",
            enable ? "; use 'debug on' to print data" : "");
        return;
    }

    CMD_PRINTF("Usage: test mode [on|off]\r\n");
}

/* ========================================================================== */
/*                         profile — 红外点火包络分类开关                      */
/* ========================================================================== */

static void cmd_profile(int argc, char **argv)
{
    if ((argc == 1) || ((argc == 2) && (strcmp(argv[1], "mode") == 0))) {
        CMD_PRINTF("profile=%s (%s)\r\n",
            APP_GetIrProfileEnabled() ? "on" : "off",
            APP_GetIrProfileEnabled() ? "lighter rejection enabled"
                                      : "lighter rejection bypassed");
        return;
    }

    const char *value = (argc == 2) ? argv[1]
                       : ((argc == 3 && strcmp(argv[1], "mode") == 0)
                          ? argv[2] : NULL);
    if (value != NULL) {
        uint8_t enable;
        if (strcmp(value, "on") == 0) {
            enable = 1U;
        } else if (strcmp(value, "off") == 0) {
            enable = 0U;
        } else {
            CMD_PRINTF("Usage: profile [mode] [on|off]\r\n");
            return;
        }

        if (APP_SetIrProfileEnabled(enable) != 0) {
            CMD_PRINTF("profile save failed\r\n");
            return;
        }
        CMD_PRINTF("profile=%s (%s, saved)\r\n",
            enable ? "on" : "off",
            enable ? "lighter rejection enabled" : "lighter may alarm");
        return;
    }

    CMD_PRINTF("Usage: profile [mode] [on|off]\r\n");
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
    CMD_PRINTF("  Mode: %s, SHOW=%s, PROFILE=%s\r\n",
        APP_GetTestMode() ? "TEST" : "APP",
        APP_GetShowMode() ? "on (UV only)" : "off (UV && IR)",
        APP_GetIrProfileEnabled() ? "on" : "off");
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
    if (!APP_GetTestMode()) {
        (void)argc;
        (void)argv;
        CMD_PRINTF("debug: enable 'test mode on' first\r\n");
        return;
    }

    if (argc < 2) {
        CMD_PRINTF("debug ir=%s uv=%s ir_win=%lums\r\n",
            TEST_GetIrEnabled() ? "on" : "off",
            TEST_GetUvEnabled() ? "on" : "off",
            (unsigned long)AP_IR_TestGetAvgWindowMs());
        return;
    }
    if (argc == 2) {
        if (strcmp(argv[1], "on") == 0) {
            AP_IR_TestSetAvgWindowMs(AP_IR_TestGetAvgWindowMs());
            TEST_SetIrEnabled(1);
            TEST_SetUvEnabled(1);
            CMD_PRINTF("debug ir=on uv=on\r\n");
        } else if (strcmp(argv[1], "off") == 0) {
            TEST_SetIrEnabled(0);
            TEST_SetUvEnabled(0);
            CMD_PRINTF("debug ir=off uv=off\r\n");
        } else {
            CMD_PRINTF("Usage: debug [ir|uv] on|off\r\n");
        }
        return;
    }
    if (argc >= 3) {
        if (strcmp(argv[1], "ir") == 0) {
            if (argc >= 4 && strcmp(argv[2], "win") == 0) {
                uint32_t sec = strtoul(argv[3], NULL, 0);
                AP_IR_TestSetAvgWindowMs(sec * 1000UL);
                CMD_PRINTF("debug ir win=%lums\r\n",
                    (unsigned long)AP_IR_TestGetAvgWindowMs());
                return;
            }
            uint8_t en = (strcmp(argv[2], "on") == 0) ? 1 : 0;
            if (en) {
                AP_IR_TestSetAvgWindowMs(AP_IR_TestGetAvgWindowMs());
            }
            TEST_SetIrEnabled(en);
            CMD_PRINTF("debug ir=%s\r\n", en ? "on" : "off");
        } else if (strcmp(argv[1], "uv") == 0) {
            uint8_t en = (strcmp(argv[2], "on") == 0) ? 1 : 0;
            TEST_SetUvEnabled(en);
            CMD_PRINTF("debug uv=%s\r\n", en ? "on" : "off");
        } else {
            CMD_PRINTF("Usage: debug [ir|uv] on|off\r\n");
        }
    }
}

/* ========================================================================== */
/*                         mark — 测试场景分隔标记                              */
/* ========================================================================== */

static void cmd_mark(int argc, char **argv)
{
    if (!APP_GetTestMode()) {
        CMD_PRINTF("mark: enable 'test mode on' first\r\n");
        return;
    }
    const char *msg = (argc > 1) ? argv[1] : NULL;
    TEST_InsertMarker(msg);
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
static void cmd_uart(int argc, char **argv)
{
    if (!APP_GetTestMode()) {
        CMD_PRINTF("uart: enable 'test mode on' first\r\n");
        return;
    }
    if (argc < 2) {
        CMD_PRINTF("Usage: uart raw <on/off> | loop <n> | recv\r\n");
        return;
    }

    if (strcmp(argv[1], "raw") == 0) {
        if (argc != 3 ||
            (strcmp(argv[2], "on") != 0 && strcmp(argv[2], "off") != 0)) {
            CMD_PRINTF("Usage: uart raw <on/off>\r\n");
            return;
        }
        if (strcmp(argv[2], "on") == 0) {
            AP_UART_SetDiagnosticMode(true);
            CMD_PRINTF("uart raw=on, protocol session cleared\r\n");
        } else {
            if (!BSP_UART_IsTxComplete(BSP_UART_COM)) {
                CMD_PRINTF("uart: COM TX busy\r\n");
                return;
            }
            AP_UART_SetDiagnosticMode(false);
            CMD_PRINTF("uart raw=off, protocol requires handshake\r\n");
        }

    } else if (strcmp(argv[1], "loop") == 0) {
        uint32_t n = (argc > 2) ? strtoul(argv[2], NULL, 0) : 16;
        if (n == 0U) n = 1U;
        if (n > 128) n = 128;
        uint8_t buf[128];
        if (!AP_UART_GetDiagnosticMode()) {
            CMD_PRINTF("uart: enable 'uart raw on' first\r\n");
            return;
        }
        for (uint32_t i = 0; i < n; i++) buf[i] = (uint8_t)(i & 0xFF);
        if (BSP_UART_Write(BSP_UART_COM, buf, (uint16_t)n) != (uint16_t)n) {
            BSP_UART_Printf("[UART] COM send failed\r\n");
        } else {
            BSP_UART_Printf("[UART] loopback %lu bytes sent on COM\r\n", (unsigned long)n);
        }

    } else if (strcmp(argv[1], "recv") == 0) {
        uint8_t buf[64];
        if (!AP_UART_GetDiagnosticMode()) {
            CMD_PRINTF("uart: enable 'uart raw on' first\r\n");
            return;
        }
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
        CMD_PRINTF("Usage: uart raw <on/off> | loop <n> | recv\r\n");
    }
}
/* ========================================================================== */
/*                         未知命令                                             */
/* ========================================================================== */

static void cmd_unknown(int argc, char **argv)
{
    (void)argc;
    CMD_PRINTF("Unknown command: %s\r\n", argv[0]);
    CMD_PRINTF("Type 'help' for available commands\r\n");
}
