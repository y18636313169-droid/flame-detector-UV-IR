/**
  ******************************************************************************
  * @file    ap_util.h
  * @brief   AP 层通用工具函数（线性插值等）
  *
  *          提供各 AP 模块间共享的小型实用函数，避免代码重复。
  *          所有函数均为 static inline，无额外调用开销。
  ******************************************************************************
  */
#ifndef __AP_UTIL_H__
#define __AP_UTIL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================== */
/*                          测试模式开关                                        */
/* ========================================================================== */

#define IR_TEST_MODE    /* 取消注释进入测试模式: 仅采集ADC+UV原始数据 */

/* ========================================================================== */

/**
  * @brief  无符号 32 位线性插值
  *         将 level 在 [0, levels-1] 范围内的值线性映射到 [min, max]
  * @param  min:    等级 0 对应的值
  * @param  max:    等级 levels-1 对应的值
  * @param  level:  当前等级 (0 ~ levels-1, 超出上限自动饱和)
  * @param  levels: 总等级数 (levels-1 作为插值分母)
  * @return 插值结果
  */
static inline uint32_t lerp_u32(uint32_t min, uint32_t max,
                                uint32_t level, uint32_t levels)
{
    if (level >= levels) level = levels - 1;
    return min + ((max - min) * level) / (levels - 1);
}

#ifdef __cplusplus
}
#endif

#endif /* __AP_UTIL_H__ */
