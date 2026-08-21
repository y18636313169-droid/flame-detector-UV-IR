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

/* 测试模式已改为CMD运行时切换，不再通过宏裁剪后重新烧录固件。 */
#define AP_ALGO_DEBUG_ENABLE 1 /* 应用算法事件+500ms快照日志；关闭时不编译调试状态 */

/* IR/UV共用：FIRE期间不做条件消警，仅在保持3分钟后自动清除。 */
#define AP_FIRE_AUTO_CLEAR_MS  (3UL * 60UL * 1000UL)

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
    if (levels <= 1U) return min;
    if (level >= levels) level = levels - 1;
    /* 64位乘法避免大参数插值溢出，同时兼容现场误配的降序端点。 */
    if (max >= min) {
        return min + (uint32_t)(((uint64_t)(max - min) * level) / (levels - 1U));
    }
    return min - (uint32_t)(((uint64_t)(min - max) * level) / (levels - 1U));
}

#ifdef __cplusplus
}
#endif

#endif /* __AP_UTIL_H__ */
