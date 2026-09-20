/**
  ******************************************************************************
  * @file    hal_eeprom.h
  * @brief   BSP 硬件 DATA EEPROM 扇区读写接口
  *
  *          STM32L151RE 内置 16KB DATA EEPROM（0x08080000），
  *          支持字写入（无需擦除），寿命 300k 次。
  *
  *          本层仅提供扇区级的读/写/校验模板，不涉及任何参数字段定义。
  *          参数结构体和分组管理在 AP 层（ap_eeprom.h）实现。
  *
  *          == 扇区布局（AP 层定义）==
  *          0x08080000  扇区0 — ADC 参数
  *          0x08080040  扇区1 — UV 参数
  *          0x08080080  扇区2 — IR 参数
  *          0x080800C0  扇区3 — 系统运行参数
  *
  *          == 使用示例 ==
  *          // 读
  *          uint32_t buf[16];
  *          BSP_EEPROM_Read(0x08080000, buf, sizeof(buf));
  *
  *          // 写
  *          BSP_EEPROM_Write(0x08080040, &my_data, sizeof(my_data));
  ******************************************************************************
  */
#ifndef __BSP_HAL_EEPROM_H__
#define __BSP_HAL_EEPROM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================== */
/*                         EEPROM 地址参数                                     */
/* ========================================================================== */

#define EEPROM_BASE                 (0x08080000UL)
#define EEPROM_SECTOR_SIZE          (0x40UL)         /* 64 字节 / 扇区 */

/* ========================================================================== */
/*                        底层读写接口                                         */
/* ========================================================================== */

/**
  * @brief  从 EEPROM 指定地址读取数据（内存映射）
  * @param  addr: EEPROM 地址（需 4 字节对齐）
  * @param  buf:  输出缓冲
  * @param  size: 读取字节数（需 4 的倍数）
  */
void BSP_EEPROM_Read(uint32_t addr, void *buf, uint32_t size);

/**
  * @brief  写入数据到 EEPROM 指定地址
  * @param  addr: EEPROM 地址（需 4 字节对齐）
  * @param  buf:  待写入数据
  * @param  size: 写入字节数（需 4 的倍数）
  * @retval 0: 成功
  * @retval -1: HAL 写入失败
  */
int  BSP_EEPROM_Write(uint32_t addr, const void *buf, uint32_t size);

/**
  * @brief  读回并逐字比较 EEPROM 内容
  * @param  addr: EEPROM 地址（需 4 字节对齐）
  * @param  buf:  期望内容（需 4 字节对齐）
  * @param  size: 比较字节数（需 4 的倍数）
  * @retval 0: 读回内容完全一致
  * @retval -1: 参数非法或任一字不一致
  */
int  BSP_EEPROM_Verify(uint32_t addr, const void *buf, uint32_t size);

/* ========================================================================== */
/*                    校验加载模板                                             */
/* ========================================================================== */

/** @brief  默认值回调函数类型（参数无效时调用） */
typedef void (*BSP_EEPROM_DefaultsFn)(void *buf);

typedef enum {
    BSP_EEPROM_LOAD_VALID = 0,
    BSP_EEPROM_LOAD_MAGIC_DEFAULTED = 1,
    BSP_EEPROM_LOAD_CRC_DEFAULTED = 2,
    BSP_EEPROM_LOAD_WRITE_ERROR = -1,
} BSP_EEPROM_LoadResult_t;

/**
  * @brief  从扇区读取 + 魔数 + CRC 校验，失败则写默认值
  * @param  addr       EEPROM 地址
  * @param  buf        输出缓冲（读取或默认值）
  * @param  size       结构体总字节数
  * @param  magic      期望魔数
  * @param  def_fn     写入默认值的回调
  * @param  crc_off    crc16 字段在结构体中的字节偏移
  * @retval BSP_EEPROM_LOAD_VALID: 原数据有效
  * @retval BSP_EEPROM_LOAD_MAGIC_DEFAULTED: 魔数不符，默认值已写入
  * @retval BSP_EEPROM_LOAD_CRC_DEFAULTED: CRC不符，默认值已写入
  * @retval BSP_EEPROM_LOAD_WRITE_ERROR: 默认值写入或读回失败
  */
int  BSP_EEPROM_LoadSector(uint32_t addr, void *buf, uint32_t size,
                            uint32_t magic, BSP_EEPROM_DefaultsFn def_fn,
                            uint16_t crc_off);

/**
  * @brief  计算 CRC、写入扇区
  * @param  addr    EEPROM 地址
  * @param  buf     数据（crc16 字段会被自动填充）
  * @param  size    结构体总字节数
  * @param  crc_off crc16 字段在结构体中的字节偏移
  * @retval 0: 成功
  * @retval -1: 写入失败
  */
int  BSP_EEPROM_SaveSector(uint32_t addr, void *buf, uint32_t size,
                            uint16_t crc_off);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HAL_EEPROM_H__ */
