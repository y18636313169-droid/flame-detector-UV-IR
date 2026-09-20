/**
  ******************************************************************************
  * @file    hal_eeprom.c
  * @brief   BSP 硬件 DATA EEPROM 扇区读写实现
  *
  *          不包含任何应用参数定义，仅提供扇区级读写 + 校验加载模板。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hal_eeprom.h"
#include "crc16.h"
#include <string.h>
#include "main.h"

/* ========================================================================== */
/*                         底层读写                                            */
/* ========================================================================== */

void BSP_EEPROM_Read(uint32_t addr, void *buf, uint32_t size)
{
    volatile const uint32_t *src = (volatile const uint32_t *)addr;
    uint32_t *dst = (uint32_t *)buf;
    uint32_t nwords = size / sizeof(uint32_t);

    for (uint32_t i = 0; i < nwords; i++) {
        dst[i] = src[i];
    }
}

int BSP_EEPROM_Write(uint32_t addr, const void *buf, uint32_t size)
{
    const uint32_t *src = (const uint32_t *)buf;
    uint32_t nwords = size / sizeof(uint32_t);

    HAL_FLASHEx_DATAEEPROM_Unlock();
    for (uint32_t i = 0; i < nwords; i++) {
        if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, src[i]) != HAL_OK) {
            HAL_FLASHEx_DATAEEPROM_Lock();
            return -1;
        }
    }
    HAL_FLASHEx_DATAEEPROM_Lock();
    return 0;
}

int BSP_EEPROM_Verify(uint32_t addr, const void *buf, uint32_t size)
{
    volatile const uint32_t *stored = (volatile const uint32_t *)addr;
    const uint32_t *expected = (const uint32_t *)buf;

    if (buf == NULL || size == 0U || (addr & 3U) != 0U ||
        (((uintptr_t)buf) & 3U) != 0U || (size & 3U) != 0U) {
        return -1;
    }

    /* DATA EEPROM为内存映射区，逐字读回可发现HAL写入成功但内容未落稳的情况。 */
    for (uint32_t i = 0U; i < size / sizeof(uint32_t); i++) {
        if (stored[i] != expected[i]) {
            return -1;
        }
    }
    return 0;
}

/* ========================================================================== */
/*                    校验加载模板                                             */
/* ========================================================================== */

int BSP_EEPROM_LoadSector(uint32_t addr, void *buf, uint32_t size,
                           uint32_t magic, BSP_EEPROM_DefaultsFn def_fn,
                           uint16_t crc_off)
{
    uint8_t valid = 0;
    BSP_EEPROM_LoadResult_t invalid_reason = BSP_EEPROM_LOAD_MAGIC_DEFAULTED;

    BSP_EEPROM_Read(addr, buf, size);

    uint32_t *p_magic = (uint32_t *)buf;
    if (*p_magic == magic) {
        uint16_t saved_crc = *(uint16_t *)((uint8_t *)buf + crc_off);
        *(uint16_t *)((uint8_t *)buf + crc_off) = 0;
        uint16_t calc_crc = CRC16_CCITT((const uint8_t *)buf, size);
        *(uint16_t *)((uint8_t *)buf + crc_off) = saved_crc;

        if (saved_crc == calc_crc) {
            valid = 1;
        } else {
            invalid_reason = BSP_EEPROM_LOAD_CRC_DEFAULTED;
        }
    }

    if (valid) {
        return 0;
    }

    /* 无效 → 写默认值 */
    def_fn(buf);
    *(uint16_t *)((uint8_t *)buf + crc_off) = 0;
    *(uint16_t *)((uint8_t *)buf + crc_off) = CRC16_CCITT((const uint8_t *)buf, size);

    if (BSP_EEPROM_Write(addr, buf, size) != 0 ||
        BSP_EEPROM_Verify(addr, buf, size) != 0) {
        return BSP_EEPROM_LOAD_WRITE_ERROR;
    }
    return invalid_reason;
}

int BSP_EEPROM_SaveSector(uint32_t addr, void *buf, uint32_t size,
                           uint16_t crc_off)
{
    int ret;

    *(uint16_t *)((uint8_t *)buf + crc_off) = 0;
    *(uint16_t *)((uint8_t *)buf + crc_off) = CRC16_CCITT((const uint8_t *)buf, size);

    ret = BSP_EEPROM_Write(addr, buf, size);
    if (ret != 0) {
        return ret;
    }

    /* SUCCESS必须代表完整结构和CRC均已落盘，协议层才能据此回复成功。 */
    return BSP_EEPROM_Verify(addr, buf, size);
}
