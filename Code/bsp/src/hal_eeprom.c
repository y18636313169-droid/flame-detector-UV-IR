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

/* ========================================================================== */
/*                    校验加载模板                                             */
/* ========================================================================== */

int BSP_EEPROM_LoadSector(uint32_t addr, void *buf, uint32_t size,
                           uint32_t magic, BSP_EEPROM_DefaultsFn def_fn,
                           uint16_t crc_off)
{
    uint8_t valid = 0;

    BSP_EEPROM_Read(addr, buf, size);

    uint32_t *p_magic = (uint32_t *)buf;
    if (*p_magic == magic) {
        uint16_t saved_crc = *(uint16_t *)((uint8_t *)buf + crc_off);
        *(uint16_t *)((uint8_t *)buf + crc_off) = 0;
        uint16_t calc_crc = CRC16_CCITT((const uint8_t *)buf, size);
        *(uint16_t *)((uint8_t *)buf + crc_off) = saved_crc;

        if (saved_crc == calc_crc) {
            valid = 1;
        }
    }

    if (valid) {
        return 0;
    }

    /* 无效 → 写默认值 */
    def_fn(buf);
    *(uint16_t *)((uint8_t *)buf + crc_off) = 0;
    *(uint16_t *)((uint8_t *)buf + crc_off) = CRC16_CCITT((const uint8_t *)buf, size);

    return BSP_EEPROM_Write(addr, buf, size);
}

int BSP_EEPROM_SaveSector(uint32_t addr, void *buf, uint32_t size,
                           uint16_t crc_off)
{
    *(uint16_t *)((uint8_t *)buf + crc_off) = 0;
    *(uint16_t *)((uint8_t *)buf + crc_off) = CRC16_CCITT((const uint8_t *)buf, size);

    return BSP_EEPROM_Write(addr, buf, size);
}
