#ifndef __CRC16_H__
#define __CRC16_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  CRC16-Modbus（右移，多项式 0x8005，初始 0xFFFF）
  *         用于串口通信协议帧校验。
  */
uint16_t CRC16_Modbus(const uint8_t *data, uint16_t len);

/**
  * @brief  CRC16-CCITT（左移，多项式 0x1021，初始 0xFFFF）
  *         用于参数存储结构的完整性校验。
  */
uint16_t CRC16_CCITT(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CRC16_H__ */
