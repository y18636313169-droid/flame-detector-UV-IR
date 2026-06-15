#include "crc16.h"

/* ========================================================================== */
/*          CRC16-Modbus — 右移，多项式 0x8005 <-> 0xA001                     */
/*          用于串口通信协议帧校验                                              */
/* ========================================================================== */

uint16_t CRC16_Modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ========================================================================== */
/*          CRC16-CCITT — 左移，多项式 0x1021，初始 0xFFFF                    */
/*          用于参数存储结构的完整性校验                                        */
/* ========================================================================== */

uint16_t CRC16_CCITT(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
