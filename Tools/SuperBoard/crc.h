#ifndef __CRC_H__
#define __CRC_H__

#include <stdint.h>

uint16_t CRC16_ANSI_Block(uint8_t * data, uint16_t length);
uint32_t crc32(uint8_t *buf, uint32_t len);

#endif // __CRC_H__
