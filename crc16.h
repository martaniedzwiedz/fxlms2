#ifndef CRC16_H_INCLUDED
#define CRC16_H_INCLUDED

#include <stddef.h>
#include <inttypes.h>

uint16_t crc16(uint16_t crc, const uint8_t *buffer, size_t len);

#endif
