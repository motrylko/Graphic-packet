#pragma once
// AutoBIN experimental CCITT-0 checksum (NOT AX.25/X.25 FCS).
#include <stdint.h>
inline uint16_t autoBinCrcStep(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (uint8_t i=0;i<8;i++)
    crc=(uint16_t)((crc<<1)^((crc & 0x8000U)?0x1021U:0));
  return crc;
}
