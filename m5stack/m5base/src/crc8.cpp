/*
  CRC8

  Copyright (C) 2024
*/

#include <stdint.h>
#include "crc8.hpp"

// Peripheral CRCs are used to store data flash, so we'll do the math straight up here.
// Polynomial 0x1d, Ini Value 0xff, Final xor Value 0xff
uint8_t CCRC8::calc (const void *buf, size_t size) {
    uint8_t *data = (uint8_t *)buf;
    uint8_t crc8 = 0xFF;

    while (size-- != 0) crc8 = CRC8_SAE_J1850[crc8 ^ *data++];
    return crc8 ^ 0xff;
}

uint8_t CCRC8::get (uint8_t *crc, uint8_t dat) {
  *crc = CRC8_SAE_J1850[(*crc) ^ dat];
  return *crc;
}