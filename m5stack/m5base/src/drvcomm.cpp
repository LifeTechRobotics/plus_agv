/*
  Motor Driver Serial Communication

  Copyright (C) 2024
*/

#include <stdint.h>
#include "drvcomm.hpp"


CDrvComm::CDrvComm(HardwareSerial *phs) {
  pSerial = phs;
}

bool CDrvComm::send(char *pdata) {
  if (!pSerial) {
    return false;
  }

  CCRC8 crc8;
  uint8_t crc = crc8.calc(pdata, strlen(pdata));
  char buf[256];
  memset(buf, 0x00, sizeof(buf));
  sprintf(buf, "<%s%02X>\r", pdata, crc);
  size_t len = strlen(buf);
  size_t l = pSerial->write((const char*)buf, len);
  pSerial->flush();
  if (l != len) return false;

  return true;
}

bool CDrvComm::read(char *pdata, size_t st) {
  if (!pSerial) {
    return false;
  }

  if (pSerial->available()) {
#if 0
    size_t len;
    int i = 0, j = 0;
    for (i = 0, j = 0; i < st;) {
      if (pSerial->available()) {
        pSerial->read(&pdata[i], 1);
        if (pdata[i] == '\r') {
          break;
        }
        i++;
        j = 0;
      }
      j++;
      if (j > 100) {
        break;
      }
      delay(1);
    }
  #endif
    pSerial->readBytesUntil((char)'\r', pdata, st);

    return true;
  }

  return true;
//  return false;
}