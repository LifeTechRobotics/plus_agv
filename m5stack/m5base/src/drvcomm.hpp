/*
  Motor Driver Serial Communication

  Copyright (C) 2024
*/

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <HardwareSerial.h>
#include "crc8.hpp"


class CDrvComm {
public:
	CDrvComm(HardwareSerial *phs);

public:
  HardwareSerial *pSerial;

  bool send(char *pdata);
  bool read(char *pdata, size_t st);
};

