#pragma once

#include "stdint.h"

extern int16_t USBDisk[4080];

void UDISK_push(int16_t temp, uint32_t timestamp);

int16_t UDISK_get(uint16_t i);

uint16_t UDISK_len();

uint32_t UDISK_tst();