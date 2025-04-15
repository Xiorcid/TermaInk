#pragma once

#include "stdint.h"
#define STORAGE_BLK_NBR                  48  // enter twice the size of the RAM that you want to use
#define STORAGE_BLK_SIZ                  0x200

uint8_t USBDisk_buffer[STORAGE_BLK_NBR*STORAGE_BLK_SIZ];