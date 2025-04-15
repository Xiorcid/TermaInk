#ifndef _BTT
#define _BTT

#include "stdbool.h"
#include <stdint.h>
#include "stm32l4xx_hal.h"

#define TYPE_HIGH_PULL  1
#define TYPE_LOW_PULL   0

typedef struct _Button{
    GPIO_TypeDef* port;
    uint16_t pin;
    bool type;
    uint32_t btn_timer;    
    uint32_t btn_hold_timer;  
    bool btn_flag;
    bool btn_hold;
    bool btn_old;
    bool btn_single;
}Button;

void init(Button *btt);
bool tick(Button *btt);
bool isClicked(Button *btt);
bool isHold(Button *btt);
bool isDoubleClicked(Button *btt);
#endif