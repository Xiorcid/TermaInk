#include "buttons.h"

void init(Button *btt){
    //gpio_SetGPIOmode_In(btt->port, btt->pin, 0);
    btt->btn_flag = true;
}

bool tick(Button *btt){
    bool btn_state = !HAL_GPIO_ReadPin(btt->port, btt->pin);
    btn_state ^= !btt->type;

    if (!btn_state && !btt->btn_flag && HAL_GetTick() - btt->btn_timer > 100) {
        btt->btn_flag = true;
        if(!btt->btn_hold){btt->btn_single = true;}
        btt->btn_hold = false;
        btt->btn_timer = HAL_GetTick();
    }
    if (btn_state && btt->btn_flag && HAL_GetTick() - btt->btn_timer > 100) {
        btt->btn_flag = false;
        btt->btn_timer = HAL_GetTick();
        btt->btn_hold_timer = HAL_GetTick();
    }

    if (!btt->btn_flag && btn_state && (HAL_GetTick() - btt->btn_hold_timer >= 200) && !btt->btn_hold) {
        btt->btn_hold = true;
        btt->btn_hold_timer = HAL_GetTick();
    }

    return btn_state;
}

bool isClicked(Button *btt){
    if(btt->btn_single){
        btt->btn_single = false;
        return true;
    } return false;
}

bool isHold(Button *btt){
    if(btt->btn_hold){
        return true;
    } return false;
}