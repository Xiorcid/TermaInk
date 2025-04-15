#pragma once

#include "stdint.h"
#include "main.h"
#include "stdbool.h"

#define KEY 0xAAFF55FF

typedef struct queue
{
    uint16_t data[48];
    uint8_t write_ptr;
    uint8_t start_ptr;
    bool is_first_writing;
    uint8_t length;
} queue;

void q_init(void);
void q_push(uint16_t temp);
uint16_t q_get(uint8_t pos);
void q_save(void);
bool q_load(void);
uint16_t q_len(void);