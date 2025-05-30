#include "USB_storage.h"
#include "stdbool.h"
#include "string.h"

int16_t USBDisk[4080] __attribute__((section(".ram2_bss"))); // Space for 85 days

uint32_t last_timestamp __attribute__((section(".ram2_bss")));
uint16_t last_pointer __attribute__((section(".ram2_bss")));
uint16_t length __attribute__((section(".ram2_bss")));
bool is_first_writing __attribute__((section(".ram2_bss")));
uint32_t data_retention_code __attribute__((section(".ram2_bss")));

void UDISK_init(){
    is_first_writing = true;
    last_pointer = 0;
    length = 0;
    data_retention_code = 0xAAFF55FF;
    memset(USBDisk, 0x00, 4080);
}

void UDISK_push(int16_t temp, uint32_t timestamp){
    if(data_retention_code != 0xAAFF55FF){
        UDISK_init();
    }
    last_timestamp = timestamp;
    USBDisk[last_pointer] = temp;
    last_pointer++;
    length++;
    if(last_pointer >= 4080){
        is_first_writing = false;
        last_pointer = 0;
    }
    if(!is_first_writing){
        length = 4079;
    }
}

int16_t UDISK_get(uint16_t i){
    if(data_retention_code != 0xAAFF55FF){
        return -32768;
    }
    if(is_first_writing){
        return USBDisk[i];
    }
    if(last_pointer+i-1 < 4080){
        return USBDisk[last_pointer+i-1];
    }
    return USBDisk[last_pointer+i-4081];
}

uint16_t UDISK_len(){
    return length;
}

uint32_t UDISK_tst(){
    return last_timestamp;
}