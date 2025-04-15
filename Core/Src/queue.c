#include "queue.h"
#include "time.h"
#include "stm32l4xx_hal_rtc.h"

extern RTC_HandleTypeDef hrtc;
queue data_queue;
//
static uint8_t flag;
//
void q_init(void){
    data_queue.write_ptr = 0;
    data_queue.length = 0;
    data_queue.is_first_writing = true;
    data_queue.start_ptr = 0;
}

void q_push(uint16_t temp){
    // TIME - REG5
    data_queue.data[data_queue.write_ptr] = temp;
    data_queue.write_ptr++;
    data_queue.length++;
    if(data_queue.write_ptr >= 48){
        data_queue.is_first_writing = false;
        data_queue.write_ptr = 0;
    }
    if(!data_queue.is_first_writing){
        data_queue.start_ptr = data_queue.write_ptr;
        data_queue.length = 47;
    }
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    struct tm t;
    time_t t_of_day;

    t.tm_year = 100+sDate.Year;  // Year - 1900
    t.tm_mon = sDate.Month-1;           // Month, where 0 = jan
    t.tm_mday = sDate.Date;          // Day of the month
    t.tm_hour = sTime.Hours;
    t.tm_min = sTime.Minutes;
    t.tm_sec = sTime.Seconds;
    t.tm_isdst = 1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
    t_of_day = mktime(&t);

    HAL_PWR_EnableBkUpAccess();
    if(flag ==0){
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, (long) t_of_day);
    flag = 1;}
    // TEST
    uint32_t testtime = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5)+1800;
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, (long) testtime);
}

uint16_t q_get(uint8_t pos){
    uint8_t ptr = pos + data_queue.start_ptr;
    if(ptr>=48){
        ptr -= 48;
    }
    return data_queue.data[ptr];
}

void q_save(void){
    HAL_PWR_EnableBkUpAccess();
    
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR6, KEY);
    for (int i = 0; i < 24; i++) {
        HAL_RTCEx_BKUPWrite(&hrtc, 8+i, ((uint32_t)data_queue.data[2*i] << 16) | (uint32_t)data_queue.data[2*i + 1]);
    }
    
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7, ((uint32_t)data_queue.length << 24) | \
    ((uint32_t)data_queue.start_ptr << 16) | ((uint32_t)data_queue.write_ptr << 8) | \
    (data_queue.is_first_writing ? 1 : 0));
}
 
bool q_load(void){
    // REGs 6 - 31
    // KEY - REG6
    // SYS - REG7
    // DATA REG8 - 31
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR6) != KEY){return false;}
    for (int i = 0; i < 24; i++) {
        data_queue.data[2*i] = (uint16_t)(HAL_RTCEx_BKUPRead(&hrtc, 8+i) >> 16);       
        data_queue.data[2*i + 1] = (uint16_t)(HAL_RTCEx_BKUPRead(&hrtc, 8+i) & 0xFFFF);
    }
    data_queue.length = (uint8_t)(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7) >> 24);
    data_queue.start_ptr = (uint8_t)(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7) >> 16);
    data_queue.write_ptr = (uint8_t)(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7) >> 8);
    data_queue.is_first_writing = (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7) & 0x01) != 0;
    return true;
}

uint16_t q_len(void){
    return data_queue.length;
}