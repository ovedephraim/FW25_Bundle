/**
* @file rtc.h
* @brief time functions wrapper
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 30.04.2022
*/
#ifndef _RTC_H
#define _RTC_H

#include <stddef.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ck_apre=LSEFreq/(ASYNC prediv + 1) = 256Hz with LSEFreq=32768Hz */
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
/* ck_spre=ck_apre/(SYNC prediv + 1) = 1 Hz */
#define RTC_SYNCH_PREDIV           ((uint32_t)0x00FF)

int RTC_set(time_t now);
int RTC_Init(void);
int RTC_get(time_t *pnow);
int RTC_save_time_stamp(uint32_t ts);
int RTC_get_time_stamp(uint32_t * pts);
int RTC_set_wakeup_timer(uint16_t tm_sec, void * cb);
uint32_t RTC_GetTimestampMillis(void);

int RTC_disable_wakeup_timer(void);
int RTC_enable_wakeup_timer(void * cb);

#ifdef __cplusplus
}
#endif

#endif //_RTC_H
