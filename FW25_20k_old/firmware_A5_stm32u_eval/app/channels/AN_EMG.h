/*
* @file AN_EMG.h
* @brief analog emg channel
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef AN_EMG_H_
#define AN_EMG_H_

#include <stdbool.h>

#define AN_EMG_SAMPLE_RATE_MAX (1024)//samples per second
#define DEF_AN_EMG_SAMPLE_RATE (AN_EMG_SAMPLE_RATE_MAX)

#define AN_EMG_SAMPLE_RES_MAX (16)
#define DEF_AN_EMG_SAMPLE_RES (AN_EMG_SAMPLE_RES_MAX)

#define AN_EMG_NUM_OF_SENSORS (1)//14 bit ADC resolution



void AN_EMG_Init(void *p);
void AN_EMG_Reset();
void AN_EMG_SetActivation(void *p, bool cmd);
bool AN_EMG_Handler(void *p);

#endif /* AN_EMG_H_ */
