/*
* @file ECG_H.h
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef ECG__H_H_
#define ECG_H_H_

#include "MAX30001/max30001_dev.h"

#define ECG_H_SAMPLE_RATE_MAX (DEF_ODR_ECG_H) //samples per second
#define DEF_ECG_H_SAMPLE_RATE (DEF_ODR_ECG_H)
#define ECG_H_SAMPLE_RES_MAX (16) //bits per sample ECG voltage 18b + ETAG/PTAG 6b
#define DEF_ECG_H_SAMPLE_RES (ECG_H_SAMPLE_RES_MAX)
#define ECG_H_NUM_OF_SENSORS (1)

void ECG_H_Init(void *p);
void ECG_H_Reset();
void ECG_H_SetActivation(void *p, bool cmd);
bool ECG_H_Handler(void *p);

#endif /* ECG_H_H_ */
