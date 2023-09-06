/*
* @file ECG_V.h
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef ECG_V_H_
#define ECG_V_H_

#include "MAX30003/max30003_dev.h"

#define ECG_V_SAMPLE_RATE_MAX (DEF_ODR_ECG_V) //samples per second
#define DEF_ECG_V_SAMPLE_RATE (DEF_ODR_ECG_V)
#define ECG_V_SAMPLE_RES_MAX (16) //bits per sample
#define DEF_ECG_V_SAMPLE_RES (ECG_V_SAMPLE_RES_MAX)
#define ECG_V_NUM_OF_SENSORS (1)

void ECG_V_Init(void *p);
void ECG_V_Reset();
void ECG_V_SetActivation(void *p, bool cmd);
bool ECG_V_Handler(void *p);

#endif /* ECG_V_H_ */
