/*
* @file TEMP_SKIN.h
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef _TEMP_SKIN_H_
#define _TEMP_SKIN_H_

#include <stdbool.h>

#define TEMP_SKIN_SAMPLE_RATE_MAX (1) //samples per second
#define DEF_TEMP_SKIN_SAMPLE_RATE (1)
#define TEMP_SKIN_SAMPLE_RES_MAX (16) //bits per sample ECG voltage 18b + ETAG/PTAG 6b
#define DEF_TEMP_SKIN_SAMPLE_RES (TEMP_SKIN_SAMPLE_RES_MAX)
#define TEMP_SKIN_NUM_OF_SENSORS (3)

void TEMP_SKIN_Init(void *p);
void TEMP_SKIN_Reset();
void TEMP_SKIN_SetActivation(void *p, bool cmd);
bool TEMP_SKIN_Handler(void *p);

#endif /* _TEMP_SKIN_H_ */
