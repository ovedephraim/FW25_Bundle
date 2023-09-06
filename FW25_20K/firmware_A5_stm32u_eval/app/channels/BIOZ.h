/*
* @file BIOZ.h
* @author Anton Kanaev
* @version 0.0.1
* @date 17.06.2023
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef BIOZ_H_
#define BIOZ_H_

#include "MAX30001/max30001_dev.h"

#define BIOZ_SAMPLE_RATE_MAX (DEF_ODR_BIOZ) //samples per second
#define DEF_BIOZ_SAMPLE_RATE (DEF_ODR_BIOZ)
#define BIOZ_SAMPLE_RES_MAX  (16)
#define DEF_BIOZ_SAMPLE_RES  (BIOZ_SAMPLE_RES_MAX)
#define BIOZ_NUM_OF_SENSORS  (1)

void BIOZ_Init(void *p);
void BIOZ_Reset();
void BIOZ_SetActivation(void *p, bool cmd);
bool BIOZ_Handler(void *p);

#endif /* BIOZ_H_ */
