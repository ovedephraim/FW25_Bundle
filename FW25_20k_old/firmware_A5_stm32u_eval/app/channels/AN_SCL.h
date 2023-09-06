/*
* @file AN_SCL.h
* @brief analog scl channel
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef AN_SCL_H_
#define AN_SCL_H_

#include <stdbool.h>

#define AN_SCL_SAMPLE_RATE_MAX (1024)//samples per second
#define DEF_AN_SCL_SAMPLE_RATE (AN_SCL_SAMPLE_RATE_MAX)

#define AN_SCL_SAMPLE_RES_MAX (16)  //14 bit ADC resolution
#define DEF_AN_SCL_SAMPLE_RES (AN_SCL_SAMPLE_RES_MAX)

#define AN_SCL_NUM_OF_SENSORS 1

void AN_SCL_Init(void *p);
void AN_SCL_Reset();
void AN_SCL_SetActivation(void *p, bool cmd);
bool AN_SCL_Handler(void *p);

#endif /* AN_SCL_H_ */
