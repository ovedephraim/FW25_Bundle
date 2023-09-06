/*
* @file AN_PACE.h
* @brief analog pace channel
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef AN_PACE_H_
#define AN_PACE_H_

#include <stdbool.h>

#define AN_PACE_SAMPLE_RATE_MAX (1024)//samples per second
#define DEF_AN_PACE_SAMPLE_RATE (AN_PACE_SAMPLE_RATE_MAX)

#define AN_PACE_SAMPLE_RES_MAX (16)  //14 bit ADC resolution
#define DEF_AN_PACE_SAMPLE_RES (AN_PACE_SAMPLE_RES_MAX)

#define AN_PACE_NUM_OF_SENSORS 1

void AN_PACE_Init(void *p);
void AN_PACE_Reset();
void AN_PACE_SetActivation(void *p, bool cmd);
bool AN_PACE_Handler(void *p);

#endif /* AN_PACE_H_ */
