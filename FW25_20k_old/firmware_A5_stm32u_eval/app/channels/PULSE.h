/*
* @file PULSE.h
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef PULSE_H_
#define PULSE_H_

#include <stdbool.h>

#define PULSE_SAMPLE_RATE_MAX (128) //samples per second
#define DEF_PULSE_SAMPLE_RATE (PULSE_SAMPLE_RATE_MAX)

#define PULSE_SAMPLE_RES_MAX (14) //bits per sample
#define DEF_PULSE_SAMPLE_RES (PULSE_SAMPLE_RES_MAX)

#define PULSE_NUM_OF_SENSORS (2)

void PULSE_Init(void *p);
void PULSE_Reset();
void PULSE_SetActivation(void *p, bool cmd);
bool PULSE_Handler(void *p);

#endif /* PULSE_H_ */
