/*
* @file AN_AUDIO.h
* @brief analog audio channel
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef AN_AUDIO_H_
#define AN_AUDIO_H_

#include <stdbool.h>

#define AN_AUDIO_SAMPLE_RATE_MAX (1024)//samples per second
#define DEF_AN_AUDIO_SAMPLE_RATE (AN_AUDIO_SAMPLE_RATE_MAX)

#define AN_AUDIO_SAMPLE_RES_MAX (16)  //14 bit ADC resolution
#define DEF_AN_AUDIO_SAMPLE_RES (AN_AUDIO_SAMPLE_RES_MAX)

#define AN_AUDIO_NUM_OF_SENSORS (1)//14 bit ADC resolution

void AN_AUDIO_Init(void *p);
void AN_AUDIO_Reset();
void AN_AUDIO_SetActivation(void *p, bool cmd);
bool AN_AUDIO_Handler(void *p);

#endif /* AN_AUDIO_H_ */
