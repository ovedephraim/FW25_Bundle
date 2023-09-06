/*
* @file AN_RESP.h
* @brief analog respiration channel
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef AN_RESP_H_
#define AN_RESP_H_

#include <stdbool.h>

#define AN_RESP_SAMPLE_RATE_MAX (1024)//samples per second
#define DEF_AN_RESP_SAMPLE_RATE (AN_RESP_SAMPLE_RATE_MAX)

#define AN_RESP_SAMPLE_RES_MAX (16)  //14 bit ADC resolution
#define DEF_AN_RESP_SAMPLE_RES (AN_RESP_SAMPLE_RES_MAX)

#define AN_RESP_NUM_OF_SENSORS 1

void AN_RESP_Init(void *p);
void AN_RESP_Reset();
void AN_RESP_SetActivation(void *p, bool cmd);
bool AN_RESP_Handler(void *p);

#endif /* AN_RESP_H_ */
