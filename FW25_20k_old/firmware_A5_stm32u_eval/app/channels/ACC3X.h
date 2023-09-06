/*
* @file ACC3X.h
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef ACC3X_H_
#define ACC3X_H_

#include "LSM6DSL/lsm6dsl_dev.h"

#define ACC3X_SAMPLE_RATE_MAX (DEF_ODR_ACC) //samples per second
#define DEF_ACC3X_SAMPLE_RATE (ACC3X_SAMPLE_RATE_MAX)

#define ACC3X_SAMPLE_RES_MAX (16) //bits per sample = 16 bit sample X 3 axes
#define DEF_ACC3X_SAMPLE_RES (ACC3X_SAMPLE_RES_MAX)

#define ACC3X_NUM_OF_SENSORS (3)//x,y,z

void ACC3X_Init(void *p);
void ACC3X_Reset();
void ACC3X_SetActivation(void *p, bool cmd);
bool ACC3X_Handler(void *p);

#endif /* ACC3X_H_ */
