/*
* @file GYR3X.h
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef GYR3X_H_
#define GYR3X_H_

#include "LSM6DSL/lsm6dsl_dev.h"

#define GYR3X_SAMPLE_RATE_MAX (DEF_ODR_ACC) //samples per second
#define DEF_GYR3X_SAMPLE_RATE (GYR3X_SAMPLE_RATE_MAX)

#define GYR3X_SAMPLE_RES_MAX (16) //bits per sample = 16 bit sample X 3 axes
#define DEF_GYR3X_SAMPLE_RES (GYR3X_SAMPLE_RES_MAX)

#define GYR3X_NUM_OF_SENSORS (3)//x,y,z

void GYR3X_Init(void *p);
void GYR3X_Reset();
void GYR3X_SetActivation(void *p, bool cmd);
bool GYR3X_Handler(void *p);

#endif /* GYR3X_H_ */
