/**************************************************************************//**
 * @file leds_handler_task.h
 * @brief Leds  task
 *
 * @version 0.0.1
 * @date 20.10.2015
 *
 * @section License
 * <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/
#ifndef _LEDS_HANDLER_TASK_H
#define _LEDS_HANDLER_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#ifdef __cplusplus
	extern "C" {
#endif



int startLedsHndlTask(QueueHandle_t *leds, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio);
void leds_handler_Task(void *para);




#ifdef __cplusplus
}
#endif

#endif /* _LEDS_HANDLER_TASK_H */
