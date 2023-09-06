/**************************************************************************//**
 * @file dispatcher_task.h
 * @brief dispatcher task
 *
 * @version 0.0.1
 * @date 30.04.2022
 *
 * @section License
 * <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/
#ifndef _DISPATCHER_TASK_H
#define _DISPATCHER_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#ifdef __cplusplus
	extern "C" {
#endif



typedef enum dispatcher_cmds
{
	start_exec,
	abort_exec,
	send_parcel
} dispatcher_cmds_t;

int startDispatcherTask(QueueHandle_t *cmdQ, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio);
int sendToDispatcher(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr, uint32_t timeout);
int sendToDispatcherFromISR(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr);
void dispatcherTask(void *para);

void set_periodic_dispatcher(uint32_t t);
#ifdef __cplusplus
}
#endif

#endif /* _DISPATCHER_TASK_H */
