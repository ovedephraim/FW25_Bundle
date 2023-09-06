/**************************************************************************//**
 * @file cmd_task.h
 * @brief Command interpreter task
 *
 * @version 0.0.1
 * @date 20.10.2015
 *
 * @section License
 * <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/
#ifndef _CMD_HANDLER_TASK_H
#define _CMD_HANDLER_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#ifdef __cplusplus
	extern "C" {
#endif



int startCmdHndlTask(QueueHandle_t *cmdQ, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio);
void cmd_handler_Task(void *para);
int sendToAuxCmdInterp(QueueHandle_t q, void *packet, uint16_t hdr, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* _CMD_HANDLER_TASK_H */
