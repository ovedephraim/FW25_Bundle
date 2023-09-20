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
#ifndef _FOTA_HANDLER_TASK_H
#define _FOTA_HANDLER_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#ifdef __cplusplus
	extern "C" {
#endif


#define UART_OK		0
#define UART_ERROR	0xff
#define UART_TIMEOUT		100
#define WRITE_BLOCK			1024



int startFuelgaugeHndlTask(QueueHandle_t *fuelgauge, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio);
void fuelgauge_handler_Task(void *para);
//int sendTofotaCmdInterp(QueueHandle_t q, void *packet, uint16_t hdr, uint32_t timeout);
int sendTofuelgaugeCmdInterp(uint8_t * buffer);



#ifdef __cplusplus
}
#endif

#endif /* _FOTA_HANDLER_TASK_H */
