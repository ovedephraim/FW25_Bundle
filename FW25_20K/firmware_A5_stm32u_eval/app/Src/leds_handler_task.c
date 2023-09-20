/**************************************************************************//**
* @file cmd_task.c
* @brief command interpreter task
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include "../../app/Inc/leds_handler_task.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lp55281.h"

#include "../../app/Inc/main.h"
#include "../../app/Inc/sys_errno.h"
#include "../../app/Inc/ver.h"


void leds_handler_Task(void *para)
{
	for (;;)
	{
		vTaskDelay(1000);

		Led_SM();
	}
}

int startLedsHndlTask(QueueHandle_t *leds, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio)
{
	TaskHandle_t t;
	QueueHandle_t q;

	xTaskCreate(leds_handler_Task, pcName, usStackDepth, q, prio, &t);

}/* end of startLedsHndlTask */


