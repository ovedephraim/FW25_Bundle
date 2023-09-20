/**************************************************************************//**
* @file fuelgauge_handler_task.c
* @brief command interpreter task
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/


//#include "../../app/Inc/cmd_handler_task.h"
#include "../../app/Inc/fuelgauge_handler_task.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "misc.h"

#include "../../app/Inc/main.h"
#include "../../app/Inc/sys_errno.h"
#include "../../app/Inc/ver.h"



//void Error_Handlerb(uint8_t *file, uint32_t line)
//{
//	char dbug[200]={0};
//
//	sprintf(dbug,"[FATAL] file %s on line %d \r\n",
//			file,(int)line);
//
//	/* send to debug aux channel */
////	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
//
//	/* wait for print out */
////	vTaskDelay(1000);
//
//	/* desable interrupts */
//
//	__disable_irq();
//	while (1)
//	{
//	}
//}



void fuelgauge_handler_Task(void *para)
{

	for (;;)
	{
		vTaskDelay(3000);

		BatterySuperviser();
	}
}


int startFuelgaugeHndlTask(QueueHandle_t *fuelgauge, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio)
{
	QueueHandle_t q;
	TaskHandle_t t;

	xTaskCreate(fuelgauge_handler_Task, pcName, usStackDepth, q, prio, &t);

}/* end of startFuelgaugeHndlTask */


