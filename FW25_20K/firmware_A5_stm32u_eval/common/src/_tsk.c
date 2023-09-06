/**
* @file _sem.c
* @brief semaphore functions wrapper
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 30.04.2022
*/
#include <stddef.h>
#include <freertos.h>
#include <task.h>
#include "_tsk.h"

void _TSK_disable(void)
{
	vTaskSuspendAll();
}

void _TSK_enable(void)
{
	xTaskResumeAll();
}

