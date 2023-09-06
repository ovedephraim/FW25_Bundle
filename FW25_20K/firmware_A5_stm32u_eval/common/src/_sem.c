/**
* @file _sem.c
* @brief semaphore functions wrapper
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 20.10.2015
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/
#include <stddef.h>
#include <freertos.h>
#include <queue.h>
#include <semphr.h>
#include "_sem.h"
#include "sys_port.h"

/**
 * @fn void *_SEM_create(int count, void *attrs)
 * A glue function to FreeRTOS counting semaphore creation function/macro with initial zero count value
 *
 * @author Anton Kanaev
 * @param count The maximum count value that can be reached. When the semaphore reaches this value it can no longer be 'given'.
 * @param attrs
 * @return Pointer to allocated counting semaphore. NULL=no semaphore was allocated
 * @date 31.08.2015
 */
void *_SEM_create(int count, void *attrs)
{
	return xSemaphoreCreateCounting(count,0);
}

/**
 * @fn void *_SEM_create_init(int count, int init_count, void *attrs)
 * A glue function to FreeRTOS counting semaphore creation function/macro
 *
 * @author Anton Kanaev
 * @param count The maximum count value that can be reached. When the semaphore reaches this value it can no longer be 'given'.
 * @param init_count The count value assigned to the semaphore when it is created.
 * @param attrs
 * @return Pointer to allocated counting semaphore. NULL=no semaphore was allocated
 * @date 31.08.2015
 */
void *_SEM_create_init(int count, int init_count, void *attrs)
{
	return xSemaphoreCreateCounting(count,init_count);
}

/**
 * @fn void _SEM_delete(void *semHandle)
 * A glue function to FreeRTOS semaphore deletion function/macro
 *
 * @author Anton Kanaev
 * @param semHandle Pointer to semaphore to be deleted.
 * @date 31.08.2015
 */
void _SEM_delete(void *semHandle)
{
	vSemaphoreDelete(semHandle);
}

/**
 * @fn int _SEM_pend(void *semHandle, unsigned int timeout)
 * A glue function to FreeRTOS semaphore taking function/macro
 *
 * @author Anton Kanaev
 * @param semHandle Pointer to the semaphore
 * @param timeout The time in ticks to wait for the semaphore to become
 * available.  The macro portTICK_PERIOD_MS can be used to convert this to a
 * real time.  A block time of zero can be used to poll the semaphore.  A block
 * time of portMAX_DELAY can be used to block indefinitely (provided
 * INCLUDE_vTaskSuspend is set to 1 in FreeRTOSConfig.h).
 * @return pdTRUE if the semaphore was obtained.  pdFALSE
 * if timeout expired without the semaphore becoming available.
 * @date 31.08.2015
 */
int _SEM_pend(void *semHandle, unsigned int timeout)
{
	return (int)xSemaphoreTake(semHandle,timeout);
}

/**
 * @fn int _SEM_post(void *semHandle)
 * A glue function to FreeRTOS semaphore posting functions/macros with in ISR detection
 *
 * @author Anton Kanaev
 * @param semHandle Pointer to the semaphore
 * @return If called inside ISR will return pdTRUE if giving the semaphore caused a task
 * to unblock, and the unblocked task has a priority higher than the currently
 * running task. A context switch should be requested before the interrupt is exited. Otherwise will return pdFALSE.
 * @date 31.08.2015
 */
int _SEM_post(void *semHandle)
{
	BaseType_t xHigherPriorityTaskWoken= pdFALSE;

	if (inIsr())
		xSemaphoreGiveFromISR(semHandle,&xHigherPriorityTaskWoken);
	else
		xSemaphoreGive(semHandle);
	return (int)xHigherPriorityTaskWoken;
}


