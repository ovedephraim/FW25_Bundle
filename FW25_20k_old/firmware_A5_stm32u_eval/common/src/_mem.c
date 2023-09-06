/**
* @file mem.c
* @brief memory functions wrapper
*
* @version 0.0.1
* @date 20.10.2015
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/
#include <stddef.h>
#include <freertos.h>
#include "_mem.h"

/**
 * @fn void *_MEM_alloc(size_t size)
 * A glue function to FreeRTOS memory allocation function, "C" malloc() style
 *
 * @author Anton Kanaev
 * @param size Requested memory block size
 * @date 08.05.2022
 */
void *_MEM_alloc(size_t size)
{
	return pvPortMalloc(size);
}

/**
 * @fn void _MEM_free(void * p)
 * A glue function to FreeRTOS memory free function, "C" free() style
 *
 * @author Anton Kanaev
 * @param p Pointer to a memory block to be freed
 * @date 08.05.2022
 */
void _MEM_free(void * p)
{
	vPortFree(p);
}



