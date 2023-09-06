/**
* @file sem.h
* @brief semaphore functions wrapper
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 30.04.2022
*/
#ifndef _SEM_H
#define _SEM_H
	
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void *_SEM_create(int count, void *attrs);
void *_SEM_create_init(int count, int init_count, void *attrs);
void _SEM_delete(void *semHandle);
int _SEM_pend(void *semHandle, unsigned int timeout);
int _SEM_post(void *semHandle);


#ifdef __cplusplus
}
#endif

#endif


