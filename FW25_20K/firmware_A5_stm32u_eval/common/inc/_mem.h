/**
* @file mem.h
* @brief memory functions wrapper
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 30.04.2022
*/
#ifndef __MEM_H
#define __MEM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void *_MEM_alloc(size_t size);
void _MEM_free(void * p);

#ifdef __cplusplus
}
#endif

#endif
