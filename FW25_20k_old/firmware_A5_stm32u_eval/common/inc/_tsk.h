/**
* @file tsk.h
* @brief task functions wrapper
* @author Anton Kanaev
*
* @version 0.0.1
* @date 30.04.2022
*/
#ifndef __TSK_H
#define __TSK_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void _TSK_disable(void);
void _TSK_enable(void);

#ifdef __cplusplus
}
#endif

#endif
