/**************************************************************************//**
* @file bt_manager_task.c
* @brief blue tooth manager task
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef BT_MANAGER_TASK_H_
#define BT_MANAGER_TASK_H_

#include <stdint.h>
#include <stdbool.h>
#include "RpcFifo.h"
#include "FreeRTOS.h"
#include "semphr.h"



typedef enum BT_cmd
{
	BT_CMD_START,
	BT_CMD_STOP,
	BT_CMD_MAX
}BT_cmd_t;



int startBTMngTask(QueueHandle_t *cmdQ, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio);
int sendToBTManager(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr, uint32_t timeout);
int sendToBTManagerFromISR(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr);

int BL653_hw_test(void * arg);

bool is_bt_vsp_con(void);
void BL653_hw_reset(void);

#endif /* BT_MANAGER_TASK_H_ */
