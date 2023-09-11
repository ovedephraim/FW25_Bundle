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

	/* Bytes defined by the protocol. */
	#define X_SOH 	0x01	//((uint8_t)0x01u)  /**< Start Of Header (128 bytes). */
	#define X_STX	0x02	// ((uint8_t)0x02u)  /**< Start Of Header (1024 bytes). */
	#define X_EOT	0x04	// ((uint8_t)0x04u)  /**< End Of Transmission. */
	#define X_ACK 	0x06	//((uint8_t)0x06u)  /**< Acknowledge. */
	#define X_NAK 	0x15	//((uint8_t)0x15u)  /**< Not Acknowledge. */
	#define X_CAN 	0x18	//((uint8_t)0x18u)  /**< Cancel. */
	#define X_C  	0x43	// ((uint8_t)0x43u)  /**< ASCII "C" to notify the host we want to use CRC16. */
	#define Y_HEADER 0x67



typedef enum fota_state
{
	f_idle = 0,
	f_fota,
	f_header,
	f_x_stx,
	f_x_eot,
	f_error
}fota_state;

typedef struct {
	uint32_t f_fota_ena;
	fota_state fota;
}fota_sm;


int startfotaHndlTask(QueueHandle_t *cmdQ, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio);
void fota_handler_Task(void *para);
//int sendTofotaCmdInterp(QueueHandle_t q, void *packet, uint16_t hdr, uint32_t timeout);
int sendTofotaCmdInterp(uint8_t * buffer);

#ifdef __cplusplus
}
#endif

#endif /* _FOTA_HANDLER_TASK_H */
