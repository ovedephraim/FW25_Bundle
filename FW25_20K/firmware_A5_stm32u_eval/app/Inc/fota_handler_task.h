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

#define  backup_a5_program		0
#define  runtime_a5_program		1
#define  runtime_a5_write		2
#define  A5_signature			3

typedef struct flash_file_param
{
	// A5 backup.
	uint8_t f_magic_val0[16];// =  {0x32,0x54,0x6a,0x95,0x4a,0x2d,0x6e,0xa7,0x80,0x90,0xa0,0xb0,0xc0,0xd0,0xe0,0xf0};
	uint8_t f_file0[16];
	uint32_t f_size0;
	uint32_t f_blocks0;
	uint16_t f_crc0;

	// A5 runtime.
	uint8_t f_magic_val1[16];// = {0x23,0x45,0xa6,0x59,0xa4,0xd2,0xe6,0x7a,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
	uint8_t f_file1[16];
	uint32_t f_size1;
	uint32_t f_blocks1;
	uint16_t f_crc1;

	// A5 runtime write.
	uint8_t f_magic_val2[16];// = {0x45,0x18,0xd1,0xa5,0x5a,0xc8,0xe2,0xdb,0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf1};
	uint8_t f_file2[16];
	uint32_t f_size2;
	uint32_t f_blocks2;
	uint32_t f_origin2;
	uint16_t f_crc2;

}flash_file_param;


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
	uint32_t f_timer;
	fota_state fota;
}fota_sm;

#define UART_OK		0
#define UART_ERROR	0xff
#define UART_TIMEOUT		100
#define WRITE_BLOCK			1024


int startfFotaHndlTask(QueueHandle_t *cmdQ, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio);
void fota_handler_Task(void *para);
//int sendTofotaCmdInterp(QueueHandle_t q, void *packet, uint16_t hdr, uint32_t timeout);
int sendTofotaCmdInterp(uint8_t * buffer);

void flasherase(uint8_t full);
void erasesignature(uint8_t sel);
void write_A5_signature(uint8_t signature);
void write_signature(uint8_t signature);
uint32_t uart_receive(uint8_t *data, uint16_t length);




#ifdef __cplusplus
}
#endif

#endif /* _FOTA_HANDLER_TASK_H */
