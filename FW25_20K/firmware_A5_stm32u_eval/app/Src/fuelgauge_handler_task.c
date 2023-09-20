/**************************************************************************//**
* @file cmd_task.c
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
#include "queue.h"
#include "semphr.h"
#include "_sem.h"
#include "_mem.h"
#include "msg_type.h"
#include "membuf.h"
#include "A4_proto/a4opc.h"
#include "A4_proto/a4err.h"
#include "A4_proto/a4frame.h"
#include "A4_proto/Parser.h"
#include "packetbuf.h"
#include "stub.h"
#include "auxcmd.h"

#include "../../app/Inc/main.h"
#include "../../app/Inc/sys_errno.h"
#include "../../app/Inc/ver.h"

#define A4_TX_BUFFER_SIZE	264
#define N_CMD_TX_BUFFERS	2

uint8_t UartReadyTx  = RESET;

#define Y_HEADER 0x67

extern IWDG_HandleTypeDef hiwdg;
extern QueueHandle_t samplerq;
static PACKETBUF_HDR * handleuelgaugeCmdPacket(MEMBUF_POOL *pool, PACKETBUF_HDR * p, SemaphoreHandle_t sync_sm);


extern UART_HandleTypeDef huart5;



void Error_Handlerb(uint8_t *file, uint32_t line)
{
	char dbug[200]={0};

	sprintf(dbug,"[FATAL] file %s on line %d \r\n",
			file,(int)line);

	/* send to debug aux channel */
//	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	/* wait for print out */
//	vTaskDelay(1000);

	/* desable interrupts */

	__disable_irq();
	while (1)
	{
	}
}



void fuelgauge_handler_Task(void *para)
{
	QueueHandle_t q=(QueueHandle_t)para;
	SemaphoreHandle_t sm=NULL;
	MSG_HDR msg;
	//PACKETBUF_HDR *resp = NULL;
	MEMBUF_POOL txBufPool;
	void *p = NULL;

#if 0
	/* create synchronization semaphore */
	sm=(SemaphoreHandle_t)_SEM_create(1,NULL);

	/* Create  buffers pool */
	if(NULL == (p=_MEM_alloc((A4_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_TX_BUFFERS)))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	initMemBufPool(&txBufPool,p,(A4_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_TX_BUFFERS, A4_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR),N_CMD_TX_BUFFERS);
	for (;;)
		if (xQueueReceive(q, &msg, portMAX_DELAY))
		{
			if (msg.hdr.bit.type==MSG_TYPE_CMD)//commands handling
			{

				if (A4ValidFrame(PACKETBUF_DATA(msg.buf), ((PACKETBUF_HDR *)msg.buf)->dlen))
				{
					/* Execute command */
					handlefotaCmdPacket(&txBufPool, (PACKETBUF_HDR *) msg.buf, sm);

					/* free command buffer */
					if(retMemBuf(msg.buf))
					{
						Error_Handler((uint8_t *)__FILE__, __LINE__);
					}
				}
				else
				{
					retMemBuf(msg.buf);
				}
			}
			else
			if (msg.hdr.bit.type==MSG_TYPE_NOTIFY)
			{
				if (msg.hdr.bit.source==MSG_SRC_DSP){

				}
				else
					retMemBuf(msg.buf);
			}
			else
			{
				retMemBuf(msg.buf);
			}
		}
#endif
}

int startFuelgaugeHndlTask(QueueHandle_t *fuelgauge, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio)
{
	QueueHandle_t q;
	TaskHandle_t t;

	/* create queue */
	if ((q = xQueueCreate(24,sizeof(MSG_HDR))) == NULL)
	{
		if (fuelgauge)
			*fuelgauge=NULL;
		if (cmdT)
			*cmdT=NULL;
		return pdFAIL;
	}

	if (pdFAIL == xTaskCreate(fuelgauge_handler_Task, pcName, usStackDepth, q, prio, &t))
	{
		vQueueDelete(q);
		if (fuelgauge)
			*fuelgauge=NULL;
		if (cmdT)
			*cmdT=NULL;
		return pdFAIL;
	}
	vQueueAddToRegistry( q, pcName);
	if (fuelgauge)
		*fuelgauge=q;
	if (cmdT)
		*cmdT=t;
	return pdPASS;
}/* end of startCmdTask */


static PACKETBUF_HDR * handlefuelgaugeCmdPacket(MEMBUF_POOL *pool, PACKETBUF_HDR * p, SemaphoreHandle_t sync_sm)
{
	PACKETBUF_HDR * b=NULL;
	uint32_t cmd_len = 0;
	char *pData = NULL;

	pData = (char*)PACKETBUF_DATA(p);
	cmd_len = p->dlen;

	if(cmd_len > 0)
	{
		pData[cmd_len]='\0';

		#ifdef DEBUG_COMMAND
		PrintLogBuffer(pData, cmd_len);
		#endif

		/* Handle commands and build output string */
		PARSER_ParseCommand((char *)pData, &b, pool);
	}
	return b;
}/* End of handleAuxCmdPacket */



int sendTofuelgaugeCmdInterp(uint8_t * buffer)
//int sendTofotaCmdInterp(QueueHandle_t q, void *packet, uint16_t hdr, uint32_t timeout)
{
//	MSG_HDR msg;
	uint32_t result;
	uint8_t abc[DATA_CHUNK];
	static uint8_t cnt = 0;
	static uint16_t index = 0;

#if 0
	if(_fota_sm.f_fota_ena == 1)
	{
		memcpy(&abc[0],&buffer[0],DATA_CHUNK);
	//	switch(&packet[0])
		switch(buffer[0])
		{
			case Y_HEADER:
							_fota_sm.fota = f_header;
							break;
			case X_STX:
							_fota_sm.fota = f_x_stx;
							break;
			case X_EOT:
				            _fota_sm.fota = f_x_eot;
							break;
			default:
							_fota_sm.fota = f_error;
							break;
		}

		switch(_fota_sm.fota)
		{
			case f_idle:
						 break;
			case f_fota:
						 break;
			case f_header:
						//	memcpy(&abc[0],&buffer[0],50);
							result = checkyheader(&buffer[0]);
						//	result = 0;
							if(result == 0)
							{
								// Erase top of flash.
								//flasherase(0);

								_fota_sm.fota = f_x_stx;
								Transmit(X_ACK);
								HAL_Delay(200);
								index = 0;
							}
							else
							{
								Transmit(X_NAK);
								HAL_Delay(200);
							}
							_fota_sm.f_timer = 0;
						    break;
			case f_x_stx:
						//	memcpy(&abc[0],&buffer[0],DATA_CHUNK);

				            if(++cnt % 30 == 1)
				            {
								/* Refresh IWDG: reload counter */
								if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
								{
								  /* Refresh Error */
								  Error_Handler((uint8_t *)__FILE__, __LINE__);
								}
				            }

							result = checkmainchunk(&buffer[0]);
						//	result = 0;
							if(result == 0)
							{
								//index = buffer[0] - 1;
								// change later destination of flash addr.
								//flash_transfer(FLASH_RUNTIME_STORE,index);
								//  flash_write(index);
					         	///		flash_read(FLASH_BACKUP_START_ADDR + index * WRITE_BLOCK,&compare_block[0],WRITE_BLOCK);
								index++;
								_fota_sm.fota = f_x_stx;
								Transmit(X_ACK);
								HAL_Delay(200);
							}
							else
							{
								_fota_sm.fota = f_idle;
								Transmit(X_NAK);
								 HAL_Delay(200);
							}
							_fota_sm.f_timer = 0;
						    break;
			case f_x_eot:
							Transmit(X_ACK);
							HAL_Delay(200);
							_fota_sm.f_fota_ena = 0;
							_fota_sm.fota = f_idle;

//							erasesignature(A5_signature);
//							write_A5_signature(runtime_a5_write);
//							erasesignature(runtime_a5_program);
//							write_signature(runtime_a5_program);
//							_JumpToProgram(FLASH_BACKUP_START_ADDR);

						    break;
			case f_error:
				            Transmit(X_NAK);
				            HAL_Delay(200);
//							Transmit(X_NAK);
//							_fota_sm.f_fota_ena = 0;
//							_fota_sm.fota = f_idle;
						    break;

		}

	}

#endif
}/* End of sendToAuxCmdInterp */


