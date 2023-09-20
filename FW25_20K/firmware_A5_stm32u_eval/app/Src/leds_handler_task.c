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
#include "../../app/Inc/leds_handler_task.h"

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
#include "lp55281.h"

#include "../../app/Inc/main.h"
#include "../../app/Inc/sys_errno.h"
#include "../../app/Inc/ver.h"

#define A4_TX_BUFFER_SIZE	264
#define N_CMD_TX_BUFFERS	2

uint8_t UartReadyTx  = RESET;

#define Y_HEADER 0x67

extern IWDG_HandleTypeDef hiwdg;
extern QueueHandle_t samplerq;
static PACKETBUF_HDR * handleledsCmdPacket(MEMBUF_POOL *pool, PACKETBUF_HDR * p, SemaphoreHandle_t sync_sm);

//leds_sm _leds_sm;
extern UART_HandleTypeDef huart5;





void leds_handler_Task(void *para)
{
	QueueHandle_t q=(QueueHandle_t)para;
	SemaphoreHandle_t sm=NULL;
	MSG_HDR msg;
	//PACKETBUF_HDR *resp = NULL;
	MEMBUF_POOL txBufPool;
	void *p = NULL;

	/* create synchronization semaphore */
	sm=(SemaphoreHandle_t)_SEM_create(1,NULL);

	/* Create  buffers pool */
	if(NULL == (p=_MEM_alloc((A4_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_TX_BUFFERS)))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	for (;;)
	{
		vTaskDelay(1000);

		Led_SM();
	}

#if 0
	initMemBufPool(&txBufPool,p,(A4_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_TX_BUFFERS, A4_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR),N_CMD_TX_BUFFERS);
	for (;;)
		if (xQueueReceive(q, &msg, portMAX_DELAY))
		{
			if (msg.hdr.bit.type==MSG_TYPE_CMD)//commands handling
			{

				if (A4ValidFrame(PACKETBUF_DATA(msg.buf), ((PACKETBUF_HDR *)msg.buf)->dlen))
				{
					/* Execute command */
					handleledsCmdPacket(&txBufPool, (PACKETBUF_HDR *) msg.buf, sm);

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

int startLedsHndlTask(QueueHandle_t *leds, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio)
{
	TaskHandle_t t;
	QueueHandle_t q;

	xTaskCreate(leds_handler_Task, pcName, usStackDepth, q, prio, &t);

#if 0
	QueueHandle_t q;
	TaskHandle_t t;



	/* create queue */
	if ((q = xQueueCreate(24,sizeof(MSG_HDR))) == NULL)
	{
		if (leds)
			*leds=NULL;
		if (cmdT)
			*cmdT=NULL;
		return pdFAIL;
	}

	if (pdFAIL == xTaskCreate(leds_handler_Task, pcName, usStackDepth, q, prio, &t))
	{
		vQueueDelete(q);
		if (leds)
			*leds=NULL;
		if (cmdT)
			*cmdT=NULL;
		return pdFAIL;
	}
	vQueueAddToRegistry( q, pcName);
	if (leds)
		*leds=q;
	if (cmdT)
		*cmdT=t;
	return pdPASS;
#endif

}/* end of startCmdTask */


static PACKETBUF_HDR * handleledsCmdPacket(MEMBUF_POOL *pool, PACKETBUF_HDR * p, SemaphoreHandle_t sync_sm)
{
	PACKETBUF_HDR * b=NULL;
	uint32_t cmd_len = 0;
	char *pData = NULL;

	pData = (char*)PACKETBUF_DATA(p);
	cmd_len = p->dlen;

#if 0
	if(cmd_len > 0)
	{
		pData[cmd_len]='\0';

		#ifdef DEBUG_COMMAND
		PrintLogBuffer(pData, cmd_len);
		#endif

		/* Handle commands and build output string */
		PARSER_ParseCommand((char *)pData, &b, pool);
	}
#endif
	return b;
}/* End of handleAuxCmdPacket */


int sendToledsCmdInterp(void)
//int sendTofotaCmdInterp(QueueHandle_t q, void *packet, uint16_t hdr, uint32_t timeout)
{
//	MSG_HDR msg;
	uint32_t result = 0;

	Led_SM();

    return result;
}/* End of sendToAuxCmdInterp */


