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
#include "../../app/Inc/fota_handler_task.h"

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
static PACKETBUF_HDR * handlefotaCmdPacket(MEMBUF_POOL *pool, PACKETBUF_HDR * p, SemaphoreHandle_t sync_sm);

fota_sm _fota_sm;
extern UART_HandleTypeDef huart5;

int16_t crc_received,crc_calculated;


void fota_handler_Task(void *para)
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
}

int startfotaHndlTask(QueueHandle_t *cmdQ, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio)
{
	QueueHandle_t q;
	TaskHandle_t t;

	/* create queue */
	if ((q = xQueueCreate(24,sizeof(MSG_HDR))) == NULL)
	{
		if (cmdQ)
			*cmdQ=NULL;
		if (cmdT)
			*cmdT=NULL;
		return pdFAIL;
	}

	if (pdFAIL == xTaskCreate(fota_handler_Task, pcName, usStackDepth, q, prio, &t))
	{
		vQueueDelete(q);
		if (cmdQ)
			*cmdQ=NULL;
		if (cmdT)
			*cmdT=NULL;
		return pdFAIL;
	}
	vQueueAddToRegistry( q, pcName);
	if (cmdQ)
		*cmdQ=q;
	if (cmdT)
		*cmdT=t;
	return pdPASS;
}/* end of startCmdTask */


static PACKETBUF_HDR * handlefotaCmdPacket(MEMBUF_POOL *pool, PACKETBUF_HDR * p, SemaphoreHandle_t sync_sm)
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


void uart_transmit_ch(uint8_t data)
{
	uint8_t a[2];

	a[0] = data;

    HAL_UART_Transmit_IT(&huart5, (uint8_t *)a, 1);
}

uint32_t Transmit(uint8_t k)
{
	 uint32_t result = 0;

	 UartReadyTx = false;
	 uart_transmit_ch(k);
	 while(UartReadyTx == false) {};

	 return result;
}


/**
 * @brief   Calculates the CRC-16 for the input package.
 * @param   *data:  Array of the data which we want to calculate.
 * @param   length: Size of the data, either 128 or 1024 bytes.
 * @return  status: The calculated CRC.
 */
static uint16_t calc_crc(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0u;
    while (length)
    {
        length--;
        crc = crc ^ ((uint16_t)*data++ << 8u);
        for (uint8_t i = 0u; i < 8u; i++)
        {
            if (crc & 0x8000u)
            {
                crc = (crc << 1u) ^ 0x1021u;
            }
            else
            {
                crc = crc << 1u;
            }
        }
    }
    return crc;
}

uint32_t checkyheader(uint8_t * buffer)
{
	uint32_t result = 0;
	int16_t crc_received,crc_calculated;
	uint32_t size,checksum;


	if(buffer[0] != Y_HEADER)
	{
		//	transmit_notification(&message_err[0]);
		result = 0xff;
		return result;
	}

	if((buffer[1] != 'A') || (buffer[2] != 'p') || (buffer[3] != 'p') ||
	   (buffer[4] != 'l') || (buffer[5] != 'i') || (buffer[6] != 'c') ||
       (buffer[7] != 'a') || (buffer[8] != 't') || (buffer[9] != 'i') ||
	   (buffer[10] != 'o') || (buffer[11] != 'n'))
	{
	//	transmit_notification(&message_err[0]);
		result = 0xff;
		return result;
	}

	size = buffer[12] +
		  (buffer[13] * 0x100) +
		  (buffer[14] * 0x10000) +
		  (buffer[15] * 0x1000000);

	checksum = buffer[16] + (buffer[17] * 0x100);

//	_flash_file_param.f_crc2 = _flash_file_param.f_crc2;
//	_flash_file_param.f_blocks2 = size;


	crc_received = ((uint16_t)buffer[19] << 8u) | ((uint16_t)buffer[18]);

	/* We calculate it too. */
    crc_calculated = calc_crc(&buffer[1],17);

    if(crc_received != crc_calculated)
    {
      // transmit_notification(&crc_bad[0]);
      HAL_Delay(1000);
	  result = 0xff;
	}

	return result;
}


uint32_t checkmainchunk(uint8_t * buffer)
{
	uint32_t result = 0;
//	int16_t crc_received,crc_calculated;

	uint8_t abc[DATA_CHUNK];

	memcpy(&abc[0],&buffer[0],DATA_CHUNK);

	if(buffer[0] != X_STX)
	{
	  Transmit(0x16);
	  result = 0xff;
	  return result;
	}

	if((buffer[1] + buffer[2]) != 0xff)
	{
	  Transmit(0x17);
	  result = 0xff;
	  return result;
	}

	crc_received = buffer[DATA_CHUNK - 1] + (buffer[DATA_CHUNK - 2] * 0x100);

	/* We calculate it too. */
	crc_calculated = calc_crc(&buffer[3],DATA_CHUNK - 5);

	//abc[12] = 0X33;

	if(crc_received != crc_calculated)
	{
	  //	transmit_notification(&crc_bad[0]);
	  Transmit(0x18);
	  result = 0xff;
	  return result;
	}
	 return result;
}

int sendTofotaCmdInterp(uint8_t * buffer)
//int sendTofotaCmdInterp(QueueHandle_t q, void *packet, uint16_t hdr, uint32_t timeout)
{
//	MSG_HDR msg;
	uint32_t result;
//	uint8_t abc[DATA_CHUNK];
    static uint8_t cnt = 0;

	if(_fota_sm.f_fota_ena == 1)
	{
//		memcpy(&abc[0],&buffer[0],DATA_CHUNK);
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
							if(result == 0)
							{
								_fota_sm.fota = f_x_stx;
								Transmit(X_ACK);
								HAL_Delay(200);
							}
							else
							{
								Transmit(X_NAK);
							}
							_fota_sm.f_timer = 0;
						    break;
			case f_x_stx:
						//	memcpy(&abc[0],&buffer[0],1029);

				            if(++cnt % 20 == 1)
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
								_fota_sm.fota = f_x_stx;
								Transmit(X_ACK);
								HAL_Delay(200);
							}
							else
							{
								_fota_sm.fota = f_idle;
								Transmit(X_NAK);
							}
							_fota_sm.f_timer = 0;
						    break;
			case f_x_eot:
							Transmit(X_ACK);
							_fota_sm.f_fota_ena = 0;
							_fota_sm.fota = f_idle;
						    break;
			case f_error:
				            Transmit(X_ACK);
				            HAL_Delay(200);
//							Transmit(X_NAK);
//							_fota_sm.f_fota_ena = 0;
//							_fota_sm.fota = f_idle;
						    break;

		}

	}

//	if (q==NULL)
//		return pdFAIL;
//	msg.hdr.all = hdr;
//	msg.data=0;
//	msg.buf=packet;
//	return xQueueSend(q,&msg,timeout);
}/* End of sendToAuxCmdInterp */


