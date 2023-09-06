/*
 * aux_task.c.c
 * @author Anton Kanaev
 *
 * @version 0.0.1
 * @date 20.10.2015
 *
 * @section License
 * <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include <A4_proto/a4comm.h>
#include <A4_proto/a4frame.h>
#include <stdint.h>
#include <freertos.h>
#include <task.h>
#include <queue.h>
#include "stm32u5xx_hal.h"
#include "membuf.h"
#include "auxcmd.h"
#include "_mem.h"
#include "membuf.h"
#include "msg_type.h"
#include "packetbuf.h"
#include "rxframer.h"
#include "A5_proto/parcel.h"

#include "stm32u5xx_hal_uart_ex.h"
#include "cmd_handler_task.h"
#include "main.h"
#include "sys_conf.h"



//#define DEBUG_COMMAND 1

xQueueHandle auxOutQ=NULL;
MEMBUF_POOL auxdbgBufPool;
MEMBUF_POOL auxdbgbigBufPool;

extern UART_HandleTypeDef huart5;
extern  __IO ITStatus Uart_dbg_TxDone;
extern UART_HandleTypeDef huart4;
extern  __IO ITStatus Uart_ble_TxDone;

const bool debug_serial_enable=ENABLE_DEBUG_SERIAL_LOG;
const bool proto_serial_enable=ENABLE_PROTO_SERIAL_LOG;

/**
 * @brief send to auxulaty communication task
 * @brief This function transfers packet types via desiered channel
 * @brief if (pdFAIL==sendToAux( resp, MSGHDR_AUXCMDRESP_PACKET, portMAX_DELAY))
 * @param packet - data
 * @param hdr - data header
 * @param timeout - tx timeout
 * @returns 0-if no error.  A non-zero value indicates an error.
 */
int aux_sendToAux(void *packet,
		uint32_t len,
		uint32_t timeout,
		bool _alloc,
		uint8_t aux) //BLE_AUX
{
	MSG_HDR msg;
	void * buff=packet;

	if (auxOutQ==NULL)
		return pdFAIL;


	if(aux==BLE_AUX)
	{
		msg.hdr.bit.type=MSG_TYPE_BT_DATA_BLK;
	}
	else if(aux==DBG_AUX)
	{
    	if(!debug_serial_enable)
    	{
    		return pdPASS;
    	}
    	msg.hdr.bit.type=MSG_TYPE_DEBUG_DATA_BLK;
    }
	else if(aux==DBG_AUX_PROTO)
	{
    	if(!proto_serial_enable)
    	{
    		return pdPASS;
    	}
    	msg.hdr.bit.type=MSG_TYPE_DEBUG_DATA_BLK;
	}
	else
    {
    	return pdFAIL;
    }

	if(_alloc)
	{
		if(!packet || !len || (len > DBG_TX_BBUFF_SIZE))
			return pdFAIL;

		if(len > DBG_TX_BUFF_SIZE)
		{
			buff=getMemBuf(&auxdbgbigBufPool);
		}
		else
		{
			buff=getMemBuf(&auxdbgBufPool);
		}

	    if(!buff)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
			return pdFAIL;
		}

		memcpy(buff,packet,len);
	}

	msg.data=len;
	msg.buf=buff;
	if (inIsr())
		return xQueueSendFromISR(auxOutQ,&msg,0);
	return xQueueSend(auxOutQ,&msg,0);

}/* End of sendToAux */

/**
 * @brief send to auxulaty communication task
 * @brief This function transfers packet types via desiered channel
 * @brief if (pdFAIL==sendToAux( resp, MSGHDR_AUXCMDRESP_PACKET, portMAX_DELAY))
 * @param packet - data
 * @param hdr - data header
 * @param timeout - tx timeout
 * @returns 0-if no error.  A non-zero value indicates an error.
 */
int aux_sendPacketToAux(void *packet,
		uint32_t len,
		uint16_t hdr,
		uint32_t timeout)
{
	MSG_HDR msg;

	if (auxOutQ==NULL)
		return pdFAIL;
	msg.hdr.all = hdr;
	msg.data=len;
	msg.buf=packet;

	if (inIsr())
		return xQueueSendFromISR(auxOutQ,&msg,0);
	return xQueueSend(auxOutQ,&msg,timeout);

}/* End of sendToAux */


/**
 * @brief debug aux tx task
 * @brief This function handles data transfer (tx) from different sources
 * @param para - extra arguments
 * @returns none
 */
void aux_TxTask(void *para)
{
	MSG_HDR aux_out_msg;
    size_t	len = 0;
    uint8_t * pld = NULL;

	for (;;)
	{
		if (xQueueReceive(auxOutQ,&aux_out_msg,portMAX_DELAY)==pdPASS)
		{
			if (aux_out_msg.hdr.bit.type==MSG_TYPE_PACKET)
			{

			}else
			if (aux_out_msg.hdr.bit.type==MSG_TYPE_DEBUG_DATA_BLK)
			{
				if (aux_out_msg.buf)
				{
					pld = aux_out_msg.buf;
					len = aux_out_msg.data;

					while(huart5.gState != HAL_UART_STATE_READY)
					{
						vTaskDelay(1);
					}

			        /* reset global sync */
					Uart_dbg_TxDone = RESET;

					/* blocking DMA send to uart   */
					if (HAL_OK != HAL_UART_Transmit_DMA(&huart5, pld, len)){
						Error_Handler((uint8_t *)__FILE__, __LINE__);
					}

					/* TODO go sleep  and wakeup when DMA complete */
					while(Uart_dbg_TxDone != SET)
					{
						vTaskDelay(1);
					}

					/* free allocated memory */
					retMemBuf(aux_out_msg.buf);
				}
			}
			else
			if (aux_out_msg.hdr.bit.type==MSG_TYPE_BT_DATA_BLK)
			{
				if (aux_out_msg.buf)
				{
					pld = aux_out_msg.buf;
					len = aux_out_msg.data;

					/* reset global sync */
					Uart_ble_TxDone = RESET;

					/* wait for driver ready */
					while(huart4.gState != HAL_UART_STATE_READY)
					{
						vTaskDelay(1);
					}

					/* blocking DMA send to uart   */
					if (HAL_OK != HAL_UART_Transmit_DMA(&huart4, pld, len)){
						Error_Handler((uint8_t *)__FILE__, __LINE__);
					}

					/* wait for tx done by dma */
					while(Uart_ble_TxDone != SET){
						vTaskDelay(1);
					}


					/* free allocated memory */
					retMemBuf(aux_out_msg.buf);
				}
			}
			else
			if (aux_out_msg.hdr.bit.type==MSG_TYPE_BT_PACKET)
			{

				if (aux_out_msg.buf)
				{
					pld = PACKETBUF_DATA(aux_out_msg.buf);
					len = ((PACKETBUF_HDR *)aux_out_msg.buf)->dlen;

					/* reset global sync */
					Uart_ble_TxDone = RESET;

					/* wait for driver ready */
					while(huart4.gState != HAL_UART_STATE_READY)
					{
						vTaskDelay(1);
					}


					/* blocking DMA send to uart   */
					if (HAL_OK != HAL_UART_Transmit_DMA(&huart4, pld, len)){
						Error_Handler((uint8_t *)__FILE__, __LINE__);
					}

					/* wait for tx done by dma */
					while(Uart_ble_TxDone != SET){
						vTaskDelay(20);
					}

					/* free allocated memory */
					retMemBuf(aux_out_msg.buf);
				}
			}
		}
	}
}/* End of aux_dbg_TxTask */




