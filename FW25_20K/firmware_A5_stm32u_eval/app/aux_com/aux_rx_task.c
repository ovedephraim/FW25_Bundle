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
#include "bt_manager_task.h"
#include "sys_conf.h"

#include "stm32u5xx_hal_uart_ex.h"
#include "cmd_handler_task.h"
#include "main.h"
#include "sys_conf.h"


//#define DEBUG_COMMAND 1
#define DEBUG_BT_COMMAND 1
#define DEBUG_BT_ECHO    0

#define A4_RX_BUFFER_SIZE	RX_A4_MAX_PAYLOAD_LEN
#define N_COM_RX_BUFFERS	3
#define AUX_RX_TIMEOUT	    100 //portMAX_DELAY

extern QueueHandle_t cmdq;

extern void aux_TxTask(void *para);
extern void aux_RxTask(void *para);

extern xQueueHandle auxOutQ;
extern MEMBUF_POOL auxdbgBufPool;
extern MEMBUF_POOL auxdbgbigBufPool;

extern uint8_t UartReadyTx;
extern uint8_t UartReadyRx;
extern uint8_t fota_mode;

xQueueHandle auxInQ=NULL;
struct sRxFramer rxFramer;

DMA_HandleTypeDef handle_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel2;
DMA_HandleTypeDef handle_GPDMA1_Channel3;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart4;

 __IO ITStatus Uart_dbg_TxDone = RESET;
 __IO ITStatus Uart_ble_TxDone = RESET;

#if (ENABLE_DBG_COM_AUX == 1U)

 /* TODO use bugger per channel - Buffer used for reception */
//static uint8_t aDbgRxBuffer[RXBUFFERSIZE];
 uint8_t aDbgRxBuffer[RXBUFFERSIZE];

static void DEBUG_GPDMA_Init(void);
static void DEBUG_UARTX_Init(void);
static callback_aux_t cb;
static void DEBUG_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif



#if (ENABLE_BLE_COM_AUX == 1U)

/* TODO use bugger per channel - Buffer used for reception */

static uint8_t aBtRxBuffer[BTRXBUFFERSIZE];


static void BT_GPDMA_Init(void);
static void BT_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
static callback_aux_t bt_cb=NULL;

#endif

//----------- API --------------

/**
* @brief aux channel initialization
* low level intitialization
* @param huart: UART handle pointer
* @retval None
*/
void initAuxDevParams(void)
{
	#if (ENABLE_DBG_COM_AUX == 1U)

	void * p=NULL;

	if ((auxOutQ=xQueueCreate(N_DBG_TX_BUFFERS,sizeof(MSG_HDR)))!=NULL)
		vQueueAddToRegistry( auxOutQ, (const char *)"auxOutQ");

	/* allocate memory area for the memory pool */
	p=_MEM_alloc((DBG_TX_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_DBG_TX_BUFFERS);
	if(p==NULL){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* Create memory pool of fixed size buffers  */
	if(initMemBufPool(&auxdbgBufPool,p,
			(DBG_TX_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_DBG_TX_BUFFERS,
			DBG_TX_BUFF_SIZE+sizeof(PACKETBUF_HDR),N_DBG_TX_BUFFERS))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* allocate memory area for the memory pool */
	p=_MEM_alloc((DBG_TX_BBUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_DBG_TX_BBUFFERS);
	if(p==NULL){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* Create memory pool of fixed size buffers  */
	if(initMemBufPool(&auxdbgbigBufPool,p,
			(DBG_TX_BBUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_DBG_TX_BBUFFERS,
			DBG_TX_BBUFF_SIZE+sizeof(PACKETBUF_HDR),N_DBG_TX_BBUFFERS))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}


	DEBUG_GPDMA_Init();
	DEBUG_UARTX_Init();

	#endif

	#if (ENABLE_BLE_COM_AUX == 1U)

	BT_GPDMA_Init();
	BT_UARTX_Init(DEF_BT_BAUD);

	#endif

}/* end of initAuxDevParams */

/**
  * @brief  This function initiates RX transfer
  * @param auz_type - DBG_AUX/BT_AUX
  * @retval None
  */
void startAuxReception(callback_aux_t _cb, int aux_type)
{

#if (ENABLE_DBG_COM_AUX == 1U)

	if(aux_type == DBG_AUX)
	{
		/* assign user callback */
		cb = _cb;

		/* Initializes Rx sequence using Reception To Idle event API.
		As DMA channel associated to UART Rx is configured as Circular,
		reception is endless.
		If reception has to be stopped, call to HAL_UART_AbortReceive() could be used.
		Use of HAL_UARTEx_ReceiveToIdle_DMA service, will generate calls to
		user defined HAL_UARTEx_RxEventCallback callback for each occurrence of
		following events :
		- DMA RX Half Transfer event (HT)
		- DMA RX Transfer Complete event (TC)
		- IDLE event on UART Rx line (indicating a pause is UART reception flow)
		*/
		if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(&huart5,\
				aDbgRxBuffer, sizeof(aDbgRxBuffer)))
		{
			char * str="\r\n[DBG AUX] fail start RX\r\n";
		    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
		}
	}

#endif

#if (ENABLE_BLE_COM_AUX == 1U)

	if(aux_type == BLE_AUX)
	{
	    char * str="\r\n[BT AUX] start RX\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);

		bt_cb=_cb;

		/* Initializes Rx sequence using Reception To Idle event API.
		As DMA channel associated to UART Rx is configured as Circular,
		reception is endless.
		If reception has to be stopped, call to HAL_UART_AbortReceive() could be used.
		Use of HAL_UARTEx_ReceiveToIdle_DMA service, will generate calls to
		user defined HAL_UARTEx_RxEventCallback callback for each occurrence of
		following events :
		- DMA RX Half Transfer event (HT)
		- DMA RX Transfer Complete event (TC)
		- IDLE event on UART Rx line (indicating a pause is UART reception flow)
		*/
		if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(&huart4,\
				aBtRxBuffer, sizeof(aBtRxBuffer)))
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}

#endif
}/* end of startAuxReception */


/**
 * @brief send to auxulaty communication task
 * @brief This function transfers packet types via desiered channel
 * @brief if (pdFAIL==sendToAux( resp, MSGHDR_AUXCMDRESP_PACKET, portMAX_DELAY))
 * @param ustxStackDepth - tx task size
 * @param txprio - tx task priority
 * @param usrxStackDepth - rx task size
 * @param rxprio - rx task priority
 * @returns 0-if no error.  A non-zero value indicates an error.
 */
int startAuxTasks(uint16_t ustxStackDepth, UBaseType_t txprio, uint16_t usrxStackDepth, UBaseType_t rxprio)
{
#if (ENABLE_BLE_COM_AUX == 1U) || (ENABLE_DBG_COM_AUX == 1U)

	/* tx relevant tasks and resources */
	if(pdFAIL == xTaskCreate(\
			aux_TxTask, (const char *) "aux_TxTask",
			ustxStackDepth,
			NULL,txprio, NULL)){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}


	/* rx relevant tasks and resources */
	if ((auxInQ=xQueueCreate(24,sizeof(MSG_HDR)))!=NULL){
		vQueueAddToRegistry( auxInQ, (const char *)"auxInQ");
	}

	if(pdFAIL == xTaskCreate(\
			aux_RxTask, (const char *) "aux_RxTask",
			usrxStackDepth,
			NULL,rxprio, NULL)){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

#endif
	return pdPASS;
}/* end of startAuxTasks */


/**
 * @brief rx framer task function
 * @brief This function handles data balks from ISR
 * @param para - extra arguments
 * @returns none
 */
void aux_RxTask(void *para)
{
	MEMBUF_POOL rxBufPool;
	MSG_HDR aux_in_msg;
	size_t buff_idx	 =	0;
	size_t buff_len	 =	0;
	uint8_t *pd;
	TickType_t aux_rx_to=portMAX_DELAY;

	void *p;
	struct sRxFramerParams framerParams;


	/*
	** Create rx buffers pool
	*/
	p=_MEM_alloc((A4_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_COM_RX_BUFFERS);
	initMemBufPool(&rxBufPool,p,(A4_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_COM_RX_BUFFERS, A4_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR),N_COM_RX_BUFFERS);

	/*
	** Initialize channel framer
	*/
	memset(&framerParams,0,sizeof(framerParams));
	framerParams.pool= &rxBufPool;
	framerParams.pools=0;
	framerParams.extraPoolCfg=NULL;
	framerParams.frx=a4FrameRx;
	framerParams.rxFrameSync=a4RxFrameSync;
	framerParams.rxEofFrameSync=a4RxEofFrameSync;
	initRxFramer(&rxFramer, &framerParams);

	/* start debug/BT/data aux continuous reception */
	startAuxReception(NULL,DBG_AUX);

    char * str="\r\n[DEBUG AUX] started RX task \r\n";
    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);

	for (;;)
	{
		if (xQueueReceive(auxInQ, &aux_in_msg, aux_rx_to))
		{
			if (aux_in_msg.hdr.bit.type==MSG_TYPE_DEBUG_DATA_BLK)
			{
				if (aux_in_msg.buf)
				{
					if(rxFramer.frx)
					{
						pd=(uint8_t *)aux_in_msg.buf;

#ifdef DEBUG_DEBUG_ECHO
						PrintLog(pd, aux_in_msg.data);
#endif

						buff_len=aux_in_msg.data;

						for (buff_idx=0; buff_idx<buff_len; buff_idx++)
						{
							p=(*(rxFramer.frx))(&rxFramer, pd[buff_idx], NULL);
							if (p)
							{
								if (((PACKETBUF_HDR *)p)->format==A4_PROTO_FORMAT)
								{
#ifdef DEBUG_COMMAND
									aux_dbg_printLogBuffer(PACKETBUF_DATA(p), ((PACKETBUF_HDR *)p)->dlen);
#endif
									if (pdFAIL==sendToAuxCmdInterp(cmdq, p, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_DBRX,MSG_TYPE_CMD), portMAX_DELAY))
									{
										retMemBuf(p);
									}

								}
								else
								{
									retMemBuf(p);
								}
							}
						}

						aux_rx_to=AUX_RX_TIMEOUT;
					}
					else
					{
						p=NULL;
					}
				}
			}
			else //MSG_TYPE_BT_DATA_BLK
			if (aux_in_msg.hdr.bit.type==MSG_TYPE_BT_DATA_BLK)
			{
				if (aux_in_msg.buf)
				{
					/* check user call back */
					if(NULL != bt_cb)
					{

							pd=(uint8_t *)aux_in_msg.buf;

#if defined( DEBUG_BT_ECHO ) && (DEBUG_BT_ECHO == 1)
							aux_dbg_printLogBuffer(pd, aux_in_msg.data);
#endif
							/* user call back function */
							bt_cb(NULL,0,pd,aux_in_msg.data);

					}

					/* check vsp bt connection */
					if(true == is_bt_vsp_con())
					{
						if(rxFramer.frx)
						{
							pd=(uint8_t *)aux_in_msg.buf;
							buff_len=aux_in_msg.data;

							for (buff_idx=0; buff_idx<buff_len; buff_idx++)
							{
								p=(*(rxFramer.frx))(&rxFramer, pd[buff_idx], NULL);

								if (p && ((PACKETBUF_HDR *)p)->format==A4_PROTO_FORMAT)
								{
									if (pdFAIL==sendToAuxCmdInterp(cmdq, p, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_BTRX,MSG_TYPE_CMD), portMAX_DELAY))
										retMemBuf(p);
								}
								else
									retMemBuf(p);
							}

							aux_rx_to=AUX_RX_TIMEOUT;

						}
						else
							p=NULL;
					}
				}
			}
		}
		else
		{
			/*
			 * Host reception timeout
			 * Reset Aux packetizer if not in packet synchronization state
			 */
			handleRxFramerTimeout(&rxFramer);

			aux_rx_to=portMAX_DELAY;
		}

	}

	for (;;)
		vTaskDelay(1000000);
}/* End of uartRxTask */

#if (ENABLE_DBG_COM_AUX == 1U)
/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
void DEBUG_GPDMA_Init(void)
{
  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

   HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
   /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 15, 1);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 15, 1);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
void DEBUG_UARTX_Init(void)
{
#if defined(DEBUG_AUX_BAUD)

  huart5.Instance            = UART5;
  huart5.Init.BaudRate       = DEBUG_AUX_BAUD;
  huart5.Init.WordLength     = UART_WORDLENGTH_8B;
  huart5.Init.StopBits       = UART_STOPBITS_1;
  huart5.Init.Parity         = UART_PARITY_NONE;
  huart5.Init.Mode           = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }

  /* register rx complete callback */
  huart5.RxEventCallback = DEBUG_RxEventCallback;

#endif
}
#endif

#if (ENABLE_BLE_COM_AUX == 1U)

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
void BT_GPDMA_Init(void)
{
	/* Peripheral clock enable */
	__HAL_RCC_GPDMA1_CLK_ENABLE();

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	/* GPDMA1 interrupt Init */
	HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 15, 1);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
	HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 15, 1);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
void BT_UARTX_Init(uint32_t br)
{
#if defined(ENABLE_BLE_COM_AUX)

	uint32_t rv=HAL_UART_GetState(&huart4);
	if (rv != HAL_UART_STATE_RESET)
	{
		if (HAL_UART_DeInit(&huart4) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		BT_GPDMA_Init();
	}

	huart4.Instance            = UART4;
	huart4.Init.BaudRate       = br?br:DEF_BT_BAUD;
	huart4.Init.WordLength     = UART_WORDLENGTH_8B;
	huart4.Init.StopBits       = UART_STOPBITS_1;
	huart4.Init.Parity         = UART_PARITY_NONE;
	huart4.Init.Mode           = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling   = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
	if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* register rx complete callback */
	huart4.RxEventCallback = BT_RxEventCallback;

#endif
}
#endif


/**
  * @brief  User implementation of the Reception Event Callback
  *         (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void BT_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	  if(huart->Instance==UART4)//BT Aux
	  {
			MSG_HDR msg;
			signed portBASE_TYPE xHigherPriorityTaskWoken=0;

//			if (bt_cb != NULL)
//			{
//				bt_cb(NULL,0,aBtRxBuffer,Size);
//			}

			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_ISR1,MSG_TYPE_BT_DATA_BLK);
			msg.data=Size;
			msg.buf=aBtRxBuffer;

			if (!xQueueSendFromISR(auxInQ,&msg,&xHigherPriorityTaskWoken))
			{
				// Failure to send message with received buffer
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}


		  /* Initializes Rx sequence using Reception To Idle event API.
			 As DMA channel associated to UART Rx is configured as Circular,
			 reception is endless.
			 If reception has to be stopped, call to HAL_UART_AbortReceive() could be used.
			 Use of HAL_UARTEx_ReceiveToIdle_DMA service, will generate calls to
			 user defined HAL_UARTEx_RxEventCallback callback for each occurrence of
			 following events :
			 - DMA RX Half Transfer event (HT)
			 - DMA RX Transfer Complete event (TC)
			 - IDLE event on UART Rx line (indicating a pause is UART reception flow)
		   */

		  /* todo send message to rx framer task */

		  if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(&huart4, aBtRxBuffer, sizeof(aBtRxBuffer)))
		  {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		  }
	  }
}/* End of HAL_UARTEx_RxEventCallback */

#if (ENABLE_DBG_COM_AUX == 1U)
/**
  * @brief  User implementation of the Reception Event Callback
  *         (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void DEBUG_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	  if(huart->Instance==UART5)//DEBUG Aux
	  {
//    	  if(fota_mode == 1)
//		  {
//			   //Size = 65000;
//    		  uart_receive(&aDbgRxBuffer[0], 1029);
//    		  UartReadyRx = true;
//    		//  return;
//		  }
 //   	  else
			   if(Size != sizeof(aDbgRxBuffer)/2U)//filter half complete events
			   {
					MSG_HDR msg;
					signed portBASE_TYPE xHigherPriorityTaskWoken=0;

					msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_ISR1,MSG_TYPE_DEBUG_DATA_BLK);
					msg.data=Size;
					msg.buf=aDbgRxBuffer;

					if (!xQueueSendFromISR(auxInQ,&msg,&xHigherPriorityTaskWoken))
					{
						// Failure to send message with received buffer
						Error_Handler((uint8_t *)__FILE__, __LINE__);
					}

				  /* Initializes Rx sequence using Reception To Idle event API.
					 As DMA channel associated to UART Rx is configured as Circular,
					 reception is endless.
					 If reception has to be stopped, call to HAL_UART_AbortReceive() could be used.
					 Use of HAL_UARTEx_ReceiveToIdle_DMA service, will generate calls to
					 user defined HAL_UARTEx_RxEventCallback callback for each occurrence of
					 following events :
					 - DMA RX Half Transfer event (HT)
					 - DMA RX Transfer Complete event (TC)
					 - IDLE event on UART Rx line (indicating a pause is UART reception flow)
				  */

				  /* todo send message to rx framer task */

				  if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(&huart5, aDbgRxBuffer, sizeof(aDbgRxBuffer)))
				  {
					Error_Handler((uint8_t *)__FILE__, __LINE__);
				  }

				  uart_receive(&aDbgRxBuffer[0], 1029);
				  UartReadyRx = true;
			   }
		  }
//		  else
//		  {
//				  uart_receive(&aDbgRxBuffer[0], 1029);
//			      UartReadyRx = true;
//		  }
//	  }
}/* End of HAL_UARTEx_RxEventCallback */
#endif

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance==UART4)// =================================  AUX_BT
	{
		 /* Set transmission flag: transfer complete */
		Uart_ble_TxDone = SET;
	}
	else
	if(UartHandle->Instance==UART5)// =================================  AUX_DEBUG
	{
		 /* Set transmission flag: transfer complete */
		Uart_dbg_TxDone = SET;
		UartReadyTx = true;
	}

}/* end of HAL_UART_TxCpltCallback */


/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART5Only_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if(huart->Instance==UART5)
  {
      /* USER CODE BEGIN USART2_MspInit 0 */

      /* USER CODE END USART2_MspInit 0 */

      /** Initializes the peripherals clock
      */
      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART5;
	  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
	  	Error_Handler((uint8_t *)__FILE__, __LINE__);
	  }

        /* Peripheral clock enable */
        __HAL_RCC_UART5_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**USART2 GPIO Configuration
        PC12     ------> USART2_TX
        PD2     ------> USART2_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
  	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  	  GPIO_InitStruct.Pull = GPIO_NOPULL;
  	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USART2 interrupt Init */
  	    HAL_NVIC_SetPriority(UART5_IRQn, 15, 1);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
      /* USER CODE BEGIN USART2_MspInit 1 */

      /* USER CODE END USART2_MspInit 1 */
   }
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if(huart->Instance==UART4)// =================================  AUX_BT
  {

    /** Initializes the peripherals clock */
    PeriphClkInit.PeriphClockSelection               = RCC_PERIPHCLK_UART4;
    PeriphClkInit.Uart4ClockSelection                = RCC_UART4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    /* Peripheral clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    TX_BT_GPIO_CLK_ENABLE();
    RX_BT_GPIO_CLK_ENABLE();

    /** UART4 GPIO Configuration
    PC11     ------> UART4_RX
    PA0     ------> UART4_TX */

    GPIO_InitStruct.Pin                               = RX_BT_GPIO_PIN;
    GPIO_InitStruct.Mode                              = RX_BT_GPIO_MODE;
    GPIO_InitStruct.Pull                              = RX_BT_GPIO_PULL;
    GPIO_InitStruct.Speed                             = RX_BT_GPIO_SPEED;
    GPIO_InitStruct.Alternate                         = RX_BT_GPIO_ALTERNATE;
    HAL_GPIO_Init(RX_BT_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin                               = TX_BT_GPIO_PIN;
    GPIO_InitStruct.Mode                              = TX_BT_GPIO_MODE;
    GPIO_InitStruct.Pull                              = TX_BT_GPIO_PULL;
    GPIO_InitStruct.Speed                             = TX_BT_GPIO_SPEED;
    GPIO_InitStruct.Alternate                         = TX_BT_GPIO_ALTERNATE;
    HAL_GPIO_Init(TX_BT_GPIO_PORT, &GPIO_InitStruct);

    /* GPDMA1_REQUEST_UART4_RX Init */
    handle_GPDMA1_Channel2.Instance                   = GPDMA1_Channel2;
    handle_GPDMA1_Channel2.Init.Request               = GPDMA1_REQUEST_UART4_RX;
    handle_GPDMA1_Channel2.Init.BlkHWRequest          = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel2.Init.Direction             = DMA_PERIPH_TO_MEMORY;
    handle_GPDMA1_Channel2.Init.SrcInc                = DMA_SINC_FIXED;
    handle_GPDMA1_Channel2.Init.DestInc               = DMA_DINC_INCREMENTED;
    handle_GPDMA1_Channel2.Init.SrcDataWidth          = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel2.Init.DestDataWidth         = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel2.Init.Priority              = DMA_HIGH_PRIORITY;
    handle_GPDMA1_Channel2.Init.SrcBurstLength        = 1;
    handle_GPDMA1_Channel2.Init.DestBurstLength       = 1;
    handle_GPDMA1_Channel2.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT1;
    handle_GPDMA1_Channel2.Init.TransferEventMode     = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel2.Init.Mode                  = DMA_NORMAL;

    if (HAL_DMA_Init(&handle_GPDMA1_Channel2) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    __HAL_LINKDMA(huart, hdmarx, handle_GPDMA1_Channel2);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel2, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    /* GPDMA1_REQUEST_UART4_TX Init */
    handle_GPDMA1_Channel3.Instance             = GPDMA1_Channel3;
    handle_GPDMA1_Channel3.Init.Request         = GPDMA1_REQUEST_UART4_TX;
    handle_GPDMA1_Channel3.Init.BlkHWRequest    = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel3.Init.Direction       = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA1_Channel3.Init.SrcInc          = DMA_SINC_INCREMENTED;
    handle_GPDMA1_Channel3.Init.DestInc         = DMA_DINC_FIXED;
    handle_GPDMA1_Channel3.Init.SrcDataWidth    = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel3.Init.DestDataWidth   = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel3.Init.Priority        = DMA_HIGH_PRIORITY;
    handle_GPDMA1_Channel3.Init.SrcBurstLength  = 1;
    handle_GPDMA1_Channel3.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel3.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT1;
    handle_GPDMA1_Channel3.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel3.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel3) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    __HAL_LINKDMA(huart, hdmatx, handle_GPDMA1_Channel3);
    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel3, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 15, 1);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

  }
  else if(huart->Instance==UART5)// =================================  AUX_DEBUG
  {

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART5;
    PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    /* Peripheral clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    /**UART5 GPIO Configuration
    PD2     ------> UART5_RX
    PC12     ------> UART5_TX
    */
    TX_DEBUG_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin       = TX_DEBUG_GPIO_PIN;
    GPIO_InitStruct.Mode      = TX_DEBUG_GPIO_MODE;
    GPIO_InitStruct.Pull      = TX_DEBUG_GPIO_PULL;
    GPIO_InitStruct.Speed     = TX_DEBUG_GPIO_SPEED;
    GPIO_InitStruct.Alternate = TX_DEBUG_GPIO_ALTERNATE;
    HAL_GPIO_Init(TX_DEBUG_GPIO_PORT, &GPIO_InitStruct);

    RX_DEBUG_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin       = RX_DEBUG_GPIO_PIN;
    GPIO_InitStruct.Mode      = RX_DEBUG_GPIO_MODE;
    GPIO_InitStruct.Pull      = RX_DEBUG_GPIO_PULL;
    GPIO_InitStruct.Speed     = RX_DEBUG_GPIO_SPEED;
    GPIO_InitStruct.Alternate = RX_DEBUG_GPIO_ALTERNATE;
    HAL_GPIO_Init(RX_DEBUG_GPIO_PORT, &GPIO_InitStruct);

    /* GPDMA1_REQUEST_UART5_RX Init */
    handle_GPDMA1_Channel1.Instance                   = GPDMA1_Channel1;
    handle_GPDMA1_Channel1.Init.Request               = GPDMA1_REQUEST_UART5_RX;
    handle_GPDMA1_Channel1.Init.BlkHWRequest          = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel1.Init.Direction             = DMA_PERIPH_TO_MEMORY;
    handle_GPDMA1_Channel1.Init.SrcInc                = DMA_SINC_FIXED;
    handle_GPDMA1_Channel1.Init.DestInc               = DMA_DINC_INCREMENTED;
    handle_GPDMA1_Channel1.Init.SrcDataWidth          = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel1.Init.DestDataWidth         = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel1.Init.Priority              = DMA_HIGH_PRIORITY;
    handle_GPDMA1_Channel1.Init.SrcBurstLength        = 1;
    handle_GPDMA1_Channel1.Init.DestBurstLength       = 1;
    handle_GPDMA1_Channel1.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT1;
    handle_GPDMA1_Channel1.Init.TransferEventMode     = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel1.Init.Mode                  = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel1) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    __HAL_LINKDMA(huart, hdmarx, handle_GPDMA1_Channel1);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel1, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    /* GPDMA1_REQUEST_UART5_TX Init */
    handle_GPDMA1_Channel0.Instance             = GPDMA1_Channel0;
    handle_GPDMA1_Channel0.Init.Request         = GPDMA1_REQUEST_UART5_TX;
    handle_GPDMA1_Channel0.Init.BlkHWRequest    = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel0.Init.Direction       = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA1_Channel0.Init.SrcInc          = DMA_SINC_INCREMENTED;
    handle_GPDMA1_Channel0.Init.DestInc         = DMA_DINC_FIXED;
    handle_GPDMA1_Channel0.Init.SrcDataWidth    = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel0.Init.DestDataWidth   = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel0.Init.Priority        = DMA_HIGH_PRIORITY;
    handle_GPDMA1_Channel0.Init.SrcBurstLength  = 1;
    handle_GPDMA1_Channel0.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel0.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT1;
    handle_GPDMA1_Channel0.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel0.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel0) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    __HAL_LINKDMA(huart, hdmatx, handle_GPDMA1_Channel0);
    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 15, 1);
    HAL_NVIC_EnableIRQ(UART5_IRQn);

  }
}


/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==UART5)
  {
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**USART2 GPIO Configuration */
    HAL_GPIO_DeInit(TX_DEBUG_GPIO_PORT, TX_DEBUG_GPIO_PIN);
    HAL_GPIO_DeInit(RX_DEBUG_GPIO_PORT, RX_DEBUG_GPIO_PIN);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* USART2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  }
  else
  if(huart->Instance==UART4)
  {
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**USART2 GPIO Configuration */
    HAL_GPIO_DeInit(TX_BT_GPIO_PORT, TX_BT_GPIO_PIN);
    HAL_GPIO_DeInit(RX_BT_GPIO_PORT, RX_BT_GPIO_PIN);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* USART2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  }
}


/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
extern bool debug_serial_enable;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance==UART4)
	{
	 //Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	if(UartHandle->Instance==UART5)
	{
		debug_serial_enable=false;
	}
}


























