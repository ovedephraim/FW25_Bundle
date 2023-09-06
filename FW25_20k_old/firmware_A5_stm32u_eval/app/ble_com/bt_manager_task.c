/*
 * ChannelManager.c
 *
 *  Created on: 03-Nov-2015
 *      Author: Eyal
 */



#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "bt_manager_task.h"
#include "auxcmd.h"
#include "cmd.h"
#include "BL653.h"
#include "sys_conf.h"


typedef struct bt_stat
{
	bool is_vsp_connected;
}bt_stat_t;

extern QueueHandle_t btmngq;

#ifdef MODULE_DEBUG
#undef MODULE_DEBUG
#endif
#define MODULE_DEBUG 1

#define N_BT_CMD_BUFFERS	2
#define N_BT_BUFF_LEN       128

SemaphoreHandle_t sm=NULL;
MEMBUF_POOL btBufPool;

static int init_bt(SemaphoreHandle_t sync_sm);
static int sendBTATCmd(const char * at_cmd, char * at_resp, SemaphoreHandle_t sync_sm, uint32_t to);
static int aux_bt_RxCallback(void *arg, int status, void *addr, size_t size);

static char at_resp_buf[N_BT_BUFF_LEN]={0};
static bt_stat_t bt_status={0};


/**
 * @brief bt_manager_task
 * @brief task routine
 * @param para - queue
 */
void bt_manager_task(void *para)
{
	MSG_HDR bt_mng_msg;
	void *p = NULL;

	/* create synchronization semaphore */
	sm=(SemaphoreHandle_t)_SEM_create(1,NULL);

	/* allocate memory area for the memory pool */
	p=_MEM_alloc((N_BT_BUFF_LEN+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_BT_CMD_BUFFERS);
	if(p==NULL){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* Create memory pool of fixed size buffers  */
	if(initMemBufPool(&btBufPool,p,
			(N_BT_BUFF_LEN+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_BT_CMD_BUFFERS,
			N_BT_BUFF_LEN+sizeof(PACKETBUF_HDR),N_BT_CMD_BUFFERS))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* bt init */
	if(init_bt(sm))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	for (;;)
	{
		if (xQueueReceive(btmngq,&bt_mng_msg,portMAX_DELAY)==pdPASS) {/* TODO */}
	}
}/* end of bt_manager_task */

#if ENABLE_BLE_COM_AUX == 1U
static int32_t setBT_Br(uint32_t br)
{
	char br_str[30];
	char at_rsp_buf[N_BT_BUFF_LEN]={0};

	/* set baud rate to 921600 */
	memset(at_rsp_buf,0,sizeof(at_rsp_buf));
	sprintf(br_str,"ats 302=%d\r",(int)br);

	if(sendBTATCmd(br_str,at_rsp_buf,sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}

	/* validate baud rate */
	memset(at_rsp_buf,0,sizeof(at_rsp_buf));
	if(sendBTATCmd(getsup_UART_Baud_rate,at_rsp_buf,sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}

	return SYS_ERROR_NONE;
}
#endif


int32_t detect_baud_bt(void)
{
	char at_rsp_buf[N_BT_BUFF_LEN]={0};
	uint32_t abr[]={57600,921600,115200,230400,76800,460800};
	int i=0;

	memset(at_rsp_buf,0,sizeof(at_rsp_buf));
	for(i=0;i<(sizeof(abr)/sizeof(abr[0]));i++)
	{
		BT_UARTX_Init(abr[i]);
		/* restart aux reception mode */
		startAuxReception(aux_bt_RxCallback, BLE_AUX);

		vTaskDelay(50);

		/* check bt module sync with host  */
		sendBTATCmd("at\r",at_rsp_buf,sm,100);
		if(strstr(at_rsp_buf,"OK") || strstr(at_rsp_buf,"00"))
		{
			return abr[i];
		}

		if(strstr(at_rsp_buf,"ERROR"))
		{
			/* check bt module sync with host  */
			sendBTATCmd("at\r",at_rsp_buf,sm,100);
			if(strstr(at_rsp_buf,"OK"))
			{
				return abr[i];
			}
			/* check bt module sync with host  */
			sendBTATCmd("at\r",at_rsp_buf,sm,100);
			if(strstr(at_rsp_buf,"OK"))
			{
				return abr[i];
			}
			/* check bt module sync with host  */
			sendBTATCmd("at\r",at_rsp_buf,sm,100);
			if(strstr(at_rsp_buf,"OK"))
			{
				return abr[i];
			}
		}

		/* wait for dam done */
		vTaskDelay(50);
	}
	return 0;
}

/**
 * @brief init_bt
 * @brief Error whilst downloading data to device. If filesystem is full,
 * please restart device with 'atz' and clear the filesystem using 'at&f 1'.
 *   Please note this will erase ALL FILES on the device, configuration keys and all bonding keys.
 *   Received: 01	5002
 *
 * @param
 */
int init_bt(SemaphoreHandle_t sync_sm)
{

#if ENABLE_BLE_COM_AUX == 1U

	/******************************/
	/* UART Autobaud  section     */
	/******************************/

	char at_rsp_buf[N_BT_BUFF_LEN]={0};

	uint32_t bt_br = detect_baud_bt();
	if(bt_br !=OPR_BT_BAUD)
	{
		setBT_Br(OPR_BT_BAUD);

		/* save registers in nvm */
		memset(at_rsp_buf,0,sizeof(at_rsp_buf));
		if(sendBTATCmd(at_save_regmp,at_rsp_buf,sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;
		}

		/* execure warm bt reset */
		sendBTATCmd(at_warm_reset,NULL,sm,100);
		vTaskDelay(200);

		/* reconfigure to op */
		BT_UARTX_Init(OPR_BT_BAUD);

		/* restart aux reception mode */
		startAuxReception(aux_bt_RxCallback, BLE_AUX);
		vTaskDelay(100);
	}
//	else
//	{
//		sendBTATCmd("$autorun$\r",NULL,sync_sm,100);
//	}


	/******************************/
	/* BL653 Sytem Parameters     */
	/******************************/

	/* read bt module name */
	if(sendBTATCmd(at_get_btname,at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}

	/* read bt module firmware version */
	if(sendBTATCmd(at_get_fw_ver,at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}

	/* read bt module app version */
	if(sendBTATCmd(at_get_ap_ver,at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}


	if(sendBTATCmd("ats 202?\r",at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}
	if(sendBTATCmd("ats 203?\r",at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}
	if(sendBTATCmd("ats 204?\r",at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}
	if(sendBTATCmd("ats 205?\r",at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}
	if(sendBTATCmd("ats 300?\r",at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}
	if(sendBTATCmd("ats 301?\r",at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}
	if(sendBTATCmd("ats 219?\r",at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}
	if(sendBTATCmd("ats 307?\r",at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}
	if(sendBTATCmd("ats 118?\r",at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}
	if(sendBTATCmd("ats 109?\r",at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}

	if(sendBTATCmd(at_get_s_reg,at_rsp_buf,sync_sm,100)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}


	/* enable DLE - data length extension  */
	if(!strstr(at_rsp_buf, "\n25\r\nOK\r"))
	{
		/* read S register 100 value 67 for example
		 * Start-up Flags
			▪ Bit 0: Set to VSPConnectable - hence populates GATT table and starts adverts
			▪ Bit 1: Ignored if bit 0 is 1 otherwise start advertising with no timeout
			▪ Bit 2: Ignored if bit 0 is 1 otherwise start scanning with no timeout
			▪ Bit 3: Set for max bidirectional throughput of about 127kbps, otherwise half that.
			▪ Bit 4: Use Data Length Extension (#define DLE_ATTRIBUTE_SIZE) in smartBASIC application
			▪ Bits 5-6: Phy Rate 00 – 1MPHY 01 – Long Range – 125kbps 10 – RFU : will set 1 MPHY 11 – 2MPHY
		 * */

		/* read Startup register 100 value */
		if(sendBTATCmd("ats 100=25\r",at_rsp_buf,sync_sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;}
		/* Max Connections as Master/Central */
		if(sendBTATCmd("ats 126=1\r",at_rsp_buf,sync_sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;}
		if(sendBTATCmd("ats 125=1\r",at_rsp_buf,sync_sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;}
		/* VSP Incoming Max Cached Packets in VSP RX Buffer */
		if(sendBTATCmd("ats 118=8\r",at_rsp_buf,sync_sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;}
		/* read DLE Attribute Size*/
		if(sendBTATCmd("ats 219=244\r",at_rsp_buf,sync_sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;}


		/* UART Transmit Buffer Size */
		if(sendBTATCmd("ats 202=8192\r",at_rsp_buf,sync_sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;}
		if(sendBTATCmd("ats 203=8192\r",at_rsp_buf,sync_sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;}
		if(sendBTATCmd("ats 204=16384\r",at_rsp_buf,sync_sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;}
		if(sendBTATCmd("ats 205=16384\r",at_rsp_buf,sync_sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;}
		if(sendBTATCmd("ats 109=0\r",at_rsp_buf,sync_sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;}

		/* save registers in nvm */
		memset(at_rsp_buf,0,sizeof(at_rsp_buf));
		if(sendBTATCmd(at_save_regmp,at_rsp_buf,sm,100)){
			return SYS_ERROR_BT_INIT_FAILURE;
		}

		/* Execute warm bt reset */
		sendBTATCmd(at_warm_reset,NULL,sm,100);
	}

#else

#if 0 //set 1 to enable autorun

	/* restart aux reception mode */
	startAuxReception(aux_bt_RxCallback, BLE_AUX);
	vTaskDelay(100);

	sendBTATCmd("at\r",NULL,sync_sm,100);
	vTaskDelay(100);

	sendBTATCmd("$autorun$\r",NULL,sync_sm,100);

#endif

#endif

    char * str="\r\n[BT] initialization complete\r\n";
    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);

	return SYS_ERROR_NONE;

}/* init_bt */

/**
 * @brief used to get vsp connection status
 * @returns vsp connection status
 */
bool is_bt_vsp_con(void)
{
	return bt_status.is_vsp_connected;
}

/**
 * @brief callback function for aux data reception
 * @brief This function transfers data balks from ISR
 * @brief In async manner
 * @param arg - extra argument
 * @param status - transfer status
 * @param address - src address
 * @param size - data size in bytes
 * @returns 0-if no error.  A non-zero value indicates an error.
 */
int aux_bt_RxCallback(void *arg, int status, void *addr, size_t size)
{
    memcpy(at_resp_buf,addr,size);
    at_resp_buf[size]='\0';

    ENTER_ISR(); /**< A prolog of ISR code */

#if 0 //MODULE_DEBUG_RECEPT
    aux_dbg_printLogStr(at_resp_buf);
#endif

    /* todo register unregister cb */

    /* handle host commands */
	_SEM_post(sm);

	/* handle unsolicited codes */
	if(strstr(at_resp_buf,unsolicited_vsp_connected)){

	    char * str="\r\n[BT] VSP connected\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);

		bt_status.is_vsp_connected = 1;
	}

	if(strstr(at_resp_buf,unsolicited_vsp_disconnected)){
		char * str="\r\n[BT] VSP disconnected\r\n";
		aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
		bt_status.is_vsp_connected = 0;
	}

	EXIT_ISR();

	return 0;
}/* End of auxRxCallback */

/**
 * @brief sendBTATCmd
 * @brief This function API used to send AT commands to BLE module
 * @param at_cmd - at command string
 * @param at_resp - at command response
 * @returns 0 success
 */
#define AT_RESP_TIMEOUT 10000 //mS
int sendBTATCmd(const char * at_cmd, char * at_resp, SemaphoreHandle_t sync_sm,uint32_t to)
{
	int	rv=SYS_ERROR_NONE;

	if(at_cmd)//at_cmd_buf
	{
		/* Debug logout print */
		aux_sendToAux((char*)at_cmd,strlen(at_cmd),0,1,DBG_AUX);

		/* forward to bt tx handle task for transition  */
		if(pdFAIL==aux_sendToAux((char*)at_cmd,strlen(at_cmd),0,1,BLE_AUX))
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
			rv=SYS_ERROR_BT_INIT_FAILURE;
		}
		else
		{
			/* wait for response */
			if (pdPASS!=_SEM_pend(sync_sm, to))
			{
				rv=SYS_ERROR_TIMEOUT;
			}


			if(at_resp)
			{
				strncpy(at_resp,at_resp_buf,N_BT_BUFF_LEN);

				/* degug logout print */
				aux_sendToAux(at_resp,strlen(at_resp),0,1,DBG_AUX);
			}
		}
	}


	return rv;
}/* end of sendATCmd */

//====================================================================================================
//public functions
//====================================================================================================
/**
 * @brief startBTMngTask
 * @brief This function initiates and starts and initiates routine
 * @param cmdQ - queue
 * @param cmdT - routine
 * @param pcName - verbose
 * @param usStackDepth - stack depth
 * @param prio - task priority
 * @returns 0 success
 */
int startBTMngTask(QueueHandle_t *cmdQ, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio)
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

	/* add queue to registry */
	vQueueAddToRegistry( q, pcName);
	if (cmdQ)
		*cmdQ=q;
	if (cmdT)
		*cmdT=t;


	/* create task routine */
	if (pdFAIL == xTaskCreate(bt_manager_task, pcName, usStackDepth, q, prio, &t))
	{
		vQueueDelete(q);
		if (cmdQ)
			*cmdQ=NULL;
		if (cmdT)
			*cmdT=NULL;
		return pdFAIL;
	}

	return pdPASS;

}/* end of startDispatcherTask */

/**
 * @brief sendToBTManager
 * @brief This function API used to send request to dispatcher
 * @param q - queue
 * @param packet - data
 * @param hdr - header
 * @param timeout - command timeout
 * @returns 0 success
 */
int sendToBTManager(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr, uint32_t timeout)
{
	MSG_HDR msg;

	if (q==NULL)
		return pdFAIL;
	msg.hdr.all = hdr;
	msg.data=data;
	msg.buf=packet;
	return xQueueSend(q,&msg,timeout);
}/* end of sendToBTManager */

/**
 * @brief sendToBTManagerFromISR
 * @brief This function API used to send data to dispatcher
 * @param q - queue
 * @param packet - data
 * @param hdr - header
 * @returns 0 success
 */
int sendToBTManagerFromISR(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr)
{
	signed portBASE_TYPE xHigherPriorityTaskWoken=0;
	MSG_HDR msg;

	if (q==NULL)
		return pdFAIL;
	msg.hdr.all = hdr;
	msg.data=data;
	msg.buf=packet;

	/* insert in ecg fifo */
	return xQueueSendFromISR(q,&msg,&xHigherPriorityTaskWoken);

}/* end of sendToBTManagerFromISR */


/**
 * @brief BL653_hw_test
 * @brief hardware validation function
 * @return system error
 */
int BL653_hw_test(void * arg)
{
	char at_rsp_buf[N_BT_BUFF_LEN]={0};

	/* read bt module name */
	if(sendBTATCmd(at_get_btname,at_rsp_buf,sm,100)){
		return SYS_ERROR_PERIPH_FAILURE;
	}

	/* handle unsolicited codes */
	if(!strstr(at_rsp_buf,btname_resp)){
		return SYS_ERROR_BT_INIT_FAILURE;
	}

return SYS_ERROR_NONE;
}/* end of MAX30001_hw_test */

/**
 * @brief BL653_hw_reset
 * @brief hardware reset function
 * @return none
 */
void BL653_hw_reset(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	BT_AUTORUN_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin   =  BT_AUTORUN_GPIO_PIN;
	GPIO_InitStruct.Mode  =  BT_AUTORUN_GPIO_MODE;
	GPIO_InitStruct.Pull  =  BT_AUTORUN_GPIO_PULL;
	GPIO_InitStruct.Speed =  BT_AUTORUN_GPIO_SPEED;
	HAL_GPIO_Init(BT_AUTORUN_GPIO_PORT, &GPIO_InitStruct);

#if RUN_BLE_APP_ON_BOOT
	/*  issue atz once modem is up to enter interactive mode */
	HAL_GPIO_WritePin(BT_AUTORUN_GPIO_PORT, BT_AUTORUN_GPIO_PIN, GPIO_PIN_RESET);
#else
	/*  issue atz once modem is up to enter interactive mode */
	HAL_GPIO_WritePin(BT_AUTORUN_GPIO_PORT, BT_AUTORUN_GPIO_PIN, GPIO_PIN_SET);
#endif

	BT_RST_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin   =  BT_RST_GPIO_PIN;
	GPIO_InitStruct.Mode  =  BT_RST_GPIO_MODE;
	GPIO_InitStruct.Pull  =  GPIO_PULLUP;
	GPIO_InitStruct.Speed =  BT_RST_GPIO_SPEED;
	HAL_GPIO_Init(BT_RST_GPIO_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(BT_RST_GPIO_PORT, BT_RST_GPIO_PIN, GPIO_PIN_RESET);
	vTaskDelay(100);
	HAL_GPIO_WritePin(BT_RST_GPIO_PORT, BT_RST_GPIO_PIN, GPIO_PIN_SET);
	vTaskDelay(200);

}/* end of BL653_hw_reset */





