/**************************************************************************//**
* @file dispatcher_task.c
* @brief dispatcher task
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/


#include "periodic_dispatcher_task.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "_sem.h"
#include "_mem.h"
#include "main.h"
#include "auxcmd.h"
#include "RpcFifo.h"
#include "A5_proto/parcel.h"
#include "A5_proto/record.h"
#include "bt_manager_task.h"

#include "channel_manager_task.h"
#include "main.h"
#include "rtc.h"
#include "sys_errno.h"

#ifdef MODULE_DEBUG
#undef MODULE_DEBUG
#endif
#define MODULE_DEBUG 1

#define N_DISP_CMD_BUFFERS	2
#define MOD_NAME "[DISP]"

extern QueueHandle_t dispq;
extern int LSM6DSL_dev_proc_stream(void);
extern int TEMPX_dev_proc_stream(void);


unsigned char rec_col_buf[20000]={0};
MEMBUF_POOL txBufPool;
uint32_t total=0;

typedef enum dispatcher_states{
	suspended,
	operational
} dispatcher_states_t;

uint32_t prev_acc_rec_len_max3001=0;
uint32_t prev_acc_rec_len_acc=0;
uint32_t prev_acc_rec_len_resp=0;
uint32_t prev_acc_rec_len_bioz=0;


/**
  * @brief ivoke periodic dispatcher to make transaction
  */
void WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	/* TODO to fix temporary solution */
	LSM6DSL_dev_proc_stream();

	/* TODO to fix temporary solution */
	TEMPX_dev_proc_stream();

	sendToDispatcherFromISR(dispq, NULL,send_parcel,
			MAKE_MSG_HDRTYPE(0,MSG_SRC_ISR1,MSG_TYPE_CMD));

}


void set_periodic_dispatcher(uint32_t t)
{
	/* set up wake up timer every 1 second */
	if(RTC_set_wakeup_timer(0,WakeUpTimerEventCallback)){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
}

/**
 * @brief dispatcher task function
 * @brief This function handles periodic data transfer
 * @brief from different sources
 * @param para - extra arguments
 * @returns none
 */
void periodic_dispatcher_task(void *para)
{

	QueueHandle_t q=(QueueHandle_t)para;
	dispatcher_states_t sm_st = suspended;
	MSG_HDR msg;
	PACKETBUF_HDR *pf=NULL;
	unsigned char * pr = NULL;// record holder
	void *p = NULL;
	BaseType_t rval = pdFALSE;
	uint8_t en_tx=false;



	/* allocate memory area for the memory pool */
	p=_MEM_alloc((MAX_PARCEL_LEN+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_DISP_CMD_BUFFERS);
	if(p==NULL){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* Create memory pool of fixed size buffers  */
	if(initMemBufPool(&txBufPool,p,
			(MAX_PARCEL_LEN+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_DISP_CMD_BUFFERS,
			MAX_PARCEL_LEN+sizeof(PACKETBUF_HDR),N_DISP_CMD_BUFFERS)){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

//	/* set up wake up timer every 1 second */
//	if(RTC_set_wakeup_timer(0,WakeUpTimerEventCallback)){
//		Error_Handler((uint8_t *)__FILE__, __LINE__);
//	}

	for (;;)
	{
		switch(sm_st)
		{
			case(suspended):{

				/*
				 * in suspended state dispatcher task
				 * will be blocked waiting for commands
				 * from other applications
				 */

				if (xQueueReceive(q, &msg, portMAX_DELAY))
				{
					if (msg.hdr.bit.type==MSG_TYPE_CMD)
					{
						switch(msg.data)
						{
							case(start_exec):{
								sm_st = operational;
							}break;
							case(send_parcel):{

								int32_t trlen=0,rv=0;
								pr=rec_col_buf;

								#if 0
								char dbug[100]={0};
								#endif

								/* collect "R" record data in the buffer */
								rv=RECORD_get_rec_col(RESPONSE_TO_CMD_RECORD_TYPE,pr,MAX_DATA_LEN);
								if(rv<0){ Error_Handler((uint8_t *)__FILE__, __LINE__); }
								trlen+=rv;

								#if 0
								sprintf(dbug,"=== record R total size [%d]\r\n",rv);
								aux_dbg_printLogStr(dbug);
								#endif


								/* collect "B" record data in the buffer */
								rv=CHAN_ChanneData2record(pr+trlen,MAX_DATA_LEN-trlen);
								if(rv<0)
								{
									Error_Handler((uint8_t *)__FILE__, __LINE__);
								}
								trlen+=rv;


                                #if 0

								sprintf(dbug,"=== record B total size [%d]\r\n",rv);
								aux_dbg_printLogStr(dbug);
                                #endif

								en_tx=trlen!=0;
								if(en_tx)
								{
									/* make parcel frame prior dispatching */
									if(NULL != (pf = makeParcelFrame(&txBufPool,0,pr,trlen)))
									{
#if ENABLE_PROTO_SERIAL_LOG
										uint8_t * pda= (uint8_t*)&pf[1];
										size_t dlen=    pf->dlen;

										if (pdFAIL==aux_sendToAux(
												pda, //data buf
												dlen,//data len
												0,   //time out
												true,//m malloc
												DBG_AUX_PROTO))
											Error_Handler((uint8_t *)__FILE__, __LINE__);
#endif
										/* check vsp connection */
										if(is_bt_vsp_con())
										{
#if defined(ENABLE_THROUGHPUT_DEBUG_LOG) && (ENABLE_THROUGHPUT_DEBUG_LOG == 1)

											static uint32_t total_del_len=0;
											char dbug[100]={0};

											total_del_len+=pf->dlen;
											sprintf(dbug,"delivered [%ld]\r\n",total_del_len);
											aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
#endif
#if defined(ENABLE_CHAN_ACLEN_DEBUG_LOG) && (ENABLE_CHAN_ACLEN_DEBUG_LOG == 1)

											static uint32_t total_del_len=0;
											char dbug[100]={0};

											extern uint32_t g_parcel_id;

											//extern uint32_t acc_rec_len_tmp117;
											//extern uint32_t acc_rec_len_max3003;
											extern uint32_t acc_rec_len_max3001;
											//extern uint32_t acc_rec_len_gyro;
											extern uint32_t acc_rec_len_acc;
											//extern uint32_t acc_rec_len_pulse;

											//extern uint32_t acc_rec_len_scl;
											extern uint32_t acc_rec_len_resp;
											extern uint32_t acc_rec_len_bioz;
											//extern uint32_t acc_rec_len_pace;
											//extern uint32_t acc_rec_len_emg;
											//extern uint32_t acc_rec_len_audio;

											total_del_len+=pf->dlen;
											sprintf(dbug,"PID[%d] 0[%d][%d] 2[%d][%d] 5[%d][%d] 11[%d][%d] [%f][%f][%f][%f]\r\n",
													(int)g_parcel_id,
													(int)acc_rec_len_max3001,(int)(acc_rec_len_max3001-prev_acc_rec_len_max3001),
													(int)acc_rec_len_acc,(int)(acc_rec_len_acc-prev_acc_rec_len_acc),
													(int)acc_rec_len_resp,(int)(acc_rec_len_resp-prev_acc_rec_len_resp),
													(int)acc_rec_len_bioz,(int)(acc_rec_len_bioz-prev_acc_rec_len_bioz),
													(float)acc_rec_len_max3001/g_parcel_id,
													(float)acc_rec_len_acc/g_parcel_id,
													(float)acc_rec_len_resp/g_parcel_id,
													(float)acc_rec_len_bioz/g_parcel_id);
											aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);


											prev_acc_rec_len_max3001=acc_rec_len_max3001;
											prev_acc_rec_len_acc=acc_rec_len_acc;
											prev_acc_rec_len_resp=acc_rec_len_resp;
											prev_acc_rec_len_bioz=acc_rec_len_bioz;

#endif


											/* forward to tx handle task for transition  */
											if (pdFAIL==aux_sendPacketToAux( pf,0, MSGHDR_AUXCMD_BLE_BULK_PACKET, 0))
											{
												retMemBuf(pf);
												Error_Handler((uint8_t *)__FILE__, __LINE__);
											}
										}
										else
										{
											retMemBuf(pf);
										}
									}
								}

							}break;
							default:
								break;
						}
					}
				}

			}
			break;
			case(operational):{

				/*
				 * in operational state dispatcher task
				 * will be periodically transfer data to gateway
				 * and wait for user commands from other applications.
				 */

				rval = xQueueReceive(q, &msg, DISP_INTERVAL);
				if (rval == pdTRUE)//data arrived
				{
					if (msg.hdr.bit.type==MSG_TYPE_CMD)
					{
						switch(msg.data)
						{
							case(abort_exec):{
								sm_st = suspended;

							}break;
							default:
								break;
						}
					}
				}
				else
				if( rval == errQUEUE_EMPTY)//timeout occurred
				{


				}
				else
				{
					Error_Handler((uint8_t *)__FILE__, __LINE__);
				}
			}
			break;
			default:
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			break;
		}/* end of switch(sm_st) */
	}/* end of for (;;) */
}/* end of void dispatcherTask(void *para) */

/**
 * @brief startDispatcherTask
 * @brief This function initiates and starts dispatcher task
 * @param cmdQ - queue
 * @param cmdT - routine
 * @param pcName - verbose
 * @param usStackDepth - stack depth
 * @param prio - task priority
 * @returns 0 success
 */
int startDispatcherTask(QueueHandle_t *cmdQ, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio)
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

	/* create task routine */
	if (pdFAIL == xTaskCreate(periodic_dispatcher_task, pcName, usStackDepth, q, prio, &t))
	{
		vQueueDelete(q);
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
	return pdPASS;
}/* end of startDispatcherTask */


/**
 * @brief sendToDispatcher
 * @brief This function API used to send request to dispatcher
 * @param q - queue
 * @param packet - data
 * @param hdr - header
 * @param timeout - command timeout
 * @returns 0 success
 */
int sendToDispatcher(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr, uint32_t timeout)
{
	MSG_HDR msg;

	if (q==NULL)
		return pdFAIL;
	msg.hdr.all = hdr;
	msg.data=data;
	msg.buf=packet;
	return xQueueSend(q,&msg,timeout);
}/* end of sendToDispatcher */


/**
 * @brief sendToDispatcherFromISR
 * @brief This function API used to send data to dispatcher
 * @param q - queue
 * @param packet - data
 * @param hdr - header
 * @returns 0 success
 */
int sendToDispatcherFromISR(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr)
{
	signed portBASE_TYPE xHigherPriorityTaskWoken=0;
	MSG_HDR msg;

	if (q==NULL)
		return pdFAIL;
	msg.hdr.all = hdr;
	msg.data=data;
	msg.buf=packet;

	/* insert in ecg fifo */
	if(pdPASS != xQueueSendFromISR(q,&msg,&xHigherPriorityTaskWoken))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	// Now the buffer is empty we can switch context if necessary.
	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		portYIELD_FROM_ISR (pdTRUE);
	}

	return 0;
}/* end of sendToDispatcherFromISR */



#if 0
static int startLog(struct sOP_LOGSTART *pgh, SemaphoreHandle_t sync_sm)
{
	struct sSyncCmdBuffer sync_cmd;
	struct sSamplerStartLogCmd cmd_in;
	struct sSamplerStartLogCmdResp cmd_out;

	memset(&sync_cmd, 0, sizeof(sync_cmd));
	memset(&cmd_in, 0, sizeof(cmd_in));
	memset(&cmd_out, 0, sizeof(cmd_out));

	cmd_in.pad= 0;
	cmd_in.autostop_sel= pgh->autostop_sel;
	cmd_in.count_time= pgh->count_time;


	sync_cmd.userInData=&cmd_in;
	sync_cmd.userOutData=&cmd_out;
	sync_cmd.userCallback=_SEM_post;
	sync_cmd.userCallbackArg=sync_sm;

	if (pdPASS!=sendToSampler(samplerq, &sync_cmd, CMD_START_LOG, MSGHDR_AUXCMD, portMAX_DELAY))
		return E_TIMEOUT;
	if (pdPASS!=_SEM_pend(sync_sm, portMAX_DELAY))
		return E_TIMEOUT;
	return cmd_out.errCode;
}

#endif
