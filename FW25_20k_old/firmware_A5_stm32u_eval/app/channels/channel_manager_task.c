/*
 * ChannelManager.c
 *
 *      Author: Anton Kanaev
 */

#include "channel_manager_task.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "A5_proto/record.h"//RECORD_set_rec_col_map_8
#include "flash_param.h"
#include "endianutils.h"
#include "rtc.h"

/* channel handlers */
#include "ACC3X.h"
#include "ECG_H.h"
#include "main.h"
#include "periodic_dispatcher_task.h"
#include "PULSE.h"
#include "BIOZ.h"


#if 0
Int16	*CHAN_Record[CHAN_BUFF_NUM];	// This is a single edf record where all channels are writing to
UInt16	CHAN_RecordSize;				// Holds actual size (subject to amount of active channels)
#endif

#if 0
#define CMD_BUFF_SIZE	sizeof(CHAN_Proc_Stream_Vals_t)
#define N_CMD_BUFFERS	(40)
static MEMBUF_POOL CHAN_cmdPool;
#endif

#define CHAN_ChannelsNum sizeof(CHAN_Struct)/sizeof(CHAN_Struct[0])
#define MODULE_DEBUG 1
#ifdef MODULE_DEBUG
char pbuff[150];
#endif

extern const CHAN_Desc_t ChannelDesc[CHAN_MAX];
extern xQueueHandle chanmngq;

extern uint32_t g_parcel_id;

extern uint32_t acc_rec_len_tmp117;
extern uint32_t acc_rec_len_max3003;
extern uint32_t acc_rec_len_max3001;
extern uint32_t acc_rec_len_gyro;
extern uint32_t acc_rec_len_acc;
extern uint32_t acc_rec_len_pulse;
extern uint32_t acc_rec_len_bioz;

extern uint32_t acc_rec_len_scl;
extern uint32_t acc_rec_len_resp;
extern uint32_t acc_rec_len_pace;
extern uint32_t acc_rec_len_emg;
extern uint32_t acc_rec_len_audio;



static CHAN_Struct_t CHAN_Struct[CHAN_MAX];

static void channel_manager_task(void *para);
static void init_channels();
static uint32_t start_stream();
static uint32_t stop_stream();

static uint8_t _is_streaming=false;
uint16_t streaming_dev_num=0;


//====================================================================================================
//private functions
//====================================================================================================

/**
 * @brief cset and store default values
 * @brief channle parameters
 * @param pparams - param structure
 */
static void set_dafault_values_and_save(CHAN_Params_t *pparams)
{
	int i=0;
	for(i=0;i<CHAN_MAX;i++)
	{
		/* write default values */
		pparams[i].IsActive    =false;
		pparams[i].sampleRate  =ChannelDesc[i].SampleRate;
		pparams[i].samplebits  =ChannelDesc[i].SampleRes;
		if(SYS_ERROR_NONE != save_channel_params(&pparams[i],i))
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}
return;
}/* end of set_dafault_values_and_save */




/**
 * @brief CHAN_Init
 * @brief
 * @param
 */
void init_channels()
{
	CHAN_Params_t params[CHAN_ChannelsNum]={0};
	uint8_t	 ch=0;

#if 0

	params[0].IsActive    =1;
	params[0].sampleRate  =100;
	params[0].samplebits  =13;

	params[1].IsActive    =2;
	params[1].sampleRate  =300;
	params[1].samplebits  =14;

	if(SYS_ERROR_NONE != save_channel_params(params,0))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	if(SYS_ERROR_NONE != save_channel_params(params+1,1))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	params[0].IsActive    =0;
	params[0].sampleRate  =0;
	params[0].samplebits  =0;

	params[1].IsActive    =0;
	params[1].sampleRate  =0;
	params[1].samplebits  =0;


	/* restore configuration from flash mem */
	restore_channel_params(params);

	params[0].IsActive    =5;
	params[0].sampleRate  =500;
	params[0].samplebits  =15;

	params[1].IsActive    =6;
	params[1].sampleRate  =700;
	params[1].samplebits  =16;

	if(SYS_ERROR_NONE != save_channel_params(params,0))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	if(SYS_ERROR_NONE != save_channel_params(params+1,1))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	params[0].IsActive    =0;
	params[0].sampleRate  =0;
	params[0].samplebits  =0;

	params[1].IsActive    =0;
	params[1].sampleRate  =0;
	params[1].samplebits  =0;

#endif


	/* restore configuration from flash mem */
	if(restore_channel_params(params))
	{
		/* write default values */
		set_dafault_values_and_save(params);

#if 0
		params[CHAN_ECG_H].IsActive    =false;
		params[CHAN_ECG_H].sampleRate  =DEF_ECG_H_SAMPLE_RATE;
		params[CHAN_ECG_H].samplebits  =DEF_ECG_H_SAMPLE_RES;
		if(SYS_ERROR_NONE != save_channel_params(&params[CHAN_ECG_H],CHAN_ECG_H)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		params[CHAN_PULSE].IsActive    =false;
		params[CHAN_PULSE].sampleRate  =DEF_PULSE_SAMPLE_RATE;
		params[CHAN_PULSE].samplebits  =DEF_PULSE_SAMPLE_RES;
		if(SYS_ERROR_NONE != save_channel_params(&params[CHAN_PULSE],CHAN_PULSE)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		params[CHAN_ACC3X].IsActive    =false;
		params[CHAN_ACC3X].sampleRate  =DEF_ACC3X_SAMPLE_RATE;
		params[CHAN_ACC3X].samplebits  =DEF_ACC3X_SAMPLE_RES;
		if(SYS_ERROR_NONE != save_channel_params(&params[CHAN_ACC3X],CHAN_ACC3X)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
#endif

	}

	/* iterate over all the channels */
	for(ch=0; ch<CHAN_ChannelsNum; ch++)
	{
		/* TODO overwrite sample rate */
		params[ch].sampleRate  =ChannelDesc[ch].SampleRate;

		/* load constant data */
		CHAN_Struct[ch].Desc = &ChannelDesc[ch];

		/* load NVM parameters */
		CHAN_Struct[ch].Params = params[ch];

#ifdef MODULE_DEBUG//==================================================================

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"

        snprintf(pbuff,sizeof(pbuff),"\r\n[%s][%d]  is active [%d] Rate [%d] samplebits [%d] \r\n",
        		CHAN_Struct[ch].Desc->Name,ch,
				CHAN_Struct[ch].Params.IsActive,
				CHAN_Struct[ch].Params.sampleRate,
				CHAN_Struct[ch].Params.samplebits);
        aux_sendToAux(pbuff,strlen(pbuff),0,1,DBG_AUX);

#pragma GCC diagnostic pop

#endif //================================================================================

		/* call channel initialization function */
		if(CHAN_Struct[ch].Desc->InitFxn != NULL)
		{
			CHAN_Struct[ch].Desc->InitFxn(&CHAN_Struct[ch]);
		}

	}/* end of for(ch=0; ch<CHAN_ChannelsNum; ch++) */

	//aux_sendToAux((char*)Channels_resp,strlen(Channels_resp),0,1,DBG_AUX);
}

/**
 * @brief channel_manager_task
 * @brief task routine
 * @param para - queue
 */
void channel_manager_task(void *para)
{
	MSG_HDR chanmng_msg;

	if(CHAN_ChannelsNum != CHAN_MAX)
	{
		char * tmp="\r\n[CHAN] ERROR channels \r\n";
		aux_sendToAux(tmp,strlen(tmp),0,1,DBG_AUX);
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* channels init */
	init_channels();

	for (;;)
	{
		if (xQueueReceive(chanmngq,&chanmng_msg,portMAX_DELAY)==pdPASS)
		{
			if (chanmng_msg.hdr.bit.type==MSG_TYPE_CMD)
			{
				switch(chanmng_msg.data)
				{
					case(CHAN_CMD_START_ALL):{

						/* add sync and error codes */
						if(start_stream() >0 )
						{
							/* change streaming status */
							_is_streaming=true;
						}
					}break;
					case(CHAN_CMD_STOP_ALL):{

						/* add sync and error codes */
						if(stop_stream()>0)
						{
							/* change streaming status */
							_is_streaming=false;
						}

					}break;
					case(CHAN_CMD_PROC_STEAM):{

						CHAN_Proc_Stream_Vals_t val;
						val=*(CHAN_Proc_Stream_Vals_t*)chanmng_msg.buf;

						if(CHAN_ECG_H == val.id)//source is MAX30001
						{
							if(CHAN_Struct[CNAH_BIOZ].Params.IsActive)
							{
								/* call activation function in case channel is active */
								if(CHAN_Struct[CNAH_BIOZ].Desc->ProcessingFxn != NULL)
								{
									CHAN_Struct[CNAH_BIOZ].Desc->ProcessingFxn(&val);
								}

							}
							else if(CHAN_Struct[CHAN_ECG_H].Params.IsActive)
							{
								/* call activation function in case channel is active */
								if(CHAN_Struct[CHAN_ECG_H].Desc->ProcessingFxn != NULL)
								{
									CHAN_Struct[CHAN_ECG_H].Desc->ProcessingFxn(&val);
								}
							}
						}
						else
						if(CHAN_Struct[val.id].Params.IsActive)
						{
							/* call activation function in case channel is active */
							if(CHAN_Struct[val.id].Desc->ProcessingFxn != NULL)
							{
								/* todo check ret val */
								CHAN_Struct[val.id].Desc->ProcessingFxn(&val);
							}
						}
						else
						{
							char * tmp="\r\n[CHAN] warning! skipping processing \r\n";
							aux_sendToAux(tmp,strlen(tmp),0,1,DBG_AUX);
							Error_Handler((uint8_t *)__FILE__, __LINE__);
						}


#if 0
						/* informative log printout */
						sprintf(pbuff,"=== id[%d] time_stamp[%d] samples[%d] ===\r\n",
								val.id,val.ts,val.num);
						aux_dbg_printLogStr(pbuff);
#endif

						/* free allocated memory */
						retMemBuf(chanmng_msg.buf);

					}break;
					default:
						break;
				}
			}
		}
	}
}/* end of void channel_manager_task(void *para) */



/**
 * @brief start_stream
 * @brief
 * @param
 */
uint32_t start_stream()
{
	uint8_t	ch=0;
	uint32_t ach_num=0;
	time_t	t=0;

	g_parcel_id=0;

	acc_rec_len_tmp117=0;
	acc_rec_len_max3003=0;
	acc_rec_len_max3001=0;
	acc_rec_len_gyro=0;
	acc_rec_len_acc=0;

	acc_rec_len_pulse=0;

	acc_rec_len_scl=0;
	acc_rec_len_resp=0;
	acc_rec_len_pace=0;
	acc_rec_len_emg=0;
	acc_rec_len_audio=0;


	/* iterate over all the channels */
	for(ch=0; ch<CHAN_ChannelsNum; ch++)
	{
		/* load constant data */
		CHAN_Struct[ch].Desc = &ChannelDesc[ch];

		if(CHAN_Struct[ch].Params.IsActive)
		{
			/* call activation function in case channel is active */
			if(CHAN_Struct[ch].Desc->ActivationFxn != NULL)
			{
				/* todo check ret val */
				CHAN_Struct[ch].Desc->ActivationFxn(&CHAN_Struct[ch], true);
				ach_num++;
				streaming_dev_num++;
			}
		}
	}/* end of for(ch=0; ch<CHAN_ChannelsNum; ch++) */

	if(ach_num)
	{
		char log_buf[100]={0};
		sprintf(log_buf,"\r\n[CHAN] [%d] channels activated\r\n",(int)ach_num);
		aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
	}
	else
	{
		streaming_dev_num=0;
		char * log_buf="\r\n[CHAN] no active channels!\r\n";
		aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
	}

	/*get time */
	RTC_get(&t);
	/* store in backup register */
	RTC_save_time_stamp(t);

	set_periodic_dispatcher(0);

return ach_num;
}/* end of start_stream */

/**
 * @brief stop_stream
 * @brief
 * @param
 */
uint32_t stop_stream()
{
	uint8_t	ch=0;
	uint32_t ach_num=0;

	g_parcel_id=0;

	acc_rec_len_tmp117=0;
	acc_rec_len_max3003=0;
	acc_rec_len_max3001=0;
	acc_rec_len_gyro=0;
	acc_rec_len_acc=0;

	acc_rec_len_pulse=0;

	acc_rec_len_scl=0;
	acc_rec_len_resp=0;
	acc_rec_len_pace=0;
	acc_rec_len_emg=0;
	acc_rec_len_audio=0;

	/* iterate over all the channels */
	for(ch=0; ch<CHAN_ChannelsNum; ch++)
	{
		/* load constant data */
		CHAN_Struct[ch].Desc = &ChannelDesc[ch];

		if(CHAN_Struct[ch].Params.IsActive)
		{
			/* call activation function in case channel is active */
			if(CHAN_Struct[ch].Desc->ActivationFxn != NULL)
			{
				/* todo check ret val */
				CHAN_Struct[ch].Desc->ActivationFxn(&CHAN_Struct[ch],false);
				ach_num++;
				streaming_dev_num--;
			}
		}
	}/* end of for(ch=0; ch<CHAN_ChannelsNum; ch++) */

	if(ach_num)
	{
		char log_buf[100]={0};
		sprintf(log_buf,"\r\n[CHAN] [%d] channels deactivated\r\n",(int)ach_num);
		aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
	}
	else
	{
		streaming_dev_num=0;
		char * log_buf="\r\n[CHAN] no active channels!\r\n";
		aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
	}

	set_periodic_dispatcher(60);//every 61 seconds

return ach_num;
}/* end of stop_stream */

//====================================================================================================
//public functions
//====================================================================================================

/**
 * @brief CHAN_GetChannelfifo
 * @param ch - channel id
 * @returns fifo pointer
 */
fifo_t* CHAN_GetChannelfifo(CHAN_Channels_t ch)
{
	if(ch > CHAN_MAX)
	{
		return (fifo_t*)NULL;
	}
	return CHAN_Struct[ch].Vals.pfifoStreamOut;
}/* End of CHAN_GetChannelfifo */

/**
 * @brief CHAN_startManagerTask
 * @brief This function initiates and starts and initiates routine
 * @param cmdQ - queue
 * @param cmdT - routine
 * @param pcName - verbose
 * @param usStackDepth - stack depth
 * @param prio - task priority
 * @returns 0 success
 */
int startChanMngTask(QueueHandle_t *cmdQ, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio)
{
	QueueHandle_t q;
	TaskHandle_t t;

#if(0)
	/* allocate memory area for the memory pool */
	p=_MEM_alloc((CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS);
	if(p==NULL){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* Create memory pool of fixed size buffers  */
	if(initMemBufPool(&CHAN_cmdPool,p,
			(CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS,
			CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR),N_CMD_BUFFERS)){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
#endif

	/* create queue */
	if ((q = xQueueCreate(48,sizeof(MSG_HDR))) == NULL)
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
	if (pdFAIL == xTaskCreate(channel_manager_task, pcName, usStackDepth, q, prio, &t))
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
 * @brief sendToChannelManager
 * @brief This function API used to send request to dispatcher
 * @param q - queue
 * @param packet - data
 * @param hdr - header
 * @param timeout - command timeout
 * @returns 0 success
 */
int sendToChannelManager(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr, uint32_t timeout)
{
	MSG_HDR msg;

	if (q==NULL)
		return pdFAIL;
	msg.hdr.all = hdr;
	msg.data=data;
	msg.buf=packet;
	return xQueueSend(q,&msg,timeout);
}/* end of sendToChannelManager */

/**
 * @brief sendToChannelManagerFromISR
 * @brief This function API used to send data to dispatcher
 * @param q - queue
 * @param packet - data
 * @param hdr - header
 * @returns 0 success
 */
int sendToChannelManagerFromISR(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr)
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
}/* end of sendToChannelManagerFromISR */


/**
 * @brief CHAN_ChanneData2record
 * @param rec_out - output buffer
 * @param len - output buffer length
 * @returns 0 on success
 */
int CHAN_ChanneData2record(unsigned char * rec_out, uint32_t len)
{
	unsigned char *pvd=NULL;
	CHAN_Params_t *pchparam=NULL;
	sStreamRecHeader_t *phd=NULL;
	fifo_t * pfifo=NULL;
	uint32_t fifo_element=0,ch=0,tlen=0,rdlen=0, smplcnt=0,num_of_recs=0;
	uint32_t col_len=0,rlen=0;
	uint8_t val=0, step=0, quit=0;


	/* iterate over all the channels */
	for(ch=0; ch<CHAN_ChannelsNum; ch++)
	{
		uint32_t sample_cnt_deb=0;
		uint32_t num_of_recs_deb=0;

		/* call activation function in case channel is active */
		if(CHAN_Struct[ch].Vals.pfifoStreamOut != NULL)
		{
			/* skipping inactive channels check */
			if(!CHAN_Struct[ch].Params.IsActive)
			{
				continue;
			}

			pchparam=&CHAN_Struct[ch].Params;
			col_len=0;

	        /* extract fifo handler for available channel */
			pfifo = CHAN_Struct[ch].Vals.pfifoStreamOut;

			/* check if fifo contains data */
			if(pfifo && fifo_empty(pfifo)){
				/* skip channel */
				continue;
			}

			/* construct "B" record */
			while(1)
			{
				if(0==fifo_get32(pfifo, &fifo_element))
				{
					if(tlen>=len)
					{
						return -1;
					}

					switch(step)
					{
						case(0): //record type, length, time stamp

							phd=(sStreamRecHeader_t *)&rec_out[tlen];

						    //add record type
							phd->sor=STREAMING_RECORD_TYPE;
						    tlen+=sizeof(phd->sor);

						    //add time stamp
							#if defined(USE_BIG_ENDIAN)
							phd->ts=longLE2BE(fifo_element);
							#else
							phd->ts=fifo_element;
							#endif

							tlen+=sizeof(phd->ts);

						    //add channel id
							phd->chan_id=ch;
							tlen+=sizeof(phd->chan_id);

							step=1;
							break;

						case(1): //length only

		                    /*add vector length */
		                    rdlen=fifo_element;
						    tlen+=sizeof(phd->len);

						    pvd=(unsigned char *)&phd[1];
						    col_len=rlen=0;
							step=2;
							break;

						case(2):{ //samples

							rlen++;//samples per record counter
						    smplcnt++;//all samples counter
						    sample_cnt_deb++;

						    if(pchparam->samplebits>0 && pchparam->samplebits<=8)
						    {
								val=0xFF & fifo_element;
								pvd[col_len++]=val;
								tlen++;
						    }else
						    if(pchparam->samplebits>8 && pchparam->samplebits<=16)
						    {
#if defined(USE_BIG_ENDIAN)
								val=0xFF & (fifo_element >> 8);
								pvd[col_len++]=val;
								tlen++;

								val=0xFF & fifo_element;
								pvd[col_len++]=val;
								tlen++;
#else
								val=0xFF & fifo_element;
								pvd[col_len++]=val;
								tlen++;

								val=0xFF & (fifo_element >> 8);
								pvd[col_len++]=val;
								tlen++;
#endif //defined(USE_BIG_ENDIAN)
						    }else
							if(pchparam->samplebits>16 && pchparam->samplebits<=24)
							{
#if defined(USE_BIG_ENDIAN)
								val=0xFF & (fifo_element >> 16);
							    pvd[col_len++]=val;
							    tlen++;

								val=0xFF & (fifo_element >> 8);
								pvd[col_len++]=val;
								tlen++;

								val=0xFF & fifo_element;
								pvd[col_len++]=val;
								tlen++;
#else
								val=0xFF & fifo_element;
								pvd[col_len++]=val;
								tlen++;

								val=0xFF & (fifo_element >> 8);
								pvd[col_len++]=val;
								tlen++;

								val=0xFF & (fifo_element >> 16);
							    pvd[col_len++]=val;
							    tlen++;
#endif //defined(USE_BIG_ENDIAN)
							}else
						    if(pchparam->samplebits>24 && pchparam->samplebits<=32)
						    {
#if defined(USE_BIG_ENDIAN)
								val=0xFF & (fifo_element >> 24);
							    pvd[col_len++]=val;
							    tlen++;

								val=0xFF & (fifo_element >> 16);
							    pvd[col_len++]=val;
							    tlen++;

								val=0xFF & (fifo_element >> 8);
								pvd[col_len++]=val;
								tlen++;

								val=0xFF & fifo_element;
								pvd[col_len++]=val;
								tlen++;
#else
								val=0xFF & fifo_element;
								pvd[col_len++]=val;
								tlen++;

								val=0xFF & (fifo_element >> 8);
								pvd[col_len++]=val;
								tlen++;

								val=0xFF & (fifo_element >> 16);
							    pvd[col_len++]=val;
							    tlen++;

								val=0xFF & (fifo_element >> 24);
							    pvd[col_len++]=val;
							    tlen++;
#endif //defined(USE_BIG_ENDIAN)
						    }
							else{ Error_Handler((uint8_t *)__FILE__, __LINE__);}

							if(rlen >= rdlen)
							{
								num_of_recs++;
								/* add end-of-record */
								pvd[col_len]=EOR;
								tlen++;

								/*add real length */
								#if defined(USE_BIG_ENDIAN)
								phd->len=shortLE2BE(col_len);
								#else
								phd->len=col_len;
								#endif

								num_of_recs_deb++;
								/* fifo not empty- continue */
								if(fifo_empty(pfifo))
								{
#if (defined ENABLE_RECORD_DEBUG_LOG) && (ENABLE_RECORD_DEBUG_LOG == 1)

									sprintf(pbuff,"\r\n ch[%d] samples all[%d] recs all[%d] ts[%ud] \r\n",(int)ch,
											(int)sample_cnt_deb,(int)num_of_recs_deb,(unsigned int)phd->ts);
									aux_sendToAux(pbuff,strlen(pbuff),0,1,DBG_AUX);

#endif
									quit=1;
								}
								step=0;//set init state
							}
						}
						break;
					}/* switch(step) */

					/* skip to another channel */
					if(quit)
					{
						quit=0;
						break;
					}
				}
				else
				{
					return -1;
				}
			}/* while(1) */
		}
	}/* for(ch=0; ch<CHAN_ChannelsNum; ch++) */

	return tlen;
}/* End of CHAN_ChanneData2record */


int32_t CHAN_SetChannelActivation(CHAN_Channels_t ch, uint32_t active)
{
	int32_t ret = -1;

	if(_is_streaming==true)
	{
		char * log_buf="\r\n[CHAN] is streaming now. STOP streaming \r\n";
		aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
		return ret;
	}

	if((ch>=0) && (ch < CHAN_ChannelsNum))
    {
		if(active==0 || active==1)
		{
			CHAN_Struct[ch].Params.IsActive = active;
			/* call channel initialization function */
			if(CHAN_Struct[ch].Desc->InitFxn != NULL)
			{
				CHAN_Struct[ch].Desc->InitFxn(&CHAN_Struct[ch]);
			}
			ret = 0;
		}
	}

	/* save channel parameters in flash for all channels */
	save_channel_params(&(CHAN_Struct[ch].Params), ch);

return ret;
}/* end of CHAN_SetChannelActivation */

int32_t CHAN_GetChannelActivation(CHAN_Channels_t ch, uint32_t * active)
{
	int32_t ret = -1;

	if((ch>=0) && (ch < CHAN_ChannelsNum))
	{
		(*active) = CHAN_Struct[ch].Params.IsActive;
		ret = 0;
	}

return ret;
}/* end of CHAN_GetChannelActivation */

int32_t CHAN_GetChannelRate(CHAN_Channels_t ch, int32_t * rate)
{
	int32_t ret = -1;

	if((ch>=0) && (ch < CHAN_ChannelsNum))
	{
		(*rate) = CHAN_Struct[ch].Params.sampleRate;
		ret = 0;
	}

return ret;
}/* end of CHAN_GetChannelActivation */

int32_t CHAN_GetallChannelNumber(uint32_t * num)
{
	int32_t ret = 0;
	*num = CHAN_ChannelsNum;
return ret;
}/* end of CHAN_GetallChannelNumber */


uint32_t CHAN_GetActiveChannelsNumber(void)
{
	uint32_t ach_num=0;
	uint8_t	ch=0;


	/* iterate over all the channels */
	for(ch=0; ch<CHAN_ChannelsNum; ch++)
	{
		/* load constant data */
		CHAN_Struct[ch].Desc = &ChannelDesc[ch];

		if(CHAN_Struct[ch].Params.IsActive)
		{
			/* call activation function in case channel is active */
			if(CHAN_Struct[ch].Desc->ActivationFxn != NULL)
			{
				ach_num++;
			}
		}
	}/* end of for(ch=0; ch<CHAN_ChannelsNum; ch++) */

return ach_num;
}/* end of CHAN_GetActiveChannelsNumber */

int32_t CHAN_GetallChannelConfig(char * buff)
{
	int32_t ret = 0;
	sprintf(buff,Channels_resp,
			CHAN_Struct[CHAN_ECG_H].Params.IsActive?    "true":"false",
			CHAN_Struct[CHAN_PULSE].Params.IsActive?    "true":"false",
			CHAN_Struct[CHAN_ACC3X].Params.IsActive?    "true":"false",
			CHAN_Struct[CHAN_AN_EMG].Params.IsActive?   "true":"false",
			CHAN_Struct[CHAN_AN_SCL].Params.IsActive?   "true":"false",
			CHAN_Struct[CHAN_AN_RESP].Params.IsActive?  "true":"false",
			CHAN_Struct[CHAN_AN_AUDIO].Params.IsActive? "true":"false",
			CHAN_Struct[CHAN_AN_PACE].Params.IsActive?  "true":"false",
			CHAN_Struct[CHAN_ECG_V].Params.IsActive?    "true":"false");
return ret;
}/* end of CHAN_GetallChannelNumber */

int32_t CHAN_GetallChannelConfig_C(char * buff)
{
	int32_t ret = 0;
	sprintf(buff,C_resp,
			CHAN_Struct[CHAN_ECG_H].Params.IsActive?"true":"false",
			CHAN_Struct[CHAN_PULSE].Params.IsActive?"true":"false",
			CHAN_Struct[CHAN_ACC3X].Params.IsActive?"true":"false",
			CHAN_Struct[CHAN_AN_EMG].Params.IsActive?"true":"false",
			CHAN_Struct[CHAN_AN_SCL].Params.IsActive?"true":"false",
			CHAN_Struct[CHAN_AN_RESP].Params.IsActive?"true":"false",
			CHAN_Struct[CHAN_AN_AUDIO].Params.IsActive?"true":"false",
			CHAN_Struct[CHAN_AN_PACE].Params.IsActive?"true":"false",
			CHAN_Struct[CHAN_ECG_V].Params.IsActive?"true":"false");
return ret;
}/* end of CHAN_GetallChannelNumber */


#if 0

//								collector_buff[collector_buff_len]   = 0xFF & (fifo_element >> 16);
//								collector_buff[collector_buff_len+1] = 0xFF & (fifo_element >> 8);
//								collector_buff[collector_buff_len+2] = 0xFF & fifo_element;

int CHAN_GetChannelNumber()
{
	return CHAN_ChannelsNum;
}


int CHAN_GetActiveChannelNumber()
{
	int cnt = 0;
	int ch;

	for (ch=0; ch < CHAN_ChannelsNum; ch++){
		if(CHAN_Struct[ch].Params.IsActive)	{
			cnt++;
		}
	}
	return cnt;
}


int CHAN_GetChannelSampleRate(CHAN_Channels_t ch)
{
	return CHAN_Struct[ch].Desc->SampleRate;
}


void CHAN_Reset()
{
	UInt8 ch, i;

	for(ch=0; ch<CHAN_ChannelsNum; ch++)	{
		for(i=0 ; i<CHAN_BUFF_NUM; i++)	{
			if(CHAN_Struct[ch].Desc->IsSync)	{
				CHAN_Struct[ch].Vals.SamplesNum[i]=0;
			}
			else	{
				CHAN_Struct[ch].Vals.SamplesNum[i]=1;
			}
		}
		if(CHAN_Struct[ch].Desc->ResetFxn)	{
			CHAN_Struct[ch].Desc->ResetFxn();
		}
	}
	CHAN_WriteBuffIdx = 0;
	TotalSamplesReceived = ADS131_GetTickCounter(); 	// if there is a gap, close it.
	SessionSamplesReceived = 0;
}





void CHAN_PrepareChannels()
{
	Uint8 ch;
	UInt8 buff;

	CHAN_RecordSize = 0;

	for(ch=0; ch<CHAN_ChannelsNum; ch++){
		if(CHAN_Struct[ch].Params.IsActive)	{
			for(buff=0; buff<CHAN_BUFF_NUM; buff++){
				CHAN_Struct[ch].Vals.Buff[buff] = CHAN_Record[buff]+CHAN_RecordSize;// relocate the buffer pointer
			}
			CHAN_RecordSize += CHAN_Struct[ch].Vals.BuffSize;
		}
	}
}


void taskCHAN_Manager()
{
	UInt8 ch;
	ADS131_Sample_t Yn;
	UInt8	tag=0;
	UInt32 ticks;
	UInt8	prevWriteBuffIdx;

	System_printf("Starting taskCHAN_Manager()\n");
	System_flush();

	#ifdef SYNTH_SIGNAL	// 16 bit
	static uint16_t t = 0;
	static ADS131_Sample_t step = 1;
	static ADS131_Sample_t val = 0;
	#endif
	while(1)	{
		Semaphore_pend(semADS131_NewSampleAvailable, BIOS_WAIT_FOREVER);
		tag++;
		#ifdef SYNTH_SIGNAL
		t++;
		val += step;	// Triangle
		if (t == 0x0fff) {	// consider hwi decimation
			t = 0;
			step *= -1;
		}
		#endif
		ch = 0;	// ????
		for(ch=0; ch<CHAN_ChannelsNum; ch++){
			if(CHAN_Struct[ch].Params.IsActive)	{
				if(CHAN_Struct[ch].Desc->ProcessingFxn(&Yn))	{	// "something" returned from the handler
					#ifdef SYNTH_SIGNAL
					Yn = val;	//(((Int16)ch)<<8) + tag;
					#endif
					if(CHAN_Struct[ch].Desc->IsSync)	{
						if(CHAN_Struct[ch].Vals.SamplesNum[CHAN_WriteBuffIdx] < CHAN_Struct[ch].Vals.BuffSize)	{
							CHAN_Struct[ch].Vals.Buff[CHAN_WriteBuffIdx][CHAN_Struct[ch].Vals.SamplesNum[CHAN_WriteBuffIdx]]=Yn;
							CHAN_Struct[ch].Vals.SamplesNum[CHAN_WriteBuffIdx]++;
						}
						else	{
							CHAN_Struct[ch].Vals.Buff[CHAN_WriteBuffIdx][CHAN_Struct[ch].Vals.BuffSize-1]=Yn;
						}
					}	// IsSync
					else	{
						CHAN_Struct[ch].Vals.Buff[CHAN_WriteBuffIdx][0] = Yn;	// In Async channels there is only 1 sample in each timeframe
						CHAN_Struct[ch].Vals.SamplesNum[CHAN_WriteBuffIdx]=1;	// redundant after initialization. never change.
					} // ASync
				} // Something returned from handler
			} // IsActive
		} // for ch...
		ADS131_AdvanceToNextSample();

		TotalSamplesReceived++;
		SessionSamplesReceived++;

		if(SessionSamplesReceived % CHANNEL_TIMEFRAME_SAMPLES == 0)	{	// Time to xmit data..
			// advance write buffer
			prevWriteBuffIdx = CHAN_WriteBuffIdx;
			CHAN_WriteBuffIdx++;
			CHAN_WriteBuffIdx %= CHAN_BUFF_NUM;

			for(ch=0; ch< CHAN_ChannelsNum; ch++)	{
				if(CHAN_Struct[ch].Params.IsActive)	{
				if(CHAN_Struct[ch].Desc->IsSync)	{
					CHAN_Struct[ch].Vals.SamplesNum[CHAN_WriteBuffIdx]=0;
				}
				else	{	// Async
					CHAN_Struct[ch].Vals.Buff[CHAN_WriteBuffIdx][0]=CHAN_Struct[ch].Vals.Buff[prevWriteBuffIdx][0]; 	// In Async channels copy value from previous buffer
					CHAN_Struct[ch].Vals.SamplesNum[CHAN_WriteBuffIdx]=1;
				}
			}
			}
			// let the xmit begin
			//////MAX14662_SwitchToggle(MAX14662_LED_GREEN);
			if(!Semaphore_pend(mutexCHAN_DataXmitPending, 0))	{	// To avoid contention when stopping recording "Z !"
				System_printf("mutexCHAN_DataXmitPending not available @%ld samples\n", TotalSamplesReceived);
				System_flush();
			}
			if(!Mailbox_post(mbxCHAN_DataXmit, &prevWriteBuffIdx, BIOS_NO_WAIT))	{
				System_printf("mbxCHAN_DataXmit not available @%ld samples\n", TotalSamplesReceived);
				System_flush();
			}
			// Store to EDF File
			#if (SD_STORE_EDF == 1)
			if(!Semaphore_pend(mutexCHAN_DataStorePending, 0))	{	// To avoid contention when stopping recording "Z !"
				System_printf("mutexCHAN_DataStorePending not available @%ld samples\n", TotalSamplesReceived);
				System_flush();
			}
			if(!Mailbox_post(mbxCHAN_DataStore, &prevWriteBuffIdx, BIOS_NO_WAIT))	{
				System_printf("mbxCHAN_DataStore not available @%ld samples\n", TotalSamplesReceived);
				System_flush();
			}
			#endif
		}
		else	{	// Full record received
		}
		ticks = ADS131_GetTickCounter();
		if( ticks - TotalSamplesReceived > 10){
			System_abort("ADS131 ticks > Counter\n");
		}
		#ifdef ADS131_HALT_INTR_DURING_PROCESSING
		GPIO_enableInt(SR_GPIO_ADS131_nDRDY);	// REMOVE !
		#endif
	}
}

void taskCHAN_TransmitChannels()
{
	UInt8   buffIdx;
///////////////////////////////////////
	System_printf("Starting taskCHAN_TransmitChannels()\n");
	System_flush();

	while(1)	{
		if(onBefore)
		{
	    if(BLE_GetStatus() == BLE_STATUS_NOT_CONNECTED)
		{
	    	if(ADSStatus()==0){
	    					 ADS131_Stop();
	    					 			Task_sleep(CHANNEL_TIMEFRAME_MS); //let time for ADS131 to stop 100MS
	    					 			CHAN_Reset();
	    					 //			ResumeHeartBeatTask();
	    					 			#if (SD_STORE_EDF==1)
	    					 			int i = 2;
	    					 			while ( (Mailbox_getNumPendingMsgs(mbxCHAN_DataStore) != 0) && i--)	{
	    					 				Task_sleep(CHANNEL_TIMEFRAME_MS);
	    					 			}
	    					 			if(i == 0){
	    					 				System_printf("mbxCHAN_DataStore did not handles in %d[ms]\n",CHANNEL_TIMEFRAME_MS*2);
	    					 				System_flush();
	    					 			}
	    					 			EDFFile_CloseCurrentFile();
	    					 			#endif
	    					 			UInt8 i= 2;
	    					 			while ( (Mailbox_getNumPendingMsgs(mbxCHAN_DataXmit) != 0) && i--)	{
	    					 				Task_sleep(CHANNEL_TIMEFRAME_MS);
	    					 			}
	    					 			if(i == 0){
	    					 				System_printf("mbxCHAN_DataXmit did not handles in %d[ms]\n",CHANNEL_TIMEFRAME_MS*2);
	    					 				System_flush();
	    					 			}

	    					 			Semaphore_post(mutexPARSER_RecordingActive);
	    				 }
	    	continue;
		}
		}
	    else
	    {
	    	 if(BLE_GetStatus() == BLE_STATUS_CONNECTED)
	    	 {
	    		 onBefore=true;
	    	 }
	    }
		Mailbox_pend(mbxCHAN_DataXmit, &buffIdx, BIOS_WAIT_FOREVER);
		// Ensure EDF record is not truncated in the middle of "Z !" is intercepted

		BLE_Lock();




		// write start-of-record signature
		if(BLE_SendData((char*)&CHAN_StartOfRecordSignature, sizeof(Int16)) == UART_ERROR)	{
			System_printf("BLE_Write failed on SOR (probably multithread concurrent access)\n");
			System_flush();
		}

		if(BLE_SendData((char*)CHAN_Record[buffIdx], CHAN_RecordSize*sizeof(Int16)) == UART_ERROR)	{
			System_printf("BLE_Write failed (probably multi-thread concurrent access)\n");
			System_flush();
		}

		// write start-of-record signature
		if(BLE_SendData((char*)&CHAN_EndOfRecordSignature, sizeof(Int16)) == UART_ERROR)	{
			 System_printf("BLE_Write failed on EOR (probably multithread concurrent access)\n");
			 System_flush();
		}

		BLE_Unlock();
		Semaphore_post(mutexCHAN_DataXmitPending);



	}	// while(1)
}


void taskCHAN_StoreChannels()
{
	#if (SD_STORE_EDF==1)
	Uint8	buffIdx;

	System_printf("Starting taskCHAN_StoreChannels()\n");
	System_flush();
	while(1)	{
		Mailbox_pend(mbxCHAN_DataStore, &buffIdx, BIOS_WAIT_FOREVER);

		EDFFile_RecordLock();
		EDFFile_StoreChannelData((char*)CHAN_Record[buffIdx], CHAN_RecordSize*sizeof(Int16));
		EDFFile_RecordUnlock();
		Semaphore_post(mutexCHAN_DataStorePending);
	}
	#endif
}


Bool CHAN_GetChannelNameStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
		strcpy(str,CHAN_Struct[ch].Desc->Name);
		return TRUE;
	}
	return FALSE;
}

Bool CHAN_GetChannelXducerStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
		strcpy(str,CHAN_Struct[ch].Desc->XducerType);
		return TRUE;
	}
	return FALSE;
}

Bool CHAN_GetChannelPhysDimStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
		strcpy(str,CHAN_Struct[ch].Desc->PhysicalDimension);
		return TRUE;
	}
	return FALSE;
}

Bool CHAN_GetChannelFilterStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
		strcpy(str,CHAN_Struct[ch].Desc->FilterType);
		return TRUE;
	}
	return FALSE;
}

Bool CHAN_GetChannelIsSyncStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
		sprintf(str,"%d",CHAN_Struct[ch].Desc->IsSync);
		return TRUE;
	}
	return FALSE;
}

Bool CHAN_GetChannelSampleRateStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
		if(CHAN_Struct[ch].Desc->IsSync){
		sprintf(str,"%u",(UInt16)(CHAN_Struct[ch].Desc->SampleRate*(UInt32)CHANNEL_TIMEFRAME_MS/(UInt32)1000));	// return Samples in Timeframe
		}
		else	{	// In Async channels sample rate is not considered
			strcpy(str,"1");
		}
		return TRUE;
	}
	else	{	// In Async channels sample rate is not considered
		strcpy(str,"1");
	}
	return FALSE;
}

Bool CHAN_GetChannelPhysMinStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
		sprintf(str,"%ld",CHAN_Struct[ch].Desc->PhysicalMin);
		return TRUE;
	}
	return FALSE;
}

Bool CHAN_GetChannelPhysMaxStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
		sprintf(str,"%ld",CHAN_Struct[ch].Desc->PhysicalMax);
		return TRUE;
	}
	return FALSE;
}

Bool CHAN_GetChannelDigMinStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
#ifdef ADS131_16BIT_MODE
		sprintf(str,"%d",CHAN_Struct[ch].Desc->DigitalMin);
#else
		System_abort("CHAN_GetChannelDigMinStr() not implemented for Int32\n");
#endif
		return TRUE;
	}
	return FALSE;
}

Bool CHAN_GetChannelDigMaxStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
#ifdef ADS131_16BIT_MODE
		sprintf(str,"%d",CHAN_Struct[ch].Desc->DigitalMax);
#else
		System_abort("CHAN_GetChannelDigMinStr() not implemented for Int32\n");
#endif
		return TRUE;
	}
	return FALSE;
}



Bool CHAN_GetTimeframeStr(char *str)
{
	//sprintf(str,"%f",(float)CHANNEL_TIMEFRAME/(float)ADS131_SAMPLE_RATE);	// %f is not supported. see: http://processors.wiki.ti.com/index.php/Printf_support_for_MSP430_CCSTUDIO_compiler
	sprintf(str,"%u [ms]",CHANNEL_TIMEFRAME_MS);
	return TRUE;
}

Bool CHAN_GetChannelIsActiveStr(CHAN_Channels_t ch, char *str)
{
	if(ch < CHAN_ChannelsNum)	{
		sprintf(str,"%d",CHAN_Struct[ch].Params.IsActive);
		return TRUE;
	}
	return FALSE;
}


Bool CHAN_GetChannelInputStr(CHAN_Channels_t ch, char *str)
{
	UInt8 val;

	if(ADS131_GetChannelInput((ADS131_Channels_t)ch, (ADS131_Input_t*)&val)) {	// this command relates to numbered ADS131 channels (not EDF channels)
		sprintf(str,"%d",val);
		return TRUE;
	}
	return FALSE;
}

#endif
