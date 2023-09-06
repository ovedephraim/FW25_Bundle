/*
* @file AN_RESP.c
* @brief Analog-respiration channel handler
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/
#include <string.h>
#include "AN_RESP.h"
#include "channel_manager_task.h"
#include "main.h"
#include "bus.h"
#include "rtc.h"

extern QueueHandle_t chanmngq;
#define MOD_NAME "[AN RESP]"
#define CMD_BUFF_SIZE	        sizeof(CHAN_Proc_Stream_Vals_t)
#define N_CMD_BUFFERS	        (10)
#define N_DATA_BUFFERS	        (2)

#if ENABLE_AN_RESP_CHAN == 1
static fifo_t fifoStreamOut;
static int32_t	* fifoBuff;
static bool AN_RESP_move_data_Handler(void *p, void *p2,uint32_t len);

ADC_ChannelConfTypeDef an_resp_conf=
{
	.Channel 	   = ADC_CHANNEL_10,
	.Rank 		   = ADC_REGULAR_RANK_3,
	.SamplingTime  = ADC_SAMPLETIME_391CYCLES_5,
	.SingleDiff    = ADC_DIFFERENTIAL_ENDED,
	.OffsetNumber  = ADC_OFFSET_2,
	.Offset        = ADC_OFFSET_VAL
};

#endif

//uint32_t an_resp_buff[512];
MEMBUF_POOL aresp_cmdPool;
MEMBUF_POOL aresp_dataPool;

uint32_t acc_rec_len_resp=0;

/**
 * @brief AN_RESP_Init
 * @brief This function used for resource allocation
 * and general initialization
 * @param p[in/out] -channel data structure
 * @returns none
 */
void AN_RESP_Init(void *p)
{
#if ENABLE_AN_RESP_CHAN == 1

	CHAN_Struct_t *_p = p;
	uint16_t size = 0;

	if(_p==NULL)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		return;
	}

	/* allocate fifo buffer for this channel */
	if(!_p->Vals.pfifoStreamOut)
	{
		uint32_t data_len=0;

		/* allocate fifo buffer for this channel */
		size = \
		(uint16_t)(_p->Params.sampleRate*4)*1.0/(DISP_INTERVALSEC)*sizeof(uint32_t)*AN_RESP_NUM_OF_SENSORS;
		if(NULL == (fifoBuff=pvPortMalloc(size))){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
		/* create fifo for this channel */
		fifo_init(&fifoStreamOut, fifoBuff,(size/sizeof(uint32_t)));
		_p->Vals.pfifoStreamOut=&fifoStreamOut;


		/* allocate memory area for the memory pool */
		p=_MEM_alloc((CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS);
		if(p==NULL){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
		/* Create memory pool of fixed size buffers  */
		if(initMemBufPool(&aresp_cmdPool,p,
				(CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS,
				 CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR),N_CMD_BUFFERS)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}


		/* allocate memory area for the memory pool */
		data_len=_p->Params.sampleRate* (DISP_INTERVALSEC)*sizeof(uint32_t);
		p=_MEM_alloc((data_len+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_DATA_BUFFERS);
		if(p==NULL){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
		/* Create memory pool of fixed size buffers  */
		if(initMemBufPool(&aresp_dataPool,p,
				(data_len+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_DATA_BUFFERS,
				 data_len+sizeof(PACKETBUF_HDR),N_DATA_BUFFERS)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}

	/* check if channel is active */
	if(true ==_p->Params.IsActive)
	{
		adc_reg_param_t p={0};

		p.sn=ADC1_CH2;
		p.cb=AN_RESP_move_data_Handler;
		p.mp=&aresp_dataPool;
		p.adc_seq_hndle=(ADC_ChannelConfTypeDef*)&an_resp_conf;
		p.inv=1;

	    /* stream channel initialization */
		if(BUS_ADC1_init_and_register(&p))//ADC1_CH2))
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* print informative log message */
	    char * str="\r\n[AN_RESP] init done\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}
	else
	{
	    /* stream channel initialization */
		if(BUS_ADC1_deinit_and_unregister(ADC1_CH2))
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		char * str="\r\n[AN_RESP] disabled! (deinit)\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}

#endif

}/* end of AN_RESP_Init */


/**
 * @brief AN_RESP_SetActivation
 * @brief This function activates device
 * and starts streaming
 * @param p[in/out] -channel data structure
 * @returns none
 */
void AN_RESP_SetActivation(void *p, bool cmd)
{

#if ENABLE_AN_RESP_CHAN == 1

	CHAN_Struct_t *_p = p;

	if(_p==NULL)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		return;
	}

	/* check ECG stream  configuration */
	if(_p->Params.IsActive)
	{
		if(true==cmd)
		{
			/* clear fifo before stopping */
			fifo_clear(CHAN_GetChannelfifo(CHAN_AN_RESP));

			/* start bus execution */
			if(BUS_ADC1_exec_run())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

			char * str="\r\n[AN_RESP] ADC bus activated\r\n";
		    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
		}
		else
		{
			/* clear fifo before stopping */
			fifo_clear(CHAN_GetChannelfifo(CHAN_AN_RESP));

			/* abort bus execution */
			if(BUS_ADC1_exec_stop())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

			char * str="\r\n[AN_RESP] ADC bus deactivated\r\n";
		    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
		}

	}/* if(_p->Params.IsActive) */

#endif

}/* End of AN_RESP_SetActivation */


/**
 * @brief AN_RESP_Handler
 * @brief this function handles recieved samples
 * @param sample
 * @returns bool
 */
bool AN_RESP_Handler(void *p)
{
	fifo_t *an_resp_fifo=CHAN_GetChannelfifo(CHAN_AN_RESP);
	uint32_t * pbuf=(uint32_t*)((CHAN_Proc_Stream_Vals_t*)p)->parg;

	if(!pbuf){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	acc_rec_len_resp+=((CHAN_Proc_Stream_Vals_t*)p)->num;
	/* insert time stamp in the queue */
	if (-1 == fifo_put32(an_resp_fifo,acc_rec_len_resp)){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* insert data length in the queue */
	if (-1 == fifo_put32(an_resp_fifo,((CHAN_Proc_Stream_Vals_t*)p)->num )){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* insert sample in the queue */
	for (int i=0; i<512; i++)
	{
		/* insert sample e1 in the queue */
	    if (-1 == fifo_put32(an_resp_fifo, pbuf[i])) {
	    	Error_Handler((uint8_t *)__FILE__, __LINE__);
	    }
	}/* for (int i=0; i<FIFO_tsh; i++) */

	/* free allocated memory */
	retMemBuf(pbuf);

	return true;
}/* end of AN_RESP_Handler */

/**
 * @brief AN_RESP_Reset
 * @brief TBD
 * @returns none
 */
void AN_RESP_Reset()
{

}/* End of AN_RESP_Reset */


/**
 * @brief AN_RESP_move_data_Handler
 * @brief when the data is ready pass it for processing
 * @param *p - argument handler
 * @returns bool
 */
static bool AN_RESP_move_data_Handler(void *p, void *p2,uint32_t len)
{
	CHAN_Proc_Stream_Vals_t * buff=NULL;

	/* get fresh command buffer */
	if(!(buff=(CHAN_Proc_Stream_Vals_t*)getMemBuf(&aresp_cmdPool))){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* fetch time stamp and other fields */
	buff->ts=*((uint32_t*)p);
	buff->id=CHAN_AN_RESP;
	buff->num=len;
	buff->parg=p2;

	/* send to channel manager command from ISR */
	if (pdPASS!=sendToChannelManagerFromISR(chanmngq, buff,
		CHAN_CMD_PROC_STEAM, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_DBRX,MSG_TYPE_CMD)))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	return true;
}/* end of AN_RESP_Handler */


