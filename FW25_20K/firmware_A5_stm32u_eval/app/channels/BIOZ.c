/*
* @file BIOZ.c
* @brief horizontal ECG channel handler
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include <string.h>

#include "BIOZ.h"
#include "MAX30001_dev.h"
#include "MAX3000N/MAX3000N.h"
#include "channel_manager_task.h"
#include "main.h"
#include "sys_conf.h"

#define MOD_NAME "[BIOZ]"
#if ENABLE_BIOZ_CHAN == 1

#define CMD_BUFF_SIZE	        sizeof(CHAN_Proc_Stream_Vals_t)
#define N_CMD_BUFFERS	        (40)

static fifo_t fifoStreamOut;
static int32_t	* fifoBuff;
extern MEMBUF_POOL ecgh_cmdPool;

#endif

/**
 * @brief BIOZ_Init
 * @brief This function used for resurce allocation
 * and general initialization
 * @param p[in/out] -channel data structure
 * @returns none
 */
void BIOZ_Init(void *p)
{
#if ENABLE_BIOZ_CHAN == 1
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
		/* allocate fifo buffer for this channel */
		size = \
		(uint16_t)(_p->Params.sampleRate*4)*1.0/(DISP_INTERVALSEC)*sizeof(uint32_t)*BIOZ_NUM_OF_SENSORS;
		if(NULL == (fifoBuff=pvPortMalloc(size))){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
		/* create fifo for this channel */
		fifo_init(&fifoStreamOut, fifoBuff,(size/sizeof(uint32_t)));
		_p->Vals.pfifoStreamOut=&fifoStreamOut;

		void * p=NULL;

		/* ========= resource allocation ============== */

		/* allocate memory area for the memory pool */
		p=_MEM_alloc((CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS);
		if(p==NULL){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* Create memory pool of fixed size buffers  */
		if(initMemBufPool(&ecgh_cmdPool,p,
				(CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS,
				CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR),N_CMD_BUFFERS)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}

	/* check if channel is active */
	if(true ==_p->Params.IsActive)
	{
		/* stream channel initialization */
		if(MAX30001_dev_BIOZ_init_sream()){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* print informative log message */
	    char * str="[BIOZ] init done\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}
	else
	{
		/* stream channel de-initialization */
		MAX30001_dev_BIOZ_deinit_sream();

		char * str="\r\n[BIOZ] disabled! (deinit)\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}
#endif /* ENABLE_BIOZ_CHAN */

}/* end of BIOZ_Init */

/**
 * @brief BIOZ_SetActivation
 * @brief This function activates ECG channel device and streaming
 * @param p[in/out] -channel data structure
 * @returns none
 */
void BIOZ_SetActivation(void *p, bool cmd)
{
#if ENABLE_BIOZ_CHAN == 1

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

			/* start steaming */
			if(MAX30001_dev_BIOZ_start_stream())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}
		else
		{
			/* stop steaming */
			if(MAX30001_dev_BIOZ_stop_stream())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
			/* clear fifo after streaming */
			fifo_clear(&fifoStreamOut);
		}
	}/* if(_p->Params.IsActive) */

#endif
}/* End of BIOZ_SetActivation */

/**
 * @brief BIOZ_Handler
 * @brief this function handles recieved samples
 * @param sample
 * @returns bool
 */
bool BIOZ_Handler(void *p)
{
#if ENABLE_BIOZ_CHAN == 1

	/* Process ecg horizontal stream */
	if(MAX30001_dev_handle_stream(p))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

#endif
	return true;
}/* end of BIOZ_Handler */

/**
 * @brief BIOZ_Reset
 * @brief TBD
 * @returns none
 */
void BIOZ_Reset()
{

}/* End of BIOZ_Reset */
