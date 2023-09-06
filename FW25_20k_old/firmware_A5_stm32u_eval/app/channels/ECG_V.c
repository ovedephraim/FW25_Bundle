/*
* @file ECG_V.c
* @brief vertical ECG channel handler
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include <string.h>

#include "ECG_V.h"
#include "MAX30003_dev.h"
#include "MAX3000N/MAX3000N.h"
#include "channel_manager_task.h"
#include "main.h"

#define MOD_NAME "[ECG_V CH8]"
#if 1 //ENABLE_ECG_V_CHAN == 1

#define CMD_BUFF_SIZE	        sizeof(CHAN_Proc_Stream_Vals_t)
#define N_CMD_BUFFERS	        (40)

extern MEMBUF_POOL ecgv_cmdPool;

static fifo_t fifoStreamOut;
static int32_t	* fifoBuff;


#endif

/**
 * @brief ECG_V_Init
 * @brief This function used for resurce allocation
 * and general initialization
 * @param p[in/out] -channel data structure
 * @returns none
 */
void ECG_V_Init(void *p)
{
#if ENABLE_ECG_V_CHAN == 1

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
		(uint16_t)(_p->Params.sampleRate*4)*1.0/(DISP_INTERVALSEC)*sizeof(uint32_t)*ECG_V_NUM_OF_SENSORS;
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
		if(initMemBufPool(&ecgv_cmdPool,p,
				(CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS,
				CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR),N_CMD_BUFFERS)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}

	/* check if channel is active */
	if(true ==_p->Params.IsActive)
	{
		/* stream channel initialization */
		if(MAX30003_dev_ECG_init_sream()){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* print informative log message */
	    char * str="[ECG_V] init done\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}
	else
	{
		/* stream channel de-initialization */
		MAX30003_dev_ECG_deinit_sream();

		char * str="\r\n[ECG_V] disabled! (deinit)\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}

#endif /* ENABLE_ECG_V_CHAN */

}/* end of ECG_V_Init */

/**
 * @brief ECG_V_SetActivation
 * @brief This function activates ECG channel device and streaming
 * @param p[in/out] -channel data structure
 * @returns none
 */
void ECG_V_SetActivation(void *p, bool cmd)
{
#if ENABLE_ECG_V_CHAN == 1

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
			/* clear fifo after streaming */
			//fifo_clear(&fifoStreamOut);

			/* start steaming */
			if(MAX3000N_Start_ECG())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

//			/* start steaming */
//			if(MAX30003_dev_ECG_start_stream())
//			{
//				Error_Handler((uint8_t *)__FILE__, __LINE__);
//			}
		}
		else
		{
			/* stop steaming */
			if(MAX30003_dev_ECG_stop_stream())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
			/* clear fifo after streaming */
			fifo_clear(&fifoStreamOut);
		}
	}/* if(_p->Params.IsActive) */

#endif
}/* End of ECG_V_SetActivation */

/**
 * @brief ECG_V_Handler
 * @brief this function handles recieved samples
 * @param sample
 * @returns bool
 */
bool ECG_V_Handler(void *p)
{
#if ENABLE_ECG_V_CHAN == 1
    int rv=0;

	/* Process stream data */
    rv=MAX30003_dev_ECG_handle_stream(p);
	if(rv)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
#endif

	return true;
}/* end of ECG_V_Handler */

/**
 * @brief ECG_V_Reset
 * @brief TBD
 * @returns none
 */
void ECG_V_Reset()
{

}/* End of ECG_V_Reset */
