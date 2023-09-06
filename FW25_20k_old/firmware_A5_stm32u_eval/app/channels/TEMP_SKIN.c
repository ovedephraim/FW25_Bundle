/*
* @file ECG.c
* @brief horizontal ECG channel handler
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include <string.h>

#include "TEMP_SKIN.h"
#include "TMP117/tmp117_dev.h"
#include "channel_manager_task.h"
#include "main.h"

#define MOD_NAME "[TEMP_SKIN]"
#if ENABLE_TEMP_SKIN_CHAN == 1

#define CMD_BUFF_SIZE	        sizeof(CHAN_Proc_Stream_Vals_t)
#define N_CMD_BUFFERS	        (10)

static fifo_t fifoStreamOut;
static int32_t	* fifoBuff;
MEMBUF_POOL cmdPool;

#endif


/**
 * @brief TEMP_SKIN_Init
 * @brief This function used for resurce allocation
 * and general initialization
 * @param p[in/out] -channel data structure
 * @returns none
 */
void TEMP_SKIN_Init(void *p)
{
#if ENABLE_TEMP_SKIN_CHAN == 1

	CHAN_Struct_t *_p = p;
	uint16_t size = 0;

	if(_p==NULL)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		return;
	}

	if(!_p->Vals.pfifoStreamOut)
	{
		/* allocate fifo buffer for this channel */
		size = \
		(uint16_t)(_p->Params.sampleRate*4)*1.0/(DISP_INTERVALSEC)*sizeof(uint32_t)*TEMP_SKIN_NUM_OF_SENSORS;
		if(NULL == (fifoBuff=pvPortMalloc(size)))
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* create fifo for this channel */
		fifo_init(&fifoStreamOut, fifoBuff,(size/sizeof(uint32_t)));
		_p->Vals.pfifoStreamOut=&fifoStreamOut;
	}

	/* check if channel is active */
	if(true ==_p->Params.IsActive)
	{
	    /* stream channel initialization */
		if(TEMPX_dev_init_sream())
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
		/* print informative log message */
	    char * str="\r\n[TEMP] init done\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}
	else
	{
		char * str="\r\n[TEMP] disabled! (deinit)\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}

#endif

}/* end of TEMP_SKIN_Init */

/**
 * @brief TEMP_SKIN_SetActivation
 * @brief This function activates ECG channel device and streaming
 * @param p[in/out] -channel data structure
 * @returns none
 */
void TEMP_SKIN_SetActivation(void *p, bool cmd)
{
#if ENABLE_TEMP_SKIN_CHAN == 1

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
			if(TEMPX_dev_start_stream())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}
		else
		{
			if(TEMPX_dev_stop_stream())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}

	}/* if(_p->Params.IsActive) */

#endif

}/* End of TEMP_SKIN_SetActivation */

/**
 * @brief TEMP_SKIN_Handler
 * @brief this function handles received samples
 * @param sample
 * @returns bool
 */
bool TEMP_SKIN_Handler(void *p)
{
	/* Process ecg horizontal stream */
	if(TEMPX_dev_handle_stream(p))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	return true;
}/* end of TEMP_SKIN_Handler */

/**
 * @brief TEMP_SKIN_Reset
 * @brief TBD
 * @returns none
 */
void TEMP_SKIN_Reset()
{

}/* End of TEMP_SKIN_Reset */

