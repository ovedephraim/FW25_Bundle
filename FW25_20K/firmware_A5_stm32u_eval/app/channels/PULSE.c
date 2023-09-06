/**************************************************************************//**
* @file PULSE.c
* @brief Pulse E1 channel handler
* @author Anton Kanaev
* @version 0.0.1
* @date 18.06.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include "PULSE.h"
#include "LIS2DW_dev.h"
#include "channel_manager_task.h"
#include "sys_conf.h"
#include "main.h"

#define MOD_NAME "[PULSE CH1]"


static fifo_t fifoStreamOut;
static int init_done=false;

/**
 * @brief PULSE_Init
 * @brief This function used for resurce allocation
 * and general initialization
 * @param p[in/out] -channel data structure
 * @returns none
 */
void PULSE_Init(void *p)
{
	CHAN_Struct_t *_p = p;
	uint16_t size=0;
	int32_t	* fifoBuff=NULL;

#if ENABLE_PULSE_CHAN == 1//

	if(false==init_done && true ==_p->Params.IsActive)
	{
		/* check args */
		if(_p==NULL)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
			return;
		}

		/* allocate fifo buffer for this channel */
		size = \
		(uint16_t)(_p->Params.sampleRate*4)*1.0/(DISP_INTERVALSEC)*sizeof(uint32_t)*PULSE_NUM_OF_SENSORS;
		if(NULL == (fifoBuff=pvPortMalloc(size)))
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* allocate fifo buffer for this channel */
		if(!_p->Vals.pfifoStreamOut)
		{
			/* create fifo for this channel */
			fifo_init(&fifoStreamOut, fifoBuff,(size/sizeof(uint32_t)));
			_p->Vals.pfifoStreamOut=&fifoStreamOut;
		}

		/* stream channel initialization */
		LIS2DW_dev_init_sream();

		/* print informative log message */
		char * str="\r\n[PULSE CH1] init complete\r\n";
		aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
		init_done=true;

		/* at least one device streaming now */
		if(streaming_dev_num>0)
		{
			if(LIS2DW_dev_PULSE_start_stream())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}
	}
	else if(false ==_p->Params.IsActive)
	{
		/* stream channel initialization */
		LIS2DW_dev_deinit_sream();

		char * str="[PULSE CH1] disabled! (deinit)\r\n";
		aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
		init_done=false;
	}

#endif

}/* end of PULSE_Init */


/**
 * @brief PULSE_SetActivation
 * @brief This function activates ECG channel device and srteaming
 * @param p[in/out] -channel data structure
 * @returns none
 */
void PULSE_SetActivation(void *p, bool cmd)
{
	CHAN_Struct_t *_p = p;

	if(_p==NULL)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		return;
	}


#if ENABLE_PULSE_CHAN == 1//

	/* check ECG stream  configuration */
	if(_p->Params.IsActive)
	{
		if(true==cmd)
		{
			/* todo check if logger enable */
			if(LIS2DW_dev_PULSE_start_stream())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}
		else
		{
			if(LIS2DW_dev_PULSE_stop_stream())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}

	}/* if(_p->Params.IsActive) */

#endif

}/* End of PULSE_SetActivation */



/**
 * @brief PULSE_Handler
 * @brief this function handles recieved samples
 * @param sample
 * @returns bool
 */
bool PULSE_Handler(void *p)
{
	/* Process ecg horizontal stream */
	if(LIS2DW_dev_PULSE_handle_stream(p))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	return true;
}/* end of PULSE_Handler */

/**
 * @brief PULSE_Reset
 * @brief TBD
 * @returns none
 */
void PULSE_Reset()
{

}/* End of PULSE_Reset */
