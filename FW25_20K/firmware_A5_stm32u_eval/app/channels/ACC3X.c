/*
* @file ECG.c
* @brief ECG channel handler
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include "ACC3X.h"
#include "channel_manager_task.h"
#include "sys_conf.h"
#include "main.h"


#define MOD_NAME "[IMU ACC]"

#if ENABLE_ACC3X_CHAN == 1
static fifo_t fifoStreamOut;
static int32_t	* fifoBuff;
#endif

/**
 * @brief ACC3X_Init
 * @brief This function used for resource allocation
 * and general initialization
 * @param p[in/out] -channel data structure
 * @returns none
 */
void ACC3X_Init(void *p)
{
#if ENABLE_ACC3X_CHAN == 1

	CHAN_Struct_t *_p = p;
	uint16_t size = 0;

	if(_p==NULL)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		return;
	}

	/* allocate fifo buffer for this channel */
	size = \
	(uint16_t)(_p->Params.sampleRate*4)*1.0/(DISP_INTERVALSEC)*sizeof(uint32_t)*ACC3X_NUM_OF_SENSORS;
	if(NULL == (fifoBuff=pvPortMalloc(size)))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* create fifo for this channel */
	fifo_init(&fifoStreamOut, fifoBuff,(size/sizeof(uint32_t)));
	_p->Vals.pfifoStreamOut=&fifoStreamOut;

	/* check if channel is active */
	if(true ==_p->Params.IsActive)
	{
	    /* stream channel initialization */
		if(LSM6DSL_dev_ACC_init_sream())
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
		/* print informative log message */
	    char * str="\r\n[ACC3X] init done\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}
	else
	{

		/* stream channel de-initialization */
		LSM6DSL_dev_ACC_deinit_sream();

		/* TODO turn of for lower power consumption */

		char * str="\r\n[ACC3X] disabled! (deinit)\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}

#endif

}/* end of ACC3X_Init */


/**
 * @brief ACC3X_SetActivation
 * @brief This function activates IMU ACC channel device and srteaming
 * @param p[in/out] -channel data structure
 * @returns none
 */
void ACC3X_SetActivation(void *p, bool cmd)
{

#if ENABLE_ACC3X_CHAN == 1

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
			if(LSM6DSL_dev_ACC_start_stream())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}
		else
		{
			if(LSM6DSL_dev_ACC_stop_stream())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}

	}/* if(_p->Params.IsActive) */

#endif

}/* End of ACC3X_SetActivation */


/**
 * @brief ACC3X_Handler
 * @brief this function handles recieved samples
 * @param sample
 * @returns bool
 */
bool ACC3X_Handler(void *p)
{
	/* Process ecg horizontal stream */
	if(LSM6DSL_dev_handle_stream(p))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	return true;
}/* end of ACC3X_Handler */

/**
 * @brief ACC3X_Reset
 * @brief TBD
 * @returns none
 */
void ACC3X_Reset()
{

}/* End of ACC3X_Reset */
