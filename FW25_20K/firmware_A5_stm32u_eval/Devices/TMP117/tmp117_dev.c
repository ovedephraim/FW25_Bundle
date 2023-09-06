/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */



#include <stdio.h>
#include "Freertos.h"
#include <stdint.h>
#include "Freertos.h"
#include <string.h>
#include <freertos.h>
#include <task.h>
#include <queue.h>

#include "stm32u5xx_hal.h"
#include "tmp117/TMP117.h"
#include "tmp117_dev.h"
#include "bus.h"
#include "channel_manager_task.h"
#include "main.h"
#include "rtc.h"

extern QueueHandle_t chanmngq;

#define CMD_BUFF_SIZE	        sizeof(CHAN_Proc_Stream_Vals_t)
#define N_CMD_BUFFERS	        (40)

#define SKIN		0
#define POLY		1
#define AMB			2

TMP117_Object_t tmp117_obj1;
TMP117_Object_t tmp117_obj2;
TMP117_Object_t tmp117_obj3;

static bool heatEn=false;

static fifo_t *temperature_fifo;
static MEMBUF_POOL dev_cmdPool;
static int16_t temp_data[3];

uint32_t acc_rec_len_tmp117=0;

extern int16_t temp_datax[];

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

uint32_t uwPeriod = 0;
uint32_t uwPulse = 0;



/**
 * @brief LSM6DSL_dev_ACC_init_sream
 * @brief device/stream init function
 * @return 0 success
 */
int TEMPX_dev_init_sream(void)
{
	TMP117_IO_t  io_ctx;
	int16_t      id=0xFF;
	void * p;
	char dbug[200]={0};

	/* ========= I2C bus initialization =========== */

	io_ctx.Init        = BUS_I2C4_Init;
	io_ctx.DeInit      = BUS_I2C4_DeInit;
	io_ctx.ReadReg     = BUS_I2C4_ReadReg;
	io_ctx.WriteReg    = BUS_I2C4_WriteReg;
	io_ctx.GetTick     = BUS_GetTick;

	io_ctx.Address     = TMP117_I2C_ADD1;
	if (TMP117_RegisterBusIO(&tmp117_obj1, &io_ctx) != TMP117_OK)
	{
	  return SYS_ERROR_BUS_FAILURE;
	}

	io_ctx.Address     = TMP117_I2C_ADD2;
	if (TMP117_RegisterBusIO(&tmp117_obj2, &io_ctx) != TMP117_OK)
	{
	  return SYS_ERROR_BUS_FAILURE;
	}

	io_ctx.Address     = TMP117_I2C_ADD3;
	if (TMP117_RegisterBusIO(&tmp117_obj3, &io_ctx) != TMP117_OK)
	{
	  return SYS_ERROR_BUS_FAILURE;
	}


	/* wait for bus will be ready before reading */
	if(BUS_I2C4_IsReady(TMP117_I2C_ADD1, 100)!= TMP117_OK)
	{
		sprintf(dbug,"[TEMP][0x%x] bus error! \r\n",
				(int)TMP117_I2C_ADD1);
		/* send to debug aux channel */
		aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
	}
	else
	{
		if(TMP117_readId(&tmp117_obj1, &id) == 0)
		{
			if(id == TMP117_WHOAMI)
			{
				sprintf(dbug,"[TMP117][%x] detected \r\n",(int)TMP117_I2C_ADD1);
			}
			else //AS6221_
			{
				sprintf(dbug,"[AS6221][%x] detected \r\n",(int)TMP117_I2C_ADD1);

			}
			/* send to debug aux channel */
			aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
		}
	}

	/* wait for bus will be ready before reading */
	if(BUS_I2C4_IsReady(TMP117_I2C_ADD2, 1000)!= TMP117_OK)
	{
		sprintf(dbug,"[TEMP][0x%x] bus error! \r\n",
				(int)TMP117_I2C_ADD2);
		/* send to debug aux channel */
		aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
	}
	else
	{
		if(TMP117_readId(&tmp117_obj2, &id) == 0)
		{
			if(id == TMP117_WHOAMI)
			{
				sprintf(dbug,"[TMP117][%x] detected \r\n",(int)TMP117_I2C_ADD2);
			}
			else //AS6221_
			{
				sprintf(dbug,"[AS6221][%x] detected \r\n",(int)TMP117_I2C_ADD2);

			}
			/* send to debug aux channel */
			aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
		}
	}

	/* wait for bus will be ready before reading */
	if(BUS_I2C4_IsReady(TMP117_I2C_ADD3, 1000)!= TMP117_OK)
	{
		sprintf(dbug,"[TEMP][0x%x] bus error! \r\n",
				(int)TMP117_I2C_ADD3);
		/* send to debug aux channel */
		aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
	}
	else
	{
		if(TMP117_readId(&tmp117_obj3, &id) == 0)
		{
			if(id == TMP117_WHOAMI)
			{
				sprintf(dbug,"[TMP117][%x] detected \r\n",(int)TMP117_I2C_ADD3);
			}
			else //AS6221_
			{
				sprintf(dbug,"[AS6221][%x] detected \r\n",(int)TMP117_I2C_ADD3);

			}
			/* send to debug aux channel */
			aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
		}
	}

	/* ========= memory initialization =========== */

	/* allocate memory area for the memory pool */
	p=_MEM_alloc((CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS);
	if(p==NULL){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* Create memory pool of fixed size buffers  */
	if(initMemBufPool(&dev_cmdPool,p,
			(CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS,
			CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR),N_CMD_BUFFERS)){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* ========= TEMP sensor initialization =========== */
	//TODO TBD

	/* Compute the Timer period to generate a signal frequency at 1Hz */
	uwPeriod = (SystemCoreClock) - 1;

	/* Compute Pulse4 value to generate a duty cycle at 10%  for channel 4 */
	uwPulse = (100 * (uwPeriod- 1)) / 1000;


	/* Initialize TIMx peripheral as follow:
	   + Prescaler = 0
	   + Period = uwPeriod  (to have an output frequency equal to 1 Hz)
	   + ClockDivision = 0
	   + Counter direction = Up
	*/
	TimHandle.Instance = TIM16;

	TimHandle.Init.Period            = 65535;//uwPeriod;
	TimHandle.Init.Prescaler         = 0;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
	TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if(HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* Common configuration for all channels */
	sConfig.OCMode      = TIM_OCMODE_PWM1;
	sConfig.OCFastMode  = TIM_OCFAST_DISABLE;
	sConfig.OCPolarity  = TIM_OCPOLARITY_LOW;
	sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfig.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfig.OCNIdleState= TIM_OCNIDLESTATE_RESET;

	/* Set the pulse value for TODO Ephraim TBD */
	sConfig.Pulse = 65535 - 6550; //uwPulse;
	if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}


	/* ========= fetch fifo handler of pulse channel ===========*/
	if(NULL == (temperature_fifo=CHAN_GetChannelfifo(CHAN_TEMP_SKIN)))
	{
		char * str="\r\n[TEMP] CHAN_GetChannelfifo FAILED \r\n";
		aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
		return SYS_ERROR_DEVICE_FAILURE;
	}


	return SYS_ERROR_NONE;
}/* End of LIS2DW_dev_init_sream */

/**
 * @brief LSM6DSL_dev_start_stream
 * @brief LSM6DSL start stream
 * @return 0 success
 */
int TEMPX_dev_start_stream(void)
{
	sys_param.cf.is_temp = 1;
    return SYS_ERROR_NONE;
}/* End of TEMPX_dev_start_stream */

/**
 * @brief TEMPX_dev_stop_stream
 * @brief TEMPX stop stream
 * @return 0 success
 */
int TEMPX_dev_stop_stream(void)
{
	sys_param.cf.is_temp = 0;
    return  SYS_ERROR_NONE;
}/* End of LIS2DW_dev_stop_stream */

/**
 * @brief TEMPX_dev_proc_stream
 * @brief TEMPX process stream
 * @return 0 success
 */
int TEMPX_dev_proc_stream(void)
{
	if(sys_param.cf.is_temp)
	{
		CHAN_Proc_Stream_Vals_t * buff=NULL;
		/* get fresh command buffer */
		if(!(buff=(CHAN_Proc_Stream_Vals_t*)getMemBuf(&dev_cmdPool))){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}


		/* fetch time stamp and other fields */
		buff->ts=0;//RTC_GetTimestampMillis();
		buff->id=CHAN_TEMP_SKIN;
		buff->num=0;

		/* send to channel manager command from ISR */
		if (pdPASS!=sendToChannelManagerFromISR(chanmngq, buff,
				CHAN_CMD_PROC_STEAM, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_DBRX,MSG_TYPE_CMD))){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}

	return  SYS_ERROR_NONE;
}/* End of TEMPX_dev_proc_stream */


/**
 * @brief TEMPX_dev_handle_stream
 * @brief reader data aka handler
 * @return 0 success
 */
int TEMPX_dev_handle_stream(void * p)
{
	fifo_t *tmp_fifo=temperature_fifo;
	uint16_t count=DEF_ODR_TMP*DEF_AXE_TMP;

	/* read sensor #1 value */
	if(TMP117_readTemp(&tmp117_obj1,&temp_data[SKIN]))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
	/* read sensor #2 value */
	if(TMP117_readTemp(&tmp117_obj2,&temp_data[POLY]))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
	/* read sensor #3 value */
	if(TMP117_readTemp(&tmp117_obj3,&temp_data[AMB]))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	temp_datax[SKIN] = temp_data[SKIN];
	temp_datax[POLY] = temp_data[POLY];
	temp_datax[AMB] = temp_data[AMB];

	/* temperature tracking mechanism*/
	/* TODO ephraim add Logic */
	if(heatEn)
	{

		if((temp_data[SKIN] - temp_data[POLY]) > 20)
		{
			if(HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
			{
			  /* Starting Error */
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

			if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
			{
			  /* Starting Error */
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}
		else
		{
			if(HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
			{
			  /* Starting Error */
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}

	}
	else
	{
		/* TODO check if stop needed */

		if(HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
		{
		  /* Starting Error */
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}


	/* insert time stamp in the queue */
	acc_rec_len_tmp117+=count;
	if (-1 == fifo_put32(tmp_fifo,acc_rec_len_tmp117)){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* insert number of samples collected in the queue */
	if (-1 == fifo_put32(tmp_fifo,count)){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}



	/* insert sample  in the queue */
	if (-1 == fifo_put32(tmp_fifo,temp_data[SKIN])) {
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
	/* insert sample  in the queue */
	if (-1 == fifo_put32(tmp_fifo,temp_data[POLY])) {
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
	/* insert sample  in the queue */
	if (-1 == fifo_put32(tmp_fifo,temp_data[AMB])) {
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

#ifdef MODULE_DEBUG

	char dbug[200]={0};
	sprintf(dbug,"[TEMP] SKIN [%d][%f] POLY [%d][%f] AMB [%d][%f] \r\n",
			temp_data[SKIN],(float)temp_data[SKIN]*TMP117_RESOLUTION,
			temp_data[POLY],(float)temp_data[POLY]*TMP117_RESOLUTION,
			temp_data[AMB], (float)temp_data[AMB]*TMP117_RESOLUTION);

	/* send to debug aux channel */
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

#endif

return 0;
}/* End of LSM6DSL_dev_ECG_handle_stream */

/**
 * @brief TEMP_HeaterEnable
 * @brief This function used for Temperature tracking
 * @param en[in] - true for enable
 * @returns none
 */
void TEMPX_HeaterEnable(bool en)
{
	heatEn=en;
}/* End of TEMP_HeaterEnable */

