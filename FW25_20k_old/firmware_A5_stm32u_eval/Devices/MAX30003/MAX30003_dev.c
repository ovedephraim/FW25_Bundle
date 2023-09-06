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
#include <string.h>
#include "Freertos.h"
#include <stdint.h>
#include <freertos.h>
#include <task.h>
#include <queue.h>
#include "stm32u5xx_hal.h"

#include "MAX3000N/MAX3000N.h"
#include "MAX30003_bus.h"
#include "MAX30003_dev.h"
#include "cmd_handler_task.h"
#include "bus.h"
#include "channel_manager_task.h"
#include "cmd_handler_task.h"
#include "main.h"
#include "rtc.h"

extern QueueHandle_t chanmngq;

#define MOD_NAME "[MAX30003 DEV]"
#define EFIT        EFIT_AS_16

// Init values for ECG_InitStart()
#define EN_ECG     EN_ECG_COMMON
#define OPENP      OPENP_COMMON
#define OPENN      OPENN_COMMON
#define POL        POL_COMMON
#define CALP_SEL   CALP_SEL_COMMON
#define CALN_SEL   CALN_SEL_COMMON

#define E_FIT      E_FIT_COMMON
#define GAIN       GAIN_COMMON
#define DHPF       DHPF_COMMON
#define DLPF       DLPF_COMMON

// Initn values for CAL_InitStart()
#define EN_VCAL    EN_VCAL_COMMON
#define VMODE      VMODE_COMMON
#define VMAG       VMAG_COMMON
#define FCAL       FCAL_COMMON
#define THIGH      THIGH_COMMON
#define FIFTY      FIFTY_COMMON

EXTI_HandleTypeDef hexti11;
EXTI_HandleTypeDef hexti12;
MAX30003_Object_t max30003_obj_0;
MAX30003_Object_t *_pobj_0;
MEMBUF_POOL ecgv_cmdPool;

uint32_t acc_rec_len_max3003=0;

/* constant of masks in each register */
const MAX30003_EN_INT_MASKS EN_INT_DEFAULT_MASK     = ENINT_INTB_TYPE|ENINT_EN_EINT|ENINT_EN_EOVF;
const MAX30003_EN_INT_MASKS EN_INT2_DEFAULT_MASK    = ENINT_INTB_TYPE|ENINT_EN_LONINT;

/**
* @brief common interrupt handler
*/
int32_t MAX30003_int_handler(MAX30003_Object_t *pObj)
{
	CHAN_Proc_Stream_Vals_t * buff=NULL;
	int32_t return_value = 0;

	/* get fresh command buffer */
	if(!(buff=(CHAN_Proc_Stream_Vals_t*)getMemBuf(&ecgv_cmdPool))){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* fetch time stamp and other fields */
	buff->ts=0;//RTC_GetTimestampMillis();
	buff->id=CHAN_ECG_V;
	buff->num=0;

	/* send to channel manager command from ISR */
	if (pdPASS!=sendToChannelManagerFromISR(chanmngq, buff,
			CHAN_CMD_PROC_STEAM, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_DBRX,MSG_TYPE_CMD))){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

  return return_value;
}/* end of MAX30003_int_handler */

/**
* @brief Preventive measure used to dismiss interrupts that fire too early during
* @brief initialization on INTB line
*
*/
void MAX30003_Mid_IntB_Handler(void){
  MAX30003_int_handler(&max30003_obj_0);
}/* end of MAX30003_int_handler */

/**
* @brief Preventive measure used to dismiss interrupts that fire too early during
* @brief initialization on INT2B line
*/
void MAX30003_Mid_Int2B_Handler(void){
	MAX30003_int_handler(&max30003_obj_0);
}/* end of MAX30003_Mid_Int2B_Handler */

/**
  * @brief  Configures EXTI line (connected to PE.14,15 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void MAX30003_IRQHandler_Config(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_ConfigTypeDef EXTI_ConfigStructure;

	/* Enable GPIOC clock */
	MAX30003_INT_GPIO_CLK_ENABLE();

	/* Configure PE.14 pin as input floating */
	GPIO_InitStructure.Mode  = MAX30003_INT_MODE;
	GPIO_InitStructure.Speed = MAX30003_INT_SPEED;
	GPIO_InitStructure.Pull  = MAX30003_INT_PULL;
	GPIO_InitStructure.Pin   = MAX30003_INT_PIN;
	HAL_GPIO_Init(MAX30003_INT_GPIO_PORT, &GPIO_InitStructure);

	/* Configure PE.15 pin as input floating */
	GPIO_InitStructure.Mode  = MAX30003_INT2_MODE;
	GPIO_InitStructure.Pull  = MAX30003_INT2_PULL;
	GPIO_InitStructure.Pin   = MAX30003_INT2_PIN;
	GPIO_InitStructure.Speed = MAX30003_INT2_SPEED;
	HAL_GPIO_Init(MAX30003_INT2_GPIO_PORT, &GPIO_InitStructure);

	/* Set configuration except Interrupt and Event mask of Exti line 14,15*/
	EXTI_ConfigStructure.Line    = MAX30003_INT_EXTI_LINE;
	EXTI_ConfigStructure.Trigger = EXTI_TRIGGER_FALLING;
	EXTI_ConfigStructure.GPIOSel = EXTI_GPIOF;
	EXTI_ConfigStructure.Mode    = EXTI_MODE_INTERRUPT;
	HAL_EXTI_SetConfigLine(&hexti11, &EXTI_ConfigStructure);

	/* get event irq handler */
	if(HAL_OK != HAL_EXTI_RegisterCallback(&hexti11, HAL_EXTI_FALLING_CB_ID,MAX30003_Mid_IntB_Handler))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* Set configuration except Interrupt and Event mask of Exti line 14,15*/
	EXTI_ConfigStructure.Line    = MAX30003_INT2_EXTI_LINE;
	EXTI_ConfigStructure.Trigger = EXTI_TRIGGER_FALLING;
	EXTI_ConfigStructure.GPIOSel = EXTI_GPIOF;
	EXTI_ConfigStructure.Mode    = EXTI_MODE_INTERRUPT;
	HAL_EXTI_SetConfigLine(&hexti12, &EXTI_ConfigStructure);

	/* get event irq handler */
	if(HAL_OK != HAL_EXTI_RegisterCallback(&hexti12, HAL_EXTI_FALLING_CB_ID,MAX30003_Mid_Int2B_Handler))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* set interrupt priority grouping */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* set interrupt priority */
	HAL_NVIC_SetPriority(MAX30003_INT_EXTI_IRQn, 15, 1);

	/* set interrupt priority */
	HAL_NVIC_EnableIRQ(MAX30003_INT_EXTI_IRQn);

	/* set interrupt priority */
	HAL_NVIC_SetPriority(MAX30003_INT2_EXTI_IRQn, 15, 1);

	/* set interrupt priority */
	HAL_NVIC_EnableIRQ(MAX30003_INT2_EXTI_IRQn);

}/* EXTI_IRQHandler_Config */


/**
 * @brief MAX30003_dev_ECG_init_sream
 * @brief max300001 device initialization function
 * @return 0 success
 */
int MAX30003_dev_ECG_init_sream(void)
{
	uint32_t id=0xFF;
	int rv=SYS_ERROR_NONE;
	int partVersion=0xFF;
	MAX30003_VALS   old_vals;
	MAX30003_VALS   new_vals;

	_pobj_0=&max30003_obj_0;

	/* spi-bus registration */
	if (MAX30003_RegisterBusIO(_pobj_0) != MAX30003_OK){
		return SYS_ERROR_BUS_FAILURE;
	}

	/* read part version from the chip */
	MAX30003_readID(_pobj_0,&id);

	/* Version validation */
	partVersion = id >> 12;
	partVersion = partVersion & 0x3;
	if (partVersion != 3){
	    char * str="\r\n[FATAL][MAX30003] not detected \r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
		return SYS_ERROR_BUS_FAILURE;
	}

	char * str3="\r\n[MAX30003] detected \r\n";
	aux_sendToAux(str3,strlen(str3),0,1,DBG_AUX);


	/*  configure ecg hpf frequency */
	io_ctl_set_hpf_freq(ecg_HPF_05HZ);

	/* reset device */
	MAX30003_sw_rst(_pobj_0);

	/* get converted odr register value */
	max30003_obj_0.odr=conv_odr_val((uint16_t)DEF_ODR_ECG_V);

#if DEBUG_ENV_CAL_MAX3

	/* ECG signal simulation */
	MAX30003_CAL_InitStart(&max30003_obj_0,
			EN_VCAL , VMODE, VMAG, FCAL, THIGH, FIFTY);

	/* ECG stream initialization only  */
	MAX30003_ECG_InitStart(&max30003_obj_0,
			0b0, 0b1, 0b1, POL, CALP_SEL, CALN_SEL,
			E_FIT, max30003_obj_0.odr, GAIN, DHPF, DLPF);

#else

	/* ECG stream initialization only  */
	MAX30003_ECG_InitStart(&max30003_obj_0,
			0b0, OPENP, OPENN, POL, CALP_SEL, CALN_SEL,
			E_FIT, max30003_obj_0.odr, GAIN, DHPF, DLPF);

#endif


    /* choose default values for en_int register */
    memset(&new_vals, 0, sizeof(new_vals));
    new_vals.en_int.en_eint    = ENINT_ENABLED;
    new_vals.en_int.en_eovf    = ENEOVF_ENABLED;
    new_vals.en_int.intb_type  = INTBTYPE_NMOS_WITH_PU;
    ecg_set_en_int(_pobj_0,new_vals.en_int, EN_INT_DEFAULT_MASK);
    ecg_get_en_int(_pobj_0,&old_vals.en_int);
    if ( new_vals.en_int.en_eint   != old_vals.en_int.en_eint &&
    	 new_vals.en_int.en_eovf   != old_vals.en_int.en_eovf &&
         new_vals.en_int.intb_type != old_vals.en_int.intb_type ) {
         return -1;
    }
    vTaskDelay(100);

    /* choose default values for en_int register */
    memset(&new_vals, 0, sizeof(new_vals));
    ecg_set_en_int2(_pobj_0, new_vals.en_int, EN_INT2_DEFAULT_MASK);

	/* configure interrupts */
	MAX30003_IRQHandler_Config();

	/* dump registers */
	MAX30003_RegDump(&max30003_obj_0);

	max30003_obj_0.is_initialized=true;

    char * strN="\r\n[MAX30003] dev init done \r\n";
    aux_sendToAux(strN,strlen(strN),0,1,DBG_AUX);

	return rv;
}/* End of MAX30003_dev_init */


/**
 * @brief MAX30003_dev_ECG_start_stream
 * @brief max300001 start stream
 * @return 0 success
 */
int MAX30003_dev_ECG_start_stream(void)
{
	int ret = SYS_ERROR_NONE;

	if( _pobj_0)
	{
		/* start ecg stream */
		ret=MAX30003_Start_ECG(_pobj_0);

		/* sync with max30003 */
		MAX30003_synch(_pobj_0);
	}
	else
	{
		char * res_buff="ECG streaming is active\n";
		aux_sendToAux(res_buff,strlen(res_buff),0,1,DBG_AUX);
	}

    return ret;
}/* End of MAX30001_dev_ECG_start_stream */

/**
 * @brief MAX30003_dev_ECG_stop_stream
 * @brief max300001 stop stream
 * @return 0 success
 */
int MAX30003_dev_ECG_stop_stream(void)
{
	int ret = SYS_ERROR_NONE;

	if(_pobj_0)
	{
		/* stop ecg stream */
		MAX30003_Stop_ECG(_pobj_0);

		/* reset max30003 fifo */
		MAX30003_fifo_rst(_pobj_0);

		/* sync with max30003 */
		MAX30003_synch(_pobj_0);
	}

    return  ret;
}/* End of ECG_stream_stop */

/**
 * @brief MAX30003_dev_ECG_handle_stream
 * @brief MAX30003_dev_ECG_proc_stream process stream
 * @return 0 success
 */
int MAX30003_dev_ECG_handle_stream(void * p)
{
	int8_t return_value = 0;
	MAX30003_STATUS_VALS sval={0};

	if(/*is_streaming==true &&*/ _pobj_0)
	{
        /* read current status register */
		ecg_get_status(_pobj_0,&sval);

		/* FIFO over flow invokes a FIFO reset */
		if (sval.eovf == 1)
		{
			/* Do a FIFO Reset*/
			MAX30003_fifo_rst(&max30003_obj_0);
			return 0;
		}

		if (sval.eint == 1 ||  sval.rrint == 1)
		{
			uint8_t result[32 * 3]; ///< 32words - 3bytes each
			uint8_t data_array[4];
			int32_t success = 0;
			uint32_t total_databytes;
			uint8_t i_index=0,data_chunk=0,loop_logic=1, etag=0, ptag=0;
			int i, j;
			fifo_t *ecg_fifo;

			data_array[0] = ((REG_ECG_FIFO_BURST << 1) & 0xff) | 1;
			ecg_fifo=CHAN_GetChannelfifo(CHAN_ECG_V);


			acc_rec_len_max3003+=(EFIT + 1);

			/* insert time stamp in the queue */
			if (fifo_put32(ecg_fifo,acc_rec_len_max3003) == -1) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
			/* insert data length in the queue */
			if (fifo_put32(ecg_fifo,EFIT + 1) == -1) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

			/* to do use configure value */
			total_databytes = (EFIT + 1) * 3;
			while (loop_logic)
			{
				if (total_databytes > 30)
				{
					data_chunk = 30;
					total_databytes = total_databytes - 30;
				}
				else
				{
					data_chunk = total_databytes;
					loop_logic = 0;
				}

				/* The extra 1 byte is for the extra byte
				 * that comes out of the SPI
				 * copy of the FIFO over here... */
				HAL_GPIO_WritePin(_pobj_0->IO.csPort,_pobj_0->IO.csPin.Pin, GPIO_PIN_RESET);
				success = _pobj_0->Ctx.write_reg(_pobj_0->Ctx.handle,&data_array[0], 1, &result[i_index], (data_chunk + 1));
				HAL_GPIO_WritePin(_pobj_0->IO.csPort, _pobj_0->IO.csPin.Pin, GPIO_PIN_SET);
				if (success != 0){
					return -1;
				}

				/* This is important, because every
				 * transaction above creates an empty
				 * redundant data at result[0] */
				for (j = i_index; j < (data_chunk + i_index); j++) /* get rid of the 1 extra byte by moving the whole array up one */{
					result[j] = result[j + 1];
				}

				/* point to the next array location to put the data in */
				i_index = i_index + 30;
			}

			/* Put the content of the FIFO based on the EFIT value,
			 * We ignore the result[0] and
			 * start concatenating indexes: 1,2,3 - 4,5,6 - 7,8,9 */
			for (i = 0, j = 0; i < EFIT + 1; i++, j = j + 3)
			{
				etag = (0b00111000 & result[j + 2]) >> 3;
				ptag = 0b00000111 & result[j + 2];

				/* unexpected value */
				if(ptag != 0b111) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

				/* FIFO overflow error */
				if (etag == 0b111) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

				/* bad data fast mode sample or last fast mode sample */
				if(etag == 0b001 || etag == 0b011) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

				#if DEBUG_ENV_SAW_MAX3

					static uint16_t saw_tooth_tmp=0;
					saw_tooth_tmp+=100;

					/* put in queue and throw etag and ptag data out */
					if (fifo_put32(ecg_fifo, (int)(saw_tooth_tmp)) == -1) {
						Error_Handler((uint8_t *)__FILE__, __LINE__);
					}

				#else

					/* index1=23-16 bit, index2=15-8 bit, index3=7-0 bit */
					uint32_t tmp = ((uint32_t)result[j] << 16) + (result[j + 1] << 8) + result[j + 2];

					/* put in queue and throw etag and ptag data out
					 * and 2 LSB bits*/
					if (fifo_put32(ecg_fifo, ((int)(tmp)) >> 8) == -1) {
						Error_Handler((uint8_t *)__FILE__, __LINE__);
					}

				#endif

			} /* End of if (global_status.bit.eint == 1 || global_status.bit.pint == 1)*/
		}

		/* ECG/BIOZ DC Lead Off test */
		if (sval.dcloffint == 1) {
			//Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		if (sval.lonint == 1) ///< ECG Lead ON test: i.e. the leads are touching the body...
		{
			//Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}

	return return_value;
}/* End of MAX30003_dev_ECG_handle_stream */

/**
 * @brief MAX30003_dev_ECG_deinit_sream
 * @brief max300001 device de-initialization function
 * @return 0 success
 */
int MAX30003_dev_ECG_deinit_sream(void)
{
	int ret = SYS_ERROR_NONE;

	/* ========= Sensor de-initialization ========= */

	//TODO -turn off to save power

	max30003_obj_0.is_initialized=false;

	/* ========= SPI bus initialization =========== */

	if (max30003_obj_0.IO.DeInit == NULL)
	{
		ret = MAX30003_ERROR;
	}
	else if (max30003_obj_0.IO.DeInit() != MAX30003_OK)
	{
		ret = MAX30003_ERROR;
	}

	return ret;
}/* End of MAX30003_dev_ECG_deinit_sream */

