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
#include <limits.h>
#include "Freertos.h"
#include <stdint.h>
#include <freertos.h>
#include <task.h>
#include <queue.h>
#include "stm32u5xx_hal.h"

#include "MAX3000N/MAX3000N.h"
#include "MAX30001_bus.h"
#include "MAX30001_dev.h"
#include "cmd_handler_task.h"
#include "bus.h"
#include "channel_manager_task.h"
#include "cmd_handler_task.h"
#include "main.h"
#include "rtc.h"

#define MOD_NAME "[MAX30001 DEV]"
#define MAX30001_DEB 1

#define MAX30001_BIOZ 1

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

// Init values for BIOZ_InitStart()
#define EN_BIOZ        0b0
#define BMUX_OPENP     OPENP_COMMON     //Open the BIP Input Switch <CNFG_BMUX Register>
#define BMUX_OPENN     OPENN_COMMON     //Open the BIN Input Switch <CNFG_BMUX Register>
#define BMUX_CALP_SEL  CALP_SEL_COMMON  //BIP Calibration Selection <CNFG_BMUX Register>
#define BMUX_CALN_SEL  CALN_SEL_COMMON  //BIN Calibration Selection <CNFG_BMUX Register>
#define BMUX_CG_MODE   0b00             //BIOZ Current Generator Mode Selection <CNFG_BMUX Register>
#define B_FIT      0b011 //000 to 111 = 1 to 8 <MNGR_INT Register>
#define CNFG_BIOZ_AHPF   0b010          //010 = 800HZ
#define CNFG_BIOZ_EXT_RBIAS   0b1       //1 =external Bias Generator
#define CNFG_BIOZ_BIOZ_GAIN   0b010     //10 = 40V/V
#define CNFG_BIOZ_BIOZ_DHPF   0b00
#define CNFG_BIOZ_BIOZ_DLPF   0b10
#define CNFG_BIOZ_FC_GEN      0b0010
#define CNFG_BIOZ_CGMON       0b0
#define CNFG_BIOZ_CGMAG       0b011
#define CNFG_BIOZ_PHOFF       0b0000

// Initn values for CAL_InitStart()
#define EN_VCAL    EN_VCAL_COMMON
#define VMODE      VMODE_COMMON
#define VMAG       VMAG_COMMON
#define FCAL       FCAL_COMMON
#define THIGH      THIGH_COMMON
#define FIFTY      FIFTY_COMMON

/// Initialization values for Rbias_FMSTR_Init()
#define EN_RBIAS 0b01
#define RBIASV   0b10
#define RBIASP   0b1
#define RBIASN   0b1
#define FMSTR    0b01

#define BUFFER_LENGTH           32
#define TESTING_TIMEOUT_SECONDS 3

extern QueueHandle_t chanmngq;
extern max30001_status_t global_status;

EXTI_HandleTypeDef hexti14;
EXTI_HandleTypeDef hexti15;

MAX30001_Object_t max30001_obj_0;
QueueHandle_t ecg_fifo=NULL;
MEMBUF_POOL ecgh_cmdPool;

uint32_t acc_rec_len_max3001=0;
uint32_t acc_rec_len_bioz=0;

static bool is_streaming = 0;
static uint32_t reset_req = 0;

static void MAX30001_IRQHandler_Config(void);
static void StreamPacketUint32_ecg(uint32_t id, uint32_t *buffer, uint32_t number,void * p);

#if MODULE_DEBUG
	char ecg_pbuff[50];
	//@brief Creating a buffer to hold the data
	uint32_t ecgBuffer[BUFFER_LENGTH];
	int ecgIndex = 0;
	char data_trigger = 0;
#endif


/**
 * @brief MAX30001_dev_vref_init
 * @brief max300001 device initialization function
 * @return 0 success
 */
int MAX30001_dev_vref_init(void)
{
	int rv = SYS_ERROR_NONE;

	if(max30001_obj_0.ecg_is_initialized==false)
	{
		/* wait for bus will be ready before reading */
		if (MAX30001_RegisterBusIO(&max30001_obj_0) != MAX30001_OK)
		{
		  return SYS_ERROR_DEVICE_FAILURE;
		}

		/*  configure ecg hpf frequency */
		io_ctl_set_hpf_freq(ecg_HPF_05HZ);

		if(0==reset_req++)
		{
			/*  Do a software reset of the MAX30001 */
			if(MAX30001_sw_rst(&max30001_obj_0)){
				return SYS_ERROR_DEVICE_FAILURE;
			}
		}

		/* get converted odr register value */
		max30001_obj_0.odr=conv_odr_val((uint16_t)DEF_ODR_ECG_H);

	#if defined(DEBUG_ENV_CAL_MAX1) && (DEBUG_ENV_CAL_MAX1)

		/* ECG signal simulation */
		MAX30001_CAL_InitStart(&max30001_obj_0,
				EN_VCAL , VMODE, VMAG, FCAL, THIGH, FIFTY);

		/* ECG stream initialization only  */
		MAX30001_ECG_InitStart(&max30001_obj_0,
				0b0, 0b1, 0b1, POL, CALP_SEL, CALN_SEL,
				E_FIT, max30001_obj_0.odr, GAIN, DHPF, DLPF);
	#else

		/* ECG stream initialization only  */
		MAX30001_ECG_InitStart(&max30001_obj_0,
				0b1, OPENP, OPENN, POL, CALP_SEL, CALN_SEL,
				E_FIT, max30001_obj_0.odr, GAIN, DHPF, DLPF);



		//MAX30001_BIOZ


	#endif

		char * str="\r\n[MAX30001] vref init done \r\n";
		aux_sendToAux(str,strlen(str),0,1,DBG_AUX);

		max30001_obj_0.ecg_is_initialized=true;
	}

	return rv;
}/* End of MAX30001_dev_init */


/**
 * @brief MAX30001_dev_ECG_init_sream
 * @brief max300001 device initialization function
 * @return 0 success
 */
int MAX30001_dev_ECG_init_sream(void)
{
	int rv = SYS_ERROR_NONE;

	/* wait for bus will be ready before reading */
	if (MAX30001_RegisterBusIO(&max30001_obj_0) != MAX30001_OK)
	{
	  return SYS_ERROR_DEVICE_FAILURE;
	}

	/* ========= Sensor initialization =========== */
#if defined(DEBUG_ENV_CAL_MAX1) && (DEBUG_ENV_CAL_MAX1)

	uint32_t id=0xFF;
	int partVersion=0xFF;/* 0 = 30004  1 = 30001  2 = 30002  3 = 30003 */

	MAX30001_readID(&max30001_obj_0,&id);
	partVersion = id >> 12;
	partVersion = partVersion & 0x3;
	/* Version validation */
	if (partVersion != 1){
	    char * str="\r\n[FATAL][MAX30001] not detected \r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	    //return SYS_ERROR_BUS_FAILURE;
	}

#endif

	/*  configure ecg hpf frequency */
	io_ctl_set_hpf_freq(ecg_HPF_05HZ);


	if(0==reset_req++)
	{
		/*  Do a software reset of the MAX30001 */
		if(MAX30001_sw_rst(&max30001_obj_0)){
			return SYS_ERROR_DEVICE_FAILURE;
		}
	}

	/* get converted odr register value */
	max30001_obj_0.odr=conv_odr_val((uint16_t)DEF_ODR_ECG_H);

#if defined(DEBUG_ENV_CAL_MAX1) && (DEBUG_ENV_CAL_MAX1)

	/* ECG signal simulation */
	MAX30001_CAL_InitStart(&max30001_obj_0,
			EN_VCAL , VMODE, VMAG, FCAL, THIGH, FIFTY);

	/* ECG stream initialization only  */
	MAX30001_ECG_InitStart(&max30001_obj_0,
			0b0, 0b1, 0b1, POL, CALP_SEL, CALN_SEL,
			E_FIT, max30001_obj_0.odr, GAIN, DHPF, DLPF);
#else

	/* ECG stream initialization only  */
	MAX30001_ECG_InitStart(&max30001_obj_0,
			0b0, OPENP, OPENN, POL, CALP_SEL, CALN_SEL,
			E_FIT, max30001_obj_0.odr, GAIN, DHPF, DLPF);

#endif


	/* interrupt pins assignment Max30001 */
	if(MAX30001_INT_ECG_assignment(&max30001_obj_0,
			                MAX30001_INT_B,    MAX30001_NO_INT,   MAX30001_NO_INT,  //  en_enint_loc,      en_eovf_loc,   en_fstint_loc,
							MAX30001_INT_2B,   MAX30001_NO_INT,  //  en_dcloffint_loc,    en_bovf_loc,
							MAX30001_INT_2B,   MAX30001_INT_2B,   MAX30001_NO_INT,  //  en_bover_loc,      en_bundr_loc,  en_bcgmon_loc,
							MAX30001_INT_B,    MAX30001_NO_INT,   MAX30001_NO_INT,  //  en_pint_loc,       en_povf_loc,   en_pedge_loc,
							MAX30001_INT_2B,   MAX30001_INT_B,    MAX30001_NO_INT,  //  en_lonint_loc,     en_rrint_loc,  en_samp_loc,
							MAX30001_INT_ODNR, MAX30001_INT_ODNR))                  //  intb_Type,         int2b_Type)
	{
		return SYS_ERROR_DEVICE_FAILURE;
	}

	MAX30001_onDataAvailable(&StreamPacketUint32_ecg);

	/* configure interrupts */
	MAX30001_IRQHandler_Config();

	/* dump registers */
	MAX30001_RegDump(&max30001_obj_0);

    char * str="\r\n[MAX30001] dev init done \r\n";
    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);

    max30001_obj_0.ecg_is_initialized=true;

	return rv;
}/* End of MAX30001_dev_init */

/**
 * @brief MAX30001_dev_ECG_deinit_sream
 * @brief max300001 device de-initialization function
 * @return 0 success
 */
int MAX30001_dev_ECG_deinit_sream(void)
{
	int ret = SYS_ERROR_NONE;

//	/* TODO ========= SPI bus initialization =========== */
//	if(true == max30001_obj_0.ecg_is_initialized)
//	{
//		if (max30001_obj_0.IO.DeInit == NULL)
//		{
//			ret = MAX30001_ERROR;
//		}
//		else if (max30001_obj_0.IO.DeInit() != MAX30001_OK)
//		{
//			ret = MAX30001_ERROR;
//		}
//	}
//
	max30001_obj_0.ecg_is_initialized=false;

	return ret;
}/* End of MAX30001_dev_ECG_deinit_sream */

/**
 * @brief MAX30001_dev_ECG_start_stream
 * @brief max300001 start stream
 * @return 0 success
 */

int MAX30001_dev_ECG_start_stream(void)
{
	int ret = SYS_ERROR_NONE;

	ret=MAX30001_Start_ECG(&max30001_obj_0);

	/* sync with max30001 */
	MAX30001_synch(&max30001_obj_0);

	is_streaming=ret==SYS_ERROR_NONE?true:false;

	max30001_obj_0.ecg_is_initialized=true;

	char * res_buff="ECG streaming is active\n";
	aux_sendToAux(res_buff,strlen(res_buff),0,1,DBG_AUX);

    return ret;
}/* End of MAX30001_dev_ECG_start_stream */

/**
 * @brief MAX30001_dev_ECG_stop_stream
 * @brief max300001 stop stream
 * @return 0 success
 */
int MAX30001_dev_ECG_stop_stream(void)
{
	int ret = SYS_ERROR_NONE;

	/* stop ecg stream */
	ret=MAX30001_Stop_ECG(&max30001_obj_0);

	/* reset max30001 fifo */
	MAX30001_fifo_rst(&max30001_obj_0);

	/* sync with max30001 */
	MAX30001_synch(&max30001_obj_0);

	is_streaming=false;

    return  ret;
}/* End of ECG_stream_stop */

/**
 * @brief MAX30001_dev_handle_stream
 * @brief MAX30001_dev_ECG_proc_stream process stream
 * @return 0 success
 */
int MAX30001_dev_handle_stream(void * p)
{
	  int8_t return_value = 0;
	  max30001_obj_0.smpl_ts=((CHAN_Proc_Stream_Vals_t*)p)->ts;

#if MODULE_DEBUG
	sprintf(ecg_pbuff,"\r\n ECG_handle_stream IN [%d] \r\n",(int)max30001_obj_0.smpl_ts);
	aux_sendToAux(ecg_pbuff,strlen(ecg_pbuff),0,1,DBG_AUX);
#endif

	  MAX30001_get_status(&max30001_obj_0,&global_status.all);

	  /* Inital Reset and any FIFO over flow invokes a FIFO reset */
	  if (global_status.bit.eovf == 1 || global_status.bit.bovf == 1 || global_status.bit.povf == 1)
	  {
	    ///< Do a FIFO Reset
		MAX30001_fifo_rst(&max30001_obj_0);
	  }

	  ///< The four data handling goes on over here
	  if (global_status.bit.eint == 1 || global_status.bit.pint == 1 || global_status.bit.bint == 1 || global_status.bit.rrint == 1) {
	    return_value = return_value | MAX30001_FIFO_LeadONOff_Read(&max30001_obj_0);
	  }

	  ///< ECG/BIOZ DC Lead Off test
	  if (global_status.bit.dcloffint == 1) {
	    return_value = return_value | MAX30001_FIFO_LeadONOff_Read(&max30001_obj_0);
	  }

	  ///< BIOZ AC Lead Off test
	  if (global_status.bit.bover == 1 || global_status.bit.bundr == 1) {
	    return_value = return_value | MAX30001_FIFO_LeadONOff_Read(&max30001_obj_0);
	  }

	  ///< BIOZ DRVP/N test using BCGMON.
	  if (global_status.bit.bcgmon == 1) {
	    return_value = return_value | MAX30001_FIFO_LeadONOff_Read(&max30001_obj_0);
	  }

	  if (global_status.bit.lonint == 1) ///< ECG Lead ON test: i.e. the leads are touching the body...
	  {
		  MAX30001_FIFO_LeadONOff_Read(&max30001_obj_0);
	  }

#if MODULE_DEBUG
	sprintf(ecg_pbuff,"\r\n ECG_handle_stream OUT  [%d] \r\n",(int)max30001_obj_0.smpl_ts);
	aux_sendToAux(ecg_pbuff,strlen(ecg_pbuff),0,1,DBG_AUX);
#endif


	  return return_value;
}/* End of MAX30001_dev_handle_stream */

/**
 * @brief ECG_proc_stream
 * @brief ECG_proc_stream process stream
 * @return 0 success
 */
static int ECG_proc_stream(uint32_t *buffer, uint32_t number,void * p)
{
	//MAX30001_Object_t * pobj=p;
	int i=0,status=0;
	uint8_t etag = 0;
	uint8_t ptag = 0;

	fifo_t *ecg_fifo=CHAN_GetChannelfifo(CHAN_ECG_H);
	//uint32_t ts=pobj->smpl_ts;

#if MODULE_DEBUG
	sprintf(ecg_pbuff,"\r\n ECG_proc_stream IN [%d] \r\n",(int)number);
	aux_sendToAux(ecg_pbuff,strlen(ecg_pbuff),0,1,DBG_AUX);
#endif

//	if(is_streaming==true )
//	{
	    acc_rec_len_max3001+=(number);

		/* insert time stamp in the queue */
		status = fifo_put32(ecg_fifo,acc_rec_len_max3001);
		if (status == -1) {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* insert data length in the queue */
		status = fifo_put32(ecg_fifo,number);
		if (status == -1) {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
//	}

	/* collect(inspect) data from the fifo */
	for (i=0; i<number; i++)
	{
		etag = (0b00111000 & buffer[i]) >> 3;
		ptag =  0b00000111 & buffer[i];

		/* unexpected value */
		if(ptag != 0b111) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

		/* FIFO overflow error */
		if (etag == 0b111) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

		/* bad data fast mode sample or last fast mode sample */
		if(etag == 0b001 || etag == 0b011) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

		/* insert sample in the queue:

			ECG FIFO Data Structure
			The data portion of the word contains the 18-bit ECG voltage information
			measured at the requested sample rate in left justified two’s complement format.
			The remaining six bits of data hold important data tagging information
			(see details in Table 48 and Table 49).
			After converting the data portion of the sample to signed magnitude format,
			the ECG input voltage is calculated by the following equation:
			VECG (mV) = ADC x VREF / (217 x ECG_GAIN)
			where:
			ADC = ADC counts in signed magnitude format, VREF = 1000mV (typ)
			(refer to the Electrical Characteristics section),
			and ECG_GAIN = 20V/V, 40V/V, 80V/V, or 160V/V, set in CNFG_ECG (0x15).

			ECG Sample Voltage Data [17:0]
			ETAG [2:0]
			PTAG [2:0]

		*/

#if DEBUG_ENV_SAW_MAX1

		static uint16_t tmp=0;
		tmp+=100;

		/* put in queue and throw etag and ptag data out */
		status = fifo_put32(ecg_fifo, ((int) (tmp)));
		if (status == -1) {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
#else

		/* put in queue and throw etag and ptag data out and 2 LSB bits */
		status = fifo_put32(ecg_fifo, ((int) (buffer[i])) >> 8/*3+3+2*/);
		if (status == -1) {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

#endif

	}/* for (i=0; i<number; i++) */

#if MODULE_DEBUG
	sprintf(ecg_pbuff,"\r\n ECG_proc_stream OUT [%d] \r\n",(int)number);
	aux_sendToAux(ecg_pbuff,strlen(ecg_pbuff),0,1,DBG_AUX);
#endif

	return  SYS_ERROR_NONE;
}/* End of MAX30001_dev_ECG_proc_stream */


/**
* @brief common interrupt handler
*/
int32_t MAX30001_int_handler(MAX30001_Object_t *pObj)
{
	CHAN_Proc_Stream_Vals_t * buff=NULL;
	int32_t return_value = 0;

	/* get fresh command buffer */
	if(!(buff=(CHAN_Proc_Stream_Vals_t*)getMemBuf(&ecgh_cmdPool))){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* fetch time stamp and other fields */
	buff->ts=0;//RTC_GetTimestampMillis();
	buff->id=CHAN_ECG_H;
	buff->num=0;

	/* send to channel manager command from ISR */
	if (pdPASS!=sendToChannelManagerFromISR(chanmngq, buff,
			CHAN_CMD_PROC_STEAM, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_DBRX,MSG_TYPE_CMD))){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

  return return_value;
}/* end of MAX30001_int_handler */

/**
* @brief Preventive measure used to dismiss interrupts that fire too early during
* @brief initialization on INTB line
*
*/
void MAX30001_Mid_IntB_Handler(void){
  MAX30001_int_handler(&max30001_obj_0);
}/* end of MAX30001_int_handler */

/**
* @brief Preventive measure used to dismiss interrupts that fire too early during
* @brief initialization on INT2B line
*/
void MAX30001_Mid_Int2B_Handler(void){
	MAX30001_int_handler(&max30001_obj_0);
}/* end of MAX30001_Mid_Int2B_Handler */

/**
  * @brief  Configures EXTI line (connected to PE.14,15 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void MAX30001_IRQHandler_Config(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_ConfigTypeDef EXTI_ConfigStructure;

	/* Enable GPIOC clock */
	MAX30001_INT_GPIO_CLK_ENABLE();

	/* Configure PE.14 pin as input floating */
	GPIO_InitStructure.Mode  = MAX30001_INT_MODE;
	GPIO_InitStructure.Speed = MAX30001_INT_SPEED;
	GPIO_InitStructure.Pull  = MAX30001_INT_PULL;
	GPIO_InitStructure.Pin   = MAX30001_INT_PIN;
	HAL_GPIO_Init(MAX30001_INT_GPIO_PORT, &GPIO_InitStructure);

	/* Configure PE.15 pin as input floating */
	GPIO_InitStructure.Mode  = MAX30001_INT2_MODE;
	GPIO_InitStructure.Pull  = MAX30001_INT2_PULL;
	GPIO_InitStructure.Pin   = MAX30001_INT2_PIN;
	GPIO_InitStructure.Speed = MAX30001_INT2_SPEED;
	HAL_GPIO_Init(MAX30001_INT2_GPIO_PORT, &GPIO_InitStructure);

	/* Set configuration except Interrupt and Event mask of Exti line 14,15*/
	EXTI_ConfigStructure.Line    = EXTI_LINE_14;
	EXTI_ConfigStructure.Trigger = EXTI_TRIGGER_FALLING;
	EXTI_ConfigStructure.GPIOSel = EXTI_GPIOE;
	EXTI_ConfigStructure.Mode    = EXTI_MODE_INTERRUPT;
	HAL_EXTI_SetConfigLine(&hexti14, &EXTI_ConfigStructure);

	/* get event irq handler */
	if(HAL_OK != HAL_EXTI_RegisterCallback(&hexti14, HAL_EXTI_FALLING_CB_ID,MAX30001_Mid_IntB_Handler))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* Set configuration except Interrupt and Event mask of Exti line 14,15*/
	EXTI_ConfigStructure.Line    = EXTI_LINE_15;
	EXTI_ConfigStructure.Trigger = EXTI_TRIGGER_FALLING;
	EXTI_ConfigStructure.GPIOSel = EXTI_GPIOE;
	EXTI_ConfigStructure.Mode    = EXTI_MODE_INTERRUPT;
	HAL_EXTI_SetConfigLine(&hexti15, &EXTI_ConfigStructure);

	/* get event irq handler */
	if(HAL_OK != HAL_EXTI_RegisterCallback(&hexti15, HAL_EXTI_FALLING_CB_ID,MAX30001_Mid_Int2B_Handler))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* set interrupt priority grouping */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* set interrupt priority */
	HAL_NVIC_SetPriority(EXTI14_IRQn, 15, 1);

	/* set interrupt priority */
	HAL_NVIC_EnableIRQ(EXTI14_IRQn);

	/* set interrupt priority */
	HAL_NVIC_SetPriority(EXTI15_IRQn, 15, 1);

	/* set interrupt priority */
	HAL_NVIC_EnableIRQ(EXTI15_IRQn);

}/* EXTI_IRQHandler_Config */




/**
 * @brief MAX30001_dev_BIOZ_init_sream
 * @brief max300001 device initialization function
 * @return 0 success
 */
int MAX30001_dev_BIOZ_init_sream(void)
{
	int rv = SYS_ERROR_NONE;

	/* wait for bus will be ready before reading */
	if (MAX30001_RegisterBusIO(&max30001_obj_0) != MAX30001_OK)
	{
	  return SYS_ERROR_DEVICE_FAILURE;
	}

	/* ========= Sensor initialization =========== */
	if(0==reset_req++)
	{
		/*  Do a software reset of the MAX30001 */
		if(MAX30001_sw_rst(&max30001_obj_0)){
			return SYS_ERROR_DEVICE_FAILURE;
		}
	}

	/* get converted odr register value */
	max30001_obj_0.bioz_odr=conv_odr_bioz_val((uint16_t)DEF_ODR_BIOZ);

#if defined(DEBUG_BIOZ_MAX1) && (DEBUG_BIOZ_MAX1)

	/* ECG signal simulation */
	MAX30001_CAL_InitStart(&max30001_obj_0,
			EN_VCAL , VMODE, VMAG, FCAL, THIGH, FIFTY);

	/* ECG stream initialization only  */
	MAX30001_BIOZ_InitStart(&max30001_obj_0,
							EN_BIOZ,
							0b1,
							0b1,
							BMUX_CALP_SEL,
							BMUX_CALN_SEL,
							BMUX_CG_MODE,
							B_FIT,
							max30001_obj_0.bioz_odr,
							CNFG_BIOZ_AHPF,
							CNFG_BIOZ_EXT_RBIAS,
							CNFG_BIOZ_BIOZ_GAIN,
							CNFG_BIOZ_BIOZ_DHPF,
							CNFG_BIOZ_BIOZ_DLPF,
							CNFG_BIOZ_FC_GEN,
							CNFG_BIOZ_CGMON,
							CNFG_BIOZ_CGMAG,
							CNFG_BIOZ_PHOFF);
#else


	/* ECG stream initialization only  */
	MAX30001_BIOZ_InitStart(&max30001_obj_0,
							EN_BIOZ,
							BMUX_OPENP,
							BMUX_OPENN,
							BMUX_CALP_SEL,
							BMUX_CALN_SEL,
							BMUX_CG_MODE,
							B_FIT,
							max30001_obj_0.bioz_odr,
							CNFG_BIOZ_AHPF,
							CNFG_BIOZ_EXT_RBIAS,
							CNFG_BIOZ_BIOZ_GAIN,
							CNFG_BIOZ_BIOZ_DHPF,
							CNFG_BIOZ_BIOZ_DLPF,
							CNFG_BIOZ_FC_GEN,
							CNFG_BIOZ_CGMON,
							CNFG_BIOZ_CGMAG,
							CNFG_BIOZ_PHOFF);

#endif


	/* interrupt pins assignment Max30001 */
	if(MAX30001_BIOZ_INT_assignment(&max30001_obj_0,MAX30001_INT_2B))
	{
		return SYS_ERROR_DEVICE_FAILURE;
	}

	MAX30001_onDataAvailable(&StreamPacketUint32_ecg);

	/* configure interrupts */
	MAX30001_IRQHandler_Config();

	/* dump registers */
	MAX30001_RegDump(&max30001_obj_0);

    char * str="\r\n[MAX3001-BIOZ] dev init done \r\n";
    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);

    max30001_obj_0.biz_is_initialized=true;

	return rv;
}/* End of MAX30001_dev_init */


/**
 * @brief MAX30001_dev_ECG_deinit_sream
 * @brief max300001 device de-initialization function
 * @return 0 success
 */
int MAX30001_dev_BIOZ_deinit_sream(void)
{
	int ret = SYS_ERROR_NONE;

	/* ========= Sensor de-initialization ========= */

	//TODO -turn off to save power


	/* TODO ========= SPI bus initialization =========== */
//	if(true == max30001_obj_0.biz_is_initialized)
//	{
//		if (max30001_obj_0.IO.DeInit == NULL)
//		{
//			ret = MAX30001_ERROR;
//		}
//		else if (max30001_obj_0.IO.DeInit() != MAX30001_OK)
//		{
//			ret = MAX30001_ERROR;
//		}
//	}

	max30001_obj_0.biz_is_initialized=false;

	return ret;
}/* End of MAX30001_dev_ECG_deinit_sream */


/**
 * @brief MAX30001_dev_BIOZ_start_stream
 * @brief max300001 start stream
 * @return 0 success
 */

int MAX30001_dev_BIOZ_start_stream(void)
{
	int ret = SYS_ERROR_NONE;

	ret=MAX30001_Start_BIOZ(&max30001_obj_0);

	/* sync with max30001 */
	MAX30001_synch(&max30001_obj_0);

	is_streaming=ret==SYS_ERROR_NONE?true:false;

	max30001_obj_0.biz_is_initialized=true;

	char * res_buff="Bio-Z streaming is active\n";
	aux_sendToAux(res_buff,strlen(res_buff),0,1,DBG_AUX);

    return ret;
}/* End of MAX30001_dev_BIOZ_start_stream */

/**
 * @brief MAX30001_dev_BIOZ_stop_stream
 * @brief max300001 stop stream
 * @return 0 success
 */
int MAX30001_dev_BIOZ_stop_stream(void)
{
	int ret = SYS_ERROR_NONE;

	/* stop ecg stream */
	ret=MAX30001_Stop_BIOZ(&max30001_obj_0);

	/* reset max30001 fifo */
	MAX30001_fifo_rst(&max30001_obj_0);

	/* sync with max30001 */
	MAX30001_synch(&max30001_obj_0);

	is_streaming=false;

    return  ret;
}/* End of MAX30001_dev_BIOZ_stop_stream */

/**
 * @brief BIOZ_proc_stream
 * @brief ECG_proc_stream process stream
 * @return 0 success
 */
static int BIOZ_proc_stream(uint32_t *buffer, uint32_t number,void * p)
{
	//MAX30001_Object_t * pobj=p;
	int i=0,status=0;
	uint8_t btag = 0;


	fifo_t *bioz_fifo=CHAN_GetChannelfifo(CNAH_BIOZ);
	//uint32_t ts=pobj->smpl_ts;

#if MODULE_DEBUG
	sprintf(bioz_pbuff,"\r\n bioz_proc_stream IN [%d] \r\n",(int)number);
	aux_sendToAux(bioz_pbuff,strlen(bioz_pbuff),0,1,DBG_AUX);
#endif

//	if(is_streaming==true )
//	{
	    acc_rec_len_bioz+=(number);

		/* insert time stamp in the queue */
		status = fifo_put32(bioz_fifo,acc_rec_len_bioz);
		if (status == -1) {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* insert data length in the queue */
		status = fifo_put32(bioz_fifo,number);
		if (status == -1) {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
//	}

	/* collect(inspect) data from the fifo */
	for (i=0; i<number; i++)
	{
		btag =  0b00000111 & buffer[i];

		/* Over/Under Range Sample */
		if(btag == 0b001) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

		/* Last Valid Sample (EOF) */
		//if(btag == 0b010) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

		/* Last Over/Under Range Sample (EOF) */
		if(btag == 0b011) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

		/* FIFO Overflow (exception) */
		if(btag == 0b111) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

		/* FIFO Empty (exception) */
		if(btag == 0b110) { Error_Handler((uint8_t *)__FILE__, __LINE__); }

#if DEBUG_ENV_SAW_MAX1

		static uint16_t tmp=0;
		tmp+=100;

		/* put in queue and throw etag and ptag data out */
		status = fifo_put32(bioz_fifo, ((int) (tmp)));
		if (status == -1) {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
#else

		/* put in queue and throw 3bits btag data out and 5 LSB bits */
		status = fifo_put32(bioz_fifo, ((int) (buffer[i])) >> (1+3+4));
		if (status == -1) {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

#endif

	}/* for (i=0; i<number; i++) */

#if MODULE_DEBUG
	sprintf(bioz_pbuff,"\r\n bioz_proc_stream OUT [%d] \r\n",(int)number);
	aux_sendToAux(bioz_pbuff,strlen(bioz_pbuff),0,1,DBG_AUX);
#endif

	return  SYS_ERROR_NONE;
}/* End of BIOZ_proc_stream */


/**
  * @brief Creates a packet that will be streamed via specified channel
  * the packet created will be inserted into a fifo to be streamed at a later time

	Converting bioz Samples to Voltage
	ECG samples are recorded in 18-bit, left justified two’s
	compliment format.
	After converting to signed magnitude
	format, the ECG input voltage is calculated by the following equation:

	VECG (mV) = ADC x VREF / (217 x ECG_GAIN)

	ADC is the ADC counts in signed magnitude format, VREF
	is 1000mV (typ) (refer to the Electrical Characteristics
	section), and ECG_GAIN is 20V/V, 40V/V, 80V/V, or
	160V/V, set in CNFG_ECG (0x15).

  * @param id Streaming ID
  * @param buffer Pointer to a uint32 array that contains the data to include in the packet
  * @param number Number of elements in the buffer
  * @retval None
  */
void StreamPacketUint32_ecg(uint32_t id, uint32_t *buffer, uint32_t number, void *p)
{
	if (id == MAX30001_DATA_ECG)
	{
		/* process ecg stream buffer */
		ECG_proc_stream(buffer, number, p);
	}/* end of if (id == MAX30001_DATA_ECG) */

   if (id == MAX30001_DATA_BIOZ) {
		/* process bioz stream buffer */
		BIOZ_proc_stream(buffer, number, p);
   }

   if (id == MAX30001_DATA_PACE) {
         ///  Add code for reading Pace data
   }

   if (id == MAX30001_DATA_RTOR) {
         /// Add code for reading RtoR data
   }

}/* end of StreamPacketUint32_ecg */
