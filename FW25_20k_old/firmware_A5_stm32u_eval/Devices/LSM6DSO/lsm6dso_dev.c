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
#include "main.h"

#include <stdint.h>
#include <freertos.h>
#include <task.h>
#include <queue.h>

#include "channel_manager_task.h"
#include "stm32u5xx_hal.h"
#include "lsm6dso_dev.h"
#include "bus.h"
#include "rtc.h"

#include "sys_errno.h"

LSM6DSO_Object_t lsm6dso_obj_0;

#if 0
#define FIFO_tsh      (uint32_t)(16)
#define ODR_value     (uint32_t)(128)
#define PeriodValue   (uint32_t)(256 - 1)
#define PulseValue    (uint32_t)((PeriodValue + 1)/2 - 1)
#define PeriodicVal   (uint32_t)(PeriodValue * 16)

#define Lptim3_it     (0)
#define mod_debug     (0)//1- enable 0 - disable

LIS2DW12_Object_t lis2dw12_obj_0;
LIS2DW12_Object_t lis2dw12_obj_1;
LPTIM_HandleTypeDef hlptim3;
LPTIM_HandleTypeDef hlptim1;
LPTIM_OC_ConfigTypeDef sConfig = {.OCPolarity = LPTIM_OCPOLARITY_LOW,.Pulse=PulseValue};

static fifo_t *pulse_fifo=NULL;
static uint32_t pulse_ts=0;
static int16_t data_raw_acceleration_e1[1];
static int16_t data_raw_acceleration_e2[1];


static int LIS2DW_sensor_init(LIS2DW12_Object_t * pinst);
static int LIS2DW_LPTIM3_Init(void);
static int LIS2DW_LPTIM1_Init(void);
static void HAL_LPTIM_ARMatchCallback(LPTIM_HandleTypeDef *hlptim);

/**
  * @brief  Autoreload match callback in non blocking mode
  * @param  hlptim : LPTIM handle
  * @retval None
  */
void HAL_LPTIM_ARMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	pulse_ts=0;//RTC_GetTimestampMillis();

	/* insert time stamp in the queue */

	if (-1 == fifo_put32(pulse_fifo,kaka)){
		//Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* insert data length in the queue */
	if (-1 == fifo_put32(pulse_fifo,FIFO_tsh*2)){
		//Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* insert sample in the queue */
	for (int i=0; i<FIFO_tsh; i++)
	{
		/* Read  Z acceleration data e2  */
		if(lis2dw12_acceleration_Z_raw_get(&(lis2dw12_obj_0.Ctx), data_raw_acceleration_e1)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* Read  Z acceleration data e1  */
		if(lis2dw12_acceleration_Z_raw_get(&(lis2dw12_obj_1.Ctx), data_raw_acceleration_e2)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* insert sample e1 in the queue */
	    if (-1 == fifo_put32(pulse_fifo, data_raw_acceleration_e1[0])) {
	    	//Error_Handler((uint8_t *)__FILE__, __LINE__);
	    }

		/* insert sample e2 in the queue */
	    if (-1 == fifo_put32(pulse_fifo, data_raw_acceleration_e2[0])) {
	    	//Error_Handler((uint8_t *)__FILE__, __LINE__);
	    }
	}/* for (int i=0; i<FIFO_tsh; i++) */

#if mod_debug == 1

	if(tgl == 0)
	{
		HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_PIN, GPIO_PIN_SET); //debug pin high
		tgl=1;
		lis2dw12_fifo_ovr_flag_get(&(lis2dw12_obj_0.Ctx), &reg_val);
		if(reg_val){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}
	else
	{
		HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_PIN, GPIO_PIN_RESET); //debug pin low
		tgl=0;
		lis2dw12_fifo_ovr_flag_get(&(lis2dw12_obj_1.Ctx), &reg_val);
		if(reg_val){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}

#endif

}/* End of HAL_LPTIM_ARMatchCallback */


/**
 * @brief LIS2DW_dev_init
 * @brief lis2dw init function
 * @return 0 success
 */
int LIS2DW_dev_init_sream(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	LIS2DW12_IO_t            io_ctx_0;
	LIS2DW12_IO_t            io_ctx_1;

	/* ========= sync IO initialization =========== */

	LIS2DW_XL_SYNC_GPIO_CLK_ENABLE();

	GPIO_InitStruct.Pin   = LIS2DW_XL_SYNC_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = LIS2DW_XL_SYNC_SPEED;

	HAL_GPIO_Init(LIS2DW_XL_SYNC_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(LIS2DW_XL_SYNC_GPIO_PORT, LIS2DW_XL_SYNC_PIN, GPIO_PIN_RESET);


	/* ========= I2C bus initialization =========== */

	/* Configure the accelerometer #0 driver */
	io_ctx_0.BusType     = LIS2DW12_I2C_BUS; /* I2C */
	io_ctx_0.Address     = LIS2DW12_I2C_ADD_H;
	io_ctx_0.Init        = BUS_I2C2_Init;
	io_ctx_0.DeInit      = BUS_I2C2_DeInit;
	io_ctx_0.ReadReg     = BUS_I2C2_ReadReg;
	io_ctx_0.WriteReg    = BUS_I2C2_WriteReg;
	io_ctx_0.GetTick     = BUS_GetTick;

	/* Configure the accelerometer #1 driver */
	io_ctx_1.BusType     = LIS2DW12_I2C_BUS; /* I2C */
	io_ctx_1.Address     = LIS2DW12_I2C_ADD_L;
	io_ctx_1.Init        = BUS_I2C2_Init;
	io_ctx_1.DeInit      = BUS_I2C2_DeInit;
	io_ctx_1.ReadReg     = BUS_I2C2_ReadReg;
	io_ctx_1.WriteReg    = BUS_I2C2_WriteReg;
	io_ctx_1.GetTick     = BUS_GetTick;

	vTaskDelay(25);

	/* wait for bus will be ready before reading */
	if (LIS2DW12_RegisterBusIO(&lis2dw12_obj_0, &io_ctx_0) != LIS2DW12_OK)
	{
	  aux_dbg_printLogStr("\r\n[LIS2DW] id 0 BUS FAILED \r\n");
	  return SYS_ERROR_DEVICE_FAILURE;
	}

	/* wait for bus will be ready before reading */
	if (LIS2DW12_RegisterBusIO(&lis2dw12_obj_1, &io_ctx_1) != LIS2DW12_OK)
	{
	  aux_dbg_printLogStr("\r\n[LIS2DW] id 1 BUS FAILED \r\n");
	  return SYS_ERROR_DEVICE_FAILURE;
	}

	/* wait for bus will be ready before reading */
	while(BUS_I2C2_IsReady(LIS2DW12_I2C_ADD_L, 10000)){
	  vTaskDelay(25);
	}

	/* wait for bus will be ready before reading */
	while(BUS_I2C2_IsReady(LIS2DW12_I2C_ADD_H, 10000)){
	  vTaskDelay(25);
	}

	/* ========= E1 and E2 sensors initialization =========== */

	/* id 0 sensor initialization */
	if (LIS2DW_sensor_init(&(lis2dw12_obj_0)) != LIS2DW12_OK)
	{
	  aux_dbg_printLogStr("\r\n[LIS2DW] id 0 FAILED \r\n");
	  return SYS_ERROR_DEVICE_FAILURE;
	}

	/* id 1 sensor initialization */
	if (LIS2DW_sensor_init(&(lis2dw12_obj_1)) != LIS2DW12_OK)
	{
	  aux_dbg_printLogStr("\r\n[LIS2DW] id 1 FAILED \r\n");
	  return SYS_ERROR_DEVICE_FAILURE;
	}

	/* ================ channel manager init ================= */

	/* fetch fifo handler of pulse channel */
	if(NULL == (pulse_fifo=CHAN_GetChannelfifo(CHAN_PULSE)))
	{
	  aux_dbg_printLogStr("\r\n[LIS2DW] CHAN_GetChannelfifo FAILED \r\n");
	  return SYS_ERROR_DEVICE_FAILURE;
	}


	return 0;
}/* End of LIS2DW_dev_init_sream */



/**
 * @brief LIS2DW_dev_start_stream

	The counter clock is LSE (32.768 KHz), Autoreload equal to 254 so the output
	frequency (FrequencyOutput) will be equal to 128

		FrequencyOutput = Counter Clock Frequency / (Autoreload + 1)

						= 32768 / 256
						= 128 Hz

	Pulse value equal to 49 and the duty cycle (DutyCycle) is computed as follow:

	DutyCycle = 1 - ((PulseValue + 1)/ (Autoreload + 1))
	DutyCycle = 50%

 * @brief lis2dw start stream
 * @return 0 success
 */
int LIS2DW_dev_PULSE_start_stream(void)
{
	/* initiate lptim 3 instance */
	LIS2DW_LPTIM3_Init();

	/* initiate lptim 1 instance */
	LIS2DW_LPTIM1_Init();

	/* configure lptim3 channel 1 */
	if (HAL_LPTIM_OC_ConfigChannel(&hlptim3, &sConfig, LPTIM_CHANNEL_1) != HAL_OK){
		return -1;
	}

#if (Lptim3_it == 1)

	/* start pwm generation in interrupt mode lptim3 channel 1 */
	if (HAL_LPTIM_PWM_Start_IT(&hlptim3, LPTIM_CHANNEL_1) != HAL_OK){
		return -1;
	}

#else

	/* start periodic irq routine using lptim1 */
	if (HAL_LPTIM_Counter_Start_IT(&hlptim1) != HAL_OK)
	{
		return -1;
	}

	/* start pwm generation in background mode lptim3 channel 1 */
	if (HAL_LPTIM_PWM_Start(&hlptim3, LPTIM_CHANNEL_1) != HAL_OK){
		return -1;
	}

#endif

    return 0;
}/* End of LIS2DW_dev_start_stream */



/**
 * @brief LIS2DW_dev_stop_stream
 * @brief lis2dw stop stream
 * @return 0 success
 */
int LIS2DW_dev_PULSE_stop_stream(void)
{
	/* start periodic irq routine using lptim1 */
	if (HAL_LPTIM_Counter_Stop_IT(&hlptim1) != HAL_OK)
	{
		return -1;
	}

	/* start pwm generation in background mode lptim3 channel 1 */
	if (HAL_LPTIM_PWM_Stop(&hlptim3, LPTIM_CHANNEL_1) != HAL_OK){
		return -1;
	}

    return  0;
}/* End of LIS2DW_dev_stop_stream */


/**
  * @brief  LIS2DW init
  * @param  pinst    sensor instance object
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int LIS2DW_sensor_init(LIS2DW12_Object_t * pinst)
{
	static uint8_t id_cnt=0;
	char res_buff[50]={0};
	uint8_t val=0,rst=1;


	/* Restore default configuration */
	lis2dw12_reset_set(&(pinst->Ctx), PROPERTY_ENABLE);

	do {
		lis2dw12_reset_get(&(pinst->Ctx), &rst);
	} while (rst);


	/* =====  read sensor revision register ======== */

	if (LIS2DW12_ReadID(pinst, &val) != LIS2DW12_OK)
	{
	  sprintf(res_buff,"\r\n[BIT LIS2DW] id %d FAILED \r\n",id_cnt);
	  aux_dbg_printLogStr((const char *)res_buff);
	}


    /* check revision acc */
	if (val != LIS2DW12_ID){
		sprintf(res_buff,"\r\n[BIT LIS2DW] id [%d] FAILED \r\n",id_cnt);
		aux_dbg_printLogStr((const char *)res_buff);
		return -1;
	}


    /* ========= sensor initialization ============ */

	/* Enable Block Data Update */
	if(lis2dw12_block_data_update_set(&(pinst->Ctx), PROPERTY_ENABLE)){
		return -1;
	}
	/* Set full scale */
	if(lis2dw12_full_scale_set(&(pinst->Ctx), LIS2DW12_2g)){
		return -1;
	}

	/* FIFO_CTRL(2Eh): STREAM mode enabled*/
	if(lis2dw12_fifo_mode_set(&(pinst->Ctx), LIS2DW12_STREAM_MODE)){
		return -1;
	}

	if(lis2dw12_fifo_watermark_set(&(pinst->Ctx), 15)){
		return -1;
	}

	/* ====== Single data conversion enable ======== */

	/*  Enable Single data controlled by INT2 Set ODR 50Hz, Single data mode */
	if(lis2dw12_data_rate_set(&(pinst->Ctx),  LIS2DW12_XL_SET_PIN_TRIG)){
		return -1;
	}

	/* ====== start sensor operation =============== */

	/* Configure power mode */
	if(lis2dw12_power_mode_set(&(pinst->Ctx), LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2)){
		return -1;
	}

	/* Settling time ( 1 sample, i.e. 1/ODR ) */
	vTaskDelay(1/ODR_value);

	id_cnt++;

  return 0;

}/* End of LIS2DW_sensor_init */



/**
  * @brief LPTIM1 Initialization Function
  *
  * used for PWM generation
  *
  * @param None
  * @retval 0-OK
  */
int LIS2DW_LPTIM3_Init(void)
{
	HAL_StatusTypeDef rv = HAL_OK;

	hlptim3.Instance               = LPTIM3;
	hlptim3.Init.Clock.Source      = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	hlptim3.Init.Clock.Prescaler   = LPTIM_PRESCALER_DIV1;
	hlptim3.Init.Trigger.Source    = LPTIM_TRIGSOURCE_SOFTWARE;
	hlptim3.Init.Period            = PeriodValue;
	hlptim3.Init.UpdateMode        = LPTIM_UPDATE_IMMEDIATE;
	hlptim3.Init.CounterSource     = LPTIM_COUNTERSOURCE_INTERNAL;
	hlptim3.Init.Input1Source      = LPTIM_INPUT1SOURCE_GPIO;
	hlptim3.Init.Input2Source      = LPTIM_INPUT2SOURCE_GPIO;
	hlptim3.Init.RepetitionCounter = 0;

	/* lptim3 initialization */
	rv = HAL_LPTIM_Init(&hlptim3);
	if (rv != HAL_OK)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

#if Lptim3_it == 1

	/* get event irq handler */
  HAL_LPTIM_RegisterCallback(&hlptim3,HAL_LPTIM_AUTORELOAD_MATCH_CB_ID,HAL_LPTIM_ARMatchCallback);

  /* set interrupt priority grouping */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* set interrupt priority */
  HAL_NVIC_SetPriority(LPTIM3_IRQn, 15, 1);

  /* set interrupt priority */
  HAL_NVIC_EnableIRQ(LPTIM3_IRQn);

#endif

  return 0;

}/* End of LIS2DW_LPTIM3_Init */


/**
  * @brief LPTIM1 Initialization Function
  *
  * used for periodic interrupt
  *
  * @param None
  * @retval 0 ok
  */
int LIS2DW_LPTIM1_Init(void)
{
  hlptim1.Instance               = LPTIM1;
  hlptim1.Init.Clock.Source      = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler   = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source    = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.Period            = PeriodicVal;
  hlptim1.Init.UpdateMode        = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource     = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source      = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source      = LPTIM_INPUT2SOURCE_GPIO;
  hlptim1.Init.RepetitionCounter = 0;

  /* lptim3 initialization */
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }


	/* get event irq handler */
  HAL_LPTIM_RegisterCallback(&hlptim1,HAL_LPTIM_AUTORELOAD_MATCH_CB_ID,HAL_LPTIM_ARMatchCallback);

  /* set interrupt priority grouping */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* set interrupt priority */
  HAL_NVIC_SetPriority(LPTIM1_IRQn, 15, 1);

  /* set interrupt priority */
  HAL_NVIC_EnableIRQ(LPTIM1_IRQn);


  return 0;

}/* End of LIS2DW_LPTIM1_Init */



#if 0

int LIS2DW_dev_PULSE_start_stream(void)
{


	/* ========= lptim initialization ============ */

	LPTIM3_Init();

	/* configure lptim3 channel 1 */
	if (HAL_LPTIM_OC_ConfigChannel(&hlptim1, &sConfig, LPTIM_CHANNEL_1) != HAL_OK){
		return -1;
	}

//    /* ========= sensor initialization ============ */
//
//	/* Enable Block Data Update */
//	if(lis2dw12_block_data_update_set(&(lis2dw12_obj_0.Ctx), PROPERTY_ENABLE)){
//		return -1;
//	}
//	/* Set full scale */
//	if(lis2dw12_full_scale_set(&(lis2dw12_obj_0.Ctx), LIS2DW12_2g)){
//		return -1;
//	}
//
//	/* FIFO_CTRL(2Eh): STREAM mode enabled*/
//	if(lis2dw12_fifo_mode_set(&(lis2dw12_obj_0.Ctx), LIS2DW12_STREAM_MODE)){
//		return -1;
//	}
//
//	if(lis2dw12_fifo_watermark_set(&(lis2dw12_obj_0.Ctx), 31)){
//		return -1;
//	}
//
//	/* ====== Single data conversion enable ======== */
//
//	/*  Enable Single data controlled by INT2 Set ODR 50Hz, Single data mode */
//	if(lis2dw12_data_rate_set(&(lis2dw12_obj_0.Ctx),  LIS2DW12_XL_SET_PIN_TRIG)){
//		return -1;
//	}
//
//	/* ====== start sensor operation =============== */
//
//	/* Configure power mode */
//	if(lis2dw12_power_mode_set(&(lis2dw12_obj_0.Ctx), LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2)){
//		return -1;
//	}
//
//	/* Settling time ( 1 sample, i.e. 1/ODR ) */
//	vTaskDelay(1/128);

	/* start pwm generation lptim3 channel 1 */
	if (HAL_LPTIM_PWM_Start_IT(&hlptim3, LPTIM_CHANNEL_1) != HAL_OK){
		return -1;
	}

    return 0;
}/* End of LIS2DW_dev_start_stream */

//EXTI_HandleTypeDef hexti = {.Line = LIS2DW_XL_SYNC_EXTI_LINE};

#if 0
while(1)
{
	//if(irq_cnt < 31) continue;
	 if(!lis2dw12_status_reg_get(&(lis2dw12_obj_0.Ctx), &reg_status))
	 {
		 if(reg_status.fifo_ths)
		 {
			 irq_cnt=0;
				if(tgl == 0)
				{
					HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_PIN, GPIO_PIN_SET); //debug pin high
					tgl=1;
				}
				else
				{
					HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_PIN, GPIO_PIN_RESET); //debug pin low
					tgl=0;
				}

				lis2dw12_fifo_data_level_get(&(lis2dw12_obj_0.Ctx), &val);
				for (i = 0; i < val; i++)
				{
					/* Read acceleration data */
					memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
					lis2dw12_acceleration_raw_get(&(lis2dw12_obj_0.Ctx), data_raw_acceleration);




					//	sprintf((char *)tx_buffer,
					//	"Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
					//  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
					//	tx_com(tx_buffer, strlen((char const *)tx_buffer));
				}
		 }
	 }



	//lis2dw12_fifo_wtm_flag_get(&(lis2dw12_obj_0.Ctx), &val);
	if(val)
	{
		if(tgl == 0)
		{
			HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_PIN, GPIO_PIN_SET); //debug pin high
			tgl=1;
		}
		else
		{
			HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_PIN, GPIO_PIN_RESET); //debug pin low
			tgl=0;
		}

		lis2dw12_fifo_data_level_get(&(lis2dw12_obj_0.Ctx), &val);
		for (i = 0; i < val; i++)
		{
			/* Read acceleration data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			lis2dw12_acceleration_raw_get(&(lis2dw12_obj_0.Ctx), data_raw_acceleration);




			//	sprintf((char *)tx_buffer,
			//	"Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
			//  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
			//	tx_com(tx_buffer, strlen((char const *)tx_buffer));
		}
	}
	else
	{
		continue;
	}

	irq_cnt=0;


}



//	  if (ret == 0) {
//	    ctrl3.slp_mode = ( (uint8_t) val & 0x30U ) >> 4;
//	    temp=*((uint32_t*)(&ctrl3));
//	    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &ctrl3, 1);
//






//	/* configure fifo th */
//	lis2dw12_fifo_watermark_set(&(lis2dw12_obj_0.Ctx), 31);
//
//	/* configure fifo mode */
//	lis2dw12_fifo_mode_set(&(lis2dw12_obj_0.Ctx), LIS2DW12_STREAM_MODE);




    return 0;
}/* End of LIS2DW_dev_start_stream */

#endif


/**
  * @brief  EXTI line for LIS2DW accelerometer cb  function
  * @param  None
  * @retval None
  */

static void LIS2DW12_int_pin_a_hardware_event_isr(void);
void LIS2DW12_int_pin_a_hardware_event_isr(void)
{
	lis2dw12_status_t status;

//	if (lis2dw12_status_reg_get(&(lis2dw12_obj_0.Ctx), &val) != LIS2DW12_OK)
//	{
//		return LIS2DW12_ERROR;
//	}

    /* Check Free Fall events */
    lis2dw12_all_sources_t src;
    lis2dw12_all_sources_get(&(lis2dw12_obj_0.Ctx), &src);




    uint8_t val, i;
    /* Read output only if new value is available */
    lis2dw12_fifo_wtm_flag_get(&(lis2dw12_obj_0.Ctx), &val);

    if (val) {
      lis2dw12_fifo_data_level_get(&(lis2dw12_obj_0.Ctx), &val);

      for (i = 0; i < val; i++) {
        /* Read acceleration data */
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        lis2dw12_acceleration_raw_get(&(lis2dw12_obj_0.Ctx), data_raw_acceleration);

//        acceleration_mg[0] = lis2dw12_from_fs2_to_mg(
//                               data_raw_acceleration[0]);
//        acceleration_mg[1] = lis2dw12_from_fs2_to_mg(
//                               data_raw_acceleration[1]);
//        acceleration_mg[2] = lis2dw12_from_fs2_to_mg(
//                               data_raw_acceleration[2]);
//        sprintf((char *)tx_buffer,
//                "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
//                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
//        tx_com(tx_buffer, strlen((char const *)tx_buffer));
      }
    }


}/* End of LIS2DW12_int_pin_a_hardware_event_isr */


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	lis2dw12_status_t val;

	if (lis2dw12_status_reg_get(&(lis2dw12_obj_0.Ctx), &val) != LIS2DW12_OK)
	{
		return LIS2DW12_ERROR;
	}

}/* HAL_GPIO_EXTI_Falling_Callback */

#if 0

/**
  * @brief  Configures EXTI line for LIS2DW accelerometer
  * @param  None
  * @retval None
  */
static void LIS2DW12_EXTI_IRQHandler_Config(void);
void LIS2DW12_EXTI_IRQHandler_Config(void)
{
	LIS2DW_XL_SYNC_GPIO_CLK_ENABLE();
	/* get event irq handler */
	HAL_EXTI_GetHandle(&hexti, LIS2DW_XL_SYNC_EXTI_LINE);

	/* register event irq handler */
	HAL_EXTI_RegisterCallback(&hexti, HAL_EXTI_COMMON_CB_ID, LIS2DW12_int_pin_a_hardware_event_isr);

	/* set interrupt priority grouping */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* set interrupt priority */
	HAL_NVIC_SetPriority(LIS2DW_XL_SYNC_EXTI_IRQn, 15, 1);

	/* set enable interrupt  */
	HAL_NVIC_EnableIRQ(LIS2DW_XL_SYNC_EXTI_IRQn);

	return;
}/* LIS2DW12_EXTI_IRQHandler_Config */



#endif




	/* Set INT1_FTH to ‘1’: Enable FIFO threshold interrupt */
	if (lis2dw12_pin_int2_route_get(&(lis2dw12_obj_0.Ctx), &val) != LIS2DW12_OK) { return LIS2DW12_ERROR; }
	val.int2_fth = PROPERTY_ENABLE;
	val.int2_ovr=PROPERTY_ENABLE;
	if (lis2dw12_pin_int2_route_set(&(lis2dw12_obj_0.Ctx), &val) != LIS2DW12_OK)
	{
		return LIS2DW12_ERROR;
	}

	/* set interrupt type */
	lis2dw12_int_notification_set(&(lis2dw12_obj_0.Ctx), LIS2DW12_);


    /* register irq routine */
	LIS2DW12_EXTI_IRQHandler_Config();

#endif
#endif

