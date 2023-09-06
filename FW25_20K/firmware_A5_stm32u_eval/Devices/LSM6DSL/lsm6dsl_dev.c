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
#include <freertos.h>
#include <task.h>
#include <queue.h>

#include "stm32u5xx_hal.h"

#include "lsm6dsl_dev.h"
#include "bus.h"
#include "channel_manager_task.h"
#include "main.h"
#include "rtc.h"

#define CMD_BUFF_SIZE	        sizeof(CHAN_Proc_Stream_Vals_t)
#define N_CMD_BUFFERS	        (40)

static int16_t raw_acceleration[DEF_ACC_AXE][DEF_ODR_ACC];
static int16_t raw_gyro[DEF_ACC_AXE][DEF_ODR_GYR];

extern QueueHandle_t chanmngq;

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

LSM6DSL_Object_t lsm6dsl_obj;
LSM6DSL_Object_t * plsm6dsl_obj;

static MEMBUF_POOL dev_cmdPool;
static int LSM6DSL_init(LSM6DSL_Object_t * pinst);
static axis3bit16_t data_raw;

uint32_t acc_rec_len_gyro=0;
uint32_t acc_rec_len_acc=0;


static int32_t LSM6DSL_ACC_Set_ODR(LSM6DSL_Object_t *pObj, float Odr)
{
	lsm6dsl_odr_xl_t new_odr;
	new_odr = (Odr <=   12.5f) ? LSM6DSL_XL_ODR_12Hz5
			: (Odr <=   26.0f) ? LSM6DSL_XL_ODR_26Hz
			: (Odr <=   52.0f) ? LSM6DSL_XL_ODR_52Hz
			: (Odr <=  104.0f) ? LSM6DSL_XL_ODR_104Hz
			: (Odr <=  208.0f) ? LSM6DSL_XL_ODR_208Hz
			: (Odr <=  416.0f) ? LSM6DSL_XL_ODR_416Hz
			: (Odr <=  833.0f) ? LSM6DSL_XL_ODR_833Hz
			: (Odr <= 1660.0f) ? LSM6DSL_XL_ODR_1k66Hz
			: (Odr <= 3330.0f) ? LSM6DSL_XL_ODR_3k33Hz
			:                    LSM6DSL_XL_ODR_6k66Hz;

	/* Output data rate selection. */
	if (lsm6dsl_xl_data_rate_set(&(pObj->Ctx), new_odr) != LSM6DSL_OK)
	{
		return LSM6DSL_ERROR;
	}

return LSM6DSL_OK;
}

static int32_t LSM6DSL_GYR_Set_ODR(LSM6DSL_Object_t *pObj, float Odr)
{
	lsm6dsl_odr_g_t new_odr;
	new_odr = (Odr <=   12.5f) ? LSM6DSL_GY_ODR_12Hz5
			: (Odr <=   26.0f) ? LSM6DSL_GY_ODR_26Hz
			: (Odr <=   52.0f) ? LSM6DSL_GY_ODR_52Hz
			: (Odr <=  104.0f) ? LSM6DSL_GY_ODR_104Hz
			: (Odr <=  208.0f) ? LSM6DSL_GY_ODR_208Hz
			: (Odr <=  416.0f) ? LSM6DSL_GY_ODR_416Hz
			: (Odr <=  833.0f) ? LSM6DSL_GY_ODR_833Hz
			: (Odr <= 1660.0f) ? LSM6DSL_GY_ODR_1k66Hz
			: (Odr <= 3330.0f) ? LSM6DSL_GY_ODR_3k33Hz
			:                    LSM6DSL_GY_ODR_6k66Hz;

	/* Output data rate selection. */
	if (lsm6dsl_gy_data_rate_set(&(pObj->Ctx), new_odr) != LSM6DSL_OK)
	{
		return LSM6DSL_ERROR;
	}

  return LSM6DSL_OK;
}

/**
  * @brief  LSM6DSL_init
  * @param  pinst    sensor instance object
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int LSM6DSL_init(LSM6DSL_Object_t * pinst)
{
	if(pinst->acc_is_enabled || pinst->gyro_is_enabled)
	{
		static uint8_t rst;

		/* Restore default configuration */
		lsm6dsl_reset_set(&(pinst->Ctx), PROPERTY_ENABLE);

		do {
			lsm6dsl_reset_get(&(pinst->Ctx), &rst);
		} while (rst);




		if(pinst->acc_is_enabled)//3D ACC enabled
		{
			/* set xl power mode */
			lsm6dsl_xl_power_mode_set(&(pinst->Ctx),
					LSM6DSL_XL_HIGH_PERFORMANCE);

			 /* Set XL full scale full scale */
			lsm6dsl_xl_full_scale_set(
					&(pinst->Ctx), LSM6DSL_4g);
		}

		if(pinst->gyro_is_enabled)
		{
			/* set GYRO power mode */
			lsm6dsl_gy_power_mode_set(&(pinst->Ctx),
					LSM6DSL_GY_HIGH_PERFORMANCE);

			/* Set GYRO full scale full scale */
			lsm6dsl_gy_full_scale_set(
					&(pinst->Ctx), LSM6DSL_250dps);
		}


		/* Enable Block Data Update (BDU) when FIFO support selected */
		lsm6dsl_block_data_update_set(
				&(pinst->Ctx), PROPERTY_ENABLE);
		/* Set FIFO mode to Stream mode */
		lsm6dsl_fifo_mode_set(
				&(pinst->Ctx), LSM6DSL_BYPASS_MODE);

		if(pinst->acc_is_enabled)//3D ACC enabled
		{
			/* Set FIFO sensor decimator */
			lsm6dsl_fifo_xl_batch_set(
					&(pinst->Ctx), LSM6DSL_FIFO_XL_NO_DEC);
		}
		else //3D ACC disabled
		{
			/* Set FIFO sensor decimator */
			lsm6dsl_fifo_xl_batch_set(
					&(pinst->Ctx), LSM6DSL_FIFO_XL_DISABLE);
		}

		if(pinst->gyro_is_enabled)//3D GYRO enabled
		{
			/* Set FIFO sensor decimator */
			lsm6dsl_fifo_gy_batch_set(
					&(pinst->Ctx), LSM6DSL_FIFO_GY_NO_DEC);
		}
		else //3D GYRO disabled
		{
			/* Set FIFO sensor decimator */
			lsm6dsl_fifo_gy_batch_set(
					&(pinst->Ctx), LSM6DSL_FIFO_GY_DISABLE);
		}

		/* Set ODR FIFO */
		LSM6DSL_FIFO_Set_ODR_Value(pinst, DEF_ODR_ACC);

		if(pinst->acc_is_enabled)//3D ACC enabled
		{
			/* Set XL Output Data Rate*/
			LSM6DSL_ACC_Set_ODR(pinst, DEF_ODR_ACC);
			/* Set XL Output Data Rate*/
			lsm6dsl_xl_lp1_bandwidth_set(&(pinst->Ctx),
					LSM6DSL_XL_LP1_ODR_DIV_2);
		}
		else //3D ACC disabled
		{
			lsm6dsl_xl_data_rate_set(
					&(pinst->Ctx),LSM6DSL_XL_ODR_OFF);
		}

		if(pinst->gyro_is_enabled)//3D GYRO enabled
		{
			/* Set XL Output Data Rate*/
			LSM6DSL_GYR_Set_ODR(pinst, DEF_ODR_GYR);
		}
		else //3D GYRO disabled
		{
			/* Set Gyro in power down mode */
			lsm6dsl_gy_data_rate_set(
					&(pinst->Ctx), LSM6DSL_GY_ODR_OFF);
		}

	}
	else
	{
		/* todo enter power safe mode */
		lsm6dsl_xl_power_mode_set(&(pinst->Ctx),
				LSM6DSL_XL_NORMAL);
	}

    /* initialization complete */
	pinst->is_initialized = 1;
	plsm6dsl_obj= pinst;

  return LSM6DSL_OK;
}/* End of LSM6DSL_init */

/**
 * @brief LSM6DSL_dev_ACC_init_sream
 * @brief device/stream init function
 * @return 0 success
 */
int LSM6DSL_dev_ACC_init_sream(void)
{
	LSM6DSL_IO_t            io_ctx;
	static void * p=NULL;

	if(!p)
	{
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

		/* ========= I2C bus initialization =========== */

		/* Configure the i2c driver */
		io_ctx.Address     = LSM6DSL_I2C_ADD_L;
		io_ctx.Init        = BUS_I2C2_Init;
		io_ctx.DeInit      = BUS_I2C2_DeInit;
		io_ctx.ReadReg     = BUS_I2C2_ReadReg;
		io_ctx.WriteReg    = BUS_I2C2_WriteReg;
		io_ctx.GetTick     = BUS_GetTick;
		if (LSM6DSL_RegisterBusIO(&lsm6dsl_obj, &io_ctx) != LSM6DSL_OK)
		{
		  return SYS_ERROR_BUS_FAILURE;
		}

		/* wait for bus will be ready before reading */
		while(BUS_I2C2_IsReady(LSM6DSL_I2C_ADD_L, 10000)){
			vTaskDelay(25);
		}
	}

	/* ========= IMU sensor initialization =========== */
	lsm6dsl_obj.acc_is_enabled=true;

	/* sensor initialization */
	if (LSM6DSL_init(&(lsm6dsl_obj)) != LSM6DSL_OK)
	{
	    char * str="\r\n[LSM6DSL] FAILED \r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	    return SYS_ERROR_DEVICE_FAILURE;
	}

	/* ================ channel manager init ================= */

	/* fetch fifo handler of pulse channel */
	if(NULL == (plsm6dsl_obj->imu_fifo_acc=CHAN_GetChannelfifo(CHAN_ACC3X)))
	{
	    char * str="\r\n[LSM6DSL] CHAN_GetChannelfifo FAILED \r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	    return SYS_ERROR_DEVICE_FAILURE;
	}

	return SYS_ERROR_NONE;
}/* End of LIS2DW_dev_init_sream */


/**
 * @brief LSM6DSL_dev_ACC_deinit_sream
 * @brief device/stream init function
 * @return 0 success
 */
int LSM6DSL_dev_ACC_deinit_sream(void)
{
	/* ========= IMU sensor initialization =========== */
	if(lsm6dsl_obj.acc_is_enabled)
	{
		lsm6dsl_obj.acc_is_enabled=false;
		/* sensor initialization */
		if (LSM6DSL_init(&(lsm6dsl_obj)) != LSM6DSL_OK)
		{
			char * str="\r\n[LSM6DSL] FAILED \r\n";
			aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
			return SYS_ERROR_DEVICE_FAILURE;
		}
	}

	return SYS_ERROR_NONE;
}/* End of LIS2DW_dev_init_sream */

/**
 * @brief LSM6DSL_dev_GYR_init_sream
 * @brief device/stream init function
 * @return 0 success
 */
int LSM6DSL_dev_GYR_init_sream(void)
{
	LSM6DSL_IO_t            io_ctx;
	static void * p=NULL;

    if(!p)
    {
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

		/* ========= I2C bus initialization =========== */

		/* Configure the i2c driver */
		io_ctx.Address     = LSM6DSL_I2C_ADD_L;
		io_ctx.Init        = BUS_I2C2_Init;
		io_ctx.DeInit      = BUS_I2C2_DeInit;
		io_ctx.ReadReg     = BUS_I2C2_ReadReg;
		io_ctx.WriteReg    = BUS_I2C2_WriteReg;
		io_ctx.GetTick     = BUS_GetTick;
		if (LSM6DSL_RegisterBusIO(&lsm6dsl_obj, &io_ctx) != LSM6DSL_OK)
		{
		  return SYS_ERROR_BUS_FAILURE;
		}

		/* wait for bus will be ready before reading */
		while(BUS_I2C2_IsReady(LSM6DSL_I2C_ADD_L, 10000)){
			vTaskDelay(25);
		}
    }

	/* ========= IMU sensor initialization =========== */
	lsm6dsl_obj.gyro_is_enabled=true;

	/* sensor initialization */
	if (LSM6DSL_init(&(lsm6dsl_obj)) != LSM6DSL_OK)
	{
	    char * str="\r\n[LSM6DSL] FAILED \r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	    return SYS_ERROR_DEVICE_FAILURE;
	}

	/* ================ channel manager init ================= */

	/* fetch fifo handler of pulse channel */
	if(NULL == (plsm6dsl_obj->imu_fifo_gyr=CHAN_GetChannelfifo(CHAN_GYR3X)))
	{
	    char * str="\r\n[LSM6DSL GYR] CHAN_GetChannelfifo FAILED \r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	    return SYS_ERROR_DEVICE_FAILURE;
	}

	return SYS_ERROR_NONE;
}/* End of LSM6DSL_dev_GYR_init_sream */

/**
 * @brief LSM6DSL_dev_GYR_deinit_sream
 * @brief device/stream init function
 * @return 0 success
 */
int LSM6DSL_dev_GYR_deinit_sream(void)
{
	/* ========= IMU sensor de-initialization =========== */
	if(lsm6dsl_obj.gyro_is_enabled)
	{
		lsm6dsl_obj.gyro_is_enabled=false;

		/* sensor initialization */
		if (LSM6DSL_init(&(lsm6dsl_obj)) != LSM6DSL_OK)
		{
			char * str="\r\n[LSM6DSL] FAILED \r\n";
			aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
			return SYS_ERROR_DEVICE_FAILURE;
		}
	}

	return SYS_ERROR_NONE;
}/* End of LIS2DW_dev_init_sream */

/**
 * @brief LSM6DSL_dev_start_stream
 * @brief LSM6DSL start stream
 * @return 0 success
 */
int LSM6DSL_dev_ACC_start_stream(void)
{
	if(plsm6dsl_obj && plsm6dsl_obj->acc_is_enabled &&
			 plsm6dsl_obj->is_initialized)
	{
		sys_param.cf.is_imu_acc = 1;
		lsm6dsl_fifo_mode_set(
				&(lsm6dsl_obj.Ctx), LSM6DSL_STREAM_MODE);
	}

    return SYS_ERROR_NONE;
}/* End of LSM6DSL_dev_start_stream */

/**
 * @brief LSM6DSL_dev_stop_stream
 * @brief LSM6DSL stop stream
 * @return 0 success
 */
int LSM6DSL_dev_ACC_stop_stream(void)
{
	if(plsm6dsl_obj && plsm6dsl_obj->acc_is_enabled &&
			 plsm6dsl_obj->is_initialized)
	{
		/* todo IRQ */
		sys_param.cf.is_imu_acc = 0;
		/* Set FIFO mode to Stream mode */
		lsm6dsl_fifo_mode_set(
				&(lsm6dsl_obj.Ctx), LSM6DSL_BYPASS_MODE);
	}

    return  SYS_ERROR_NONE;
}/* End of LIS2DW_dev_stop_stream */

int LSM6DSL_dev_GYR_start_stream(void)
{
	if(plsm6dsl_obj && plsm6dsl_obj->is_initialized &&
			plsm6dsl_obj->gyro_is_enabled)
	{
		sys_param.cf.is_imu_gyr = 1;
		lsm6dsl_fifo_mode_set(
				&(lsm6dsl_obj.Ctx), LSM6DSL_STREAM_MODE);
	}

    return SYS_ERROR_NONE;
}/* End of LSM6DSL_dev_GYR_start_stream */

/**
 * @brief LSM6DSL_dev_stop_stream
 * @brief LSM6DSL stop stream
 * @return 0 success
 */
int LSM6DSL_dev_GYR_stop_stream(void)
{
	if(plsm6dsl_obj && plsm6dsl_obj->is_initialized &&
			plsm6dsl_obj->gyro_is_enabled)
	{
		/* todo IRQ */
		sys_param.cf.is_imu_gyr = 0;
		/* Set FIFO mode to Stream mode */
		lsm6dsl_fifo_mode_set(
				&(lsm6dsl_obj.Ctx), LSM6DSL_BYPASS_MODE);
	}

    return  SYS_ERROR_NONE;
}/* End of LIS2DW_dev_stop_stream */

/**
 * @brief LSM6DSL_dev_ACC_proc_stream
 * @brief LSM6DSL process stream
 * @return 0 success
 */
int LSM6DSL_dev_proc_stream(void)
{
	if(sys_param.cf.is_imu_acc || sys_param.cf.is_imu_gyr)
	{
		CHAN_Proc_Stream_Vals_t * buff=NULL;
		/* get fresh command buffer */
		if(!(buff=(CHAN_Proc_Stream_Vals_t*)getMemBuf(&dev_cmdPool))){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}


		/* fetch time stamp and other fields */
		buff->ts=0;//RTC_GetTimestampMillis();
		buff->id=CHAN_ACC3X;
		buff->num=0;

		/* send to channel manager command from ISR */
		if (pdPASS!=sendToChannelManagerFromISR(chanmngq, buff,
				CHAN_CMD_PROC_STEAM, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_DBRX,MSG_TYPE_CMD))){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}

	return  SYS_ERROR_NONE;
}/* End of LSM6DSL_dev_ACC_proc_stream */

/**
 * @brief LSM6DSL_dev_GYR_proc_stream
 * @brief LSM6DSL process stream
 * @return 0 success
 */
int LSM6DSL_dev_GYR_proc_stream(void)
{
	if(sys_param.cf.is_imu_gyr)
	{
		CHAN_Proc_Stream_Vals_t * buff=NULL;
		/* get fresh command buffer */
		if(!(buff=(CHAN_Proc_Stream_Vals_t*)getMemBuf(&dev_cmdPool))){
			Error_Handler((uint8_t *)__FILE__, __LINE__);;
		}

		/* fetch time stamp and other fields */
		buff->ts=0;//RTC_GetTimestampMillis();
		buff->id=CHAN_GYR3X;
		buff->num=0;

		/* send to channel manager command from ISR */
		if (pdPASS!=sendToChannelManagerFromISR(chanmngq, buff,
				CHAN_CMD_PROC_STEAM, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_DBRX,MSG_TYPE_CMD))){
			Error_Handler((uint8_t *)__FILE__, __LINE__);;
		}
	}

	return  SYS_ERROR_NONE;
}/* End of LSM6DSL_dev_GYR_proc_stream */


/**
 * @brief LSM6DSL_dev_handle_stream
 * @brief https://github.com/torvalds/linux/blob/master/drivers/iio/imu/st_lsm6dsx/st_lsm6dsx_buffer.c
 * @return 0 success
 */
int LSM6DSL_dev_handle_stream(void * p)
{
	fifo_t *imu_fifo=NULL;
	uint16_t num = 0;

	if(!(sys_param.cf.is_imu_acc && sys_param.cf.is_imu_gyr))
	{
		uint16_t len=DEF_ODR_ACC;
		int i=0;

		/* Read number of word in FIFO */
		lsm6dsl_fifo_data_level_get(&(lsm6dsl_obj.Ctx), &num);
		len=num/3;

		if(sys_param.cf.is_imu_acc)
		{
			imu_fifo=plsm6dsl_obj->imu_fifo_acc;
		}
		else
		{
			imu_fifo=plsm6dsl_obj->imu_fifo_gyr;
		}



		/* collect samples in the global buffer */
		for (int i=0; i<len; i++)
		{
			if(lsm6dsl_fifo_raw_data_get(&(lsm6dsl_obj.Ctx),data_raw.u8bit,
								                       3*sizeof(int16_t))){
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

			raw_acceleration[0][i]=data_raw.i16bit[0];//x
			raw_acceleration[1][i]=data_raw.i16bit[1];//y
			raw_acceleration[2][i]=data_raw.i16bit[2];//z
		}/* for (int i=0; i<count; i++) */

		#if  MODULE_DEBUG

			char dbug[200]={0};
			if(plsm6dsl_obj->acc_is_enabled)
			{
				sprintf(dbug,"[%d][ACC] X [%d] Y [%d] Z [%d] \r\n",num,
						raw_acceleration[0][0],
						raw_acceleration[1][0],
						raw_acceleration[2][0]);
			}
			else
			{
				sprintf(dbug,"[%d][GYRO] X [%d] Y [%d] Z [%d] \r\n",num,
						raw_acceleration[0][0],
						raw_acceleration[1][0],
						raw_acceleration[2][0]);
			}

			/* send to debug aux channel */
			aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

		#endif

		/* insert time stamp in the queue */
		acc_rec_len_acc+=num;

		if (-1 == fifo_put32(imu_fifo,acc_rec_len_acc)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* insert number of samples collected in the queue */
		if (-1 == fifo_put32(imu_fifo,len*3)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		for ( i=0; i<len; i++)//inject X
		{
			/* insert sample X acceleration in the queue */
			if (-1 == fifo_put32(imu_fifo,
					raw_acceleration[0][i])) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}/* for (int i=0; i<count; i++) */
		for ( i=0; i<len; i++)
		{
			/* insert sample X acceleration in the queue */
			if (-1 == fifo_put32(imu_fifo,
					raw_acceleration[1][i])) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}/* for (int i=0; i<count; i++) */
		for ( i=0; i<len; i++)//inject Z
		{
			/* insert sample X acceleration in the queue */
			if (-1 == fifo_put32(imu_fifo,
					raw_acceleration[2][i])) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}/* for (int i=0; i<count; i++) */
	}
	else
	{
		uint16_t len=DEF_ODR_ACC;
		int i=0;

		/* Read number of word in FIFO */
		lsm6dsl_fifo_data_level_get(&(lsm6dsl_obj.Ctx), &num);
		len=num/6;

		/* collect samples in the global buffer */
		for (i=0; i<len; i++)
		{
			/* ======== gyro data reading ========================== */
			if(lsm6dsl_fifo_raw_data_get(&(lsm6dsl_obj.Ctx), data_raw.u8bit,
								                        3*sizeof(int16_t))){
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
			raw_gyro[0][i]=data_raw.i16bit[0];//x
			raw_gyro[1][i]=data_raw.i16bit[1];//y
			raw_gyro[2][i]=data_raw.i16bit[2];//z

			/* ======== accelerometer data reading ================= */
			if(lsm6dsl_fifo_raw_data_get(&(lsm6dsl_obj.Ctx),data_raw.u8bit,
								                       3*sizeof(int16_t))){
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

			raw_acceleration[0][i]=data_raw.i16bit[0];//x
			raw_acceleration[1][i]=data_raw.i16bit[1];//y
			raw_acceleration[2][i]=data_raw.i16bit[2];//z


#if  MODULE_DEBUG

			char dbug[200]={0};

			sprintf(dbug,"[%d][GYRO]X[%d]Y[%d]Z[%d] [ACC]X[%d]Y[%d]Z[%d]\r\n",num,
					raw_gyro[0][i],
					raw_gyro[1][i],
					raw_gyro[2][i],
					raw_acceleration[0][i],
					raw_acceleration[1][i],
					raw_acceleration[2][i]);

			/* send to debug aux channel */
			aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

#endif

		}/* for (int i=0; i<count; i++) */



		fifo_t *imu_fifo_a=plsm6dsl_obj->imu_fifo_acc;
		fifo_t *imu_fifo_g=plsm6dsl_obj->imu_fifo_gyr;

		//acc data handling================================================
		acc_rec_len_acc+=(num>>1);
		/* insert time stamp in the queue */
		if (-1 == fifo_put32(imu_fifo_a,acc_rec_len_acc)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* insert number of samples collected in the queue */
		if (-1 == fifo_put32(imu_fifo_a,len*3)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}


		for ( i=0; i<len; i++)//inject X
		{
			/* insert sample X acceleration in the queue */
			if (-1 == fifo_put32(imu_fifo_a,
					raw_acceleration[0][i])) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}/* for (int i=0; i<count; i++) */
		for ( i=0; i<len; i++)
		{
			/* insert sample X acceleration in the queue */
			if (-1 == fifo_put32(imu_fifo_a,
					raw_acceleration[1][i])) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}/* for (int i=0; i<count; i++) */
		for ( i=0; i<len; i++)//inject Z
		{
			/* insert sample X acceleration in the queue */
			if (-1 == fifo_put32(imu_fifo_a,
					raw_acceleration[2][i])) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}/* for (int i=0; i<count; i++) */

		//gyro data handling================================================

		acc_rec_len_gyro+=(num>>1);

		/* insert time stamp in the queue */
		if (-1 == fifo_put32(imu_fifo_g,acc_rec_len_gyro)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* insert number of samples collected in the queue */
		if (-1 == fifo_put32(imu_fifo_g,len*3)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}


		for ( i=0; i<len; i++)//inject X
		{
			/* insert sample X acceleration in the queue */
			if (-1 == fifo_put32(imu_fifo_g,
					raw_gyro[0][i])) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}/* for (int i=0; i<count; i++) */
		for ( i=0; i<len; i++)
		{
			/* insert sample X acceleration in the queue */
			if (-1 == fifo_put32(imu_fifo_g,
					raw_gyro[1][i])) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}/* for (int i=0; i<count; i++) */
		for ( i=0; i<len; i++)//inject Z
		{
			/* insert sample X acceleration in the queue */
			if (-1 == fifo_put32(imu_fifo_g,
					raw_gyro[2][i])) {
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}
		}/* for (int i=0; i<count; i++) */

	}


return 0;
}/* End of LSM6DSL_dev_ECG_handle_stream */



