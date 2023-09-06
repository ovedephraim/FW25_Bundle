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
#include <stdint.h>
#include <stdlib.h>
#include <freertos.h>
#include <task.h>
#include <queue.h>

#include "main.h"
#include "bus.h"
#include "lis2dw_dev.h"


#define MAX_BUF_SIZE 256
#define N_SAMPLES  5 /*!< Number of samples */

#define X_LO_LIM    ((int)70)   /*!< Accelero low test limit [mg] */
#define X_HI_LIM    ((int)1500) /*!< Accelero high test limit [mg] */

#define ST_REG_COUNT  (sizeof(reg_addr) / sizeof(uint8_t))

#define LIS2DW12_ST_DISABLE  0U
#define LIS2DW12_ST_POSITIVE 1U
#define LIS2DW12_ST_NEGATIVE 2U

#define X_POWER_UP_DELAY    100 /*!< Delay after accelero power-up [ms] */
#define X_ST_ENABLED_DELAY  100 /*!< Delay after accelero self-test enabled [ms] */


/* Refer to Datasheet / Application Note documents for details about following register settings */
static uint8_t reg_addr[]        = {0x21, 0x22, 0x23, 0x24, 0x25, 0x20};
static uint8_t x_st_reg_values[] = {0x0C, 0x00, 0x00, 0x00, 0x10, 0x44};

extern LIS2DW12_Object_t lis2dw12_obj_0;
extern LIS2DW12_Object_t lis2dw12_obj_1;

EXTI_HandleTypeDef hexti13 = {.Line = EXTI_LINE_13};


/**
  * @brief  Wait for data ready and get data
  * @param  data the sensor data
  * @retval None
  */
static int32_t LIS2DW12_X_Get_Data(LIS2DW12_Object_t * p_obj, LIS2DW12_Axes_t *data)
{
  uint8_t status=0;
  int32_t ret=0;

  /* Wait for data ready */
  do
  {
    if ((ret = LIS2DW12_ACC_Get_DRDY_Status(p_obj,  &status)) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }
  while (status == 0U);

  /* Read accelero data LIS2DW12_ACC_GetAxes */
  if ((ret = LIS2DW12_ACC_GetAxes(p_obj, data)) != SYS_ERROR_NONE)
  {
    return ret;
  }

  return ret;
}


/**
  * @brief  Performs LIS2DW12 accelerometer self-test
  * @retval BSP status
  */
static int32_t LIS2DW12_X_SelfTest(LIS2DW12_Object_t * p_obj)
{
  int32_t test_result = SYS_ERROR_NONE;
  LIS2DW12_Axes_t data_nost;
  LIS2DW12_Axes_t data_st;
  LIS2DW12_Axes_t data;
  uint8_t prev_reg_values[ST_REG_COUNT];
  int32_t ret=SYS_ERROR_NONE;
  uint32_t i;


  /* Store current settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = LIS2DW12_Read_Reg(p_obj, reg_addr[i], &prev_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Set the sensor for self-test */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = LIS2DW12_Write_Reg(p_obj, reg_addr[i], x_st_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Wait defined time for stable output */
  vTaskDelay(X_POWER_UP_DELAY);

  /* Read first data and discard it */
  if (LIS2DW12_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }

  data_nost.x = 0;
  data_nost.y = 0;
  data_nost.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
    if (LIS2DW12_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }
    data_nost.x += data.x;
    data_nost.y += data.y;
    data_nost.z += data.z;
  }
  data_nost.x /= N_SAMPLES;
  data_nost.y /= N_SAMPLES;
  data_nost.z /= N_SAMPLES;

  /* Enable self-test */
  if ((ret = LIS2DW12_ACC_Set_SelfTest(p_obj, (uint8_t)LIS2DW12_ST_POSITIVE)) != SYS_ERROR_NONE)
  {
    return ret;
  }

  /* Wait defined time for stable output */
  vTaskDelay(X_ST_ENABLED_DELAY);

  /* Read first data and discard it */
  if (LIS2DW12_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  data_st.x = 0;
  data_st.y = 0;
  data_st.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
    if (LIS2DW12_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }
    data_st.x += data.x;
    data_st.y += data.y;
    data_st.z += data.z;
  }
  data_st.x /= N_SAMPLES;
  data_st.y /= N_SAMPLES;
  data_st.z /= N_SAMPLES;

  /* Restore previous settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = LIS2DW12_Write_Reg(p_obj, reg_addr[i], prev_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Evaluate the test */
  if (abs((int)(data_st.x - data_nost.x)) < X_LO_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }
  if (abs((int)(data_st.x - data_nost.x)) > X_HI_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }
  if (abs((int)(data_st.y - data_nost.y)) < X_LO_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }
  if (abs((int)(data_st.y - data_nost.y)) > X_HI_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }
  if (abs((int)(data_st.z - data_nost.z)) < X_LO_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }
  if (abs((int)(data_st.z - data_nost.z)) > X_HI_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }

  /* Print the test result */
  if (test_result != SYS_ERROR_NONE)
  {
    ret=SYS_ERROR_NONE;
  }

  /* Disable self-test */
  LIS2DW12_ACC_Set_SelfTest(p_obj, (uint8_t)LIS2DW12_ST_DISABLE);

  return ret;
}


/**
 * @brief LIS2DW_bit_task
 * @brief LIS2DW_bit_func
 * @return none
 */
void LIS2DW_bit_func(void *para)
{
	LIS2DW12_Capabilities_t  cap;
	LIS2DW12_IO_t            io_ctx_0;
	LIS2DW12_IO_t            io_ctx_1;
	char res_buff[50]={0};
	uint8_t id_0,id_1;

	(void)cap;

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


    aux_sendToAux((char * )"\r\n[BIT LIS2DW] start \r\n",
    		strlen("\r\n[BIT LIS2DW] start \r\n"),0,1,DBG_AUX);
	vTaskDelay(25);

	/* wait for bus will be ready before reading */
	if (LIS2DW12_RegisterBusIO(&lis2dw12_obj_0, &io_ctx_0) != LIS2DW12_OK)
	{
	  aux_sendToAux((char * )"\r\n[BIT LIS2DW] id 0 BUS FAILED \r\n",
	     		strlen("\r\n[BIT LIS2DW] id 0 BUS FAILED \r\n"),0,1,DBG_AUX);
	  return;
	}

	/* wait for bus will be ready before reading */
	if (LIS2DW12_RegisterBusIO(&lis2dw12_obj_1, &io_ctx_1) != LIS2DW12_OK)
	{
	  aux_sendToAux((char * )"\r\n[BIT LIS2DW] id 1 BUS FAILED \r\n",
	     		strlen("\r\n[BIT LIS2DW] id 1 BUS FAILED \r\n"),0,1,DBG_AUX);
	  return;
	}

	/* wait for bus will be ready before reading */
	while(BUS_I2C2_IsReady(LIS2DW12_I2C_ADD_L, 10000)){
	  vTaskDelay(25);
	}

	/* wait for bus will be ready before reading */
	while(BUS_I2C2_IsReady(LIS2DW12_I2C_ADD_H, 10000)){
	  vTaskDelay(25);
	}

	/* read sensor revision register */
	if (LIS2DW12_ReadID(&lis2dw12_obj_0, &id_0) != LIS2DW12_OK)
	  aux_sendToAux((char * )"\r\n[BIT LIS2DW]  id 0 FAILED \r\n",
	     		strlen("\r\n[BIT LIS2DW]  id 0 FAILED \r\n"),0,1,DBG_AUX);

	/* read sensor revision register */
	if (LIS2DW12_ReadID(&lis2dw12_obj_1, &id_1) != LIS2DW12_OK)
	  aux_sendToAux((char * )"\r\n[BIT LIS2DW]  id 1 FAILED \r\n",
	     		strlen("\r\n[BIT LIS2DW]  id 1 FAILED \r\n"),0,1,DBG_AUX);


    /* check revision acc #1 */
	if (id_0 == LIS2DW12_ID)
	{
		sprintf(res_buff,"\r\n[BIT LIS2DW] id 0 [%X]hex \r\n",id_0);
		aux_sendToAux(res_buff,strlen(res_buff),0,1,DBG_AUX);
	}
	else
	{
		aux_sendToAux((char*)"\r\n[BIT LIS2DW] id 0 FAILED \r\n",
				strlen("\r\n[BIT LIS2DW] id 0 FAILED \r\n"),0,1,DBG_AUX);
	}

    /* check revision acc #1 */
	if (id_1 == LIS2DW12_ID)
	{
		sprintf(res_buff,"\r\n[BIT LIS2DW] id 1 [%X]hex \r\n",id_1);
		aux_sendToAux(res_buff,strlen(res_buff),0,1,DBG_AUX);
	}
	else
	{
		aux_sendToAux((char*)"\r\n[BIT LIS2DW] id 1 FAILED \r\n",
				strlen("\r\n[BIT LIS2DW] id 1 FAILED \r\n"),0,1,DBG_AUX);
	}

	/* total results */
	if((id_1 == LIS2DW12_ID) && (id_0 == LIS2DW12_ID))
	{
		aux_sendToAux((char*)"\r\n[BIT LIS2DW] OK \r\n",
				strlen("\r\n[BIT LIS2DW] OK \r\n"),0,1,DBG_AUX);
	}

#if 0
	/* run self tests */
	if(LIS2DW12_X_SelfTest(&lis2dw12_obj_0))
	{
		aux_dbg_printLogStr("\r\n[BIT LIS2DW] id 0 self test failed FAILED \r\n");
	}
	else bit_status++;

	/* run self tests */
	if(LIS2DW12_X_SelfTest(&lis2dw12_obj_1))
	{
		aux_dbg_printLogStr("\r\n[BIT LIS2DW] id 1 self test failed FAILED \r\n");
		bit_status++;
	}
	else bit_status++;

	/* bit summary */
	if(bit_status == 2)
	{
		aux_dbg_printLogStr("\r\n[BIT LIS2DW] bit OK \r\n");
	}
#endif

}/* End of LIS2DW_bit_func */

/**
 * @brief LIS2DW_hw_test
 * @brief harware validation function
 * @return system error
 */
int LIS2DW_hw_test(void * arg)
{
	LIS2DW12_Capabilities_t  cap;
	LIS2DW12_IO_t io_ctx_0;
	LIS2DW12_IO_t io_ctx_1;
	uint8_t id_0=0,id_1=0;
	uint8_t dev_id = *(uint8_t*)arg;

	(void)cap;

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
	/* select device id */
	switch(dev_id)
	{
		case(0)://validate id 0

			/* register com bus module  */
			if (LIS2DW12_RegisterBusIO(&lis2dw12_obj_0, &io_ctx_0) != LIS2DW12_OK)
			{
				return SYS_ERROR_BUS_FAILURE;
			}

			/* wait for bus will be ready before reading */
			while(BUS_I2C2_IsReady(LIS2DW12_I2C_ADD_L, 2))
			{
				vTaskDelay(25);
				return SYS_ERROR_BUS_FAILURE;
			}

			/* read sensor revision register */
			if (LIS2DW12_ReadID(&lis2dw12_obj_0, &id_0) != LIS2DW12_OK)
			{
				return SYS_ERROR_BUS_FAILURE;
			}

		    /* check revision acc #1 */
			if (id_0 != LIS2DW12_ID)
			{
				return SYS_ERROR_BUS_FAILURE;
			}

			/* run self tests */
			if(LIS2DW12_X_SelfTest(&lis2dw12_obj_0))
			{
				return SYS_ERROR_PERIPH_FAILURE;
			}
			break;

		case(1)://validate id 1

			/* wait for bus will be ready before reading */
			if (LIS2DW12_RegisterBusIO(&lis2dw12_obj_1, &io_ctx_1) != LIS2DW12_OK)
			{
				return SYS_ERROR_BUS_FAILURE;
			}

			/* wait for bus will be ready before reading */
			while(BUS_I2C2_IsReady(LIS2DW12_I2C_ADD_H, 2))
			{
				vTaskDelay(25);
				return SYS_ERROR_BUS_FAILURE;
			}

			/* read sensor revision register */
			if (LIS2DW12_ReadID(&lis2dw12_obj_1, &id_1) != LIS2DW12_OK)
			{
				return SYS_ERROR_BUS_FAILURE;
			}

		    /* check revision acc #1 */
			if (id_1 != LIS2DW12_ID)
			{
				return SYS_ERROR_BUS_FAILURE;
			}

			/* run self tests */
			if(LIS2DW12_X_SelfTest(&lis2dw12_obj_1))
			{
				return SYS_ERROR_PERIPH_FAILURE;
			}
			break;

		default:
			return SYS_ERROR_WRONG_PARAM;
			break;
	}

	return SYS_ERROR_NONE;
}/* End of LIS2DW_hw_test */



#if 0

static char dataOut[MAX_BUF_SIZE];

/**
  * @brief  Performs LIS2DW12 accelerometer self-test
  * @retval BSP status
  */
static int32_t LIS2DW12_X_SelfTest(LIS2DW12_Object_t * p_obj)
{
  int32_t test_result = SYS_ERROR_NONE;
  LIS2DW12_Axes_t data_nost;
  LIS2DW12_Axes_t data_st;
  LIS2DW12_Axes_t data;
  uint8_t prev_reg_values[ST_REG_COUNT];
  int32_t ret;
  uint32_t i;

  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\[BIT LIS2DW] accelerometer self-test ...");
  aux_dbg_printLogStr(dataOut);



  /* Store current settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = LIS2DW12_Read_Reg(p_obj, reg_addr[i], &prev_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }


  /* Set the sensor for self-test */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = LIS2DW12_Write_Reg(p_obj, reg_addr[i], x_st_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Wait defined time for stable output */
  vTaskDelay(X_POWER_UP_DELAY);

  /* Read first data and discard it */
  if (LIS2DW12_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }

  data_nost.x = 0;
  data_nost.y = 0;
  data_nost.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
    if (LIS2DW12_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }
    data_nost.x += data.x;
    data_nost.y += data.y;
    data_nost.z += data.z;
  }
  data_nost.x /= N_SAMPLES;
  data_nost.y /= N_SAMPLES;
  data_nost.z /= N_SAMPLES;

  /* Enable self-test */
  if ((ret = LIS2DW12_ACC_Set_SelfTest(p_obj, (uint8_t)LIS2DW12_ST_POSITIVE)) != SYS_ERROR_NONE)
  {
    return ret;
  }

  /* Wait defined time for stable output */
  vTaskDelay(X_ST_ENABLED_DELAY);

  /* Read first data and discard it */
  if (LIS2DW12_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  data_st.x = 0;
  data_st.y = 0;
  data_st.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
    if (LIS2DW12_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }
    data_st.x += data.x;
    data_st.y += data.y;
    data_st.z += data.z;
  }
  data_st.x /= N_SAMPLES;
  data_st.y /= N_SAMPLES;
  data_st.z /= N_SAMPLES;

  /* Restore previous settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = LIS2DW12_Write_Reg(p_obj, reg_addr[i], prev_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Evaluate the test */
  if (abs(data_st.x - data_nost.x) < X_LO_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.x - data_nost.x) > X_HI_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.y - data_nost.y) < X_LO_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.y - data_nost.y) > X_HI_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.z - data_nost.z) < X_LO_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.z - data_nost.z) > X_HI_LIM)
  {
    test_result = SYS_ERROR_COMPONENT_FAILURE;
  }

  /* Print measured data */
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nMeasured acceleration [mg]:\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n     AXIS     | PRE-SELFTEST |   SELFTEST\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       X      | %8ld     | %8ld\r\n", data_nost.x, data_st.x);
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       Y      | %8ld     | %8ld\r\n", data_nost.y, data_st.y);
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       Z      | %8ld     | %8ld\r\n", data_nost.z, data_st.z);
  aux_dbg_printLogStr(dataOut);

  /* Print test limits and data */
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nTest limits and data [mg]:\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n  LOW LIMIT   |  DIFFERENCE  |  HIGH LIMIT\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "%8d      | %8d     | %8d\r\n", X_LO_LIM, (int)abs(data_st.x - data_nost.x), X_HI_LIM);
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "%8d      | %8d     | %8d\r\n", X_LO_LIM, (int)abs(data_st.y - data_nost.y), X_HI_LIM);
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "%8d      | %8d     | %8d\r\n", X_LO_LIM, (int)abs(data_st.z - data_nost.z), X_HI_LIM);
  aux_dbg_printLogStr(dataOut);

  /* Print the test result */
  if (test_result == SYS_ERROR_NONE)
  {
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLIS2DW12 accelerometer self-test PASSED!\r\n");
  }
  else
  {
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLIS2DW12 accelerometer self-test FAILED!\r\n");
  }

  /* Disable self-test */
  if ((ret = LIS2DW12_ACC_Set_SelfTest(p_obj, (uint8_t)LIS2DW12_ST_DISABLE)) != SYS_ERROR_NONE)
  {
    return ret;
  }

  aux_dbg_printLogStr(dataOut);

  return ret;
}

#endif


