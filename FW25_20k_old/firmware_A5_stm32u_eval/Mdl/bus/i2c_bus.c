/**
  ******************************************************************************
  * @file           : stm32f4xx_nucleo_bus.c
  * @brief          : source file for the BSP BUS IO driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bus.h"
#include "sys_errno.h"
#include "sys_conf.h"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

static uint32_t I2C2InitCounter = 0;
static uint32_t I2C4InitCounter = 0;

static HAL_StatusTypeDef I2C_Init(I2C_HandleTypeDef* hi2c);
static HAL_StatusTypeDef I2C_DeInit(I2C_HandleTypeDef* hi2c);


/**
  * @brief   Initialize I2C HAL
  * @retval BSP status
  */
int32_t BUS_I2C2_Init(void)
{
	int32_t ret = SYS_ERROR_NONE;
	hi2c1.Instance  = I2C2;

	if(I2C2InitCounter++ == 0)
	{
		if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET)
		{
			/* Init the I2C */
			I2C_Init(&hi2c1);
		}
	}
  return ret;
}/* End of BUS_I2C2_Init */

/**
  * @brief  DeInitialize I2C HAL.
  * @retval BUS status
  */
int32_t BUS_I2C2_DeInit(void)
{
	int32_t ret = SYS_ERROR_NONE;

	if (I2C2InitCounter > 0)
	{
		if (--I2C2InitCounter == 0)
		{
			/* MSP DeInit the I2C */
			I2C_DeInit(&hi2c1);

			/* DeInit the I2C */
			if (HAL_I2C_DeInit(&hi2c1) != HAL_OK)
			{
				ret = SYS_ERROR_BUS_FAILURE;
			}
		}
	}
  return ret;
}/* End of BUS_I2C2_DeInit */

/**
  * @brief  Check whether the I2C bus is ready.
  * @param DevAddr : I2C device address
  * @param Trials : Check trials number
  * @retval BUS status
  */
int32_t BUS_I2C2_IsReady(uint16_t DevAddr, uint32_t Trials)
{
  int32_t ret = SYS_ERROR_NONE;

  if (HAL_I2C_IsDeviceReady(&hi2c1, DevAddr, Trials, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    ret = SYS_ERROR_BUSY;
  }

  return ret;
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write
  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BUS status
  */

int32_t BUS_I2C2_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = SYS_ERROR_NONE;

  if (HAL_I2C_Mem_Write(&hi2c1, DevAddr,Reg, I2C_MEMADD_SIZE_8BIT,pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
    {
      ret = SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  SYS_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Read a register of the device through BUS
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to read
  * @param  pData  Pointer to data buffer to read
  * @param  Length Data Length
  * @retval BUS status
  *
  * At Memory end of read transfer, HAL_I2C_MemRxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_MemRxCpltCallback()
  */

int32_t  BUS_I2C2_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	int32_t ret = SYS_ERROR_NONE;
	HAL_StatusTypeDef halval=HAL_OK;

	halval= HAL_I2C_Mem_Read(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, BUS_I2C2_POLL_TIMEOUT);
	if (halval !=HAL_OK)
	{
		if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
		{
			ret = SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE;
		}
		else
		{
			ret = SYS_ERROR_PERIPH_FAILURE;
		}
	}

	return ret;
}/* end of BUS_I2C2_ReadReg */

/**

  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write

  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BUS statu
  */
int32_t BUS_I2C2_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = SYS_ERROR_NONE;

  if (HAL_I2C_Mem_Write(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
    {
      ret = SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  SYS_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Read registers through a bus (16 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  Length Data Length
  * @retval BUS status
  */
int32_t  BUS_I2C2_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = SYS_ERROR_NONE;

  if (HAL_I2C_Mem_Read(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      ret =  SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  SYS_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Send an amount width data through bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BUS status
  */
int32_t BUS_I2C2_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length) {
  int32_t ret = SYS_ERROR_NONE;

  if (HAL_I2C_Master_Transmit(&hi2c1, DevAddr, pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      ret = SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  SYS_ERROR_PERIPH_FAILURE;
    }
  }

  return ret;
}

/**
  * @brief  Receive an amount of data through a bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BUS status
  */
int32_t BUS_I2C2_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length) {
  int32_t ret = SYS_ERROR_NONE;

  if (HAL_I2C_Master_Receive(&hi2c1, DevAddr, pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      ret = SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  SYS_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}


//==========================================================================================
//I2C4
//==========================================================================================

/**
  * @brief  Initialize I2C4 HAL
  * @retval status
  */
int32_t BUS_I2C4_Init(void)
{
	int32_t ret = SYS_ERROR_NONE;
	hi2c4.Instance  = I2C4;

	/* prevent multi-registration */
	if(I2C4InitCounter++ == 0)
	{
		/* check i2c hal status */
		if (HAL_I2C_GetState(&hi2c4) == HAL_I2C_STATE_RESET)
		{
			/* Init the I2C Msp */
			I2C_Init(&hi2c4);
		}
	}
return ret;
}/* End of BUS_I2C4_Init */

/**
  * @brief  DeInitialize I2C HAL.
  * @retval BUS status
  */
int32_t BUS_I2C4_DeInit(void)
{
	int32_t ret = SYS_ERROR_NONE;

	if (I2C4InitCounter > 0)
	{
		if (--I2C4InitCounter == 0)
		{
			/* MSP DeInit the I2C */
			I2C_DeInit(&hi2c4);

			/* DeInit the I2C */
			if (HAL_I2C_DeInit(&hi2c4) != HAL_OK)
			{
				ret = SYS_ERROR_BUS_FAILURE;
			}
		}
	}

return ret;
}/* End of BUS_I2C4_DeInit */

/**
  * @brief  Check whether the I2C bus is ready.
  * @param DevAddr : I2C device address
  * @param Trials : Check trials number
  * @retval BUS status
  */
int32_t BUS_I2C4_IsReady(uint16_t DevAddr, uint32_t Trials)
{
  int32_t ret = SYS_ERROR_NONE;

  if (HAL_I2C_IsDeviceReady(&hi2c4,
		  DevAddr,
		  Trials,
		  BUS_I2C4_POLL_TIMEOUT) != HAL_OK)
  {
    ret = SYS_ERROR_BUSY;
  }
  return ret;
}/* end of BUS_I2C4_IsReady */

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write
  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BUS status
  */

int32_t BUS_I2C4_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = SYS_ERROR_NONE;
//	uint8_t data_t[2];
//	int32_t ret;

//	data_t[0] = Reg;
//	data_t[1] = &data[0];

  if (HAL_I2C_Mem_Write(&hi2c4, DevAddr,Reg, I2C_MEMADD_SIZE_8BIT,
		  pData, Length, BUS_I2C4_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF)
    {
      ret = SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  SYS_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}


/**
  * @brief  Read a register of the device through BUS
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to read
  * @param  pData  Pointer to data buffer to read
  * @param  Length Data Length
  * @retval BUS status
  */

int32_t  BUS_I2C4_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	HAL_StatusTypeDef halval=HAL_OK;
	int32_t ret = SYS_ERROR_NONE;
	uint16_t reg;


	halval = HAL_I2C_Master_Transmit(&hi2c4,
			DevAddr,
			(uint8_t*)&Reg,//DevAddress
			1,//Size
			100);//Timeout

	vTaskDelay(1);

	if (halval == HAL_OK)
	{
		halval= HAL_I2C_Master_Receive(&hi2c4,DevAddr,(uint8_t*)&reg,Length,100);

		if(Length==2)
			reg = ((reg >> 8) | (reg << 8));

		*(uint16_t*)pData=reg;


//		halval= HAL_I2C_Mem_Read(&hi2c4,
//				DevAddr+1,
//				Reg,
//				I2C_MEMADD_SIZE_8BIT,
//				pData,
//				Length,
//				BUS_I2C2_POLL_TIMEOUT);


	}



	/* error handler */
 	if (halval !=HAL_OK)
	{
		if (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF)
		{
			ret = SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE;
		}
		else
		{
			ret = SYS_ERROR_PERIPH_FAILURE;
		}
	}

	return ret;
}/* end of BUS_I2C2_ReadReg */

//==========================================================================================
//MISC
//==========================================================================================

/**
  * @brief  Return system tick in ms
  * @retval Current HAL time base time stamp
  */
int32_t BUS_GetTick(void) {
  return xTaskGetTickCount();
}

static HAL_StatusTypeDef I2C_Init(I2C_HandleTypeDef* hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	HAL_StatusTypeDef ret = HAL_OK;

	if(hi2c->Instance==I2C2)
	{
	  /** Initializes the peripherals clock  */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
		PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
		  Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* io clock enable */
		BUS_I2C2_SCL_GPIO_CLK_ENABLE();

		/* io initialization */
		GPIO_InitStruct.Pin     =
				BUS_I2C2_SCL_GPIO_PIN|
				BUS_I2C2_SDA_GPIO_PIN;

		GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = BUS_I2C2_SCL_GPIO_AF;
		HAL_GPIO_Init(BUS_I2C2_SCL_GPIO_PORT, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C2_CLK_ENABLE();


	    hi2c->Init.Timing           = 0x00C01F67;
	    hi2c->Init.OwnAddress1      = 0;
	    hi2c->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	    hi2c->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	    hi2c->Init.OwnAddress2      = 0;
	    hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	    hi2c->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	    hi2c->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
	    if (HAL_I2C_Init(hi2c) != HAL_OK){
	      ret = HAL_ERROR;
	    }

	    if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK){
	      ret = HAL_ERROR;
	    }

	    if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK){
	      ret = HAL_ERROR;
	    }

	    /** I2C Fast mode Plus enable */
	    if (HAL_I2CEx_ConfigFastModePlus(hi2c, I2C_FASTMODEPLUS_ENABLE) != HAL_OK){
	      Error_Handler((uint8_t *)__FILE__, __LINE__);
	    }
	}
	else if(hi2c->Instance==I2C4)
	{
	  /** Initializes the peripherals clock
	  */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
		PeriphClkInit.I2c2ClockSelection   = RCC_I2C4CLKSOURCE_PCLK1;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
		  Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		BUS_I2C4_SCL_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin     =
				BUS_I2C4_SCL_GPIO_PIN|
				BUS_I2C4_SDA_GPIO_PIN;

		GPIO_InitStruct.Mode        = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull        = GPIO_NOPULL;
		GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate   = BUS_I2C4_SCL_GPIO_AF;
		HAL_GPIO_Init(BUS_I2C4_SCL_GPIO_PORT, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C4_CLK_ENABLE();

		hi2c->Init.Timing           = 0x00100E14;
		hi2c->Init.OwnAddress1      = 0;
		hi2c->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
		hi2c->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
		hi2c->Init.OwnAddress2      = 0;
		hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
		hi2c->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
		hi2c->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
		if (HAL_I2C_Init(hi2c) != HAL_OK){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
		/** Configure Analog filter */
		if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/** Configure Digital filter */
		if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}

return ret;
}/* end of I2C_Init */

static HAL_StatusTypeDef I2C_DeInit(I2C_HandleTypeDef* hi2c)
{
	HAL_StatusTypeDef ret = HAL_OK;

	if(hi2c->Instance==I2C2)
	{
		/* Peripheral clock disable */
		__HAL_RCC_I2C2_CLK_DISABLE();

		/**I2C2 GPIO Configuration*/
	    HAL_GPIO_DeInit(BUS_I2C2_SCL_GPIO_PORT, BUS_I2C2_SCL_GPIO_PIN);
	    HAL_GPIO_DeInit(BUS_I2C2_SDA_GPIO_PORT, BUS_I2C2_SDA_GPIO_PIN);
	}
	else if(hi2c->Instance==I2C3)
	{
		  /* Peripheral clock disable */
		__HAL_RCC_I2C3_CLK_DISABLE();
	}
	else if(hi2c->Instance==I2C4)
	{
		/* Peripheral clock disable */
		__HAL_RCC_I2C4_CLK_DISABLE();

		/**I2C4 GPIO Configuration*/
	    HAL_GPIO_DeInit(BUS_I2C4_SCL_GPIO_PORT, BUS_I2C4_SCL_GPIO_PIN);
	    HAL_GPIO_DeInit(BUS_I2C4_SDA_GPIO_PORT, BUS_I2C4_SDA_GPIO_PIN);
	}

return ret;
}/* end of I2C_DeInit */







