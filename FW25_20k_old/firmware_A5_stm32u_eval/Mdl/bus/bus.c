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

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi2;
HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef* hspi);

static uint32_t SPI3InitCounter = 0;
static uint32_t SPI2InitCounter = 0;
static void SPI_MspInit(SPI_HandleTypeDef* hspi);
static void SPI_MspDeInit(SPI_HandleTypeDef* hspi);

#if (USE_CUBEMX_BSP_V2 == 1)
static uint32_t I2C_GetTiming(uint32_t clock_src_hz, uint32_t i2cfreq_hz);
static void Compute_PRESC_SCLDEL_SDADEL(uint32_t clock_src_freq, uint32_t I2C_Speed);
static uint32_t Compute_SCLL_SCLH (uint32_t clock_src_freq, uint32_t I2C_speed);
#endif


///=====================================================================================///
//                            BUS OPERATIONS OVER SPI[1/2/3]
///=====================================================================================///

/**
  * @brief  Initialize SPI HAL
  * @retval BSP status
  */
int32_t BUS_SPI3_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	int32_t ret = SYS_ERROR_NONE;

	MAX30001_CS_GPIO_CLK_ENABLE();

	GPIO_InitStruct.Pin   =  MAX30001_CS_PIN;
	GPIO_InitStruct.Mode  =  MAX30001_CS_GPIO_MODE;
	GPIO_InitStruct.Pull  =  MAX30001_CS_GPIO_PULL;
	GPIO_InitStruct.Speed =  MAX30001_CS_GPIO_SPEED;
	HAL_GPIO_Init(MAX30001_CS_GPIO_PORT, &GPIO_InitStruct);


	hspi3.Instance  = SPI3;
	if(SPI3InitCounter++ == 0)
	{
		if (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_RESET)
		{
			#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0U)
			/* Init the I2C Msp */
			SPI_MspInit(&hspi3);
			#else
			if(IsI2C2MspCbValid == 0U)
			{
				if(BUS_I2C2_RegisterDefaultMspCallbacks() != SYS_ERROR_NONE)
				{
					return SYS_ERROR_MSP_FAILURE;
				}
			}
		#endif
			if(ret == SYS_ERROR_NONE)
			{
				/* Init the SPI */
				if(MX_SPI_Init(&hspi3) != HAL_OK)
				{
					ret = SYS_ERROR_BUS_FAILURE;
				}
				else
				{
					ret = SYS_ERROR_NONE;
				}
			}
		}
	}

	return ret;
}/* BUS_SPI3_Init */

int32_t BUS_SPI2_Init(void)
{
	int32_t ret = SYS_ERROR_NONE;

	hspi2.Instance  = SPI2;
	if(SPI2InitCounter++ == 0)
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		MAX30001_CS_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin   =  MAX30001_CS_PIN;
		GPIO_InitStruct.Mode  =  MAX30001_CS_GPIO_MODE;
		GPIO_InitStruct.Pull  =  MAX30001_CS_GPIO_PULL;
		GPIO_InitStruct.Speed =  MAX30001_CS_GPIO_SPEED;
		HAL_GPIO_Init(MAX30001_CS_GPIO_PORT, &GPIO_InitStruct);

		MAX30003_CS_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin   =  MAX30003_CS_PIN;
		GPIO_InitStruct.Mode  =  MAX30003_CS_GPIO_MODE;
		GPIO_InitStruct.Pull  =  MAX30003_CS_GPIO_PULL;
		GPIO_InitStruct.Speed =  MAX30003_CS_GPIO_SPEED;
		HAL_GPIO_Init(MAX30003_CS_GPIO_PORT, &GPIO_InitStruct);


//		switch(SpiDev)
//		{
//		case(1):/** Initializes chip select [CS#1] io pins */
//				MAX30001_CS_GPIO_CLK_ENABLE();
//				GPIO_InitStruct.Pin   =  MAX30001_CS_PIN;
//				GPIO_InitStruct.Mode  =  MAX30001_CS_GPIO_MODE;
//				GPIO_InitStruct.Pull  =  MAX30001_CS_GPIO_PULL;
//				GPIO_InitStruct.Speed =  MAX30001_CS_GPIO_SPEED;
//				HAL_GPIO_Init(MAX30001_CS_GPIO_PORT, &GPIO_InitStruct);
//				break;
//		case(2):/** Initializes chip select [CS#2] io pins */
//				MAX30003_CS_GPIO_CLK_ENABLE();
//				GPIO_InitStruct.Pin   =  MAX30003_CS_PIN;
//				GPIO_InitStruct.Mode  =  MAX30003_CS_GPIO_MODE;
//				GPIO_InitStruct.Pull  =  MAX30003_CS_GPIO_PULL;
//				GPIO_InitStruct.Speed =  MAX30003_CS_GPIO_SPEED;
//				HAL_GPIO_Init(MAX30003_CS_GPIO_PORT, &GPIO_InitStruct);
//				break;
//		default:
//			ret=SYS_ERROR_WRONG_PARAM;
//			break;
//		}

		if(ret==SYS_ERROR_NONE)
		{
			if (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_RESET)
			{
				/* Init the SPI Msp */
				SPI_MspInit(&hspi2);

				/* Init the SPI */
				if(MX_SPI_Init(&hspi2) != HAL_OK)
				{
					ret = SYS_ERROR_BUS_FAILURE;
				}
				else
				{
					ret = SYS_ERROR_NONE;
					/* todo remove */
					while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				}
			}
		}
	}

	return ret;
}/* BUS_SPI2_Init */


/**
  * @brief  DeInitialize SPI HAL.
  * @retval BUS status
  */
int32_t BUS_SPI3_DeInit(void)
{
  int32_t ret = SYS_ERROR_NONE;

  if (SPI3InitCounter > 0)
  {
    if (--SPI3InitCounter == 0)
    {
      /* DeInit spi3*/
    	SPI_MspDeInit(&hspi3);

      /* DeInit spi3 hal */
      if (HAL_SPI_DeInit(&hspi3) != HAL_OK)
      {
        ret = SYS_ERROR_BUS_FAILURE;
      }
    }
  }
  return ret;
}/* end of BUS_SPI3_DeInit */

int32_t BUS_SPI2_DeInit(void)
{
  int32_t ret = SYS_ERROR_NONE;

  if (SPI2InitCounter > 0)
  {
    if (--SPI2InitCounter == 0)
    {
        /* deinit CS io pins */
    	HAL_GPIO_DeInit(MAX30001_CS_GPIO_PORT, MAX30001_CS_PIN);
    	HAL_GPIO_DeInit(MAX30003_CS_GPIO_PORT, MAX30003_CS_PIN);

		/* DeInit the SPI2 */
		SPI_MspDeInit(&hspi2);

		/* DeInit the SPI2[HAL] */
		if (HAL_SPI_DeInit(&hspi2) != HAL_OK)
		{
			ret = SYS_ERROR_BUS_FAILURE;
		}
    }
  }
  return ret;
}/* end of BUS_SPI2_DeInit */


/**
  * @brief  Check whether the SPI bus is ready.
  * @param DevAddr : SPI device number
  * @param Trials : Check trials number
  * @retval BUS status
  */
int32_t BUS_SPI3_IsReady(uint16_t SpiDev, uint32_t Trials)
{
  int32_t ret = SYS_ERROR_NONE;


  return ret;
}/* end of BUS_SPI3_IsReady */

int32_t BUS_SPI2_IsReady(uint16_t SpiDev, uint32_t Trials)
{
  int32_t ret = SYS_ERROR_NONE;

  return ret;
}/* end of BUS_SPI3_IsReady */


/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write
  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BUS status
  */

int32_t BUS_SPI3_Write(uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size)
{
	int32_t ret = SYS_ERROR_NONE;

	/*Configure maxim30001 cs GPIO pin Output Level */
	HAL_GPIO_WritePin(MAX30001_CS_GPIO_PORT, MAX30001_CS_PIN, GPIO_PIN_RESET);//CS pin low

	if(HAL_OK != HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, rx_size, 1000))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		ret = SYS_ERROR_PERIPH_FAILURE;
	}

	HAL_GPIO_WritePin(MAX30001_CS_GPIO_PORT, MAX30001_CS_PIN, GPIO_PIN_SET); //CS pin high

 return ret;
}/* end of BUS_SPI3_Write */

int32_t BUS_SPI2_Write(uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size)
{
	int32_t ret = SYS_ERROR_NONE;

	if(HAL_OK != HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, rx_size, 1000))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		ret = SYS_ERROR_PERIPH_FAILURE;
	}

	return ret;
}/* end of BUS_SPI2_Write */


/**
  * @brief  Read a register of the device through BUS
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to read
  * @param  pData  Pointer to data buffer to read
  * @param  Length Data Length
  * @retval BUS status
  */
int32_t  BUS_SPI3_Read(uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size)
{
	int32_t ret = SYS_ERROR_NONE;

	/*Configure maxim30001 cs GPIO pin Output Level */
	//HAL_GPIO_WritePin(MAX30001_CS_GPIO_PORT, MAX30001_CS_PIN, GPIO_PIN_RESET);//CS pin low


	if(HAL_OK != HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, rx_size, 1000))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		ret = SYS_ERROR_PERIPH_FAILURE;
	}

	//HAL_GPIO_WritePin(MAX30001_CS_GPIO_PORT, MAX30001_CS_PIN, GPIO_PIN_SET); //CS pin high

return ret;
}/* end of BUS_SPI3_Read */

int32_t  BUS_SPI2_Read(uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size)
{
	int32_t ret = SYS_ERROR_NONE;

	if(HAL_OK != HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, rx_size, 1000))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		ret = SYS_ERROR_PERIPH_FAILURE;
	}

return ret;
}/* end of BUS_SPI2_Read */


/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	if(hspi->Instance==SPI3)
	{
		/** Initializes the peripherals clock
		*/
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
		PeriphClkInit.Spi3ClockSelection = RCC_SPI3CLKSOURCE_SYSCLK;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
		  Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* Peripheral clock enable */
		__HAL_RCC_SPI3_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**SPI3 GPIO Configuration
		PB3 (JTDO/TRACESWO)     ------> SPI3_SCK
		PB4 (NJTRST)     ------> SPI3_MISO
		PB5     ------> SPI3_MOSI
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	if(hspi->Instance==SPI2) //[SPI 2]=========================================
	{

		/** Initializes the peripherals clock */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
		PeriphClkInit.Spi2ClockSelection   = RCC_SPI2CLKSOURCE_SYSCLK;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
		  Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* Peripheral clock enable */
		__HAL_RCC_SPI2_CLK_ENABLE();

		BUS_SPI2_MOSI_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin       = BUS_SPI2_MOSI_GPIO_PIN|BUS_SPI2_MISO_GPIO_PIN;
		GPIO_InitStruct.Mode      = BUS_SPI2_MOSI_GPIO_MODE;
		GPIO_InitStruct.Pull      = BUS_SPI2_MOSI_PULL;
		GPIO_InitStruct.Speed     = BUS_SPI2_MOSI_GPIO_SPEED;
		GPIO_InitStruct.Alternate = BUS_SPI2_MOSI_GPIO_AF;
		HAL_GPIO_Init(BUS_SPI2_MOSI_GPIO_PORT, &GPIO_InitStruct);


		BUS_SPI2_SCLK_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin       = BUS_SPI2_SCLK_GPIO_PIN;
		GPIO_InitStruct.Mode      = BUS_SPI2_SCLK_GPIO_MODE;
		GPIO_InitStruct.Pull      = BUS_SPI2_SCLK_PULL;
		GPIO_InitStruct.Speed     = BUS_SPI2_SCLK_GPIO_SPEED;
		GPIO_InitStruct.Alternate = BUS_SPI2_SCLK_GPIO_AF;
		HAL_GPIO_Init(BUS_SPI2_SCLK_GPIO_PORT, &GPIO_InitStruct);
	}
}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PB3 (JTDO/TRACESWO)     ------> SPI3_SCK
    PB4 (NJTRST)     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

  }
  if(hspi->Instance==SPI2)//[SPI 2]=========================================
  {
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    HAL_GPIO_DeInit(BUS_SPI2_MOSI_GPIO_PORT, BUS_SPI2_MOSI_GPIO_PIN);
    HAL_GPIO_DeInit(BUS_SPI2_MISO_GPIO_PORT, BUS_SPI2_MISO_GPIO_PIN);
    HAL_GPIO_DeInit(BUS_SPI2_SCLK_GPIO_PORT, BUS_SPI2_SCLK_GPIO_PIN);
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef* hspi)
{
	SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

	if(hspi->Instance==SPI3)//================================================================
	{
		/* SPI3 parameter configuration*/
		hspi3.Init.Mode                       = SPI_MODE_MASTER;
		hspi3.Init.Direction                  = SPI_DIRECTION_2LINES;
		hspi3.Init.DataSize                   = SPI_DATASIZE_8BIT;
		hspi3.Init.CLKPolarity                = SPI_POLARITY_LOW;
		hspi3.Init.CLKPhase                   = SPI_PHASE_1EDGE;
		hspi3.Init.NSS                        = SPI_NSS_SOFT;
		hspi3.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_2;
		hspi3.Init.FirstBit                   = SPI_FIRSTBIT_MSB;
		hspi3.Init.TIMode                     = SPI_TIMODE_DISABLE;
		hspi3.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
		hspi3.Init.CRCPolynomial              = 0x7;
		hspi3.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;
		hspi3.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
		hspi3.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
		hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
		hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
		hspi3.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
		hspi3.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
		hspi3.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
		hspi3.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
		hspi3.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;
		hspi3.Init.ReadyMasterManagement      = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
		hspi3.Init.ReadyPolarity              = SPI_RDY_POLARITY_HIGH;
		if (HAL_SPI_Init(&hspi3) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState     = SPI_AUTO_MODE_DISABLE;
		HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH1_TCF_TRG;
		HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity  = SPI_TRIG_POLARITY_RISING;
		if (HAL_SPIEx_SetConfigAutonomousMode(&hspi3, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}

	if(hspi->Instance==SPI2)//==============================================================
	{
		/* SPI2 parameter configuration*/
		hspi->Init.Mode                       = SPI_MODE_MASTER;
		hspi->Init.Direction                  = SPI_DIRECTION_2LINES;
		hspi->Init.DataSize                   = SPI_DATASIZE_8BIT;
		hspi->Init.CLKPolarity                = SPI_POLARITY_LOW;
		hspi->Init.CLKPhase                   = SPI_PHASE_1EDGE;
		hspi->Init.NSS                        = SPI_NSS_SOFT;
		hspi->Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_2;
		hspi->Init.FirstBit                   = SPI_FIRSTBIT_MSB;
		hspi->Init.TIMode                     = SPI_TIMODE_DISABLE;
		hspi->Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
		hspi->Init.CRCPolynomial              = 0x7;
		hspi->Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;
		hspi->Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
		hspi->Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
		hspi->Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
		hspi->Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
		hspi->Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
		hspi->Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
		hspi->Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
		hspi->Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
		hspi->Init.IOSwap                     = SPI_IO_SWAP_DISABLE;
		hspi->Init.ReadyMasterManagement      = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
		hspi->Init.ReadyPolarity              = SPI_RDY_POLARITY_HIGH;
		if (HAL_SPI_Init(hspi) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState     = SPI_AUTO_MODE_DISABLE;
		HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH1_TCF_TRG;
		HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity  = SPI_TRIG_POLARITY_RISING;
		if (HAL_SPIEx_SetConfigAutonomousMode(hspi, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}//======================================================================================

	return HAL_OK;
}

#if 0
/*******************************************************************************
                            BUS OPERATIONS OVER I2C
*******************************************************************************/
/**
  * @brief   Initialize I2C HAL
  * @autor Anton Kanaev
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
    #if (USE_HAL_I2C_REGISTER_CALLBACKS == 0U)
      /* Init the I2C Msp */
      I2C2_MspInit(&hi2c1);
    #else
      if(IsI2C2MspCbValid == 0U)
      {
        if(BUS_I2C2_RegisterDefaultMspCallbacks() != SYS_ERROR_NONE)
        {
          return SYS_ERROR_MSP_FAILURE;
        }
      }
    #endif
      if(ret == SYS_ERROR_NONE)
      {
        /* Init the I2C */
        if(MX_I2C2_Init(&hi2c1) != HAL_OK)
        {
          ret = SYS_ERROR_BUS_FAILURE;
        }
        else
        {
          ret = SYS_ERROR_NONE;
        }
      }
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
  #if (USE_HAL_I2C_REGISTER_CALLBACKS == 0U)
      /* DeInit the I2C */
      I2C2_MspDeInit(&hi2c1);
  #endif
      /* DeInit the I2C */
      if (HAL_I2C_DeInit(&hi2c1) != HAL_OK)
      {
        ret = SYS_ERROR_BUS_FAILURE;
      }
    }
  }
  return ret;
}

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
  *
  *
  *
  */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t test=0;
	test++;
}

int32_t  BUS_I2C2_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	int32_t ret = SYS_ERROR_NONE;
	HAL_StatusTypeDef halval=HAL_OK;

//	halval=HAL_I2C_Mem_Read_DMA(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length);
//	if (halval !=HAL_OK)
//	{
//		if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
//		{
//			ret = SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE;
//		}
//		else
//		{
//			ret = SYS_ERROR_PERIPH_FAILURE;
//		}
//	}


//  At Memory end of read transfer, HAL_I2C_MemRxCpltCallback() is executed and users can
//             add their own code by customization of function pointer HAL_I2C_MemRxCpltCallback()


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
}

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

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
/**
  * @brief Register Default BUS I2C2 Bus Msp Callbacks
  * @retval BUS status
  */
int32_t BUS_I2C2_RegisterDefaultMspCallbacks (void)
{

  __HAL_I2C_RESET_HANDLE_STATE(&hi2c1);

  /* Register MspInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MSPINIT_CB_ID, I2C2_MspInit)  != HAL_OK)
  {
    return SYS_ERROR_PERIPH_FAILURE;
  }

  /* Register MspDeInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MSPDEINIT_CB_ID, I2C2_MspDeInit) != HAL_OK)
  {
    return SYS_ERROR_PERIPH_FAILURE;
  }
  IsI2C2MspCbValid = 1;

  return SYS_ERROR_NONE;
}

/**
  * @brief BUS I2C2 Bus Msp Callback registering
  * @param Callbacks     pointer to I2C2 MspInit/MspDeInit callback functions
  * @retval BUS status
  */
int32_t BUS_I2C2_RegisterMspCallbacks (BUS_I2C_Cb_t *Callbacks)
{
  /* Prevent unused argument(s) compilation warning */
  __HAL_I2C_RESET_HANDLE_STATE(&hi2c1);

   /* Register MspInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MSPINIT_CB_ID, Callbacks->pMspInitCb)  != HAL_OK)
  {
    return SYS_ERROR_PERIPH_FAILURE;
  }

  /* Register MspDeInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MSPDEINIT_CB_ID, Callbacks->pMspDeInitCb) != HAL_OK)
  {
    return SYS_ERROR_PERIPH_FAILURE;
  }

  IsI2C2MspCbValid = 1;

  return SYS_ERROR_NONE;
}
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/**
  * @brief  Return system tick in ms
  * @retval Current HAL time base time stamp
  */
int32_t BUS_GetTick(void) {
  return xTaskGetTickCount();
}

/* I2C2 init function */




HAL_StatusTypeDef MX_I2C2_Init(I2C_HandleTypeDef* hi2c)
{
    HAL_StatusTypeDef ret = HAL_OK;

    hi2c->Instance              = I2C2;
    hi2c->Init.Timing           = 0x00C01F67;//0x30909DEC;
    hi2c->Init.OwnAddress1      = 0;
    hi2c->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2      = 0;
    hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;


    if (HAL_I2C_Init(hi2c) != HAL_OK)
    {
      ret = HAL_ERROR;
    }

    if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      ret = HAL_ERROR;
    }

    if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK)
    {
      ret = HAL_ERROR;
    }

    /** I2C Fast mode Plus enable
    */
    if (HAL_I2CEx_ConfigFastModePlus(hi2c, I2C_FASTMODEPLUS_ENABLE) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    return ret;
}

static void I2C2_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if(i2cHandle->Instance==I2C2)
  {
	  /** Initializes the peripherals clock
	  */
	    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	    {
	      Error_Handler((uint8_t *)__FILE__, __LINE__);
	    }


		__HAL_RCC_GPIOF_CLK_ENABLE();
		/**I2C2 GPIO Configuration
		PB8     ------> I2C2_SCL
		PB9     ------> I2C2_SDA
		*/
		GPIO_InitStruct.Pin     = BUS_I2C2_SCL_GPIO_PIN|BUS_I2C2_SDA_GPIO_PIN;
		GPIO_InitStruct.Mode    = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull    = GPIO_PULLUP;
		GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = BUS_I2C2_SCL_GPIO_AF;
		HAL_GPIO_Init(BUS_I2C2_SCL_GPIO_PORT, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C2_CLK_ENABLE();
  }

}

static void I2C2_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB8     ------> I2C2_SCL
    PB9     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(BUS_I2C2_SCL_GPIO_PORT, BUS_I2C2_SCL_GPIO_PIN);

    HAL_GPIO_DeInit(BUS_I2C2_SDA_GPIO_PORT, BUS_I2C2_SDA_GPIO_PIN);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
}

#endif
