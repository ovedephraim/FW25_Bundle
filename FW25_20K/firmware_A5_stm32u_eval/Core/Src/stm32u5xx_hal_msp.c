/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32u5xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();

	/* Ensure that all 4 interrupt priority bits are used as the pre-emption
	priority. */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

}


//====================================================================================================
//===================== LPTIM/TIM TODO change to callbacks ===========================================
//====================================================================================================

/**
* @brief TIM PWM MSP Initialization
* This function configures the hardware resources used in this example
* @param hlptim: TIM handle pointer
* @retval None
*/
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /* TIM1 Peripheral clock enable */
  __HAL_RCC_TIM16_CLK_ENABLE();

  /* Common configuration for all channels */
  GPIO_InitStruct.Pin = HEAT_EN_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM16;

  HAL_GPIO_Init(HEAT_EN_GPIO_PORT, &GPIO_InitStruct);
}


/**
* @brief LPTIM MSP Initialization
* This function configures the hardware resources used in this example
* @param hlptim: LPTIM handle pointer
* @retval None
*/
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef* hlptim)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	if(hlptim->Instance==LPTIM3)
	{
		/* Enable autonomous mode for LPTIM1 */
		__HAL_RCC_LPTIM3_CLKAM_ENABLE();

		/** Initializes the peripherals clock
		*/
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM34;
		PeriphClkInit.Lptim34ClockSelection = RCC_LPTIM34CLKSOURCE_LSE;

		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* Peripheral clock enable */
		__HAL_RCC_LPTIM3_CLK_ENABLE();

		LIS2DW_XL_SYNC_GPIO_CLK_ENABLE();

		/**LPTIM1 GPIO Configuration
		*/
		GPIO_InitStruct.Pin  = LIS2DW_XL_SYNC_PIN;
		GPIO_InitStruct.Mode = LIS2DW_XL_SYNC_GPIO_MODE;
		GPIO_InitStruct.Pull = LIS2DW_XL_SYNC_GPIO_PULL;
		GPIO_InitStruct.Speed = LIS2DW_XL_SYNC_SPEED;
		GPIO_InitStruct.Alternate = LIS2DW_XL_SYNC_ALTERNATE;
		HAL_GPIO_Init(LIS2DW_XL_SYNC_GPIO_PORT, &GPIO_InitStruct);
	}

	if(hlptim->Instance==LPTIM1)//=====================================
	{
		/* Enable autonomous mode for LPTIM1 */
		__HAL_RCC_LPTIM1_CLKAM_ENABLE();

		/** Initializes the peripherals clock
		*/
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
		PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* Peripheral clock enable */
		__HAL_RCC_LPTIM1_CLK_ENABLE();
	}

	if(hlptim->Instance==LPTIM2) //=====================================
	{
		/** Initializes the peripherals clock
		*/
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM2;
		PeriphClkInit.Lptim2ClockSelection = RCC_LPTIM2CLKSOURCE_LSE;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* Peripheral clock enable */
		__HAL_RCC_LPTIM2_CLK_ENABLE();
	}

}/* end of HAL_LPTIM_MspInit */



/**
* @brief LPTIM MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hlptim: LPTIM handle pointer
* @retval None
*/
void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef* hlptim)
{
  if(hlptim->Instance==LPTIM1)
  {
  /* USER CODE BEGIN LPTIM1_MspDeInit 0 */

  /* USER CODE END LPTIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LPTIM1_CLK_DISABLE();

    /**LPTIM1 GPIO Configuration
    PB2     ------> LPTIM1_CH1
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);

  /* USER CODE BEGIN LPTIM1_MspDeInit 1 */

  /* USER CODE END LPTIM1_MspDeInit 1 */
  }

}


/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hi2c->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */
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
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
  else if(hi2c->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
    PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
    	Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PG7     ------> I2C3_SCL
    PG8     ------> I2C3_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
  else if(hi2c->Instance==I2C4)
  {
  /* USER CODE BEGIN I2C4_MspInit 0 */

  /* USER CODE END I2C4_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
    PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
    	Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**I2C4 GPIO Configuration
    PF14     ------> I2C4_SCL
    PF15     ------> I2C4_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C4_CLK_ENABLE();
  /* USER CODE BEGIN I2C4_MspInit 1 */

  /* USER CODE END I2C4_MspInit 1 */
  }

}


/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_1);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
  else if(hi2c->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PG7     ------> I2C3_SCL
    PG8     ------> I2C3_SDA
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_8);

  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
  else if(hi2c->Instance==I2C4)
  {
  /* USER CODE BEGIN I2C4_MspDeInit 0 */

  /* USER CODE END I2C4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C4_CLK_DISABLE();

    /**I2C4 GPIO Configuration
    PF14     ------> I2C4_SCL
    PF15     ------> I2C4_SDA
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_14);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_15);

  /* USER CODE BEGIN I2C4_MspDeInit 1 */

  /* USER CODE END I2C4_MspDeInit 1 */
  }

}

//==================================================================================================
//==================================================================================================
//==================================================================================================


