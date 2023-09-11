/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

#include "io_ctl.h"
#include "xmodem.h"
#include "stm32u5xx_ll_pwr.h"
#include "stdio.h"
#include "string.h"

static void SystemClock_Config(void);
static void SystemPower_Config(void);
static void SystemHal_Config(void);
static void MX_ICACHE_Init(void);


#define IWDG_WINDOW IWDG_WINDOW_DISABLE
#define IWDG_RELOAD (0x0FFF)

uint8_t response_ok[] =  "GOOD packet number xxxx received \r\n";
uint8_t response_bad[] = "BAD  Packet number xxxx received \r\n";
uint8_t command_wait[] = "\r\n Waiting for command !!! \r\n";
uint8_t application_jump[] = "\r\n Jump to Application !!! \r\n";
uint8_t timeout_jump[] = "\r\n Timeout Jump to Application !!! \r\n";
uint8_t note1[] = "Dial RUN1 to start download of new firmware. \r\n";
uint8_t note2[] = "Dial SKIP to start firmware. \r\n";
uint8_t note3[] = "RUN1 command accepted. \r\n";
uint8_t timeout_note[] = "Timeout occured. \r\n";
uint8_t crc_bad[] = "CRC is wrong \r\n";
uint8_t end_of_tx[] = "\r\n End of fimware update \r\n";

IWDG_HandleTypeDef hiwdg;
UART_HandleTypeDef huart4,huart5;

int16_t temp_datax[3];
uint32_t command;
uint32_t xmodem_on = false;

typedef void (*pFunction)(void);
pFunction Jump_To_Application;

extern uint32_t get_xmodem(void);
extern void flasherase(uint8_t full);
extern uint8_t UartReadyTx;
extern uint8_t UartReadyRx;
extern uint8_t aRxBuffer[];
extern uint8_t compare_block[];
extern uint32_t uart_receive(uint8_t *data, uint16_t length);

extern uint8_t backup_a5_unique[];
extern uint8_t runtime_a5_unique[];
extern uint8_t runtime_a5_write_data[];

extern uint8_t runtime_a5_string[];
extern uint8_t backup_a5_string[];
extern uint8_t runtime_a5_write_string[];

extern flash_file_param _flash_file_param;


/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void IWDG_Init(void)
{
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Window = IWDG_WINDOW;
	hiwdg.Init.Reload = IWDG_RELOAD;//~35seconds
	hiwdg.Init.EWI = 0;

	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	//	Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
}/* End of IWDG_Init */


//static void _JumpToProgram(uint32_t a_iAddress)
void _JumpToProgram(uint32_t a_iAddress)
{

	uint32_t JumpAddress;
	pFunction Jump_To_Application;

    // Jump to app at given address.
    JumpAddress = *(__IO uint32_t*) (a_iAddress + 4);
    Jump_To_Application = (pFunction) JumpAddress;
    __set_MSP(*(__IO uint32_t*) a_iAddress);
    Jump_To_Application();
}


/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_UART5_Init(void)
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 921600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
    //Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
    //Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
    //Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
    //Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* register rx complete callback */
 // 	huart4.RxEventCallback = BT_RxEventCallback;
  /* USER CODE END USART4_Init 2 */
}


/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_UART5_Init(void)
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 921600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
    //Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
    //Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
    //Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
    //Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* register rx complete callback */
  //	huart5.RxEventCallback = RxEventCallback;
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief System Hal config
  *  Reset of all peripherals,
  *  Initializes the Flash
  *  interface and the Systick.
  *
  * @retval None
  */
void SystemHal_Config(void)
{
 	HAL_Init();

}/* End of SystemHal_Config */


uint32_t wait4command(void)
{
	uint32_t result = 0;
	uint32_t timeout = 0;

	while(result == 0)
	{
		if(strstr(&compare_block[0],"RUN1") != NULL)
		{
			result = 1;
			return result;
		}
		if(strstr(&compare_block[0],"SKIP") != NULL)
		{
			result = 2;
			return result;
		}

		if(++timeout > 200000000)  // equal 5 minutes.
		{
			transmit_notification(&timeout_jump[0]);
			result = 2;
			return result;
		}
	}
	return result;
}




/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t result;
	uint8_t a;

	/* Configure the system HAL --- */
	SystemHal_Config();

	/* Configure the system clock - */
	SystemClock_Config();

	/* Configure the system power - */
	SystemPower_Config();

	/* Configure watch dog -------- */
//	IWDG_Init();

	/* Configure initial io state - */
	io_ctl_gpio_init();

	/* Configure power supplies --- */
	io_ctl_enable_power();

	MX_ICACHE_Init();

	// Start of XMODEM initialization.

	MX_UART5_Init();
	HAL_UART_MspInit(&huart5);
	__HAL_UART_ENABLE_IT( &huart5, UART_IT_RXNE);
//	uart_receive(&compare_block[0], 40);
    uart_receive(&aRxBuffer[0],DATA_CHUNK);

//    program_switcher();

	transmit_notification(&command_wait[0]);
	transmit_notification(&note1[0]);
	transmit_notification(&note2[0]);
	// Wait for command "RUN1" or "SKIP".
//    result = wait4command();
    transmit_notification(&note3[0]);

    // Restart UART5 again.
    MX_UART5_Init();
    HAL_UART_MspInit(&huart5);
   __HAL_UART_ENABLE_IT( &huart5, UART_IT_RXNE);
//   uart_receive(&compare_block[0],40);
   uart_receive(&aRxBuffer[0], 1029);

   UartReadyRx = false;
   xmodem_on = true;
   result = 1;
//   program_switcher();

   for(a = 0;a < 16;a++)
   {
	   _flash_file_param.f_magic_val0[a] = backup_a5_unique[a];
	   _flash_file_param.f_file0[a] = backup_a5_string[a];

	   _flash_file_param.f_magic_val1[a] = runtime_a5_unique[a];
	   _flash_file_param.f_file1[a] = runtime_a5_string[a];

	   _flash_file_param.f_magic_val2[a] = runtime_a5_write_data[a];
	   _flash_file_param.f_file2[a] = runtime_a5_write_string[a];
   }

	switch(result)
	{
		case 1:
//			      get_ymodem();
			      flasherase(1);
				  if(get_xmodem() == X_OK)
				  {
						erasesignature(A5_signature);
					    write_A5_signature(backup_a5_program);

					    erasesignature(backup_a5_program);
					    write_signature(backup_a5_program);

					    erasesignature(runtime_a5_program);

					   _JumpToProgram(0x08020000);
				  }
				  else
				  {
					  NVIC_SystemReset();
				     _JumpToProgram(0x08000000);
				  }
				  break;
		case 2:
				  transmit_notification(&application_jump[0]);
				  _JumpToProgram(0x08100000);
				  break;
		default:
			      break;
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
		  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

		  /** Configure the main internal regulator output voltage
		  */
		  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
		  {
			  Error_Handler();
		   // Error_Handler((uint8_t *)__FILE__, __LINE__);
		  }

		  /** Configure LSE Drive Capability
		  */
		  HAL_PWR_EnableBkUpAccess();
		  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

		  /** Initializes the CPU, AHB and APB buses clocks
		  */
		  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|
				  	  	  	  	  	  	  	 RCC_OSCILLATORTYPE_LSE|
											 RCC_OSCILLATORTYPE_MSI;

		  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
		  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
		  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
		  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
		  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
		  RCC_OscInitStruct.MSIClockRange =       RCC_MSIRANGE_0;
		  RCC_OscInitStruct.PLL.PLLState =        RCC_PLL_NONE;

		  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		  {
			  Error_Handler();
		   // Error_Handler((uint8_t *)__FILE__, __LINE__);
		  }

		  /** Initializes the CPU, AHB and APB buses clocks
		  */
		  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
		                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
		                              |RCC_CLOCKTYPE_PCLK3;
		  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
		  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
		  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
		  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;


		  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
		  {
			  Error_Handler();
		  //  Error_Handler((uint8_t *)__FILE__, __LINE__);
		  }

	      /* TODO move to DEVICES lower power */
		  /** LSCO configuration  , 32.768 khz to LPTIM2. */
		  HAL_RCCEx_EnableLSCO(RCC_MCO1SOURCE_LSE);

		  /** MCO configuration  , 32.768 khz to MAX3000X. */
		  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /* Enable the independent
  	 *  analog and I/Os supply */
  	LL_PWR_EnableVDDA();
  	LL_PWR_EnableVDDIO2();

}


/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}




/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void)
//{
////  GPIO_InitTypeDef GPIO_InitStruct = {0};
//}




/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle -> Instance == UART5)
	{
	   /* Set Receive flag:e */

	   if(xmodem_on == false)
	   {
		   uart_receive(&compare_block[0], 16);  // 20
	   }
	   else
	   {
		   uart_receive(&aRxBuffer[0], DATA_CHUNK);
	   }
       UartReadyRx = true; //SET;
	}
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
