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
#include <string.h>

#include <max17303.h>
#include "main.h"
#include "cmsis_os.h"
#include "root_task.h"
#include "io_ctl.h"
#include "auxcmd.h"
#include "misc.h"
//#include "xmodem.h"
#include "lp55281.h"

static void SystemClock_Config(void);
static void SystemPower_Config(void);
static void SystemHal_Config(void);


#define IWDG_WINDOW IWDG_WINDOW_DISABLE
#define IWDG_RELOAD (0x0FFF)

I2C_HandleTypeDef hi2c4;
IWDG_HandleTypeDef hiwdg;
led_sm     _led_sm;
battery_limits _battery_limits;
//charge_sm  _charge_sm;


uint8_t response_ok[] =  "GOOD packet number xxxx received \r\n";
uint8_t response_bad[] = "BAD  Packet number xxxx received \r\n";
uint8_t command_wait[] = "\r\n Waiting for command !!! \r\n";
uint8_t application_jump[] = "\r\n Jump to Application !!! \r\n";
uint8_t timeout_jump[] = "\r\n Timeout Jump to Application !!! \r\n";
uint8_t note1[] = "Dial RUN1 to start download of new firmware. \r\n";
uint8_t note2[] = "Dial SKIP to start firmware. \r\n";
uint8_t note3[] = "RUN1 command accepted. \r\n";
uint8_t timeout_note[] = "Timeout occurred. \r\n";
uint8_t crc_bad[] = "CRC is wrong \r\n";
uint8_t end_of_tx[] = "\r\n End of firmware update \r\n";
uint8_t y_modem[] = "\r\n Enter Boot loader \r\n";
uint8_t message_err[] = "\r\n Message error \r\n";

int16_t temp_datax[3];

uint8_t charger_sm = 0;

uint32_t xmodem_on = false;


typedef void (*pFunction)(void);
pFunction Jump_To_Application;

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00100E14;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
	  Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
	  Error_Handler((uint8_t *)__FILE__, __LINE__);
  }

  /** Configure Digital filter
  * @brief  Acquisition completed callback in non blocking mode
  * @param  TscHandle: pointer to a TSC_HandleTypeDef structure that contains
  *         the configuration information for the specified TSC.
  * @retval None
  */


  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
	  Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */
}

//static void _JumpToProgram(uint32_t a_iAddress)
void _JumpToProgram(uint32_t a_iAddress)
{

	uint32_t JumpAddress;
	pFunction Jump_To_Application;

	//     Jump to app at given address.
	JumpAddress = *(__IO uint32_t*) (a_iAddress + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	__set_MSP(*(__IO uint32_t*) a_iAddress);
	Jump_To_Application();
}



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
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
}/* End of IWDG_Init */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
	/* Configure the system HAL --- */
	SystemHal_Config();

	/* Configure the system clock - */
	SystemClock_Config();

	/* Configure the system power - */
	SystemPower_Config();

	/* Configure watch dog -------- */
	IWDG_Init();

	MX_I2C4_Init();

	/* Configure initial io state - */
	io_ctl_gpio_init();

	/* Configure power supplies --- */
	io_ctl_enable_power();

	init_max17303();
	max17303_init_file();

	LP55281_Init();
	LP55281_reset();

//	led('4','B',0x48);
//	led('1','R',0x48);


    _led_sm.led = l_seleftest_fail;//l_seleftest_ok;//l_selftest;  //l_drain;  // l_idle


	/* OS initialization */
	osKernelInitialize();

	/* create root initialization task instance */
	if(pdFAIL == xTaskCreate(rootTask, (const char *) "root", (configMINIMAL_STACK_SIZE<<1), NULL, 1 , NULL))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

    /* Start scheduler */
	osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}/* End of main */



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

/**
  * @brief PoweSupply_Config
  * @retval None
  */
void SystemPower_Config(void)
{
	/* Enable the independent
	 *  analog and I/Os supply */
	LL_PWR_EnableVDDA();
	LL_PWR_EnableVDDIO2();

}/* End of PoweSupply_Config */

/**
  * @brief System Clock Configuration
  * @retval None
  *
  */
void SystemClock_Config(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	  {
	    Error_Handler((uint8_t *)__FILE__, __LINE__);
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
	    Error_Handler((uint8_t *)__FILE__, __LINE__);
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
	    Error_Handler((uint8_t *)__FILE__, __LINE__);
	  }

      /* TODO move to DEVICES lower power */
	  /** LSCO configuration  , 32.768 khz to LPTIM2. */
	  HAL_RCCEx_EnableLSCO(RCC_MCO1SOURCE_LSE);

	  /** MCO configuration  , 32.768 khz to MAX3000X. */
	  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);

}/* End of SystemClock_Config */

/**
  * @brief  This function is used to fetch tick count
  * @retval tick count
  *
  */
uint32_t HAL_GetTick(void)
{
	return xTaskGetTickCount();
}/* End of HAL_GetTick */



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  *
  */
void Error_Handler(uint8_t *file, uint32_t line)
{
	char dbug[200]={0};

	sprintf(dbug,"[FATAL] file %s on line %d \r\n",
			file,(int)line);

	/* send to debug aux channel */
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	/* wait for print out */
	vTaskDelay(1000);

	/* desable interrupts */

	__disable_irq();
	while (1)
	{
	}
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

