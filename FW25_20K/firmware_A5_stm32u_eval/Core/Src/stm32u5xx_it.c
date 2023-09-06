/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32u5xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32u5xx_it.h"

/* External variables --------------------------------------------------------*/

extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern DMA_HandleTypeDef handle_GPDMA1_Channel3;
extern DMA_HandleTypeDef handle_GPDMA1_Channel10;

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart4;

uint8_t isrLevel=0;

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}


/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}


/******************************************************************************/
/* STM32U5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32u5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  //HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/******************************************************************************/
/* STM32U5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32u5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void GPDMA1_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel0);
}

/**
  * @brief This function handles GPDMA1 Channel 1 global interrupt.
  */
void GPDMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel1);
}

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void GPDMA1_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel2);
}

/**
  * @brief This function handles GPDMA1 Channel 1 global interrupt.
  */
void GPDMA1_Channel3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel3);
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void GPDMA1_Channel10_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel10);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}



/**
  * @brief This function handles USART2 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief  This function handles external line 14 interrupt request.
  * @param  None
  * @retval None
  */

extern EXTI_HandleTypeDef hexti14;
void EXTI14_IRQHandler(void)
{
	HAL_EXTI_IRQHandler(&hexti14);
}

/**
  * @brief  This function handles external line 15 interrupt request.
  * @param  None
  * @retval None
  */
extern EXTI_HandleTypeDef hexti15;
void EXTI15_IRQHandler(void)
{
	HAL_EXTI_IRQHandler(&hexti15);
}

/**
  * @brief  This function handles external line 14 interrupt request.
  * @param  None
  * @retval None
  */

extern EXTI_HandleTypeDef hexti11;
void EXTI11_IRQHandler(void)
{
	HAL_EXTI_IRQHandler(&hexti11);
}

/**
  * @brief  This function handles external line 15 interrupt request.
  * @param  None
  * @retval None
  */
extern EXTI_HandleTypeDef hexti12;
void EXTI12_IRQHandler(void)
{
	HAL_EXTI_IRQHandler(&hexti12);
}



/**
  * @brief  This function handles lptim 3 interrupt request
  * @param  None
  * @retval None
  */

extern LPTIM_HandleTypeDef hlptim3;
void LPTIM3_IRQHandler(void)
{
	HAL_LPTIM_IRQHandler(&hlptim3);
}

/**
  * @brief  This function handles lptim 1 interrupt request
  * @param  None
  * @retval None
  */

extern LPTIM_HandleTypeDef hlptim1;
void LPTIM1_IRQHandler(void)
{
	HAL_LPTIM_IRQHandler(&hlptim1);
}

/**
  * @brief This function handles ADC1  interrupts.
  */
extern ADC_HandleTypeDef hadc1;
void ADC1_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
}
