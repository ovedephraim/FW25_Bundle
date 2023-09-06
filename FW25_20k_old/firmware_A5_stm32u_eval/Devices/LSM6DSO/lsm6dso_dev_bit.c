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

/**
  ******************************************************************************
  * File Name          : app_mems.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.9.3.0 instances.
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

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h> /* abs */

#include "lsm6dso_dev.h"
#include "bus.h"

extern LSM6DSO_Object_t lsm6dso_obj_0;

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  STATUS_SELFTEST,
  STATUS_SLEEP
} DEMO_STATUS;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256
#define INDICATION_DELAY  1000 /* LED is ON for this period [ms]. */

#define X_POWER_UP_DELAY    100 /*!< Delay after accelero power-up [ms] */
#define X_ST_ENABLED_DELAY  100 /*!< Delay after accelero self-test enabled [ms] */
#define G_POWER_UP_DELAY    150 /*!< Delay after gyro power-up [ms] */
#define G_ST_ENABLED_DELAY   50 /*!< Delay after gyro self-test enabled [ms] */

#define N_SAMPLES  5 /*!< Number of samples */

#define X_LO_LIM      90 /*!< Accelero low test limit [mg] */
#define X_HI_LIM    1700 /*!< Accelero high test limit [mg] */
#define G_LO_LIM  150000 /*!< Gyro low test limit [mdps] */
#define G_HI_LIM  700000 /*!< Gyro high test limit [mdps] */

#define ST_REG_COUNT  (sizeof(reg_addr) / sizeof(uint8_t))

#if 0
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t PushButtonDetected = 0;
static char dataOut[MAX_BUF_SIZE];
static DEMO_STATUS DemoStatus = STATUS_SLEEP;
/* Refer to Datasheet / Application Note documents for details about following register settings */
static uint8_t reg_addr[]        = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19};
static uint8_t x_st_reg_values[] = {0x38, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t g_st_reg_values[] = {0x00, 0x5C, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static int32_t PushButtonState = GPIO_PIN_RESET;

/* Private function prototypes -----------------------------------------------*/
static void MX_IKS01A3_LSM6DSO_SelfTest_Init(void);
static void MX_IKS01A3_LSM6DSO_SelfTest_Process(void);
static void Sleep_Mode(void);
static int32_t LSM6DSO_X_SelfTest(void);
static int32_t LSM6DSO_G_SelfTest(void);
static int32_t LSM6DSO_X_Get_Data(IKS01A3_MOTION_SENSOR_Axes_t *data);
static int32_t LSM6DSO_G_Get_Data(IKS01A3_MOTION_SENSOR_Axes_t *data);

void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */

  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  MX_IKS01A3_LSM6DSO_SelfTest_Init();

  /* USER CODE BEGIN MEMS_Init_PostTreatment */

  /* USER CODE END MEMS_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_MEMS_Process(void)
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */

  /* USER CODE END MEMS_Process_PreTreatment */

  MX_IKS01A3_LSM6DSO_SelfTest_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */

  /* USER CODE END MEMS_Process_PostTreatment */
}

/**
  * @brief  Initialize the LSM6DSO Self Test application
  * @retval None
  */
void MX_IKS01A3_LSM6DSO_SelfTest_Init(void)
{
  /* Initialize LED */
  BSP_LED_Init(LED2);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Check what is the Push Button State when the button is not pressed. It can change across families */
  PushButtonState = (BSP_PB_GetState(BUTTON_KEY)) ?  0 : 1;

  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  (void)IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO);

  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n------ LSM6DSO self-test DEMO ------\r\n");
  printf("%s", dataOut);
}



/**
  * @brief  Process of the LSM6DSO Self Test application
  * @retval None
  */
void MX_IKS01A3_LSM6DSO_SelfTest_Process(void)
{
  if (PushButtonDetected != 0U)
  {
    /* Debouncing */
    HAL_Delay(50);

    /* Wait until the button is released */
    while ((BSP_PB_GetState( BUTTON_KEY ) == PushButtonState));

    /* Debouncing */
    HAL_Delay(50);

    /* Reset Interrupt flag */
    PushButtonDetected = 0;

    /* _NOTE_: Pushing button creates interrupt/event and wakes up MCU from sleep mode */
    DemoStatus = STATUS_SELFTEST;
  }

  /* Handle DEMO State Machine */
  switch (DemoStatus)
  {
    case STATUS_SELFTEST:
      if (LSM6DSO_X_SelfTest() != BSP_ERROR_NONE)
      {
        Error_Handler((uint8_t *)__FILE__, __LINE__);
      }
      if (LSM6DSO_G_SelfTest() != BSP_ERROR_NONE)
      {
        Error_Handler((uint8_t *)__FILE__, __LINE__);
      }
      DemoStatus = STATUS_SLEEP;
      break;

    case STATUS_SLEEP:
      (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress USER button to start the DEMO ...\r\n");
      printf("%s", dataOut);
      /* Enter sleep mode */
      Sleep_Mode();
      break;

    default:
      Error_Handler((uint8_t *)__FILE__, __LINE__);
      break;
  }
}

/**
  * @brief  Enter sleep mode and wait for interrupt
  * @retval None
  */
static void Sleep_Mode(void)
{
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; /* Systick IRQ OFF */
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; /* Systick IRQ ON */
}

/**
  * @brief  Performs LSM6DSO accelerometer self-test
  * @retval BSP status
  */
static int32_t LSM6DSO_X_SelfTest(void)
{
  int32_t test_result = BSP_ERROR_NONE;
  uint32_t i;
  IKS01A3_MOTION_SENSOR_Axes_t data_nost;
  IKS01A3_MOTION_SENSOR_Axes_t data_st;
  IKS01A3_MOTION_SENSOR_Axes_t data;
  uint8_t prev_reg_values[ST_REG_COUNT];
  int32_t ret;

  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nStarting LSM6DSO accelerometer self-test ...\r\nKeep the device still!!!\r\n");
  printf("%s", dataOut);

  HAL_Delay(INDICATION_DELAY);
  BSP_LED_On(LED2);

  /* Store current settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = IKS01A3_MOTION_SENSOR_Read_Register(IKS01A3_LSM6DSO_0, reg_addr[i], &prev_reg_values[i])) != BSP_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Set the sensor for self-test */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = IKS01A3_MOTION_SENSOR_Write_Register(IKS01A3_LSM6DSO_0, reg_addr[i], x_st_reg_values[i])) != BSP_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Wait defined time for stable output */
  HAL_Delay(X_POWER_UP_DELAY);

  /* Read first data and discard it */
  if (LSM6DSO_X_Get_Data(&data) != BSP_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }

  data_nost.x = 0;
  data_nost.y = 0;
  data_nost.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
    if (LSM6DSO_X_Get_Data(&data) != BSP_ERROR_NONE)
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
  if ((ret = IKS01A3_MOTION_SENSOR_Set_SelfTest(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, (uint8_t)LSM6DSO_XL_ST_POSITIVE)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  /* Wait defined time for stable output */
  HAL_Delay(X_ST_ENABLED_DELAY);

  /* Read first data and discard it */
  if (LSM6DSO_X_Get_Data(&data) != BSP_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  data_st.x = 0;
  data_st.y = 0;
  data_st.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
    if (LSM6DSO_X_Get_Data(&data) != BSP_ERROR_NONE)
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
    if ((ret = IKS01A3_MOTION_SENSOR_Write_Register(IKS01A3_LSM6DSO_0, reg_addr[i], prev_reg_values[i])) != BSP_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Evaluate the test */
  if (abs(data_st.x - data_nost.x) < X_LO_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.x - data_nost.x) > X_HI_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.y - data_nost.y) < X_LO_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.y - data_nost.y) > X_HI_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.z - data_nost.z) < X_LO_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.z - data_nost.z) > X_HI_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }

  /* Print measured data */
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nMeasured acceleration [mg]:\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n     AXIS     | PRE-SELFTEST |   SELFTEST\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       X      | %8ld     | %8ld\r\n", (long)data_nost.x, (long)data_st.x);
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       Y      | %8ld     | %8ld\r\n", (long)data_nost.y, (long)data_st.y);
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       Z      | %8ld     | %8ld\r\n", (long)data_nost.z, (long)data_st.z);
  printf("%s", dataOut);

  /* Print test limits and data */
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nTest limits and data [mg]:\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n  LOW LIMIT   |  DIFFERENCE  |  HIGH LIMIT\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "%8d      | %8d     | %8d\r\n", X_LO_LIM, (int)abs(data_st.x - data_nost.x), X_HI_LIM);
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "%8d      | %8d     | %8d\r\n", X_LO_LIM, (int)abs(data_st.y - data_nost.y), X_HI_LIM);
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "%8d      | %8d     | %8d\r\n", X_LO_LIM, (int)abs(data_st.z - data_nost.z), X_HI_LIM);
  printf("%s", dataOut);

  /* Print the test result */
  if (test_result == BSP_ERROR_NONE)
  {
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSO accelerometer self-test PASSED!\r\n");
  }
  else
  {
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSO accelerometer self-test FAILED!\r\n");
  }
  printf("%s", dataOut);

  BSP_LED_Off(LED2);

  return ret;
}

/**
  * @brief  Performs LSM6DSO gyroscope self-test
  * @retval BSP status
  */
static int32_t LSM6DSO_G_SelfTest(void)
{
  uint32_t i;
  int32_t test_result = BSP_ERROR_NONE;
  IKS01A3_MOTION_SENSOR_Axes_t data_nost;
  IKS01A3_MOTION_SENSOR_Axes_t data_st;
  IKS01A3_MOTION_SENSOR_Axes_t data;
  uint8_t prev_reg_values[ST_REG_COUNT];
  int32_t ret;

  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nStarting LSM6DSO gyroscope self-test ...\r\nKeep the device still!!!\r\n");
  printf("%s", dataOut);

  HAL_Delay(INDICATION_DELAY);
  BSP_LED_On(LED2);

  /* Store current settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = IKS01A3_MOTION_SENSOR_Read_Register(IKS01A3_LSM6DSO_0, reg_addr[i], &prev_reg_values[i])) != BSP_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Set the sensor for self-test */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = IKS01A3_MOTION_SENSOR_Write_Register(IKS01A3_LSM6DSO_0, reg_addr[i], g_st_reg_values[i])) != BSP_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Wait defined time for stable output */
  HAL_Delay(G_POWER_UP_DELAY);

  /* Read first data and discard it */
  if (LSM6DSO_G_Get_Data(&data) != BSP_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }

  data_nost.x = 0;
  data_nost.y = 0;
  data_nost.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
    if (LSM6DSO_G_Get_Data(&data) != BSP_ERROR_NONE)
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
  if ((ret = IKS01A3_MOTION_SENSOR_Set_SelfTest(IKS01A3_LSM6DSO_0, MOTION_GYRO, (uint8_t)LSM6DSO_GY_ST_POSITIVE)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  /* Wait defined time for stable output */
  HAL_Delay(G_ST_ENABLED_DELAY);

  /* Read first data and discard it */
  if (LSM6DSO_G_Get_Data(&data) != BSP_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  data_st.x = 0;
  data_st.y = 0;
  data_st.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
    if (LSM6DSO_G_Get_Data(&data) != BSP_ERROR_NONE)
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
    if ((ret = IKS01A3_MOTION_SENSOR_Write_Register(IKS01A3_LSM6DSO_0, reg_addr[i], prev_reg_values[i])) != BSP_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Evaluate the test */
  if (abs(data_st.x - data_nost.x) < G_LO_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.x - data_nost.x) > G_HI_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.y - data_nost.y) < G_LO_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.y - data_nost.y) > G_HI_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.z - data_nost.z) < G_LO_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }
  if (abs(data_st.z - data_nost.z) > G_HI_LIM)
  {
    test_result = BSP_ERROR_COMPONENT_FAILURE;
  }

  /* Print measured data */
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nMeasured angular velocity [mdps]:\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n     AXIS     | PRE-SELFTEST |   SELFTEST\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       X      |  %8ld    |  %8ld\r\n", (long)data_nost.x, (long)data_st.x);
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       Y      |  %8ld    |  %8ld\r\n", (long)data_nost.y, (long)data_st.y);
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       Z      |  %8ld    |  %8ld\r\n", (long)data_nost.z, (long)data_st.z);
  printf("%s", dataOut);

  /* Print test limits and data */
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nTest limits and data [mdps]:\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n  LOW LIMIT   |  DIFFERENCE  |  HIGH LIMIT\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "  %8d    |  %8d    |  %8d\r\n", G_LO_LIM, (int)abs(data_st.x - data_nost.x), G_HI_LIM);
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "  %8d    |  %8d    |  %8d\r\n", G_LO_LIM, (int)abs(data_st.y - data_nost.y), G_HI_LIM);
  printf("%s", dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "  %8d    |  %8d    |  %8d\r\n", G_LO_LIM, (int)abs(data_st.z - data_nost.z), G_HI_LIM);
  printf("%s", dataOut);

  /* Print the test result */
  if (test_result == BSP_ERROR_NONE)
  {
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSO gyroscope self-test PASSED!\r\n");
  }
  else
  {
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSO gyroscope self-test FAILED!\r\n");
  }
  printf("%s", dataOut);

  BSP_LED_Off(LED2);

  return ret;
}

/**
  * @brief  Wait for data ready and get data
  * @param  data the sensor data
  * @retval None
  */
static int32_t LSM6DSO_X_Get_Data(IKS01A3_MOTION_SENSOR_Axes_t *data)
{
  uint8_t status;
  int32_t ret;

  /* Wait for data ready */
  do
  {
    if ((ret = IKS01A3_MOTION_SENSOR_Get_DRDY_Status(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, &status)) != BSP_ERROR_NONE)
    {
      return ret;
    }
  }
  while (status == 0U);

  /* Read accelero data */
  if ((ret = IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, data)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  return ret;
}

/**
  * @brief  Wait for data ready and get data
  * @param  data the sensor data
  * @retval BSP status
  */
static int32_t LSM6DSO_G_Get_Data(IKS01A3_MOTION_SENSOR_Axes_t *data)
{
  uint8_t status;
  int32_t ret;

  /* Wait for data ready */
  do
  {
    if ((ret = IKS01A3_MOTION_SENSOR_Get_DRDY_Status(IKS01A3_LSM6DSO_0, MOTION_GYRO, &status)) != BSP_ERROR_NONE)
    {
      return ret;
    }
  }
  while (status == 0U);

  /* Read accelero data */
  if ((ret = IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LSM6DSO_0, MOTION_GYRO, data)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  return ret;
}
#endif

/**
 * @brief LSM6DSO_hw_test
 * @brief harware validation function
 * @return system error
 */
int LSM6DSO_hw_test(void * arg)
{
	LSM6DSO_IO_t            io_ctx;
	uint8_t                 id=0;

	/* Configure the accelero driver */
	io_ctx.BusType     = LSM6DSO_I2C_BUS; /* I2C */
	io_ctx.Address     = LSM6DSO_I2C_ADD_L;
	io_ctx.Init        = BUS_I2C2_Init;
	io_ctx.DeInit      = BUS_I2C2_DeInit;
	io_ctx.ReadReg     = BUS_I2C2_ReadReg;
	io_ctx.WriteReg    = BUS_I2C2_WriteReg;
	io_ctx.GetTick     = BUS_GetTick;

	if (LSM6DSO_RegisterBusIO(&lsm6dso_obj_0, &io_ctx) != LSM6DSO_OK)
	{
	  return SYS_ERROR_BUS_FAILURE;
	}

	/* wait for bus will be ready before reading */
	while(BUS_I2C2_IsReady(LSM6DSO_I2C_ADD_L, 10000))
	{
		vTaskDelay(25);
	}

	/* read sensor revision register */
	if (LSM6DSO_ReadID(&lsm6dso_obj_0, &id) != LIS2DW12_OK)
	{
		return SYS_ERROR_BUS_FAILURE;
	}

    /* check revision  */
	if (id != LSM6DSO_ID)
	{
		return SYS_ERROR_BUS_FAILURE;
	}


//  else if (LSM6DSO_ReadID(&lsm6dso_obj_0, &id) != LSM6DSO_OK)
//  {
//	ret = BSP_ERROR_UNKNOWN_COMPONENT;
//  }
//  else if (id != LSM6DSO_ID)
//  {
//	ret = BSP_ERROR_UNKNOWN_COMPONENT;
//  }



#if 0


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
			while(BUS_I2C2_IsReady(LIS2DW12_I2C_ADD_L, 10000))
			{
				vTaskDelay(25);
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
			while(BUS_I2C2_IsReady(LIS2DW12_I2C_ADD_H, 10000))
			{
				vTaskDelay(25);
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
#endif
	return SYS_ERROR_NONE;
}/* End of LIS2DW_hw_test */

#ifdef __cplusplus
}
#endif








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

	aux_dbg_printLogStr("\r\n[BIT LIS2DW] start \r\n");
	vTaskDelay(25);

	/* wait for bus will be ready before reading */
	if (LIS2DW12_RegisterBusIO(&lis2dw12_obj_0, &io_ctx_0) != LIS2DW12_OK)
	{
	  aux_dbg_printLogStr("\r\n[BIT LIS2DW] id 0 BUS FAILED \r\n");
	  return;
	}

	/* wait for bus will be ready before reading */
	if (LIS2DW12_RegisterBusIO(&lis2dw12_obj_1, &io_ctx_1) != LIS2DW12_OK)
	{
	  aux_dbg_printLogStr("\r\n[BIT LIS2DW] id 1 BUS FAILED \r\n");
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
	  aux_dbg_printLogStr("\r\n[BIT LIS2DW]  id 0 FAILED \r\n");

	/* read sensor revision register */
	if (LIS2DW12_ReadID(&lis2dw12_obj_1, &id_1) != LIS2DW12_OK)
	  aux_dbg_printLogStr("\r\n[BIT LIS2DW]  id 1 FAILED \r\n");

    /* check revision acc #1 */
	if (id_0 == LIS2DW12_ID)
	{
		sprintf(res_buff,"\r\n[BIT LIS2DW] id 0 [%X]hex \r\n",id_0);
		aux_dbg_printLogStr((const char *)res_buff);
	}
	else aux_dbg_printLogStr("\r\n[BIT LIS2DW] id 0 FAILED \r\n");

    /* check revision acc #1 */
	if (id_1 == LIS2DW12_ID)
	{
		sprintf(res_buff,"\r\n[BIT LIS2DW] id 1 [%X]hex \r\n",id_1);
		aux_dbg_printLogStr((const char *)res_buff);
	}
	else aux_dbg_printLogStr("\r\n[BIT LIS2DW] id 1 FAILED \r\n");

	/* total results */
	if((id_1 == LIS2DW12_ID) && (id_0 == LIS2DW12_ID))
	{
		aux_dbg_printLogStr("\r\n[BIT LIS2DW] OK \r\n");
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

#endif


