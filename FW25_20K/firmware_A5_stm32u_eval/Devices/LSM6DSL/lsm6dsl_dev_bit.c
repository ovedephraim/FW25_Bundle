

#ifdef __cplusplus
extern "C" {
#endif



#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <freertos.h>
#include <task.h>
#include <queue.h>

#include "main.h"
#include "bus.h"

#include "lsm6dsl_dev.h"
#include "bus.h"

extern LSM6DSL_Object_t lsm6dsl_obj;

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

#define GYRO_SELFTEST_LOG 0U
#define ACC_SELFTEST_LOG  0U

/* Private variables ---------------------------------------------------------*/
#if(GYRO_SELFTEST_LOG || ACC_SELFTEST_LOG)
static char dataOut[MAX_BUF_SIZE];
#endif

/* Refer to Datasheet / Application Note documents for details about following register settings */
static uint8_t reg_addr[]        = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19};
static uint8_t x_st_reg_values[] = {0x38, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t g_st_reg_values[] = {0x00, 0x5C, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Wait for data ready and get data
  * @param  data the sensor data
  * @retval BSP status
  */
static int32_t LSM6DSL_G_Get_Data(LSM6DSL_Object_t * p_obj, LSM6DSL_Axes_t *data)
{
  uint8_t status;
  int32_t ret;

  /* Wait for data ready */
  do
  {
    if ((ret = LSM6DSL_GYRO_Get_DRDY_Status(p_obj,  &status)) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }
  while (status == 0U);

  /* Read accelero data */
  if ((ret = LSM6DSL_GYRO_GetAxes(p_obj, data)) != SYS_ERROR_NONE)
  {
    return ret;
  }

  return ret;
}/* end of LSM6DSL_G_Get_Data */

/**
  * @brief  Performs LSM6DSL gyroscope self-test
  * @retval BSP status
  */
static int32_t LSM6DSL_G_SelfTest(LSM6DSL_Object_t * p_obj)
{
  int32_t test_result = SYS_ERROR_NONE;
  LSM6DSL_Axes_t data_nost;
  LSM6DSL_Axes_t data_st;
  LSM6DSL_Axes_t data;
  uint8_t prev_reg_values[ST_REG_COUNT];
  int32_t ret;
  uint32_t i;

  /* Store current settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = LSM6DSL_Read_Reg(p_obj, reg_addr[i], &prev_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Set the sensor for self-test */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = LSM6DSL_Write_Reg(p_obj, reg_addr[i], g_st_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }


  /* Wait defined time for stable output */
  vTaskDelay(G_POWER_UP_DELAY);

  /* Read first data and discard it */
  if (LSM6DSL_G_Get_Data(p_obj, &data) != SYS_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }

  data_nost.x = 0;
  data_nost.y = 0;
  data_nost.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
	if (LSM6DSL_G_Get_Data(p_obj, &data) != SYS_ERROR_NONE)
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
  if ((ret = LSM6DSL_GYRO_Set_SelfTest(p_obj, (uint8_t)LSM6DSL_GY_ST_POSITIVE)) != SYS_ERROR_NONE)
  {
    return ret;
  }

  /* Wait defined time for stable output */
  vTaskDelay(G_ST_ENABLED_DELAY);

  /* Read first data and discard it */
  if (LSM6DSL_G_Get_Data(p_obj, &data) != SYS_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  data_st.x = 0;
  data_st.y = 0;
  data_st.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
	if (LSM6DSL_G_Get_Data(p_obj, &data) != SYS_ERROR_NONE)
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
	if ((ret = LSM6DSL_Write_Reg(p_obj, reg_addr[i], prev_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Evaluate the test */
  if (abs(data_st.x - data_nost.x) < G_LO_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }
  if (abs(data_st.x - data_nost.x) > G_HI_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }
  if (abs(data_st.y - data_nost.y) < G_LO_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }
  if (abs(data_st.y - data_nost.y) > G_HI_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }
  if (abs(data_st.z - data_nost.z) < G_LO_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }
  if (abs(data_st.z - data_nost.z) > G_HI_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }

  /* Disable self-test */
  if ((ret = LSM6DSL_GYRO_Set_SelfTest(p_obj, (uint8_t)LSM6DSL_GY_ST_DISABLE)) != SYS_ERROR_NONE)
  {
    return ret;
  }

#if defined ( GYRO_SELFTEST_LOG ) && ( GYRO_SELFTEST_LOG )
  /* Print measured data */
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nMeasured angular velocity [mdps]:\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n     AXIS     | PRE-SELFTEST |   SELFTEST\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       X      |  %8ld    |  %8ld\r\n", (long)data_nost.x, (long)data_st.x);
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       Y      |  %8ld    |  %8ld\r\n", (long)data_nost.y, (long)data_st.y);
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       Z      |  %8ld    |  %8ld\r\n", (long)data_nost.z, (long)data_st.z);
  aux_dbg_printLogStr(dataOut);

  /* Print test limits and data */
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nTest limits and data [mdps]:\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n  LOW LIMIT   |  DIFFERENCE  |  HIGH LIMIT\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "  %8d    |  %8d    |  %8d\r\n", G_LO_LIM, (int)abs(data_st.x - data_nost.x), G_HI_LIM);
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "  %8d    |  %8d    |  %8d\r\n", G_LO_LIM, (int)abs(data_st.y - data_nost.y), G_HI_LIM);
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "  %8d    |  %8d    |  %8d\r\n", G_LO_LIM, (int)abs(data_st.z - data_nost.z), G_HI_LIM);
  aux_dbg_printLogStr(dataOut);

  /* Print the test result */
  if (test_result == SYS_ERROR_NONE)
  {
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSL gyroscope self-test PASSED!\r\n");
  }
  else
  {
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSL gyroscope self-test FAILED!\r\n");
  }
  aux_dbg_printLogStr(dataOut);
#endif

  ret=test_result;
  return ret;
}

/**
  * @brief  Wait for data ready and get data
  * @param  data the sensor data
  * @retval None
  */
static int32_t LSM6DSL_X_Get_Data(LSM6DSL_Object_t * p_obj, LSM6DSL_Axes_t *data)
{
  uint8_t status;
  int32_t ret;

  /* Wait for data ready */
  do
  {
    if ((ret = LSM6DSL_ACC_Get_DRDY_Status(p_obj,  &status)) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }
  while (status == 0U);

  /* Read accelero data */
  if ((ret = LSM6DSL_ACC_GetAxes(p_obj, data)) != SYS_ERROR_NONE)
  {
    return ret;
  }

  return ret;
}

/**
  * @brief  Performs LSM6DSL accelerometer self-test
  * @retval BSP status
  */
static int32_t LSM6DSL_X_SelfTest(LSM6DSL_Object_t * p_obj)
{
  int32_t test_result = SYS_ERROR_NONE;
  LSM6DSL_Axes_t data_nost;
  LSM6DSL_Axes_t data_st;
  LSM6DSL_Axes_t data;
  uint8_t prev_reg_values[ST_REG_COUNT];
  int32_t ret;
  uint32_t i;


  /* Store current settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = LSM6DSL_Read_Reg(p_obj, reg_addr[i], &prev_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Set the sensor for self-test */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if ((ret = LSM6DSL_Write_Reg(p_obj, reg_addr[i], x_st_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Wait defined time for stable output */
  vTaskDelay(X_POWER_UP_DELAY);

  /* Read first data and discard it */
  if (LSM6DSL_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }

  data_nost.x = 0;
  data_nost.y = 0;
  data_nost.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
    if (LSM6DSL_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
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
  if ((ret = LSM6DSL_ACC_Set_SelfTest(p_obj, (uint8_t)LSM6DSL_XL_ST_POSITIVE)) != SYS_ERROR_NONE)
  {
    return ret;
  }

  /* Wait defined time for stable output */
  vTaskDelay(X_ST_ENABLED_DELAY);

  /* Read first data and discard it */
  if (LSM6DSL_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
  {
    Error_Handler((uint8_t *)__FILE__, __LINE__);
  }
  data_st.x = 0;
  data_st.y = 0;
  data_st.z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < (uint32_t)N_SAMPLES; i++)
  {
    if (LSM6DSL_X_Get_Data(p_obj,&data) != SYS_ERROR_NONE)
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
    if ((ret = LSM6DSL_Write_Reg(p_obj, reg_addr[i], prev_reg_values[i])) != SYS_ERROR_NONE)
    {
      return ret;
    }
  }

  /* Evaluate the test */
  if (abs(data_st.x - data_nost.x) < X_LO_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }
  if (abs(data_st.x - data_nost.x) > X_HI_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }
  if (abs(data_st.y - data_nost.y) < X_LO_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }
  if (abs(data_st.y - data_nost.y) > X_HI_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }
  if (abs(data_st.z - data_nost.z) < X_LO_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }
  if (abs(data_st.z - data_nost.z) > X_HI_LIM)
  {
    test_result = SYS_ERROR_SELF_TEST_FAILURE;
  }

  /* Disable self-test */
  if ((ret = LSM6DSL_ACC_Set_SelfTest(p_obj, (uint8_t)LSM6DSL_XL_ST_DISABLE)) != SYS_ERROR_NONE)
  {
    return ret;
  }

#if defined ( ACC_SELFTEST_LOG ) && ( ACC_SELFTEST_LOG )
  /* Print measured data */
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nMeasured acceleration [mg]:\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n     AXIS     | PRE-SELFTEST |   SELFTEST\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       X      | %8ld     | %8ld\r\n", (long)data_nost.x, (long)data_st.x);
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       Y      | %8ld     | %8ld\r\n", (long)data_nost.y, (long)data_st.y);
  aux_dbg_printLogStr(dataOut);
  (void)snprintf(dataOut, MAX_BUF_SIZE, "       Z      | %8ld     | %8ld\r\n", (long)data_nost.z, (long)data_st.z);
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
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSL accelerometer self-test PASSED!\r\n");
  }
  else
  {
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSL accelerometer self-test FAILED!\r\n");
  }

  aux_dbg_printLogStr(dataOut);

#endif


  ret=test_result;
  return ret;
}

/**
 * @brief LSM6DSL_hw_test
 * @brief harware validation function
 * @return system error
 */
int LSM6DSL_hw_test(void * arg)
{
	LSM6DSL_IO_t            io_ctx;
	uint8_t                 id=0;

	/* Configure the accelero driver */
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
	while(BUS_I2C2_IsReady(LSM6DSL_I2C_ADD_L, 10000))
	{
		vTaskDelay(25);
	}

	/* read sensor revision register */
	if (LSM6DSL_ReadID(&lsm6dsl_obj, &id) != LSM6DSL_OK)
	{
		return SYS_ERROR_BUS_FAILURE;
	}

    /* check revision  */
	if (id != LSM6DSL_ID)
	{
		return SYS_ERROR_BUS_FAILURE;
	}

	/* run accel self tests */
    if (LSM6DSL_X_SelfTest(&lsm6dsl_obj))
    {
    	return SYS_ERROR_SELF_TEST_FAILURE;
    }

    /* run gyro self tests */
    if (LSM6DSL_G_SelfTest(&lsm6dsl_obj))
    {
    	return SYS_ERROR_PERIPH_FAILURE;
    }

	return SYS_ERROR_NONE;
}/* End of LSM6DSL_hw_test */

#ifdef __cplusplus
}
#endif
