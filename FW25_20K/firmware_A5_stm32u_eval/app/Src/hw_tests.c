/**************************************************************************//**
* @file hw_test.c
* @brief hardware validation tests
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include "hw_tests.h"
#include "bt_manager_task.h"
#include "LIS2DW/lis2dw_dev.h"
#include "MAX30001/MAX30001_dev.h"
#include "MAX30003/MAX30003_dev.h"
#include "LSM6DSL/lsm6dsl_dev.h"
#include "TMP117/tmp117_dev.h"
#include "LP55281/LP55281_dev.h"
#include "sys_errno.h"


#define HW_TEST_MAX (15)

typedef int (*hw_test_ptr)(void * arg);

typedef struct hw_test_var
{
	hw_test_ptr fp;
	uint8_t arg;
}hw_test_var_t;


hw_test_var_t hw_test_map[HW_TEST_MAX]=
{
	{.fp=LIS2DW_hw_test,  .arg=0}, //0  LIS2TWD1_DEV
	{.fp=LIS2DW_hw_test,  .arg=1}, //1  LIS2TWD2_DEV
	{.fp=MAX30001_hw_test,.arg=0}, //2  MAX30001_DEV
	{.fp=LSM6DSL_hw_test, .arg=0}, //3  LSM6DSO_DEV
	{.fp=BL653_hw_test,   .arg=0}, //4  BL653_DEV
	{.fp=NULL,            .arg=0}, //5  MCP4353_DEV
	{.fp=NULL,.            arg=0}, //6  MAX17303_DEV
	{.fp=MAX30003_hw_test,.arg=0}, //7  MAX30003_DEV
	{.fp=tmp117_hw_test,  .arg=0}, //8  TMP117_1 (SKIN)
	{.fp=tmp117_hw_test,  .arg=1}, //9  TMP117_2 (CONTROL)
	{.fp=tmp117_hw_test,  .arg=2}, //10 TMP117_3 (AMBIENT)
	{.fp=LP55281_hw_test, .arg=0}  //11 LP55281 (COM)
};

int hw_tester_test(hw_dev_t dev)
{
	/* check args value */
	if(dev>=HW_TEST_MAX)
	{
		return SYS_ERROR_WRONG_PARAM;
	}

	if(hw_test_map[dev].fp==NULL)
	{
		return SYS_ERROR_WRONG_PARAM;
	}

return hw_test_map[dev].fp(&hw_test_map[dev].arg);
}/* end of hw_tester_test */







