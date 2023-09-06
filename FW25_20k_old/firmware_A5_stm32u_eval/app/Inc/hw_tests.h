/**************************************************************************//**
* @file hw_test.h
* @brief hardware validation tests
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef HW_TESTS_H_
#define HW_TESTS_H_

typedef enum hw_dev
{
	LIS2TWD1_DEV =0,
	LIS2TWD2_DEV =1,
	MAX30001_DEV =2,
	LSM6DSO_DEV  =3,
	BL653_DEV    =4,
	MCP4353_DEV  =5,
	MAX17303_DEV =6,
	MAX30003_DEV =7,
	TMP117_DEV1  =8,
	TMP117_DEV2  =9,
	TMP117_DEV3  =10,
	MAX_DEV
}hw_dev_t;

int hw_tester_test(hw_dev_t dev);

#endif /* HW_TESTS_H_ */
