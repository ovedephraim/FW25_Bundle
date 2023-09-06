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
#include "Freertos.h"
#include <task.h>
#include <queue.h>

#include "main.h"
#include "MAX30003_bus.h"
#include "MAX30003_dev.h"
#include "bus.h"

#define MOD_NAME "[MAX30003 DEV_BIT]"
extern MAX30003_Object_t max30003_obj_0;


/**
 * @brief MAX30003_hw_test
 * @brief hardware validation function
 * versions: 0 = 30004  1 = 30001  2 = 30002  3 = 30003
 * @return system error
 */
int MAX30003_hw_test(void * arg)
{
	uint32_t id=0xFF;
	int partVersion=0xFF;

	/* spi bus registration */
	if (MAX30003_RegisterBusIO(&max30003_obj_0) != MAX30003_OK){
		return SYS_ERROR_BUS_FAILURE;
	}

	/* read part version from the chip */
	MAX30003_readID(&max30003_obj_0,&id);

	partVersion = id >> 12;
	partVersion = partVersion & 0x3;

	/* Version validation */
	if (partVersion != 3){
		return SYS_ERROR_BUS_FAILURE;

	}

	return SYS_ERROR_NONE;
}/* end of MAX30003_hw_test */



