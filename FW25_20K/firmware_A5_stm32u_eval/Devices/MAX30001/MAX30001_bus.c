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
#include <string.h>
#include <limits.h>
#include "Freertos.h"
#include <stdint.h>
#include <freertos.h>
#include <task.h>
#include <queue.h>
#include "stm32u5xx_hal.h"

#include "MAX30001_dev.h"
#include "MAX30001_bus.h"
#include "bus.h"
#include "main.h"

#define MOD_NAME "[MAX30001 BUS]"


/**
 * @brief  Wrap Read register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t ReadRegWrap(void *Handle, uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size)
{
  MAX30001_Object_t *pObj = (MAX30001_Object_t *)Handle;
  return pObj->IO.Read(tx_buf, tx_size, rx_buf,rx_size);
}

/**
 * @brief  Wrap Write register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t WriteRegWrap(void *Handle, uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size)
{
  MAX30001_Object_t *pObj = (MAX30001_Object_t *)Handle;
  return pObj->IO.Write(tx_buf, tx_size, rx_buf,rx_size);
}


int MAX30001_reg_read(maxdev_ctx_t *ctx, MAX30001_REG_map_t addr,uint32_t *return_data)
{
    uint8_t result[4];
    uint8_t data_array[1];
    int32_t success = 0;

    data_array[0] = ((addr << 1) & 0xff) | 1; // For Read, Or with 1

    success = ctx->read_reg(ctx->handle, data_array, 1, result, 4);

    *return_data = (uint32_t)(result[1] << 16) +
                   (result[2] << 8) + result[3];
    if (success != 0) {
      return -1;
    } else {
      return 0;
    }
}

int MAX30001_reg_write(maxdev_ctx_t *ctx, MAX30001_REG_map_t addr, uint32_t data)
{
    uint8_t result[4];
    uint8_t data_array[4];
    int32_t ret = 0;

    data_array[0] = (addr << 1) & 0xff;
    data_array[3] =  data & 0xff;
    data_array[2] = (data >> 8) & 0xff;
    data_array[1] = (data >> 16) & 0xff;

    ret = ctx->write_reg(ctx->handle, data_array, 4, result, 4);

    if (ret != 0) {
      return -1;
    } else {
      return 0;
    }
}


int32_t MAX30001_RegisterBusIO(MAX30001_Object_t *pObj)
{
	int32_t ret = MAX30001_OK;

	if (pObj == NULL)
	{
		ret = MAX30001_ERROR;
	}
	else
	{
	   /* assign cs io parameters and init gpio */
		pObj->IO.csPin.Mode      = MAX30001_CS_GPIO_MODE;
		pObj->IO.csPin.Pin       = MAX30001_CS_PIN;
		pObj->IO.csPin.Pull      = MAX30001_CS_GPIO_PULL;
		pObj->IO.csPin.Speed     = MAX30001_CS_GPIO_SPEED;
		pObj->IO.csPort          = MAX30001_CS_GPIO_PORT;

		pObj->IO.Init      = BUS_SPI2_Init;
		pObj->IO.DeInit    = BUS_SPI2_DeInit;
		pObj->IO.Write     = BUS_SPI2_Write;
		pObj->IO.Read      = BUS_SPI2_Read;
		pObj->IO.GetTick   = BUS_GetTick;

		pObj->Ctx.read_reg  = (maxdev_read_ptr)ReadRegWrap;
		pObj->Ctx.write_reg = (maxdev_write_ptr)WriteRegWrap;
		pObj->Ctx.handle   = pObj;

		if (pObj->IO.Init == NULL)
		{
			ret = MAX30001_ERROR;
		}
		else if (pObj->IO.Init() != MAX30001_OK)
		{
			ret = MAX30001_ERROR;
		}
	}

return ret;
}/* end of MAX30001_RegisterBusIO */



