
#include <stdio.h>
#include <string.h>

#include <main.h>
#include "MAX3000N.h"
#include "auxcmd.h"  //logs

#include "MAX30001_bus.h"
#include "MAX30003_bus.h"

extern MAX30003_Object_t max30003_obj_0;
extern MAX30001_Object_t max30001_obj_0;



uint8_t conv_odr_bioz_val(uint16_t val)
{
	uint8_t rate=0b1;//32 sps;

	if((val > 32))
	{
		rate=0b00;//64 sps
	}

	return rate;
}/* end of conv_odr_val */

uint8_t conv_odr_val(uint16_t val)
{
	uint8_t rate=0b00;//-512;

	if((val > 0) && (val <= 128))
	{
		rate=0b10;//-128
	}
	else
	if((val > 128) && (val <= 256))
	{
		rate=0b01;//-256
	}

	return rate;
}/* end of conv_odr_val */

static uint8_t ecg_spi_read (MAX30001_Object_t *pObj,MAX30003_Object_t *pObj1,
		uint8_t command, uint32_t *data_out)
{
	int rv=0;

	if(pObj && (pObj->ecg_is_initialized || pObj->biz_is_initialized))
	{
		//CS pin low
		if(pObj && (pObj->ecg_is_initialized || pObj->biz_is_initialized)) HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if(pObj1 && pObj1->is_initialized) HAL_GPIO_WritePin(pObj1->IO.csPort, pObj1->IO.csPin.Pin, GPIO_PIN_RESET);

		rv=MAX30001_reg_read(&(pObj->Ctx),(uint32_t)command, data_out);

		//CS pin high
		if(pObj && (pObj->ecg_is_initialized || pObj->biz_is_initialized)) HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
		if(pObj1 && pObj1->is_initialized) HAL_GPIO_WritePin(pObj1->IO.csPort, pObj1->IO.csPin.Pin, GPIO_PIN_SET);
	}
	else
	if(pObj1 && pObj1->is_initialized)
	{
		//CS pin low
		if(pObj && (pObj->ecg_is_initialized || pObj->biz_is_initialized)) HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if(pObj1 && pObj1->is_initialized) HAL_GPIO_WritePin(pObj1->IO.csPort, pObj1->IO.csPin.Pin, GPIO_PIN_RESET);

		rv=MAX30003_reg_read(&(pObj1->Ctx),(uint32_t)command, data_out);

		//CS pin high
		if(pObj && (pObj->ecg_is_initialized || pObj->biz_is_initialized)) HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
		if(pObj1 && pObj1->is_initialized) HAL_GPIO_WritePin(pObj1->IO.csPort, pObj1->IO.csPin.Pin, GPIO_PIN_SET);
	}
	else
	{
		return -1;
	}

return rv;
}/* end of ecg_spi_read */

static uint8_t ecg_spi_write(MAX30001_Object_t *pObj,MAX30003_Object_t *pObj1,
		uint8_t command, uint32_t data_in)
{
    int rv=0;

	if(pObj && (pObj->ecg_is_initialized || pObj->biz_is_initialized))
	{
		//CS pin low
		if(pObj && (pObj->ecg_is_initialized || pObj->biz_is_initialized)) HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if(pObj1 && pObj1->is_initialized) HAL_GPIO_WritePin(pObj1->IO.csPort, pObj1->IO.csPin.Pin, GPIO_PIN_RESET);

		rv=MAX30001_reg_write(&(pObj->Ctx),(uint32_t)command, data_in);

		//CS pin high
		if(pObj && (pObj->ecg_is_initialized || pObj->biz_is_initialized))  HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
		if(pObj1 && pObj1->is_initialized) HAL_GPIO_WritePin(pObj1->IO.csPort, pObj1->IO.csPin.Pin, GPIO_PIN_SET);
	}
	else
	if(pObj1 && pObj1->is_initialized)
	{
		//CS pin low
		if(pObj && (pObj->ecg_is_initialized || pObj->biz_is_initialized)) HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if(pObj1 && pObj1->is_initialized) HAL_GPIO_WritePin(pObj1->IO.csPort, pObj1->IO.csPin.Pin, GPIO_PIN_RESET);

		rv=MAX30003_reg_write(&(pObj1->Ctx),(uint32_t)command, data_in);

		//CS pin high
		if(pObj && (pObj->ecg_is_initialized || pObj->biz_is_initialized)) HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
		if(pObj1 && pObj1->is_initialized) HAL_GPIO_WritePin(pObj1->IO.csPort, pObj1->IO.csPin.Pin, GPIO_PIN_SET);
	}
	else
	{
		return -1;
	}
return rv;
}/* end of ecg_spi_write */

/**
* @brief For MAX30001/3 ONLY
* @brief This function enables the ECG.
* @brief Uses Register CNFG_GEN-0x10.
* @returns 0-if no error.  A non-zero value indicates an error.
*
*/
int32_t MAX3000N_Start_ECG(void) {

	max30001_cnfg_gen_t cnfg_gen;
	max30001_status_t    status;
	uint32_t max30001_timeout;
	MAX30001_Object_t *pObj =&max30001_obj_0;
	MAX30003_Object_t *pObj1=&max30003_obj_0;

	ecg_spi_read(pObj,pObj1, CNFG_GEN,  &cnfg_gen.all);
	if(!cnfg_gen.bit.en_ecg)
	{
		cnfg_gen.bit.en_ecg = 0b1;
		ecg_spi_write(pObj,pObj1, CNFG_GEN, cnfg_gen.all);

		/* Wait for PLL Lock & References to settle down */
		max30001_timeout = 0;
		do
		{
			ecg_spi_read(pObj,pObj1, STATUS,  &status.all);
		} while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);

		ecg_spi_write(pObj,pObj1, FIFO_RST, 0x000000);
		ecg_spi_write(pObj,pObj1, SYNCH, 0x000000);
	}
return MAX3000N_OK;
}/* End of MAX30001_Start_ECG */

