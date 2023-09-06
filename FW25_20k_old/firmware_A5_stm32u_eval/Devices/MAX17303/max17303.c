/*
 ******************************************************************************
 * @file    lp55281_reg.c
 * @author  Sensors Software Solution Team
 * @brief   lp55281 driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */


#include <main.h>
#include <max17303.h>
#include <stddef.h>

#define false			0
#define true			1

uint16_t ini_file[112] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
						  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
						  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
						  0x00,0x00,0x00,0x00,0x2c42,0x14d,0xa562,0x4696,
						  0x3c00,0x1b80,0x0b04,0x0885,0x00,0x1388,0x08cc,0x223e,
						  0x00,0x1388,0x00,0x00,0xcdaf,0x00,0x00,0x00,
						  0x2210,0x0204,0x00,0x1388,0x00,0x1101,0x0838,0x2241,
						  0x200,0x0986,0xfe0a,0x0909,0x00,0x00,0x00,0x00,
						  0x8480,0x8780,0x00,0xde00,0x00,0x3e8,0x00,0x00,
						  0x4000,0x0025,0xee56,0x1da4,0x00,0x00,0x00,0x1388,
						  0x5084,0x3700,0x5528,0x4bb5,0xdc00,0x2d0a,0x7a28,0x0a04,
						  0x644b,0x0059,0x5054,0xc884,0xab3d,0x0eaf,0x4355,0xb8,
						  0x00,0x00,0x00,0xa5b9,0x00,0x00,0x00,0x00,
						  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

extern I2C_HandleTypeDef hi2c4;

max17303_fuelgauge  _max17303_fuelgauge;

void Init_MAX17303 (stmdev_max17303_ctx_t *a_p_max17303)
{
	a_p_max17303->read_reg = max17303_read_reg;
	a_p_max17303->write_reg = max17303_write_reg;
}

stmdev_max17303_ctx_t max17303_chnl;


void init_max17303(void)
{
	Init_MAX17303(&max17303_chnl);
}


int32_t max17303_read_reg(stmdev_max17303_ctx_t  *ctx,
								uint16_t reg,
								uint8_t *data,
								uint8_t len)
{
	int32_t ret;

	// Send configuration register data
//	ret = HAL_I2C_Master_Transmit(&hi2c4, MAX17303_I2C_ADD, &reg, 1, 50);
//
//	if(ret != HAL_OK)
//	{
//		return ret;
//	}


	// Receive voltage data, two bytes
	if(reg >= 0x100)
		ret = HAL_I2C_Mem_Read(&hi2c4,MAX17303_I2C_ADD_HIGH + 1, reg - 0x100, 1, &data[0], len, 50);
	else
	    ret = HAL_I2C_Mem_Read(&hi2c4,MAX17303_I2C_ADD + 1, reg, 1, &data[0], len, 50);

	return ret;
}


/**
  * @brief  Write generic device register
  *
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */

int32_t max17303_write_reg (stmdev_max17303_ctx_t *ctx,
								 uint16_t reg,
							     uint8_t *data,
								 uint8_t len)
{
	int32_t ret;

	if(reg >= 0x100)
		ret = HAL_I2C_Mem_Write(&hi2c4,MAX17303_I2C_ADD_HIGH, reg - 0x100, 1, &data[0], len, 50);
	else
		ret = HAL_I2C_Mem_Write(&hi2c4,MAX17303_I2C_ADD, reg, 1, &data[0], len, 50);

	return ret;
}



int32_t max17303_init_file(void)
{
	int32_t ret,temp;
	uint8_t a = 0;
	uint16_t b = 0x180;

	for(a = 0;a < 112;a++)
	{
		ret = max17303_chnl.write_reg(&max17303_chnl.handle,b,&ini_file[a],2);
		++b;
	}

	b = 0x3e8;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_FullCapRep,&b,2);

#if 0
	// nFullCapNom = 0x1a5.     500maH.
    temp = 0x3e8;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_nFullCapNom,&temp,2);

	// nFullCapRepm = 0x1a9.    500maH.
	temp = 0x3e8;
    ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_nFullCapRep,&temp,2);

	// nDesignCap = 0x1c5.		500maH.
	temp = 0x3e8;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_nDesignCap,&temp,2);

	// nRsense = 0x1cf. 		50 miliohm.
	temp = 0x1388;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_nRSense,&temp,2);

	// nTCurve = 0x1c9.
	temp = 0x0025;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_nCurve,&temp,2);
#endif
#if 0
	// nTgain = 0x1ca.
	temp = 0xee56;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_nTGain,&temp,2);

	// nToff = 0x1cb.
	temp = 0x1da4;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_nTOff,&temp,2);

	// nVEmpty = 0x19e.  VE = 3.1v -> 0x136 , VR = 3.2v - > 0x50.
	temp = 0x9b50;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_nVEmpty,&temp,2);

	// nLearnCfg = 0x19f.  default LS = 0 , bits 4,5,6.
	temp = 0x4696;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_nLearnCfg,&temp,2);

	// nPackCfg = 0x1b5. NTC = 10k..
	temp = 0x1101;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX17303_nPackCfg,&temp,2);
#endif

	return ret;
}


//int32_t max17303_temperature(uint8_t *val)
//{
//	int32_t ret;
//
//	ret = tmp117_read(TMP117_TVAL,0,val,2);
//
//    return ret;
//}


int32_t max17303_id(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_VCELL,val,3);

    return ret;
}


int32_t max17303_Cell_Voltage(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_VCELL,val,2);

    return ret;
}

int32_t max17303_Cell_MaxMin_Volt(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_MAXMINVOLT,val,2);

    return ret;
}

int32_t max17303_Cell_MaxMin_Curr(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_MAXMINCURR,val,2);

    return ret;
}

int32_t max17303_Cell_DieTemp(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_DIETEMP,val,2);

    return ret;
}

int32_t max17303_DevName(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_DEVNAME,val,2);

    return ret;
}

int32_t max17303_nDesignCap(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX17303_nDesignCap,val,2);

    return ret;
}

int32_t max17303_nFullCapNom(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX17303_nFullCapNom,val,2);

    return ret;
}

int32_t max17303_TTE(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_TTE,val,2);

    return ret;
}


int32_t max17303_TTF(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_TTF,val,2);

    return ret;
}


int32_t max17303_Battery_Status(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX17303_nBattStatus,val,2);

    return ret;
}



int32_t max17303_Max_Peak_Power(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_MAXPEAKPWR,val,2);

    return ret;
}


int32_t max17303_Avg_Vcell(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_AVGVCELL,val,2);

    return ret;
}


int32_t max17303_Avg_Curr(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_AVGCURRENT,val,2);

    return ret;
}

int32_t max17303_Avg_Cap(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_AVCAP,val,2);

    return ret;
}

int32_t max17303_Avg_Die_Temp(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_AVGDIETEMP,val,2);

    return ret;
}


int32_t max17303_Set_nDesign_Voltage(void)
{
	int32_t ret;
	uint8_t data[4];

	data[0] = _max17303_fuelgauge.Vminsys;
	data[1] = _max17303_fuelgauge.Vdesign;

	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX1730X_nDesignVoltage,&data[0],2);

    return ret;
}



int32_t max17303_Get_Temp(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_TEMP,val,2);

    return ret;
}


int32_t max17303_Get_RepSoc(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_REPSOC,val,2);

    return ret;
}


int32_t max17303_Get_Full_Cap_Rep(uint8_t *val)
{
	int32_t ret;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_FULLCAPREP,val,2);
	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX17303_RepCap,val,2);
    return ret;
}

int32_t max17303_Get_Full_Cap_Rep_a(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_FULLCAPREP,val,2);
//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX17303_RepCap,val,2);
    return ret;
}

int32_t max17303_Get_Full_Cap_Rep_b(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX17303_nDesignCap,val,2);
    return ret;
}

int32_t max17303_Set_Full_Cap_Rep(uint16_t val)
{
	int32_t ret;
	uint8_t data[4];

	data[1] = (val >> 8) & 0xff;
	data[0] = val & 0xff;

	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX1730X_FULLCAPREP,&data[0],2);

    return ret;
}


int32_t max17303_Rep_SOC(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_REPSOC,val,2);

    return ret;
}

int32_t max17303_Set_Fets(uint8_t discharge , uint8_t charge)
{
	int32_t ret;
	uint8_t data[2];

	data[1] = 0x00;
	data[0] = 0x00;

	if(discharge == 1)
	   data[1] = 0x02;

	if(charge == 1)
		   data[1] |= 0x01;

	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX1730X_COMMSTAT,&data[0],2);

    return ret;
}


int32_t max17303_Get_Fets(uint8_t *val)
{
	int32_t ret;

		ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_COMMSTAT,val,2);

	    return ret;
}


int32_t max17303_Set_Cell_MaxMin_Curr(uint16_t val)
{
	int32_t ret;
	uint8_t data[4];

	data[1] = (val >> 8) & 0xff;
	data[0] = val & 0xff;

	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX1730X_MAXMINCURR,&data[0],2);

    return ret;
}


int32_t max17303_Set_Config(uint16_t val)
{
	int32_t ret;
	uint8_t data[4];

	data[1] = (val >> 8) & 0xff;
	data[0] = val & 0xff;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_VCELL,val,3);
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX1730X_CONFIG,&data[0],2);

    return ret;
}


int32_t max17303_Current(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_CURRENT,val,2);

    return ret;
}

int32_t max17303_FilterCfg(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_FILTERCFG,val,2);

    return ret;
}


//
//int32_t max17303_Set_Config(uint16_t val)
//{
//	int32_t ret;
//
////	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_VCELL,val,3);
//	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX1730X_CONFIG,val,2);
//
//    return ret;
//}


int32_t max17303_Fuel_Gauge_Temp(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_VCELL,val,2);

    return ret;
}

int32_t max17303_Die_Temp(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_VCELL,val,2);

    return ret;
}

int32_t max17303_Pack_Current(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_VCELL,val,2);

    return ret;
}

int32_t max17303_Pack_Avg_Current(uint8_t *val)
{
	int32_t ret;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_VCELL,val,2);

    return ret;
}

int32_t max17303_Reset_Full(void)
{
	int32_t ret;
	int8_t val[2] = {0x00,0x0f};

	ret = max17303_chnl.write_reg(&max17303_chnl.handle,0x60,&val[0],2);

	// Wait for 10ms.

    return ret;
}

int32_t max17303_Reset_Fuel_Gauge(void)
{
	int32_t ret;
	int8_t val[2] = {0x80,0x00};

	ret = max17303_chnl.write_reg(&max17303_chnl.handle,MAX1730X_CONFIG2,&val[0],2);

	// Wait ofr POR_CMD bit (bit 15) of the Config2 to be cleared

    return ret;
}


//int32_t max17303_Battery_Status(uint8_t *val)
//{
//	int32_t ret;
//
//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_nBattStatus,val,2);
//
//    return ret;
//}


int32_t max17303_State_of_Charge(void)
{
	int32_t ret;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_nBattStatus,val,2);

    return ret;
}

int32_t max17303_Remain_Capacity(void)
{
	int32_t ret;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_nBattStatus,val,2);

    return ret;
}

int32_t max17303_Time_to_Full(void)
{
	int32_t ret;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_nBattStatus,val,2);

    return ret;
}

int32_t max17303_Time_to_Empty(void)
{
	int32_t ret;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_nBattStatus,val,2);

    return ret;
}

int32_t max17303_Cell_Full_Capacity(void)
{
	int32_t ret;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_nBattStatus,val,2);

    return ret;
}

int32_t max17303_Cell_Health(void)
{
	int32_t ret;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_nBattStatus,val,2);

    return ret;
}

int32_t max17303_Cell_Lifetime_Cycles(void)
{
	int32_t ret;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_nBattStatus,val,2);

    return ret;
}

int32_t max17303_Cell_Age(void)
{
	int32_t ret;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_nBattStatus,val,2);

    return ret;
}

int32_t max17303_Cell_Resistance(void)
{
	int32_t ret;

//	ret = max17303_chnl.read_reg(&max17303_chnl.handle,MAX1730X_nBattStatus,val,2);

    return ret;
}

int32_t max17303_init_modelgauge(void)
{
	int32_t ret;

//	_max17303_fuelgauge.Vdesign = (uint8_t) ((3.7f * 0x100) / 5.12);
//	_max17303_fuelgauge.Vminsys = (uint8_t) ((3.4f * 0x100) / 5.12);

    return ret;
}


uint32_t get_lock_page(uint8_t *page)
{
	int32_t ret,temp;

	ret = max17303_chnl.read_reg(&max17303_chnl.handle,LOCKREG,&temp,1);

	page = temp & 0x1f;

	return ret;
}

uint32_t set_lock_page(uint8_t page)
{
	int32_t ret,lock_reg,temp;
	uint8_t done = true;

	if(page > 4)
	{
		return -1;
	}
	else
	{

		lock_reg = nvlock_reg | (1 << page);

		while(done == true)
		{
		    // Clear CommStat.NVError.
			ret = max17303_chnl.read_reg(&max17303_chnl.handle,COMMSTAT,&temp,1);
			temp &= 0xfffb;
			ret = max17303_chnl.write_reg(&max17303_chnl.handle,COMMSTAT,&temp,1);

			ret = max17303_chnl.write_reg(&max17303_chnl.handle,COMMREG,&lock_reg,1);
			// Wait for tupdate time = 64 to 1280 msec.

			// Check again CommStat.NVError to be set.
			ret = max17303_chnl.read_reg(&max17303_chnl.handle,COMMSTAT,&temp,1);

			if((temp & 0x0004) == 0x0)
				 done = false;
		}

	}

	 return ret;
}


uint32_t reset_ic(void)
{
	int32_t ret,lock_reg,temp;

	temp = 0x0;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,0x80,&temp,1);

	temp = POR;
	ret = max17303_chnl.write_reg(&max17303_chnl.handle,COMMREG,&temp,1);

	return ret;
}


uint8_t lockqm(void)
{
	int32_t temp;
	uint8_t a;

	max17303_chnl.read_reg(&max17303_chnl.handle,0x7f,&temp,1);

	a = temp & 0x1f;
	return a;
}

