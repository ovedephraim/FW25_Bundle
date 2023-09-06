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

#include "lp55281.h"
#include "main.h"


 led_sm     _led_sm;

#define ADC2VOLT_COEF (27.345) //mV

// LP55281_Object_t * gp_obj=NULL;
 static LP55281_Object_t LP55281_obj;

/**
  * @brief  Wrap Read register component function to Bus IO function
  * @param  Handle the device handler
  * @param  Reg the register address
  * @param  pData the stored data pointer
  * @param  Length the length
  * @retval 0 in case of success, an error code otherwise
  */
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LP55281_Object_t *pObj = (LP55281_Object_t *)Handle;

  return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
}

/**
  * @brief  Wrap Write register component function to Bus IO function
  * @param  Handle the device handler
  * @param  Reg the register address
  * @param  pData the stored data pointer
  * @param  Length the length
  * @retval 0 in case of success, an error code otherwise
  */
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LP55281_Object_t *pObj = (LP55281_Object_t *)Handle;

  return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
}



//=======================================================================================================
// Public API
//=======================================================================================================

/**
  * @brief  Register Component Bus IO operations
  * @param  pObj the device pObj
  * @retval 0 in case of success, an error code otherwise
  */
int32_t LP55281_RegisterBusIO(LP55281_Object_t *pObj, LP55281_IO_t *pIO)
{
  int32_t ret = LP55281_OK;

  if (pObj == NULL)
  {
    ret = LP55281_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read_reg  = ReadRegWrap;
    pObj->Ctx.write_reg = WriteRegWrap;
    pObj->Ctx.handle   = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = LP55281_ERROR;
    }
    else if (pObj->IO.Init() != LP55281_OK)
    {
      ret = LP55281_ERROR;
    }
    else
    {

    }
  }

  return ret;
}

/**
  * @brief  hw enable to led driver and device
  * @retval 0 in case of success, an error code otherwise
  */
int32_t LP55281_Enable_Leds(void)
{
	int32_t ret = 0;
	HAL_GPIO_WritePin(LED_RST_GPIO_PORT, LED_RST_GPIO_PIN, GPIO_PIN_SET);
	return ret;
}


/**
  * @brief  hw enable to led driver and device
  * @retval 0 in case of success, an error code otherwise
  */
int32_t LP55281_Disable_Leds(void)
{
	int32_t ret = 0;
	HAL_GPIO_WritePin(LED_RST_GPIO_PORT, LED_RST_GPIO_PIN, GPIO_PIN_RESET);
	return ret;
}



int32_t LP55281_reset (void )
{
	int32_t ret = 0;
	uint8_t data[2] = {0,0};

	data[0] = RESETL;
	data[1] = 0x21;

	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,data[0], &data[1], 1);
	if(ret != HAL_OK)
	{
		return ret;
	}

return ret;

}/* End of LP55281_reset */


int32_t LP55281_Test_Procedure ( void )
{
	int32_t ret = 0;
	uint8_t data[2] = {0,0};
	float led_test[13]={0};

	/* reset */
	data[0] = RESETL;
	data[1] = 0x21;
	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,data[0], &data[1], 1);
	if(ret != HAL_OK)
	{
		return ret;
	}

	vTaskDelay(25);

	for(int i=0;i<0xd;i++)
	{
		/* Set the preferred values */
		data[0] = i;
		data[1] = 0xFF;
		ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,data[0], &data[1], 1);
		if(ret != HAL_OK){
			return ret;
		}
	}

//	data[0] = CTRL1;
//	data[1] = 0x0;
//	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,data[0], &data[1], 1);
//	if(ret != HAL_OK){
//		return ret;
//	}
//
//	data[0] = CTRL2;
//	data[1] = 0x0;
//	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,data[0], &data[1], 1);
//	if(ret != HAL_OK){
//		return ret;
//	}

	/* Set the preferred value for RED1 */
	data[0] = BOOST;
	data[1] = 0xFF;
	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,data[0], &data[1], 1);
	if(ret != HAL_OK){
		return ret;
	}

	/* Set the preferred value for RED1 */
	data[0] = FREQ;
	data[1] = 0x7;
	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,data[0], &data[1], 1);
	if(ret != HAL_OK){
		return ret;
	}

	/*  Enable boost and RGB drivers  */
	data[0] = ENABLES;
	data[1] = 0xCF;
	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,data[0], &data[1], 1);
	if(ret != HAL_OK){
		return ret;
	}

	/* Wait for 20mili */
	vTaskDelay(20);

	for(int i=0x10;i<0x1d;i++)
	{
		uint16_t cnt=0;
		uint8_t dout=0;

		/* Enable LED test and select output (write 1xh to register 12h) */
		data[0] = LED_TEST;
		data[1] = i;
		ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,data[0], &data[1], 1);
		if(ret != HAL_OK){
			return ret;
		}

		vTaskDelay(1);

		data[0] = ADC_OUT;
		ret = LP55281_obj.Ctx.read_reg(LP55281_obj.Ctx.handle,data[0], &dout, 1);
		if(ret != HAL_OK)
		{
			return ret;
		}

		cnt=i-0x10;
		led_test[cnt]=(float)dout*27.3;

//		char log_buf[50]={0};
//		sprintf(log_buf,"\r\n[LP55281] dout[%d]",dout);
//		aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
	}


	char log_buf[300]={0};
	sprintf(log_buf,"\r\n[LP55281] R1[%f] G1[%f] B1[%f] "
			        "\r\n          R2[%f] G2[%f] B2[%f] "
			        "\r\n          R3[%f] G3[%f] B3[%f] "
			        "\r\n          R4[%f] G4[%f] B4[%f] "
			        "\r\n          ALED[%f]",
					led_test[0],
					led_test[1],
					led_test[2],
					led_test[3],
					led_test[4],
					led_test[5],
					led_test[6],
					led_test[7],
					led_test[8],
					led_test[9],
					led_test[10],
					led_test[11],
					led_test[12]);
	aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);


	/* Go to step 1 of measurement phase and define next output to
	 * be measured as many times as needed */

	/* Disable LED test (write 00h to register 12h) or give reset to
	 *  the device (see step 1 in basic setup phase) */

	/* reset */
	data[0] = RESETL;
	data[1] = 0x21;

	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,data[0], &data[1], 1);
	if(ret != HAL_OK)
	{
		return ret;
	}

return ret;

}/* End of LP55281_reset */


//========================================================================================================

int32_t LP55281_Init(void)
{
	LP55281_IO_t io_ctx;
	uint32_t to=10;

	/* enable hardware interface */
	LP55281_Enable_Leds();

	/* Configure the lp55281 driver */
	io_ctx.Address     = LP55281_I2C_ADD;
	io_ctx.Init        = BUS_I2C4_Init;
	io_ctx.DeInit      = BUS_I2C4_DeInit;
	io_ctx.ReadReg     = BUS_I2C4_ReadReg;
	io_ctx.WriteReg    = BUS_I2C4_WriteReg;
	io_ctx.GetTick     = BUS_GetTick;

	/* i2c bus registration method */
	if (LP55281_RegisterBusIO(&LP55281_obj, &io_ctx) != LP55281_OK){
	  return SYS_ERROR_BUS_FAILURE;
	}

	/* wait for bus will be ready before reading */
	while(BUS_I2C4_IsReady(LP55281_I2C_ADD, 10000))
	{
		if(!to--) return SYS_ERROR_PERIPH_FAILURE;

		/* suspend 25m */
		vTaskDelay(25);
	}

	return 0;
}

int32_t LP55281_Denit(void)
{
	/* enable hardware interface */
	LP55281_Disable_Leds();

	return 0;
}




/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
//int32_t lp55281_read_reg(stmdev_ctx_t *ctx,
//		                 uint8_t reg,
//                         uint8_t *data,
//                         uint16_t len)
//{
//  int32_t ret;
//  ret = ctx->read_reg(ctx->handle, reg, data, len);
//  return ret;
//}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
//int32_t lp55281_write_reg(stmdev_ctx_t *ctx,
//		                  uint8_t reg,
//                          uint8_t  *data,
//                          uint16_t len)
//{
//  int32_t ret;
//  ret = ctx->write_reg(ctx->handle, reg, data, len);
//  return ret;
//}


//int32_t lp55281_read_reg(stmdev_ctx_t  *ctx,
//								uint8_t reg,
//								uint8_t *data,
//								uint8_t len)
//{
//	int32_t ret;
//
//#if 0
//	// Send configuration register data
//	ret = HAL_I2C_Master_Transmit(&hi2c4, LP55281_I2C_ADD, &reg, 1, 50);
//
//	if(ret != HAL_OK)
//	{
//		return ret;
//	}
//
//
//	// Receive voltage data, two bytes
//	ret = HAL_I2C_Mem_Read(&hi2c4,LP55281_I2C_ADD + 1, reg, 1, &data[0], len, 50);
//
//	return ret;
//#else
//
//	  ret = ctx->read_reg(ctx->handle, reg, data, len);
//	  return ret;
//
//#endif
//}
//
//
///**
//  * @brief  Write generic device register
//  *
//  * @param  reg   register to write
//  * @param  data  pointer to data to write in register reg(ptr)
//  * @param  len   number of consecutive register to write
//  * @retval          interface status (MANDATORY: return 0 -> no Error)
//  *
//  */
//
//int32_t lp55281_write_reg (stmdev_ctx_t *ctx,
//								 uint8_t reg,
//							     uint8_t *data,
//								 uint8_t len)
//{
//
//	int32_t ret;
//#if 0
//	uint8_t data_t[2];
//	int32_t ret;
//
//	data_t[0] = reg;
//	data_t[1] = data[0]; // ephraim
//
//	//ret = HAL_I2C_Mem_Write(&hi2c4,LP55281_I2C_ADD, reg, 1, &data_t[1], 1, 50);
//	ret = HAL_I2C_Mem_Write(&hi2c4,LP55281_I2C_ADD, reg, 1, &data_t[1], len, 50);
//
//	return ret;
//
//#else
//
//	ret = ctx->write_reg(ctx->handle, reg, data, len);
//	return ret;
//
//#endif
//
//}



int32_t config_LP55281(void)
{
	int32_t ret=0;
	//uint8_t a;

	//a = 0x0f;
   // ret = lp55281_chnl.write_reg(&lp55281_chnl.handle,BOOST,&a,1);

	//a = 0x07;
	//ret = lp55281_chnl.write_reg(&lp55281_chnl.handle,FREQ,&a,1);

	return ret;
}



void led(wchar_t ledname, wchar_t color, uint8_t pwr )
{
	//color {W-white, R-red, G-green, B-blue, M-magenta, Y-yellow}
	//ledname {1, 2, 3, 4, L-left(1+4), R-right(2+3), U-up(1+2), D-down(3+4), A-all(1+2+3+4) }

	uint8_t redpwr = 0;
	uint8_t greenpwr = 0;
	uint8_t bluepwr = 0;

	//set LED power according to PWM with current scaling factor to equalize brightness
	//scaling: 0xC0 = 100% , 0x80 = 75% , 0x40 = 50% , 0x00 = 25%
	switch(color)
	{
		case 'W' :
			redpwr = pwr + 0x40;
			greenpwr = pwr + 0x80;
			bluepwr = pwr + 0x40;
			break;
		case 'R' :
			redpwr = pwr + 0xC0;
			break;
		case 'G' :
			greenpwr=pwr+0x80;
			break;
		case 'B' :
			bluepwr = pwr + 0xC0;
			break;
		case 'M' :
			redpwr = pwr + 0x80;
			bluepwr = pwr + 0x40;
			break;
		case 'Y' :
			redpwr = pwr + 0x40;
			greenpwr = pwr + 0x80;
			break;
	}



	if (ledname == 1 || ledname == '1' || ledname == 'L' || ledname == 'U' || ledname == 'A')
	{
		LP55281_red(RED1, false,1,redpwr);
		LP55281_green(GREEN1, false,1,greenpwr);
		LP55281_blue(BLUE1, false,1,bluepwr);
	}
	if (ledname == 2 || ledname == '2' || ledname == 'R' || ledname == 'U' || ledname == 'A')
	{
		LP55281_red(RED1, false,2,redpwr);
		LP55281_green(GREEN1, false,2,greenpwr);
		LP55281_blue(BLUE1, false,2,bluepwr);
	}
	if (ledname == 3 || ledname == '3' || ledname == 'R' || ledname == 'D' || ledname == 'A')
	{
		LP55281_red(RED1, false,3,redpwr);
		LP55281_green(GREEN1, false,3,greenpwr);
		LP55281_blue(BLUE1, false,3,bluepwr);
	}
	if (ledname == 4 || ledname == '4' || ledname == 'L' || ledname == 'D' || ledname == 'A')
	{
		LP55281_red(RED1, false,4,redpwr);
		LP55281_green(GREEN1, false,4,greenpwr);
		LP55281_blue(BLUE1, false,4,bluepwr);
	}
}


int32_t chargedemo(void)
{
//	int32_t ret = 0;
//	int32_t i;
//	uint8_t a = 0x06;
//
//	for (i=0; i<18 ;i++)
//		{
//			//SomeDelay();
//		}
//
//	led('4','B',a);
//	SomeDelay();
//	led('3','B',a);
//	SomeDelay();
//	led('2','B',a);
//	SomeDelay();
//	led('1','B',a);
//	SomeDelay();
//	led('A','B',0);
//	SomeDelay();
//	SomeDelay();
//	led('4','B',a);
//	led('3','B',a);
//	for (i=0; i<8 ;i++)
//	{
//		led('2','B',a);
//		SomeDelay();
//		led('2','B',0);
//		SomeDelay();
//	}
//	led('2','B',a);
//	for (i=0; i<8 ;i++)
//	{
//		led('1','B',a);
//		SomeDelay();
//		led('1','B',0);
//		SomeDelay();
//	}
//	led('A','G',a);
//	SomeDelay();
//	SomeDelay();
//	SomeDelay();
//	SomeDelay();
//	SomeDelay();
//	led('A','G',0);
	return 0;
}






int32_t LP55281_red(uint8_t dev_address, uint8_t rst,uint8_t led,uint8_t pwr)
{
	int32_t ret = 0;
	uint8_t a;

	if(rst == true)
	{
		ret = LP55281_obj.Ctx.write_reg((&LP55281_obj)->Ctx.handle,(uint16_t)(dev_address), &pwr, 1);
		//ret = HAL_I2C_Mem_Write(&hi2c4, (uint16_t)(dev_address), 0x01, 1, &pwr, 1, 50);
		if(ret != HAL_OK)
		{
			return ret;
		}
	}

	switch(led)
	{
		case 1 :  //ret = HAL_I2C_Mem_Write(&hi2c4,LP55281_I2C_ADD, RED1, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,RED1, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
		case 2 :  //ret = HAL_I2C_Mem_Write(&hi2c4, LP55281_I2C_ADD, RED2, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,RED2, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
		case 3 :  //ret = HAL_I2C_Mem_Write(&hi2c4, LP55281_I2C_ADD, RED3, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,RED3, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
		case 4 :  //ret = HAL_I2C_Mem_Write(&hi2c4, LP55281_I2C_ADD, RED4, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,RED4, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
	}

	a = NSTBY | EN_BOOST | EN_RGB4 | EN_RGB3 | EN_RGB2 | EN_RGB1;
	//ret = HAL_I2C_Mem_Write(&hi2c4,LP55281_I2C_ADD,ENABLES, 1, &a, 1, 50);
	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,ENABLES, &a, 1);
	if(ret != HAL_OK)
	{
		return ret;
	}

	return ret;
}


int32_t LP55281_blue(uint8_t dev_address, uint8_t rst,uint8_t led,uint8_t pwr)
{
	int32_t ret = 0;
	uint8_t a;

	if(rst == true)
	{
		ret = LP55281_obj.Ctx.write_reg((&LP55281_obj)->Ctx.handle,(uint16_t)(dev_address), &pwr, 1);
		//ret = HAL_I2C_Mem_Write(&hi2c4, (uint16_t)(dev_address), 0x01, 1, &pwr, 1, 50);
		if(ret != HAL_OK)
		{
			return ret;
		}
	}

	switch(led)
	{
		case 1 :  //ret = HAL_I2C_Mem_Write(&hi2c4,LP55281_I2C_ADD, RED1, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,BLUE1, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
		case 2 :  //ret = HAL_I2C_Mem_Write(&hi2c4, LP55281_I2C_ADD, BLUE2, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,BLUE2, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
		case 3 :  //ret = HAL_I2C_Mem_Write(&hi2c4, LP55281_I2C_ADD, BLUE3, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,BLUE3, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
		case 4 :  //ret = HAL_I2C_Mem_Write(&hi2c4, LP55281_I2C_ADD, BLUE4, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,BLUE4, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
	}

	a = NSTBY | EN_BOOST | EN_RGB4 | EN_RGB3 | EN_RGB2 | EN_RGB1;
	//ret = HAL_I2C_Mem_Write(&hi2c4,LP55281_I2C_ADD,ENABLES, 1, &a, 1, 50);
	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,ENABLES, &a, 1);
	if(ret != HAL_OK)
	{
		return ret;
	}

	return ret;
}



int32_t LP55281_green(uint8_t dev_address, uint8_t rst,uint8_t led,uint8_t pwr)
{
	int32_t ret = 0;
	uint8_t a;

	if(rst == true)
	{
		ret = LP55281_obj.Ctx.write_reg((&LP55281_obj)->Ctx.handle,(uint16_t)(dev_address), &pwr, 1);
		//ret = HAL_I2C_Mem_Write(&hi2c4, (uint16_t)(dev_address), 0x01, 1, &pwr, 1, 50);
		if(ret != HAL_OK)
		{
			return ret;
		}
	}

	switch(led)
	{
		case 1 :  //ret = HAL_I2C_Mem_Write(&hi2c4,LP55281_I2C_ADD, RED1, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,GREEN1, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
		case 2 :  //ret = HAL_I2C_Mem_Write(&hi2c4, LP55281_I2C_ADD, GREEN2, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,GREEN2, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
		case 3 :  //ret = HAL_I2C_Mem_Write(&hi2c4, LP55281_I2C_ADD, GREEN3, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,GREEN3, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
		case 4 :  //ret = HAL_I2C_Mem_Write(&hi2c4, LP55281_I2C_ADD, GREEN4, 1, &pwr, 1, 50);
		          ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,GREEN4, &pwr, 1);
				  if(ret != HAL_OK)
				  {
					return ret;
				  }
				  break;
	}

	a = NSTBY | EN_BOOST | EN_RGB4 | EN_RGB3 | EN_RGB2 | EN_RGB1;
	//ret = HAL_I2C_Mem_Write(&hi2c4,LP55281_I2C_ADD,ENABLES, 1, &a, 1, 50);
	ret = LP55281_obj.Ctx.write_reg(LP55281_obj.Ctx.handle,ENABLES, &a, 1);
	if(ret != HAL_OK)
	{
		return ret;
	}

	return ret;
}





//int32_t LP55281_ntsby(void)
//{
//	int32_t ret = 0;
//	uint8_t a;
//
//	a = 0x00;
//	 ret = lp55281_chnl.write_reg(&lp55281_chnl.handle,ENABLES,&a,1);
//	return ret;
//}

//typedef enum leds_state
//{
//	l_idle = 0,
//	l_charge,
//	l_lowbat,
//	l_connect,
//	l_connecting,
//	l_electrode_disconnect,
//	l_handshake,
//	l_findme,
//	l_selftest_ok,
//	l_seleftest_fail,
//	l_shipping
//}leds_state;
//
//typedef struct {
//	uint32_t l_timer;
//	leds_state led;
//}led_sm;

void TurnOffAllLeds(void)
{
	led('1','W',0x0);
	led('2','W',0x0);
	led('3','W',0x0);
	led('4','W',0x0);
}

uint32_t Led_SM(void)
{
	static uint8_t change = 0xff;
	static uint8_t a = 0,b = 0;

	switch(_led_sm.led)
	{
		case l_idle:
						 // All Leds Off.
			             if(change != l_idle)
			             {
							led('1','W',0x0);
							led('2','W',0x0);
							led('3','W',0x0);
							led('4','W',0x0);

							change = l_idle;
			             }
						 break;
		case l_charge:
						 if(change != l_charge)
						 {
							led('1','R',0x58);
							led('2','W',0x0);
							led('3','W',0x0);
							led('4','W',0x0);

							change = l_charge;
						 }
						 break;
		case l_lowbat:
						 if(change != l_lowbat)
						 {
							led('1','R',0x68);
							led('2','R',0x68);
							led('3','R',0x68);
							led('4','R',0x68);

							change = l_lowbat;
						 }
						 break;

		case l_seleftest_fail:
						 if(change != l_lowbat)
			//						 {
			//							led('1','R',0x68);
			//							led('2','R',0x68);
			//							led('3','R',0x68);
			//							led('4','R',0x68);
			//
			//							change = l_lowbat;
			//						 }
			//						 else
			//						 {
						 if((++b % 2) == 0)
						 {
							 led('1','R',0x38);
							 led('2','R',0x38);
							 led('3','R',0x38);
							 led('4','R',0x38);
						 }
						 else
							TurnOffAllLeds();

						 if(++a > 10)
						 {
							 a = 0;
							 _led_sm.led = l_idle;
						 }
						 break;
		case l_seleftest_ok:
						 if(change != l_lowbat)
//						 {
//							led('1','R',0x68);
//							led('2','R',0x68);
//							led('3','R',0x68);
//							led('4','R',0x68);
//
//							change = l_lowbat;
//						 }
//						 else
//						 {
						 if((++b % 2) == 0)
						 {
							 led('1','G',0x38);
							 led('2','G',0x38);
							 led('3','G',0x38);
							 led('4','G',0x38);
						 }
						 else
							TurnOffAllLeds();

						 if(++a > 10)
						 {
							 a = 0;
							 _led_sm.led = l_idle;
						 }
						 break;
		case l_selftest:

						 TurnOffAllLeds();
						 switch(a)
						 {
							 case 0:
									 led('1','G',0x38);
									 break;
							 case 1:
									 led('2','G',0x38);
									 break;
							 case 2:
									 led('3','G',0x38);
									 break;
							 case 3:
									 led('4','G',0x38);
									 break;
						 }

						 if(++a > 3) a = 0;
						 break;
	 case l_connect_gw:
						 TurnOffAllLeds();
						 switch(a)
						 {
							 case 0:
									 led('1','B',0x38);
									 break;
							 case 1:
									 led('2','B',0x38);
									 break;
							 case 2:
									 led('3','B',0x38);
									 break;
							 case 3:
									 led('4','B',0x38);
									 break;
						 }

						 if(++a > 3) a = 0;
						 break;
	case l_connect_gw_fail:
						 if(change != l_lowbat)
			//						 {
			//							led('1','R',0x68);
			//							led('2','R',0x68);
			//							led('3','R',0x68);
			//							led('4','R',0x68);
			//
			//							change = l_lowbat;
			//						 }
			//						 else
			//						 {
						 if((++b % 2) == 0)
						 {
							 led('1','R',0x38);
							 led('2','R',0x38);
							 led('3','R',0x38);
							 led('4','R',0x38);
						 }
						 else
							TurnOffAllLeds();

						 if(++a > 10)
						 {
							 a = 0;
							 _led_sm.led = l_idle;
						 }
						 break;
	case l_connect_gw_ok:
						 if(change != l_lowbat)
 //						 {
 //							led('1','R',0x68);
 //							led('2','R',0x68);
 //							led('3','R',0x68);
 //							led('4','R',0x68);
 //
 //							change = l_lowbat;
 //						 }
 //						 else
 //						 {
						 if((++b % 2) == 0)
						 {
							 led('1','B',0x38);
							 led('2','B',0x38);
							 led('3','B',0x38);
							 led('4','B',0x38);
						 }
						 else
							TurnOffAllLeds();

						 if(++a > 10)
						 {
							 a = 0;
							 _led_sm.led = l_idle;
						 }
						 break;
	 case l_body:
						 TurnOffAllLeds();
						 switch(a)
						 {
							 case 0:
									 led('1','G',0x38);
									 break;
							 case 1:
									 led('2','G',0x38);
									 break;
							 case 2:
									 led('3','G',0x38);
									 break;
							 case 3:
									 led('4','G',0x38);
									 break;
						 }

						 if(++a > 3) a = 0;
						 break;
	case l_body_fail:
						 if(change != l_lowbat)
			//						 {
			//							led('1','R',0x68);
			//							led('2','R',0x68);
			//							led('3','R',0x68);
			//							led('4','R',0x68);
			//
			//							change = l_lowbat;
			//						 }
			//						 else
			//						 {
						 if((++b % 2) == 0)
						 {
							 led('1','G',0x38);
							 led('2','G',0x38);
							 led('3','G',0x38);
							 led('4','G',0x38);
						 }
						 else
							TurnOffAllLeds();

						 if(++a > 10)
						 {
							 a = 0;
							 _led_sm.led = l_idle;
						 }
						 break;
	case l_body_ok:
						 if(change != l_lowbat)
 //						 {
 //							led('1','R',0x68);
 //							led('2','R',0x68);
 //							led('3','R',0x68);
 //							led('4','R',0x68);
 //
 //							change = l_lowbat;
 //						 }
 //						 else
 //						 {
						 if((++b % 2) == 0)
						 {
							 led('1','R',0x38);
							 led('2','R',0x38);
							 led('3','R',0x38);
							 led('4','R',0x38);
						 }
						 else
							TurnOffAllLeds();

						 if(++a > 10)
						 {
							 a = 0;
							 _led_sm.led = l_idle;
						 }
						 break;
		case l_connecting:
						 TurnOffAllLeds();
						 switch(a)
						 {
							 case 0:
									 led('1','G',0x38);
									 break;
							 case 1:
									 led('2','G',0x38);
									 break;
							 case 2:
									 led('3','G',0x38);
									 break;
							 case 3:
									 led('4','G',0x38);
									 break;
						 }

						 if(++a > 3) a = 0;
						 break;
		case l_drain:
		default:
						 if(change != l_drain)
						 {
						     led('1','W',0x78);
							 led('2','W',0x78);
							 led('3','W',0x78);
							 led('4','W',0x78);

							change = l_drain;
						 }
						 break;
	}

return 0;
}
