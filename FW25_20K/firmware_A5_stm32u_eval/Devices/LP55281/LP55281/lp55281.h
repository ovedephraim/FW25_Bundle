/*
 ******************************************************************************
 * @file    lp55281_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          lp55281_reg.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LP55281_REGS_H
#define LP55281_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

#define LP55281_OK                       0
#define LP55281_ERROR                   -1


/** I2C Device Address 8 bit format  if SA0=0 -> D5 if SA0=1 -> D7 **/
#define LP55281_I2C_ADD                 0x98


#define			 RED1				    0x00
#define			 GREEN1					0x01
#define			 BLUE1					0x02
#define			 RED2			 	    0x03
#define			 GREEN2					0x04
#define			 BLUE2					0x05
#define			 RED3				    0x06
#define			 GREEN3					0x07
#define			 BLUE3					0x08
#define			 RED4				    0x09
#define			 GREEN4					0x0a
#define			 BLUE4					0x0b
#define			 ALED					0x0c
#define			 CTRL1					0x0d
#define			 CTRL2					0x0e
#define			 BOOST					0x0f
#define			 FREQ			 	    0x10
#define			 ENABLES				0x11
#define			 LED_TEST	 		 	0x12
#define			 ADC_OUT				0x13
#define			 RESETL					0x60


#define			 NSTBY					0x80
#define			 EN_BOOST				0x40
#define			 EN_AUTOLOAD			0x20
#define			 EN_LTEST				0x10
#define			 EN_RGB4				0x08
#define			 EN_RGB3				0x04
#define			 EN_RGB2				0x02
#define			 EN_RGB1				0x01

#define			 FPWM1					0x20
#define			 FPWM0        			0x10

#define			 DC_FREQ				0x10
#define			 EN_AGC					0x08
#define			 EN_SYNC				0x04
#define			 SPEED_CTRL0			0x01
#define			 SPEED_CTRL1			0x02

#define			 FRQ_SEL0				0x01
#define			 FRQ_SEL1				0x02
#define			 FRQ_SEL2				0x04

#define			 GAIN_SEL0				0x20
#define			 GAIN_SEL1				0x40
#define			 GAIN_SEL2				0x80

#define			 THRESHOLD0				0x01
#define			 THRESHOLD1				0x02
#define			 THRESHOLD2				0x04
#define			 THRESHOLD3				0x08

#define			 MUX_LED0				0x01
#define			 MUX_LED1				0x02
#define			 MUX_LED2				0x04
#define			 MUX_LED3				0x08

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

typedef int32_t (*LP55281_Init_Func)(void);
typedef int32_t (*LP55281_DeInit_Func)(void);
typedef int32_t (*LP55281_GetTick_Func)(void);
typedef int32_t (*LP55281_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LP55281_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);


typedef struct
{
  LP55281_Init_Func          Init;
  LP55281_DeInit_Func        DeInit;
  uint8_t                    Address;
  LP55281_WriteReg_Func      WriteReg;
  LP55281_ReadReg_Func       ReadReg;
  LP55281_GetTick_Func       GetTick;
} LP55281_IO_t;

typedef struct
{
  LP55281_IO_t        IO;
  stmdev_ctx_t        Ctx;
  uint8_t             is_initialized;
} LP55281_Object_t;


typedef enum leds_state
{
	l_idle = 0,
	l_charge,
	l_lowbat,
	l_connected,
	l_connecting,
	l_electrode_disconnect,
	l_handshake,
	l_findme,
	l_selftest,
	l_seleftest_fail,
	l_seleftest_ok,
	l_connect_gw,
	l_connect_gw_ok,
	l_connect_gw_fail,
	l_body,
	l_body_ok,
	l_body_fail,
	l_shipping,
	l_drain
}leds_state;

typedef struct {
	uint32_t l_timer;
	leds_state led;
}led_sm;

int32_t LP55281_reset(void);
int32_t LP55281_Test_Procedure (void);

int32_t LP55281_red(uint8_t dev_address, uint8_t rst,uint8_t led,uint8_t pwr);
int32_t LP55281_blue(uint8_t dev_address, uint8_t rst,uint8_t led,uint8_t pwr);
int32_t LP55281_green(uint8_t dev_address, uint8_t rst,uint8_t led,uint8_t pwr);


int32_t LP55281_Enable_Leds(void);
int32_t LP55281_Disable_Leds(void);

int32_t LP55281_Init(void);
int32_t LP55281_Denit(void);

int32_t config_LP55281(void);
void led(wchar_t ledname, wchar_t color, uint8_t pwr );
int32_t chargedemo(void);
int32_t LP55281_ntsby(void);
uint32_t Led_SM(void);


#endif /*lp55281_DRIVER_H */


