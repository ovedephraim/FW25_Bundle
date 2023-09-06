/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Includes ------------------------------------------------------------------*/


#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_ucpd.h"
#include "stm32u5xx_ll_bus.h"
#include "stm32u5xx_ll_cortex.h"
#include "stm32u5xx_ll_rcc.h"
#include "stm32u5xx_ll_system.h"
#include "stm32u5xx_ll_utils.h"
#include "stm32u5xx_ll_pwr.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_dma.h"
#include "stm32u5xx_ll_exti.h"

#include "stm32u5xx_hal_gpio.h"
#include "stm32u5xx_hal_lptim.h"

#include "board.h"

#include "msg_type.h"
#include "membuf.h"
#include "RpcFifo.h"
#include "packetbuf.h"

#include "sys_errno.h"
#include "sys_conf.h"
#include "sys_port.h"
#include "sys.h"
#include "io_ctl.h"

#include "stm32u5xx_ll_rtc.h"

#include "_mem.h"
#include "_sem.h"
#include "ver.h"

#include "periodic_dispatcher_task.h"
#include "auxcmd.h"
#include "bus.h"

/* Private includes ----------------------------------------------------------*/


#define A4_PROTO 1
#define A5_PROTO 2
#define COM_PROTOCOL A4_PROTO

#define DISP_INTERVAL 1000 //interval in mS
#define DISP_INTERVALSEC DISP_INTERVAL/(float)1000.0 //in S

#define CHG_STAT1_Pin GPIO_PIN_7
#define CHG_STAT2_Pin GPIO_PIN_9
#define CHG_STAT_GPIO_Port GPIOE

#define BUZZER_EN_Port  GPIOF
#define BUZZER_EN_Pin  GPIO_PIN_3
#define CAPSENSE_EN_Port  GPIOF
#define CAPSENSE_EN_Pin  GPIO_PIN_2

#define ECG_HPF0_Pin HEAT_EN_Pin
#define ECG_HPF1_Pin GPIO_PIN_3
#define SCL_NRG_Pin GPIO_PIN_10
#define CAPSENSE_EN_Pin GPIO_PIN_2

/*
typedef enum leds_state
{
	l_idle = 0,
	l_charge,
	l_lowbat,
	l_connect,
	l_connecting,
	l_electrode_disconnect,
	l_handshake,
	l_findme,
	l_selftest_ok,
	l_seleftest_fail,
	l_shipping,
	l_drain
}leds_state;

typedef struct {
	uint32_t l_timer;
	leds_state led;
}led_sm;
*/


typedef enum power_state
{
	p_idle = 0,
	p_charge,
	p_session,
	p_shutdown,
	p_timer

}power_state;

typedef struct {
	uint32_t p_timer;
	power_state pwr;
}power;




typedef struct
{
    float   	b_voltage;
    float   	b_current;
    float   	b_temperature;
    uint16_t    b_timer;
} battery_limits;


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(uint8_t *file, uint32_t line);
void WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
void _JumpToProgram(uint32_t a_iAddress);

/* I2C2 Frequeny in Hz  */
#ifndef BUS_I2C2_FREQUENCY
   #define BUS_I2C2_FREQUENCY  1000000U /* Frequency of I2Cn = 100 KHz*/
#endif



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
