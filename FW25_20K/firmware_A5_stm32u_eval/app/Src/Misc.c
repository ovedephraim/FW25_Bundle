/**
* @file misc.c
* @brief misceleneous
*
* @version 0.0.1
* @date 20.10.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

//#include "../../app/Inc/main.h"
//#include "as6221.h"
//#include "tmp117.h"
#include "max17303.h"
//#include "lp55281.h"
#include <string.h>
#include <stdio.h>
#include "MAX30001/MAX30001/max30001.h"
#include "MAX30003/MAX30003/max30003.h"
#include "main.h"
#include "misc.h"
#include "lp55281.h"
//#include "touchsensing.h"

//extern heater_sm _heater_sm;
//extern charge_sm  _charge_sm;
extern led_sm     _led_sm;
//extern temperatures _temperatures;
//extern battery_super _battery_super;
extern battery_limits _battery_limits;
extern uint8_t temp_sel[3];
//extern DAC_HandleTypeDef hdac1;

//extern stmdev_tmp117_ctx_t tmp117_chnl_1;
//extern stmdev_tmp117_ctx_t tmp117_chnl_2;
//extern stmdev_tmp117_ctx_t tmp117_chnl_3;

extern bool heat_burst;
extern uint32_t calibs,this_delta;


#define VPTH 		280
#define VREG		410
#define ITERM        65
#define IREG        850

#define HEAT_EN_GPIO_Port  GPIOE
#define HEAT_EN_Pin        GPIO_PIN_0


void SetLowPassFilter(uint8_t sel)
{
    switch(sel)
    {
		case 0:
			    HAL_GPIO_WritePin(GPIOE, ECG_HPF0_Pin, GPIO_PIN_RESET);
		        HAL_GPIO_WritePin(GPIOE, ECG_HPF1_Pin, GPIO_PIN_RESET);
				break;
		case 1:
			    HAL_GPIO_WritePin(GPIOE, ECG_HPF0_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, ECG_HPF1_Pin, GPIO_PIN_RESET);
				break;
		case 2:
			    HAL_GPIO_WritePin(GPIOE, ECG_HPF0_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, ECG_HPF1_Pin, GPIO_PIN_SET);
				break;
		case 3:
			    //HAL_GPIO_WritePin(GPIOE, ECG_HPF0_Pin, GPIO_PIN_RESET);
				//HAL_GPIO_WritePin(GPIOE, ECG_HPF1_Pin, GPIO_PIN_RESET);
				break;
    }
}

void SCLNRG(uint8_t sel)
{
    switch(sel)
    {
		case 0:
			    HAL_GPIO_WritePin(GPIOE, SCL_NRG_Pin, GPIO_PIN_RESET);
				break;
		case 1:
			    HAL_GPIO_WritePin(GPIOE, SCL_NRG_Pin, GPIO_PIN_SET);

				break;
    }
}

void CAPSENSE(uint8_t sel)
{
    switch(sel)
    {
		case 0:
			    HAL_GPIO_WritePin(GPIOF, CAPSENSE_EN_Pin, GPIO_PIN_RESET);
				break;
		case 1:
			    HAL_GPIO_WritePin(GPIOF, CAPSENSE_EN_Pin, GPIO_PIN_SET);

				break;
    }
}


//void Enable_Power(void)
//{
//	HAL_GPIO_WritePin(EN_PWR_GPIO_Port, EN_1V8_Pin, GPIO_PIN_SET);
////	HAL_GPIO_WritePin(EN_PWR_GPIO_Port, EN_2V8_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(EN_PWR_GPIO_Port, EN_3V3_Pin, GPIO_PIN_SET);
//}


void Enable_Heater(void)
{
	HAL_GPIO_WritePin(HEAT_EN_GPIO_Port, HEAT_EN_Pin, GPIO_PIN_SET);
}

void Disable_Heater(void)
{
	HAL_GPIO_WritePin(HEAT_EN_GPIO_Port, HEAT_EN_Pin, GPIO_PIN_RESET);
}

void InitializeHeaterState(void)
{
//	_heater_sm.heater = h_off;
//	_heater_sm.h_start  = false;
//	_heater_sm.h_margin  = 5000;
//
//	Disable_Heater();
}

void HeaterState(void)
{
//	switch(_heater_sm.heater)
//    {
//	    case h_off:
//	    	         break;
//	    case h_on:
//	    	    	 break;
//	    case h_ramp:
//	    	    	 break;
//	    case h_pid:
//	    	    	 break;
//	    case h_idle:
//	    	    	 break;
//    }
}


//void LedState(void)
//{
//	switch(_led_sm.led)
//    {
//	    case l_on:
//	    	         break;
//	    case l_charge:
//	    	    	 break;
//	    case l_lowbat:
//	    	    	 break;
//	    case l_connect:
//	    	    	 break;
//	    case l_connecting:
//	    	    	 break;
//	    case l_electrode_disconnect:
//					 break;
//		case l_handshake:
//					 break;
//		case l_findme:
//					 break;
//		case l_selftest_ok:
//					 break;
//		case l_shipping:
//					 break;
//    }
//}


//void ReadTemperatures(void)
//{
//	char log_buf[60]={0};
//	uint8_t val[2],a;
//	float f;
//
//	for(a = 0;a < 3;a++)
//	{
//		val[0] = 0;
//		val[1] = 0;
//		if(temp_sel[a] == 0)
//		    tmp117_temperature_sel(a,&val[0]);
//		else
//			as6221_temperature_sel(a,&val[0]);
//		if(val[0] >= 0x80)
//		{
//			_temperatures.t_sign[a] = true;
//			_temperatures.t_temp[a] = 0xffff - (val[0] * 0x100 + val[1]);
//
//		}
//		else
//		{
//			_temperatures.t_sign[a] = false;
//			f = (val[0] * 0x100 + val[1]) * 0.0078125F;
//			_temperatures.t_temp[a] = (uint16_t) (100 * f);
//		}
//	}
//
//	sprintf(log_buf,"Temperature =  %4d %4d %4d\r\n",_temperatures.t_temp[SKIN],
//													 _temperatures.t_temp[POLY],
//													 _temperatures.t_temp[AMB]);
//	aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
//}

/*
void FindValidTemperatureIC(void)
{
	uint8_t val[2];
	int temp;

	val[0] = 0;
	val[1] = 0;
	tmp117_ID_sel(0 ,&val[0]);
	temp = (int) ((val[0] & 0xff) * 256) + val[1];
	if(temp == 0x117)
		temp_sel[0] = 0;
	else
		temp_sel[0] = 1;

	val[0] = 0;
	val[1] = 0;
	tmp117_ID_sel(1 ,&val[0]);
	temp = (int) ((val[0] & 0xff) * 256) + val[1];
	if(temp == 0x117)
		temp_sel[1] = 0;
	else
		temp_sel[1] = 1;

	val[0] = 0;
	val[1] = 0;
	tmp117_ID_sel(2 ,&val[0]);
	temp = (int) ((val[0] & 0xff) * 256) + val[1];
	if(temp == 0x117)
		temp_sel[2] = 0;
	else
		temp_sel[2] = 1;

}
*/

//void InitializeChargeState(void)
//{
//	_charge_sm.charge = qualification;
//}

void BatterySuperviser(void)
{
   static uint8_t result[2];
   char log_buf[40]={0};
   float f;
   uint16_t temp;
   uint8_t stat1;


   max17303_Get_Fets(&result[0]);
   stat1 = result[1] & 0x03;
   switch(stat1)
   {
	   case 3: sprintf(log_buf,"         DISOff    CHGOff \r\n");
			   break;
	   case 2: sprintf(log_buf,"         DISOff    CHGOn  \r\n");
	   		   break;
	   case 1: sprintf(log_buf,"         DISOn     CHGOff \r\n");
	   		   break;
	   case 0: sprintf(log_buf,"DISOn     CHGOn  \r\n");
	   		   break;
   }
   aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

//   sprintf(log_buf," \r\n");
//   aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

   // Voltage.
   max17303_Cell_Voltage(&result[0]);

   _battery_limits.b_voltage = (result[1] * 256 + result[0]) * 0.078125f / 1000;

   sprintf(log_buf,"Voltage =  %1.2f volt \r\n",_battery_limits.b_voltage);
   aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

   // Current.
   max17303_Current(&result[0]);

   temp = (result[1] & 0x7f) * 256 + result[0];

   if((result[1] & 0x80) == 0x80)
   {
	  temp = 0xffff - result[1] * 256 + result[0];
	  _battery_limits.b_current = (float) (temp * 1.5625f / RSENSE / 1000);
	//  sprintf(log_buf,"DisCharge Current = %3.3f  ma \r\n",_battery_limits.b_current);
	  sprintf(log_buf,"DisCharge Current = %3.3f   \r\n",_battery_limits.b_current);
   }
   else
   {
	  temp = result[1] * 256 + result[0];
	  _battery_limits.b_current = (float) (temp * 1.5625f / RSENSE / 1000);
	//  sprintf(log_buf,"Charge Current = %3.3f  ma \r\n",_battery_limits.b_current);
	  sprintf(log_buf,"Charge Current = %3.3f   \r\n",_battery_limits.b_current);
   }
   aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

   // Temperature.
   max17303_Cell_DieTemp(&result[0]);

   if((result[1] & 0x80) == 0x80)
   {
	  temp = 0xffff - result[1] * 256 + result[0];
	  _battery_limits.b_temperature = (float) (temp / 256.0);
	//  sprintf(log_buf,"Die Temperature = %2.2f  Cdeg \r\n",_battery_limits.b_temperature);
	  sprintf(log_buf,"Die Temperature = %2.2f    \r\n",_battery_limits.b_temperature);
   }
   else
   {
	  temp = result[1] * 256 + result[0];
	  _battery_limits.b_temperature = (float) (temp / 256.0);
	//  sprintf(log_buf,"Die Temperature = %2.2f  Cdeg \r\n",_battery_limits.b_temperature);
	  sprintf(log_buf,"Die Temperature = %2.2f    \r\n",_battery_limits.b_temperature);
   }
   aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

   // Capacity.
   max17303_Get_Full_Cap_Rep_a(&result[0]);
   f = (result[1] * 256 + result[0]) * 0.1f;
   temp = (uint16_t) f;
   sprintf(log_buf,"Capacity 0x10 =  %4d mah \r\n",temp);
   aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

   // Time.
   // Check if charger is connected.
   stat1 = HAL_GPIO_ReadPin(CHG_STAT_GPIO_Port, CHG_STAT1_Pin);
   if(stat1 == 0)
   {
	   max17303_TTF(&result[0]);
	   f = (result[1] * 256 + result[0]) * 5.625f / 3600;
	   temp = (uint16_t) f;
	   sprintf(log_buf,"TTF =  %3.3f Hours \r\n",f);
	   aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
   }
   else
   {
	   max17303_TTE(&result[0]);
	   f = (result[1] * 256 + result[0]) * 5.625f / 3600;
	   temp = (uint16_t) f;
	   sprintf(log_buf,"TTE =  %3.3f Hours \r\n",f);
	   aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
   }

   sprintf(log_buf," \r\n");
   aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

}


void NotifyRephael(uint8_t message)
{
	switch(message)
	{
		case 0 :
				break;
		case 1 :
				break;
		case 2 :
				break;
	}

}

#if 0
void ChargeState(void)
{
   uint8_t stat1,stat2;
   static uint8_t charge_complete_bit = false;
   static uint8_t cnt = 0;
   char log_buf[30]={0};


   stat1 = HAL_GPIO_ReadPin(CHG_STAT_GPIO_Port, CHG_STAT1_Pin);
   stat2 = HAL_GPIO_ReadPin(CHG_STAT_GPIO_Port, CHG_STAT2_Pin);
   _charge_sm.timer++;


   sprintf(log_buf,"stat1 = %1d , stat2 = %1d \r\n",stat1,stat2);
   aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);


 //  return;

   switch(_charge_sm.charge)
   {
	   case  qualification:
		   	   	   	   	   	 sprintf(log_buf,"qualification [%4d] \r\n",_charge_sm.timer);
		   		   		   	 aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

		   	   	   	   	     if(stat1 == on)
	                         {
		   	   	   	         	   _charge_sm.timer = 0;
		   	   	   	         	   _charge_sm.charge = precondition;
		   	   	   	         	   break;
	                         }


							 if(stat2 == on)
								   _charge_sm.charge = fault;

							 _battery_limits.b_timer = 0;
							 break;
	   case  precondition:
		                     sprintf(log_buf,"precondition [%4d] \r\n",_charge_sm.timer);
		   		   	   		 aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

		   	   	   	   	   	 if(stat2 == on)
							 {
								  _charge_sm.charge = fault;
								  return;
							 }

							 if(stat1 == off)
							 {
								_charge_sm.charge = qualification;
								return;
							 }

 							 // Check if charger is on.
//							 if(stat1 == off)
//							 {
//								 _charge_sm.charge = qualification;
//								 break;
//							 }

							 if(_battery_limits.b_voltage > VPTH)
							 {
								  _charge_sm.charge = constant_current;
								  _charge_sm.timer = 0;
								  sprintf(log_buf,"constant_fast_charge [%4d] \r\n",_charge_sm.timer);
								  aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
								  return;
							 }
							 else
							 {
								// char * str="";
								 sprintf(log_buf,"precondition [%4d] \r\n",_charge_sm.timer);
								 aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
							 }

							 if(++_battery_limits.b_timer > 3720)

							 break;
	   case  constant_current:
		   	   	   	   	     sprintf(log_buf,"constant voltage [%4d] \r\n",_charge_sm.timer);
		   		    		 aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

							 if(stat2 == on)
							 {
								  _charge_sm.charge = fault;
								  return;
							 }

							 if(stat1 == off)
							 {
								_charge_sm.charge = qualification;
								return;
							 }

							 if(_battery_limits.b_current < 1000)
							 {
								 _charge_sm.charge = charge_complete;
								 return;
							 }


						     if(_battery_limits.b_voltage > VREG)
							 {
								  _charge_sm.charge = constant_voltage;
								  _charge_sm.timer = 0;
								  return;
							 }

							 break;
	   case  constant_voltage:
		   	   	   	   	     sprintf(log_buf,"constant voltage [%4d] \r\n",_charge_sm.timer);
		    		   		 aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

							 if(stat2 == on)
							 {
								_charge_sm.charge = fault;
								return;
							 }

							 if(stat1 == off)
							 {
							    _charge_sm.charge = charge_complete;
							    _charge_sm.timer = 0;
							    return;
							 }

							 if(_battery_limits.b_current < 30)
							 {
								 _charge_sm.charge = current_ternination;
								 return;
							 }

							 if(_battery_limits.b_current < ITERM)
							 {
								  _charge_sm.charge = charge_complete;
								  _charge_sm.timer = 0;
								  sprintf(log_buf,"charge complete [%4d] \r\n",_charge_sm.timer);
								  aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
								  return;
							 }

							 break;
 	   case  current_ternination:
 		                     sprintf(log_buf,"current ternination [%4d] \r\n",_charge_sm.timer);
 		   					 aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

 		   	   	   	   	   	 if(stat2 == on)
 							 {
 								  _charge_sm.charge = fault;
 								  return;
 							 }

 							 // Fast Charge safety Timer Period 5400 seconds
 							 // normally wait here for 3600 seconds
 //	   							 if(_charge_sm.timer > 60)
// 							 {
// 								  _charge_sm.charge = constant_current;
// 								//  _charge_sm.timer = 0;
// 								  sprintf(log_buf,"constant current [%4d] \r\n",_charge_sm.timer);
// 								  aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

							 if(stat1 == off)
							 {
								_charge_sm.charge = charge_complete;
								_charge_sm.timer = 0;
								sprintf(log_buf,"charge complete [%4d] \r\n",_charge_sm.timer);
								aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
								break;
							 }

 							 break;
	   case  charge_complete:
		   	   	   	   	     sprintf(log_buf,"charge complete [%4d] \r\n",_charge_sm.timer);
		   		  			 aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

							 if(stat2 == on)
							 {
								_charge_sm.charge = fault;
								return;
							 }

							 if(stat1 == on)
							 {
								 charge_complete_bit = true;
							 }

							 if(_charge_sm.timer > 20)
							 {
								 if(charge_complete_bit == false)
								 {
									_charge_sm.charge = qualification;
									return;
								 }

								 charge_complete_bit = false;
								 _charge_sm.timer = 0;
							 }

							 break;
	   case  fault:
		                     sprintf(log_buf,"fault [%4d] \r\n",_charge_sm.timer);
		   		   		     aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

		                     if(stat2 == on)
							 {
								if(++cnt > 10)
								{
									// Alert REPHAEL.
									_charge_sm.charge = fault;
									cnt = 14;
								}
							 }

							 break;
	   case  therm_invalid:
		                     sprintf(log_buf,"therm invalid [%4d] \r\n",_charge_sm.timer);
		   		   		   	 aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);

							 if(stat1 == on)
							 {
								_charge_sm.charge = constant_current;
								_charge_sm.timer = 0;
							 }

							 break;
	   case  disabled_sleep_mode:
							 // We never get here.
							 // EN is always ON.
							 break;
	   case  input_disconnected:
							 if(stat2 == on)
								  _charge_sm.charge = fault;

							 if(stat1 == on)
								  _charge_sm.charge = constant_voltage;

							 if((stat1 == off) && (stat2 == off))
							 {
								 if(++_charge_sm.timer > 40)
									 _charge_sm.charge = qualification;
							 }

							 break;
	   default:
							 break;
   }
}
#endif


void SomeDelay(void)
{
	volatile long a,b;
	volatile uint8_t c;

	for (a = 0;a < 2000;a++)
	  for(b = 0;b < 150;b++)
		 c++;
}

#if 0
void Pid(void)
{
	if((_temperatures.t_temp[SKIN] - _temperatures.t_temp[POLY]) > 20)
	  heat_burst = true;
}
#endif


#if 0
void TestMax3000x(void)
{
	uint32_t pid;

    #define INFO 0x10

	extern MAX30001_Object_t max30001_obj_0;
	extern MAX30003_Object_t max30003_obj_0;


	 MAX30001_readID(&max30001_obj_0, &pid);
	 if(((pid >> 8) & 0x10) == 0x10)
	    led(1 ,'G', 0x3C );
	 else
		led(1 ,'R', 0x2C );

     pid = 0;
	 MAX30003_readID(&max30003_obj_0, &pid);
	 if(((pid >> 8) & 0x30) == 0x30)
		led(2 ,'G', 0x3C );
	 else
		led(2 ,'R', 0x2C );

}
#endif


void PiezoTest(void)
{
//	int32_t ret = 0;

	HAL_GPIO_WritePin(BUZZER_EN_Port, BUZZER_EN_Pin, GPIO_PIN_RESET);


}

#if 0
void Touch_Sense_test(void)
{
	int32_t log_buf[100];
	tsl_user_status_t tsl_status;


				sprintf(log_buf,"\r\n Not Valid \r\n");
				switch(tsl_status)
				{
				   case 0:
					       sprintf(log_buf,"\r\n TSL_USER_STATUS_BUSY \r\n");
					       break;
				   case 1:
						   sprintf(log_buf,"\r\n TSL_USER_STATUS_OK_NO_ECS \r\n");
						   break;
				   case 2:
						   sprintf(log_buf,"\r\n TSL_USER_STATUS_OK_ECS_ON \r\n");
						   break;
				   case 3:
						   sprintf(log_buf,"\r\n TSL_USER_STATUS_OK_ECS_OFF \r\n");
						   break;
				}
			 	aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);


				switch(MyTKeys[0].p_Data->StateId)
				{
					case 0:
						    sprintf(log_buf,"\r\n TSL_STATEID_CALIB \r\n");
						    break;
					case 1:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_CALIB \r\n");
						    break;
					case 2:
						    sprintf(log_buf,"\r\n TSL_STATEID_RELEASE %4d \r\n",calibs);
							break;
					case 3:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_RELEASE_PROX \r\n");
							break;
					case 4:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_RELEASE_DETECT \r\n");
							break;
					case 5:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_RELEASE_TOUCH \r\n");
							break;
					case 6:
						  	sprintf(log_buf,"\r\n TSL_STATEID_PROX %4d \r\n",this_delta);
							break;
					case 7:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_PROX %4d \r\n",this_delta);
							break;
					case 8:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_PROX_DETECT \r\n");
							break;
					case 9:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_PROX_TOUCH \r\n");
							break;
					case 10:
						    sprintf(log_buf,"\r\n TSL_STATEID_DETECT \r\n");
							break;
					case 11:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_DETECT \r\n");
					        break;
					case 12:
						    sprintf(log_buf,"\r\n TSL_STATEID_TOUCH \r\n");
							break;
					case 13:
						    sprintf(log_buf,"\r\n TSL_STATEID_ERROR \r\n");
							break;
					case 14:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_ERROR_CALIB \r\n");
							break;
					case 15:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_ERROR_RELEASE \r\n");
							break;
					case 16:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_ERROR_PROX \r\n");
							break;
					case 17:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_ERROR_DETECT \r\n");
							break;
					case 18:
						    sprintf(log_buf,"\r\n TSL_STATEID_DEB_ERROR_TOUCH \r\n");
							break;
					case 19:
						    sprintf(log_buf,"\r\n TSL_STATEID_OFF \r\n");
							break;
				}
		sprintf(log_buf,"\r\n RAW DATA %4d \r\n",calibs);
		aux_sendToAux(log_buf,strlen(log_buf),0,1,DBG_AUX);
}

void Touchkey(void)
{
//	static uint8_t ena = false;
//
//	if((_touch_sm.active == true) && (ena == false))
//	{
//		_touch_sm.duration = 0;
//		CAPSENSE(true);
//		ena = true;
//	}
//
//
//	if(++_touch_sm.duration > _touch_sm.maxdur)
//	{
//		_touch_sm.active = false;
//		CAPSENSE(false);
//		ena = false;
//	}
}

void DACOutput(uint8_t num)
{
	/*##-0- Set DAC Channel1 DHR register ######################################*/
    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, num) != HAL_OK)
    {
	  /* Setting value Error */
	  Error_Handler();
    }
}

#endif
