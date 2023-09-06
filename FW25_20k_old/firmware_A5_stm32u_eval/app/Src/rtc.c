/**
* @file unix_time.c
* @brief time functions wrapper
*
* @version 0.0.1
* @date 20.10.2015
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/
#include "main.h"
#include "rtc.h"


extern QueueHandle_t dispq;
RTC_HandleTypeDef hrtc;

//====================================================================================================
//============================================== RTC =================================================
//====================================================================================================

/**
* @brief RTC MSP Initialization
* This function configures the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if(hrtc->Instance==RTC)
  {

    /** Initializes the peripherals clock */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler((uint8_t *)__FILE__, __LINE__);
    }

    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
  }
}

/**
* @brief RTC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
    __HAL_RCC_RTCAPB_CLK_DISABLE();

    /* RTC interrupt DeInit */
    HAL_NVIC_DisableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}/* End of HAL_RTC_MspDeInit */


/**
  * @brief This function handles RTC wake-up interrupt
  */
void RTC_IRQHandler(void)
{
	 HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}

/**
 * @fn RTC_GetTimestampMillis
 * @brief get timestamp in mili seconds
 * @author Anton Kanaev
 * @date 08.05.2022
 */
uint32_t RTC_GetTimestampMillis(void)
{
	RTC_DateTypeDef rtcDate;
	RTC_TimeTypeDef rtcTime;
	struct tm tim = {0};
	uint64_t diff_t =0;
	uint32_t subsec=0;
	time_t curr;
	time_t stored;


	/* Get the RTC current Time */
	if (HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		return -1;
	}

	/* Get the RTC current Date */
	if (HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		return -1;
	}

	uint8_t hh = rtcTime.Hours;
	uint8_t mm = rtcTime.Minutes;
	uint8_t ss = rtcTime.Seconds;
	uint8_t d = rtcDate.Date;
	uint8_t m = rtcDate.Month;
	uint16_t y = rtcDate.Year;
	uint16_t yr = (uint16_t)(y+2000-1900);

	tim.tm_year = yr;
	tim.tm_mon = m - 1;
	tim.tm_mday = d;
	tim.tm_hour = hh;
	tim.tm_min = mm;
	tim.tm_sec = ss;

	/* read current time */
	curr = mktime(&tim);
	/* read stored value */
	stored = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1);

	subsec= (rtcTime.SecondFraction - rtcTime.SubSeconds)*1000UL/rtcTime.SecondFraction-1;
	if(rtcTime.SubSeconds>rtcTime.SecondFraction)
	{
		curr--;
	}

	/* Difference in sec */
	diff_t = difftime(curr, stored)*1000ULL+subsec;

    return (uint32_t)diff_t;
}/* End of RTC_GetTimestampMillis */

/**
 * @fn int RTC_save_time_stamp(time_t now)
 * @brief save time stamp val
 * @author Anton Kanaev
 * @param  value
 * @date 08.05.2022
 */
int RTC_save_time_stamp(uint32_t ts)
{
	/* write to the backup registers */
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,ts);

	return 0;
}/* end of RTC_save_time_stamp */

/**
 * @fn int RTC_save_time_stamp(time_t now)
 * @brief save time stamp val
 * @author Anton Kanaev
 * @param  value
 * @date 08.05.2022
 */
int RTC_get_time_stamp(uint32_t * pts)
{
	if(pts ==NULL)
	{
		return -1;
	}
	/* load from backup registers */
	*pts = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1);

	return 0;
}/* end of RTC_save_time_stamp */


///**
// * @fn RTC_reset_wakeup_timer
// * @brief get  unix time from from RTC
// * @author Anton Kanaev
// * @param  pnow - time stamp
// * @date 08.05.2022
// */
//int RTC_disable_wakeup_timer(void)
//{
////  /* register callback function */
////  if (HAL_RTC_UnRegisterCallback(&hrtc, HAL_RTC_WAKEUPTIMER_EVENT_CB_ID) != HAL_OK)
////  {
////	return -1;
////  }
//
////  /* enable wake up timer every second  */
////  if (HAL_RTCEx_DeactivateWakeUpTimer(&hrtc) != HAL_OK)
////  {
////	return -1;
////  }
////
////  /* Set priority for RTC global Interrupt */
////  HAL_NVIC_SetPriority(RTC_IRQn, 15 , 1U);
////
////  /* Enable the RTC global Interrupt */
//   HAL_NVIC_DisableIRQ(RTC_IRQn);
//
//  return 0;
//}/* end of RTC_get */
//
//
//int RTC_enable_wakeup_timer(void * cb)
//{
////	  /* register callback function */
////	  if (HAL_RTC_RegisterCallback(&hrtc, HAL_RTC_WAKEUPTIMER_EVENT_CB_ID, cb) != HAL_OK)
////	  {
////		return -1;
////	  }
//
////	  /* Set priority for RTC global Interrupt */
////	  HAL_NVIC_SetPriority(RTC_IRQn, 15 , 1U);
////
////	  /* Enable the RTC global Interrupt */
////	  HAL_NVIC_EnableIRQ(RTC_IRQn);
//
////	  /* enable wake up timer every second  */
////	  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS , 0) != HAL_OK)
////	  {
////		return -1;
////	  }
//
//return 0;
//}

/**
 * @fn RTC_set_wakeup_timer
 * @brief get  unix time from from RTC
 * @author Anton Kanaev
 * @param  pnow - time stamp
 * @date 08.05.2022
 */
int RTC_set_wakeup_timer(uint16_t tm_sec, void * cb)
{
  /* register callback function */
  if (HAL_RTC_RegisterCallback(&hrtc, HAL_RTC_WAKEUPTIMER_EVENT_CB_ID, cb) != HAL_OK)
  {
	return -1;
  }

  /* enable wake up timer every second  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, tm_sec, RTC_WAKEUPCLOCK_CK_SPRE_16BITS , 0) != HAL_OK)
  {
	return -1;
  }

  /* Set priority for RTC global Interrupt */
  HAL_NVIC_SetPriority(RTC_IRQn, 15 , 1U);

  /* Enable the RTC global Interrupt */
  HAL_NVIC_EnableIRQ(RTC_IRQn);

  return 0;
}/* end of RTC_get */


/**
 * @fn int RTC_get(time_t now)
 * @brief get  unix time from from RTC
 * @author Anton Kanaev
 * @param  pnow - time stamp
 * @date 08.05.2022
 */
int RTC_get(time_t *pnow)
{
	RTC_DateTypeDef rtcDate;
	RTC_TimeTypeDef rtcTime;
	struct tm tim = {0};

	if(!pnow)
	{
		return -1;
	}

	/* Get the RTC current Time */
	if (HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		return -1;
	}

	/* Get the RTC current Date */
	if (HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		return -1;
	}

	 uint8_t hh = rtcTime.Hours;
	 uint8_t mm = rtcTime.Minutes;
	 uint8_t ss = rtcTime.Seconds;
	 uint8_t d = rtcDate.Date;
	 uint8_t m = rtcDate.Month;
	 uint16_t y = rtcDate.Year;
	 uint16_t yr = (uint16_t)(y+2000-1900);

	 tim.tm_year = yr;
	 tim.tm_mon = m - 1;
	 tim.tm_mday = d;
	 tim.tm_hour = hh;
	 tim.tm_min = mm;
	 tim.tm_sec = ss;

	*pnow = mktime(&tim);

	return 0;
}/* end of RTC_get */

/**
 * @fn int RTC_set(time_t now)
 * @brief update RCT from unix time
 * @author Anton Kanaev
 * @param  now - time stamp
 * @date 08.05.2022
 */
int RTC_set(time_t now)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	struct tm time_tm;
	time_tm = *(localtime(&now));

	/** update the RTC time */
	sTime.Hours = (uint8_t)time_tm.tm_hour;
	sTime.Minutes = (uint8_t)time_tm.tm_min;
	sTime.Seconds = (uint8_t)time_tm.tm_sec;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK){
		return -1;
	}

	if (time_tm.tm_wday == 0) { time_tm.tm_wday = 7; } // the chip goes mon tue wed thu fri sat sun
	sDate.WeekDay  = (uint8_t)time_tm.tm_wday;
	sDate.Month    = (uint8_t)time_tm.tm_mon+1; //momth 1- This is why date math is frustrating.
	sDate.Date     = (uint8_t)time_tm.tm_mday;
	sDate.Year     = (uint16_t)(time_tm.tm_year+1900-2000); // time.h is years since 1900, chip is years since 2000

	/** update the RTC date */
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK){
		return -1;
	}

//	/* enable bkup access */
//	HAL_PWR_EnableBkUpAccess();

	/* lock it in with the backup registers */
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);

//	HAL_PWR_DisableBkUpAccess();

	return 0;
}/* end of RTC_set */



/**
  * @brief RTC Initialization Function
  * @param None
  * @retval 0 ok
  */
int RTC_Init(void)
{
  RTC_PrivilegeStateTypeDef privilegeState = {0};

  /* Enable RTC Clock */
  __HAL_RCC_RTC_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();

  /** Initialize RTC Only */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  hrtc.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp   = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode        = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK){
    return -1;
  }
  privilegeState.rtcPrivilegeFull         = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone   = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK){
    return -1;
  }


 HAL_RTCEx_SetSmoothCalib(&hrtc, RTC_SMOOTHCALIB_PERIOD_32SEC, RTC_SMOOTHCALIB_PLUSPULSES_RESET,0);

 return 0;

} /* End of RTC_Init */


