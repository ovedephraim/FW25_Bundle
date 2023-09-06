/**
  ******************************************************************************
  * @file           : adc_bus.c
  * @brief          : source file for the adc driver
  ******************************************************************************

*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bus.h"
#include "sys_errno.h"
#include "sys_conf.h"
#include "string.h"
#include "stdio.h"
#include "rtc.h"

/*
	The counter clock is LSI (32.768 KHz), Autoreload equal to 255 so the output
	frequency (FrequencyOutput) will be equal to 128

		FrequencyOutput = Counter Clock Frequency / (Autoreload + 1)

						= 32768 / 256
						= 128 Hz

	Pulse value equal to 49 and the duty cycle (DutyCycle) is computed as follow:

	DutyCycle = 1 - ((PulseValue + 1)/ (Autoreload + 1))
	DutyCycle = 50%
*/
#define FREQ_value           (512)//freq X2 = odr
#define Autoreload_lptim2    (uint32_t)((32768/FREQ_value)-1)
#define PeriodValue_lptim2   (Autoreload_lptim2)
#define PulseValue_lptim2    (uint32_t)(((PeriodValue_lptim2 + 1)/2) - 1)//50% dc

/* Definition of ADCx conversions data table size */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)FREQ_value*2)
#define ADC_CHANNELS 5
/* ADC group regular conversion data (array of data) */

__IO   int   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE * ADC_CHANNELS];
char print_buff[70];
extern int adc_result[];

/* api use to enable vref */
extern int MAX30001_dev_vref_init(void);

const uint32_t adc_seq_ranks[ADC_CHANNELS]=
{
	ADC_REGULAR_RANK_1,
	ADC_REGULAR_RANK_2,
	ADC_REGULAR_RANK_3,
	ADC_REGULAR_RANK_4,
	ADC_REGULAR_RANK_5
};

typedef struct bus_chan_config
{
	adc_reg_param_t bus_chan_config_hndle[ADC_CHANNELS];
	uint16_t ads_seq_len;
}bus_chan_config_t;
bus_chan_config_t cf={0};

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef handle_GPDMA1_Channel10;
LPTIM_HandleTypeDef hlptim2;
DMA_NodeTypeDef ADCNode;
DMA_QListTypeDef ADCQueue;


static void ADC_Init(ADC_HandleTypeDef * hadc);
static void ADC_DeInit(ADC_HandleTypeDef * hadc);

static void GPDMA1_Init(void);
static void LPTIM2_Init(void);

static uint16_t getadc_sampleSeqLen(void);
static void print_sampleSeqLen(void);


HAL_StatusTypeDef ADC1_Queue_Config(void);
static uint32_t ADC1InitCounter = 0;
static uint32_t ADC1ExecState = 0;

/**
  * @brief  execute ADC bus
  * @retval BSP status
  */

int32_t BUS_ADC1_exec_stop(void)
{
	int32_t ret=SYS_ERROR_NONE;
	HAL_StatusTypeDef rc;

	if(ADC1ExecState >= 0)
	{
		/* Run low power timer pwm generation */
		if ((rc=HAL_LPTIM_PWM_Stop(&hlptim2, LPTIM_CHANNEL_1)) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		ADC1ExecState=0;
	}

	return ret;
}/* BUS_ADC1_exec_run */


/**
  * @brief  execute ADC bus
  * @retval BSP status
  */

int32_t BUS_ADC1_exec_run(void)
{
	int32_t ret=SYS_ERROR_NONE;

	/* check BUS_ADC1_init_and_register called before */
	if(ADC1ExecState == 0 && ADC1InitCounter > 0)
	{
		LPTIM_OC_ConfigTypeDef sConfig = {.OCPolarity = 0,.Pulse=0};

		sConfig.Pulse      = PulseValue_lptim2;
		sConfig.OCPolarity = LPTIM_OCPOLARITY_LOW;
		if (HAL_LPTIM_OC_ConfigChannel(&hlptim2, &sConfig, LPTIM_CHANNEL_1) != HAL_OK){
			/* lptim configuration Error */
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}


		/* Run low power timer pwm generation */
		if (HAL_LPTIM_PWM_Start(&hlptim2, LPTIM_CHANNEL_1) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		ADC1ExecState=1;

		/* adc channels needs vref from max30001 */
		MAX30001_dev_vref_init();
	}

	return ret;
}/* BUS_ADC1_exec_run */

/**
  * @brief  Initialize ADC HAL
  * @retval BSP status
  */

int32_t BUS_ADC1_init_and_register(adc_reg_param_t *rp)
{
	int32_t ret=SYS_ERROR_NONE;

	hadc1.Instance=ADC1;
	ADC1InitCounter=1;

	/* de-init adc controller before configuration */
	if (HAL_ADC_GetState(&hadc1) != HAL_ADC_STATE_RESET)
	{
		ADC_DeInit(&hadc1);
	}

	if (HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_RESET)
	{
		/* channel id registration */
		adc1_seq_num tmp_sn=rp->sn;

		/* set adc channel configuration */
		cf.bus_chan_config_hndle[tmp_sn].adc_seq_hndle=rp->adc_seq_hndle;
		cf.bus_chan_config_hndle[tmp_sn].extBuff=rp->extBuff;
		cf.bus_chan_config_hndle[tmp_sn].extBuffLen=rp->extBuffLen;
		cf.bus_chan_config_hndle[tmp_sn].sn=rp->sn;
		cf.bus_chan_config_hndle[tmp_sn].cb=rp->cb;
		cf.bus_chan_config_hndle[tmp_sn].mp=rp->mp;
		cf.bus_chan_config_hndle[tmp_sn].inv=rp->inv;

		/* get sequence length */
		cf.ads_seq_len=getadc_sampleSeqLen();

		/* adc configuration */
		ADC_Init(&hadc1);

		/* informative log printout */
		print_sampleSeqLen();

		ADC1InitCounter++;
	}
	else
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	return ret;
}/* BUS_ADC1_Init */

/**
  * @brief  Initialize ADC HAL
  * @retval BSP status
  */

int32_t BUS_ADC1_deinit_and_unregister(adc1_seq_num sn)
{
	int32_t ret=SYS_ERROR_NONE;

	/* de-init adc controller before configuration */
	if (HAL_ADC_GetState(&hadc1) != HAL_ADC_STATE_RESET)
	{
		if(ADC1ExecState == 0)
		{
			ADC1InitCounter=0;
			hadc1.Instance=ADC1;

			if(cf.bus_chan_config_hndle[sn].adc_seq_hndle)
			{

				/* channel id de-registration */
				memset(cf.bus_chan_config_hndle+sn,0,sizeof(cf.bus_chan_config_hndle[0]));

				/* full adc deinit */
				ADC_DeInit(&hadc1);

				/* get num of registered channels */
				if(0!=getadc_sampleSeqLen())
				{
					/* adc configuration */
					ADC_Init(&hadc1);
				}
			}

		    /* informative log printout */
		    print_sampleSeqLen();

		    ADC1InitCounter--;
		}
	}



	return ret;
}/* BUS_ADC1_DeInit */

uint16_t getadc_sampleSeqLen(void)
{
	uint16_t tmp_ads_seq_len=0;

	/* get num of registered channels */
	for(int i=0;i<ADC_CHANNELS;i++){
		tmp_ads_seq_len+=cf.bus_chan_config_hndle[i].adc_seq_hndle?1:0;
	}

	return tmp_ads_seq_len;
}/* end of getadc_sampleSeqLen */

void print_sampleSeqLen(void)
{
	char pbuff[100]={0};
	sprintf(pbuff,"\r\n ADC1 seq len [%d] [%lu][%lu][%lu][%lu][%lu]\r\n",
			getadc_sampleSeqLen(),
			cf.bus_chan_config_hndle[0].adc_seq_hndle!=NULL ? cf.bus_chan_config_hndle[0].adc_seq_hndle->Rank:0,
			cf.bus_chan_config_hndle[1].adc_seq_hndle!=NULL ? cf.bus_chan_config_hndle[1].adc_seq_hndle->Rank:0,
			cf.bus_chan_config_hndle[2].adc_seq_hndle!=NULL ? cf.bus_chan_config_hndle[2].adc_seq_hndle->Rank:0,
			cf.bus_chan_config_hndle[3].adc_seq_hndle!=NULL ? cf.bus_chan_config_hndle[3].adc_seq_hndle->Rank:0,
			cf.bus_chan_config_hndle[4].adc_seq_hndle!=NULL ? cf.bus_chan_config_hndle[4].adc_seq_hndle->Rank:0);
	aux_sendToAux(pbuff,strlen(pbuff),0,1,DBG_AUX);
}/* end of print_sampleSeqLen */

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void ADC_DeInit(ADC_HandleTypeDef * hadc)
{
	if(hadc->Instance == ADC1)
	{
		uint32_t rv=HAL_ADC_GetState(&hadc1);
		if (rv != HAL_ADC_STATE_RESET)
		{
			/* Stop ADC group regular conversion with DMA */
			if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
			{
			   /* ADC conversion start error */
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

			if (HAL_ADC_DeInit(hadc) != HAL_OK)
			{
				/* ADC deinit error */
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

			HAL_NVIC_DisableIRQ(LPTIM2_IRQn);
		}
	}
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void ADC_Init(ADC_HandleTypeDef * hadc)
{
	if(hadc->Instance == ADC1)
	{
		uint32_t seqLen=getadc_sampleSeqLen();

		/* dma init */
		GPDMA1_Init();

		/* lptim init */
		LPTIM2_Init();

		HAL_NVIC_EnableIRQ(LPTIM2_IRQn);

		//Following command, Set 2.5V on VREF+ Output.
		__HAL_RCC_PWR_CLK_ENABLE();

		/** Enable the VREF clock */
		__HAL_RCC_VREF_CLK_ENABLE();

		/* Configure the internal voltage reference buffer voltage scale  */
		HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE2);

		/* Enable the Internal Voltage Reference buffer */
		HAL_SYSCFG_EnableVREFBUF();

		/* Configure the internal voltage reference buffer high impedance mode  */
		HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);

		/**
		* ===========================================================================
		*            Common adc1 configuration
		* ===========================================================================
		* */
		hadc->Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV1;
		hadc->Init.Resolution               = ADC_RESOLUTION_14B;
		hadc->Init.DataAlign                = ADC_DATAALIGN_RIGHT;
		hadc->Init.ScanConvMode             = ADC_SCAN_ENABLE;
		hadc->Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
		hadc->Init.ExternalTrigConv         = ADC_EXTERNALTRIG_LPTIM2_CH1;
		hadc->Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
		hadc->Init.TriggerFrequencyMode     = ADC_TRIGGER_FREQ_HIGH;
		hadc->Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
		hadc->Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
		hadc->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
		hadc->Init.LowPowerAutoWait         = DISABLE;
		hadc->Init.ContinuousConvMode       = DISABLE;
		hadc->Init.DMAContinuousRequests    = ENABLE;
		hadc->Init.OversamplingMode         = DISABLE;
		hadc->Init.DiscontinuousConvMode    = DISABLE;
		hadc->Init.NbrOfConversion          = seqLen;
		hadc->Init.GainCompensation         = 0;

//		  /* Gain compensation x1 factor */
//		  #define GAIN_COMPENSATION_X1_FACTOR      (0x1000UL)
//		  hadc1.Init.GainCompensation = VDDA_APPLI * GAIN_COMPENSATION_X1_FACTOR / DIGITAL_SCALE_12BITS;

		if (HAL_ADC_Init(hadc) != HAL_OK)
		{
			/* ADC initialization Error */
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}



		/* get num of registered channels */
		for(int i=0,j=0;i<ADC_CHANNELS;i++)
		{
			if(cf.bus_chan_config_hndle[i].adc_seq_hndle)
			{
				/* update rank values according to sequence length */
				cf.bus_chan_config_hndle[i].adc_seq_hndle->Rank=adc_seq_ranks[j++];
				if ((HAL_ADC_ConfigChannel(hadc,
						cf.bus_chan_config_hndle[i].adc_seq_hndle)) != HAL_OK)
				{
					Error_Handler((uint8_t *)__FILE__, __LINE__);
				}
			}
		}

		/* Run the ADC automatic selfcalibration */
		if (HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
		{
			/* adc Calibration Error */
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

#if 0
		uint32_t calib_factor=0;
		/* Calibration value */
		calib_factor=HAL_ADCEx_Calibration_GetValue(hadc,ADC_DIFFERENTIAL_ENDED);
#endif

		/* Start ADC group regular conversion with DMA */
		if (HAL_ADC_Start_DMA(hadc,(uint32_t *)aADCxConvertedData,
							ADC_CONVERTED_DATA_BUFFER_SIZE*seqLen
						   ) != HAL_OK)
		{
			/* ADC conversion start error */
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}
}


/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void GPDMA1_Init(void)
{
	/* Peripheral clock enable */
	__HAL_RCC_GPDMA1_CLK_ENABLE();

	/* GPDMA1 interrupt Init */
	HAL_NVIC_SetPriority(GPDMA1_Channel10_IRQn, 15, 1);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel10_IRQn);

	/* dma configuration */
	handle_GPDMA1_Channel10.Instance = GPDMA1_Channel10;
	handle_GPDMA1_Channel10.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
	handle_GPDMA1_Channel10.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
	handle_GPDMA1_Channel10.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
	handle_GPDMA1_Channel10.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
	handle_GPDMA1_Channel10.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
	if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel10) != HAL_OK)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
	if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel10, DMA_CHANNEL_NPRIV) != HAL_OK)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

}/* end of GPDMA1_Init */

/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	if(hadc->Instance==ADC1)
	{
		/** Initializes the peripherals clock */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
		PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_MSIK;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* Peripheral clock enable */
		__HAL_RCC_ADC1_CLK_ENABLE();

		/* configure analog inputs */
		ADC_EMG_POS_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin =  ADC_EMG_POS_GPIO_PIN;
		GPIO_InitStruct.Mode = ADC_EMG_POS_GPIO_MODE;
		GPIO_InitStruct.Pull = ADC_EMG_POS_GPIO_PULL;
		HAL_GPIO_Init(ADC_EMG_POS_GPIO_PORT, &GPIO_InitStruct);

		ADC_EMG_NEG_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin =  ADC_EMG_NEG_GPIO_PIN;
		GPIO_InitStruct.Mode = ADC_EMG_NEG_GPIO_MODE;
		GPIO_InitStruct.Pull = ADC_EMG_NEG_GPIO_PULL;
		HAL_GPIO_Init(ADC_EMG_NEG_GPIO_PORT, &GPIO_InitStruct);

		ADC_SCL_POS_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin =  ADC_SCL_POS_GPIO_PIN;
		GPIO_InitStruct.Mode = ADC_SCL_POS_GPIO_MODE;
		GPIO_InitStruct.Pull = ADC_SCL_POS_GPIO_PULL;
		HAL_GPIO_Init(ADC_SCL_POS_GPIO_PORT, &GPIO_InitStruct);

		ADC_SCL_NEG_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin =  ADC_EMG_NEG_GPIO_PIN;
		GPIO_InitStruct.Mode = ADC_SCL_NEG_GPIO_MODE;
		GPIO_InitStruct.Pull = ADC_SCL_NEG_GPIO_PULL;
		HAL_GPIO_Init(ADC_SCL_NEG_GPIO_PORT, &GPIO_InitStruct);

		ADC_RESP_POS_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin =  ADC_RESP_POS_GPIO_PIN;
		GPIO_InitStruct.Mode = ADC_RESP_POS_GPIO_MODE;
		GPIO_InitStruct.Pull = ADC_RESP_POS_GPIO_PULL;
		HAL_GPIO_Init(ADC_RESP_POS_GPIO_PORT, &GPIO_InitStruct);

		ADC_RESP_NEG_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin =  ADC_RESP_NEG_GPIO_PIN;
		GPIO_InitStruct.Mode = ADC_RESP_NEG_GPIO_MODE;
		GPIO_InitStruct.Pull = ADC_RESP_NEG_GPIO_PULL;
		HAL_GPIO_Init(ADC_RESP_NEG_GPIO_PORT, &GPIO_InitStruct);

		ADC_AUDIO_POS_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin =  ADC_AUDIO_POS_GPIO_PIN;
		GPIO_InitStruct.Mode = ADC_AUDIO_POS_GPIO_MODE;
		GPIO_InitStruct.Pull = ADC_AUDIO_POS_GPIO_PULL;
		HAL_GPIO_Init(ADC_AUDIO_POS_GPIO_PORT, &GPIO_InitStruct);

		ADC_AUDIO_NEG_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin =  ADC_AUDIO_NEG_GPIO_PIN;
		GPIO_InitStruct.Mode = ADC_AUDIO_NEG_GPIO_MODE;
		GPIO_InitStruct.Pull = ADC_AUDIO_NEG_GPIO_PULL;
		HAL_GPIO_Init(ADC_AUDIO_NEG_GPIO_PORT, &GPIO_InitStruct);

		ADC_PACE_POS_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin =  ADC_PACE_POS_GPIO_PIN;
		GPIO_InitStruct.Mode = ADC_PACE_POS_GPIO_MODE;
		GPIO_InitStruct.Pull = ADC_PACE_POS_GPIO_PULL;
		HAL_GPIO_Init(ADC_PACE_POS_GPIO_PORT, &GPIO_InitStruct);

		ADC_PACE_NEG_GPIO_CLK_ENABLE();
		GPIO_InitStruct.Pin =  ADC_PACE_NEG_GPIO_PIN;
		GPIO_InitStruct.Mode = ADC_PACE_NEG_GPIO_MODE;
		GPIO_InitStruct.Pull = ADC_PACE_NEG_GPIO_PULL;
		HAL_GPIO_Init(ADC_PACE_NEG_GPIO_PORT, &GPIO_InitStruct);

		/* ADC1 interrupt Init */
		HAL_NVIC_SetPriority(ADC1_IRQn, 15, 1);
		HAL_NVIC_EnableIRQ(ADC1_IRQn);


		ADC1_Queue_Config();

		__HAL_LINKDMA(&hadc1, DMA_Handle, handle_GPDMA1_Channel10);

		HAL_DMAEx_List_SetCircularMode(&ADCQueue);
		if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel10, &ADCQueue) != HAL_OK)
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
	}
}

/**
* @brief ADC MSP De-Initialization TODO
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
	  /* USER CODE BEGIN ADC1_MspDeInit 0 */

	    /* USER CODE END ADC1_MspDeInit 0 */
	      /* Peripheral clock disable */
	      __HAL_RCC_ADC1_CLK_DISABLE();

	      /**ADC1 GPIO Configuration
	      PC1     ------> ADC1_IN2
	      PC0     ------> ADC1_IN1

	      PA2     ------> ADC1_IN7
	      PA1 tetete
	      PA6     ------> ADC1_IN11
	      PA5     ------> ADC1_IN12

	      PC4     ------> ADC1_IN13
	      PA7     ------> ADC1_IN14

	      PB1     ------> ADC1_IN16
	      PB0     ------> ADC1_IN15
	      */
	      HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_4);

	      HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6
	                            |GPIO_PIN_7);

	      HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1|GPIO_PIN_0);

	      /* ADC1 interrupt DeInit */
	      HAL_NVIC_DisableIRQ(ADC1_IRQn);
	    /* USER CODE BEGIN ADC1_MspDeInit 1 */

	    /* USER CODE END ADC1_MspDeInit 1 */
  }

}


/**
  * @brief  DMA Linked-list ADCQueue configuration
  * @param  None
  * @retval None
  */
HAL_StatusTypeDef ADC1_Queue_Config(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  /* DMA node configuration declaration */
  DMA_NodeConfTypeDef pNodeConfig;

  /* Set node configuration */
  pNodeConfig.NodeType =           DMA_GPDMA_LINEAR_NODE;
  pNodeConfig.Init.Request =       GPDMA1_REQUEST_ADC1;
  pNodeConfig.Init.BlkHWRequest =  DMA_BREQ_SINGLE_BURST;
  pNodeConfig.Init.Direction =     DMA_PERIPH_TO_MEMORY;
  pNodeConfig.Init.SrcInc =        DMA_SINC_FIXED;
  pNodeConfig.Init.DestInc =       DMA_DINC_INCREMENTED;
  pNodeConfig.Init.SrcDataWidth =  DMA_SRC_DATAWIDTH_WORD;
  pNodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  pNodeConfig.Init.SrcBurstLength = 1;
  pNodeConfig.Init.DestBurstLength = 1;
  pNodeConfig.Init.TransferAllocatedPort =       DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  pNodeConfig.Init.TransferEventMode =           DMA_TCEM_BLOCK_TRANSFER;
  pNodeConfig.TriggerConfig.TriggerPolarity =    DMA_TRIG_POLARITY_MASKED;
  pNodeConfig.DataHandlingConfig.DataExchange =  DMA_EXCHANGE_NONE;
  pNodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  pNodeConfig.SrcAddress = 0;
  pNodeConfig.DstAddress = 0;
  pNodeConfig.DataSize = 0;

  /* Build ADCNode Node */
  ret |= HAL_DMAEx_List_BuildNode(&pNodeConfig, &ADCNode);

  /* Insert ADCNode to Queue */
  ret |= HAL_DMAEx_List_InsertNode_Tail(&ADCQueue, &ADCNode);

   return ret;
}

/**
  * @brief LPTIM2 Initialization Function
  * @param None
  * @retval None
  */
static void LPTIM2_Init(void)
{
	hlptim2.Instance               = LPTIM2;
	hlptim2.Init.Clock.Source      = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	hlptim2.Init.Clock.Prescaler   = LPTIM_PRESCALER_DIV1;
	hlptim2.Init.Trigger.Source    = LPTIM_TRIGSOURCE_SOFTWARE;
	hlptim2.Init.Period            = Autoreload_lptim2;
	hlptim2.Init.UpdateMode        = LPTIM_UPDATE_IMMEDIATE;
	hlptim2.Init.CounterSource     = LPTIM_COUNTERSOURCE_INTERNAL;
	hlptim2.Init.Input1Source      = LPTIM_INPUT1SOURCE_GPIO;
	hlptim2.Init.Input2Source      = LPTIM_INPUT2SOURCE_GPIO;
	hlptim2.Init.RepetitionCounter = 0;


	/* lptim2 initialization */
	if (HAL_LPTIM_Init(&hlptim2) != HAL_OK)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

}/* End of LPTIM2_Init */


/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  hadc: ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint32_t * ch_buf_hold[ADC_CHANNELS]={0,0,0,0,0};
	uint32_t ch_fill_len[ADC_CHANNELS]={0,0,0,0,0};
	uint32_t tmp_index=0,i=0,it=0,is=0;
	//uint32_t deb[ADC_CHANNELS][513]={0};
	//uint32_t deb[ADC_CHANNELS]={0x3232,0x3434,0x3636,0x3838,0x4141};

	/* fetch time stamp and other fields */
	uint32_t ts=0;//RTC_GetTimestampMillis();
	uint32_t seqLen=getadc_sampleSeqLen();

	/* get fresh buffer for each channel */
	for(int i=0;i<ADC_CHANNELS;i++)
	{
		if(cf.bus_chan_config_hndle[i].adc_seq_hndle)
		{
			/* get fresh buffer for channel data */
			if(cf.bus_chan_config_hndle[i].mp != NULL)
			{
				/* get fresh command buffer */
				if(!(ch_buf_hold[i]=(uint32_t*)getMemBuf(cf.bus_chan_config_hndle[i].mp)))
				{
					Error_Handler((uint8_t *)__FILE__, __LINE__);
				}
			}
		}
	}

	/* Computation of ADC conversions raw data to physical values               */
	for (tmp_index = ((ADC_CONVERTED_DATA_BUFFER_SIZE*seqLen)/2); tmp_index < ADC_CONVERTED_DATA_BUFFER_SIZE*seqLen; tmp_index++,i++)
	{
		/* get num of registered channels */
		for(;it<ADC_CHANNELS;)
		{
			if(ch_buf_hold[it]!= NULL)
			{
				int adc_val=aADCxConvertedData[tmp_index];
				ch_buf_hold[it][ch_fill_len[it]++]=adc_val;

				it++;
				is++;
				break;
			}
			it++;
		}

		if(is>=seqLen)
		{
			is=it=0;
		}

		if(it>=ADC_CHANNELS)
		{
			it=0;
		}
	}

	/* forward data buffers for processing */
	for(int i=0;i<ADC_CHANNELS;i++)
	{
		if(cf.bus_chan_config_hndle[i].inv)
		{
			/* add digital invert here */
			for(int j=0;j<ch_fill_len[i];j++)
			{
				ch_buf_hold[i][j]=-(1)*ch_buf_hold[i][j];
			}
		}

		/* forward for processing */
		if(cf.bus_chan_config_hndle[i].cb != NULL)
		{
			cf.bus_chan_config_hndle[i].cb(&ts,ch_buf_hold[i],ch_fill_len[i]);
		}
	}
}/* end of HAL_ADC_ConvCpltCallback */


/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint32_t * ch_buf_hold[ADC_CHANNELS]={0,0,0,0,0};
	uint32_t ch_fill_len[ADC_CHANNELS]={0,0,0,0,0};
	uint32_t tmp_index=0,i=0,it=0,is=0;
	//uint32_t deb[ADC_CHANNELS][513]={0};
	//uint32_t deb[ADC_CHANNELS]={0x3131,0x3333,0x3535,0x3737,0x3939};

	/* fetch time stamp and other fields */
	uint32_t ts=0;//RTC_GetTimestampMillis();
	uint32_t seqLen=getadc_sampleSeqLen();

	/* get fresh buffer for each channel */
	for(int i=0;i<ADC_CHANNELS;i++)
	{
		if(cf.bus_chan_config_hndle[i].adc_seq_hndle)
		{
			/* get fresh buffer for channel data */
			if(cf.bus_chan_config_hndle[i].mp != NULL)
			{
				/* get fresh command buffer */
				if(!(ch_buf_hold[i]=(uint32_t*)getMemBuf(cf.bus_chan_config_hndle[i].mp)))
				{
					Error_Handler((uint8_t *)__FILE__, __LINE__);
				}
			}
		}
	}

//	uint8_t stm=0;
//	for (tmp_index = 0; tmp_index < ADC_CONVERTED_DATA_BUFFER_SIZE*5; tmp_index++)
//	{
//       switch(stm){
//       case(0):{
//    	   stm=1;
//    	   aADCxConvertedDataDEB[tmp_index]=0x3030;
//    	   break;
//       }
//       case(1):{
//    	   stm=2;
//    	   aADCxConvertedDataDEB[tmp_index]=0x3131;
//    	   break;
//        }
//       case(2):{
//    	   stm=3;
//    	   aADCxConvertedDataDEB[tmp_index]=0x3232;
//    	   break;
//        }
//       case(3):{
//    	   stm=4;
//    	   aADCxConvertedDataDEB[tmp_index]=0x3333;
//    	   break;
//        }
//       case(4):{
//    	   stm=0;
//    	   aADCxConvertedDataDEB[tmp_index]=0x3434;
//    	   break;
//        }
//       }
//	}


	/* Computation of ADC conversions raw data to physical values               */
	for (tmp_index = 0; tmp_index < ((ADC_CONVERTED_DATA_BUFFER_SIZE*seqLen)/2); tmp_index++,i++)
	{

		/* get num of registered channels */
		for(;it<ADC_CHANNELS;)
		{
			if(ch_buf_hold[it]!= NULL)
			{
				ch_buf_hold[it][ch_fill_len[it]++]=aADCxConvertedData[tmp_index];
				it++;
				is++;
				break;
			}
			it++;
		}

		if(is>=seqLen)
		{
			is=it=0;
		}

		if(it>=ADC_CHANNELS)
		{
			it=0;
		}
	}

	/* forward data buffers for processing */
	for(int i=0;i<ADC_CHANNELS;i++)
	{
		/* forward for processing */
		if(cf.bus_chan_config_hndle[i].cb != NULL)
		{
			if(cf.bus_chan_config_hndle[i].inv)
			{
				/* add digital invert here */
				for(int j=0;j<ch_fill_len[i];j++)
				{
					ch_buf_hold[i][j]=-(1)*ch_buf_hold[i][j];
				}
			}

			cf.bus_chan_config_hndle[i].cb(&ts,ch_buf_hold[i],ch_fill_len[i]);
		}
	}
}/* end of HAL_ADC_ConvHalfCpltCallback */

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	  /* In case of ADC error, call main error handler */
	  Error_Handler((uint8_t *)__FILE__, __LINE__);
}



