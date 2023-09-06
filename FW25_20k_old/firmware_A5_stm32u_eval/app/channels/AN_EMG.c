/*
* @file AN_EMG.c
* @brief Analog-emg channel handler
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include <string.h>

#include "AN_EMG.h"
#include "channel_manager_task.h"
#include "main.h"
#include "bus.h"
#include "rtc.h"
#include "arm_math.h"
#include "sys_conf.h"

extern QueueHandle_t chanmngq;
#define MOD_NAME "[AN EMG]"
#define CMD_BUFF_SIZE	        sizeof(CHAN_Proc_Stream_Vals_t)
#define N_CMD_BUFFERS	        (40)

#define N_DATA_BUFFERS	        (2)

#if ENABLE_AN_EMG_CHAN == 1
static fifo_t fifoStreamOut;
static int32_t	* fifoBuff;
static bool AN_EMG_move_data_Handler(void *p, void *p2,uint32_t len);



ADC_ChannelConfTypeDef an_emg_conf=
{
	.Channel      = ADC_CHANNEL_15,
	.Rank         = ADC_REGULAR_RANK_1,
	.SamplingTime = ADC_SAMPLETIME_391CYCLES_5,
	.SingleDiff   = ADC_DIFFERENTIAL_ENDED,
	.OffsetNumber = ADC_OFFSET_1,
	.Offset       = ADC_OFFSET_VAL
};

#endif

MEMBUF_POOL aemg_cmdPool;
MEMBUF_POOL aemg_dataPool;

uint32_t acc_rec_len_emg=0;

float32_t FIR_dec1_coeffs[] = {
  -142.9442739751772820E-6,
  -252.2211831462206530E-6,
  -347.5945623945052030E-6,
  -321.0839601413753140E-6,
  -97.22225175627986000E-6,
  326.3117239250896090E-6,
  836.1439112274239280E-6,
  0.001209027523486800,
  0.001184222537420025,
  590.9136933192837660E-6,
  -520.3527006113926060E-6,
  -0.001812698722149656,
  -0.002724775642735784,
  -0.002673582490820696,
  -0.001340728074926084,
  0.001078011666779184,
  0.003797965250984124,
  0.005638404522781618,
  0.005468470132752291,
  0.002761334048239419,
  -0.001980549872799683,
  -0.007167492106103557,
  -0.010557955026781203,
  -0.010112507043340404,
  -0.004978767523812532,
  0.003808453095602158,
  0.013295208276611466,
  0.019416922705541947,
  0.018489926838076525,
  0.008904119903017756,
  -0.007643552646951099,
  -0.025966841359731561,
  -0.038527883680910445,
  -0.037684191047641612,
  -0.018373491864758760,
  0.019689814633387617,
  0.071157875324433559,
  0.125978118210888251,
  0.171947745227487625,
  0.198145667117833019,
  0.198145667117833019,
  0.171947745227487625,
  0.125978118210888251,
  0.071157875324433559,
  0.019689814633387617,
  -0.018373491864758760,
  -0.037684191047641612,
  -0.038527883680910445,
  -0.025966841359731561,
  -0.007643552646951099,
  0.008904119903017756,
  0.018489926838076525,
  0.019416922705541947,
  0.013295208276611466,
  0.003808453095602158,
  -0.004978767523812532,
  -0.010112507043340404,
  -0.010557955026781203,
  -0.007167492106103557,
  -0.001980549872799683,
  0.002761334048239419,
  0.005468470132752291,
  0.005638404522781618,
  0.003797965250984124,
  0.001078011666779184,
  -0.001340728074926084,
  -0.002673582490820696,
  -0.002724775642735784,
  -0.001812698722149656,
  -520.3527006113926060E-6,
  590.9136933192837660E-6,
  0.001184222537420025,
  0.001209027523486800,
  836.1439112274239280E-6,
  326.3117239250896090E-6,
  -97.22225175627986000E-6,
  -321.0839601413753140E-6,
  -347.5945623945052030E-6,
  -252.2211831462206530E-6,
  -142.9442739751772820E-6
};
#define  N_DEC_COEFFS  sizeof(FIR_dec1_coeffs)/sizeof(FIR_dec1_coeffs[0])
//arm_fir_decimate_instance_f32 FIR_dec1;

float32_t FIR_decim_state[1024 + N_DEC_COEFFS - 1];

// decimation with 80 tap FIR lowpass
arm_fir_decimate_instance_f32 FIR_dec1;

/**
 * @brief AN_EMG_Init
 * @brief This function used for resource allocation
 * and general initialization
 * @param p[in/out] -channel data structure
 * @returns none
 */
void AN_EMG_Init(void *p)
{
#if ENABLE_AN_EMG_CHAN == 1

	CHAN_Struct_t *_p = p;
	uint16_t size = 0;

	if(_p==NULL)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		return;
	}

	/* allocate fifo buffer for this channel */
	if(!_p->Vals.pfifoStreamOut)
	{
		uint32_t data_len=0;

		/* allocate fifo buffer for this channel */
		size = \
		(uint16_t)(_p->Params.sampleRate*4)*1.0/(DISP_INTERVALSEC)*sizeof(uint32_t)*AN_EMG_NUM_OF_SENSORS;
		if(NULL == (fifoBuff=pvPortMalloc(size))){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
		/* create fifo for this channel */
		fifo_init(&fifoStreamOut, fifoBuff,(size/sizeof(uint32_t)));
		_p->Vals.pfifoStreamOut=&fifoStreamOut;


		/* allocate memory area for the memory pool */
		p=_MEM_alloc((CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS);
		if(p==NULL){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
		/* Create memory pool of fixed size buffers  */
		if(initMemBufPool(&aemg_cmdPool,p,
				(CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_BUFFERS,
				 CMD_BUFF_SIZE+sizeof(PACKETBUF_HDR),N_CMD_BUFFERS)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}


		/* allocate memory area for the memory pool */
		data_len=_p->Params.sampleRate* (DISP_INTERVALSEC)*sizeof(uint32_t);
		p=_MEM_alloc((data_len+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_DATA_BUFFERS);
		if(p==NULL){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}
		/* Create memory pool of fixed size buffers  */
		if(initMemBufPool(&aemg_dataPool,p,
				(data_len+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_DATA_BUFFERS,
				 data_len+sizeof(PACKETBUF_HDR),N_DATA_BUFFERS)){
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/*
			[in,out]	S	points to an instance of the floating-point FIR decimator structure
			[in]	numTaps	number of coefficients in the filter (15)
			[in]	M	decimation factor (8 for RESP, 2 for EMG)
			[in]	pCoeffs	points to the filter coefficients (my mail)
			[in]	pState	points to the state buffer (note required size of this buffer in documentation)
			[in]	blockSize	number of input samples to process per call (1024)
		*/

		/* DSP related initialization */
		if(arm_fir_decimate_init_f32(&FIR_dec1, N_DEC_COEFFS, 4, FIR_dec1_coeffs, FIR_decim_state,1024)) {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

	}

	/* check if channel is active */
	if(true ==_p->Params.IsActive)
	{
		adc_reg_param_t p={0};

		p.sn=ADC1_CH0;
		p.cb=AN_EMG_move_data_Handler;
		p.mp=&aemg_dataPool;
		p.adc_seq_hndle=(ADC_ChannelConfTypeDef*)&an_emg_conf;
		p.inv=0;

	    /* stream channel initialization */
		if(BUS_ADC1_init_and_register(&p))
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* print informative log message */
	    char * str="\r\n[AN_EMG] init done\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}
	else
	{
	    /* stream channel initialization */
		if(BUS_ADC1_deinit_and_unregister(ADC1_CH0))
		{
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		char * str="\r\n[AN_EMG] disabled! (deinit)\r\n";
	    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
	}

#endif

}/* end of AN_EMG_Init */


/**
 * @brief AN_EMG_SetActivation
 * @brief This function activates device
 * and starts streaming
 * @param p[in/out] -channel data structure
 * @returns none
 */
void AN_EMG_SetActivation(void *p, bool cmd)
{

#if ENABLE_AN_EMG_CHAN == 1

	CHAN_Struct_t *_p = p;

	if(_p==NULL)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		return;
	}

	/* check ECG stream  configuration */
	if(_p->Params.IsActive)
	{
		if(true==cmd)
		{
			/* clear fifo before stopping */
			fifo_clear(CHAN_GetChannelfifo(CHAN_AN_EMG));

			/* start bus execution */
			if(BUS_ADC1_exec_run())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

			char * str="\r\n[AN_EMG] ADC bus activated\r\n";
		    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
		}
		else
		{
			/* clear fifo before stopping */
			fifo_clear(CHAN_GetChannelfifo(CHAN_AN_EMG));

			/* abort bus execution */
			if(BUS_ADC1_exec_stop())
			{
				Error_Handler((uint8_t *)__FILE__, __LINE__);
			}

			char * str="\r\n[AN_EMG] ADC bus deactivated\r\n";
		    aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
		}

	}/* if(_p->Params.IsActive) */

#endif

}/* End of AN_EMG_SetActivation */


/**
 * @brief AN_EMG_Handler
 * @brief this function handles recieved samples
 * @param sample
 * @returns bool
 */
bool AN_EMG_Handler(void *p)
{
	fifo_t *an_emg_fifo=CHAN_GetChannelfifo(CHAN_AN_EMG);
	uint32_t * pbuf=(uint32_t*)((CHAN_Proc_Stream_Vals_t*)p)->parg;

	if(!pbuf){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	acc_rec_len_emg+=((CHAN_Proc_Stream_Vals_t*)p)->num;

	/* insert time stamp in the queue */
	if (-1 == fifo_put32(an_emg_fifo,acc_rec_len_emg)){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* insert data length in the queue */
	if (-1 == fifo_put32(an_emg_fifo,((CHAN_Proc_Stream_Vals_t*)p)->num )){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	if(((CHAN_Proc_Stream_Vals_t*)p)->num > 512)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
	/* insert sample in the queue */
	for (int i=0; i<((CHAN_Proc_Stream_Vals_t*)p)->num; i++)
	{
		/* insert sample e1 in the queue */
	    if (-1 == fifo_put32(an_emg_fifo, pbuf[i])) {
	    	Error_Handler((uint8_t *)__FILE__, __LINE__);
	    }
	}/* for (int i=0; i<FIFO_tsh; i++) */

	/* free allocated memory */
	retMemBuf(pbuf);

	return true;
}/* end of AN_EMG_Handler */

/**
 * @brief AN_EMG_Reset
 * @brief TBD
 * @returns none
 */
void AN_EMG_Reset()
{

}/* End of AN_EMG_Reset */


/**
 * @brief AN_EMG_move_data_Handler
 * @brief when the data is ready pass it for processing
 * @param *p - argument handler
 * @returns bool
 */
static bool AN_EMG_move_data_Handler(void *p, void *p2, uint32_t len)
{
	CHAN_Proc_Stream_Vals_t * buff=NULL;

	/* get fresh command buffer */
	if(!(buff=(CHAN_Proc_Stream_Vals_t*)getMemBuf(&aemg_cmdPool))){
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* fetch time stamp and other fields */
	buff->ts=*((uint32_t*)p);
	buff->id=CHAN_AN_EMG;
	buff->num=len;
	buff->parg=p2;

	/* send to channel manager command from ISR */
	if (pdPASS!=sendToChannelManagerFromISR(chanmngq, buff,
		CHAN_CMD_PROC_STEAM, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_DBRX,MSG_TYPE_CMD)))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	return true;
}/* end of AN_EMG_Handler */

