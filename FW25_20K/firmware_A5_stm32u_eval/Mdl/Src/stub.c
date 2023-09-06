/**
* @file stub.c
* @brief stub functionality
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 05.11.2015
*/

#if 0

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "mathconst.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "adc_task.h"
#include "msg_type.h"
#include "membuf.h"
#include "packetbuf.h"
#include "cmd.h"
#include "stub.h"
#include "gcell1frame.h"
#include "syserr.h"
#include "i2c.h"

extern I2C_Params i2c0DevParams;
extern const struct I2C_Params I2C_PARAMS;

void efmI2cInit(void);

void initI2C0DevParams(void)
{
	memcpy(&i2c0DevParams, &I2C_PARAMS, sizeof(i2c0DevParams));

	i2c0DevParams.opMode=I2C_OpMode_INTERRUPT;
	i2c0DevParams.bitRate=I2C_BitRate_100K;
	efmI2cInit();
}






#ifdef KUKU
#define GCELL1_TX_BUFFER_SIZE	264
#define N_ADC_TX_BUFFERS	3
#define GCELL1_SAMPLE_SETS_IN_FRAME 16

typedef struct sGcell1SampleSet
{
	uint32_t ts;
	uint8_t ch0_lo;
	uint8_t ch0_hi:4;
	uint8_t ch1_hi:4;
	uint8_t ch1_lo;
	uint8_t ch2_lo;
	uint8_t ch2_hi:4;
	uint8_t ch3_hi:4;
	uint8_t ch3_lo;
	uint8_t ch4_lo;
	uint8_t ch4_hi:4;
	uint8_t ch5_hi:4;
	uint8_t ch5_lo;
	uint8_t ch6_lo;
	uint8_t ch6_hi:4;
	uint8_t ch7_hi:4;
	uint8_t ch7_lo;
} GCELL1_SAMPLE_SET;

xTimerHandle adcStubTimer=NULL;

struct sWave
{
	float scale;
	float offset;
	float pu;
	float (*f)(float arg);
};

struct sWaveGen
{
	uint32_t ts;
	float dpu_dt[8];
	struct sWave wave[8];
};

struct sLogControl
{
	uint8_t active;
	uint8_t in_frame_cnt;
	uint16_t autostop_sel;
	uint32_t count_time;
	uint32_t count;
	uint32_t elapsed_time;
	GCELL1_SAMPLE_SET sample_set[GCELL1_SAMPLE_SETS_IN_FRAME];
};

int sendToAux(void *packet, uint16_t hdr, uint32_t timeout);

static void adcStubTimerCallback(xTimerHandle pxTimer);
static int handleAdcStubCmd(uint16_t cmd, struct sSyncCmdBuffer *cmdBuf, struct sLogControl *logControl, MEMBUF_POOL *pool);
static void generateSampleSet(uint32_t ts, GCELL1_SAMPLE_SET *sample_set, struct sWaveGen *wave_gen);
static float genWaveSample(struct sWave *wave);
static float genDCWave(float pu);
static float genSinWave(float pu);
static float genCosWave(float pu);
static float genSquareWave(float pu);
static float genSawtoothWave(float pu);
static float genTriangularWave(float pu);



int startAdcStubTask(QueueHandle_t *adcQ, TaskHandle_t *adcT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio)
{
	QueueHandle_t q;
	TaskHandle_t t;

	/* create queue */
	if ((q = xQueueCreate(24,sizeof(MSG_HDR))) == NULL)
	{
		if (adcQ)
			*adcQ=NULL;
		if (adcT)
			*adcT=NULL;
		return pdFAIL;
	}

	if (pdFAIL == xTaskCreate(adcStubTask, pcName, usStackDepth, q, prio, &t))
	{
		vQueueDelete(q);
		if (adcQ)
			*adcQ=NULL;
		if (adcT)
			*adcT=NULL;
		return pdFAIL;
	}
	vQueueAddToRegistry( q, pcName);
	if (adcQ)
		*adcQ=q;
	if (adcT)
		*adcT=t;
	return pdPASS;
}

void adcStubTask(void *para)
{
	QueueHandle_t q=(QueueHandle_t)para;
	MSG_HDR msg;
	PACKETBUF_HDR *pkt;
	MEMBUF_POOL txBufPool;
	void *p;
	uint32_t ts=0;
	struct sWaveGen wave_gen;
	uint8_t syncWaveGen=1;
	struct sLogControl logControl;

	memset(&wave_gen, 0, sizeof(wave_gen));
	wave_gen.ts=0;
	wave_gen.dpu_dt[0]=1.0;
	wave_gen.dpu_dt[1]=1.0;
	wave_gen.dpu_dt[2]=1.0;
	wave_gen.dpu_dt[3]=1.0;
	wave_gen.dpu_dt[4]=1.0;
	wave_gen.dpu_dt[5]=1.0;
	wave_gen.dpu_dt[6]=1.0;
	wave_gen.dpu_dt[7]=1.0;

	wave_gen.wave[0].scale=1.0;
	wave_gen.wave[1].scale=1.0;
	wave_gen.wave[2].scale=1.0;
	wave_gen.wave[3].scale=1.0;
	wave_gen.wave[4].scale=1.0;
	wave_gen.wave[5].scale=1.0;
	wave_gen.wave[6].scale= -1.0;
	wave_gen.wave[7].scale= -1.0;

	wave_gen.wave[0].f=genDCWave;
	wave_gen.wave[1].f=genSquareWave;
	wave_gen.wave[2].f=genSawtoothWave;
	wave_gen.wave[3].f=genTriangularWave;
	wave_gen.wave[4].f=genSinWave;
	wave_gen.wave[5].f=genCosWave;
	wave_gen.wave[6].f=genTriangularWave;
	wave_gen.wave[7].f=genSquareWave;

	/*
	** Create tx buffers pool
	*/
	p=pvPortMalloc((GCELL1_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_ADC_TX_BUFFERS);
	initMemBufPool(&txBufPool,p,(GCELL1_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_ADC_TX_BUFFERS, GCELL1_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR),N_ADC_TX_BUFFERS);

	adcStubTimer=xTimerCreate( "adcs", 10, pdTRUE, (void * const)q, adcStubTimerCallback );
	memset(&logControl,0,sizeof(logControl));

	for (;;)
	{
		if (xQueueReceive(q, &msg, portMAX_DELAY))
		{
			if (msg.hdr.bit.type==MSG_TYPE_EVENT)
			{
				ts+=10;
				if (syncWaveGen)
				{
					wave_gen.ts=ts;
					syncWaveGen=0;
				}
				generateSampleSet(ts,&logControl.sample_set[logControl.in_frame_cnt], &wave_gen);
				logControl.in_frame_cnt++;
				logControl.count++;

				if (logControl.active)
				{

					switch (logControl.autostop_sel)
					{
					case REC_LIMIT_NONE:

						break;
					case REC_LIMIT_IN_SAMPLES:
						if (logControl.count==logControl.count_time)
						{
							logControl.active=false;
							xTimerStop(adcStubTimer, portMAX_DELAY);
						}
						break;
					case REC_LIMIT_IN_MS:
						break;
					}
				}

				if (GCELL1_SAMPLE_SETS_IN_FRAME==logControl.in_frame_cnt || (logControl.active==false && 0<logControl.in_frame_cnt))
				{
					pkt=makeGcell1StreamFrame(&txBufPool, GCELL1_FRAME_SYNC_DSTREAM_FIRST, logControl.sample_set, logControl.in_frame_cnt*sizeof(GCELL1_SAMPLE_SET), portMAX_DELAY);
					if (pdPASS!=sendToAux(pkt,MSGHDR_SAMPLER,portMAX_DELAY))
						retMemBuf(pkt);
					logControl.in_frame_cnt=0;
				}
			}
			else if (msg.hdr.bit.type==MSG_TYPE_CMD)
			{
				handleAdcStubCmd(msg.data, (struct sSyncCmdBuffer *)msg.buf, &logControl, &txBufPool);
			}

		}
	}
}

static int handleAdcStubCmd(uint16_t cmd, struct sSyncCmdBuffer *cmdBuf, struct sLogControl *logControl, MEMBUF_POOL *pool)
{
	PACKETBUF_HDR *pkt;

	switch (cmd)
	{
	case CMD_NOP:
		break;
	case CMD_START_LOG:
		if (cmdBuf)
		{
			if (adcStubTimer)
			{
				if (pdFALSE==xTimerIsTimerActive(adcStubTimer))
				{
					if (cmdBuf->userInData)
					{
						logControl->active=true;
						logControl->autostop_sel=((struct sSamplerStartLogCmd *)cmdBuf->userInData)->autostop_sel;
						logControl->count_time=((struct sSamplerStartLogCmd *)cmdBuf->userInData)->count_time;
						logControl->count=0;
						logControl->elapsed_time=0;
					}
					if (pdPASS!=xTimerStart(adcStubTimer, portMAX_DELAY))
					{
						if (cmdBuf->userOutData)
							((struct sSamplerStartLogCmdResp *)cmdBuf->userOutData)->errCode=E_TIMEOUT;
					}
					else
					{
						if (cmdBuf->userOutData)
							((struct sSamplerStartLogCmdResp *)cmdBuf->userOutData)->errCode=E_NOERR;
					}
				}
			}
			else
			{
				if (cmdBuf->userOutData)
					((struct sSamplerStartLogCmdResp *)cmdBuf->userOutData)->errCode=E_BUSY;
			}
			if (cmdBuf->userCallback)
			{
				(*cmdBuf->userCallback)(cmdBuf->userCallbackArg);
			}
		}
		break;
	case CMD_STOP_LOG:
		if (cmdBuf)
		{
			if (pdTRUE==xTimerIsTimerActive(adcStubTimer))
			{
				logControl->active=false;
				if (pdPASS!=xTimerStop(adcStubTimer, portMAX_DELAY))
				{
					if (cmdBuf->userOutData)
					{
						((struct sSamplerStopLogCmdResp *)cmdBuf->userOutData)->errCode=E_TIMEOUT;
						((struct sSamplerStopLogCmdResp *)cmdBuf->userOutData)->autostop_sel=logControl->autostop_sel;
						((struct sSamplerStopLogCmdResp *)cmdBuf->userOutData)->count_time=logControl->count_time;
						((struct sSamplerStopLogCmdResp *)cmdBuf->userOutData)->cycles_count=logControl->count;
						((struct sSamplerStopLogCmdResp *)cmdBuf->userOutData)->elapsed_time=logControl->elapsed_time;
					}
				}
				else
				{
					if (cmdBuf->userOutData)
					{
						((struct sSamplerStopLogCmdResp *)cmdBuf->userOutData)->errCode=E_NOERR;
						((struct sSamplerStopLogCmdResp *)cmdBuf->userOutData)->autostop_sel=logControl->autostop_sel;
						((struct sSamplerStopLogCmdResp *)cmdBuf->userOutData)->count_time=logControl->count_time;
						((struct sSamplerStopLogCmdResp *)cmdBuf->userOutData)->cycles_count=logControl->count;
						((struct sSamplerStopLogCmdResp *)cmdBuf->userOutData)->elapsed_time=logControl->elapsed_time;
					}
					if (logControl->active==false && 0<logControl->in_frame_cnt)
					{
						pkt=makeGcell1StreamFrame(pool, GCELL1_FRAME_SYNC_DSTREAM_FIRST, logControl->sample_set, logControl->in_frame_cnt*sizeof(GCELL1_SAMPLE_SET), portMAX_DELAY);
						if (pdPASS!=sendToAux(pkt,MSGHDR_SAMPLER,portMAX_DELAY))
							retMemBuf(pkt);
						logControl->in_frame_cnt=0;
					}
				}
			}
			if (cmdBuf->userCallback)
			{
				(*cmdBuf->userCallback)(cmdBuf->userCallbackArg);
			}
		}
		break;
	case CMD_GET_GAIN:
		break;
	case CMD_SET_GAIN:
		break;
	default:
		break;
	}
	return 0;
}

static void adcStubTimerCallback(xTimerHandle pxTimer)
{
	MSG_HDR msg;
	void *p;

	if (pxTimer==NULL)
		return;
	if (NULL==(p=pvTimerGetTimerID(pxTimer)))
		return;
	msg.hdr.all = MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR, MSG_TYPE_EVENT);
	msg.data=0;
	msg.buf=NULL;
	xQueueSend((xQueueHandle)p,&msg,0);
}

static void generateSampleSet(uint32_t ts, GCELL1_SAMPLE_SET *sample_set, struct sWaveGen *wave_gen)
{
	int16_t val;
	uint32_t dts;
	float dt;
	float dpu;

	dts=ts-sample_set->ts;
	sample_set->ts=ts;
	dt=dts*(1.0/configTICK_RATE_HZ);
	/* Channel 0 - 0 */
	dpu=wave_gen->dpu_dt[0]*dt;
	wave_gen->wave[0].pu=fmodf(wave_gen->wave[0].pu+dpu,1.0);
	val=genWaveSample(&wave_gen->wave[0])*2048.0;
	sample_set->ch0_lo=val &0xff;
	sample_set->ch0_hi=(val>>8) &0xf;

	/* Channel 1 - sawtooth waveform */
	dpu=wave_gen->dpu_dt[1]*dt;
	wave_gen->wave[1].pu=fmodf(wave_gen->wave[1].pu+dpu,1.0);
	val=genWaveSample(&wave_gen->wave[1])*2048.0;
	sample_set->ch1_lo=val &0xff;
	sample_set->ch1_hi=(val>>8) &0xf;

	/* Channel 2 - square waveform */
	dpu=wave_gen->dpu_dt[2]*dt;
	wave_gen->wave[2].pu=fmodf(wave_gen->wave[2].pu+dpu,1.0);
	val=genWaveSample(&wave_gen->wave[2])*2048.0;
	sample_set->ch2_lo=val &0xff;
	sample_set->ch2_hi=(val>>8) &0xf;

	/* Channel 3 - sine waveform */
	dpu=wave_gen->dpu_dt[3]*dt;
	wave_gen->wave[3].pu=fmodf(wave_gen->wave[3].pu+dpu,1.0);
	val=genWaveSample(&wave_gen->wave[3])*2048.0;
	sample_set->ch3_lo=val &0xff;
	sample_set->ch3_hi=(val>>8) &0xf;

	/* Channel 4 - cosine waveform */
	dpu=wave_gen->dpu_dt[4]*dt;
	wave_gen->wave[4].pu=fmodf(wave_gen->wave[4].pu+dpu,1.0);
	val=genWaveSample(&wave_gen->wave[4])*2048.0;
	sample_set->ch4_lo=val &0xff;
	sample_set->ch4_hi=(val>>8) &0xf;

	/* Channel 5 - triangular waveform */
	dpu=wave_gen->dpu_dt[5]*dt;
	wave_gen->wave[5].pu=fmodf(wave_gen->wave[5].pu+dpu,1.0);
	val=genWaveSample(&wave_gen->wave[5])*2048.0;
	sample_set->ch5_lo=val &0xff;
	sample_set->ch5_hi=(val>>8) &0xf;

	/* Channel 6 - Periodic shock wave */
	dpu=wave_gen->dpu_dt[6]*dt;
	wave_gen->wave[6].pu=fmodf(wave_gen->wave[6].pu+dpu,1.0);
	val=genWaveSample(&wave_gen->wave[6])*2048.0;
	sample_set->ch6_lo=val &0xff;
	sample_set->ch6_hi=(val>>8) &0xf;

	/* Channel 7 - 0 */
	dpu=wave_gen->dpu_dt[7]*dt;
	wave_gen->wave[7].pu=fmodf(wave_gen->wave[7].pu+dpu,1.0);
	val=genWaveSample(&wave_gen->wave[7])*2048.0;
	sample_set->ch7_lo=val &0xff;
	sample_set->ch7_hi=(val>>8) &0xf;
}


int sendToSampler(QueueHandle_t *q, void *p, uint16_t data, uint16_t hdr, uint32_t timeout)
{
	MSG_HDR msg;

	if (q==NULL)
		return pdFAIL;
	msg.hdr.all = hdr;
	msg.data=data;
	msg.buf=p;
	return xQueueSend(q,&msg,timeout);
}

static float genWaveSample(struct sWave *wave)
{
	float val;
	if (wave->f)
		val=(*wave->f)(wave->pu);
	else
		val=0.0;
	val = (val * wave->scale) + wave->offset;
	return val;
}

static float genDCWave(float pu)
{
	return 0.0;
}

static float genSinWave(float pu)
{
	return sinf((2.0*PI)*pu);
}

static float genCosWave(float pu)
{
	return cosf((2.0*PI)*pu);
}
static float genSquareWave(float pu)
{
	return (pu<0.5) ? 1.0 : -1.0;
}

static float genSawtoothWave(float pu)
{
	return (pu-1.0)*2.0;
}

static float genTriangularWave(float pu)
{
	return 4.0 * ((pu<0.25) ? pu : (pu<0.75) ? 0.25-pu : pu-1.0);
}

#endif

#endif
