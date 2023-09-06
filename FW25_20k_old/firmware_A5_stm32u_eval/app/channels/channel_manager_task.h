/**************************************************************************//**
* @file channel_manager_task.c
* @brief channel manager
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef CHANNEL_MANAGER_TASK_H_
#define CHANNEL_MANAGER_TASK_H_

#include <stdint.h>
#include <stdbool.h>
#include "RpcFifo.h"

#include "../../app/Inc/periodic_dispatcher_task.h"


typedef enum CHAN_cmd
{
	CHAN_CMD_START_ALL,
	CHAN_CMD_STOP_ALL,
	CHAN_CMD_PROC_STEAM,
	CHAN_CMD_MAX
}CHAN_cmd_t;

typedef enum CHAN_Channels
{
	CHAN_ECG_H    =0,    /* ecg horizontal */
	CHAN_PULSE    =1,    /* pulse acc e1/acc e2*/
	CHAN_ACC3X    =2,    /* acc 3 axes (xyz) */
	CHAN_AN_EMG   =3,    /* analog in ext emg*/
	CHAN_AN_SCL   =4,    /* analog in scl */
	CHAN_AN_RESP  =5,    /* analog in respiration */
	CHAN_AN_AUDIO =6,    /* analog in audio */
	CHAN_AN_PACE  =7,    /* analog in piezo pace */
	CHAN_ECG_V    =8,    /* CHAN_ECG_V ecg vertical*/
	CHAN_TEMP_SKIN = 9,  /* Temperatures: skin,control,ambient*/
	CHAN_GYR3X    =10,   /* GYR 3 axes (xyz) */
	CNAH_BIOZ     =11,
	CHAN_MAX
}CHAN_Channels_t;

typedef struct	{

	uint16_t sampleRate;
	bool IsActive;
	uint8_t samplebits;
}CHAN_Params_t;

typedef struct	{
	fifo_t * pfifoStreamOut;///  stream out fifo
	uint16_t buffSize;
}CHAN_Vals_t;

typedef struct	{
	uint32_t ts;
	uint32_t num;
	uint32_t id;
	void * parg;
}CHAN_Proc_Stream_Vals_t;


typedef bool(*CHAN_ProcessingFxn_t)(void*);
typedef void(*CHAN_ActivationFxn_t)(void*, bool cmd);
typedef void(*CHAN_InitFxn_t)(void*);
typedef void(*CHAN_ResetFxn_t)();

// Channel descriptor structure. Reflects all channel properties
typedef struct	{
	char 					Name[16];
	bool 					IsSync;
	uint16_t                SampleRes;  //resolution
	uint16_t			    SampleRate;	// Hz.
	CHAN_InitFxn_t			InitFxn;
	CHAN_ResetFxn_t			ResetFxn;
	CHAN_ActivationFxn_t	ActivationFxn;
	CHAN_ProcessingFxn_t	ProcessingFxn;
	char					XducerType[80];
	char					PhysicalDimension[8];	// e.g. "g"
	int32_t					PhysicalMin;
	int32_t					PhysicalMax;
	int32_t					DigitalMin;	// e.g. ADXL362: -2048 for -2g
	int32_t					DigitalMax;
	char					FilterType[80];
}CHAN_Desc_t;

typedef struct	{
	CHAN_Desc_t		const *Desc;	// pointer to const
	CHAN_Params_t	Params;
	CHAN_Vals_t		Vals;
}CHAN_Struct_t;

void CHAN_Reset();
void CHAN_PrepareChannels();

int CHAN_GetChannelNumber();
int CHAN_GetActiveChannelNumber();
int CHAN_GetChannelSampleRate(CHAN_Channels_t ch);
int CHAN_ChanneData2record(unsigned char * rec_out, uint32_t len);

fifo_t* CHAN_GetChannelfifo(CHAN_Channels_t ch);


// Utility functions used by Parser module
bool CHAN_GetChannelNameStr(CHAN_Channels_t ch, char *str);
bool CHAN_GetChannelXducerStr(CHAN_Channels_t ch, char *str);
bool CHAN_GetChannelPhysDimStr(CHAN_Channels_t ch, char *str);
bool CHAN_GetChannelFilterStr(CHAN_Channels_t ch, char *str);
bool CHAN_GetChannelIsSyncStr(CHAN_Channels_t ch, char *str);
bool CHAN_GetChannelSampleRateStr(CHAN_Channels_t ch, char *str);
bool CHAN_GetChannelPhysMinStr(CHAN_Channels_t ch, char *str);
bool CHAN_GetChannelPhysMaxStr(CHAN_Channels_t ch, char *str);
bool CHAN_GetChannelDigMinStr(CHAN_Channels_t ch, char *str);
bool CHAN_GetChannelDigMaxStr(CHAN_Channels_t ch, char *str);
//bool CHAN_GetChannelNumberStr(char *str);
bool CHAN_GetTimeframeStr(char *str);
//bool CHAN_GetChannelIsActiveStr(CHAN_Channels_t ch, char *str);
bool CHAN_GetChannelInputStr(CHAN_Channels_t ch, char *str);

int32_t CHAN_GetallChannelNumber(uint32_t * num);
uint32_t CHAN_GetActiveChannelsNumber(void);

int32_t CHAN_GetallChannelConfig(char * buff);
int32_t CHAN_GetallChannelConfig_C(char * buff);
int32_t CHAN_SetChannelActivation(CHAN_Channels_t ch, uint32_t active);
int32_t CHAN_GetChannelActivation(CHAN_Channels_t ch, uint32_t *active);

int32_t CHAN_GetChannelRate(CHAN_Channels_t ch, int32_t * rate);

int startChanMngTask(QueueHandle_t *cmdQ, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio);
int sendToChannelManager(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr, uint32_t timeout);
int sendToChannelManagerFromISR(QueueHandle_t q, void *packet,uint16_t data, uint16_t hdr);

extern uint16_t streaming_dev_num;//read only
extern const char Channels_resp[];
extern const char C_resp[];

#endif /* CHANNEL_MANAGER_TASK_H_ */
