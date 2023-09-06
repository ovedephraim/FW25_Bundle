/* serial channles */
#include "ACC3X.h"
#include "ECG_H.h"
#include "PULSE.h"
#include "ECG_V.h"
#include "TEMP_SKIN.h"
#include "GYR3X.h"
#include "BIOZ.h"

/* Analog channels */
#include "AN_PACE.h"
#include "AN_EMG.h"
#include "AN_RESP.h"
#include "AN_SCL.h"
#include "AN_AUDIO.h"

#include "channel_manager_task.h"

/* Name	, IsSync, SampleRate, Init_Fxn, Reset_Fxn, ActivationFxn, ProcessingFxn  , Xducer, PhsDim,Min, Max	, DigMin, DigMax, Filter */
const CHAN_Desc_t ChannelDesc[CHAN_MAX] =
{
	/* ID0 */
	{"ECG_CH1_HIRIZ", true,
			DEF_ECG_H_SAMPLE_RES,
			DEF_ECG_H_SAMPLE_RATE,
			ECG_H_Init,
			ECG_H_Reset,
			ECG_H_SetActivation,
			ECG_H_Handler,"Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID1 */
	{"PULSE_ACCLE12", true,
			DEF_PULSE_SAMPLE_RES,
			DEF_PULSE_SAMPLE_RATE,
			PULSE_Init,
			PULSE_Reset,
			PULSE_SetActivation,
			PULSE_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID2 */
	{"IMU_ACC_3AXES", true,
			DEF_ACC3X_SAMPLE_RES,
			DEF_ACC3X_SAMPLE_RATE,
			ACC3X_Init,
			ACC3X_Reset,
			ACC3X_SetActivation,
			ACC3X_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID3 */
	{"ANALOGIN_EMG", true,
			DEF_AN_EMG_SAMPLE_RES,
			DEF_AN_EMG_SAMPLE_RATE,
			AN_EMG_Init,
			AN_EMG_Reset,
			AN_EMG_SetActivation,
			AN_EMG_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID4 */
	{"ANALOGIN_SCL", true,
			DEF_AN_SCL_SAMPLE_RES,
			DEF_AN_SCL_SAMPLE_RATE,
			AN_SCL_Init,
			AN_SCL_Reset,
			AN_SCL_SetActivation,
			AN_SCL_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID5 */
	{"ANALOGIN_RESP", true,
			DEF_AN_RESP_SAMPLE_RES,
			DEF_AN_RESP_SAMPLE_RATE,
			AN_RESP_Init,
			AN_RESP_Reset,
			AN_RESP_SetActivation,
			AN_RESP_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID6 */
	{"ANALOGIN_AUDIO", true,
			DEF_AN_AUDIO_SAMPLE_RES,
			DEF_AN_AUDIO_SAMPLE_RATE,
			AN_AUDIO_Init,
			AN_AUDIO_Reset,
			AN_AUDIO_SetActivation,
			AN_AUDIO_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID7 */
	{"ANALOGIN_PACE", true,
			DEF_AN_PACE_SAMPLE_RES,
			DEF_AN_PACE_SAMPLE_RATE,
			AN_PACE_Init,
			AN_PACE_Reset,
			AN_PACE_SetActivation,
			AN_PACE_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID8 */
	{"ECG_CH8_VRTCL", true,
			DEF_ECG_V_SAMPLE_RES,
			DEF_ECG_V_SAMPLE_RATE,
			ECG_V_Init,
			ECG_V_Reset,
			ECG_V_SetActivation,
			ECG_V_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID9 */
	{"TEMP_SKIN", true,
			DEF_TEMP_SKIN_SAMPLE_RES,
			DEF_TEMP_SKIN_SAMPLE_RATE,
			TEMP_SKIN_Init,
			TEMP_SKIN_Reset,
			TEMP_SKIN_SetActivation,
			TEMP_SKIN_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID10 */
	{"GYRO_3D", true,
			DEF_GYR3X_SAMPLE_RES,
			DEF_GYR3X_SAMPLE_RATE,
			GYR3X_Init,
			GYR3X_Reset,
			GYR3X_SetActivation,
			GYR3X_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

	/* ID11 */
	{"BIOZ", true,
			DEF_BIOZ_SAMPLE_RES,
			DEF_BIOZ_SAMPLE_RATE,
			BIOZ_Init,
			BIOZ_Reset,
			BIOZ_SetActivation,
			BIOZ_Handler, "Ag_PCB", "mV",-1650, 1650, 0, 0, "" },

};
