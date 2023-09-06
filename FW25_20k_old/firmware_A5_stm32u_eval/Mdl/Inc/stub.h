/**
* @file stub.h
* @brief stub functionality
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 05.11.2015
*/

#ifndef _STUB_H
#define _STUB_H


#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


#define CMD_NOP			0
#define CMD_START_LOG	1
#define CMD_STOP_LOG	2
#define CMD_GET_GAIN	3
#define CMD_SET_GAIN	4

#define REC_LIMIT_NONE		(0)
#define REC_LIMIT_IN_SAMPLES	(1)
#define REC_LIMIT_IN_MS			(2)

#ifdef KUKU
struct sSamplerStartLogCmd
{
	uint16_t pad;
	uint16_t autostop_sel;
	uint32_t count_time;
};

struct sSamplerStartLogCmdResp
{
	uint16_t errCode;
};

struct sSamplerStopLogCmdResp
{
	uint16_t errCode;
	uint16_t autostop_sel;
	uint32_t count_time;
	uint32_t cycles_count;
	uint32_t elapsed_time;
};

struct sSamplerGetGainCmd
{
	uint8_t ch_id;
};

struct sSamplerSetGainCmd
{
	uint8_t ch_id;
	uint8_t gain;
};

struct sSamplerGetGainCmdResp
{
	uint16_t errCode;
	uint8_t gain;
};
#endif

#define MT_PACKET_FROM_SAMPLER		0x61
#define IS_MT_PACKET_FROM_SAMPLER(x) ((x)==(MT_PACKET_FROM_SAMPLER))
#define MSGHDR_SAMPLER MAKE_MSG_HDRTYPE(0,MSG_SRC_SAMPLER,MSG_TYPE_PACKET)



#ifdef __cplusplus
extern "C" {
#endif

void initI2C0DevParams(void);


#ifdef KUKU
int startAdcStubTask(QueueHandle_t *adcQ, TaskHandle_t *adcT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio);
void adcStubTask(void *para);
int sendToSampler(QueueHandle_t *q, void *p, uint16_t data, uint16_t hdr, uint32_t timeout);
#endif

#ifdef __cplusplus
}
#endif


#endif




