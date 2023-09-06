/**
* @file rxframer.h
* @brief serial reception framer
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 04.05.2022
*/

#ifndef _RXFRAMER_H
#define _RXFRAMER_H


#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../Inc/packetbuf.h"
#ifdef USE_TIMESTAMPS
#include "timebase.h"
#endif

#ifndef MAX_FRAMER_USER_STAT_COUNT
#define MAX_FRAMER_USER_STAT_COUNT 0
#endif

#define RX_FRAMER_INITIAL_STATE 0

#ifdef __cplusplus
extern "C" {
#endif



struct sRxFramerStat
{
	uint32_t no_buffers;
	uint32_t timeout;
	uint32_t err_length;
	uint32_t err_frame;
	#ifdef MAX_FRAMER_USER_STAT_COUNT
	#if 0<MAX_FRAMER_USER_STAT_COUNT
	uint32_t count[MAX_FRAMER_USER_STAT_COUNT];
	#endif
	#endif
};

struct sExtraPoolBufConfig
{
	size_t bsize;
	size_t nbuf;

};

struct sRxFramer
{
	uint16_t state;
	uint16_t prevState;
	MEMBUF_POOL *pool;
	size_t pools;	/** Nomber of extra pools */
	uint32_t bufSize; /**< Length of buffer data area  */
	uint32_t payloadLength; /**< Number of bytes to receive in payload reception states of length oriented protocols  */
	uint32_t payloadReceived; /**< Number of received payload bytes  */
	uint32_t idx;				/**< Index of current character  */
	char prevChar;
	unsigned char type;	/**< packet type */
	PACKETBUF_HDR *buf;
	PACKETBUF_HDR *lastPutInBuf;
	uint32_t lastPutInBufIdx;
	#ifdef USE_TIMESTAMPS
	PACKETBUF_HDR *(*frx)(struct sRxFramer *framer, uint8_t ch, void *arg, TIMESTAMP rxTS);
	int (*ftimeout)(struct sRxFramer *p);
	PACKETBUF_HDR *(*fidle)(struct sRxFramer *framer, void *arg, TIMESTAMP rxTS);
	void (*rxFrameSync)(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS);
	void (*rxEofFrameSync)(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS);
	#else
	PACKETBUF_HDR * (*frx)(struct sRxFramer *framer, uint8_t ch, void *arg);
	int (*ftimeout)(struct sRxFramer *p);
	PACKETBUF_HDR *(*fidle)(struct sRxFramer *framer, void *arg);
	void (*rxFrameSync)(struct sRxFramer *p, char rxChar);
	void (*rxEofFrameSync)(struct sRxFramer *p, char rxChar);
	#endif

	void (*startTimer)(uint16_t, uint16_t);
	void (*stopTimer)(uint16_t);
	void (*onTimer)(uint16_t);
	uint32_t timeout;
	struct sRxFramerStat stat;
};

struct sRxFramerParams
{
	MEMBUF_POOL *pool;
	size_t pools;	/* number of extra pools */
	struct sExtraPoolBufConfig *extraPoolCfg;
	#ifdef USE_TIMESTAMPS
	PACKETBUF_HDR *(*frx)(struct sRxFramer *framer, uint8_t ch, void *arg, TIMESTAMP rxTS);
	int (*ftimeout)(struct sRxFramer *p);
	int (*idle)(struct sRxFramer *p, TIMESTAMP rxTS);
	void (*rxFrameSync)(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS);
	void (*rxEofFrameSync)(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS);
	#else
	PACKETBUF_HDR *(*frx)(struct sRxFramer *framer, uint8_t ch, void *arg);
	int (*ftimeout)(struct sRxFramer *p);
	int (*idle)(struct sRxFramer *p);
	void (*rxFrameSync)(struct sRxFramer *p, char rxChar);
	void (*rxEofFrameSync)(struct sRxFramer *p, char rxChar);
	#endif

	void (*startTimer)(uint16_t, uint16_t);
	void (*stopTimer)(uint16_t);
	void (*onTimer)(uint16_t);
	uint32_t timeout;

};


int initRxFramer(struct sRxFramer *p, struct sRxFramerParams *param);
int handleRxFramerTimeout(struct sRxFramer *p);
void rxFramerResync(struct sRxFramer *p);
void rxEofFramerResync(struct sRxFramer *p);

#ifdef USE_TIMESTAMPS
void rxFramerPutInBuffer(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS);
#else
void rxFramerPutInBuffer(struct sRxFramer *p, char rxChar);
#endif

int rxFramerFitPacketBuffer(struct sRxFramer *p, size_t minPacketDataSize);

#ifdef __cplusplus
}
#endif


#endif




