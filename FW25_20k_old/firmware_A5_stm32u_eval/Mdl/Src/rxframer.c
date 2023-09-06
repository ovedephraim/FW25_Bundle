/**
*  @file rxframer.c
*  @brief serial reception framer
 * @author Anton Kanaev
 *
 * @version 0.0.1
 * @date 20.10.2015
 *
 * @section License
 * <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/


#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "membuf.h"
#include "packetbuf.h"
#include "rxframer.h"

int initRxFramer(struct sRxFramer *p, struct sRxFramerParams *param)
{
	memset(p, 0, sizeof(struct sRxFramer));
	p->prevState=p->state=RX_FRAMER_INITIAL_STATE;
	p->pool=param->pool;
	p->pools=param->pools;
	if (p->pool)
		p->bufSize=(sizeof(PACKETBUF_HDR)<=p->pool->bufSize) ? p->pool->bufSize-sizeof(PACKETBUF_HDR) : 0;
	p->frx=param->frx;
	p->ftimeout=param->ftimeout;
	p->rxFrameSync=param->rxFrameSync;
	p->rxEofFrameSync=param->rxEofFrameSync;
	p->startTimer=param->startTimer;
	p->stopTimer=param->stopTimer;
	p->onTimer=param->onTimer;
	p->timeout=param->timeout;
	return 0;
}


int handleRxFramerTimeout(struct sRxFramer *p)
{
	if (p->state==RX_FRAMER_INITIAL_STATE)
		return 0;
	p->stat.timeout++;
	rxFramerResync(p);
	#ifdef USE_TIMESTAMPS
	if (p->buf)
	{
		memset(&p->buf->startTimestamp, 0, sizeof(TIMESTAMP));
		memset(&p->buf->endTimestamp, 0, sizeof(TIMESTAMP));
	}
	#endif
	return 1;
}


#ifdef USE_TIMESTAMPS
void rxFramerPutInBuffer(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS)
{
	p->lastPutInBuf=p->buf;
	p->lastPutInBufIdx=p->idx;
	if ((p->idx+1)<p->bufSize)	// leave one space in the end for terminating '\0'
	{
		p->buf->endTimestamp=rxTS;
		(PACKETBUF_DATA(p->buf))[p->idx]=rxChar;
		p->idx++;
		p->buf->dlen++;
	}
}

void rxFramerDefFrameSync(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS)
{
}


#else
void rxFramerPutInBuffer(struct sRxFramer *p, char rxChar)
{
	p->lastPutInBuf = p->buf;
	p->lastPutInBufIdx = p->idx;
	if ((p->idx) < p->bufSize)	// leave one space in the end for terminating '\0'
	{
		(PACKETBUF_DATA(p->buf))[p->idx] = rxChar;
		p->idx++;
		p->buf->dlen++;
	}
}

void rxFramerDefFrameSync(struct sRxFramer *p, char rxChar)
{
}

#endif

void rxFramerResync(struct sRxFramer *p)
{
	p->idx=0;
	if (p->buf)
		p->buf->dlen=0;
	p->payloadLength=p->payloadReceived=0;
	p->prevState=p->state;
	p->state=RX_FRAMER_INITIAL_STATE;
}

void rxEofFramerResync(struct sRxFramer *p)
{
	p->idx=0;
	p->payloadLength=p->payloadReceived=0;
	p->prevState=p->state;
	p->state=RX_FRAMER_INITIAL_STATE;
}

int rxFramerFitPacketBuffer(struct sRxFramer *p, size_t minPacketDataSize)
{
	PACKETBUF_HDR *buf=NULL;
	size_t bsize;
	size_t idx;

	if (p==NULL)
		return -1;
	if (p->pool==NULL || p->buf==NULL)
		return -2;
	if 	(minPacketDataSize<=p->bufSize)
		return 0;


	/* expected data is larger than buffer size, find a larger buffer */
	for (idx=0;idx<=p->pools;idx++)
	{
		if ((sizeof(PACKETBUF_HDR)+minPacketDataSize)<=p->pool[idx].bufSize)
		{
			if (NULL==(buf=getPacketBuffer(&p->pool[idx],FIRST_PACKET_SEGMENT|LAST_PACKET_SEGMENT, p->type, UNDEFINED_FORMAT, 0)))
				bsize=0;
			else
			{
				bsize=p->pool[idx].bufSize-sizeof(PACKETBUF_HDR);

				/* move to new buffer */
				buf->h.cbFunc=p->buf->h.cbFunc;
				buf->h.cbArg=p->buf->h.cbArg;
				#ifdef USE_TIMESTAMPS
				buf->startTimestamp=p->startTimestamp;
				buf->endTimestamp=p->endTimestamp;
				#endif
				buf->rsvd=p->buf->rsvd;
				buf->flags=p->buf->flags;
				buf->type=p->buf->type;
				buf->format=p->buf->format;
				buf->doffset=p->buf->doffset;
				buf->dlen=p->buf->dlen;
				buf->offset=p->buf->offset;
				memcpy(PACKETBUF_DATA(buf), PACKETBUF_DATA(p->buf), p->bufSize);
				retMemBuf(p->buf);
				p->buf=buf;
				p->bufSize=bsize;
				return 0;
			}
		}
	}
	return -4;
}

