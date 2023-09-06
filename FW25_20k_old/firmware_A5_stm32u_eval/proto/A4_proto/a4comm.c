/**
* @file  a4comm.c
* @brief A-4 protocol #1 Handler
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 20.10.2015
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include <A4_proto/a4comm.h>
#include <A4_proto/a4frame.h>
#include <stddef.h>
#include <stdint.h>
#include <ctype.h>
#include "rxframer.h"
#include "main.h"


#ifdef USE_TIMESTAMPS
PACKETBUF_HDR *a4FrameRx(struct sRxFramer *framer, uint8_t ch, void *arg, TIMESTAMP rxTS)
#else
PACKETBUF_HDR *a4FrameRx(struct sRxFramer *framer, uint8_t ch, void *arg)
#endif
{
	volatile PACKETBUF_HDR *frame=NULL;
	if (framer==NULL)
		return NULL;
	if (framer->buf==NULL && framer->state==RX_FRAMER_INITIAL_STATE)
	{
		// Expecting first sync byte, so still can allocate a fresh buffer
		framer->buf=getPacketBuffer(framer->pool,FIRST_PACKET_SEGMENT|LAST_PACKET_SEGMENT, framer->type, UNDEFINED_FORMAT, 0);

		if (framer->buf==NULL)
			framer->stat.no_buffers++;
	}

	if (framer->buf)
	{
		switch (framer->state)
		{
		case RX_FRAMER_INITIAL_STATE:

			#ifdef USE_TIMESTAMPS
			a4RxFrameSync(framer, ch, TIMESTAMP rxTS);
			#else
			a4RxFrameSync(framer, ch);
			#endif
			break;
		case RX_A4_PAYLOAD_STATE:
			#ifdef USE_TIMESTAMPS
			rxFramerPutInBuffer(framer,ch,rxTS);
			#else
			rxFramerPutInBuffer(framer,ch);
			#endif

			framer->payloadReceived++;
			if(framer->payloadReceived < RX_A4_MAX_PAYLOAD_LEN)
			{
				if(ch == A4_FRAME_EOF /*|| ch == ']'*/)
				{
					frame=framer->buf;
					frame->dlen=framer->idx;
					frame->format=A4_PROTO_FORMAT;

					framer->buf=NULL;
					rxFramerResync(framer);
				}
			}
			else
			{
				rxFramerResync(framer);
			}
			break;
		}
	}
	return (PACKETBUF_HDR *)frame;
}

#ifdef USE_TIMESTAMPS
void a4RxFrameSync(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS)
#else
void a4RxFrameSync(struct sRxFramer *p, char rxChar)
#endif
{
	if(rxChar <= A4_FRAME_SYNC1)
	{
		#ifdef USE_TIMESTAMPS
		p->buf->startTimestamp=rxTS;
		rxFramerPutInBuffer(p,rxChar,rxTS);
		#else
		rxFramerPutInBuffer(p,rxChar);
		#endif

		p->state = RX_A4_PAYLOAD_STATE;
	}
}

#ifdef USE_TIMESTAMPS
void a4RxEofFrameSync(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS)
#else
void a4RxEofFrameSync(struct sRxFramer *p, char rxChar)
#endif
{
	p->idx = 0;
	p->payloadLength = p->payloadReceived=0;
	p->prevState = p->state;

	#ifdef USE_TIMESTAMPS
	p->buf->startTimestamp=rxTS;
	rxFramerPutInBuffer(p,rxChar,rxTS);
	#else
	rxFramerPutInBuffer(p,rxChar);
	#endif
}

