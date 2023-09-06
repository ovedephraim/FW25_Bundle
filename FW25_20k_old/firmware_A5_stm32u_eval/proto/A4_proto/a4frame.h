/**
* @file  a4frame.h
* @briefA4 protocol frame support
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 30.04.2022
*/


#ifndef _A4FRAME_H
#define _A4FRAME_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "crc16.h"
#include "membuf.h"
#include "packetbuf.h"

#define A4_PROTO_FORMAT			1

/*
 * For standard ASCII character set, bytes 0-127 are used
 */
#define A4_FRAME_SYNC1 				(127)
#define A4_FRAME_EOF 				(0x0A)

#define A4_STREAM_SOF 0x8008
#define A4_STREAM_EOF 0x7ff7


struct sA4StreamHeader
{
	uint8_t sync1;
	uint8_t sync2;
}__attribute__((packed));

typedef struct sA4StreamHeader A4_STREAM_HEADER;


static inline bool A4ValidHeader(void *header)
{
	return true;
}

static inline bool A4ValidPayload(void *payload, size_t length)
{
	return  true;
}

static inline bool A4ValidFrame(void *frame, size_t len)
{
	return true;
}

PACKETBUF_HDR *makeA4StreamFrame(MEMBUF_POOL *pool,void *extraData, size_t extraDataLen, uint32_t timeout);

#endif /* _A4FRAME_H */
