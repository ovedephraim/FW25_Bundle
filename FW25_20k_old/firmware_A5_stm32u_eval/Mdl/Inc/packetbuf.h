/**
* @file packetbuf.h
* @brief packet buffers system.
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 30.05.2022
*
*/

#ifndef _PACKETBUF_H
#define _PACKETBUF_H

#include <stddef.h>

#include "../Inc/buff.h"
#include "../Inc/membuf.h"
#ifdef USE_TIMESTAMPS
#include "timebase.h"
#endif

#define FIRST_PACKET_SEGMENT		0x01
#define LAST_PACKET_SEGMENT			0x02
#define TX_ECHO_PACKET				0x04

#define UNDEFINED_FORMAT			0

struct sPacketBufHdr
{
	struct sBuffHdr h; 	/**< message buffer header */
	#ifdef USE_TIMESTAMPS
	TIMESTAMP startTimestamp; /**< packet first character time stamp */
	TIMESTAMP endTimestamp;	/**< packet last character time stamp */
	#endif
	unsigned long rsvd;	/**< reserved for message level information */
	unsigned char flags;	/**< reserved for message level information */
	unsigned char type;	/**< packet type */
	unsigned short format; /**< packet format identifier */
	size_t doffset;	/**< data chunk offset */
	size_t dlen;	/**< data chunk length */
	size_t offset;	/**< data offset in buffer */

};

typedef struct sPacketBufHdr PACKETBUF_HDR;

#define PACKETBUF_DATA(ptr) (unsigned char *)(&((PACKETBUF_HDR *)ptr)[1])
#define PACKETBUF_OFFSET_DATA(ptr,offset) &((PACKETBUF_DATA(ptr))[offset])
#define PACKETBUF_HDR_FROM_DATA(ptr) (PACKETBUF_HDR *)(&((PACKETBUF_HDR *)ptr)[-1])

#ifdef __cplusplus
extern "C" {
#endif

PACKETBUF_HDR *getPacketBuffer(void *memPool, unsigned char flags, unsigned char type, unsigned short format, unsigned short doffset);
PACKETBUF_HDR *getPacketBufferWithWait(void *memPool, unsigned char flags, unsigned char type, unsigned short format, unsigned short doffset, unsigned int timeout);

#ifdef __cplusplus
}
#endif

#endif


