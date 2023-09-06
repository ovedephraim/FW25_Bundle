/**
* @file  a4frame.c
* @brief  protocol frame support
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.1985
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "membuf.h"
#include "A4_proto/a4frame.h"
#include "packetbuf.h"
#include "auxcmd.h"
#include "endianutils.h"

#define A4_STREAM_EOF_B1 0x7f
#define A4_STREAM_EOF_B2 0xf7

PACKETBUF_HDR *makeA4StreamFrame(MEMBUF_POOL *pool,void *extraData, size_t extraDataLen, uint32_t timeout)
{
	PACKETBUF_HDR *buf=NULL;
	A4_STREAM_HEADER *phf=NULL;

	if (pool==NULL)
		return NULL;
	buf=getPacketBufferWithWait(pool,0,MT_PACKET_FROM_AUX_CMD, A4_PROTO_FORMAT, 0, timeout);
	if (buf==NULL)
		return NULL;

	phf=(A4_STREAM_HEADER *)PACKETBUF_DATA(buf);
	phf->sync1=A4_STREAM_SOF & 0xff;
	phf->sync2=(A4_STREAM_SOF>>8) & 0xff;

	if (extraData)
	{
		memmove(&phf[1],extraData,extraDataLen);
		((uint8_t *)&phf[1])[extraDataLen]=A4_STREAM_EOF & 0xff;
		((uint8_t *)&phf[1])[extraDataLen+1]=(A4_STREAM_EOF>>8) & 0xff;
	}
	else
	{
		memset(&phf[1],0,extraDataLen);
		((uint8_t *)&phf[1])[extraDataLen]=A4_STREAM_EOF & 0xff;
		((uint8_t *)&phf[1])[extraDataLen+1]=(A4_STREAM_EOF>>8) & 0xff;
	}
	buf->dlen=extraDataLen+(2+sizeof(A4_STREAM_HEADER));
	return (PACKETBUF_HDR *)buf;

}/* End of makeA4StreamFrame */


