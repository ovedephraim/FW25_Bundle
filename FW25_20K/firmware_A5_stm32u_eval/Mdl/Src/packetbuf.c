/**
* @file packetbuf.h
* @brief packet buffers system.
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
#include <freertos.h>
#include <task.h>
#include "packetbuf.h"
#include "membuf.h"

/**
* @fn PACKETBUF_HDR *getPacketBuffer(void *memPool, unsigned char flags, unsigned char type, unsigned char type)
*
* This function gets a packet buffer from a memory pool.
*
* @author Anton Kanaev
*
* @param memPool pointer to memory pool
*
* @return pointer to allocated buffer, NULL if no buffers
*
* @date 30.04.2022
*/
PACKETBUF_HDR *getPacketBuffer(void *memPool, unsigned char flags, unsigned char type, unsigned short format,unsigned short doffset)
{
	PACKETBUF_HDR *buff;

	buff=(PACKETBUF_HDR *)getMemBuf(memPool);	// get a reception buffer
	if (buff)
	{
		buff->h.link=NULL;
		buff->h.cbFunc=NULL;
		buff->h.cbArg=NULL;
		buff->rsvd=0;
		buff->flags=flags;
		buff->type=type;
		buff->format=format;
		buff->doffset=doffset;
		buff->dlen=0;
		buff->offset=0;
	}
	return buff;
}

/**
* @fn PACKETBUF_HDR *getPacketBufferWithWait(void *memPool, unsigned char flags, unsigned char type, unsigned short format, unsigned short doffset, unsigned int timeout)
*
* This function gets a packet buffer from a memory pool.
*
* @author Anton Kanaev
*
* @param memPool pointer to memory pool
*
* @return pointer to allocated buffer, NULL if no buffers
*
* @date 30.04.2022
*/
PACKETBUF_HDR *getPacketBufferWithWait(void *memPool, unsigned char flags, unsigned char type, unsigned short format, unsigned short doffset, unsigned int timeout)
{
	PACKETBUF_HDR *p=NULL;

	if (timeout==0)
		p=getPacketBuffer(memPool,flags, type, format, doffset);
	else
	{
		while ((p=getPacketBuffer(memPool,flags, type, format, doffset))==NULL)
		{
			vTaskDelay(1);
			if (timeout!=portMAX_DELAY)
			{
				timeout--;
				if (timeout==0)
					break;
			}
		}
	}

	return p;
}

