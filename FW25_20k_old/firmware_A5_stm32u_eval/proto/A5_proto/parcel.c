/**
* @file  parsel.c
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

#include "main.h"
#include "rtc.h"
#include "A5_proto/parcel.h"
#include "A5_proto/record.h"

#include "packetbuf.h"
#include "auxcmd.h"
#include "endianutils.h"


uint32_t g_parcel_id=0;

#if 0
PACKETBUF_HDR *makeParcelFrame(MEMBUF_POOL *pool,void *extraData, size_t extraDataLen, uint32_t timeout)
{
	PACKETBUF_HDR *buf=NULL;
	PARCEL_HDR *pf=NULL;

	if (pool==NULL)
		return NULL;
	if (extraDataLen > MAX_DATA_LEN)
	    return NULL;

	buf=getPacketBufferWithWait(pool,0,MT_PACKET_FROM_AUX_CMD, A5_PROTO_FORMAT, 0, timeout);
	if (buf==NULL)
		return NULL;

	pf=(PARCEL_HDR *)PACKETBUF_DATA(buf);

	pf->sop = SOP & 0xff;
	pf->ts = 0; //todo rtc
	pf->dlen=extraDataLen;

	if (extraData)
	{
		memmove(&pf[1],extraData,extraDataLen);
		((uint8_t *)&pf[1])[extraDataLen]=EOP & 0xff;
	}
	else
	{
		memset(&pf[1],0,extraDataLen);
		((uint8_t *)&pf[1])[extraDataLen]=EOP & 0xff;
	}

	buf->dlen=extraDataLen+(1/*eop*/+sizeof(PARCEL_HDR));

	return (PACKETBUF_HDR *)buf;
}/* End of makeParselFrame */
#else
PACKETBUF_HDR *makeParcelFrame(MEMBUF_POOL *pool, uint32_t timeout, unsigned char * exd,uint32_t exdlen)
{
	PACKETBUF_HDR *buf=NULL;
	PARCEL_HDR *pf=NULL;
	void *extraData=NULL;
	uint16_t extraDataLen=exdlen;

	if (pool==NULL)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		return NULL;
	}

	/* get buffer from the provided pool */
	buf=getPacketBufferWithWait(pool,0,MT_PACKET_FROM_AUX_CMD, A5_PROTO_FORMAT, 0, timeout);
	if (buf==NULL)
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
		return NULL;
	}

	pf=(PARCEL_HDR *)PACKETBUF_DATA(buf);
	extraData=&pf[1];

#if(0)
	/* fill the record part of the parcel on the buffer */
	extraDataLen=RECORD_get_rec_col_map(extraData,MAX_DATA_LEN);
	if (extraDataLen <= 0){
		retMemBuf(buf);
	    return NULL;
	}
#endif

	if(exd)
	{
		memcpy(extraData,exd,extraDataLen);
	}

	pf->sop=SOP&0xff;
	pf->crc=0xffff;

	pf->parcel_id=g_parcel_id++;
	pf->parcel_ts=RTC_GetTimestampMillis();



#if defined(USE_BIG_ENDIAN)
	pf->dlen=shortLE2BE(extraDataLen);
#else
	pf->dlen=extraDataLen;
#endif

	buf->dlen=extraDataLen+(1/*eop*/+sizeof(PARCEL_HDR));
	((uint8_t *)&pf[1])[extraDataLen]=EOP & 0xff;

	return (PACKETBUF_HDR *)buf;
}/* End of makeParselFrame */

#endif



