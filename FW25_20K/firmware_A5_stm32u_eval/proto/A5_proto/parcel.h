/**
* @file  a4frame.h
* @briefA4 protocol frame support
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 30.04.2022
*/


#ifndef _A5FRAME_H
#define _A5FRAME_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "crc16.h"
#include "membuf.h"
#include "packetbuf.h"

#define A5_PROTO_FORMAT			1


#define SOP  '{'
#define EOP  '}'

#define MAX_DATA_LEN  100000
#define MAX_PARCEL_LEN  (sizeof(PARCEL_HDR)+MAX_DATA_LEN+1/*1 byte eop*/)

struct sParcelHeader
{
	uint8_t sop;   //start of parcel
	uint32_t parcel_id;
	uint16_t crc;   //time stamp
	uint16_t dlen; //data length
	uint32_t parcel_ts;
}__attribute__((packed));

typedef struct sParcelHeader PARCEL_HDR;

PACKETBUF_HDR *makeParcelFrame(MEMBUF_POOL *pool, uint32_t timeout, unsigned char * exd,uint32_t exdlen);


#endif /* _A5FRAME_H */
