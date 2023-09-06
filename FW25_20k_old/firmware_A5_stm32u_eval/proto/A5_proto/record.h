/**
* @file  record.h
* @brief A5 protocol record codes
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 26.05.2022
*
*/

#ifndef _RECORD_H
#define _RECORD_H

#include <stdint.h>

/* For record standard ASCII character set, bytes 0-127 are used */
#define RESPONSE_TO_CMD_RECORD_TYPE         'R'
#define EDF_SYNC_CONFIG_RECORD_TYPE         'H'
#define EDF_SYNC_HISTORY_CONFIG_RECORD_TYPE 'h'
#define EDF_SYNC_RECORD_TYPE                'S'
#define STREAMING_RECORD_TYPE               'B'

#define MAX_RECORD_TYPE  0x7f
#define ZER_RECORD_TYPE  0x00

#define EOR 0xF8
/* maximum length of record data */
#define MAX_RESPONCE_TO_CMD_RECORD_SIZE 2048
/* maximum length of record data */
#define MAX_STREAM_TO_CMD_RECORD_SIZE 2048

typedef uint8_t rec_t;

///* Response (R) header structure: */
struct sRespRecHeader
{
	uint8_t sor; //record type
}__attribute__((packed));
typedef struct sCmdRespRecHeader sCmdRespRecHeader_t;

///* Response (B) header structure: */
struct sStreamRecHeader
{
	uint8_t sor;     //record type
	uint8_t chan_id; //channel id
	uint16_t len;    //vector length
	uint32_t ts;     //time stamp
}__attribute__((packed));
typedef struct sStreamRecHeader sStreamRecHeader_t;

int RECORD_init(void);
int RECORD_set_rec_col_map_8(rec_t tp, uint8_t *buf_in,uint32_t len);
int RECORD_get_rec_col_map(uint8_t *buf_out,uint32_t len);
int RECORD_set_rec_val_map_8(rec_t tp,uint8_t  in);
int RECORD_get_rec_col(rec_t tp,uint8_t * buf_out,uint32_t len);


#endif /* _RECORD_H */
