/**
* @file  record.c
* @brief  precor desctiptor
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>



#include "A5_proto/parcel.h"
#include "A5_proto/record.h"

#include "RpcFifo.h"
#include "endianutils.h"
#include "_mem.h" //malloc


typedef struct sShmem
{
	uint8_t * buf;
	fifo_t col;
}shmem_t;

/* record descriptor structure */
typedef struct sRecord_desc
{
	uint8_t id;        // record id
	uint16_t max_len;  // record maximum length
}sRecord_desc_t;

/* record descriptor structure */
typedef struct sRecord_mem
{
	shmem_t rec_mem;
	sRecord_desc_t desc;
}sRecord_mem_t;

static const sRecord_desc_t registered_rt[]=
{    /* record id */                       /* maximum record length */
	{RESPONSE_TO_CMD_RECORD_TYPE,        MAX_RESPONCE_TO_CMD_RECORD_SIZE},
	{EDF_SYNC_CONFIG_RECORD_TYPE,        0},
	{EDF_SYNC_HISTORY_CONFIG_RECORD_TYPE,0},
	{EDF_SYNC_RECORD_TYPE,               0}
};

static sRecord_mem_t rec_map[ MAX_RECORD_TYPE ];
static uint8_t g_init_done=0;

/**
 * @brief RECORD_init
 * @brief This function uses for  allocation of shared memory areas
 * per record type
 * @returns 0 ok
 */
int RECORD_init(void)
{
	int i=0;

	/* single ton protection */
	if(g_init_done==0)
	{
		/* fill only zeros in the record map */
		memset(&rec_map, 0 ,sizeof(rec_map));

		/* for each element in the record map allocate shared fifo element */
		for(i=0;i<sizeof(registered_rt)/sizeof(registered_rt[0]);i++)
		{
			int8_t tmp_id= registered_rt[i].id;

			if(registered_rt[i].max_len)
			{
				/* check if the record was previously initialized */
				if(rec_map[tmp_id].desc.id == ZER_RECORD_TYPE)
				{
					uint16_t size = registered_rt[i].max_len;// * sizeof(uint32_t);
					/* 1. copy record descriptor */
					rec_map[tmp_id].desc = registered_rt[i];

					/* 2. fifo storage allocation */
					rec_map[tmp_id].rec_mem.buf = _MEM_alloc(size);
					if(rec_map[tmp_id].rec_mem.buf == NULL)
					{
						/* Memory alloc error */
						return -1;
					}

					/* 3. fifo per valid record type initialization */
					fifo_init(&rec_map[tmp_id].rec_mem.col, rec_map[tmp_id].rec_mem.buf,size);///sizeof(uint32_t));
				}
				else
				{
					/* configuration error */
					return -1;
				}
			}
		}

		/* lock-init */
		g_init_done=1;
	}

	return 0;
}/* end of RECORD_init */


/**
 * @brief RECORD_set_rec_val_map_8
 * @brief This function inserts record in the shared memory area
 * @param tp - record type
 * @param from - record data
 * @param len - record data length
 * @returns 0 ok
 */
int RECORD_set_rec_val_map_8(rec_t tp,uint8_t  in)
{
	/* arguments validation */
	if((tp == ZER_RECORD_TYPE) || (tp >=MAX_RECORD_TYPE ))
	{
		/* bad argument error */
		return -1;
	}

	/* 1. insert end of record */
	if(fifo_put8(&rec_map[tp].rec_mem.col, in))
	{
		/* overflow error */
		return -1;
	}

	return 0;
}/* End of RECORD_set_rec */

/**
 * @brief RECORD_set_rec_col_map
 * @brief This function inserts record in the shared memory area
 * @param tp - record type
 * @param from - record data
 * @param len - record data length
 * @returns 0 ok
 */
int RECORD_set_rec_col_map_8(rec_t tp,uint8_t * buf_in,uint32_t len)
{
	int i=0;

	/* arguments validation error */
	if(buf_in == NULL || len == 0)
	{
		/* bad argument error */
		return -1;
	}

	/* arguments validation */
	if((tp == ZER_RECORD_TYPE) || (tp >=MAX_RECORD_TYPE ))
	{
		/* bad argument error */
		return -1;
	}

	/* 1. insert end of record */
	if(fifo_put8(&rec_map[tp].rec_mem.col, tp))
	{
		/* overflow error */
		return -1;
	}

	/* 2. insert data to the relevant fifo */
	for(i=0;i<len;i++)
	{
		/* insert zone id */
		if(fifo_put8(&rec_map[tp].rec_mem.col, buf_in[i]))
		{
			/* overflow error */
			return -1;
		}
	}

	/* 3. insert end of record */
	if(fifo_put8(&rec_map[tp].rec_mem.col, EOR))
	{
		/* overflow error */
		return -1;
	}

	return 0;
}/* End of RECORD_set_rec */

/**
 * @brief RECORD_get_rec_col_map
 * @brief This function builds all records in the buffer
 * @brief according to the record types
 * @param to - buffer to store the record
 * @param len - buffer length
 * @returns numer of elements in the buff
 */
int RECORD_get_rec_col_map(uint8_t *buf_out,uint32_t len)
{
	int j=0,i=0,k=0;

	/* arguments validation error */
	if(buf_out == NULL || len == 0)
	{
		return -1;
	}

	/* for each registered record type get its payload  */
	for(i=0;i<sizeof(registered_rt)/sizeof(registered_rt[0]);i++)
	{
		sRecord_mem_t *tmp_map_el = &rec_map[registered_rt[i].id];

		/* check if there is data to be fetched */
		if(fifo_empty(&tmp_map_el->rec_mem.col))
		{
			continue;
		}
		/* check buffer size */
		if(k>=len)
		{
			/* buffer length error */
			return -1;
		}

		/* pull data up to max len */
		for(j=0;j<len;j++,k++)
		{
			/* insert zone id */
			if(fifo_get8(&tmp_map_el->rec_mem.col,&buf_out[k]))
			{
				break;
			}
		}
	}

	return k;
}/* End of RECORD_get_rec_col_map */

/**
 * @brief RECORD_get_rec_col
 * @brief This function builds specific record in the buffer
 * @param to - buffer to store the record
 * @param len - buffer length
 * @returns numer of elements in the buff
 */
int RECORD_get_rec_col(rec_t tp,uint8_t * buf_out,uint32_t len)
{
	sRecord_mem_t *tmp_map_el=NULL;
	int j=0,k=0;

	/* arguments validation error */
	if(buf_out == NULL || len == 0)
	{
		return -1;
	}


	tmp_map_el = &rec_map[tp];

	/* check if there is data to be fetched */
	if(fifo_empty(&tmp_map_el->rec_mem.col))
	{
		return k;
	}
	/* check buffer size */
	if(k>len)
	{
		/* buffer length error */
		return -1;
	}

	/* insert record data */
	for(j=0;j<len;j++,k++)
	{
		/* insert zone id */
		if(fifo_get8(&tmp_map_el->rec_mem.col,&buf_out[k]))
		{
			break;
		}
	}

	return k;
}/* End of RECORD_get_rec_col */









