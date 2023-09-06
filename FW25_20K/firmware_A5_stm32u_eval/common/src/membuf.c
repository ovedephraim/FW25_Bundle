/**
* @file membuf.c
* @brief Equal size memory buffers system
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 30.04.2022
*/

#include <stddef.h>
#include <assert.h>
#include "membuf.h"
#include "stm32u5xx_hal.h"

/**
 * @fn int initMemBufPool(MEMBUF_POOL *pool, void *memArea, size_t memSize, size_t bufSize, size_t bufNum)
 * A memory pool initialization function
 *
 * @author Anton Kanaev
 * @param pool Pointer to memory pool
 * @param memArea Pointer to buffers memory area
 * @param memSize Buffers memory area
 * @param bufSize Buffer size
 * @param bufNum Number of buffers
 * @return
 * @date 30.04.2022
 */
int initMemBufPool(MEMBUF_POOL *pool, void *memArea, size_t memSize, size_t bufSize, size_t bufNum)
{
	struct sMemBufHeader *buf;
	
	__disable_irq();
	pool->freeList=NULL;
	pool->bufSize=bufSize;
	pool->freeBuf=0;
	pool->nBuf=0;
	if (bufSize==0 || memSize<bufSize)
	{
		__enable_irq();
		return -1;
	}
	pool->freeList= (struct sMemBufHeader *)memArea;
	if (memArea!=NULL)
	{
		buf=(struct sMemBufHeader *)memArea;
		pool->nBuf++;
		
		memSize-=(bufSize+MEM_BUF_HEADER_SIZE);
		while ((bufSize+MEM_BUF_HEADER_SIZE)<=memSize)
		{
			buf->link=((char *)buf)+(bufSize+MEM_BUF_HEADER_SIZE);
			pool->nBuf++;
			memSize-=(bufSize+MEM_BUF_HEADER_SIZE);
			buf=(struct sMemBufHeader *)(buf->link);
		}
		buf->link=NULL;
		pool->freeBuf=pool->nBuf;
	}
	__enable_irq();
	return 0;
}

/**
 * @fn void *getMemBuf(MEMBUF_POOL *pool)
 * A memory pool initialization function
 *
 * @author Anton Kanaev
 * @param pool Pointer to memory pool
 * @return Pointer to allocated buffer. NULL=failed to allocate buffer
 * @date 30.04.2022
 */
void *getMemBuf(MEMBUF_POOL *pool)
{
	register struct sMemBufHeader *buf;
	
	__disable_irq();
	
	if (pool->freeList)
	{
		buf=pool->freeList;
		pool->freeList=(struct sMemBufHeader *)buf->link;
		#ifdef MEMBUF_GET_CHECK_BOUNDS
		assert((uint32_t)pool->freeList!=0xa5a5a5a5);
		#endif
		pool->freeBuf--;
		__enable_irq();
		buf->link=pool;
		return buf+1;//buf+1;
	}
	else 
	{
		__enable_irq();
		return NULL;
	}
}

/**
 * @fn int retMemBuf(void *buf)
 * A memory pool initialization function
 *
 * @author Anton Kanaev
 * @param buf Pointer to previously allocated buffer.
 * @return 0 if success. Otherwise failure to return the buffer.
 * @date 30.04.2022
 */
int retMemBuf(void *buf)
{
	register struct sMemBufHeader *b;
	register MEMBUF_POOL *pool;

	if (buf==NULL)
		return	0;
	b=((struct sMemBufHeader *)buf)-1;//buf)-1;
	if (b->link==NULL)
		return -1;
	
	pool=(MEMBUF_POOL *)b->link;
	__disable_irq();
	b->link=pool->freeList;
	pool->freeList=b;
	#ifdef MEMBUF_RET_CHECK_BOUNDS
	assert((uint32_t)pool->freeList!=0xa5a5a5a5);
	#endif
	pool->freeBuf++;
	__enable_irq();
	return 0;
}


