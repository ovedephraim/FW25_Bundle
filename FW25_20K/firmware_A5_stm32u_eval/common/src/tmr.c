/*
 * tmr.c
 *
 *  Created on: Oct 6, 2015
 *      Author: Anton Kanaev
 */


#include <stddef.h>
#include <stdint.h>
#include "tmr.h"


static uint8_t inUse=0;

static const TIMER_TypeDef *tmrTbl[4]={TIMER0, TIMER1, TIMER2, TIMER3};

TIMER_TypeDef *allocateTimer(TIMER_TypeDef *timer)
{
	register uint32_t i;
	register uint32_t key;

	if (timer==NULL)
	{
		for (i=0;i<4;i++)
		{
			key=__disableInterrupts();
			if (inUse & (1<<i))
				__restoreInterrupts(key);
			else
			{
				inUse|=(1<<i);
				__restoreInterrupts(key);
				return (TIMER_TypeDef *)tmrTbl[i];
			}

		}
		return NULL;
	}
	else
	{
		for (i=0;i<4;i++)
		{
			if (timer==tmrTbl[i])
			{
				key=__disableInterrupts();
				if (inUse & (1<<i))
				{
					__restoreInterrupts(key);
					return NULL;
				}
				else
				{
					__restoreInterrupts(key);
					return timer;
				}
			}
		}
		return NULL;
	}
}

void freeTimer(TIMER_TypeDef *timer)
{
	register uint32_t i;
	register uint32_t key;

	for (i=0;i<4;i++)
	{
		if (timer==tmrTbl[i])
		{
			key=__disableInterrupts();
			inUse &= ~(1<<i);
			__restoreInterrupts(key);
			return;
		}
	}
}
