

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <freertos.h>
#include <task.h>
#include <queue.h>

#include "main.h"
#include "bus.h"
#include "tmp117/TMP117.h"
#include "tmp117_dev.h"
#include "AS6221/as6221/AS6221.h"

extern TMP117_Object_t tmp117_obj1;
extern TMP117_Object_t tmp117_obj2;
extern TMP117_Object_t tmp117_obj3;

/**
 * @brief tmp117_hw_test
 * @brief hardware validation function
 * @return system error
 */
int tmp117_hw_test(void * arg)
{
	TMP117_IO_t  io_ctx;
	int16_t      id=0xFF;
	uint8_t dev_id = *(uint8_t*)arg;
	TMP117_Object_t *ptmp117=0;
	char dbug[100]={0};

	io_ctx.Init        = BUS_I2C4_Init;
	io_ctx.DeInit      = BUS_I2C4_DeInit;
	io_ctx.ReadReg     = BUS_I2C4_ReadReg;
	io_ctx.WriteReg    = BUS_I2C4_WriteReg;
	io_ctx.GetTick     = BUS_GetTick;

	switch(dev_id)
	{
		case(0):{

			/* Configure the tmp117 driver */
			io_ctx.Address     = TMP117_I2C_ADD1;
			ptmp117=&tmp117_obj1;

			if (TMP117_RegisterBusIO(ptmp117, &io_ctx) != TMP117_OK)
			{
			  return SYS_ERROR_BUS_FAILURE;
			}

			/* wait for bus will be ready before reading */
			if(BUS_I2C4_IsReady(TMP117_I2C_ADD1, 100)!= TMP117_OK)
			{
				sprintf(dbug,"[TEMP][0x%x] bus error! \r\n",
						(int)TMP117_I2C_ADD1);
				/* send to debug aux channel */
				aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
				return SYS_ERROR_BUS_FAILURE;
			}
			else
			{
				if(TMP117_readId(ptmp117, &id) == 0)
				{
					if(id == TMP117_WHOAMI)
					{
						sprintf(dbug,"[TMP117][%x] detected \r\n",(int)TMP117_I2C_ADD1);
					}
					else //AS6221_
					{
						sprintf(dbug,"[AS6221][%x] detected \r\n",(int)TMP117_I2C_ADD1);

					}
					/* send to debug aux channel */
					aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
				}
			}
		}break;

		case(1):{

			/* Configure the tmp117 driver */
			io_ctx.Address     = TMP117_I2C_ADD2;
			ptmp117=&tmp117_obj2;

			if (TMP117_RegisterBusIO(ptmp117, &io_ctx) != TMP117_OK)
			{
			  return SYS_ERROR_BUS_FAILURE;
			}

			/* wait for bus will be ready before reading */
			if(BUS_I2C4_IsReady(TMP117_I2C_ADD2, 1000)!= TMP117_OK)
			{
				sprintf(dbug,"[TEMP][0x%x] bus error! \r\n",
						(int)TMP117_I2C_ADD2);
				/* send to debug aux channel */
				aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
				return SYS_ERROR_BUS_FAILURE;
			}
			else
			{
				if(TMP117_readId(ptmp117, &id) == 0)
				{
					if(id == TMP117_WHOAMI)
					{
						sprintf(dbug,"[TMP117][%x] detected \r\n",(int)TMP117_I2C_ADD2);
					}
					else //AS6221_
					{
						sprintf(dbug,"[AS6221][%x] detected \r\n",(int)TMP117_I2C_ADD2);

					}
					/* send to debug aux channel */
					aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
				}
			}

		}break;

		case(2):{

			/* Configure the tmp117 driver */
			io_ctx.Address     = TMP117_I2C_ADD3;
			ptmp117=&tmp117_obj3;

			if (TMP117_RegisterBusIO(ptmp117, &io_ctx) != TMP117_OK)
			{
			  return SYS_ERROR_BUS_FAILURE;
			}

			/* wait for bus will be ready before reading */
			if(BUS_I2C4_IsReady(TMP117_I2C_ADD3, 1000)!= TMP117_OK)
			{
				sprintf(dbug,"[TEMP][0x%x] bus error! \r\n",
						(int)TMP117_I2C_ADD3);
				/* send to debug aux channel */
				aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
				return SYS_ERROR_BUS_FAILURE;
			}
			else
			{
				if(TMP117_readId(ptmp117, &id) == 0)
				{
					if(id == TMP117_WHOAMI)
					{
						sprintf(dbug,"[TMP117][%x] detected \r\n",(int)TMP117_I2C_ADD3);
					}
					else //AS6221_
					{
						sprintf(dbug,"[AS6221][%x] detected \r\n",(int)TMP117_I2C_ADD3);

					}
					/* send to debug aux channel */
					aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);
				}
			}
		}break;
	}

	return SYS_ERROR_NONE;
}/* End of tmp117_hw_test */

#ifdef __cplusplus
}
#endif
