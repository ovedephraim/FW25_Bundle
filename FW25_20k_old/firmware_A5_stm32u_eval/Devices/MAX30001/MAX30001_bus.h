
#ifndef MAX30001_BUS_H_
#define MAX30001_BUS_H_

#include <MAX30001/max30001.h>
#include "main.h"
#include <stdio.h>
#include <stdbool.h>


typedef int32_t (*maxdev_write_ptr)(void *, uint8_t *, uint32_t , uint8_t *, uint32_t );
typedef int32_t (*maxdev_read_ptr) (void *, uint8_t *, uint32_t , uint8_t *, uint32_t );

typedef struct {
  /** Component mandatory fields **/
  maxdev_write_ptr  write_reg;
  maxdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} maxdev_ctx_t;


typedef int32_t (*MAX30001_Init_Func)(void);
typedef int32_t (*MAX30001_DeInit_Func)(void);
typedef int32_t (*MAX30001_GetTick_Func)(void);
typedef int32_t (*MAX30001_Write_Func)(uint8_t *, uint32_t , uint8_t *, uint32_t );
typedef int32_t (*MAX30001_Read_Func)(uint8_t *, uint32_t , uint8_t *, uint32_t );

typedef struct
{
	MAX30001_Init_Func         Init;
	MAX30001_DeInit_Func       DeInit;
	MAX30001_Write_Func        Write;
	MAX30001_Read_Func         Read;
	MAX30001_GetTick_Func      GetTick;
	GPIO_InitTypeDef           csPin;
	GPIO_TypeDef *             csPort;
} MAX30001_IO_t;


typedef struct
{
  MAX30001_IO_t             IO;
  maxdev_ctx_t              Ctx;
  uint32_t                  smpl_ts;
  uint8_t                   ecg_is_initialized;
  uint8_t                   biz_is_initialized;
  uint8_t                   odr;
  uint8_t                   bioz_odr;
} MAX30001_Object_t;


int32_t MAX30001_RegisterBusIO(MAX30001_Object_t *pObj);
int MAX30001_reg_read(maxdev_ctx_t *ctx, MAX30001_REG_map_t addr,uint32_t *return_data);
int MAX30001_reg_write(maxdev_ctx_t *ctx, MAX30001_REG_map_t addr, uint32_t data);

#endif /* MAX30001_BUS_H_ */
