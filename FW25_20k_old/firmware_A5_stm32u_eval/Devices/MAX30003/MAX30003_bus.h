
#ifndef MAX30003_BUS_H_
#define MAX30003_BUS_H_

#include <stdio.h>
#include <stdbool.h>

#include "MAX30003/MAX30003.h"



typedef int32_t (*max3dev_write_ptr)(void *, uint8_t *, uint32_t , uint8_t *, uint32_t );
typedef int32_t (*max3dev_read_ptr) (void *, uint8_t *, uint32_t , uint8_t *, uint32_t );

typedef struct {
  /** Component mandatory fields **/
  max3dev_write_ptr  write_reg;
  max3dev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} max3dev_ctx_t;

typedef int32_t (*MAX30003_Init_Func)(void);
typedef int32_t (*MAX30003_DeInit_Func)(void);
typedef int32_t (*MAX30003_GetTick_Func)(void);
typedef int32_t (*MAX30003_Write_Func)(uint8_t *, uint32_t , uint8_t *, uint32_t );
typedef int32_t (*MAX30003_Read_Func)(uint8_t *, uint32_t , uint8_t *, uint32_t );

typedef struct
{
	MAX30003_Init_Func         Init;
	MAX30003_DeInit_Func       DeInit;
	MAX30003_Write_Func        Write;
	MAX30003_Read_Func         Read;
	MAX30003_GetTick_Func      GetTick;
	GPIO_InitTypeDef           csPin;
	GPIO_TypeDef *             csPort;
} MAX30003_IO_t;

typedef struct
{
	MAX30003_IO_t             IO;
	max3dev_ctx_t             Ctx;
	uint32_t                  smpl_ts;
	uint8_t                   is_initialized;
	uint8_t                   odr;
} MAX30003_Object_t;

/**
  * @brief This function allows writing to a register.
  * @param addr:  Address of the register to write to
  * @param data:  24-bit data read from the register.
  * @returns 0-if no error.  A non-zero value indicates an error.
  *
  */
int MAX30003_reg_write(max3dev_ctx_t *ctx, MAX30003_REG_map_t addr, uint32_t data);

 /**
  * @brief This function allows reading from a register
  * @param addr:   Address of the register to read from.
  * @param *return_data: pointer to the value read from the register.
  * @returns 0-if no error.  A non-zero value indicates an error.
  *
  */
int MAX30003_reg_read(max3dev_ctx_t *ctx, MAX30003_REG_map_t addr,uint32_t *return_data);

/**
 * @brief This function allows reading from a register
 * @param addr:   Address of the register to read from.
 * @param *return_data: pointer to the value read from the register.
 * @returns 0-if no error.  A non-zero value indicates an error.
 *
 */
int32_t MAX30003_RegisterBusIO(MAX30003_Object_t *pObj);


#endif /* MAX30003_BUS_H_ */


