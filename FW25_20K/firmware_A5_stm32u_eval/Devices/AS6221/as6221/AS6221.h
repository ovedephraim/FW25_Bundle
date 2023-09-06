
#ifndef _TMP117_H_
#define _TMP117_H_


#include "stdint.h"
#include "stdbool.h"

#include "AS6221_reg.h"
#include "RpcFifo.h"

#define AS6221_OK                       0
#define AS6221_ERROR                   -1


/** I2C Device Address 8 bit format  if SA0=0 -> D5 if SA0=1 -> D7 **/
#define AS6221_I2C_ADD1                 0x88
#define AS6221_I2C_ADD2                 0x8C
#define AS6221_I2C_ADD3                 0x92

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

typedef int32_t (*AS6221_Init_Func)(void);
typedef int32_t (*AS6221_DeInit_Func)(void);
typedef int32_t (*AS6221_GetTick_Func)(void);
typedef int32_t (*AS6221_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*AS6221_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  AS6221_Init_Func          Init;
  AS6221_DeInit_Func        DeInit;
  uint8_t                   Address;
  AS6221_WriteReg_Func      WriteReg;
  AS6221_ReadReg_Func       ReadReg;
  AS6221_GetTick_Func       GetTick;
} AS6221_IO_t;

typedef struct
{
  AS6221_IO_t        IO;
  stmdev_ctx_t        Ctx;
  uint8_t             is_initialized;
  fifo_t             *fifo;
} AS6221_Object_t;


int32_t AS6221_RegisterBusIO(AS6221_Object_t *pObj, AS6221_IO_t *pIO);
int32_t AS6221_readTemp(stmdev_ctx_t *ctx, int16_t * pval);
int AS6221_readTempC(stmdev_ctx_t *ctx, float * pval);


#endif
