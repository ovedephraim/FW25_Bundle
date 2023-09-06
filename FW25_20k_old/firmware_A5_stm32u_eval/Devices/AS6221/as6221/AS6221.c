
#include <stddef.h>

#include "AS6221_reg.h"
#include "AS6221.h"


/**
  * @brief  Wrap Read register component function to Bus IO function
  * @param  Handle the device handler
  * @param  Reg the register address
  * @param  pData the stored data pointer
  * @param  Length the length
  * @retval 0 in case of success, an error code otherwise
  */
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  AS6221_Object_t *pObj = (AS6221_Object_t *)Handle;

  return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
}

/**
  * @brief  Wrap Write register component function to Bus IO function
  * @param  Handle the device handler
  * @param  Reg the register address
  * @param  pData the stored data pointer
  * @param  Length the length
  * @retval 0 in case of success, an error code otherwise
  */
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  AS6221_Object_t *pObj = (AS6221_Object_t *)Handle;

  return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
}


/**
  * @brief  Register Component Bus IO operations
  * @param  pObj the device pObj
  * @retval 0 in case of success, an error code otherwise
  */
int32_t AS6221_RegisterBusIO(AS6221_Object_t *pObj, AS6221_IO_t *pIO)
{
  int32_t ret = AS6221_OK;

  if (pObj == NULL)
  {
    ret = AS6221_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read_reg  = ReadRegWrap;
    pObj->Ctx.write_reg = WriteRegWrap;
    pObj->Ctx.handle   = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = AS6221_ERROR;
    }
    else if (pObj->IO.Init() != AS6221_OK)
    {
      ret = AS6221_ERROR;
    }
    else
    {

    }
  }

  return ret;
}

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t AS6221_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                         uint8_t *data,
                         uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t AS6221_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}


/*
	This function reads the temperature reading from the sensor
	and returns the value in degrees celsius.
*/
int32_t AS6221_readTemp(stmdev_ctx_t *ctx, int16_t * pval)
{
	int16_t digitalTempC = 0;
	AS6221_read_reg(ctx, AS6221_TVAL, (uint8_t *)&digitalTempC, 2);

	*pval = digitalTempC;
	return 0;
}

/* READ TEMPERATURE CELSIUS
	This function reads the temperature reading from the sensor
	and returns the value in degrees celsius.
*/
int AS6221_readTempC(stmdev_ctx_t *ctx, float * pval)
{
	int16_t digitalTempC = 0;
	AS6221_read_reg(ctx, AS6221_TVAL, (uint8_t *)&digitalTempC, 2);

	/* Multiplies by the resolution for digital to final temp */
	float finalTempC = digitalTempC * 0.0078125f;

	*pval = finalTempC;
	return 0;
}

