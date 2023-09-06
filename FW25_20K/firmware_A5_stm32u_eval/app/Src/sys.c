/**
* @file sys.c
* @brief system functions
*
* @version 0.0.1
* @date 20.10.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include "../../app/Inc/sys.h"

#include "../../app/Inc/sys_errno.h"

SYS_param_t sys_param={0};

const char * get_sys_error_verb(int err)
{
	if(err==SYS_ERROR_NONE)                    return "SYS_ERROR_NONE";else
	if(err==SYS_ERROR_NO_INIT)                 return "SYS_ERROR_NO_INIT";else
	if(err==SYS_ERROR_WRONG_PARAM)             return "SYS_ERROR_WRONG_PARAM";else
	if(err==SYS_ERROR_BUSY)                    return "SYS_ERROR_BUSY";else
	if(err==SYS_ERROR_PERIPH_FAILURE)          return "SYS_ERROR_PERIPH_FAILURE";else
	if(err==SYS_ERROR_UNKNOWN_FAILURE)         return "SYS_ERROR_UNKNOWN_FAILURE";else
	if(err==SYS_ERROR_UNKNOWN_COMPONENT)       return "SYS_ERROR_UNKNOWN_COMPONENT";else
	if(err==SYS_ERROR_BUS_FAILURE)             return "SYS_ERROR_BUS_FAILURE";else
	if(err==SYS_ERROR_CLOCK_FAILURE)           return "SYS_ERROR_CLOCK_FAILURE";else
	if(err==SYS_ERROR_SELF_TEST_FAILURE)       return "SYS_ERROR_SELF_TEST_FAILURE";else
	if(err==SYS_ERROR_FEATURE_NOT_SUPPORTED)   return "SYS_ERROR_FEATURE_NOT_SUPPORTED";else
	if(err==SYS_ERROR_COMPONENT_FAILURE)       return "SYS_ERROR_COMPONENT_FAILURE";else
	if(err==SYS_ERROR_FLASH_FAILURE)           return "SYS_ERROR_FLASH_FAILURE";else
	if(err==SYS_ERROR_TIMEOUT)                 return "SYS_ERROR_TIMEOUT";else

	/* SYS BUS error codes */
	if(err==SYS_ERROR_BUS_TRANSACTION_FAILURE) return "SYS_ERROR_BUS_TRANSACTION_FAILURE"; else
	if(err==SYS_ERROR_BUS_ARBITRATION_LOSS)    return "SYS_ERROR_BUS_ARBITRATION_LOSS"; else
	if(err==SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE) return "SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE"; else
	if(err==SYS_ERROR_BUS_PROTOCOL_FAILURE)    return "SYS_ERROR_BUS_PROTOCOL_FAILURE"; else

	if(err==SYS_ERROR_BUS_MODE_FAULT)          return "SYS_ERROR_BUS_MODE_FAULT"; else
	if(err==SYS_ERROR_BUS_FRAME_ERROR)         return "SYS_ERROR_BUS_FRAME_ERROR"; else
	if(err==SYS_ERROR_BUS_CRC_ERROR)           return "SYS_ERROR_BUS_CRC_ERROR"; else
	if(err==SYS_ERROR_BUS_DMA_FAILURE)         return "SYS_ERROR_BUS_DMA_FAILURE"; else

	 /* Device Common Error codes */
	if(err==SYS_ERROR_DEVICE_FAILURE)          return "SYS_ERROR_DEVICE_FAILURE"; else

	 /* COM error codes */
	if(err==SYS_ERROR_BT_INIT_FAILURE)         return "SYS_ERROR_BT_INIT_FAILURE"; else
		return "SYS_ERROR_NONE";

}/* end of get_sys_error_verb */
