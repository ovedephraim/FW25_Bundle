/**
* @file flash_param.h
* @brief sread and write parameters to /from flash
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 04.05.2022
*/

#ifndef _FLASH_PARAM_H
#define _FLASH_PARAM_H


#include <stddef.h>
#include <stdint.h>
#include <string.h>



#ifdef __cplusplus
extern "C" {
#endif


/**
  * @brief reads data from flash memory
  * @param data - where to read
  * @param dataLen - how much to read
  * @param flash_address - from where to read
  * @retval The bank of a given address
  */
int save_channel_params(void *params, int chan_id);


/**
 * @brief restore_channel_params
 * @brief restore parametrs from flash mem
 * @param[out] params - parameters
 */

int restore_channel_params(void *params);

#ifdef __cplusplus
}
#endif


#endif /* _FLASH_PARAM_H */




