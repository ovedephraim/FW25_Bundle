/**
* @file crc16.h
* @brief crc16 genrator/checker
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 20.10.2015
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef __CRC16_H
#define __CRC16_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t crc16 ( uint16_t   crc, uint8_t const *buffer, size_t len); 
uint16_t crc16_byte(uint16_t crc, uint8_t byte);


#ifdef __cplusplus
}
#endif



#endif


