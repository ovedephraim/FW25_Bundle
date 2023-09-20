/**
 * @file    xmodem.h
 * @author  Ferenc Nemeth
 * @date    21 Dec 2018
 * @brief   This module is the implementation of the Xmodem protocol.
 *
 *          Copyright (c) 2018 Ferenc Nemeth - https://github.com/ferenc-nemeth
 */

#ifndef XMODEM_H_
#define XMODEM_H_

//#include "flash.h"
#include "stdbool.h"

/* Xmodem (128 bytes) packet format
 * Byte  0:       Header
 * Byte  1:       Packet number
 * Byte  2:       Packet number complement
 * Bytes 3-130:   Data
 * Bytes 131-132: CRC
 */

/* Xmodem (1024 bytes) packet format
 * Byte  0:         Header
 * Byte  1:         Packet number
 * Byte  2:         Packet number complement
 * Bytes 3-1026:    Data
 * Bytes 1027-1028: CRC
 */

/* Maximum allowed errors (user defined). */
#define X_MAX_ERRORS 			3   //((uint8_t)3u)

/* Sizes of the packets. */
#define X_PACKET_NUMBER_SIZE    2	//((uint16_t)2u)
#define X_PACKET_128_SIZE     	128	//((uint16_t)128u)
#define X_PACKET_1024_SIZE   	1024	// ((uint16_t)1024u)
#define X_PACKET_CRC_SIZE    	2	// ((uint16_t)2u)

/* Indexes inside packets. */
#define X_PACKET_NUMBER_INDEX             	0	//((uint16_t)0u)
#define X_PACKET_NUMBER_COMPLEMENT_INDEX 	1	// ((uint16_t)1u)
#define X_PACKET_START_DATA          	    3
#define X_PACKET_CRC_HIGH_INDEX          	1027	// ((uint16_t)0u)
#define X_PACKET_CRC_LOW_INDEX           	1028	// ((uint16_t)1u)


/* Bytes defined by the protocol. */
#define X_SOH 	0x01	//((uint8_t)0x01u)  /**< Start Of Header (128 bytes). */
#define X_STX	0x02	// ((uint8_t)0x02u)  /**< Start Of Header (1024 bytes). */
#define X_EOT	0x04	// ((uint8_t)0x04u)  /**< End Of Transmission. */
#define X_ACK 	0x06	//((uint8_t)0x06u)  /**< Acknowledge. */
#define X_NAK 	0x15	//((uint8_t)0x15u)  /**< Not Acknowledge. */
#define X_CAN 	0x18	//((uint8_t)0x18u)  /**< Cancel. */
#define X_C  	0x43	// ((uint8_t)0x43u)  /**< ASCII "C" to notify the host we want to use CRC16. */
#define Y_HEADER 0x67

#define DATA_CHUNK   1029


/* Status report for the functions. */
typedef enum {
  X_OK            = 0x00u, /**< The action was successful. */
  X_ERROR_CRC     = 0x01u, /**< CRC calculation error. */
  X_ERROR_NUMBER  = 0x02u, /**< Packet number mismatch error. */
  X_ERROR_UART    = 0x04u, /**< UART communication error. */
  X_ERROR_FLASH   = 0x08u, /**< Flash related error. */
  X_ERROR         = 0xFFu  /**< Generic error. */
} xmodem_status;


typedef struct flash_file_param
{
	// A5 backup.
	uint8_t f_magic_val0[16];// =  {0x32,0x54,0x6a,0x95,0x4a,0x2d,0x6e,0xa7,0x80,0x90,0xa0,0xb0,0xc0,0xd0,0xe0,0xf0};
	uint8_t f_file0[16];
	uint32_t f_size0;
	uint32_t f_blocks0;
	uint16_t f_crc0;

	// A5 runtime.
	uint8_t f_magic_val1[16];// = {0x23,0x45,0xa6,0x59,0xa4,0xd2,0xe6,0x7a,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
	uint8_t f_file1[16];
	uint32_t f_size1;
	uint32_t f_blocks1;
	uint16_t f_crc1;

	// A5 runtime write.
	uint8_t f_magic_val2[16];// = {0x45,0x18,0xd1,0xa5,0x5a,0xc8,0xe2,0xdb,0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf1};
	uint8_t f_file2[16];
	uint32_t f_size2;
	uint32_t f_blocks2;
	uint32_t f_origin2;
	uint16_t f_crc2;

}flash_file_param;


//#define  FLASH_USER_NS_START_ADDR   0x08040000

#define  backup_a5_program		0
#define  runtime_a5_program		1
#define  runtime_a5_write		2
#define  A5_signature			3


uint32_t get_ymodem(void);
uint32_t get_xmodem(void);
//void flash_write(uint32_t numerator);
void transmit_notification(uint8_t * command);
uint32_t a5_program_presence(uint8_t src);
//uint32_t update_signature_switch(uint8_t sel);
//void write_signature(uint8_t signature);
uint32_t a5_signature_switch(void);
void erasesignature(uint8_t sel);
uint16_t checksum_adder(void);
void write_signature_switch(uint8_t signature);
uint32_t checksyheader(void);
void program_switcher(void);
void flash_read (uint32_t StartPageAddress, uint8_t *RxBuf, uint16_t numberofwords);
uint32_t transfer_flash_runtime_A5(uint32_t star);
void flash_transfer(uint32_t origin,uint32_t numerator);
void update_A5_signature_switch(uint8_t dest);
void write_A5_signature(uint8_t signature);
//void flasherasesection(uint32_start,uint32_stop);
void write_signature(uint8_t signature);


#endif /* XMODEM_H_ */
