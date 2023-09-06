/**
* @file msg_type.h
* @brief message types and data structure
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 16.04.2022
*/
#ifndef _MSG_TYPE_H
#define _MSG_TYPE_H

#include <stddef.h>
#include <stdint.h>

#define MSG_TYPE_PACKET				0x00	/**< Communication packet attached to message */
#define MSG_TYPE_CMD				0x01	/**< Command in and/or attached to message */
#define MSG_TYPE_RESP				0x02	/**< Response in and/or attached to message */
#define MSG_TYPE_NOTIFY				0x03	/**< Notification in and/or attached to message */
#define MSG_TYPE_DEBUG_DATA_BLK	    0x04	/**< Data block attached to message. Number of elements in data field of the message */
#define MSG_TYPE_BT_DATA_BLK	    0x05	/**< Data block attached to message. Number of elements in data field of the message */
#define MSG_TYPE_BT_PACKET          0x06


/*
** Message source identifiers
*/
#define MSG_SRC_ISR_ECG			0		/**< Message from interrupt service routine */
#define MSG_SRC_ISR1	   	    1		/**< Message from interrupt service routine [RTC timer] */
#define MSG_SRC_ISR2		    2		/**< Message from interrupt service routine */
#define MSG_SRC_ISR3		    3		/**< Message from interrupt service routine */
#define MSG_SRC_ISR4		    4		/**< Message from interrupt service routine */
#define MSG_SRC_ISR5		    5		/**< Message from interrupt service routine */
#define MSG_SRC_ISR6		    6		/**< Message from interrupt service routine */
#define MSG_SRC_ISR7		    7		/**< Message from interrupt service routine */

#define MSG_SRC_AUX_DBRX		8		/**< Message from aux receiver task */
#define MSG_SRC_AUX_BTRX        9

#define MSG_SRC_AUXTX		9		/**< Message from aux transmitter task */
#define MSG_SRC_AUXCMD		10		/**< Message from aux cmd task */
#define MSG_SRC_SAMPLER		11		/**< Message from sampler task */
#define MSG_SRC_DSP			12		/**< Message from dispatcher task */
#define MSG_SRC_BTMNG		13		/**< Message from bt manager task */


// Message source identifiers 26-30 are reserved for future use */
#define MSG_SRC_TEST		31		/**< Message from test task */

struct sMsgHdrTypeBits
{
	uint16_t type:8;
	uint16_t source:5;
	uint16_t len:3;
};

union uMsgHdrType
{
	uint16_t all;
	struct sMsgHdrTypeBits bit;
};

#define MAKE_MSG_HDRTYPE(len,source,type) (uint16_t)(((((uint16_t)(len))&0x7)<<13)|((((uint16_t)(source))&0x1F)<<8)|(((uint16_t)(type))&0xFF))

typedef struct sMsgHdr
{
	union uMsgHdrType hdr;
	uint16_t data;
	void *buf;
} MSG_HDR;

typedef void (*CB_SYNC_COMPLETION_FN_t)(void *);

#ifdef __cplusplus
	extern "C" {
#endif


#ifdef __cplusplus
}
#endif


#endif

