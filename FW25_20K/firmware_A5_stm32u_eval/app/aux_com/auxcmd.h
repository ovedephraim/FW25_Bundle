/**
* @file  auxcmd.h
* @brief auxiliary commands info
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 22.05.2022
*/

#ifndef _AUXCMD_H
#define _AUXCMD_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "msg_type.h"
#include "A5_proto/parcel.h"


#define BLE_AUX        1
#define DBG_AUX        2
#define DBG_AUX_PROTO  3

#define DBG_TX_BUFF_SIZE	128
#define N_DBG_TX_BUFFERS	(80)

#define DBG_TX_BBUFF_SIZE	(MAX_PARCEL_LEN)
#define N_DBG_TX_BBUFFERS	(1)

#define RXBUFFERSIZE                2048
#define BTRXBUFFERSIZE              2048
#define MT_PACKET_FROM_AUX_CMD		0x60
#define MT_PACKET_FROM_ECG_ISR		0x71

#define CONCAT_MODNAME(n, s) (n " " s)

#define IS_MT_PACKET_FROM_AUX_CMD(x) ((x)==(MT_PACKET_FROM_AUX_CMD))

#define MSGHDR_AUXCMDRESP_PACKET            MAKE_MSG_HDRTYPE(0,MSG_SRC_AUXCMD,MSG_TYPE_PACKET)
#define MSGHDR_AUXCMD_DEB_BULK_PACKET       MAKE_MSG_HDRTYPE(0, MSG_SRC_DSP, MSG_TYPE_DEBUG_DATA_BLK)
#define MSGHDR_AUXCMD_BLE_BULK_PACKET       MAKE_MSG_HDRTYPE(0, MSG_SRC_DSP, MSG_TYPE_BT_PACKET)
#define MSGHDR_AUXCMD_DEB_BULK_FPACKET      MAKE_MSG_HDRTYPE(1, MSG_SRC_DSP, MSG_TYPE_DEBUG_DATA_BLK)
#define MSGHDR_AUXCMD_BLE_BULK_FPACKET      MAKE_MSG_HDRTYPE(1, MSG_SRC_DSP, MSG_TYPE_BT_DATA_BLK)

#define MSGHDR_AUXCMD_BLE_SYNC_CMD          MAKE_MSG_HDRTYPE(0, MSG_SRC_BTMNG, MSG_TYPE_CMD)
#define MSGHDR_AUXCMD_NOTIFICATION_PACKET   MAKE_MSG_HDRTYPE(0,MSG_SRC_AUXCMD,MSG_TYPE_PACKET)

#define MSGHDR_AUXCMD MAKE_MSG_HDRTYPE(0,MSG_SRC_AUXCMD,MSG_TYPE_CMD)

typedef int (*callback_aux_t)(void *, int , void *, size_t);

void \
initAuxDevParams(void);

int \
startAuxTasks(uint16_t ustxStackDepth, UBaseType_t txprio, uint16_t usrxStackDepth, UBaseType_t rxprio);

void \
startAuxReception(callback_aux_t _cb, int aux_type);

int \
aux_sendToAux(void *packet,
		uint32_t len,
		uint32_t timeout,
		bool _alloc,
		uint8_t aux);

int \
aux_sendPacketToAux(void *packet,
		uint32_t len,
		uint16_t hdr,
		uint32_t timeout);


void BT_UARTX_Init(uint32_t br);

#endif /* _AUXCMD_H */
