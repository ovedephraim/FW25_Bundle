/**
* @file  gcell1comm.h
* @brief G-CELL protocol #1 Handler
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 15.10.2015
*/
#include <stddef.h>
#include <stdint.h>
#include "packetbuf.h"
#include "rxFramer.h"


#ifndef _A4COMM_H
#define _A4COMM_H

#define RX_A4_PAYLOAD_STATE 		1

#define RX_A4_MAX_PAYLOAD_LEN       256

#ifdef USE_TIMESTAMPS
PACKETBUF_HDR *a4FrameRx(struct sRxFramer *framer, uint8_t ch, void *arg, TIMESTAMP rxTS);
#else
PACKETBUF_HDR *a4FrameRx(struct sRxFramer *framer, uint8_t ch, void *arg);
#endif

#ifdef USE_TIMESTAMPS
void a4RxFrameSync(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS);
void a4RxEofFrameSync(struct sRxFramer *p, char rxChar, TIMESTAMP rxTS);
#else
void a4RxFrameSync(struct sRxFramer *p, char rxChar);
void a4RxEofFrameSync(struct sRxFramer *p, char rxChar);
#endif

#endif /* _A4COMM_H */

