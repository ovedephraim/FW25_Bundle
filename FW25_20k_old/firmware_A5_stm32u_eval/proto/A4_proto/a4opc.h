/**
* @file  a4opc.h
* @brief A4 protocol #1 operation codes
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 20.10.2015
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef _A4OPC_H
#define _A4OPC_H

#include <stdint.h>

/* General operation codes */
#define GC1_OP_ACK				 		0x00
#define GC1_OP_NACK				 		0x01
#define GC1_OP_EXT				 		0x02

/* Functional operation codes */
#define GC1_OP_STATUS 					0x03
#define GC1_OP_INFO 					0x04
#define GC1_OP_LOGMASK 					0x05
#define GC1_OP_LOGSTART 				0x06
#define GC1_OP_LOGSTOP 					0x07
#define GC1_OP_GAIN 					0x08
#define GC1_OP_RATE 					0x09
#define GC1_OP_DETTHR 					0x0A
#define GC1_OP_DETLEVEL 				0x0B
#define GC1_OP_DETHYST 					0x0C
#define GC1_OP_ALERTRA 					0x0D
#define GC1_OP_ALERTCL 					0x0E
#define GC1_OP_RADIOCTL 				0x0F
#define GC1_OP_RADIOSEND				0x10
#define GC1_OP_RADIOSTATS				0x11



struct sOP_STATUS
{
	uint8_t opcode;
	uint8_t anscode;
};

#define GC_STATUS_LOGGER_ACTIVE_BIT		(0)
#define GC_STATUS_LOGGER_ACTIVE			(1<<GC_STATUS_LOGGER_ACTIVE_BIT)
#define GC_STATUS_RADIO_ON_BIT			(1)
#define GC_STATUS_RADIO_ON				(1<<GC_STATUS_RADIO_ON_BIT)
#define GC_STATUS_RADIO_CW_ON_BIT		(2)
#define GC_STATUS_RADIO_CW_ON			(1<<GC_STATUS_RADIO_CW_ON_BIT)
#define GC_STATUS_RADIO_ECHO_ON_BIT		(3)
#define GC_STATUS_RADIO_ECHO_ON			(1<<GC_STATUS_RADIO_ECHO_ON_BIT)
#define GC_STATUS_RADIO_RX_TAP_ON_BIT	(4)
#define GC_STATUS_RADIO_RX_TAP_ON		(1<<GC_STATUS_RADIO_RX_TAP_ON_BIT)
#define GC_STATUS_RADIO_TX_TAP_ON_BIT	(5)
#define GC_STATUS_RADIO_TX_TAP_ON		(1<<GC_STATUS_RADIO_TX_TAP_ON_BIT)


struct sACK_OP_STATUS_EXTRA
{
	uint32_t status;
};

struct sNACK_OP_STATUS_EXTRA
{
	uint16_t err_code;
};


#define GCID_SN				0x0000
#define GCID_HWVER			0x0001
#define GCID_SWVER			0x0002


struct sOP_OPINFO
{
	uint8_t opcode;
	uint8_t anscode;
	uint16_t info_id;
};

struct sACK_OP_INFO
{
	uint16_t info_id;
};

struct sACK_OP_INFO_SN
{
	uint16_t info_id;
	uint8_t sn[16];
};

struct sACK_OP_INFO_HW_VER
{
	uint16_t info_id;
	uint32_t hw_ver  __attribute__ ((packed));
};

struct sACK_OP_INFO_SW_VER
{
	uint16_t info_id;
	uint32_t sw_ver  __attribute__ ((packed));
};


union uACK_OP_INFO_EXTRA
{
	struct sACK_OP_INFO hdr;
	struct sACK_OP_INFO_SN sn;
	struct sACK_OP_INFO_SW_VER sw_ver;
	struct sACK_OP_INFO_HW_VER hw_ver;
};

struct sNACK_OP_INFO_EXTRA
{
	uint16_t err_code;
	uint16_t info_id;
};

struct sOP_LOGSTART
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t autostop_sel;
	uint8_t flags;
	uint32_t count_time;
};

#define LOG_AUTOSTOP_SEL_FOREVER	(0)
#define LOG_AUTOSTOP_SEL_CYCLES		(1)
#define LOG_AUTOSTOP_SEL_TIME		(2)

#define LOG_FLAG_ALGRESET		(1<<0)


struct sACK_OP_LOGSTART_EXTRA
{
	uint8_t autostop_sel;
	uint8_t flags;
	uint32_t count_time __attribute__ ((packed));
};

struct sNACK_OP_LOGSTART_EXTRA
{
	uint16_t err_code;
	uint8_t autostop_sel;
	uint8_t flags;
};


struct sOP_LOGSTOP
{
	uint8_t opcode;
	uint8_t anscode;
};

struct sACK_OP_LOGSTOP_EXTRA
{
	uint8_t autostop_sel;
	uint8_t rsvd;
	uint32_t count_time __attribute__ ((packed));
	uint32_t cycles_count __attribute__ ((packed));
	uint32_t elapsed_time __attribute__ ((packed));
};

struct sNACK_OP_LOGSTOP_EXTRA
{
	uint16_t err_code;
};


struct sOP_GAIN_GET
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t ch_id;
};

struct sOP_GAIN_SET
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t ch_id;
	uint8_t gain;
};

struct sACK_OP_GAIN_EXTRA
{
	uint8_t ch_id;
	uint8_t gain;
};

struct sNACK_OP_GAIN_EXTRA
{
	uint16_t err_code;
	uint8_t ch_id;
};

struct sOP_DETTHR_GET
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t ch_id;
};

struct sOP_DETTHR_SET
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t ch_id;
	uint16_t detthr __attribute__ ((packed));
};

struct sACK_OP_DETTHR_EXTRA
{
	uint8_t ch_id;
	uint16_t detthr __attribute__ ((packed));
};

struct sNACK_OP_DETTHR_EXTRA
{
	uint16_t err_code;
	uint8_t ch_id;
};

struct sOP_DETLEVEL_GET
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t ch_id;
};

struct sOP_DETLEVEL_SET
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t ch_id;
	uint8_t detlevel;
};

struct sACK_OP_DETLEVEL_EXTRA
{
	uint8_t ch_id;
	uint8_t detlevel;
};

struct sNACK_OP_DETLEVEL_EXTRA
{
	uint16_t err_code;
	uint8_t ch_id;
};

struct sOP_DETHYST_GET
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t ch_id;
};

struct sOP_DETHYST_SET
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t ch_id;
	uint16_t dethyst __attribute__ ((packed));
};

struct sACK_OP_DETHYST_EXTRA
{
	uint8_t ch_id;
	uint16_t dethyst __attribute__ ((packed));
};

struct sNACK_OP_DETHYST_EXTRA
{
	uint16_t err_code;
	uint8_t ch_id;
};

struct sOP_ALERTRA_READ
{
	uint8_t opcode;
	uint8_t anscode;
};

struct sOP_ALERTRA_READ_ACK
{
	uint8_t opcode;
	uint8_t anscode;
	uint32_t ackmask __attribute__ ((packed));
};

struct sACK_OP_ALERTRA_EXTRA
{
	uint32_t ackmask;
	uint32_t alerts;
	uint32_t ackstatus;
};

struct sNACK_OP_ALERTRA_EXTRA
{
	uint16_t err_code;
	uint32_t ackmask __attribute__ ((packed));
};

struct sOP_ALERTCL
{
	uint8_t opcode;
	uint8_t anscode;
	uint32_t clearmask __attribute__ ((packed));
};

struct sACK_OP_ALERTCL_EXTRA
{
	uint32_t clearmask;
	uint32_t oldalerts;
	uint32_t oldackstatus;
	uint32_t newalerts;
	uint32_t newackstatus;
};

struct sNACK_OP_ALERTCL_EXTRA
{
	uint16_t err_code;
	uint32_t clearmask __attribute__ ((packed));
};

#define RADIO_CTL_PWR_OFF		(0)
#define RADIO_CTL_PWR_ON		(1)
#define RADIO_CTL_CW_OFF		(2)
#define RADIO_CTL_CW_ON			(3)
#define RADIO_CTL_ECHO_OFF		(4)
#define RADIO_CTL_ECHO_ON		(5)
#define RADIO_CTL_RXTAP_OFF		(6)
#define RADIO_CTL_RXTAP_ON		(7)
#define RADIO_CTL_TXTAP_OFF		(8)
#define RADIO_CTL_TXTAP_ON		(9)

struct sOP_RADIOCTL
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t radio_cmd;
};

struct sACK_OP_RADIOCTL_EXTRA
{
	uint8_t radio_cmd;
	uint8_t radio_status;
};

struct sNACK_OP_RADIOCTL_EXTRA
{
	uint16_t err_code;
	uint8_t radio_cmd;
};

struct sOP_RADIOSEND_HDR
{
	uint8_t opcode;
	uint8_t anscode;
	uint16_t rsvd __attribute__ ((packed));
};

struct sACK_OP_RADIOSEND_EXTRA
{
	uint16_t rsvd;
	uint8_t txlen;
};

struct sNACK_OP_RADIOSEND_EXTRA
{
	uint16_t err_code;
	uint16_t rsvd;
	uint8_t txlen;
};

#define RADIO_STATS_READ		(0)
#define RADIO_STATS_READ_RESET	(1)

struct sOP_RADIOSTATS
{
	uint8_t opcode;
	uint8_t anscode;
	uint8_t radiostats_cmd;
};

struct sACK_OP_RADIOSTATS_EXTRA
{
	uint8_t radiostats_cmd;
	uint8_t rsvd;
	uint32_t tx_frames __attribute__ ((packed));
	uint32_t tx_echo_frames __attribute__ ((packed));
	uint32_t rx_frames __attribute__ ((packed));
	uint32_t rx_error_frames __attribute__ ((packed));
};

struct sNACK_OP_RADIOSTATS_EXTRA
{
	uint16_t err_code;
	uint8_t radiostats_cmd;
};


#endif /* _A4OPC_H */
