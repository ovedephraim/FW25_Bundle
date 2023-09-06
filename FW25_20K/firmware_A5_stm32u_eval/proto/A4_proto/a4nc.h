/**
* @file gcell1nc.h
* @brief A4 protocol #1 notification codes
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 20.10.2015
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef _A4NC_H
#define _A4NC_H

/* General notification codes */
#define GC1_NT_EXT 					0x00

/* Functional notification codes */
#define GC1_NT_NOP 					0x01
#define GC1_NT_RESET 				0x02
#define GC1_NT_ALERT 				0x03

/* Reset cause codes */
#define	RS_UNKNOWN	0	/**< Unknown reset cause */
#define	RS_POR		1	/**< Power ON reset */
#define	RS_BROWNOUT	2	/**< Brownout reset */
#define	RS_SYSRESET	3	/**< Software requested reset */
#define	RS_LOCKUP	4	/**< Reset because of unrecoverable exception */
#define	RS_WATCHDOG	5	/**< Reset because of watchdog */


struct sNT_RESET
{
	uint8_t cause;							/**< Reset cause */
	uint32_t id __attribute__ ((packed));	/**< Reset identifier */
};

#ifdef __cplusplus
extern "C" {
#endif
 

#ifdef __cplusplus
}
#endif

#endif /* _A4NC_H */
