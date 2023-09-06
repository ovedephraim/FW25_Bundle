/**
* @file  a4err.h
* @brief A4 protocol #1 error codes
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 21.10.2015
*/


#ifndef _A4ERR_H
#define _A4ERR_H

#define A4_NOERR			 		0x0000
#define A4_BADOP 					0x0001
#define A4_BADFCS 					0x0002
#define A4_BADLEN 					0x0003
#define A4_BADSEL 					0x0004
#define A4_INTERR 					0x0005
#define A4_FLAGERR 					0x0006
#define A4_RANGE 					0x0007
#define A4_RADIOOFF 				0x0008
#define A4_RADIOCW 					0x0009

#endif /* _A4ERR_H */
