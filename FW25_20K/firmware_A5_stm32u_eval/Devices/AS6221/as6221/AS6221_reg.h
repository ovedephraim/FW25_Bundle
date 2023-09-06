
#ifndef _AS6221_REG_H_
#define _AS6221_REG_H_


#define AS6221_TVAL		    			0x00
#define AS6221_CONFIG  				    0x01
#define AS6221_TLOW						0x02
#define AS6221_THIGH				    0x03


#define AS6221_AL						0x20
#define AS6221_CR0						0x00
#define AS6221_CR1						0x40
#define AS6221_CR2						0x80
#define AS6221_CR3						0xc0

#define AS6221_SleepMode                0x0100
#define AS6221_IntMode					0x0200
#define AS6221_Polarity				    0x0400
#define AS6221_ConFault0				0x0
#define AS6221_ConFault1				0x0800
#define AS6221_ConFault2				0x1000
#define AS6221_ConFault3				0x1800
#define AS6221_SingleShot				0x8000

#endif
