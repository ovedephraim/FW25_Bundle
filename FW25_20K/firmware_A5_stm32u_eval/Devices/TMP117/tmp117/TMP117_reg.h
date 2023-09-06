

#ifndef _TMP117_REG_H_
#define _TMP117_REG_H_

#define TMP117_TEMP_RESULT   0X00
#define TMP117_CONFIGURATION 0x01
#define TMP117_T_HIGH_LIMIT  0X02
#define TMP117_T_LOW_LIMIT   0X03
#define TMP117_EEPROM_UL     0X04
#define TMP117_EEPROM1       0X05
#define TMP117_EEPROM2       0X06
#define TMP117_TEMP_OFFSET   0X07
#define TMP117_EEPROM3       0X08
#define TMP117_DEVICE_ID     0X0F

#define TMP117_H_ALERT  				0x8000
#define TMP117_L_ALERT  				0x4000
#define TMP117_DATA_READY  				0x2000
#define TMP117_EPROM_BUSY  				0x1000
#define TMP117_MOD1  			 	    0x0800
#define TMP117_MOD0  				    0x0400
#define TMP117_CONV2	  				0x0200
#define TMP117_CONV1	  				0x0100
#define TMP117_CONV0	  				0x0080
#define TMP117_AVG1		  				0x0040
#define TMP117_AVG0		 				0x0020
#define TMP117_T_nA		  				0x0010
#define TMP117_POL		  				0x0008
#define TMP117_DR_ALERT  				0x0004
#define TMP117_SOFT_RESET  				0x0002

#define TMP117_EUN                     	0x8000
#define TMP117_EPRM_BUSY				0x4000

#endif //_TMP117_REG_H_
