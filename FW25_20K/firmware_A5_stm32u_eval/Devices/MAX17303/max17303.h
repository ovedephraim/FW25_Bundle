/*
 ******************************************************************************
 * @file    max17303_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          MAX17303_reg.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAX17303_REGS_H
#define MAX17303_REGS_H

//#ifdef __cplusplus
//extern "C" {
//#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>


typedef int32_t (*max_stmdev_write_ptr)(void *, uint16_t, uint8_t *,
                                    uint16_t);
typedef int32_t (*max_stmdev_read_ptr) (void *, uint16_t, uint8_t *,
                                    uint16_t);

typedef struct {
  /** Component mandatory fields **/
  max_stmdev_write_ptr  write_reg;
  max_stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_max17303_ctx_t;

int32_t max17303_write_reg(stmdev_max17303_ctx_t *ctx,
									  uint16_t reg,
									  uint8_t *data,
									  uint8_t len);

int32_t max17303_read_reg(stmdev_max17303_ctx_t *ctx,
									 uint16_t reg,
								     uint8_t *data,
								     uint8_t len);



#define MAX17303_OK                       0
#define MAX17303_ERROR                   -1

#define RSENSE							 0.05f


/** I2C Device Address 8 bit format  if SA0=0 -> D5 if SA0=1 -> D7 **/
#define MAX17303_I2C_ADD                 0x6c
#define MAX17303_I2C_ADD_HIGH            0x16


enum max1730x_nvram {
	MAX1730X_NVRAM_START 	= 0x80,
	MAX1730X_NMANFCTRNAME0	= 0xCC,
	MAX1730X_NMANFCTRNAME1	= 0xCD,
	MAX1730X_NVPRTTH1 	= 0xD0,
	MAX1730X_NDPLIMIT	= 0xE0,
	MAX1730X_NSCOCVLIM	= 0xE1,

	MAX1730X_NVRAM_END 	= 0xEF,
	MAX1730X_HISTORY_START 	= 0xF0,
	MAX1730X_HISTORY_WRITE_STATUS_START = 0xF2,
	MAX1730X_HISTORY_VALID_STATUS_END = 0xFB,
	MAX1730X_HISTORY_WRITE_STATUS_END = 0xFE,
	MAX1730X_HISTORY_END	= 0xFF,
};

enum max1730x_register {
	MAX1730X_REPSOC = 0x06,
	MAX1730X_MAXMINVOLT = 0x08,
	MAX1730X_MAXMINTEMP = 0x09,
	MAX1730X_MAXMINCURR = 0x0A,
	MAX1730X_CONFIG = 0x0B,

	MAX1730X_FULLCAPREP = 0x10,
	MAX1730X_TTE = 0x11,
	MAX1730X_FULLSOCTHR = 0x13,
	MAX1730X_NFULLSOCTHR = 0x16,
	MAX1730X_AVGVCELL = 0x19,
	MAX1730X_VCELL = 0x1A,
	MAX1730X_TEMP = 0x1B,
	MAX1730X_CURRENT = 0x1C,
	MAX1730X_AVGCURRENT = 0x1D,
	MAX1730X_AVCAP = 0x1F,
	MAX1730X_TTF = 0x20,
	MAX1730X_DEVNAME = 0x21,
	MAX1730X_FILTERCFG = 0x29,
	MAX1730X_MIXCAP = 0x2B,
    MAX1730X_DIETEMP =  0x34,
	MAX1730X_FULLCAP = 0x35,
	MAX1730X_AVGDIETEMP = 0x40,
	MAX1730X_COMMSTAT = 0x61,
	MAX1730X_LEARNCFG = 0xA1,
	MAX1730X_MAXPEAKPWR = 0xA4,
	MAX1730X_SUSPEAKPWR = 0xA5,
	MAX1730X_PACKRESISTANCE = 0xA6,
	MAX1730X_SYSRESISTANCE = 0xA7,
	MAX1730X_MINSYSVOLTAGE = 0xA8,
	MAX1730X_MPPCURRENT = 0xA9,
	MAX1730X_SPPCURRENT = 0xAA,

	MAX1730X_CONFIG2 = 0xAB,
	MAX1730X_IALRTTH = 0xAC,
	MAX1730X_MINVOLT = 0xAD,
	MAX1730X_MINCURR = 0xAE,
	MAX1730X_NVPRTTH1BAK = 0xD6,
	MAX1730X_NPROTCFG = 0xD7,

	MAX1730X_nBattStatus = 0xA8,
	MAX1730X_ProtStatus = 0xD9,
	MAX1730X_ProtAlrt   = 0xAF,
	MAX1730X_nDesignVoltage   = 0x1E3,
	MAX1730X_HConfig2   = 0xF5,
	MAX1730X_nProtMiscTh  = 0xD6,
	MAX1730X_nStepChg  = 0xDB,
};

enum max1730x_command_bits {
	MAX1730X_COMMAND_FUEL_GAUGE_RESET = 0x8000,
	MAX1730X_READ_HISTORY_CMD_BASE = 0xE22E,
	MAX1730X_COMMAND_HISTORY_RECALL_WRITE_0 = 0xE29C,
	MAX1730X_COMMAND_HISTORY_RECALL_VALID_0 = 0xE29C,
	MAX1730X_COMMAND_HISTORY_RECALL_VALID_1 = 0xE29D,
};

typedef struct {
	uint8_t 	 Vdesign;
	uint8_t 	 Vminsys;

}max17303_fuelgauge;

//enum max17xxx_register {
//	MAX17XXX_COMMAND	= MAX1720X_COMMAND,
//};
//
//enum max17xxx_nvram {
//	MAX17XXX_QHCA = MAX1720X_NUSER18C,
//	MAX17XXX_QHQH = MAX1720X_NUSER18D,
//};

enum max17xxx_command_bits {
	MAX17XXX_COMMAND_NV_RECALL 	  = 0xE001,
};

#define MAX1730X_HISTORY_PAGE_SIZE \
	(MAX1730X_HISTORY_END - MAX1730X_HISTORY_START + 1)

#define MAX1730X_N_OF_HISTORY_FLAGS_REG \
	(MAX1730X_HISTORY_END - \
		MAX1730X_HISTORY_END + 1 + \
		MAX1730X_HISTORY_VALID_STATUS_END - \
		MAX1730X_HISTORY_START + 1)

enum max17x0x_reg_types {
	GBMS_ATOM_TYPE_MAP = 0,
	GBMS_ATOM_TYPE_REG = 1,
	GBMS_ATOM_TYPE_ZONE = 2,
	GBMS_ATOM_TYPE_SET = 3,
};

enum max17x0x_reg_tags {
	MAX17X0X_TAG_avgc,
	MAX17X0X_TAG_cnfg,
	MAX17X0X_TAG_mmdv,
	MAX17X0X_TAG_vcel,
	MAX17X0X_TAG_temp,
	MAX17X0X_TAG_curr,
	MAX17X0X_TAG_mcap,
	MAX17X0X_TAG_avgr,

	MAX17X0X_TAG_BCNT,
	MAX17X0X_TAG_SNUM,
	MAX17X0X_TAG_HSTY,
	MAX17X0X_TAG_BCEA,
	MAX17X0X_TAG_rset,
	MAX17X0X_TAG_BRES,
};


#define MAX17303_STATUS			0x00
#define MAX17303_ValrtTh		0x01
#define MAX17303_TalrtTh		0x02
#define MAX17303_SalrtTh		0x03
#define MAX17303_AtRate			0x04
#define MAX17303_RepCap			0x05
#define MAX17303_RepSOC         0x06
#define MAX17303_Age            0x07
#define MAX17303_MaxMinVolt     0x08
#define MAX17303_MaxMinTemp     0x09
#define MAX17303_MaxMinCurr     0x0a
#define MAX17303_Config         0x0b
#define MAX17303_QResidual      0x0c
#define MAX17303_MixSOC         0x0d
#define MAX17303_AvSOC          0x0e
#define MAX17303_MiscCfg        0x0f
#define MAX17303_FullCapRep     0x10
#define MAX17303_TTE	        0x11
#define MAX17303_VCellRep       0x12
#define MAX17303_FullSocThr     0x13
#define MAX17303_RSlow          0x14
#define MAX17303_RFast          0x15
#define MAX17303_AvgTA          0x16
#define MAX17303_Cycles         0x17
#define MAX17303_DesinCap       0x18
#define MAX17303_AvgVCell       0x19
#define MAX17303_VCell          0x1a
#define MAX17303_Temp           0x1b
#define MAX17303_Current        0x1c
#define MAX17303_AvgCurrent     0x1d
#define MAX17303_IChgTerm       0x1e
#define MAX17303_AvCapC         0x1f

#define MAX17303_TTF	        0x20
#define MAX17303_DevName        0x21
#define MAX17303_CurrRep        0x22
#define MAX17303_FullCapNom     0x23

#define MAX17303_AIN0           0x27
#define MAX17303_ChargingCurrent         0x28
#define MAX17303_FilterCfg      0x29
#define MAX17303_ChargingVoltage         0x2a
#define MAX17303_MixCap         0x2b

#define MAX17303_COFF           0x2f


#define MAX17303_QRTable20      0x32

#define MAX17303_DieTemp        0x34
//#define MAX17303_FullCap        0x35
#define MAX17303_IAvgEmpty      0x36

#define MAX17303_FStat2         0x39
#define MAX17303_VEmpty         0x3a

#define MAX17303_FStat          0x3d
#define MAX17303_Timer          0x3e
#define MAX17303_Vrelax         0x3f

#define MAX17303_AvgDieTemp     0x40

#define MAX17303_QRTable30      0x42
#define MAX17303_AtAvgCurrent   0x43

#define MAX17303_dQAss          0x45
#define MAX17303_dPAss          0x46

#define MAX17303_ProtTmrStat    0x49

#define MAX17303_VFRemCap       0x4a

#define MAX17303_QH             0x4d


#define MAX17303_RelaxCfg       0xa0
#define MAX17303_LearnCfg       0xa1
#define MAX17303_QHStart        0xa2

#define MAX17303_MaxPeakPower     0xa4
#define MAX17303_SusPeakPower     0xa5
#define MAX17303_PackResistance   0xa6
#define MAX17303_SysResistance    0xa7

#define MAX17303_Config2        0xab
#define MAX17303_IAlertTh       0xac
#define MAX17303_MinVolt        0xad
#define MAX17303_MinCurr        0xae
#define MAX17303_ProAlrts       0xaf

#define MAX17303_Status2        0xb0
#define MAX17303_Power          0xb1
#define MAX17303_VRipple        0xb2
#define MAX17303_AvgPower       0xb3

#define MAX17303_TTFCfg         0xb5
#define MAX17303_CVMixCap       0xb6
#define MAX17303_CVHalfTime     0xb7
#define MAX17303_CGTempCo       0xb8
#define MAX17303_AgeForecast    0xb9

#define MAX17303_FOTPStat       0xbb
#define MAX17303_PatchSel       0xbc

#define MAX17303_TimerH         0xbe


#define MAX17303_AvgCell1       0xd4
#define MAX17303_CELL1          0xd8
#define MAX17303_ProtStatus     0xd9
#define MAX17303_FProtStat      0xda
#define MAX17303_ModelCfg       0xdb
#define MAX17303_AtQResidual    0xdc
#define MAX17303_atTTE          0xdd
#define MAX17303_AtAvSOC        0xde
#define MAX17303_AtAvCap        0xdf

#define MAX17303_LeakCurr       0x6f

//#define MAX17303_nBattStatus    0xa8


#define MAX17303_nXTable0       0x180
#define MAX17303_nXTable1       0x181
#define MAX17303_nXTable2       0x182
#define MAX17303_nXTable3       0x183
#define MAX17303_nXTable4       0x184
#define MAX17303_nXTable5       0x185
#define MAX17303_nXTable6       0x186
#define MAX17303_nXTable7       0x187
#define MAX17303_nXTable8       0x188
#define MAX17303_nXTable9       0x189
#define MAX17303_nXTable10      0x18a
#define MAX17303_nXTable11      0x18b
#define MAX17303_nVAlrtTh       0x18c
#define MAX17303_nTAlrtTh       0x18d
#define MAX17303_nIAlrtTh       0x18e
#define MAX17303_nSAlrtTh       0x18f


#define MAX17303_nOCVTable0     0x190
#define MAX17303_nOCVTable1     0x191
#define MAX17303_nOCVTable2     0x192
#define MAX17303_nOCVTable3     0x193
#define MAX17303_nOCVTable4     0x194
#define MAX17303_nOCVTable5     0x195
#define MAX17303_nOCVTable6     0x196
#define MAX17303_nOCVTable7     0x197
#define MAX17303_nOCVTable8     0x198
#define MAX17303_nOCVTable9     0x199
#define MAX17303_nOCVTable10    0x19a
#define MAX17303_nOCVTable11    0x19b
#define MAX17303_nIChgTerm      0x18c
#define MAX17303_nFilterCfg     0x19d
#define MAX17303_nVEmpty        0x19e
#define MAX17303_nLearnCfg      0x19f


#define MAX17303_nFullCapNom    0x1a5
#define MAX17303_nRComp0        0x1a6
#define MAX17303_nTempCo        0x1a7
#define MAX17303_nBattStatus    0x1a8
#define MAX17303_nFullCapRep    0x1a9
#define MAX17303_nVoltTemp      0x1aa
#define MAX17303_nMaxMinCurr    0x1ab
#define MAX17303_nMaxMinVolt    0x1ac
#define MAX17303_nMaxMinTemp    0x1ad
#define MAX17303_nFullCapFlt    0x1ae
#define MAX17303_nTimerH        0x1af

#define MAX17303_nConfig        0x1b0
#define MAX17303_nRippleCfg     0x1b1
#define MAX17303_nMiscCfg       0x1b2
#define MAX17303_nDesignCap     0x1b3
#define MAX17303_nSBSCfg        0x1b4
#define MAX17303_nPackCfg       0x1b5
#define MAX17303_nRelaxCfg      0x1b6
#define MAX17303_nConvgCfg      0x1b7
#define MAX17303_nNVCfg0        0x1b8
#define MAX17303_nNVCfg1        0x1b9
#define MAX17303_nNVCfg2        0x1ba
#define MAX17303_nHibCfg        0x1bb
#define MAX17303_nROMID0        0x1bc
#define MAX17303_nROMID1        0x1bd
#define MAX17303_nROMID2        0x1be
#define MAX17303_nROMID3        0x1bf

#define MAX17303_nPReserved0    0x1c0
#define MAX17303_nPReserved1    0x1c1
#define MAX17303_nPReserved2    0x1c2
#define MAX17303_nPReserved3    0x1c3
#define MAX17303_nRGain         0x1c4
#define MAX17303_nPackResistance   0x1c5
#define MAX17303_nFullSOCThr    0x1c6
#define MAX17303_nTTFCfg        0x1c7
#define MAX17303_nCGain         0x1c8
#define MAX17303_nCurve      0x1c9
#define MAX17303_nTGain         0x1ca
#define MAX17303_nTOff          0x1cb
#define MAX17303_nManfctrName0  0x1cc
#define MAX17303_nManfctrName1  0x1cd
#define MAX17303_nManfctrName2  0x1ce
#define MAX17303_nRSense        0x1cf


#define MAX17303_nVPrtTh1       0x1d0
#define MAX17303_nTPrtTh1       0x1d1
#define MAX17303_nTPrtTh3       0x1d2
#define MAX17303_nIPrtTh1       0x1d3
#define MAX17303_nVPrtTh2       0x1d4
#define MAX17303_nTPrtTh2       0x1d5
#define MAX17303_nProtMiscTh    0x1d6
#define MAX17303_nProtCfg       0x1d7
#define MAX17303_nJEITAC        0x1d8
#define MAX17303_nJEITAV        0x1d9
#define MAX17303_nJEITACfg      0x1da
#define MAX17303_nStepChg       0x1db
#define MAX17303_nDelayCfg      0x1dc
#define MAX17303_nODSCTh        0x1dd
#define MAX17303_nODSCCfg       0x1de
#define MAX17303_nCheckSum      0x1df


#define MAX17303_nDPLimit       0x1e0
#define MAX17303_nScOcvLim      0x1e1
#define MAX17303_nAgeFcCfg      0x1e2
#define MAX17303_nDesignVoltage    0x1e3
#define MAX17303_nMiscCfg2      0x1e4
#define MAX17303_nRFast         0x1e5
#define MAX17303_nManfctrDate   0x1e6
#define MAX17303_nFirstUsed     0x1e7
#define MAX17303_nSerialNumber0 0x1e8
#define MAX17303_nSerialNumber1 0x1e9
#define MAX17303_nSerialNumber2 0x1ea
#define MAX17303_nDeviceName0   0x1eb
#define MAX17303_nDeviceName1   0x1ec
#define MAX17303_nDeviceName2   0x1ed
#define MAX17303_nDeviceName3   0x1ee
#define MAX17303_nDeviceName4   0x1ef


#define MAX17303_HProtCfg       0xf0
#define MAX17303_HProtCfg2      0xf1
#define MAX17303_ODSCHTh        0xf2
#define MAX17303_ODSCCfg        0xf3
#define MAX17303_MTPHCfg        0xf4
#define MAX17303_HConfig2       0xf5

#define MAX17303_VFOCV          0xfb
#define MAX17303_RComp          0xfc
#define MAX17303_HConfig        0xfd

#define MAX17303_VFSOC          0xff

#define LOCK1					0
#define LOCK2					1
#define LOCK3					2
#define LOCK4					3
#define LOCK5					4

#define nvlock_reg				0x6a00
#define COMMSTAT				0x61
#define COMMREG					0x60
#define LOCKREG 				0x7F
#define POR						0x000f


//struct max17x0x_reg {
//	int type;
//	int size;
//	union {
//		unsigned int base;
//		unsigned int reg;
//		const u16 *map16;
//		const u8 *map;
//	};
//};
//
//struct max17x0x_cache_data {
//	struct max17x0x_reg atom;
//	u16 *cache_data;
//};
///* Capacity Estimation */
// struct gbatt_capacity_estimation {
//	struct mutex batt_ce_lock;
//	struct delayed_work settle_timer;
//	int cap_tsettle;
//	int cap_filt_length;
//	int estimate_state;
//	bool cable_in;
//	int delta_cc_sum;
//	int delta_vfsoc_sum;
//	int cap_filter_count;
//	int start_cc;
//	int start_vfsoc;
//};
//
// /* see b/119416045 for layout */
// static const struct max17x0x_reg max1730x[] = {
// 	[MAX17X0X_TAG_avgc] = { ATOM_INIT_REG16(MAX1730X_AVGCURRENT)},
// 	[MAX17X0X_TAG_cnfg] = { ATOM_INIT_REG16(MAX1730X_CONFIG)},
// 	[MAX17X0X_TAG_mmdv] = { ATOM_INIT_REG16(MAX1730X_MAXMINVOLT)},
// 	[MAX17X0X_TAG_vcel] = { ATOM_INIT_REG16(MAX1730X_VCELL)},
// 	[MAX17X0X_TAG_temp] = { ATOM_INIT_REG16(MAX1730X_TEMP)},
// 	[MAX17X0X_TAG_curr] = { ATOM_INIT_REG16(MAX1730X_CURRENT)},
// 	[MAX17X0X_TAG_mcap] = { ATOM_INIT_REG16(MAX1730X_MIXCAP)},
// 	[MAX17X0X_TAG_avgr] = { ATOM_INIT_REG16(MAX1730X_NMANFCTRNAME1) },
//
// 	[MAX17X0X_TAG_BCNT] = { ATOM_INIT_MAP(0x8e, 0x8f, 0x9d, 0x9e, 0x9f,
// 					      0xb2, 0xb4, 0xb6, 0xc7, 0xe2)},
// 	[MAX17X0X_TAG_SNUM] = { ATOM_INIT_MAP(0xce, 0xe6, 0xe7, 0xe8, 0xe9,
// 					      0xea, 0xeb, 0xec, 0xed, 0xee,
// 					      0xef) },
//
// 	[MAX17X0X_TAG_HSTY] = { ATOM_INIT_SET(0xf0, 0xf2, 0xfb, 0xfe, 0xff) },
// 	[MAX17X0X_TAG_BCEA] = { ATOM_INIT_SET(MAX1730X_NMANFCTRNAME0,
// 					      MAX1730X_NDPLIMIT,
// 					      MAX1730X_NSCOCVLIM) },
// 	[MAX17X0X_TAG_rset] = { ATOM_INIT_SET16(MAX1730X_CONFIG2,
// 					MAX1730X_COMMAND_FUEL_GAUGE_RESET,
// 					700)},
// 	[MAX17X0X_TAG_BRES] = { ATOM_INIT_SET(MAX1730X_NMANFCTRNAME1,
// 					      MAX1730X_NMANFCTRNAME0) },
// };





#define MAX17303_EUN                     	0x8000
#define MAX17303_EPRM_BUSY				0x4000





int32_t max17303_id(uint8_t *val);
void init_max17303(void);
int32_t max17303_Reset_Full(void);
int32_t max17303_Reset_Fuel_Gauge(void);
int32_t max17303_Battery_Status(uint8_t *val);
int32_t max17303_Cell_Voltage(uint8_t *val);
int32_t max17303_Set_Config(uint16_t val);
int32_t max17303_Cell_MaxMin_Volt(uint8_t *val);
int32_t max17303_Current(uint8_t *val);
int32_t max17303_FilterCfg(uint8_t *val);
int32_t max17303_Cell_MaxMin_Curr(uint8_t *val);
int32_t max17303_Set_Cell_MaxMin_Curr(uint16_t val);
int32_t max17303_Cell_DieTemp(uint8_t *val);
int32_t max17303_DevName(uint8_t *val);
int32_t max17303_Max_Peak_Power(uint8_t *val);
int32_t max17303_Avg_Curr(uint8_t *val);
int32_t max17303_Avg_Die_Temp(uint8_t *val);
int32_t max17303_Set_nDesign_Voltage(void);
int32_t max17303_Get_Full_Cap_Rep(uint8_t *val);
int32_t max17303_init_modelgauge(void);
int32_t max17303_Full_Cap_Rep(uint8_t *val);
int32_t max17303_Rep_SOC(uint8_t *val);
int32_t max17303_nFullCapNom(uint8_t *val);
int32_t max17303_TTE(uint8_t *val);
int32_t max17303_TTF(uint8_t *val);
int32_t max17303_Battery_Status(uint8_t *val);
int32_t max17303_init_file(void);
int32_t max17303_Avg_Cap(uint8_t *val);
int32_t max17303_Set_Full_Cap_Rep(uint16_t val);
int32_t max17303_Get_RepSoc(uint8_t *val);
int32_t max17303_Get_Temp(uint8_t *val);
int32_t max17303_Avg_Vcell(uint8_t *val);
int32_t max17303_Set_Fets(uint8_t discharge , uint8_t charge);
int32_t max17303_Get_Fets(uint8_t *val);
uint8_t lockqm(void);
int32_t max17303_nDesignCap(uint8_t *val);
int32_t max17303_Get_Full_Cap_Rep_a(uint8_t *val);
int32_t max17303_Get_Full_Cap_Rep_b(uint8_t *val);

//int32_t Init_as6221(stmdev_ctx_t *ctx);
//int32_t AS6221_reset (stmdev_ctx_t *ctx);
//int32_t AS6221_red(stmdev_ctx_t *ctx,unsigned char rst,unsigned char led,unsigned char pwr);
//int32_t AS6221_blue(stmdev_ctx_t *ctx,unsigned char rst,unsigned char led,unsigned char pwr);
//int32_t AS6221_green(stmdev_ctx_t *ctx,unsigned char rst,unsigned char led,unsigned char pwr);
//int32_t AS6221_read (stmdev_ctx_t *ctx,unsigned char reg);



#endif /*lp55281_DRIVER_H */


