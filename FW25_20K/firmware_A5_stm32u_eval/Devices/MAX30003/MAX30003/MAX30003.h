/* MIT License
 *
 * Copyright (c) 2018 SealHAT: Seal Heart and Activity Tracker
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MAX30003_H
#define MAX30003_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <stdbool.h>
#include <stdint.h>


#define MAX30003_W_INDICATOR    (0)
#define MAX30003_R_INDICATOR    (1)
#define MAX30003_DATA_BYTES     (3)
#define MAX30003_CMND_BYTES     (1)

/**
 * REG addresses for the MAX30003 biopotential AFE
 **/
typedef enum {
    REG_NO_OP          = 0x00,  /* RW - no internal effect. DOUT = 0 during R, W ignored */
    REG_STATUS         = 0x01,  /* R  - */
    REG_EN_INT         = 0x02,  /* RW - */
    REG_EN_INT2        = 0x03,  /* RW - */
    REG_MNGR_INT       = 0x04,  /* RW - */
    REG_MNGR_DYN       = 0x05,  /* RW - */
    REG_SW_RST         = 0x08,  /* W  - */
    REG_SYNCH          = 0x09,  /* W  - */
    REG_FIFO_RST       = 0x0A,  /* W  - */
    REG_INFO           = 0x0F,  /* R  - */
    REG_CNFG_GEN       = 0x10,  /* RW - */
    REG_CNFG_CAL       = 0x12,  /* RW - */
    REG_CNFG_EMUX      = 0x14,  /* RW - */
    REG_CNFG_ECG       = 0x15,  /* RW - */
    REG_CNFG_RTOR1     = 0x1D,  /* RW - */
    REG_CNFG_RTOR2     = 0x1E,  /* RW - */
    REG_ECG_FIFO_BURST = 0x20,  /* R+ - */
    REG_ECG_FIFO       = 0x21,  /* R  - */
    REG_RTOR           = 0x25,  /* R  - */
    REG_NO_OP2         = 0x7F   /* RW - no internal effect. DOUT = 0 during R, W ignored */
}MAX30003_REG_map_t ;

/*** @brief STATUS (0x01) */

typedef union max30003_status_reg {
  uint32_t all;

  struct {
    uint32_t loff_nl   : 1;
    uint32_t loff_nh   : 1;
    uint32_t loff_pl   : 1;
    uint32_t loff_ph   : 1;
    uint32_t reserved1 : 4;
    uint32_t pllint    : 1;
    uint32_t samp      : 1;
    uint32_t rrint     : 1;
    uint32_t lonint    : 1;
    uint32_t reserved2 : 8;
    uint32_t dcloffint : 1;
    uint32_t fstint    : 1;
    uint32_t eovf      : 1;
    uint32_t eint      : 1;
    uint32_t reserved  : 8;
  } bit;
} max30003_status_t;

/*** @brief MNGR_INT (0x04) */

typedef union max30003_mngr_int_reg {
  uint32_t all;

  struct {
    uint32_t samp_it   : 2;
    uint32_t clr_samp  : 1;
    uint32_t reserved1 : 1;
    uint32_t clr_rrint : 2;
    uint32_t clr_fast  : 1;
    uint32_t reserved2 : 1;
    uint32_t reserved3 : 4;
    uint32_t reserved4 : 7;
    uint32_t e_fit     : 5;
    uint32_t reserved : 8;
  } bit;
} max30003_mngr_int_t;


/*** @brief CNFG_GEN (0x10) */

typedef union max30003_cnfg_gen {
   uint32_t all;
	struct {
	   uint32_t rbiasn    :1;
	   uint32_t rbiasp    :1;
	   uint32_t rbiasv    :2;
	   uint32_t en_rbias  :2;
	   uint32_t vth       :2;
	   uint32_t imag      :3;
	   uint32_t ipol      :1;
	   uint32_t en_dcloff :2;
	   uint32_t reserved1 :5;
	   uint32_t en_ecg    :1;
	   uint32_t fmstr     :2;
	   uint32_t en_ulp_lon :2;
	   uint32_t reserved  : 8;
	}bit;
} max30003_cnfg_gen_t;

/*** @brief CNFG_CAL (0x12) */

typedef union max30003_cnfg_cal {
	uint32_t all;
	struct {
	     uint32_t thigh     : 11;
	      uint32_t fifty     : 1;
	      uint32_t fcal      : 3;
	      uint32_t reserved1 : 5;
	      uint32_t vmag      : 1;
	      uint32_t vmode     : 1;
	      uint32_t en_vcal   : 1;
	      uint32_t reserved2 : 1;
	      uint32_t reserved  : 8;
	}bit;
} max30003_cnfg_cal_t;

/*** @brief CNFG_EMUX  (0x14) */

typedef union max30003_cnfg_emux {
   uint32_t all;
	struct {
	      uint32_t reserved1 : 16;
	      uint32_t caln_sel  : 2;
	      uint32_t calp_sel  : 2;
	      uint32_t openn     : 1;
	      uint32_t openp     : 1;
	      uint32_t reserved2 : 1;
	      uint32_t pol       : 1;
	      uint32_t reserved : 8;
	}bit;
} max30003_cnfg_emux_t;

/*** @brief CNFG_ECG   (0x15)  */

typedef union max30003_cnfg_ecg {
    uint32_t all;
    struct {
        uint32_t reserved1 : 12;
        uint32_t dlpf      : 2;
        uint32_t dhpf      : 1;
        uint32_t reserved2 : 1;
        uint32_t gain      : 2;
        uint32_t reserved3 : 4;
        uint32_t rate      : 2;
        uint32_t reserved  : 8;
    }bit;
} max30003_cnfg_ecg_t;




typedef enum {
    STATUS_LDOFF_NL    = 0x000001,
    STATUS_LDOFF_NH    = 0x000002,
    STATUS_LDOFF_PL    = 0x000004,
    STATUS_LDOFF_PH    = 0x000008,
    STATUS_PLLINT      = 0x000100,
    STATUS_SAMP        = 0x000200,
    STATUS_RRINT       = 0x000400,
    STATUS_LONINT      = 0x000800,
    STATUS_DCLOFFINT   = 0x100000,
    STATUS_FSTINT      = 0x200000,
    STATUS_EOVF        = 0x400000,
    STATUS_EINT        = 0x800000,
    STATUS_RESERVED    = 0x0FF0F0,
} MAX30003_STATUS_MASKS;

/***
 * EN_INT register's masks and values
 ***/
typedef enum {
    ENINT_INTB_TYPE   = 0x000003,
    ENINT_EN_PLLINT   = 0x000100,
    ENINT_EN_SAMP     = 0x000200,
    ENINT_EN_RRINT    = 0x000400,
    ENINT_EN_LONINT   = 0x000800,
    ENINT_EN_DCLOFFINT= 0x100000,
    ENINT_EN_FSTINT   = 0x200000,
    ENINT_EN_EOVF     = 0x400000,
    ENINT_EN_EINT     = 0x800000
} MAX30003_EN_INT_MASKS;

typedef enum {
    INTBTYPE_DISABLED    = 0,   /* Disabled (three-state) */
    INTBTYPE_CMOS_DRIVER = 1,   /* CMOS Driver */
    INTBTYPE_NMOS_DRIVER = 2,   /* Open-Drain NMOS Driver */
    INTBTYPE_NMOS_WITH_PU= 3    /* Open-Drain NMOS Driver w/ Internal 125k Pullup */
} ENINT_INTBTYPE_VAL;

typedef enum {
    ENPLLINT_DISABLED   = 0,
    ENPLLINT_ENABLED    = 1
} ENINT_ENPLLINT_VAL;

typedef enum {
    ENSAMP_DISABLED = 0,
    ENSAMP_ENABLED  = 1
} ENINT_ENSAMP_VAL;

typedef enum {
    ENRRINT_DISABLED    = 0,
    ENRRINT_ENABLED     = 1
} ENINT_ENRRINT_VAL;

typedef enum {
    ENLONINT_DISABLED   = 0,
    ENLONINT_ENABLED    = 1
} ENINT_ENLONINT_VAL;

typedef enum {
    ENDCLOFFINT_DISABLED    = 0,
    ENDCLOFFINT_ENABLED     = 1
} ENINT_ENDCLOFFINT_VAL;

typedef enum {
    ENFSTINT_DISABLED   = 0,
    ENFSTINT_ENABLED    = 1
} ENINT_ENFSTINT_VAL;

typedef enum {
    ENEOVF_DISABLED = 0,
    ENEOVF_ENABLED  = 1
} ENINT_ENEOVF_VAL;

typedef enum {
    ENINT_DISABLED  = 0,
    ENINT_ENABLED   = 1
} ENINT_ENEINT_VAL;

/***
 * MNG_INT register's masks and values
 ***/
typedef enum {
    MNGRINT_SAMP_IT     = 0x000003,
    MNGRINT_CLR_SAMP    = 0x000004,
    MNGRINT_CLR_RRINT   = 0x000030,
    MNGRINT_CLR_FAST    = 0x000040,
    MNGRINT_EFIT        = 0xF80000,
    MNGRINT_RESERVED    = 0x07FF88
} MAX30003_MNGR_INT_MASKS;

typedef enum {
    SAMPIT_EVERY_INST   = 0,
    SAMPIT_EVERY_2ND    = 1,
    SAMPIT_EVERY_4TH    = 2,
    SAMPIT_EVERY_16TH   = 3
} MNGRINT_SAMPIT_VAL;

typedef enum {
    CLRSAMP_ON_READ = 0,
    CLRSAMP_SELF    = 1
} MNGRINT_CLRSAMP_VAL;

typedef enum {
    CLRRINT_ON_STATUS = 0,
    CLRRINT_ON_RTOR   = 1,
    CLRRINT_SELF      = 2,
    _CLRRINT_RESERVED = 3
} MNGRINT_CLRRRINT_VAL;

typedef enum {
    CLRFAST_ON_FAST   = 0,
    CLRFAST_ON_STATUS = 1
} MNGRINT_CLRFAST_VAL;

// TODO enum for 0000 = 1 to 1111 = 32 to enforce "off-by-one" behaviour
typedef enum {
	EFIT_AS_1 = 0,
	EFIT_AS_2 = 1,
	EFIT_AS_3 = 2,
	EFIT_AS_4 = 3,
	EFIT_AS_5 = 4,
	EFIT_AS_6 = 5,
	EFIT_AS_7 = 6,
	EFIT_AS_8 = 7,
	EFIT_AS_9 = 8,
	EFIT_AS_10 = 9,
	EFIT_AS_11 = 10,
	EFIT_AS_12 = 11,
	EFIT_AS_13 = 12,
	EFIT_AS_14 = 13,
	EFIT_AS_15 = 14,
	EFIT_AS_16 = 15,
	EFIT_AS_17 = 16,
	EFIT_AS_18 = 17,
	EFIT_AS_19 = 18,
	EFIT_AS_20 = 19,
	EFIT_AS_21 = 20,
	EFIT_AS_22 = 21,
	EFIT_AS_23 = 22,
	EFIT_AS_24 = 23,
	EFIT_AS_25 = 24,
	EFIT_AS_26 = 25,
	EFIT_AS_27 = 26,
	EFIT_AS_28 = 27,
	EFIT_AS_29 = 28,
	EFIT_AS_30 = 29,
	EFIT_AS_31 = 30,
	EFIT_AS_32 = 31
	} MNGRINT_EFIT_VAL;

/***
 * MNG_DYN register's masks and values
 ***/
typedef enum {
    MNGRDYN_FAST        = 0xC00000,
    MNGRDYN_FAST_TH     = 0x3F0000,
    MNGRDYN_RESERVED    = 0x00FFFF
} MAX30003_MNGR_DYN_MASKS;

// TODO do something about
typedef uint8_t MNGRDYN_FASTTH_VAL;

typedef enum {
    FAST_NORMAL     = 0,
    FAST_MANUAL     = 1,
    FAST_AUTO       = 2,
    _FAST_RESERVED  = 3
} MNGRDYN_FAST_VAL;

//TODO SW_RST, SYNCH, FIFO_RST

/***
 * INFO register's masks and values
 ***/
typedef enum {
    INFO_REV_ID      = 0x0F0000,
    INFO_RESERVED    = 0x00CFFF,
    INFO_REQUIRED    = 0xF03000
} MAX30003_INFO_MASKS;

typedef uint8_t INFO_REV_ID_VAL;

typedef enum {
	REQUIRED_INFO	= 0x503000
} INFO_REQUIRED_VAL;

/***
 * CNFG_GEN register's masks and values
 ***/
typedef enum {
    CNFGGEN_RBIASN      = 0x000001,
    CNFGGEN_RBIASP      = 0x000002,
    CNFGGEN_RBIASV      = 0x00000C,
    CNFGGEN_EN_RBIAS    = 0x000030,
    CNFGGEN_VTH         = 0x0000C0,
    CNFGGEN_IMAG        = 0x000700,
    CNFGGEN_IPOL        = 0x000800,
    CNFGGEN_EN_DCLOFF   = 0x003000,
    CNFGGEN_EN_ECG      = 0x080000,
    CNFGGEN_FMSTR       = 0x300000,
    CNFGGEN_EN_ULP_LON  = 0xC00000,
    _CNFGGEN_RESERVED   = 0x07C000
} MAX30003_CNFG_GEN_MASKS;

typedef enum {
    RBIASN_NOT_CONNECTED    = 0,
    RBIASN_CONNECTED        = 1
} CNFGGEN_RBIASN_VAL;

typedef enum {
    RBIASP_NOT_CONNECTED    = 0,
    RBIASP_CONNECTED        = 1
} CNFGGEN_RBIASP_VAL;

typedef enum {
    RBIASV_50_MOHM      = 0,
    RBIASV_100_MOHM     = 1,
    RBIASV_200_MOHM     = 2,
    _RBIASV_RESERVED    = 3
} CNFGGEN_RBIASV_VAL;

typedef enum {
    ENRBIAS_DISABLED    = 0,
    EBRBIAS_ENABLED     = 1,
    _ENRBIAS_RESERVED1  = 2,
    _ENRBIAS_RESERVED2  = 3
} CNFGGEN_EN_RBIAS_VAL;

typedef enum {
    DCLOFFVTH_300_mV    = 0,
    DCLOFFVTH_400_mV    = 1,
    DCLOFFVTH_450_mV    = 2,
    DCLOFFVTH_500_mV    = 3
} CNFGGEN_DCLOFF_VTH_VAL;

typedef enum {
    DCLOFFIMAG_0_nA         = 0,
    DCLOFFIMAG_5_nA         = 1,
    DCLOFFIMAG_10_nA        = 2,
    DCLOFFIMAG_20_nA        = 3,
    DCLOFFIMAG_50_nA        = 4,
    DCLOFFIMAG_100_nA       = 5,
    _DCLOFFIMAG_RESERVED1   = 6,
    _DCLOFFIMAG_RESERVED2   = 7
} CNFGGEN_DCLOFF_IMAG_VAL;

typedef enum {
    DCLOFFIPOL_P_UP_N_DOWN  = 0,
    DCLOFFIPOL_P_DOWN_N_UP  = 1
} CNFGGEN_DCLOFF_IPOL_VAL;

typedef enum {
    ENDCLOFF_DISABLED   = 0,
    ENDCLOFF_ENABLED    = 1,
    _ENDCLOFF_RESERVED1 = 2,
    _ENDCLOFF_RESERVED2 = 3
} CNFGGEN_EN_DCLOFF_VAL;

typedef enum {
    ENECG_DISABLED  = 0,
    ENECG_ENABLED   = 1
} CNFGGEN_EN_ECG_VAL;

typedef enum {
    FMSTR_512_HZ    = 0, /* Fmstr = 32768Hz, Tres = 15.26us */
    FMSTR_500_HZ    = 1, /* Fmstr = 32000Hz, Tres = 15.63us */
    FMSTR_200_HZ    = 2, /* Fmstr = 32000Hz, Tres = 15.63us */
    FMSTR_199_HZ    = 3  /* (199.8049Hz) Fmstr = 31968.78Hz, Tres = 15.64us */
} CNFGGEN_FMSTR_VAL;

typedef enum {
    ENULPLON_DISABLED       = 0,
    ENULPLON_ENABLED        = 1,
    _ENULPLON_RESERVED1     = 2,
    _ENULPLON_RESERVED2     = 3
} CNFGGEN_EN_ULP_LON_VAL;

/***
 * CNFG_CAL register's masks and values
 ***/
typedef enum {
    CNFGCAL_THIGH       = 0x0003FF,
    CNFGCAL_FIFTY       = 0x000400,
    CNFGCAL_FCAL        = 0x007800,
    CNFGCAL_VMAG        = 0x100000,
    CNFGCAL_VMODE       = 0x200000,
    CNFGCAL_EN_VCAL     = 0x400000,
    CNFGCAL_RESERVED    = 0x8F8000
} MAX30003_CNFG_CAL_MASKS;

// TODO enforce 3 bytes
typedef uint16_t CNFGCAL_THIGH_VAL;

typedef enum {
    FIFTY_CAL_THIGH     = 0,
    FIFTY_50_PERCENT    = 1
} CNFGCAL_FIFTY_VAL;

typedef enum {
    FCAL_FREQ_256_HZ    = 0,
    FCAL_FREQ_64_HZ     = 1,
    FCAL_FREQ_16_HZ     = 2,
    FCAL_FREQ_4_HZ      = 3,
    FCAL_FREQ_1_HZ      = 4,
    FCAL_FREQ_1_4_HZ    = 5,
    FCAL_FREQ_1_16_HZ   = 6,
    FCAL_FREQ_1_64_HZ   = 7
} CNFGCAL_FCAL_VAL;

typedef enum {
    VMAG_250_uV = 0,
    VMAG_500_uV = 1
} CNFGCAL_VMAG_VAL;

typedef enum {
    VMODE_UNIPOLAR  = 0,
    VMODE_BIPOLAR   = 1
} CNFGCAL_VMODE_VAL;

typedef enum {
    ENVCAL_DISABLED = 0,
    ENVCAL_ENABLED  = 1
} CNFGCAL_EN_VCAL_VAL;

/***
 * CNFG_EMUX register's masks and values
 ***/
typedef enum {
    CNFGEMUX_CALN_SEL    = 0x030000,
    CNFGEMUX_CALP_SEL    = 0x0C0000,
    CNFGEMUX_OPENN       = 0x100000,
    CNFGEMUX_OPENP       = 0x200000,
    CNFGEMUX_POL         = 0x800000,
    _CNFGEMUX_RESERVED   = 0x40FFFF
} MAX30003_CNFG_EMUX_MASKS;

typedef enum {
    CALNSEL_IN_NONE     = 0,
    CALNSEL_IN_VMID     = 1,
    CALNSEL_IN_VCALP    = 2,
    CALNSEL_IN_VCALN    = 3
} CNFGEMUX_CALN_SEL_VAL;

typedef enum {
    CALPSEL_IN_NONE     = 0,
    CALPSEL_IN_VMID     = 1,
    CALPSEL_IN_VCALP    = 2,
    CALPSEL_IN_VCALN    = 3
} CNFGEMUX_CALP_SEL_VAL;

typedef enum {
    OPENN_CONNECTED = 0,
    OPENN_ISOLATED  = 1
} CNFGEMUX_OPENN_VAL;

typedef enum {
    OPENP_CONNECTED = 0,
    OPENP_ISOLATED  = 1
} CNFGEMUX_OPENP_VAL;

typedef enum {
    POL_NON_INVERTED    = 0,
    POL_INVERTED        = 1
} CNFGEMUX_POL_VAL;

/***
 * CNFG_ECG register's masks and values
 ***/
typedef enum {
    CNFGECG_DLPF        = 0x003000,
    CNFGECG_DHPF        = 0x004000,
    CNFGECG_GAIN        = 0x030000,
    CNFGECG_RATE        = 0xC00000,
    CNFGECG_RESERVED	= 0x3C8FFF
} MAX30003_CNFG_ECG_MASKS;

typedef enum {
    DLPF_BYPASS	= 0,
    DLPF_40_HZ	= 1,
    DLPF_100_HZ	= 2,
    DLPF_150_HZ	= 3
} CNFGECG_DLPF_VAL;

typedef enum {
    DHPF_BYPASS	= 0,
    DHPF_HALF	= 1
} CNFGECG_DHPF_VAL;

typedef enum {
    GAIN_20_V	= 0,
    GAIN_40_V	= 1,
    GAIN_80_V	= 2,
    GAIN_160_V	= 3
} CNFGECG_GAIN_VAL;

typedef enum {
	RATE_MAX_SPS	= 0,
	RATE_MED_SPS	= 1,
	RATE_MIN_SPS	= 2,
	RATE_RESERVED	= 3
} CNFGECG_RATE_VAL;

typedef enum {
    _RATE199_RESERVED1	= 0,
    _RATE199_RESERVED2  = 1,
    RATE199_SPS_199	    = 2,
    _RATE199_RESERVED3  = 3
} CNFGECG_RATE_199_PROG_VAL;

typedef enum {
    _RATE200_RESERVED1	= 0,
    _RATE200_RESERVED2	= 1,
    RATE200_SPS_200		= 2,
    _RATE200_RESERVED3 	= 3
} CNFGECG_RATE_200_PROG_VAL;

typedef enum {
    RATE500_SPS_500		= 0,
    RATE500_SPS_250		= 1,
    RATE500_SPS_125		= 2,
    _RATE500_RESERVED	= 3
} CNFGECG_RATE_500_PROG_VAL;

typedef enum {
    RATE512_SPS_512		= 0,
    RATE512_SPS_256		= 1,
    RATE512_SPS_128		= 2,
    _RATE512_RESERVED	= 3
} CNFGECG_RATE_512_PROG_VAL;

/***
 * CNFG_RTOR1 register's masks and values
 ***/
typedef enum {
    CNFGRTOR1_PTSF		= 0x000F00,
    CNFGRTOR1_PAVG		= 0x003000,
    CNFGRTOR1_EN_RTOR	= 0x008000,
    CNFGRTOR1_GAIN		= 0x0F0000,
    CNFGRTOR1_WNDW		= 0xF00000,
    CNFGRTOR1_RESERVED  = 0x0040FF
} MAX30003_CNFG_RTOR1_MASKS;

// TODO enforce 1 to 16 or remove
typedef uint8_t CNFGRTOR1_PTSF_VAL;

typedef enum {
    PAVG_WEIGHT_2	= 0,
    PAVG_WEIGHT_4	= 1,
    PAVG_WEIGHT_8	= 2,
    PAVG_WEIGHT_16  = 3
} CNFGRTOR1_PAVG_VAL;

typedef enum {
    ENRTOR_DISABLED	= 0,
    ENRTOR_ENABLED  = 1
} CNFGRTOR1_EN_RTOR_VAL;

typedef enum {
	GAIN_RTOR_1		= 0x0,
	GAIN_RTOR_2		= 0x1,
	GAIN_RTOR_4		= 0x2,
	GAIN_RTOR_8		= 0x3,
	GAIN_RTOR_16	= 0x4,
	GAIN_RTOR_32	= 0x5,
	GAIN_RTOR_64	= 0x6,
	GAIN_RTOR_128	= 0x7,
	GAIN_RTOR_256	= 0x8,
	GAIN_RTOR_512	= 0x9,
	GAIN_RTOR_1024	= 0xA,
	GAIN_RTOR_2048	= 0xB,
	GAIN_RTOR_4096	= 0xC,
	GAIN_RTOR_8192	= 0xD,
	GAIN_RTOR_16384	= 0xE,
	GAIN_RTOR_AUTO	= 0xF
} CNFGRTOR1_GAIN_VAL;

typedef enum {
	WNDW_RTOR_6		= 0x0,
	WNDW_RTOR_8		= 0x1,
	WNDW_RTOR_10	= 0x2,
	WNDW_RTOR_12	= 0x3,
	WNDW_RTOR_14	= 0x4,
	WNDW_RTOR_16	= 0x5,
	WNDW_RTOR_18	= 0x6,
	WNDW_RTOR_20	= 0x7,
	WNDW_RTOR_22	= 0x8,
	WNDW_RTOR_24	= 0x9,
	WNDW_RTOR_26	= 0xA,
	WNDW_RTOR_28	= 0xB,
	WNDW_RESERVED1	= 0xC,
	WNDW_RESERVED2	= 0xD,
	WNDW_RESERVED3	= 0xE,
	WNDW_RESERVED4	= 0xF
} CNFGRTOR1_WNDW_VAL;

/***
 * CNFG_RTOR2 register's masks and values
 ***/
typedef enum {
    CNFGRTOR2_RHSF		= 0x000700,
    CNFGRTOR2_RAVG		= 0x003000,
    CNFGRTOR2_HOFF		= 0x3F0000,
    CNFGRTOR2_RESERVED	= 0xC0C8FF
} MAX30003_CNFG_RTOR2_MASKS;

// TODO enforce 0-7 size or remove
typedef uint8_t CNFGRTOR2_RHSF_VAL;

typedef enum {
    RAVG_WEIGHT_2	= 0,
    RAVG_WEIGHT_4	= 1,
    RAVG_WEIGHT_8	= 2,
    RAVG_WEIGHT_16	= 3
} CNFGRTOR2_RAVG_VAL;

// TODO enforce 0 to 63 or remove
typedef uint8_t CNFGRTOR2_HOFF_VAL;

/***
 * FIFO register's masks and values
 ***/
typedef enum {
	ECGFIFO_PTAG	= 0x00000007,	// TODO what is this?
	ECGFIFO_ETAG	= 0x00000038,
	ECGFIFO_DATA	= 0xFFFFFFC0, // Allow for sign extension
} MAX30003_ECG_FIFO_MASKS;


typedef uint8_t ECGFIFO_PTAG_VAL;

typedef enum {
	ETAG_VALID			= 0,
	ETAG_FAST			= 1,
	ETAG_VALID_EOF		= 2,
	ETAG_FAST_EOF		= 3,
	_ETAG_RESERVED1		= 4,
	_ETAG_RESERVED2		= 5,
	ETAG_FIFO_EMPTY		= 6,
	ETAG_FIFO_OVERFLOW	= 7
} ECGFIFO_ETAG_VAL;

typedef int32_t ECGFIFO_DATA_VAL;

/***
 * RTOR register's masks and values
 ***/
typedef enum {
	RTOR_DATA		= 0xFFFC00,
	_RTOR_RESERVED  = 0x0003FF /* should be 0s */
} MAX30003_RTOR_MASKS;

typedef uint32_t RTOR_DATA_VAL;

/***
 * STRUCTURES For holding the values of each register
 ***/
// TODO consider creating union of vals structs for more generic interface
typedef struct MAX30003_STATUS_VALS {	/* all active high */
	bool ldoff_nl;
	bool ldoff_nh;
	bool ldoff_pl;
	bool ldoff_ph;
	bool pllint;
	bool samp;
	bool rrint;
	bool lonint;
	bool dcloffint;
	bool fstint;
	bool eovf;
	bool eint;
} MAX30003_STATUS_VALS;

typedef struct MAX30003_EN_INT_VALS {
	ENINT_INTBTYPE_VAL     intb_type;
	ENINT_ENPLLINT_VAL     en_pllint;
	ENINT_ENSAMP_VAL       en_samp;
	ENINT_ENRRINT_VAL      en_rrint;
	ENINT_ENLONINT_VAL     en_lonint;
	ENINT_ENDCLOFFINT_VAL  en_dcloffint;
	ENINT_ENFSTINT_VAL     en_fstint;
	ENINT_ENEOVF_VAL       en_eovf;
	ENINT_ENEINT_VAL       en_eint;
} MAX30003_EN_INT_VALS;

typedef struct MAX30003_MNGR_INT_VALS {
	MNGRINT_SAMPIT_VAL		samp_it;
	MNGRINT_CLRSAMP_VAL		clr_samp;
	MNGRINT_CLRRRINT_VAL	clr_rrint;
	MNGRINT_CLRFAST_VAL		clr_fast;
	MNGRINT_EFIT_VAL		efit;
} MAX30003_MNGR_INT_VALS;

typedef struct MAX30003_MNGR_DYN_VALS {
	MNGRDYN_FASTTH_VAL		fast_th;
	MNGRDYN_FAST_VAL		fast;
} MAX30003_MNGR_DYN_VALS;

typedef struct MAX30003_INFO_VALS {
	uint8_t				_verification;
	INFO_REV_ID_VAL		rev_id;
	uint8_t				_partid;
	uint16_t			_serialnumber;
} MAX30003_INFO_VALS;

typedef struct MAX30003_CNFG_GEN_VALS {
	CNFGGEN_RBIASN_VAL        rbiasn;
	CNFGGEN_RBIASP_VAL        rbiasp;
	CNFGGEN_RBIASV_VAL        rbiasv;
	CNFGGEN_EN_RBIAS_VAL      en_rbias;
	CNFGGEN_DCLOFF_VTH_VAL    vth;
	CNFGGEN_DCLOFF_IMAG_VAL   imag;
	CNFGGEN_DCLOFF_IPOL_VAL   ipol;
	CNFGGEN_EN_DCLOFF_VAL     en_dcloff;
	CNFGGEN_EN_ECG_VAL        en_ecg;
	CNFGGEN_FMSTR_VAL         fmstr;
	CNFGGEN_EN_ULP_LON_VAL    en_ulp_lon;
} MAX30003_CNFG_GEN_VALS;

typedef struct MAX30003_CNFG_CAL_VALS {
	CNFGCAL_THIGH_VAL	thigh;
	CNFGCAL_FIFTY_VAL	fifty;
	CNFGCAL_FCAL_VAL	fcal;
	CNFGCAL_VMAG_VAL	vmag;
	CNFGCAL_VMODE_VAL	vmode;
	CNFGCAL_EN_VCAL_VAL	en_vcal;
} MAX30003_CNFG_CAL_VALS;

typedef struct MAX30003_CNFG_EMUX_VALS {
	CNFGEMUX_CALN_SEL_VAL	caln_sel;
	CNFGEMUX_CALP_SEL_VAL	calp_sel;
	CNFGEMUX_OPENN_VAL		openn;
	CNFGEMUX_OPENP_VAL		openp;
	CNFGEMUX_POL_VAL		pol;
} MAX30003_CNFG_EMUX_VALS;

typedef struct MAX30003_CNFG_ECG_VALS {
	CNFGECG_DLPF_VAL	dlpf;
	CNFGECG_DHPF_VAL	dhpf;
	CNFGECG_GAIN_VAL	gain;
	CNFGECG_RATE_VAL	rate;
} MAX30003_CNFG_ECG_VALS;

typedef struct MAX30003_CNFG_RTOR1_VALS {
	CNFGRTOR1_PTSF_VAL		ptsf;
	CNFGRTOR1_PAVG_VAL		pavg;
	CNFGRTOR1_EN_RTOR_VAL	en_rtor;
	CNFGRTOR1_GAIN_VAL		gain;
	CNFGRTOR1_WNDW_VAL		wndw;
} MAX30003_CNFG_RTOR1_VALS;

typedef struct MAX30003_CNFG_RTOR2_VALS {
	CNFGRTOR2_RHSF_VAL	rhsf;
	CNFGRTOR2_RAVG_VAL	ravg;
	CNFGRTOR2_HOFF_VAL	hoff;
} MAX30003_CNFG_RTOR2_VALS;

typedef struct MAX30003_FIFO_VALS {
	ECGFIFO_PTAG_VAL	ptag;
	ECGFIFO_ETAG_VAL	etag;
	ECGFIFO_DATA_VAL	data;
} MAX30003_FIFO_VALS;

typedef struct MAX30003_RTOR_VALS {
	RTOR_DATA_VAL	data;
} MAX30003_RTOR_VALS;

typedef union MAX30003_VALS {
    MAX30003_STATUS_VALS        status;
    MAX30003_EN_INT_VALS        en_int;
    MAX30003_MNGR_INT_VALS      mngr_int;
    MAX30003_MNGR_DYN_VALS      mngr_dyn;
    MAX30003_INFO_VALS          info;
    MAX30003_CNFG_GEN_VALS      cnfg_gen;
    MAX30003_CNFG_CAL_VALS      cnfg_cal;
    MAX30003_CNFG_EMUX_VALS     cnfg_emux;
    MAX30003_CNFG_ECG_VALS      cnfg_ecg;
    MAX30003_CNFG_RTOR1_VALS    cnfg_rtor1;
    MAX30003_CNFG_RTOR2_VALS    cnfg_rtor2;
    MAX30003_FIFO_VALS          fifo;
    MAX30003_RTOR_VALS          rtor;
} MAX30003_VALS;


#define MAX30003_OK                       0
#define MAX30003_ERROR                   -1

/* MACROS for applying a read/write bit and shift to register address values */
#define ECG_REG_R(REG)  ( (uint8_t)(REG << 1) | MAX30003_R_INDICATOR )
#define ECG_REG_W(REG)  ( (uint8_t)(REG << 1) | MAX30003_W_INDICATOR )


/* MACROS for applying a read/write bit and shift to register address values */
#define ECG_REG_R(REG)  ( (uint8_t)(REG << 1) | MAX30003_R_INDICATOR )
#define ECG_REG_W(REG)  ( (uint8_t)(REG << 1) | MAX30003_W_INDICATOR )

#define ECG_BUF_SZ      (12)	/* SPI buffer size	                        */
#define ECG_BUF_CLR     (0x00)	/* clear byte		                        */
#define ECG_TIMEOUT     (4)     /* number of attempts for reading buffer    */

// TODO activate these and change ecg_spi_msg size as needed for static memory optimization */
#define ECG_CMND_SZ		(1)
#define ECG_DATA_SZ		(3)

/* SPI variables */
extern struct spi_xfer ecg_spi_msg;		/* SPI message struct	*/
extern uint8_t	ECG_BUF_I[ECG_BUF_SZ];	/* SPI input buffer		*/
extern uint8_t	ECG_BUF_O[ECG_BUF_SZ];	/* SPI output buffer	*/

enum ECG_WORD_POS {
    ECG_CMND_POS = 0,
    ECG_DATA_POS = 1
};

// TODO consider switching to bitfields
/* typedef to enforce size of SPI message components */
typedef struct MAX30003_DATA_t { uint8_t byte[3]; } MAX30003_DATA_t;
typedef uint8_t MAX30003_ADDR_t;

/* MAX30003_MSG type
 *	structure for storing a message to send/receive over SPI
 */
typedef struct MAX30003_MSG {
    uint8_t command;		/* address of the register to communicate with	*/
    MAX30003_DATA_t data;	/* data word to send to the addresses			*/
} MAX30003_MSG;

// TODO consider abstracting these again for portability
/* ASF function pointers for using SAML21 calls without cluttering the MAX30003 namespace */
//int32_t (*ecg_spi_xfer)(void * descriptor, const void *buffer);		/* spi_xfer */
//void    (*ecg_set_csb_level)(const uint8_t pin, const bool level);	/* gpio_set_pin_level */

/* initialization functions, run before using device */
//void ecg_init_spi(void *spi_desc, const void *spi_msg, uint32_t* spi_msg_sz);
//void ecg_init_csb(const uint8_t ecg_csb_pin);

// TODO sort/rename here
void ecg_fifo_reset();
void ecg_sw_reset();
void ecg_synch();
void ecg_sw_reset();



/* MAX30003 register DECODE functions *********************************************************************
 *	internal register functions for shifting and masking values out of words and updating value structs
 *  INPUTS: (use enumerated types)
 *		*vals	= pointer to a value struct to populate with the decoded data
 *		DATA	= 32-bits of data received from the ECG device
 *	OUTPUTS:
 *		*vals	= updated with current register values
 *********************************************************************************************************/
void ecg_decode_status		(MAX30003_STATUS_VALS		*vals, const MAX30003_DATA_t DATA);
void ecg_decode_en_int		(MAX30003_EN_INT_VALS		*vals, const MAX30003_DATA_t DATA);
void ecg_decode_mngr_int	(MAX30003_MNGR_INT_VALS		*vals, const MAX30003_DATA_t DATA);
void ecg_decode_mngr_dyn	(MAX30003_MNGR_DYN_VALS		*vals, const MAX30003_DATA_t DATA);
void ecg_decode_info		(MAX30003_INFO_VALS			*vals, const MAX30003_DATA_t DATA);
void ecg_decode_cnfg_gen	(MAX30003_CNFG_GEN_VALS		*vals, const MAX30003_DATA_t DATA);
void ecg_decode_cnfg_cal	(MAX30003_CNFG_CAL_VALS		*vals, const MAX30003_DATA_t DATA);
void ecg_decode_cnfg_emux	(MAX30003_CNFG_EMUX_VALS	*vals, const MAX30003_DATA_t DATA);
void ecg_decode_cnfg_ecg	(MAX30003_CNFG_ECG_VALS		*vals, const MAX30003_DATA_t DATA);
void ecg_decode_cnfg_rtor1	(MAX30003_CNFG_RTOR1_VALS	*vals, const MAX30003_DATA_t DATA);
void ecg_decode_cnfg_rtor2	(MAX30003_CNFG_RTOR2_VALS	*vals, const MAX30003_DATA_t DATA);
void ecg_decode_ecg_fifo	(MAX30003_FIFO_VALS			*vals, const MAX30003_DATA_t DATA);
void ecg_decode_rtor		(MAX30003_RTOR_VALS			*vals, const MAX30003_DATA_t DATA);

/* MAX30003 register ENCODE functions *********************************************************************
 *	internal register functions for shifting and masking values into words from value structs
 *  INPUTS: (use enumerated types)
 *		VALS	= structure of values to mask into a 32-bit data word
 *		*data	= 32-bit data word representing values to send to the ECG device
 *	OUTPUTS:
 *		*data	= data word updated with order bits (MSB -> LSB)
 *********************************************************************************************************/
void ecg_encode_en_int		(const MAX30003_EN_INT_VALS		VALS, MAX30003_DATA_t *data);
void ecg_encode_mngr_int	(const MAX30003_MNGR_INT_VALS	VALS, MAX30003_DATA_t *data);
void ecg_encode_mngr_dyn	(const MAX30003_MNGR_DYN_VALS	VALS, MAX30003_DATA_t *data);
void ecg_encode_cnfg_gen	(const MAX30003_CNFG_GEN_VALS	VALS, MAX30003_DATA_t *data);
void ecg_encode_cnfg_cal	(const MAX30003_CNFG_CAL_VALS	VALS, MAX30003_DATA_t *data);
void ecg_encode_cnfg_emux	(const MAX30003_CNFG_EMUX_VALS	VALS, MAX30003_DATA_t *data);
void ecg_encode_cnfg_ecg	(const MAX30003_CNFG_ECG_VALS	VALS, MAX30003_DATA_t *data);
void ecg_encode_cnfg_rtor1	(const MAX30003_CNFG_RTOR1_VALS VALS, MAX30003_DATA_t *data);
void ecg_encode_cnfg_rtor2	(const MAX30003_CNFG_RTOR2_VALS	VALS, MAX30003_DATA_t *data);

/* internal helper functions */
void ecg_clear_ibuf();
void ecg_clear_obuf();
void ecg_clear_iobuf();

/* ecg_mask ***********************************************************************************************
 *	Endian safe operation that builds a data word by combining old register values with values to update
 *	INPUTS: (use enumerated types)
 *		*new_vals	= the register values to include in the data word
 *		OLD_VALS	= constant register values currently programmed to the device
 *		MASKS		= bitwise OR of the values that should be updated
 *	OUTPUTS:
 *		*new_vals	= updated data word with the old values, overwritten with new values where requested
 **********************************************************************************************************/
void ecg_mask(MAX30003_DATA_t *new_vals, const MAX30003_DATA_t OLD_VALS, const uint32_t MASKS);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MAX30003_H */
