

#ifndef MAX3000N_H_
#define MAX3000N_H_

#include <stdint.h>
#include <math.h>

#define MAX3000N_OK                       0
#define MAX3000N_ERROR                   -1

typedef enum {
    ECG_GAIN_20_V	= 0,
    ECG_GAIN_40_V	= 1,
    ECG_GAIN_80_V	= 2,
    ECG_GAIN_160_V	= 3
} ECG_GAIN_t;

#define EN_ECG_COMMON     0b1
#define OPENP_COMMON      0b0
#define OPENN_COMMON      0b0
#define POL_COMMON        0b0
#define CALP_SEL_COMMON   0b10
#define CALN_SEL_COMMON   0b11

#define E_FIT_COMMON      15    //31//0xf//15
#define GAIN_COMMON       ECG_GAIN_40_V //0b01
#define DHPF_COMMON       0b01  //0b0
#define DLPF_COMMON       0b01  //0b01

/// Initialization values for CAL_InitStart()
#define EN_VCAL_COMMON    0b1
#define VMODE_COMMON      0b1
#define VMAG_COMMON       0b1
#define FCAL_COMMON       0b100 //0b011
#define THIGH_COMMON      0x520 //0x7FF
#define FIFTY_COMMON      0b1

uint8_t conv_odr_val(uint16_t val);
uint8_t conv_odr_bioz_val(uint16_t val);


int32_t MAX3000N_Start_ECG(void);

#endif /* MAX30001_H_ */
