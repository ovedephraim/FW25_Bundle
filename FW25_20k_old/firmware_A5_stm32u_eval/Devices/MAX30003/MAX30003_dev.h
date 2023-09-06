
#ifndef MAX30003_DEV_H_
#define MAX30003_DEV_H_

#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "MAX30003/MAX30003.h"
#include "MAX30003_bus.h"

#define DEF_ODR_ECG_V (512)

/* ECG_SAMPLE type
 *	struct for storing a bit-mapped ECG sample in a 32-bit number
 *	packing and bit order is ignored as the total struct size is 32-bits
 */
typedef struct __attribute__((__packed__)) {
	int32_t	tag  : 3;	/* ETAG data from the ECG_FIFO	*/
	int32_t	step : 11;	/* time step of the sample		*/
    int32_t	data : 18;	/* voltage of the sample		*/
} ECG_SAMPLE_t;

typedef enum {
    ECG_LP_BYPASS	= 0,
    ECG_LP_40_HZ	= 1,
    ECG_LP_100_HZ	= 2,
    ECG_LP_150_HZ	= 3
} ECG_LOW_PASS_t;



typedef enum {
    ECG_RATE_MAX_SPS	= 0,
    ECG_RATE_MED_SPS	= 1,
    ECG_RATE_MIN_SPS	= 2,
    ECG_RATE_RESERVED	= 3
} ECG_SAMPLE_RATE_t;


/* ECG related api functions */
int MAX30003_dev_ECG_start_stream    (void);
int MAX30003_dev_ECG_stop_stream     (void);
int MAX30003_dev_ECG_init_sream      (void);
int MAX30003_dev_ECG_deinit_sream    (void);
int MAX30003_dev_ECG_handle_stream(void *p);

int MAX30003_hw_test(void * arg);

int32_t MAX30003_sw_rst(   MAX30003_Object_t *pObj);
int32_t MAX30003_synch(    MAX30003_Object_t *pObj);
int32_t MAX30003_RegDump(  MAX30003_Object_t *pObj);
int32_t MAX30003_fifo_rst( MAX30003_Object_t *pObj);
int32_t MAX30003_readID(   MAX30003_Object_t *pObj,
								uint32_t * pid);

int32_t MAX30003_CAL_InitStart(MAX30003_Object_t *pObj,
								uint8_t En_Vcal,
								uint8_t Vmode,
								uint8_t Vmag,
								uint8_t Fcal,
								uint16_t Thigh,
								uint8_t Fifty);

int32_t MAX30003_ECG_InitStart(MAX30003_Object_t *pObj,
								uint8_t En_ecg,
								uint8_t Openp,
								uint8_t Openn,
								uint8_t Pol,
								uint8_t Calp_sel,
								uint8_t Caln_sel,
								uint8_t E_fit,
								uint8_t Rate,
								uint8_t Gain,
								uint8_t Dhpf,
								uint8_t Dlpf);

int32_t MAX30003_Start_ECG(MAX30003_Object_t *pObj);
int32_t MAX30003_Stop_ECG(MAX30003_Object_t *pObj);


void ecg_sw_reset();
void ecg_synch      (MAX30003_Object_t *pObj);
void ecg_fifo_reset (MAX30003_Object_t *pObj);


/* MAX30003 register GET functions ************************************************************************
 *	each function reads the values from a register of the MAX30003 ECG device
 *  INPUTS: (use enumerated types)
 *		*vals	= pointer to a value struct to populate with the register's current values
 *	OUTPUTS:
 *		*vals	= updated with current register values
 *********************************************************************************************************/
void ecg_get_sample		(MAX30003_Object_t *pObj,MAX30003_FIFO_VALS			*vals);
void ecg_get_status		(MAX30003_Object_t *pObj,MAX30003_STATUS_VALS		*vals);
void ecg_get_en_int		(MAX30003_Object_t *pObj,MAX30003_EN_INT_VALS		*vals);
void ecg_get_en_int2	(MAX30003_Object_t *pObj,MAX30003_EN_INT_VALS		*vals);
void ecg_get_mngr_int	(MAX30003_Object_t *pObj,MAX30003_MNGR_INT_VALS		*vals);
void ecg_get_mngr_dyn	(MAX30003_Object_t *pObj,MAX30003_MNGR_DYN_VALS		*vals);
void ecg_get_cnfg_gen	(MAX30003_Object_t *pObj,MAX30003_CNFG_GEN_VALS		*vals);
void ecg_get_cnfg_cal	(MAX30003_Object_t *pObj,MAX30003_CNFG_CAL_VALS		*vals);
void ecg_get_cnfg_emux	(MAX30003_Object_t *pObj,MAX30003_CNFG_EMUX_VALS	*vals);
void ecg_get_cnfg_ecg	(MAX30003_Object_t *pObj,MAX30003_CNFG_ECG_VALS		*vals);

/* ecg_get_sample_burst *************************************************************************************
 *	Reads from the MAX30003 ECG sample fifo while there is valid data in the fifo
 *	INPUTS:
 *		*log	= pointer to an array of sample structs to hold the sample reading with data, step, tag
 *		SIZE	= constant size of the log array
 *	OUTPUTS:
 *		*log	= updated with sampled values
 *	RETURNS:
 *		the number of samples recorded into the log array, -1 if overflow
 ***********************************************************************************************************/
//int32_t ecg_get_sample_burst(ECG_SAMPLE_t *log, const uint16_t SIZE); /* returns number of samples recorded */

/* MAX30003 register SET functions ************************************************************************
 *	each function writes to a command register of the MAX30003 ECG device
 *  INPUTS: (use enumerated types)
 *		VALS	= constant register value struct
 *		MASKS	= bitwise OR of the the fields that should change
 *	OUTPUTS:
 *		n/a
 *********************************************************************************************************/
void ecg_set_en_int     (MAX30003_Object_t *pObj,const MAX30003_EN_INT_VALS     VALS, const MAX30003_EN_INT_MASKS       MASKS);
void ecg_set_en_int2    (MAX30003_Object_t *pObj,const MAX30003_EN_INT_VALS     VALS, const MAX30003_EN_INT_MASKS       MASKS);
void ecg_set_mngr_int	(MAX30003_Object_t *pObj,const MAX30003_MNGR_INT_VALS	VALS, const MAX30003_MNGR_INT_MASKS		MASKS);
void ecg_set_cnfg_gen	(MAX30003_Object_t *pObj,const MAX30003_CNFG_GEN_VALS	VALS, const MAX30003_CNFG_GEN_MASKS		MASKS);
void ecg_set_cnfg_ecg	(MAX30003_Object_t *pObj,const MAX30003_CNFG_ECG_VALS	VALS, const MAX30003_CNFG_ECG_MASKS		MASKS);

void ecg_set_mngr_dyn	(MAX30003_Object_t *pObj,const MAX30003_MNGR_DYN_VALS	VALS, const MAX30003_MNGR_DYN_MASKS		MASKS);
void ecg_set_cnfg_cal	(MAX30003_Object_t *pObj,const MAX30003_CNFG_CAL_VALS	VALS, const MAX30003_CNFG_CAL_MASKS		MASKS);
void ecg_set_cnfg_emux	(MAX30003_Object_t *pObj,const MAX30003_CNFG_EMUX_VALS	VALS, const MAX30003_CNFG_EMUX_MASKS	MASKS);
void ecg_set_cnfg_rtor1	(MAX30003_Object_t *pObj,const MAX30003_CNFG_RTOR1_VALS VALS, const MAX30003_CNFG_RTOR1_MASKS	MASKS);
void ecg_set_cnfg_rtor2	(MAX30003_Object_t *pObj,const MAX30003_CNFG_RTOR2_VALS VALS, const MAX30003_CNFG_RTOR2_MASKS	MASKS);

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

#endif /* MAX30003_DEV_H_ */
