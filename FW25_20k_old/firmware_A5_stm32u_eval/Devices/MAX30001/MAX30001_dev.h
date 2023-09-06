
#ifndef MAX30001_DEV_H_
#define MAX30001_DEV_H_

#include <MAX30001/max30001.h>
#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "MAX30001_bus.h"

#define TESTING_ECG_FLAG 0
#define TESTING_BIOZ_FLAG 1
#define TESTING_PACE_FLAG 2
#define TESTING_RTOR_FLAG 3

#define TESTING_MAX30001_TIMEOUT_SECONDS 10
#define DEF_ODR_ECG_H (512)
#define DEF_ODR_BIOZ  (32)//

typedef enum eFlags {
  eStreaming_ECG,
  eStreaming_PACE,
  eStreaming_BIOZ,
  eStreaming_RtoR
} eFlags;


extern bool testing_max30001;
extern uint32_t testing_ecg_flags[4];
extern bool is_30001_found;

/* ECG related api functions */
int MAX30001_bit_func(void *para);
int MAX30001_dev_ECG_start_stream(void);
int MAX30001_dev_ECG_stop_stream(void);
int MAX30001_dev_ECG_init_sream(void);
int MAX30001_dev_ECG_deinit_sream(void);

int MAX30001_dev_ECG_init_sreamTest(void);

int MAX30001_dev_vref_init(void);

int MAX30001_dev_BIOZ_start_stream(void);
int MAX30001_dev_BIOZ_stop_stream(void);
int MAX30001_dev_BIOZ_init_sream(void);
int MAX30001_dev_BIOZ_deinit_sream(void);

int MAX30001_dev_handle_stream(void * p);
int MAX30001_hw_test(void * arg);

int MAX30001_Dev_IsStreaming(eFlags flag);
int MAX30001_AnyStreamingSet(void);
void MAX30001_Dev_Debug_ShowStreamFlags(void);
void MAX30001_Dev_StartSync(void);
void MAX30001_Dev_SetupInterrupts(void);
void MAX30001_Dev_SetStreamingFlag(eFlags flag, uint8_t state);
void MAX30001_Dev_Stop(void);
void MAX30001_Dev_ClearStreamingFlags(void);
uint8_t *MAX30001_Dev_getVersion(void);


int32_t MAX30001_Stop_ECG(MAX30001_Object_t *pObj);
int32_t MAX30001_Start_ECG(MAX30001_Object_t *pObj);
int32_t MAX30001_ECG_InitStart(MAX30001_Object_t *pObj,
	uint8_t En_ecg,  uint8_t Openp,   uint8_t Openn, uint8_t Pol,
	uint8_t Calp_sel,uint8_t Caln_sel,uint8_t E_fit, uint8_t Rate,
	uint8_t Gain,    uint8_t Dhpf,    uint8_t Dlpf);

int MAX30001_Stop_BIOZ(MAX30001_Object_t *pObj);
int MAX30001_Start_BIOZ(MAX30001_Object_t *pObj);
int MAX30001_BIOZ_InitStart(MAX30001_Object_t *pObj,
	uint8_t En_bioz,
	uint8_t Openp,
	uint8_t Openn,
	uint8_t Calp_sel,
	uint8_t Caln_sel,
	uint8_t CG_mode,
	uint8_t B_fit,
	uint8_t Rate,
	uint8_t Ahpf,
	uint8_t Ext_rbias,
	uint8_t Gain,
	uint8_t Dhpf,
	uint8_t Dlpf,
	uint8_t Fcgen,
	uint8_t Cgmon,
	uint8_t Cgmag,
	uint8_t Phoff);


int32_t MAX30001_INT_assignment(MAX30001_Object_t *pObj,
	max30001_intrpt_Location_t en_enint_loc,
	max30001_intrpt_Location_t en_eovf_loc,
	max30001_intrpt_Location_t en_fstint_loc,
	max30001_intrpt_Location_t en_dcloffint_loc,
	max30001_intrpt_Location_t en_bint_loc,
	max30001_intrpt_Location_t en_bovf_loc,
	max30001_intrpt_Location_t en_bover_loc,
	max30001_intrpt_Location_t en_bundr_loc,
	max30001_intrpt_Location_t en_bcgmon_loc,
	max30001_intrpt_Location_t en_pint_loc,
	max30001_intrpt_Location_t en_povf_loc,
	max30001_intrpt_Location_t en_pedge_loc,
	max30001_intrpt_Location_t en_lonint_loc,
	max30001_intrpt_Location_t en_rrint_loc,
	max30001_intrpt_Location_t en_samp_loc,
	max30001_intrpt_type_t  intb_Type,
	max30001_intrpt_type_t int2b_Type);



int32_t MAX30001_INT_ECG_assignment(MAX30001_Object_t *pObj,
	max30001_intrpt_Location_t en_enint_loc,
	max30001_intrpt_Location_t en_eovf_loc,
	max30001_intrpt_Location_t en_fstint_loc,
	max30001_intrpt_Location_t en_dcloffint_loc,
	max30001_intrpt_Location_t en_bovf_loc,
	max30001_intrpt_Location_t en_bover_loc,
	max30001_intrpt_Location_t en_bundr_loc,
	max30001_intrpt_Location_t en_bcgmon_loc,
	max30001_intrpt_Location_t en_pint_loc,
	max30001_intrpt_Location_t en_povf_loc,
	max30001_intrpt_Location_t en_pedge_loc,
	max30001_intrpt_Location_t en_lonint_loc,
	max30001_intrpt_Location_t en_rrint_loc,
	max30001_intrpt_Location_t en_samp_loc,
	max30001_intrpt_type_t  intb_Type,
	max30001_intrpt_type_t int2b_Type);

int32_t MAX30001_BIOZ_INT_assignment(MAX30001_Object_t *pObj,
	max30001_intrpt_Location_t en_bint_loc);



int32_t MAX30001_CAL_InitStart(MAX30001_Object_t *pObj,
	uint8_t En_Vcal,
	uint8_t Vmode,
	uint8_t Vmag,
	uint8_t Fcal,
	uint16_t Thigh,
	uint8_t Fifty);
int32_t MAX30001_Enable_DcLeadOFF_Init(
	MAX30001_Object_t *pObj,
	int8_t En_dcloff,
	int8_t Ipol,
	int8_t Imag,
	int8_t Vth);
int32_t MAX30001_get_status(MAX30001_Object_t *pObj,
	uint32_t * all);
int32_t MAX30001_readID(MAX30001_Object_t *pObj,
	uint32_t *pid);
int32_t MAX30001_FIFO_LeadONOff_Read(MAX30001_Object_t *pObj);
int32_t MAX30001_fifo_rst(MAX30001_Object_t *pObj);
int32_t MAX30001_synch(MAX30001_Object_t *pObj);
int32_t MAX30001_sw_rst(MAX30001_Object_t *pObj);
int32_t MAX30001_RegDump(MAX30001_Object_t *pObj);





#endif /* MAX30001_DEV_H_ */
