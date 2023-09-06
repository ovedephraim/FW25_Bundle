

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BUS_H
#define BUS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


//============== I2C bus API =====================================================================

#define BUS_I2C2_POLL_TIMEOUT                0x1000U
#define BUS_I2C4_POLL_TIMEOUT                0x1000U

extern I2C_HandleTypeDef hi2c1;

/* BUS IO driver over I2C Peripheral */

int32_t BUS_I2C2_Init(void);
int32_t BUS_I2C2_DeInit(void);
int32_t BUS_I2C2_IsReady(uint16_t DevAddr, uint32_t Trials);
int32_t BUS_I2C2_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BUS_I2C2_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BUS_I2C2_WriteReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BUS_I2C2_ReadReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BUS_I2C2_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BUS_I2C2_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BUS_I2C2_SendRecv(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t Length);

int32_t BUS_I2C4_Init(void);
int32_t BUS_I2C4_DeInit(void);
int32_t BUS_I2C4_IsReady(uint16_t DevAddr, uint32_t Trials);
int32_t BUS_I2C4_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BUS_I2C4_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);


//============== SPI bus API =====================================================================

int32_t BUS_SPI3_Init(void);
int32_t BUS_SPI3_DeInit(void);
int32_t BUS_SPI3_IsReady(uint16_t DevAddr, uint32_t Trials);
int32_t BUS_SPI3_Write(uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size);
int32_t BUS_SPI3_Read(uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size);

//int32_t BUS_SPI2_Init(uint8_t SpiDev);
int32_t BUS_SPI2_Init(void);
int32_t BUS_SPI2_DeInit(void);
int32_t BUS_SPI2_IsReady(uint16_t DevAddr, uint32_t Trials);
int32_t BUS_SPI2_Write(uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size);
int32_t BUS_SPI2_Read(uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size);

//=============== ADC bus API ===================================================================

#define DIGITAL_SCALE_14BITS  (0x3FFFUL)
#define ADC_OFFSET_VAL        (DIGITAL_SCALE_14BITS>>1)

typedef enum seq_num
{
	ADC1_CH0 =0,
	ADC1_CH1 =1,
	ADC1_CH2 =2,
	ADC1_CH3 =3,
	ADC1_CH4
}adc1_seq_num;

typedef bool (*tf_cmplt_cbf)(void *p, void *p2,uint32_t len);

typedef struct reg_param
{
	MEMBUF_POOL * mp;
	ADC_ChannelConfTypeDef * adc_seq_hndle;
	adc1_seq_num sn; //adc sequence id
	tf_cmplt_cbf cb;
	uint8_t * extBuff;
	int inv; //Inverted 0 not 1 yes
	uint16_t extBuffLen;
}adc_reg_param_t;

int32_t BUS_ADC1_exec_run(void);
int32_t BUS_ADC1_exec_stop(void);
int32_t BUS_ADC1_init_and_register(adc_reg_param_t *rp);
int32_t BUS_ADC1_deinit_and_unregister(adc1_seq_num sn);


//============== common bus API ==================================================================

int32_t BUS_GetTick(void);


#ifdef __cplusplus
}
#endif

#endif /* _BUS_H */


