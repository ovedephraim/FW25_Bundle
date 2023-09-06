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
 
#include <main.h>
#include <string.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"

#include "MAX30003/max30003_dev.h"
#include "MAX30003/max30003_bus.h"
#include "max30003.h"



static const MAX30003_DATA_t NULL_DATA = {
    .byte[0] = 0x00,
    .byte[1] = 0x00,
    .byte[2] = 0x00,
};

uint8_t ecg_spi_read (MAX30003_Object_t *pObj,uint8_t command, uint32_t *data_out)
{
	int rv=0;
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);//CS pin low
	rv=MAX30003_reg_read(&(pObj->Ctx),(uint32_t)command, data_out);
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);	//CS pin high
return rv;
}/* end of ecg_spi_read */

uint8_t ecg_spi_write(MAX30003_Object_t *pObj, uint8_t command, uint32_t data_in)
{
    int rv=0;

	//CS pin low
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
	rv=MAX30003_reg_write(&(pObj->Ctx),(uint32_t)command, data_in);
	//CS pin high
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

	return rv;
}/* end of ecg_spi_write */

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */
int32_t MAX30003_RegDump(MAX30003_Object_t *pObj)
{
	unsigned int all;
	char dbug[100]={0};

	strcpy(dbug,"\r\n[MAX30003]Register map:\r\n");
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//STATUS = 0x01 ===============================================
	ecg_spi_read(pObj,REG_STATUS, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30003] STATUS     = 0x01 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//EN_INT = 0x02 ===============================================
	ecg_spi_read(pObj,REG_EN_INT, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30003] REG_EN_INT     = 0x02 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//EN_INT2  = 0x03 =============================================
	ecg_spi_read(pObj,REG_EN_INT2, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30003] REG_EN_INT2     = 0x03 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//MNGR_INT = 0x04 =============================================
	ecg_spi_read(pObj,REG_MNGR_INT, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30003] REG_MNGR_INT     = 0x04 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//MNGR_DYN = 0x05 =============================================
	ecg_spi_read(pObj,REG_MNGR_DYN, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30003] REG_MNGR_DYN     = 0x05 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//FIFO_RST = 0x0A =============================================
	ecg_spi_read(pObj,REG_FIFO_RST, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30003] REG_FIFO_RST     = 0x0A [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//INFO = 0x0F =========================================
	ecg_spi_read(pObj,REG_INFO, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30003] REG_INFO    = 0x0F [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);


	//CNFG_GEN = 0x10 ====================================================
	max30003_cnfg_gen_t cnfg_gen_reg;
	ecg_spi_read(pObj,REG_CNFG_GEN, (uint32_t*)&cnfg_gen_reg.all);
	sprintf(dbug,"\r\n[MAX30003] REG_CNFG_GEN     = 0x10 [%03X]\r\n",(unsigned int)cnfg_gen_reg.all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_CAL = 0x12 ====================================================
	max30003_cnfg_cal_t cnfg_cal_reg;
	ecg_spi_read(pObj,REG_CNFG_CAL, (uint32_t*)&cnfg_cal_reg.all);
	sprintf(dbug,"\r\n[MAX30003] REG_CNFG_CAL     = 0x12 [%03X]\r\n",(unsigned int)cnfg_cal_reg.all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_EMUX = 0x14 ===================================================
	max30003_cnfg_emux_t cnfg_emux_reg;
	ecg_spi_read(pObj,REG_CNFG_EMUX, (uint32_t*)&cnfg_emux_reg.all);
	sprintf(dbug,"\r\n[MAX30003] REG_CNFG_EMUX     = 0x14 [%03X]\r\n",(unsigned int)cnfg_emux_reg.all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_ECG = 0x15 ====================================================
	max30003_cnfg_ecg_t cnfg_ecg_reg;
	ecg_spi_read(pObj,REG_CNFG_ECG, (uint32_t*)&cnfg_ecg_reg.all);
	sprintf(dbug,"\r\n[MAX30003] REG_CNFG_ECG     = 0x15 [%03X]\r\n",(unsigned int)cnfg_ecg_reg.all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);



	//CNFG_RTOR1 = 0x1D ===========================================
	ecg_spi_read(pObj,REG_CNFG_RTOR1, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30003] REG_CNFG_RTOR1     = 0x1d [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_RTOR2 = 0x1E ===========================================
	ecg_spi_read(pObj,REG_CNFG_RTOR2, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30003] REG_CNFG_RTOR2     = 0x1E [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	return 0;
}

int32_t MAX30003_CAL_InitStart(MAX30003_Object_t *pObj,
		                   uint8_t En_Vcal,
		                   uint8_t Vmode,
                           uint8_t Vmag,
						   uint8_t Fcal,
						   uint16_t Thigh,
                           uint8_t Fifty) {

	max30003_cnfg_cal_t cnfg_cal;

	///< CNFG_CAL
	ecg_spi_read(pObj,REG_CNFG_CAL, (uint32_t*)&cnfg_cal.all);
	cnfg_cal.bit.vmode = Vmode;
	cnfg_cal.bit.vmag  = Vmag;
	cnfg_cal.bit.fcal  = Fcal;
	cnfg_cal.bit.thigh = Thigh;
	cnfg_cal.bit.fifty = Fifty;
	ecg_spi_write(pObj, REG_CNFG_CAL, cnfg_cal.all);
	vTaskDelay(100);

	ecg_spi_read(pObj,REG_CNFG_CAL, (uint32_t*)&cnfg_cal.all);
	cnfg_cal.bit.en_vcal = En_Vcal;
	ecg_spi_write(pObj, REG_CNFG_CAL, cnfg_cal.all);
	vTaskDelay(100);

return MAX30003_OK;
}

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
						   uint8_t Dlpf) {

	max30003_cnfg_emux_t cnfg_emux;
	max30003_cnfg_gen_t  cnfg_gen;
	max30003_status_t    status;
	max30003_mngr_int_t  mngr_int;
	max30003_cnfg_ecg_t  cnfg_ecg;
	uint32_t max30003_timeout;

	ecg_spi_read(pObj, REG_CNFG_EMUX, &cnfg_emux.all);
	cnfg_emux.bit.openp    = Openp;
	cnfg_emux.bit.openn    = Openn;
	cnfg_emux.bit.pol      = Pol;
	cnfg_emux.bit.calp_sel = Calp_sel;
	cnfg_emux.bit.caln_sel = Caln_sel;
	ecg_spi_write(pObj, REG_CNFG_EMUX, cnfg_emux.all);

	ecg_spi_read(pObj, REG_CNFG_GEN,  &cnfg_gen.all);
	cnfg_gen.bit.en_ecg = En_ecg;
	ecg_spi_write(pObj, REG_CNFG_GEN, cnfg_gen.all);

	/* Wait for PLL Lock & References to settle down */
	max30003_timeout = 0;
	do
	{
		ecg_spi_read(pObj, REG_STATUS,  &status.all);
	} while (status.bit.pllint == 1 && max30003_timeout++ <= 1000);

	ecg_spi_read(pObj, REG_MNGR_INT,  &mngr_int.all);
	mngr_int.bit.e_fit = E_fit;
	ecg_spi_write(pObj, REG_MNGR_INT, mngr_int.all);

	ecg_spi_read(pObj, REG_CNFG_ECG,  &cnfg_ecg.all);
	cnfg_ecg.bit.rate = Rate;
	cnfg_ecg.bit.gain = Gain;
	cnfg_ecg.bit.dhpf = Dhpf;
	cnfg_ecg.bit.dlpf = Dlpf;
	ecg_spi_write(pObj, REG_CNFG_ECG, cnfg_ecg.all);

return MAX30003_OK;
}/* end of  MAX30003_ECG_InitStart */

/**
* @brief For MAX30001/3 ONLY
* @brief This function enables the ECG.
* @brief Uses Register CNFG_GEN-0x10.
* @returns 0-if no error.  A non-zero value indicates an error.
*
*/
int32_t MAX30003_Start_ECG(MAX30003_Object_t *pObj) {

	max30003_cnfg_gen_t cnfg_gen;
	max30003_status_t    status;
	uint32_t max30003_timeout;

	ecg_spi_read(pObj, REG_CNFG_GEN,  &cnfg_gen.all);
	cnfg_gen.bit.en_ecg = 0b1;
	ecg_spi_write(pObj, REG_CNFG_GEN, cnfg_gen.all);

	/* Wait for PLL Lock & References to settle down */
	max30003_timeout = 0;
	do
	{
		ecg_spi_read(pObj, REG_STATUS,  &status.all);
	} while (status.bit.pllint == 1 && max30003_timeout++ <= 1000);

return MAX30003_OK;
}/* End of MAX30001_Start_ECG */

/**
* @brief For MAX30001/3 ONLY
* @brief This function disables the ECG.
* @brief Uses Register CNFG_GEN-0x10.
* @returns 0-if no error.  A non-zero value indicates an error.
*
*/
int32_t MAX30003_Stop_ECG(MAX30003_Object_t *pObj) {

	max30003_cnfg_gen_t cnfg_gen;

	ecg_spi_read(pObj, REG_CNFG_GEN,  &cnfg_gen.all);
	cnfg_gen.bit.en_ecg = 0b0;
	ecg_spi_write(pObj, REG_CNFG_GEN, cnfg_gen.all);
	return MAX30003_OK;
}/* End of MAX30003_Stop_ECG */


int32_t MAX30003_readID(MAX30003_Object_t *pObj, uint32_t * pid)
{
	return ecg_spi_read(pObj,REG_INFO, pid);
}/* end of MAX30003_readID */


/**
* @brief This function provides a SYNCH operation.
*  Uses Register SYCNH-0x09. Please refer to the data sheet for
* @brief the details on how to use this.
* @returns 0-if no error.  A non-zero value indicates an error.
*/
int32_t MAX30003_synch(MAX30003_Object_t *pObj)
{
	int32_t ret=MAX30003_OK;
	ecg_spi_write(pObj, REG_SYNCH, 0x000000);
return ret;
}/* end of MAX30031_synch */

/**
 * @brief This function performs a FIFO Reset.
 *  Uses Register FIFO_RST-0x0A. Please refer to the data sheet
 * @brief for the details on how to use this.
 * @returns 0-if no error.
 */
int32_t MAX30003_fifo_rst(MAX30003_Object_t *pObj)
{
	int32_t ret=MAX30003_OK;
	ecg_spi_write(pObj, REG_FIFO_RST, 0x000000);
return ret;
}/* end of MAX30003_fifo_rst */


/**
* @brief This function causes the MAX30003 to reset.
* @return 0-if no error.
*
*/
int32_t MAX30003_sw_rst(MAX30003_Object_t *pObj)
{
	int32_t ret=MAX30003_OK;
	ecg_spi_write(pObj, REG_SW_RST, 0x000000);
return ret;
}/* end of MAX30003_sw_rst */


void ecg_set_en_int(MAX30003_Object_t *pObj,const MAX30003_EN_INT_VALS VALS, const MAX30003_EN_INT_MASKS MASKS)
{
    MAX30003_MSG msg;
    MAX30003_DATA_t newdata;
    uint32_t l_iData = 0;

    msg.command = REG_EN_INT;

    /* get the 24-bit data word for the current and new configurations */
    ecg_spi_read (pObj, msg.command, &l_iData);
	newdata.byte[0] = l_iData & 0xff;
	newdata.byte[1] = (l_iData >> 8) & 0xff;
	newdata.byte[2] = (l_iData >> 16) & 0xff;

    ecg_encode_en_int(VALS, &newdata);

    /* modify the current data with the new data */
    ecg_mask(&newdata, msg.data, MASKS);

    /* write out the message */
    msg.command = REG_EN_INT;//REG_EN_INT);
    msg.data    = newdata;

    l_iData =  msg.data.byte[0];
    l_iData |= msg.data.byte[1] << 8;
    l_iData |= msg.data.byte[2] << 16;
    ecg_spi_write(pObj, msg.command, l_iData);

}/* end of ecg_set_en_int */


void ecg_set_en_int2(MAX30003_Object_t *pObj,const MAX30003_EN_INT_VALS VALS, const MAX30003_EN_INT_MASKS MASKS)
{
    MAX30003_MSG msg;
    MAX30003_DATA_t newdata;
    uint32_t l_iData = 0;

    msg.command = REG_EN_INT2;

    /* get the 24-bit data word for the current and new configurations */
    ecg_spi_read (pObj, msg.command, &l_iData);

	newdata.byte[0] = l_iData & 0xff;
	newdata.byte[1] = (l_iData >> 8) & 0xff;
	newdata.byte[2] = (l_iData >> 16) & 0xff;

    ecg_encode_en_int(VALS, &newdata);

    /* modify the current data with the new data */
    ecg_mask(&newdata, msg.data, MASKS);

    /* write out the message */
    msg.command = REG_EN_INT2;
    msg.data    = newdata;

    l_iData = msg.data.byte[0];
	l_iData |= msg.data.byte[1] << 8;
	l_iData |= msg.data.byte[2] << 16;
	ecg_spi_write(pObj, msg.command, l_iData);
}/* end of ecg_set_en_int2 */

void ecg_get_status(MAX30003_Object_t *pObj, MAX30003_STATUS_VALS *vals)
{
    MAX30003_MSG msg;
    uint32_t l_iData = 0;

    /* build the message to send */
    msg.command = REG_STATUS;
    ecg_spi_read (pObj, msg.command, &l_iData);

	msg.data.byte[0] = l_iData & 0xff;
	msg.data.byte[1] = (l_iData >> 8) & 0xff;
	msg.data.byte[2] = (l_iData >> 16) & 0xff;
    ecg_decode_status(vals, msg.data);
}

void ecg_get_en_int(MAX30003_Object_t *pObj, MAX30003_EN_INT_VALS *vals)
{
    MAX30003_MSG msg;
    uint32_t l_iData = 0;
    
    msg.command = REG_EN_INT;//REG_EN_INT);
	ecg_spi_read (pObj, msg.command, &l_iData);
	msg.data.byte[0] = l_iData & 0xff;
	msg.data.byte[1] = (l_iData >> 8) & 0xff;
	msg.data.byte[2] = (l_iData >> 16) & 0xff;

    ecg_decode_en_int(vals, msg.data);
}

void ecg_get_en_int2(MAX30003_Object_t *pObj, MAX30003_EN_INT_VALS *vals)
{
    MAX30003_MSG msg;
    uint32_t l_iData = 0;
    
    msg.command = REG_EN_INT2;
	ecg_spi_read (pObj, msg.command, &l_iData);

	msg.data.byte[0] = l_iData & 0xff;
	msg.data.byte[1] = (l_iData >> 8) & 0xff;
	msg.data.byte[2] = (l_iData >> 16) & 0xff;

    ecg_decode_en_int(vals, msg.data);
}
void ecg_get_mngr_int(MAX30003_Object_t *pObj, MAX30003_MNGR_INT_VALS *vals)
{
    MAX30003_MSG msg;
    uint32_t l_iData = 0;
    
    msg.command = REG_MNGR_INT;
    msg.data    = NULL_DATA;

    //  ecg_spi_read(&msg);
	ecg_spi_read (pObj, msg.command, &l_iData);

	msg.data.byte[0] = l_iData & 0xff;
	msg.data.byte[1] = (l_iData >> 8) & 0xff;
	msg.data.byte[2] = (l_iData >> 16) & 0xff;

	ecg_decode_mngr_int(vals, msg.data);
}
void ecg_get_mngr_dyn(MAX30003_Object_t *pObj, MAX30003_MNGR_DYN_VALS *vals)
{
    MAX30003_MSG msg;
    uint32_t l_iData = 0;
    
    msg.command = REG_MNGR_DYN;
    msg.data    = NULL_DATA;

	ecg_spi_read (pObj, msg.command, &l_iData);

	msg.data.byte[0] = l_iData & 0xff;
	msg.data.byte[1] = (l_iData >> 8) & 0xff;
	msg.data.byte[2] = (l_iData >> 16) & 0xff;

	ecg_decode_mngr_dyn(vals, msg.data);
}
void ecg_get_cnfg_gen(MAX30003_Object_t *pObj, MAX30003_CNFG_GEN_VALS *vals)
{
	MAX30003_MSG msg;
	uint32_t l_iData = 0;
	
	/* create a (read) command by shifting in the read indicator */
	msg.command = REG_CNFG_GEN;

	/* perform the spi read action */
	ecg_spi_read (pObj, msg.command, &l_iData);

	msg.data.byte[0] = l_iData & 0xff;
	msg.data.byte[1] = (l_iData >> 8) & 0xff;
	msg.data.byte[2] = (l_iData >> 16) & 0xff;

	ecg_decode_cnfg_gen(vals, msg.data);
}
void ecg_get_cnfg_cal(MAX30003_Object_t *pObj, MAX30003_CNFG_CAL_VALS *vals)
{
	MAX30003_MSG msg;
	uint32_t l_iData = 0;
	
	/* create a (read) command by shifting in the read indicator */
	msg.command = REG_CNFG_CAL;
	msg.data		= NULL_DATA;

	/* perform the spi read action */
	ecg_spi_read (pObj, msg.command, &l_iData);

	msg.data.byte[0] = l_iData & 0xff;
	msg.data.byte[1] = (l_iData >> 8) & 0xff;
	msg.data.byte[2] = (l_iData >> 16) & 0xff;

	ecg_decode_cnfg_cal(vals, msg.data);
}
void ecg_get_cnfg_emux(MAX30003_Object_t *pObj, MAX30003_CNFG_EMUX_VALS *vals)
{
	MAX30003_MSG msg;
	uint32_t l_iData = 0;

	/* create a (read) command by shifting in the read indicator */
	msg.command = REG_CNFG_EMUX;
	msg.data		= NULL_DATA;

	/* perform the spi read action */
	ecg_spi_read (pObj, msg.command, &l_iData);

	msg.data.byte[0] = l_iData & 0xff;
	msg.data.byte[1] = (l_iData >> 8) & 0xff;
	msg.data.byte[2] = (l_iData >> 16) & 0xff;

	ecg_decode_cnfg_emux(vals, msg.data);
}
void ecg_get_cnfg_ecg(MAX30003_Object_t *pObj, MAX30003_CNFG_ECG_VALS *vals)
{
	MAX30003_MSG msg;
	uint32_t l_iData = 0;
	
	/* create a (read) command by shifting in the read indicator */
	msg.command = REG_CNFG_ECG;
	msg.data    = NULL_DATA;

	/* perform the spi read action */
	ecg_spi_read (pObj, msg.command, &l_iData);

	msg.data.byte[0] = l_iData & 0xff;
	msg.data.byte[1] = (l_iData >> 8) & 0xff;
	msg.data.byte[2] = (l_iData >> 16) & 0xff;

    ecg_decode_cnfg_ecg(vals, msg.data);
}


void ecg_set_mngr_int(MAX30003_Object_t *pObj,const MAX30003_MNGR_INT_VALS VALS, const MAX30003_MNGR_INT_MASKS MASKS)
{
    MAX30003_MSG msg;
    MAX30003_DATA_t newdata;
    uint32_t l_iData = 0;

    msg.command = REG_MNGR_INT;

    /* get the 24-bit data word for the current and new configurations */
    ecg_spi_read (pObj, msg.command, &l_iData);

	newdata.byte[0] = l_iData & 0xff;
	newdata.byte[1] = (l_iData >> 8) & 0xff;
	newdata.byte[2] = (l_iData >> 16) & 0xff;

    ecg_encode_mngr_int(VALS, &newdata);

    /* modify the current data with the new data */
    ecg_mask(&newdata, msg.data, MASKS);

    /* write out the message */
    msg.command = REG_MNGR_INT;
    msg.data    = newdata;

	l_iData = msg.data.byte[0];
	l_iData |= msg.data.byte[1] << 8;
	l_iData |= msg.data.byte[2] << 16;
	ecg_spi_write(pObj, msg.command, l_iData);
}

void ecg_set_mngr_dyn(MAX30003_Object_t *pObj,const MAX30003_MNGR_DYN_VALS VALS, const MAX30003_MNGR_DYN_MASKS MASKS)
{
    MAX30003_MSG msg;
    MAX30003_DATA_t newdata;
    uint32_t l_iData = 0;

    msg.command = REG_MNGR_DYN;

    /* get the 24-bit data word for the current and new configurations */
   	ecg_spi_read (pObj, msg.command, &l_iData);

   	newdata.byte[0] = l_iData & 0xff;
	newdata.byte[1] = (l_iData >> 8) & 0xff;
	newdata.byte[2] = (l_iData >> 16) & 0xff;

    ecg_encode_mngr_dyn(VALS, &newdata);

    /* modify the current data with the new data */
    ecg_mask(&newdata, msg.data, MASKS);

    /* write out the message */
    msg.command = REG_MNGR_DYN;
    msg.data    = newdata;
    //  ecg_spi_write(&msg);
	l_iData = msg.data.byte[0];
	l_iData |= msg.data.byte[1] << 8;
	l_iData |= msg.data.byte[2] << 16;
	ecg_spi_write(pObj, msg.command, l_iData);
}
void ecg_set_cnfg_gen(MAX30003_Object_t *pObj,const MAX30003_CNFG_GEN_VALS VALS, const MAX30003_CNFG_GEN_MASKS MASKS)
{
	MAX30003_MSG msg;
	MAX30003_DATA_t newdata;
	uint32_t l_iData = 0;

	msg.command = REG_CNFG_GEN;

	/* get the 24-bit data word for the current and new configurations */
	ecg_spi_read (pObj, msg.command, &l_iData);

	newdata.byte[0] = l_iData & 0xff;
	newdata.byte[1] = (l_iData >> 8) & 0xff;
	newdata.byte[2] = (l_iData >> 16) & 0xff;

	ecg_encode_cnfg_gen(VALS, &newdata);

	/* modify the current data with the new data */
    ecg_mask(&newdata, msg.data, MASKS);

	/* write out the message */
    msg.command = REG_CNFG_GEN;
    msg.data    = newdata;
    //  ecg_spi_write(&msg);
	l_iData = msg.data.byte[0];
	l_iData |= msg.data.byte[1] << 8;
	l_iData |= msg.data.byte[2] << 16;
	ecg_spi_write(pObj, msg.command, l_iData);
}
void ecg_set_cnfg_cal(MAX30003_Object_t *pObj,const MAX30003_CNFG_CAL_VALS VALS, const MAX30003_CNFG_CAL_MASKS MASKS)
{
    MAX30003_MSG msg;
    MAX30003_DATA_t newdata;
    uint32_t l_iData = 0;

    msg.command = REG_CNFG_CAL;

    /* get the 24-bit data word for the current and new configurations */
    //  ecg_spi_read(&msg);
	ecg_spi_read (pObj, msg.command, &l_iData);

	newdata.byte[0] = l_iData & 0xff;
	newdata.byte[1] = (l_iData >> 8) & 0xff;
	newdata.byte[2] = (l_iData >> 16) & 0xff;

    ecg_encode_cnfg_cal(VALS, &newdata);

    /* modify the current data with the new data */
    ecg_mask(&newdata, msg.data, MASKS);

    /* write out the message */
    msg.command = REG_CNFG_CAL;
    msg.data    = newdata;
    //  ecg_spi_write(&msg);
	l_iData = msg.data.byte[0];
	l_iData |= msg.data.byte[1] << 8;
	l_iData |= msg.data.byte[2] << 16;
	ecg_spi_write(pObj, msg.command, l_iData);
}
void ecg_set_cnfg_emux(MAX30003_Object_t *pObj,const MAX30003_CNFG_EMUX_VALS VALS, const MAX30003_CNFG_EMUX_MASKS MASKS)
{
    MAX30003_MSG msg;
    MAX30003_DATA_t newdata;
    uint32_t l_iData = 0;

    msg.command = REG_CNFG_EMUX;

    /* get the 24-bit data word for the current and new configurations */
   	ecg_spi_read (pObj, msg.command, &l_iData);

   	newdata.byte[0] = l_iData & 0xff;
	newdata.byte[1] = (l_iData >> 8) & 0xff;
	newdata.byte[2] = (l_iData >> 16) & 0xff;

    ecg_encode_cnfg_emux(VALS, &newdata);

    /* modify the current data with the new data */
    ecg_mask(&newdata, msg.data, MASKS);

    /* write out the message */
    msg.command = REG_CNFG_EMUX;
    msg.data    = newdata;
	l_iData = msg.data.byte[0];
	l_iData |= msg.data.byte[1] << 8;
	l_iData |= msg.data.byte[2] << 16;
	ecg_spi_write(pObj, msg.command, l_iData);
}
void ecg_set_cnfg_ecg(MAX30003_Object_t *pObj,const MAX30003_CNFG_ECG_VALS VALS, const MAX30003_CNFG_ECG_MASKS MASKS)
{
    MAX30003_MSG msg;
    MAX30003_DATA_t newdata;
    uint32_t l_iData = 0;

    msg.command = REG_CNFG_ECG;

    /* get the 24-bit data word for the current and new configurations */
    //  ecg_spi_read(&msg);
   	ecg_spi_read (pObj, msg.command, &l_iData);

   	newdata.byte[0] = l_iData & 0xff;
	newdata.byte[1] = (l_iData >> 8) & 0xff;
	newdata.byte[2] = (l_iData >> 16) & 0xff;

    ecg_encode_cnfg_ecg(VALS, &newdata);

    /* modify the current data with the new data */
    ecg_mask(&newdata, msg.data, MASKS);

    /* write out the message */
    msg.command = REG_CNFG_ECG;
    msg.data    = newdata;
    //  ecg_spi_write(&msg);
	l_iData = msg.data.byte[0];
	l_iData |= msg.data.byte[1] << 8;
	l_iData |= msg.data.byte[2] << 16;
	ecg_spi_write(pObj, msg.command, l_iData);
}
void ecg_set_cnfg_rtor1(MAX30003_Object_t *pObj,const MAX30003_CNFG_RTOR1_VALS VALS, const MAX30003_CNFG_RTOR1_MASKS MASKS)
{
    MAX30003_MSG msg;
    MAX30003_DATA_t newdata;
    uint32_t l_iData = 0;

    msg.command = REG_CNFG_RTOR1;
	
    /* get the 24-bit data word for the current and new configurations */

    ecg_encode_cnfg_rtor1(VALS, &newdata);

    /* modify the current data with the new data */
    ecg_mask(&newdata, msg.data, MASKS);

    /* write out the message */
    msg.command = REG_CNFG_RTOR1;
    msg.data    = newdata;
    //  ecg_spi_write(&msg);
	l_iData = msg.data.byte[0];
	l_iData |= msg.data.byte[1] << 8;
	l_iData |= msg.data.byte[2] << 16;
	ecg_spi_write(pObj, msg.command, l_iData);
}
void ecg_set_cnfg_rtor2(MAX30003_Object_t *pObj,const MAX30003_CNFG_RTOR2_VALS VALS, const MAX30003_CNFG_RTOR2_MASKS MASKS)
{
    MAX30003_MSG msg;
    MAX30003_DATA_t newdata;
    uint32_t l_iData = 0;

    msg.command = REG_CNFG_RTOR2;

    /* get the 24-bit data word for the current and new configurations */
    //  ecg_spi_read(&msg);
   	ecg_spi_read (pObj, msg.command, &l_iData);

	newdata.byte[0] = l_iData & 0xff;
	newdata.byte[1] = (l_iData >> 8) & 0xff;
	newdata.byte[2] = (l_iData >> 16) & 0xff;

    ecg_encode_cnfg_rtor2(VALS, &newdata);

    /* modify the current data with the new data */
    ecg_mask(&newdata, msg.data, MASKS);

    /* write out the message */
    msg.command = REG_CNFG_RTOR2;
    msg.data    = newdata;
    //  ecg_spi_write(&msg);
   	l_iData = msg.data.byte[0];
   	l_iData |= msg.data.byte[1] << 8;
   	l_iData |= msg.data.byte[2] << 16;
   	ecg_spi_write(pObj, msg.command, l_iData);
}


void ecg_decode_status(MAX30003_STATUS_VALS *vals, const MAX30003_DATA_t data)
{
	uint32_t word; /* store the 3x 8-bit data words into a 32-bit number */

	word = 0x00000000;

	/* extract and assign bytes from data word to be endian safe */
	word |= ((uint32_t)(data.byte[2]) << 16);
	word |= ((uint32_t)(data.byte[1]) << 8 );
	word |= ((uint32_t)(data.byte[0]) << 0 );
	
	/* mask in the bits to their respective values as booleans */
	vals->ldoff_nl  = (bool)(word & STATUS_LDOFF_NL);
	vals->ldoff_nh  = (bool)(word & STATUS_LDOFF_NH);
	vals->ldoff_pl  = (bool)(word & STATUS_LDOFF_PL);
	vals->ldoff_ph  = (bool)(word & STATUS_LDOFF_PH);
	vals->pllint    = (bool)(word & STATUS_PLLINT);
	vals->samp      = (bool)(word & STATUS_SAMP);
	vals->rrint     = (bool)(word & STATUS_RRINT);
	vals->lonint    = (bool)(word & STATUS_LONINT);
	vals->dcloffint = (bool)(word & STATUS_DCLOFFINT);
	vals->fstint    = (bool)(word & STATUS_FSTINT);
	vals->eovf      = (bool)(word & STATUS_EOVF);
	vals->eint      = (bool)(word & STATUS_EINT);
}
void ecg_decode_en_int(MAX30003_EN_INT_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint32_t word; /* store the 3x 8-bit data words into a 32-bit number */

	word = 0x00000000;

	/* extract and assign bytes from data word to be endian safe */
	word |= ((uint32_t)(DATA.byte[2]) << 16);
	word |= ((uint32_t)(DATA.byte[1]) << 8 );
	word |= ((uint32_t)(DATA.byte[0]) << 0 );
	
	/* extract and assign bytes from data word to be endian safe */
	vals->intb_type     = (ENINT_INTBTYPE_VAL   )( (word & ENINT_INTB_TYPE) >> 0 );
	vals->en_pllint     = (ENINT_ENPLLINT_VAL   )( (word & ENINT_EN_PLLINT) >> 8 );
	vals->en_samp       = (ENINT_ENSAMP_VAL     )( (word & ENINT_EN_SAMP  ) >> 9 );
	vals->en_rrint      = (ENINT_ENRRINT_VAL    )( (word & ENINT_EN_RRINT ) >> 10);
	vals->en_lonint     = (ENINT_ENLONINT_VAL   )( (word & ENINT_EN_LONINT) >> 11);
	vals->en_dcloffint  = (ENINT_ENDCLOFFINT_VAL)( (word & ENINT_EN_DCLOFFINT) >> 20);
	vals->en_fstint     = (ENINT_ENFSTINT_VAL   )( (word & ENINT_EN_FSTINT) >> 21);
	vals->en_eovf       = (ENINT_ENEOVF_VAL     )( (word & ENINT_EN_EOVF) >> 22);
	vals->en_eint       = (ENINT_ENEINT_VAL     )( (word & ENINT_EN_EINT) >> 23);
}
void ecg_decode_mngr_int(MAX30003_MNGR_INT_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint32_t word; /* store the 3x 8-bit data words into a 32-bit number */

	word = 0x00000000;

	/* extract and assign bytes from data word to be endian safe */
	word |= ((uint32_t)(DATA.byte[2]) << 16);
	word |= ((uint32_t)(DATA.byte[1]) << 8 );
	word |= ((uint32_t)(DATA.byte[0]) << 0 );
	
	/* extract and assign bytes from data word to be endian safe */
	vals->samp_it       = (MNGRINT_SAMPIT_VAL   )( (word & MNGRINT_SAMP_IT  ) >> 0 );
	vals->clr_samp      = (MNGRINT_CLRSAMP_VAL  )( (word & MNGRINT_CLR_SAMP ) >> 2 );
	vals->clr_rrint     = (MNGRINT_CLRRRINT_VAL )( (word & MNGRINT_CLR_RRINT) >> 4 );
	vals->clr_fast      = (MNGRINT_CLRFAST_VAL  )( (word & MNGRINT_CLR_FAST ) >> 6 );
	vals->efit          = (MNGRINT_EFIT_VAL     )( (word & MNGRINT_EFIT     ) >> 19);
}
void ecg_decode_mngr_dyn(MAX30003_MNGR_DYN_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint8_t word; /* store the 3x 8-bit data words into a 8-bit number */

	/* extract and assign bytes from data word to be endian safe */
	word = DATA.byte[2];
	
	/* extract and assign bytes from data word to be endian safe */
	vals->fast_th   = (MNGRDYN_FASTTH_VAL)( (word & MNGRDYN_FAST_TH) >> 0 );
	vals->fast		= (MNGRDYN_FAST_VAL  )( (word & MNGRDYN_FAST   ) >> 6 );
}
void ecg_decode_info(MAX30003_INFO_VALS *vals, const MAX30003_DATA_t DATA)
{
	// TODO check to see if bits other than REV_ID are meaningful
    uint32_t word;
    	
    word = 0x00000000;
    /* extract and assign bytes from data word to be endian safe */
    word |= ((uint32_t)(DATA.byte[2]) << 16);
    word |= ((uint32_t)(DATA.byte[1]) << 8 );
    word |= ((uint32_t)(DATA.byte[0]) << 0 );
    
    vals->_serialnumber = (uint16_t         )( (word & 0x000FFF) >> 0 );
    vals->_partid       = (uint8_t          )( (word & 0x00F000) >> 12);
    vals->rev_id        = (INFO_REV_ID_VAL  )( (word & INFO_REV_ID) >> 16 );
    vals->_verification = (uint8_t          )( (word & 0xF00000) >> 20);
}
void ecg_decode_cnfg_gen(MAX30003_CNFG_GEN_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint32_t word;
	
	word = 0x00000000;
	/* extract and assign bytes from data word to be endian safe */
	word |= ((uint32_t)(DATA.byte[2]) << 16);
	word |= ((uint32_t)(DATA.byte[1]) << 8 );
	word |= ((uint32_t)(DATA.byte[0]) << 0 );
	
	/* shift values from the 24-bit data word into the value struct */
	vals->rbiasn		= (CNFGGEN_RBIASN_VAL     )( (word & CNFGGEN_RBIASN)    >> 0 );
    vals->rbiasp        = (CNFGGEN_RBIASP_VAL     )( (word & CNFGGEN_RBIASP)    >> 1 );
	vals->rbiasv		= (CNFGGEN_RBIASV_VAL     )( (word & CNFGGEN_RBIASV)    >> 2 );
	vals->en_rbias		= (CNFGGEN_EN_RBIAS_VAL   )( (word & CNFGGEN_EN_RBIAS)  >> 4 );
	vals->vth			= (CNFGGEN_DCLOFF_VTH_VAL )( (word & CNFGGEN_VTH)       >> 6 );
	vals->imag			= (CNFGGEN_DCLOFF_IMAG_VAL)( (word & CNFGGEN_IMAG)      >> 8 );
	vals->ipol			= (CNFGGEN_DCLOFF_IPOL_VAL)( (word & CNFGGEN_IPOL)      >> 11);
	vals->en_dcloff		= (CNFGGEN_EN_DCLOFF_VAL  )( (word & CNFGGEN_EN_DCLOFF) >> 12);
	vals->en_ecg		= (CNFGGEN_EN_ECG_VAL     )( (word & CNFGGEN_EN_ECG)    >> 19);
	vals->fmstr			= (CNFGGEN_FMSTR_VAL      )( (word & CNFGGEN_FMSTR)     >> 20);
	vals->en_ulp_lon	= (CNFGGEN_EN_ULP_LON_VAL )( (word & CNFGGEN_EN_ULP_LON)>> 22);
}
void ecg_decode_cnfg_cal(MAX30003_CNFG_CAL_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint32_t word;
	
	word = 0x00000000;
	/* extract and assign bytes from data word to be endian safe */
	word |= ((uint32_t)(DATA.byte[2]) << 16);
	word |= ((uint32_t)(DATA.byte[1]) << 8 );
	word |= ((uint32_t)(DATA.byte[0]) << 0 );
	
	/* shift values from the 24-bit data word into the value struct */
	vals->thigh		= (CNFGCAL_THIGH_VAL  )( (word & CNFGCAL_THIGH  ) >> 0 );
	vals->fifty		= (CNFGCAL_FIFTY_VAL  )( (word & CNFGCAL_FIFTY  ) >> 10);
	vals->fcal		= (CNFGCAL_FCAL_VAL   )( (word & CNFGCAL_FCAL   ) >> 11);
	vals->vmag		= (CNFGCAL_VMAG_VAL   )( (word & CNFGCAL_VMAG   ) >> 20);
	vals->vmode		= (CNFGCAL_VMODE_VAL  )( (word & CNFGCAL_VMODE  ) >> 21);
	vals->en_vcal	= (CNFGCAL_EN_VCAL_VAL)( (word & CNFGCAL_EN_VCAL) >> 22);
}
void ecg_decode_cnfg_emux(MAX30003_CNFG_EMUX_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint32_t word;
	
	/* extract and assign bytes from data word to be endian safe */
	word = (uint32_t)(DATA.byte[2]) << 16;
	
	/* shift values from the 24-bit data word into the value struct */
	vals->caln_sel	= (CNFGEMUX_CALN_SEL_VAL)( (word & CNFGEMUX_CALN_SEL) >> 16);
	vals->calp_sel	= (CNFGEMUX_CALP_SEL_VAL)( (word & CNFGEMUX_CALP_SEL) >> 18);
	vals->openn		= (CNFGEMUX_OPENN_VAL   )( (word & CNFGEMUX_OPENN	) >> 20);
	vals->openp		= (CNFGEMUX_OPENP_VAL   )( (word & CNFGEMUX_OPENP	) >> 21);
	vals->pol		= (CNFGEMUX_POL_VAL		)( (word & CNFGEMUX_POL		) >> 23);
}
void ecg_decode_cnfg_ecg(MAX30003_CNFG_ECG_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint32_t word;
	
	word = 0x00000000;
	/* extract and assign bytes from data word to be endian safe */
	word |= ((uint32_t)(DATA.byte[2]) << 16);
	word |= ((uint32_t)(DATA.byte[1]) << 8 );
	word |= ((uint32_t)(DATA.byte[0]) << 0 );
	
	/* shift values from the 24-bit data word into the value struct */
	vals->dlpf		= (CNFGECG_DLPF_VAL)( (word & CNFGECG_DLPF) >> 12);
	vals->dhpf		= (CNFGECG_DHPF_VAL)( (word & CNFGECG_DHPF) >> 14);
	vals->gain		= (CNFGECG_GAIN_VAL)( (word & CNFGECG_GAIN) >> 16);
	vals->rate		= (CNFGECG_RATE_VAL)( (word & CNFGECG_RATE) >> 22);
}
void ecg_decode_cnfg_rtor1(MAX30003_CNFG_RTOR1_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint32_t word;
	
	word = 0x00000000;
	/* extract and assign bytes from data word to be endian safe */
	word |= ((uint32_t)(DATA.byte[2]) << 16);
	word |= ((uint32_t)(DATA.byte[1]) << 8 );
	word |= ((uint32_t)(DATA.byte[0]) << 0 );
	
	/* shift values from the 24-bit data word into the value struct */
	vals->ptsf		= (CNFGRTOR1_PTSF_VAL   )( (word & CNFGRTOR1_PTSF   ) >> 8);
	vals->pavg		= (CNFGRTOR1_PAVG_VAL   )( (word & CNFGRTOR1_PAVG   ) >> 12);
	vals->en_rtor	= (CNFGRTOR1_EN_RTOR_VAL)( (word & CNFGRTOR1_EN_RTOR) >> 15);
	vals->gain		= (CNFGRTOR1_GAIN_VAL   )( (word & CNFGRTOR1_GAIN	) >> 16);
	vals->wndw		= (CNFGRTOR1_WNDW_VAL   )( (word & CNFGRTOR1_WNDW   ) >> 20);
}
void ecg_decode_cnfg_rtor2(MAX30003_CNFG_RTOR2_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint32_t word;
	
	word = 0x00000000;
	/* extract and assign bytes from data word to be endian safe */
	word |= ((uint32_t)(DATA.byte[2]) << 16);
	word |= ((uint32_t)(DATA.byte[1]) << 8 );
	word |= ((uint32_t)(DATA.byte[0]) << 0 );
	
	/* shift values from the 24-bit data word into the value struct */
	vals->rhsf	= (CNFGRTOR2_RHSF_VAL)( (word & CNFGRTOR2_RHSF) >> 8);
	vals->ravg	= (CNFGRTOR2_RAVG_VAL)( (word & CNFGRTOR2_RAVG) >> 12);
	vals->hoff	= (CNFGRTOR2_HOFF_VAL)( (word & CNFGRTOR2_HOFF) >> 16);
}
void ecg_decode_ecg_fifo(MAX30003_FIFO_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint32_t word;
	
	word = DATA.byte[2] & 0x80 ? 0xFF000000 : 0x00000000;
	/* extract and assign bytes from data word to be endian safe */
	word |= ((uint32_t)(DATA.byte[2]) << 24 );
	word |= ((uint32_t)(DATA.byte[1]) << 16 );
	word |= ((uint32_t)(DATA.byte[0]) << 8  );
	
	/* shift values from the 24-bit data word into the value struct */
	vals->ptag	= (ECGFIFO_PTAG_VAL)( (word & ECGFIFO_PTAG) >> 0);
	vals->etag	= (ECGFIFO_ETAG_VAL)( (word & ECGFIFO_ETAG) >> 3);
	vals->data	= (ECGFIFO_DATA_VAL)( (int32_t)(word & ECGFIFO_DATA) >> 6);
}
void ecg_decode_rtor(MAX30003_RTOR_VALS *vals, const MAX30003_DATA_t DATA)
{
	uint32_t word;
	
	word = 0x00000000;
	/* extract and assign bytes from data word to be endian safe */
	word |= ((uint32_t)(DATA.byte[2]) << 16);
	word |= ((uint32_t)(DATA.byte[1]) << 8 );
	word |= ((uint32_t)(DATA.byte[0]) << 0 );
	
	/* shift values from the 24-bit data word into the value struct */
	vals->data	= (RTOR_DATA_VAL)( (word & RTOR_DATA) >> 10);
}

void ecg_encode_en_int(const MAX30003_EN_INT_VALS VALS, MAX30003_DATA_t *data)
{
    uint32_t word;

    word = 0x00000000;

    word |= (uint32_t)VALS.intb_type      << 0;
    word |= (uint32_t)VALS.en_pllint      << 8;
    word |= (uint32_t)VALS.en_samp        << 9;
    word |= (uint32_t)VALS.en_rrint       << 10;
    word |= (uint32_t)VALS.en_lonint      << 11;
    word |= (uint32_t)VALS.en_dcloffint   << 20;
    word |= (uint32_t)VALS.en_fstint      << 21;
    word |= (uint32_t)VALS.en_eovf        << 22;
    word |= (uint32_t)VALS.en_eint        << 23;

    data->byte[0] = (uint8_t)( (word & 0x000000FF) >> 0 );
    data->byte[1] = (uint8_t)( (word & 0x0000FF00) >> 8 );
    data->byte[2] = (uint8_t)( (word & 0x00FF0000) >> 16);
}
void ecg_encode_mngr_int(const MAX30003_MNGR_INT_VALS VALS, MAX30003_DATA_t *data)
{
	uint32_t word;

	word = 0x00000000;

	word |= VALS.samp_it	<< 0;
	word |= VALS.clr_samp	<< 2;
	word |= VALS.clr_rrint	<< 4;
	word |= VALS.clr_fast	<< 6;
	word |= VALS.efit		<< 19;
	
	data->byte[0] = (uint8_t)( (word >> 0  ) & ( 0x00FFFFFF >> 16 ) );
	data->byte[1] = (uint8_t)( (word >> 8  ) & ( 0x00FFFFFF >> 8  ) );
	data->byte[2] = (uint8_t)( (word >> 16  ) & ( 0x00FFFFFF >> 0 ) );
}
void ecg_encode_mngr_dyn(const MAX30003_MNGR_DYN_VALS VALS, MAX30003_DATA_t *data)
{
	uint8_t word;

	word = 0x00;

	word |= VALS.fast_th	<< 0;
	word |= VALS.fast		<< 6;
	
	data->byte[0] = 0x00;
	data->byte[1] = 0x00;
	data->byte[2] = word;
}
void ecg_encode_cnfg_gen(const MAX30003_CNFG_GEN_VALS VALS, MAX30003_DATA_t *data)
{
	uint32_t word;
	
	word = 0x00000000;

	/* shift values into the 24-bit data word */
	word |= VALS.rbiasn        << 0;
	word |= VALS.rbiasp        << 1;
	word |= VALS.rbiasv        << 2;
	word |= VALS.en_rbias      << 4;
	word |= VALS.vth           << 6;
	word |= VALS.imag          << 8;
	word |= VALS.ipol          << 11;
	word |= VALS.en_dcloff     << 12;
	word |= VALS.en_ecg        << 19;
	word |= VALS.fmstr         << 20;
	word |= VALS.en_ulp_lon    << 22;

	/* extract and assign bytes from data word to be endian safe */
	data->byte[0] = (uint8_t)( (word >> 0  ) & ( 0x00FFFFFF >> 16 ) );
	data->byte[1] = (uint8_t)( (word >> 8  ) & ( 0x00FFFFFF >> 8  ) );
	data->byte[2] = (uint8_t)( (word >> 16 ) & ( 0x00FFFFFF >> 0  ) );
}
void ecg_encode_cnfg_cal(const MAX30003_CNFG_CAL_VALS VALS, MAX30003_DATA_t *data)
{
	uint32_t word;
	
	word = 0x00000000;

	/* shift values into the 24-bit data word */
	word |= VALS.thigh		<< 0;
	word |= VALS.fifty      << 10;
	word |= VALS.fcal		<< 11;
	word |= VALS.vmag		<< 20;
	word |= VALS.vmode      << 21;
	word |= VALS.en_vcal	<< 22;

	/* extract and assign bytes from data word to be endian safe */
	data->byte[0] = (uint8_t)( (word >> 0  ) & ( 0x00FFFFFF >> 16 ) );
	data->byte[1] = (uint8_t)( (word >> 8  ) & ( 0x00FFFFFF >> 8  ) );
	data->byte[2] = (uint8_t)( (word >> 16 ) & ( 0x00FFFFFF >> 0  ) );
}
void ecg_encode_cnfg_emux(const MAX30003_CNFG_EMUX_VALS VALS, MAX30003_DATA_t *data)
{
	uint32_t word;
	
	word = 0x00000000;

	/* shift values into the 24-bit data word */
	word |= VALS.caln_sel	<< 16;
	word |= VALS.calp_sel	<< 18;
	word |= VALS.openn		<< 20;
	word |= VALS.openp		<< 21;
	word |= VALS.pol		<< 23;

	/* extract and assign bytes from data word to be endian safe */
	data->byte[0] = (uint8_t)( (word >> 0  ) & ( 0x00FFFFFF >> 16 ) );
	data->byte[1] = (uint8_t)( (word >> 8  ) & ( 0x00FFFFFF >> 8  ) );
	data->byte[2] = (uint8_t)( (word >> 16 ) & ( 0x00FFFFFF >> 0  ) );
}
void ecg_encode_cnfg_ecg(const MAX30003_CNFG_ECG_VALS VALS, MAX30003_DATA_t *data)
{
	uint32_t word;
	
	word = 0x00000000;

	/* shift values into the 24-bit data word */
	word |= VALS.dlpf	<< 12;
	word |= VALS.dhpf	<< 14;
	word |= VALS.gain	<< 16;
	word |= VALS.rate	<< 22;

	/* extract and assign bytes from data word to be endian safe */
	data->byte[0] = (uint8_t)( (word >> 0  ) & ( 0x00FFFFFF >> 16 ) );
	data->byte[1] = (uint8_t)( (word >> 8  ) & ( 0x00FFFFFF >> 8  ) );
	data->byte[2] = (uint8_t)( (word >> 16 ) & ( 0x00FFFFFF >> 0  ) );
}
void ecg_encode_cnfg_rtor1(const MAX30003_CNFG_RTOR1_VALS VALS, MAX30003_DATA_t *data)
{
	uint32_t word;
	
	word = 0x00000000;

	/* shift values into the 24-bit data word */
	word |= VALS.ptsf		<< 8;
	word |= VALS.pavg		<< 12;
	word |= VALS.en_rtor	<< 15;
	word |= VALS.gain		<< 16;
	word |= VALS.wndw		<< 20;

	/* extract and assign bytes from data word to be endian safe */
	data->byte[0] = (uint8_t)( (word >> 0  ) & ( 0x00FFFFFF >> 16 ) );
	data->byte[1] = (uint8_t)( (word >> 8  ) & ( 0x00FFFFFF >> 8  ) );
	data->byte[2] = (uint8_t)( (word >> 16 ) & ( 0x00FFFFFF >> 0  ) );
}
void ecg_encode_cnfg_rtor2(const MAX30003_CNFG_RTOR2_VALS VALS, MAX30003_DATA_t *data)
{
	uint32_t word;
	
	word = 0x00000000;

	/* shift values into the 24-bit data word */
	word |= VALS.rhsf	<< 8;
	word |= VALS.ravg	<< 12;
	word |= VALS.hoff	<< 16;

	/* extract and assign bytes from data word to be endian safe */
	data->byte[0] = (uint8_t)( (word >> 0  ) & ( 0x00FFFFFF >> 16 ) );
	data->byte[1] = (uint8_t)( (word >> 8  ) & ( 0x00FFFFFF >> 8  ) );
	data->byte[2] = (uint8_t)( (word >> 16 ) & ( 0x00FFFFFF >> 0  ) );	
}




#if 0
void ecg_get_sample(MAX30003_Object_t *pObj,MAX30003_FIFO_VALS *vals)
{
	MAX30003_MSG msg;
	uint32_t l_iData = 0;
	
	/* create a (read) command by shifting in the read indicator */
	msg.command = REG_ECG_FIFO);

	/* perform the spi read action */
	//  ecg_spi_read(&msg);
	MAX30003_reg_read(&max30003_obj_0.Ctx,(uint32_t)msg.command, &l_iData);

	msg.data.byte[0] = l_iData & 0xff;
	msg.data.byte[1] = (l_iData >> 8) & 0xff;
	msg.data.byte[2] = (l_iData >> 16) & 0xff;

	ecg_decode_ecg_fifo(vals, msg.data);

}
#endif

//int32_t ecg_get_sample_burst(ECG_SAMPLE_t *log, const uint16_t SIZE)
//{
////   bool eof;
////   int  sample;
////   uint16_t step;              /* unit-less time increment */
////
////   MAX30003_MSG msg;
////   MAX30003_FIFO_VALS vals;
////
////   eof = false;
////   step = 0x0000;
////
////   /* start the burst transfer, but hold CSB low */
////   ECG_BUF_O[ECG_CMND_POS] = ECG_REG_R(REG_ECG_FIFO_BURST);
////
////   /* start collecting samples from FIFO */
////   gpio_set_pin_level(MOD_CS, false);
////   ecg_spi_msg.size = 1;
////   spi_m_sync_transfer(&SPI_MOD, &ecg_spi_msg);
////
////   ecg_spi_msg.size = ECG_BUF_SZ;
////   ECG_BUF_O[ECG_CMND_POS] = 0x00;
////
////   /* evaluate and store sample, take action if error */
////   do {
////       /* get and process samples */
////       if (!(step % 4)) {
////           spi_m_sync_transfer(&SPI_MOD, &ecg_spi_msg);
////       }
////
////       sample = (step % 4)*ECG_DATA_SZ;
////       msg.data.byte[0] = ECG_BUF_I[sample + 2];
////       msg.data.byte[1] = ECG_BUF_I[sample + 1];
////       msg.data.byte[2] = ECG_BUF_I[sample + 0];
////
////       ecg_decode_ecg_fifo(&vals, msg.data);
////
////       switch (vals.etag) {
////           case ETAG_VALID_EOF :
////           case ETAG_FAST_EOF  :
////               eof = true; /* exit, but save the sample as a valid sample */
////           case ETAG_VALID     :
////           case ETAG_FAST      :
////               /* format and store the sample */
////               log[step].tag  = vals.etag;
////               log[step].step = step;
////               log[step].data = vals.data;
////
////               /* increment, clear, and get next sample */
////               step++;
////               break;
////
////           case ETAG_FIFO_OVERFLOW :
////               gpio_set_pin_level(MOD_CS, true);
////               ecg_fifo_reset(); /* or synch */
////           case ETAG_FIFO_EMPTY    :
////               eof = true;
////               break;
////           default :
////               gpio_set_pin_level(MOD_CS, true);
////               ecg_synch();
////               break; /* TODO error handling */
////       }
////   } while (!eof && step < SIZE);
////
////   /* done sampling spi */
////   gpio_set_pin_level(MOD_CS, true);
////
////   return step;
//}

void ecg_fifo_reset(MAX30003_Object_t *pObj)
{
    MAX30003_MSG msg;
    uint32_t l_iData = 0;

    msg.command = REG_FIFO_RST;
    msg.data    = NULL_DATA;

    //  ecg_spi_write(&msg);
	l_iData = msg.data.byte[0];
	l_iData |= msg.data.byte[1] << 8;
	l_iData |= msg.data.byte[2] << 16;
	ecg_spi_write(pObj, msg.command, l_iData);
}

void ecg_synch(MAX30003_Object_t *pObj)
{
    MAX30003_MSG msg;
    uint32_t l_iData = 0;

    msg.command = REG_SYNCH;
    msg.data    = NULL_DATA;

	l_iData = msg.data.byte[0];
	l_iData |= msg.data.byte[1] << 8;
	l_iData |= msg.data.byte[2] << 16;
	ecg_spi_write(pObj, msg.command, l_iData);
}

void ecg_mask(MAX30003_DATA_t *new_vals, const MAX30003_DATA_t OLD_VALS, const uint32_t MASKS)
{
    uint32_t old_word;
    uint32_t new_word;

    /* extract byte values as 32-bit words */
    old_word = ((uint32_t)OLD_VALS.byte[2] << 16) | ((uint32_t)OLD_VALS.byte[1] << 8) | ((uint32_t)OLD_VALS.byte[0] << 0);
    new_word = ((uint32_t)new_vals->byte[2] << 16) | ((uint32_t)new_vals->byte[1] << 8) | ((uint32_t)new_vals->byte[0] << 0);

    /* mask out old values and mask in new values */
    new_word = (new_word & MASKS) | (old_word & ~MASKS);

    /* build new data array */
    new_vals->byte[0] = (uint8_t)(new_word >> 0);
    new_vals->byte[1] = (uint8_t)(new_word >> 8);
    new_vals->byte[2] = (uint8_t)(new_word >> 16);
}   


