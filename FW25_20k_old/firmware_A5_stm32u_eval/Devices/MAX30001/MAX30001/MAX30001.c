/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */
#include <stdio.h>
#include <string.h>

#include <main.h>
#include "MAX30001/MAX30001_dev.h"
#include "MAX30001/MAX30001_bus.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include <freertos.h>
#include "max30001.h"
#include <task.h>

#include "auxcmd.h"//logs



  // Data
  uint32_t max30001_ECG_FIFO_buffer[32]; ///< (303 for internal test)
  uint32_t max30001_BIOZ_FIFO_buffer[8]; ///< (303 for internal test)

  uint32_t max30001_PACE[18]; ///< Pace Data 0-5

  uint32_t max30001_RtoR_data; ///< This holds the RtoR data

  uint32_t max30001_DCLeadOff; ///< This holds the LeadOff data, Last 4 bits give
                               ///< the status, BIT3=LOFF_PH, BIT2=LOFF_PL,
                               ///< BIT1=LOFF_NH, BIT0=LOFF_NL
                               ///< 8th and 9th bits tell Lead off is due to ECG or BIOZ.
                               ///< 0b01 = ECG Lead Off and 0b10 = BIOZ Lead off

  uint32_t max30001_ACLeadOff; ///< This gives the state of the BIOZ AC Lead Off
                               ///< state.  BIT 1 = BOVER,   BIT 0 = BUNDR

  uint32_t max30001_bcgmon; ///< This holds the BCGMON data, BIT 1 = BCGMP, BIT0 =
                            ///< BCGMN

  uint32_t max30001_LeadOn; ///< This holds the LeadOn data, BIT1 = BIOZ Lead ON,
                            ///< BIT0 = ECG Lead ON, BIT8= Lead On Status Bit

  uint32_t max30001_timeout; ///< If the PLL does not respond, timeout and get out.

  max30001_bledata_t hspValMax30001; // R2R, FMSTR

  max30001_status_t global_status;


  /// callback function when interrupt data is available
  PtrFunction onDataAvailableCallback;
  char dbug[100]={0};


//******************************************************************************
void MAX30001_ReadHeartrateData(max30001_bledata_t *_hspValMax30001) {
  _hspValMax30001->R2R = hspValMax30001.R2R;
  _hspValMax30001->fmstr = hspValMax30001.fmstr;
}

//******************************************************************************
void MAX30001_onDataAvailable(PtrFunction _onDataAvailable) {
  onDataAvailableCallback = _onDataAvailable;
}

//******************************************************************************
void MAX30001_dataAvailable(uint32_t id, uint32_t *buffer, uint32_t length,void * p) {
  if (onDataAvailableCallback != NULL) {
    (*onDataAvailableCallback)(id, buffer, length,p);
  }
}

static uint8_t ecg_spi_read (MAX30001_Object_t *pObj,uint8_t command, uint32_t *data_out)
{
	int rv=0;
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);//CS pin low
	rv=MAX30001_reg_read(&(pObj->Ctx),(uint32_t)command, data_out);
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);	//CS pin high
return rv;
}/* end of ecg_spi_read */

static uint8_t ecg_spi_write(MAX30001_Object_t *pObj, uint8_t command, uint32_t data_in)
{
    int rv=0;

	//CS pin low
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
	rv=MAX30001_reg_write(&(pObj->Ctx),(uint32_t)command, data_in);
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
int32_t MAX30001_RegDump(MAX30001_Object_t *pObj)
{
	unsigned int all;

	strcpy(dbug,"\r\n[MAX30001]Register map:\r\n");
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//STATUS = 0x01 ===============================================
	ecg_spi_read(pObj,STATUS,(uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] STATUS     = 0x01 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//EN_INT = 0x02 ===============================================
	ecg_spi_read(pObj,EN_INT, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] EN_INT     = 0x02 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//EN_INT2  = 0x03 =============================================
	ecg_spi_read(pObj,EN_INT2, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] EN_INT2    = 0x03 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//MNGR_INT = 0x04 =============================================
	ecg_spi_read(pObj,MNGR_INT, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] MNGR_INT   = 0x04 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//MNGR_DYN = 0x05 =============================================
	ecg_spi_read(pObj,MNGR_DYN, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] MNGR_DYN   = 0x05 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//SW_RST = 0x08 ===============================================
	ecg_spi_read(pObj,SW_RST, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] SW_RST     = 0x08 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//SYNCH = 0x09 ================================================
	ecg_spi_read(pObj,SYNCH, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] SYNCH      = 0x09 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//FIFO_RST = 0x0A =============================================
	ecg_spi_read(pObj,FIFO_RST, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] FIFO_RST   = 0x0A [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//INFO = 0x0F =========================================
	ecg_spi_read(pObj,INFO, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] INFO       = 0x0F [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);


	//CNFG_GEN = 0x10 ====================================================
	max30001_cnfg_gen_t cnfg_gen_reg;
	ecg_spi_read(pObj,CNFG_GEN,  (uint32_t*)&cnfg_gen_reg.all);
	sprintf(dbug,"\r\n[MAX30001] CNFG_GEN   = 0x10 [%03X]\r\n", (unsigned int)cnfg_gen_reg.all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_CAL = 0x12 ====================================================
	max30001_cnfg_cal_t cnfg_cal_reg;
	ecg_spi_read(pObj,CNFG_CAL,  (uint32_t*)&cnfg_cal_reg.all);
	sprintf(dbug,"\r\n[MAX30001] CNFG_CAL   = 0x12 [%03X]\r\n",(unsigned int)cnfg_cal_reg.all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_EMUX = 0x14 ===================================================
	max30001_cnfg_emux_t cnfg_emux_reg;
	ecg_spi_read(pObj,CNFG_EMUX,  (uint32_t*)&cnfg_emux_reg.all);
	sprintf(dbug,"\r\n[MAX30001] CNFG_EMUX  = 0x14 [%03X]\r\n",(unsigned int)cnfg_emux_reg.all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_ECG = 0x15 ====================================================
	max30001_cnfg_ecg_t cnfg_ecg_reg;
	ecg_spi_read(pObj,CNFG_ECG,  (uint32_t*)&cnfg_ecg_reg.all);
	sprintf(dbug,"\r\n[MAX30001] CNFG_ECG   = 0x15 [%03X]\r\n",(unsigned int)cnfg_ecg_reg.all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);



	//CNFG_BMUX = 0x17 ===========================================
	ecg_spi_read(pObj,CNFG_BMUX, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] CNFG_BMUX  = 0x17 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_BIOZ = 0x18 ===========================================
	ecg_spi_read(pObj,CNFG_BIOZ, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] CNFG_BIOZ  = 0x18 [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_PACE = 0x1A ===========================================
	ecg_spi_read(pObj,CNFG_PACE, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] CNFG_PACE  = 0x1A [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_RTOR1 = 0x1D ===========================================
	ecg_spi_read(pObj,CNFG_RTOR1, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] CNFG_RTOR1 = 0x1D [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	//CNFG_RTOR2 = 0x1E ===========================================
	ecg_spi_read(pObj,CNFG_RTOR2, (uint32_t*)&all);
	sprintf(dbug,"\r\n[MAX30001] CNFG_RTOR2 = 0x1E [%03X]\r\n",all);
	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	return 0;
}
/**
* @brief This function causes the MAX30001 to reset.
* @return 0-if no error.
*
*/
int32_t MAX30001_sw_rst(MAX30001_Object_t *pObj)
{
	int32_t ret=MAX30001_OK;
	ecg_spi_write(pObj, SW_RST, 0x000000);
return ret;
}/* end of MAX30001_sw_rst */

/**
* @brief This function used to read sensor id value
* @returns 0-if no error.  A non-zero value indicates an error.
*/
int32_t MAX30001_readID(MAX30001_Object_t *pObj, uint32_t * pid)
{
	return ecg_spi_read(pObj,INFO, pid);
}/* end of MAX30001_readID */

/**
* @brief This function provides a SYNCH operation.
*  Uses Register SYCNH-0x09. Please refer to the data sheet for
* @brief the details on how to use this.
* @returns 0-if no error.  A non-zero value indicates an error.
*/
int32_t MAX30001_synch(MAX30001_Object_t *pObj)
{
	int32_t ret=MAX30001_OK;
	ecg_spi_write(pObj, SYNCH, 0x000000);
return ret;
}/* end of MAX30001_synch */

/**
 * @brief This function performs a FIFO Reset.
 *  Uses Register FIFO_RST-0x0A. Please refer to the data sheet
 * @brief for the details on how to use this.
 * @returns 0-if no error.
 */
int32_t MAX30001_fifo_rst(MAX30001_Object_t *pObj)
{
	int32_t ret=MAX30001_OK;
	ecg_spi_write(pObj, FIFO_RST, 0x000000);
return ret;
}/* end of MAX30001_fifo_rst */

//==============================================================================
//CAL
//==============================================================================
/**
 * @brief This function uses sets up the calibration signal
 * internally. If it is desired to use the internal signal, then
 * @brief this function must be called and the registers set,
 *  prior to setting the CALP_SEL and CALN_SEL in the ECG_InitStart
 * @brief and BIOZ_InitStart functions.
 * @brief Uses Register: CNFG_CAL-0x12
 * @param En_Vcal: Calibration Source (VCALP and VCALN) Enable
 * @param Vmode:   Calibration Source Mode Selection
 * @param Vmag:    Calibration Source Magnitude Selection (VMAG)
 * @param Fcal:    Calibration Source Frequency Selection (FCAL)
 * @param Thigh:   Calibration Source Time High Selection
 * @param Fifty:   Calibration Source Duty Cycle Mode Selection
 * @returns 0-if no error.  A non-zero value indicates an error.
 */
int32_t MAX30001_CAL_InitStart(MAX30001_Object_t *pObj,
		                   uint8_t En_Vcal,
		                   uint8_t Vmode,
                           uint8_t Vmag,
						   uint8_t Fcal,
						   uint16_t Thigh,
                           uint8_t Fifty) {

	max30001_cnfg_cal_t cnfg_cal;

	///< CNFG_CAL
	ecg_spi_read(pObj,CNFG_CAL, (uint32_t*)&cnfg_cal.all);

	cnfg_cal.bit.vmode = Vmode;
	cnfg_cal.bit.vmag  = Vmag;
	cnfg_cal.bit.fcal  = Fcal;
	cnfg_cal.bit.thigh = Thigh;
	cnfg_cal.bit.fifty = Fifty;

	ecg_spi_write(pObj, CNFG_CAL, cnfg_cal.all);
	vTaskDelay(100);

	ecg_spi_read(pObj, CNFG_CAL, (uint32_t*)&cnfg_cal.all);
	cnfg_cal.bit.en_vcal = En_Vcal;
	ecg_spi_write(pObj, CNFG_CAL, cnfg_cal.all);
	vTaskDelay(100);

 return 0;
}

/**
 * @brief For MAX30001/2 ONLY
 * @brief This function sets up the MAX30001 for BIOZ measurement.
 * @brief Registers used: MNGR_INT-0x04, CNFG_GEN-0X10, CNFG_BMUX-0x17,CNFG_BIOZ-0x18.
 * @param En_bioz: BIOZ Channel Enable <CNFG_GEN Register>
 * @param Openp: Open the BIP Input Switch <CNFG_BMUX Register>
 * @param Openn: Open the BIN Input Switch <CNFG_BMUX Register>
 * @param Calp_sel: BIP Calibration Selection <CNFG_BMUX Register>
 * @param Caln_sel: BIN Calibration Selection <CNFG_BMUX Register>
 * @param CG_mode:  BIOZ Current Generator Mode Selection <CNFG_BMUX Register>
 * @param B_fit:  BIOZ FIFO Interrupt Threshold (issues BINT based on number of unread FIFO records) <MNGR_INT Register>
 * @param Rate: BIOZ Data Rate <CNFG_BIOZ Register>
 * @param Ahpf: BIOZ/PACE Channel Analog High Pass Filter Cutoff Frequency and Bypass <CNFG_BIOZ Register>
 * @param Ext_rbias:  External Resistor Bias Enable <CNFG_BIOZ Register>
 * @param Gain: BIOZ Channel Gain Setting <CNFG_BIOZ Register>
 * @param Dhpf: BIOZ Channel Digital High Pass Filter Cutoff Frequency <CNFG_BIOZ Register>
 * @param Dlpf:  BIOZ Channel Digital Low Pass Filter Cutoff Frequency <CNFG_BIOZ Register>
 * @param Fcgen:  BIOZ Current Generator Modulation Frequency <CNFG_BIOZ Register>
 * @param Cgmon:  BIOZ Current Generator Monitor <CNFG_BIOZ Register>
 * @param Cgmag:  BIOZ Current Generator Magnitude <CNFG_BIOZ Register>
 * @param Phoff: BIOZ Current Generator Modulation Phase Offset <CNFG_BIOZ Register>
 * @returns 0-if no error.  A non-zero value indicates an error.
 *
 */

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
							uint8_t Phoff) {

	max30001_cnfg_bmux_t cnfg_bmux;
	max30001_cnfg_gen_t  cnfg_gen;
	max30001_status_t    status;
	max30001_mngr_int_t  mngr_int;
	max30001_cnfg_bioz_t cnfg_bioz;
	uint32_t max30001_timeout;


	ecg_spi_read(pObj, CNFG_BMUX, &cnfg_bmux.all);
	cnfg_bmux.bit.openp    = Openp;
	cnfg_bmux.bit.openn    = Openn;
	cnfg_bmux.bit.calp_sel = Calp_sel;
	cnfg_bmux.bit.caln_sel = Caln_sel;
	cnfg_bmux.bit.cg_mode  = CG_mode;
	ecg_spi_write(pObj, CNFG_BMUX, cnfg_bmux.all);

	ecg_spi_read(pObj, CNFG_GEN,  &cnfg_gen.all);
	cnfg_gen.bit.en_bioz = En_bioz;
	ecg_spi_write(pObj, CNFG_GEN, cnfg_gen.all);

	/* Wait for PLL Lock & References to settle down */
	max30001_timeout = 0;
	do
	{
		ecg_spi_read(pObj, STATUS,  &status.all);
	} while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);

    /* MNGR_INT*/
	ecg_spi_read(pObj, MNGR_INT,  &mngr_int.all);
	mngr_int.bit.b_fit = B_fit;
	ecg_spi_write(pObj, MNGR_INT, mngr_int.all);


	/* CNFG_BIOZ */
	ecg_spi_read(pObj, CNFG_BIOZ, &cnfg_bioz.all);
	cnfg_bioz.bit.rate      = Rate;
	cnfg_bioz.bit.ahpf      = Ahpf;
	cnfg_bioz.bit.ext_rbias = Ext_rbias;
	cnfg_bioz.bit.gain      = Gain;
	cnfg_bioz.bit.dhpf      = Dhpf;
	cnfg_bioz.bit.dlpf      = Dlpf;
	cnfg_bioz.bit.fcgen     = Fcgen;
	cnfg_bioz.bit.cgmon     = Cgmon;
	cnfg_bioz.bit.cgmag     = Cgmag;
	cnfg_bioz.bit.phoff     = Phoff;
	ecg_spi_write(pObj, CNFG_BIOZ, cnfg_bioz.all);


return MAX30001_OK;
}/* End of MAX30001_BIOZ_InitStart */

/**
* @brief For MAX30001/3 ONLY
* @brief This function enables the BIOZ.
* @brief Uses Register CNFG_GEN-0x10.
* @returns 0-if no error.  A non-zero value indicates an error.
*
*/
int MAX30001_Start_BIOZ(MAX30001_Object_t *pObj) {

	max30001_cnfg_gen_t cnfg_gen;
//	max30001_status_t    status;
//	uint32_t max30001_timeout;


	ecg_spi_read(pObj, CNFG_GEN,  &cnfg_gen.all);
	cnfg_gen.bit.en_bioz = 0b1;
	ecg_spi_write(pObj, CNFG_GEN, cnfg_gen.all);

//	if(0==cnfg_gen.bit.en_bioz)
//	{
//		/* Wait for PLL Lock & References to settle down */
//		max30001_timeout = 0;
//		do
//		{
//			ecg_spi_read(pObj, STATUS,  &status.all);
//		} while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);
//
//		ecg_spi_write(pObj, FIFO_RST, 0x000000);
//		ecg_spi_write(pObj, SYNCH, 0x000000);
//	}

  return 0;
}

/**
* @brief For MAX30001/3 ONLY
* @brief This function disables the BIOZ.
* @brief Uses Register CNFG_GEN-0x10.
* @returns 0-if no error.  A non-zero value indicates an error.
*
*/
int MAX30001_Stop_BIOZ(MAX30001_Object_t *pObj) {

	max30001_cnfg_gen_t cnfg_gen;

	ecg_spi_read(pObj, CNFG_GEN,  &cnfg_gen.all);
	cnfg_gen.bit.en_bioz = 0b0; // Stop BIOZ
	ecg_spi_write(pObj, CNFG_GEN, cnfg_gen.all);

return MAX30001_OK;
}/* End of MAX30001_Stop_BIOZ */


/**
 * @brief For MAX30001/3 ONLY
 * @brief This function sets up the MAX30001 for the ECG measurements.
 * @brief Registers used:  CNFG_EMUX, CNFG_GEN, MNGR_INT, CNFG_ECG.
 * @param En_ecg: ECG Channel Enable <CNFG_GEN register bits>
 * @param Openp: Open the ECGN Input Switch (most often used for testing
 *  and calibration studies) <CNFG_EMUX register bits>
 * @param Openn: Open the ECGN Input Switch (most often used for testing
 * and calibration studies) <CNFG_EMUX register bits>
 * @param Calp_sel: ECGP Calibration Selection <CNFG_EMUX register bits>
 * @param Caln_sel: ECGN Calibration Selection <CNFG_EMUX register bits>
 * @param E_fit: ECG FIFO Interrupt Threshold (issues EINT based on number
 *  of unread FIFO records) <CNFG_GEN register bits>
 * @param Clr_rrint: RTOR R Detect Interrupt (RRINT) Clear Behavior
 * <CNFG_GEN register bits>
 * @param Rate: ECG Data Rate
 * @param Gain: ECG Channel Gain Setting
 * @param Dhpf: ECG Channel Digital High Pass Filter Cutoff Frequency
 * @param Dlpf:  ECG Channel Digital Low Pass Filter Cutoff Frequency
 * @returns 0-if no error.  A non-zero value indicates an error.
 *
 */
int32_t MAX30001_ECG_InitStart(MAX30001_Object_t *pObj,
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

	max30001_cnfg_emux_t cnfg_emux;
	max30001_cnfg_gen_t  cnfg_gen;
	max30001_status_t    status;
	max30001_mngr_int_t  mngr_int;
	max30001_cnfg_ecg_t  cnfg_ecg;
	uint32_t max30001_timeout;


	ecg_spi_read(pObj, CNFG_EMUX, &cnfg_emux.all);
	cnfg_emux.bit.openp    = Openp;
	cnfg_emux.bit.openn    = Openn;
	cnfg_emux.bit.pol      = Pol;
	cnfg_emux.bit.calp_sel = Calp_sel;
	cnfg_emux.bit.caln_sel = Caln_sel;
	ecg_spi_write(pObj, CNFG_EMUX, cnfg_emux.all);

	ecg_spi_read(pObj, CNFG_GEN,  &cnfg_gen.all);
	cnfg_gen.bit.en_ecg = En_ecg;
	ecg_spi_write(pObj, CNFG_GEN, cnfg_gen.all);

	/* Wait for PLL Lock & References to settle down */
	max30001_timeout = 0;
	do
	{
		ecg_spi_read(pObj, STATUS,  &status.all);
	} while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);

	/* MNGR_INT*/
	ecg_spi_read(pObj, MNGR_INT,  &mngr_int.all);
	mngr_int.bit.e_fit = E_fit;
	ecg_spi_write(pObj, MNGR_INT, mngr_int.all);

	/* CNFG_ECG */
	ecg_spi_read(pObj, CNFG_ECG,  &cnfg_ecg.all);
	cnfg_ecg.bit.rate = Rate;
	cnfg_ecg.bit.gain = Gain;
	cnfg_ecg.bit.dhpf = Dhpf;
	cnfg_ecg.bit.dlpf = Dlpf;
	ecg_spi_write(pObj, CNFG_ECG, cnfg_ecg.all);

return MAX30001_OK;
}/* end of  MAX30001_ECG_InitStart */

/**
* @brief For MAX30001/3 ONLY
* @brief This function enables the ECG.
* @brief Uses Register CNFG_GEN-0x10.
* @returns 0-if no error.  A non-zero value indicates an error.
*
*/
int32_t MAX30001_Start_ECG(MAX30001_Object_t *pObj) {

	max30001_cnfg_gen_t cnfg_gen;
	max30001_status_t    status;
	uint32_t max30001_timeout;

	ecg_spi_read(pObj, CNFG_GEN,  &cnfg_gen.all);
	cnfg_gen.bit.en_ecg = 0b1;
	ecg_spi_write(pObj, CNFG_GEN, cnfg_gen.all);

	/* Wait for PLL Lock & References to settle down */
	max30001_timeout = 0;
	do
	{
		ecg_spi_read(pObj, STATUS,  &status.all);
	} while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);

return MAX30001_OK;
}/* End of MAX30001_Start_ECG */

/**
* @brief For MAX30001/3 ONLY
* @brief This function disables the ECG.
* @brief Uses Register CNFG_GEN-0x10.
* @returns 0-if no error.  A non-zero value indicates an error.
*
*/
int32_t MAX30001_Stop_ECG(MAX30001_Object_t *pObj) {

	max30001_cnfg_gen_t cnfg_gen;

	ecg_spi_read(pObj, CNFG_GEN,  &cnfg_gen.all);
	cnfg_gen.bit.en_ecg = 0b0;
	ecg_spi_write(pObj, CNFG_GEN, cnfg_gen.all);

return MAX30001_OK;
}/* End of MAX30001_Stop_ECG */


//=============================================================================
//COMMON
//=============================================================================

/**
* @brief This function enables the DC Lead Off detection. Either ECG or BIOZ can be detected, one at a time.
* @brief Registers Used:  CNFG_GEN-0x10
* @param En_dcloff: BIOZ Digital Lead Off Detection Enable
* @param Ipol: DC Lead Off Current Polarity (if current sources are enabled/connected)
* @param Imag: DC Lead off current Magnitude Selection
* @param Vth: DC Lead Off Voltage Threshold Selection
* @returns 0-if no error.  A non-zero value indicates an error.
*
*/
int32_t MAX30001_Enable_DcLeadOFF_Init(MAX30001_Object_t *pObj,
		               int8_t En_dcloff,
					   int8_t Ipol,
                       int8_t Imag,
					   int8_t Vth) {
	///<  the leads are not touching the body
	max30001_cnfg_gen_t cnfg_gen;

	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
	if (MAX30001_reg_read(&(pObj->Ctx),CNFG_GEN, &cnfg_gen.all) == -1) {
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
		return -1;
	}
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

	cnfg_gen.bit.en_dcloff = En_dcloff;
	cnfg_gen.bit.ipol = Ipol;
	cnfg_gen.bit.imag = Imag;
	cnfg_gen.bit.vth = Vth;

	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
	if (MAX30001_reg_write(&(pObj->Ctx),CNFG_GEN, cnfg_gen.all) == -1) {
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
		return -1;
	}
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

	return 0;
}

/**
 * @brief This is function called from the max30001_int_handler() function and processes all the ECG, BIOZ, PACE
 * @brief and the RtoR data and sticks them in appropriate arrays and variables each unsigned 32 bits.
 * @param ECG data will be in the array (input): max30001_ECG_FIFO_buffer[]
 * @param Pace data will be in the array (input): max30001_PACE[]
 * @param RtoRdata will be in the variable (input): max30001_RtoR_data
 * @param BIOZ data will be in the array (input): max30001_BIOZ_FIFO_buffer[]
 * @param global  max30001_ECG_FIFO_buffer[]
 * @param global  max30001_PACE[]
 * @param global  max30001_BIOZ_FIFO_buffer[]
 * @param global  max30001_RtoR_data
 * @param global  max30001_DCLeadOff
 * @param global  max30001_ACLeadOff
 * @param global  max30001_LeadON
 * @returns 0-if no error.  A non-zero value indicates an error.
 *
 */
int32_t MAX30001_FIFO_LeadONOff_Read(MAX30001_Object_t *pObj) {

	uint8_t result[32 * 3]; ///< 32words - 3bytes each
	uint8_t data_array[4];
	int32_t success = 0;
	int i, j;
	uint32_t total_databytes;
	uint8_t i_index;
	uint8_t data_chunk;
	uint8_t loop_logic;
	uint8_t etag, ptag, btag;
	uint8_t adr;
	int8_t ReadAllPaceOnce;
	uint32_t validcnt = 0;

	static uint8_t dcloffint_OneShot = 0;
	static uint8_t acloffint_OneShot = 0;
	static uint8_t bcgmon_OneShot = 0;
	static uint8_t acleadon_OneShot = 0;

	max30001_mngr_int_t mngr_int;
	max30001_cnfg_gen_t cnfg_gen;

	int8_t ret_val;

	etag = 0;
  
	if (global_status.bit.eint == 1 || global_status.bit.pint == 1)
	{
		adr = ECG_FIFO_BURST;
		data_array[0] = ((adr << 1) & 0xff) | 1;

		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_read(&pObj->Ctx,MNGR_INT, &mngr_int.all) == -1){
			HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
			return -1;
		}
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_read(&pObj->Ctx,CNFG_GEN, &cnfg_gen.all) == -1){
			HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
			return -1;
		}
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

		total_databytes = (mngr_int.bit.e_fit + 1) * 3;

		i_index = 0;
		loop_logic = 1;

		while (loop_logic)
		{
			if (total_databytes > 30)
			{
				data_chunk = 30;
				total_databytes = total_databytes - 30;
			}
			else
			{
				data_chunk = total_databytes;
				loop_logic = 0;
			}

			/* The extra 1 byte is for the extra byte that comes out of the SPI
			 * copy of the FIFO over here... */
			HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
			success = pObj->Ctx.write_reg(pObj->Ctx.handle,&data_array[0], 1, &result[i_index], (data_chunk + 1));
			if (success != 0)
			{
				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
				return -1;
			}
			HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

			///< This is important, because every transaction above creates an empty
			///< redundant data at result[0]
			for (j = i_index; j < (data_chunk + i_index); j++) /* get rid of the 1 extra byte by moving the whole array up one */
			{
				result[j] = result[j + 1];
			}

			/* point to the next array location to put the data in */
			i_index = i_index + 30;
		}

		 ReadAllPaceOnce = 0;

		///< Put the content of the FIFO based on the EFIT value, We ignore the
		///< result[0] and start concatenating indexes: 1,2,3 - 4,5,6 - 7,8,9 -
		for (i = 0, j = 0; i < mngr_int.bit.e_fit + 1; i++, j = j + 3) ///< index1=23-16 bit, index2=15-8 bit, index3=7-0 bit
		{
			max30001_ECG_FIFO_buffer[i] = ((uint32_t)result[j] << 16) + (result[j + 1] << 8) + result[j + 2];

			etag = (0b00111000 & result[j + 2]) >> 3;
			ptag = 0b00000111 & result[j + 2];

			if (ptag != 0b111 && ReadAllPaceOnce == 0)
			{
				ReadAllPaceOnce = 1; ///< This will prevent extra read of PACE, once group
								 ///< 0-5 is read ONCE.

				adr = PACE0_FIFO_BURST;

				data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
				success =pObj->Ctx.write_reg(pObj->Ctx.handle,&data_array[0], 1, &result[0], 10);
				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

				max30001_PACE[0] = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
				max30001_PACE[1] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
				max30001_PACE[2] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

				adr = PACE1_FIFO_BURST;

				data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
				success = pObj->Ctx.write_reg(pObj->Ctx.handle,&data_array[0], 1, &result[0], 10);
				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

				max30001_PACE[3] = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
				max30001_PACE[4] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
				max30001_PACE[5] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

				adr = PACE2_FIFO_BURST;

				data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
				success = pObj->Ctx.write_reg(pObj->Ctx.handle,&data_array[0], 1, &result[0], 10);
				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

				max30001_PACE[6] = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
				max30001_PACE[7] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
				max30001_PACE[8] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

				adr = PACE3_FIFO_BURST;

				data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
				success = pObj->Ctx.write_reg(pObj->Ctx.handle,&data_array[0], 1, &result[0], 10);
				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

				max30001_PACE[9]  = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
				max30001_PACE[10] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
				max30001_PACE[11] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

				adr = PACE4_FIFO_BURST;

				data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
				success = pObj->Ctx.write_reg(pObj->Ctx.handle,&data_array[0], 1, &result[0], 10);
				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

				max30001_PACE[12] = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
				max30001_PACE[13] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
				max30001_PACE[14] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

				adr = PACE5_FIFO_BURST;

				data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
				success = pObj->Ctx.write_reg(pObj->Ctx.handle,&data_array[0], 1, &result[0], 10);
				HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

				max30001_PACE[15] = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
				max30001_PACE[16] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
				max30001_PACE[17] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

				MAX30001_dataAvailable(MAX30001_DATA_PACE, max30001_PACE, 18,pObj); ///< Send out the Pace data once only
			}

			if (etag == 0b000) //Valid sample
			{
				validcnt++;
			}
		}



		if (etag != 0b110) //FIFO empty
		{
			validcnt = 0;
			MAX30001_dataAvailable(MAX30001_DATA_ECG, max30001_ECG_FIFO_buffer, (mngr_int.bit.e_fit + 1),pObj);
		}
		else
		{
			validcnt++;
		}

  } /* End of if (global_status.bit.eint == 1 || global_status.bit.pint == 1)*/


  ///< Handling BIOZ data... ================================================================================================
	if (global_status.bit.bint == 1)
	{
		adr = 0x22;
		data_array[0] = ((adr << 1) & 0xff) | 1;

		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_read(&pObj->Ctx,MNGR_INT, &mngr_int.all) == -1){
			HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
			return -1;
		}
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

		///< [(BFIT+1)*3byte]+1extra byte due to the addr

		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if(pObj->Ctx.write_reg(pObj->Ctx.handle,&data_array[0], 1, &result[0],((mngr_int.bit.b_fit + 1) * 3) + 1) == -1)
		{ ///< Make a copy of the FIFO over here...
			HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
			return -1;
		}

		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
		btag = 0b00000111 & result[3];

		///< Put the content of the FIFO based on the BFIT value, We ignore the
		///< result[0] and start concatenating indexes: 1,2,3 - 4,5,6 - 7,8,9 -
		for (i = 0, j = 0; i < mngr_int.bit.b_fit + 1; i++, j = j + 3) ///< index1=23-16 bit, index2=15-8 bit, index3=7-0 bit
		{
			max30001_BIOZ_FIFO_buffer[i] = ((uint32_t)result[j + 1] << 16) + (result[j + 2] << 8) + result[j + 3];
		}

		/* 	FIFO Empty (exception) */
		if (btag != 0b110)
		{
			MAX30001_dataAvailable(MAX30001_DATA_BIOZ, max30001_BIOZ_FIFO_buffer, mngr_int.bit.b_fit+1,pObj);
		}
	}


    /* RtoR ================================================================================================================*/
	if (global_status.bit.rrint == 1)
	{
		if (MAX30001_reg_read(&pObj->Ctx,RTOR, &max30001_RtoR_data) == -1)
		{
			return -1;
		}

		max30001_RtoR_data = (0x00FFFFFF & max30001_RtoR_data) >> 10;

		hspValMax30001.R2R = (uint16_t)max30001_RtoR_data;
		hspValMax30001.fmstr = (uint16_t)cnfg_gen.bit.fmstr;

		MAX30001_dataAvailable(MAX30001_DATA_RTOR, &max30001_RtoR_data, 1,pObj);
	}

  ret_val = 0;

  if (global_status.bit.dcloffint == 1) { ///< ECG/BIOZ Lead Off
    dcloffint_OneShot = 1;
    max30001_DCLeadOff = 0;
    max30001_DCLeadOff = max30001_DCLeadOff | (cnfg_gen.bit.en_dcloff << 8) | (global_status.all & 0x00000F);
    MAX30001_dataAvailable(MAX30001_DATA_LEADOFF_DC, &max30001_DCLeadOff, 1,pObj);
    ///< Do a FIFO Reset
    HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
    MAX30001_reg_write(&pObj->Ctx,FIFO_RST, 0x000000);
    HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

   // ret_val = 0b100;

  } else if (dcloffint_OneShot == 1 && global_status.bit.dcloffint == 0) { ///< Just send once when it comes out of dc lead off
    max30001_DCLeadOff = 0;
    max30001_DCLeadOff = max30001_DCLeadOff | (cnfg_gen.bit.en_dcloff << 8) | (global_status.all & 0x00000F);
    MAX30001_dataAvailable(MAX30001_DATA_LEADOFF_DC, &max30001_DCLeadOff, 1,pObj);
    dcloffint_OneShot = 0;
  }

  if (global_status.bit.bover == 1 || global_status.bit.bundr == 1) { ///< BIOZ AC Lead Off
    acloffint_OneShot = 1;
    max30001_ACLeadOff = 0;
    max30001_ACLeadOff =
        max30001_ACLeadOff | ((global_status.all & 0x030000) >> 16);
    MAX30001_dataAvailable(MAX30001_DATA_LEADOFF_AC, &max30001_ACLeadOff, 1,pObj);
    ///< Do a FIFO Reset
    HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
    MAX30001_reg_write(&pObj->Ctx,FIFO_RST, 0x000000);
    HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

    ret_val = 0b1000;
  } else if (acloffint_OneShot == 1 && global_status.bit.bover == 0 && global_status.bit.bundr == 0) { ///< Just send once when it comes out of ac lead off
    max30001_ACLeadOff = 0;
    max30001_ACLeadOff = max30001_ACLeadOff | ((global_status.all & 0x030000) >> 16);
    MAX30001_dataAvailable(MAX30001_DATA_LEADOFF_AC, &max30001_ACLeadOff, 1,pObj);
    acloffint_OneShot = 0;
  }

  if (global_status.bit.bcgmon == 1) {///< BIOZ BCGMON check
    bcgmon_OneShot = 1;
    max30001_bcgmon = 0;
    max30001_bcgmon = max30001_bcgmon | ((global_status.all & 0x000030) >> 4);
    MAX30001_dataAvailable(MAX30001_DATA_BCGMON, &max30001_bcgmon, 1,pObj);
    // Do a FIFO Reset
    HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
    MAX30001_reg_write(&pObj->Ctx,FIFO_RST, 0x000000);
    HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

    ret_val = 0b10000;
  } else if (bcgmon_OneShot == 1 && global_status.bit.bcgmon == 0) {
    max30001_bcgmon = 0;
    max30001_bcgmon = max30001_bcgmon | ((global_status.all & 0x000030) >> 4);
    bcgmon_OneShot = 0;
    MAX30001_dataAvailable(MAX30001_DATA_BCGMON, &max30001_bcgmon, 1,pObj);
  }

  if (global_status.bit.lonint == 1 && acleadon_OneShot == 0) {///< AC LeadON Check, when lead is on
    max30001_LeadOn = 0;
    HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
    MAX30001_reg_read(&pObj->Ctx,STATUS, &global_status.all);
    HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
    max30001_LeadOn =
        max30001_LeadOn | (cnfg_gen.bit.en_ulp_lon << 8) |
        ((global_status.all & 0x000800) >>
         11); ///< 0b01 will mean ECG Lead On, 0b10 will mean BIOZ Lead On

    // LEAD ON has been detected... Now take actions
    acleadon_OneShot = 1;
    MAX30001_dataAvailable(MAX30001_DATA_ACLEADON, &max30001_LeadOn, 1,pObj); ///< One shot data will be sent...
  } else if (global_status.bit.lonint == 0 && acleadon_OneShot == 1) {
    max30001_LeadOn = 0;
    HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
    MAX30001_reg_read(&pObj->Ctx,STATUS, &global_status.all);
    HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
    max30001_LeadOn =
        max30001_LeadOn | (cnfg_gen.bit.en_ulp_lon << 8) | ((global_status.all & 0x000800) >> 11); ///< 0b01 will mean ECG Lead On, 0b10 will mean BIOZ Lead On
    MAX30001_dataAvailable(MAX30001_DATA_ACLEADON, &max30001_LeadOn, 1,pObj); ///< One shot data will be sent...
    acleadon_OneShot = 0;
  }

  return ret_val;
}

/**
 * @brief This function handles the assignment of the two interrupt pins (INTB & INT2B) with various
 * @brief functions/behaviors  of the MAX30001.  Also, each pin can be configured for different drive capability.
 * @brief Uses Registers: EN_INT-0x02 and EN_INT2-0x03.
 * @param max30001_intrpt_Locatio_t  <argument>:  All the arguments with the aforementioned enumeration essentially
 *        can be configured to generate an interrupt on either INTB or INT2B or NONE.
 * @param max30001_intrpt_type_t  intb_Type:  INTB Port Type (EN_INT Selections).
 * @param max30001_intrpt_type _t int2b_Type:   INT2B Port Type (EN_INT2 Selections)
 * @returns 0-if no error.  A non-zero value indicates an error.
 *
 */
int32_t MAX30001_INT_assignment(MAX30001_Object_t *pObj,
		                     max30001_intrpt_Location_t en_enint_loc,     max30001_intrpt_Location_t en_eovf_loc,  max30001_intrpt_Location_t en_fstint_loc,
		                     max30001_intrpt_Location_t en_dcloffint_loc, max30001_intrpt_Location_t en_bint_loc,  max30001_intrpt_Location_t en_bovf_loc,
		                     max30001_intrpt_Location_t en_bover_loc,     max30001_intrpt_Location_t en_bundr_loc, max30001_intrpt_Location_t en_bcgmon_loc,
		                     max30001_intrpt_Location_t en_pint_loc,      max30001_intrpt_Location_t en_povf_loc,  max30001_intrpt_Location_t en_pedge_loc,
		                     max30001_intrpt_Location_t en_lonint_loc,    max30001_intrpt_Location_t en_rrint_loc, max30001_intrpt_Location_t en_samp_loc,
		                     max30001_intrpt_type_t  intb_Type,           max30001_intrpt_type_t int2b_Type)


{

	max30001_en_int_t en_int;
	max30001_en_int2_t en_int2;
	int32_t ret = MAX30001_OK;

	//CS pin low
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
	///< INT1
	if (MAX30001_reg_read(&(pObj->Ctx),EN_INT, &en_int.all) == -1) {
		ret=MAX30001_ERROR;
	}
	//CS pin high
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

	// max30001_en_int2.bit.en_pint       = 0b1;  // Keep this off...
	en_int.bit.en_eint   = 0b1 & en_enint_loc;
	en_int.bit.en_eovf   = 0b1 & en_eovf_loc;
	en_int.bit.en_fstint = 0b1 & en_fstint_loc;

	en_int.bit.en_dcloffint = 0b1 & en_dcloffint_loc;
	en_int.bit.en_bint      = 0b1 & en_bint_loc;
	en_int.bit.en_bovf      = 0b1 & en_bovf_loc;

	en_int.bit.en_bover  = 0b1 & en_bover_loc;
	en_int.bit.en_bundr  = 0b1 & en_bundr_loc;
	en_int.bit.en_bcgmon = 0b1 & en_bcgmon_loc;

	en_int.bit.en_pint   = 0b1 & en_pint_loc;
	en_int.bit.en_povf   = 0b1 & en_povf_loc;
	en_int.bit.en_pedge  = 0b1 & en_pedge_loc;

	en_int.bit.en_lonint = 0b1 & en_lonint_loc;
	en_int.bit.en_rrint  = 0b1 & en_rrint_loc;
	en_int.bit.en_samp   = 0b1 & en_samp_loc;

	en_int.bit.intb_type = int2b_Type;

	if(ret==MAX30001_OK)
	{
		//CS pin low
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_write(&(pObj->Ctx),EN_INT, en_int.all) == -1) {
			ret=MAX30001_ERROR;
		}
		//CS pin high
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
	}

	if(ret==MAX30001_OK)
	{
		//CS pin low
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_read(&(pObj->Ctx),EN_INT2, &en_int2.all) == -1) {
			ret=MAX30001_ERROR;
		}
		//CS pin high
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
	}

	en_int2.bit.en_eint   = 0b1 & (en_enint_loc >> 1);
	en_int2.bit.en_eovf   = 0b1 & (en_eovf_loc >> 1);
	en_int2.bit.en_fstint = 0b1 & (en_fstint_loc >> 1);

	en_int2.bit.en_dcloffint = 0b1 & (en_dcloffint_loc >> 1);
	en_int2.bit.en_bint      = 0b1 & (en_bint_loc >> 1);
	en_int2.bit.en_bovf      = 0b1 & (en_bovf_loc >> 1);

	en_int2.bit.en_bover  = 0b1 & (en_bover_loc >> 1);
	en_int2.bit.en_bundr  = 0b1 & (en_bundr_loc >> 1);
	en_int2.bit.en_bcgmon = 0b1 & (en_bcgmon_loc >> 1);

	en_int2.bit.en_pint  = 0b1 & (en_pint_loc >> 1);
	en_int2.bit.en_povf  = 0b1 & (en_povf_loc >> 1);
	en_int2.bit.en_pedge = 0b1 & (en_pedge_loc >> 1);

	en_int2.bit.en_lonint = 0b1 & (en_lonint_loc >> 1);
	en_int2.bit.en_rrint  = 0b1 & (en_rrint_loc >> 1);
	en_int2.bit.en_samp   = 0b1 & (en_samp_loc >> 1);

	en_int2.bit.intb_type = intb_Type;

	if(ret==MAX30001_OK)
	{
		//CS pin low
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_write(&(pObj->Ctx),EN_INT2, en_int2.all) == -1) {
			ret=MAX30001_ERROR;
		}
		//CS pin high
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
	}

  return ret;
}/* end of MAX30001_INT_assignment */


/**
 * @brief This function handles the assignment of the two interrupt pins (INTB & INT2B) with various
 * @brief functions/behaviors  of the MAX30001.  Also, each pin can be configured for different drive capability.
 * @brief Uses Registers: EN_INT-0x02 and EN_INT2-0x03.
 * @param max30001_intrpt_Locatio_t  <argument>:  All the arguments with the aforementioned enumeration essentially
 *        can be configured to generate an interrupt on either INTB or INT2B or NONE.
 * @param max30001_intrpt_type_t  intb_Type:  INTB Port Type (EN_INT Selections).
 * @param max30001_intrpt_type _t int2b_Type:   INT2B Port Type (EN_INT2 Selections)
 * @returns 0-if no error.  A non-zero value indicates an error.
 *
 */
int32_t MAX30001_INT_ECG_assignment(MAX30001_Object_t *pObj,
		                     max30001_intrpt_Location_t en_enint_loc,     max30001_intrpt_Location_t en_eovf_loc,  max30001_intrpt_Location_t en_fstint_loc,
		                     max30001_intrpt_Location_t en_dcloffint_loc, max30001_intrpt_Location_t en_bovf_loc,
		                     max30001_intrpt_Location_t en_bover_loc,     max30001_intrpt_Location_t en_bundr_loc, max30001_intrpt_Location_t en_bcgmon_loc,
		                     max30001_intrpt_Location_t en_pint_loc,      max30001_intrpt_Location_t en_povf_loc,  max30001_intrpt_Location_t en_pedge_loc,
		                     max30001_intrpt_Location_t en_lonint_loc,    max30001_intrpt_Location_t en_rrint_loc, max30001_intrpt_Location_t en_samp_loc,
		                     max30001_intrpt_type_t  intb_Type,           max30001_intrpt_type_t int2b_Type)


{

	max30001_en_int_t en_int;
	max30001_en_int2_t en_int2;
	int32_t ret = MAX30001_OK;

	//CS pin low
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
	///< INT1
	if (MAX30001_reg_read(&(pObj->Ctx),EN_INT, &en_int.all) == -1) {
		ret=MAX30001_ERROR;
	}
	//CS pin high
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

	// max30001_en_int2.bit.en_pint       = 0b1;  // Keep this off...
	en_int.bit.en_eint   = 0b1 & en_enint_loc;
	en_int.bit.en_eovf   = 0b1 & en_eovf_loc;
	en_int.bit.en_fstint = 0b1 & en_fstint_loc;

	en_int.bit.en_dcloffint = 0b1 & en_dcloffint_loc;
	en_int.bit.en_bovf      = 0b1 & en_bovf_loc;

	en_int.bit.en_bover  = 0b1 & en_bover_loc;
	en_int.bit.en_bundr  = 0b1 & en_bundr_loc;
	en_int.bit.en_bcgmon = 0b1 & en_bcgmon_loc;

	en_int.bit.en_pint   = 0b1 & en_pint_loc;
	en_int.bit.en_povf   = 0b1 & en_povf_loc;
	en_int.bit.en_pedge  = 0b1 & en_pedge_loc;

	en_int.bit.en_lonint = 0b1 & en_lonint_loc;
	en_int.bit.en_rrint  = 0b1 & en_rrint_loc;
	en_int.bit.en_samp   = 0b1 & en_samp_loc;

	en_int.bit.intb_type = int2b_Type;

	if(ret==MAX30001_OK)
	{
		//CS pin low
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_write(&(pObj->Ctx),EN_INT, en_int.all) == -1) {
			ret=MAX30001_ERROR;
		}
		//CS pin high
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
	}

	if(ret==MAX30001_OK)
	{
		//CS pin low
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_read(&(pObj->Ctx),EN_INT2, &en_int2.all) == -1) {
			ret=MAX30001_ERROR;
		}
		//CS pin high
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
	}

	en_int2.bit.en_eint   = 0b1 & (en_enint_loc >> 1);
	en_int2.bit.en_eovf   = 0b1 & (en_eovf_loc >> 1);
	en_int2.bit.en_fstint = 0b1 & (en_fstint_loc >> 1);

	en_int2.bit.en_dcloffint = 0b1 & (en_dcloffint_loc >> 1);
	en_int2.bit.en_bovf      = 0b1 & (en_bovf_loc >> 1);

	en_int2.bit.en_bover  = 0b1 & (en_bover_loc >> 1);
	en_int2.bit.en_bundr  = 0b1 & (en_bundr_loc >> 1);
	en_int2.bit.en_bcgmon = 0b1 & (en_bcgmon_loc >> 1);

	en_int2.bit.en_pint  = 0b1 & (en_pint_loc >> 1);
	en_int2.bit.en_povf  = 0b1 & (en_povf_loc >> 1);
	en_int2.bit.en_pedge = 0b1 & (en_pedge_loc >> 1);

	en_int2.bit.en_lonint = 0b1 & (en_lonint_loc >> 1);
	en_int2.bit.en_rrint  = 0b1 & (en_rrint_loc >> 1);
	en_int2.bit.en_samp   = 0b1 & (en_samp_loc >> 1);

	en_int2.bit.intb_type = intb_Type;

	if(ret==MAX30001_OK)
	{
		//CS pin low
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_write(&(pObj->Ctx),EN_INT2, en_int2.all) == -1) {
			ret=MAX30001_ERROR;
		}
		//CS pin high
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
	}

  return ret;
}/* end of MAX30001_INT_ECG_assignment */


/**
 * @brief This function handles the assignment of the two interrupt pins (INTB & INT2B) with various
 * @brief functions/behaviors  of the MAX30001.  Also, each pin can be configured for different drive capability.
 * @brief Uses Registers: EN_INT-0x02 and EN_INT2-0x03.
 * @param max30001_intrpt_Locatio_t  <argument>:  All the arguments with the aforementioned enumeration essentially
 *        can be configured to generate an interrupt on either INTB or INT2B or NONE.
 * @param max30001_intrpt_type_t  intb_Type:  INTB Port Type (EN_INT Selections).
 * @param max30001_intrpt_type _t int2b_Type:   INT2B Port Type (EN_INT2 Selections)
 * @returns 0-if no error.  A non-zero value indicates an error.
 *
 */
int32_t MAX30001_BIOZ_INT_assignment(MAX30001_Object_t *pObj,
		max30001_intrpt_Location_t en_bint_loc)
{

	max30001_en_int_t en_int;
	max30001_en_int2_t en_int2;
	int32_t ret = MAX30001_OK;

	//CS pin low
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
	///< INT1
	if (MAX30001_reg_read(&(pObj->Ctx),EN_INT, &en_int.all) == -1) {
		ret=MAX30001_ERROR;
	}
	//CS pin high
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

	en_int.bit.en_bint      = 0b1 & en_bint_loc;

	if(ret==MAX30001_OK)
	{
		//CS pin low
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_write(&(pObj->Ctx),EN_INT, en_int.all) == -1) {
			ret=MAX30001_ERROR;
		}
		//CS pin high
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
	}


	if(ret==MAX30001_OK)
	{
		//CS pin low
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_read(&(pObj->Ctx),EN_INT2, &en_int2.all) == -1) {
			ret=MAX30001_ERROR;
		}
		//CS pin high
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
	}

	en_int2.bit.en_bint      = 0b1 & (en_bint_loc >> 1);
	if(ret==MAX30001_OK)
	{
		//CS pin low
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
		if (MAX30001_reg_write(&(pObj->Ctx),EN_INT2, en_int2.all) == -1) {
			ret=MAX30001_ERROR;
		}
		//CS pin high
		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
	}

  return ret;
}/* end of MAX30001_BIOZ_INT_assignment */


/**
 * @brief This function reads status register
 * @brief for the details on how to use this.
 * @returns 0-if no error.
 */
int32_t MAX30001_get_status(MAX30001_Object_t *pObj, uint32_t * all)
{
	int32_t ret=MAX30001_OK;
	//CS pin low
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
	//read status reg
	if (MAX30001_reg_read(&(pObj->Ctx),STATUS, all) == -1){
	  ret=MAX30001_ERROR;
	}
	//CS pin high
	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);

return ret;
}/* end of MAX30001_get_status */



#if 0
//******************************************************************************

//******************************************************************************
int MAX30001_BIOZ_InitBist(uint8_t En_bist, uint8_t Rnom,
                            uint8_t Rmod, uint8_t Fbist) {

  max30001_cnfg_bmux_t cnfg_bmux;

  ///< CNFG_BMUX

  if (MAX30001_reg_read(CNFG_BMUX, &cnfg_bmux.all) == -1) {
    return -1;
  }

  cnfg_bmux.bit.en_bist = En_bist;
  cnfg_bmux.bit.rnom = Rnom;
  cnfg_bmux.bit.rmod = Rmod;
  cnfg_bmux.bit.fbist = Fbist;

  if (MAX30001_reg_write(CNFG_BMUX, cnfg_bmux.all) == -1) {
    return -1;
  }

  return 0;
}
#endif
#if 0

//==============================================================================
//ECG
//==============================================================================
/**
 * @brief For MAX30001/3 ONLY
 * @brief This function sets up the MAX30001 for the ECG measurements.
 * @brief Registers used:  CNFG_EMUX, CNFG_GEN, MNGR_INT, CNFG_ECG.
 * @param En_ecg: ECG Channel Enable <CNFG_GEN register bits>
 * @param Openp: Open the ECGN Input Switch (most often used for testing and calibration studies) <CNFG_EMUX register bits>
 * @param Openn: Open the ECGN Input Switch (most often used for testing and calibration studies) <CNFG_EMUX register bits>
 * @param Calp_sel: ECGP Calibration Selection <CNFG_EMUX register bits>
 * @param Caln_sel: ECGN Calibration Selection <CNFG_EMUX register bits>
 * @param E_fit: ECG FIFO Interrupt Threshold (issues EINT based on number of unread FIFO records) <CNFG_GEN register bits>
 * @param Clr_rrint: RTOR R Detect Interrupt (RRINT) Clear Behavior <CNFG_GEN register bits>
 * @param Rate: ECG Data Rate
 * @param Gain: ECG Channel Gain Setting
 * @param Dhpf: ECG Channel Digital High Pass Filter Cutoff Frequency
 * @param Dlpf:  ECG Channel Digital Low Pass Filter Cutoff Frequency
 * @returns 0-if no error.  A non-zero value indicates an error.
 *
 */
//int32_t MAX30001_ECG_InitStart(MAX30001_Object_t *pObj,
//		                   uint8_t En_ecg,
//						   uint8_t Openp,
//                           uint8_t Openn,
//						   uint8_t Pol,
//                           uint8_t Calp_sel,
//						   uint8_t Caln_sel,
//                           uint8_t E_fit,
//						   uint8_t Rate,
//						   uint8_t Gain,
//                           uint8_t Dhpf,
//						   uint8_t Dlpf) {
//
//	max30001_cnfg_emux_t cnfg_emux;
//	max30001_cnfg_gen_t  cnfg_gen;
//	max30001_status_t    status;
//	max30001_mngr_int_t  mngr_int;
//	max30001_cnfg_ecg_t  cnfg_ecg;
//
////	/* setting CNFG_EMUX register */
////	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
////	if (MAX30001_reg_read(&(pObj->Ctx),CNFG_EMUX, &cnfg_emux.all) == MAX30001_ERROR) {
////		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
////		return MAX30001_ERROR;
////	}
////	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
////
////	cnfg_emux.bit.openp    = Openp;
////	cnfg_emux.bit.openn    = Openn;
////	cnfg_emux.bit.pol      = Pol;
////	cnfg_emux.bit.calp_sel = Calp_sel;
////	cnfg_emux.bit.caln_sel = Caln_sel;
////
////	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
////	if (MAX30001_reg_write(&(pObj->Ctx),CNFG_EMUX, cnfg_emux.all) == MAX30001_ERROR) {
////		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
////		return MAX30001_ERROR;
////	}
////	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//
//	/* setting CNFG_EMUX register */
//	ecg_spi_read(pObj, CNFG_EMUX, &cnfg_emux.all);
//	cnfg_emux.bit.openp    = Openp;
//	cnfg_emux.bit.openn    = Openn;
//	cnfg_emux.bit.pol      = Pol;
//	cnfg_emux.bit.calp_sel = Calp_sel;
//	cnfg_emux.bit.caln_sel = Caln_sel;
//	ecg_spi_write(pObj, CNFG_EMUX, cnfg_emux.all);
//
//
//	/* setting CNFG_EMUX register */
//	ecg_spi_read(pObj, CNFG_GEN,  &cnfg_gen.all);
//	cnfg_gen.bit.en_ecg = En_ecg;
//	ecg_spi_write(pObj, CNFG_GEN, cnfg_emux.all);
//
////	/* =================== ENABLE CHANNELS =====================*/
////
////	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);	///< CNFG_GEN
////	if (MAX30001_reg_read(&(pObj->Ctx),CNFG_GEN, &cnfg_gen.all) == MAX30001_ERROR) {
////		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
////		return MAX30001_ERROR;
////	}
////	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
////
////	cnfg_gen.bit.en_ecg = En_ecg; // 0b1
////
////	///< fmstr is default
////	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
////	if (MAX30001_reg_write(&(pObj->Ctx),CNFG_GEN, cnfg_gen.all) == MAX30001_ERROR) {
////		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
////		return MAX30001_ERROR;
////	}
////	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//
//	/* Wait for PLL Lock & References to settle down */
//	max30001_timeout = 0;
//	do {
//		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
//		if (MAX30001_reg_read(&(pObj->Ctx),STATUS, &status.all) == MAX30001_ERROR) {// Wait and spin for PLL to lock...
//			HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//			return MAX30001_ERROR;
//		}
//		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//	} while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);
//
//	///< MNGR_INT
//	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
//	if (MAX30001_reg_read(&(pObj->Ctx),MNGR_INT, &mngr_int.all) == MAX30001_ERROR) {
//		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//		return MAX30001_ERROR;
//	}
//	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//	mngr_int.bit.e_fit = E_fit;
//
//	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
//	if (MAX30001_reg_write(&(pObj->Ctx),MNGR_INT, mngr_int.all) == MAX30001_ERROR) {
//		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//		return MAX30001_ERROR;
//	}
//	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//
//	///< CNFG_ECG
//	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
//	if (MAX30001_reg_read(&(pObj->Ctx),CNFG_ECG, &cnfg_ecg.all) == MAX30001_ERROR) {
//		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//		return MAX30001_ERROR;
//	}
//	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//
//	cnfg_ecg.bit.rate = Rate;
//	cnfg_ecg.bit.gain = Gain;
//	cnfg_ecg.bit.dhpf = Dhpf;
//	cnfg_ecg.bit.dlpf = Dlpf;
//
//	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_RESET);
//	if (MAX30001_reg_write(&(pObj->Ctx),CNFG_ECG, cnfg_ecg.all) == MAX30001_ERROR) {
//		HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//		return MAX30001_ERROR;
//	}
//	HAL_GPIO_WritePin(pObj->IO.csPort, pObj->IO.csPin.Pin, GPIO_PIN_SET);
//
//return MAX30001_OK;
//}/* end of  MAX30001_ECG_InitStart */
//******************************************************************************
int MAX30001_Rbias_FMSTR_Init(uint8_t En_rbias, uint8_t Rbiasv,
                               uint8_t Rbiasp, uint8_t Rbiasn,
                               uint8_t Fmstr) {

  max30001_cnfg_gen_t cnfg_gen;

  if (MAX30001_reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_rbias = En_rbias;
  cnfg_gen.bit.rbiasv   = Rbiasv;
  cnfg_gen.bit.rbiasp   = Rbiasp;
  cnfg_gen.bit.rbiasn   = Rbiasn;
  cnfg_gen.bit.fmstr    = Fmstr;

  if (MAX30001_reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }
  return 0;
}

//******************************************************************************


//******************************************************************************
int MAX30001_CAL_Stop(void) {

  max30001_cnfg_cal_t cnfg_cal;

  if (MAX30001_reg_read(CNFG_CAL, &cnfg_cal.all) == -1) {
    return -1;
  }

  cnfg_cal.bit.en_vcal = 0; // Disable VCAL, all other settings are left unaffected

  if (MAX30001_reg_write(CNFG_CAL, cnfg_cal.all) == -1) {
    return -1;
  }

  return 0;
}
//******************************************************************************





//******************************************************************************
int MAX30001_ECGFast_Init(uint8_t Clr_Fast, uint8_t Fast, uint8_t Fast_Th) {

  max30001_mngr_int_t mngr_int;
  max30001_mngr_dyn_t mngr_dyn;

  if (MAX30001_reg_read(MNGR_INT, &mngr_int.all) == -1) {
    return -1;
  }

  mngr_int.bit.clr_fast = Clr_Fast;

  if (MAX30001_reg_write(MNGR_INT, mngr_int.all) == -1) {
    return -1;
  }

  if (MAX30001_reg_read(MNGR_DYN, &mngr_dyn.all) == -1) {
    return -1;
  }

  mngr_dyn.bit.fast = Fast;
  mngr_dyn.bit.fast_th = Fast_Th;

  if (MAX30001_reg_write(MNGR_INT, mngr_dyn.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************

//******************************************************************************
int MAX30001_PACE_InitStart(uint8_t En_pace, uint8_t Clr_pedge,
                             uint8_t Pol, uint8_t Gn_diff_off,
                             uint8_t Gain, uint8_t Aout_lbw,
                             uint8_t Aout, uint8_t Dacp,
                             uint8_t Dacn) {

  /**** SET MASTER FREQUENCY, ENABLE CHANNELS ****/

   max30001_cnfg_gen_t  cnfg_gen;
   max30001_status_t    status;
   max30001_mngr_int_t  mngr_int;
   max30001_cnfg_pace_t cnfg_pace;

  ///< CNFG_GEN

  if (MAX30001_reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_pace = En_pace; // 0b1;

  if (MAX30001_reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  /**** Wait for PLL Lock & References to settle down ****/
  max30001_timeout = 0;

  do {
    if (MAX30001_reg_read(STATUS, &status.all) ==
        -1) // Wait and spin for PLL to lock...
    {
      return -1;
    }

  } while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);

  ///< MNGR_INT

  if (MAX30001_reg_read(MNGR_INT, &mngr_int.all) == -1) {
    return -1;
  }

  mngr_int.bit.clr_pedge = Clr_pedge; // 0b0;

  if (MAX30001_reg_write(MNGR_INT, mngr_int.all) == -1) {
    return -1;
  }

  ///< CNFG_PACE

  MAX30001_reg_read(CNFG_PACE, &cnfg_pace.all);

  cnfg_pace.bit.pol         = Pol;
  cnfg_pace.bit.gn_diff_off = Gn_diff_off;
  cnfg_pace.bit.gain        = Gain;
  cnfg_pace.bit.aout_lbw    = Aout_lbw;
  cnfg_pace.bit.aout        = Aout;
  cnfg_pace.bit.dacp        = Dacp;
  cnfg_pace.bit.dacn        = Dacn;

  MAX30001_reg_write(CNFG_PACE, cnfg_pace.all);

  return 0;
}

//******************************************************************************
int MAX30001_Stop_PACE(void) {

  max30001_cnfg_gen_t cnfg_gen;

  if (MAX30001_reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_pace = 0; ///< Stop PACE

  if (MAX30001_reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************

//******************************************************************************
int MAX30001_RtoR_InitStart(uint8_t En_rtor, uint8_t Wndw,
                             uint8_t Gain, uint8_t Pavg, uint8_t Ptsf,
                             uint8_t Hoff, uint8_t Ravg, uint8_t Rhsf,
                             uint8_t Clr_rrint) {

  max30001_mngr_int_t mngr_int;
  max30001_cnfg_rtor1_t cnfg_rtor1;
  max30001_cnfg_rtor2_t cnfg_rtor2;

  ///< MNGR_INT
  if (MAX30001_reg_read(MNGR_INT, &mngr_int.all) == -1) {
    return -1;
  }

  mngr_int.bit.clr_rrint = Clr_rrint;
  ///< 0b01 & 0b00 are for interrupt mode...
  ///< 0b10 is for monitoring mode... it just overwrites the data...

  if (MAX30001_reg_write(MNGR_INT, mngr_int.all) == -1) {
    return -1;
  }

  ///< RTOR1
  if (MAX30001_reg_read(CNFG_RTOR1, &cnfg_rtor1.all) == -1) {
    return -1;
  }

  cnfg_rtor1.bit.wndw = Wndw;
  cnfg_rtor1.bit.gain = Gain;
  cnfg_rtor1.bit.en_rtor = En_rtor;
  cnfg_rtor1.bit.pavg = Pavg;
  cnfg_rtor1.bit.ptsf = Ptsf;

  if (MAX30001_reg_write(CNFG_RTOR1, cnfg_rtor1.all) == -1) {
    return -1;
  }

  ///< RTOR2
  if (MAX30001_reg_read(CNFG_RTOR2, &cnfg_rtor2.all) == -1) {
    return -1;
  }
  cnfg_rtor2.bit.hoff = Hoff;
  cnfg_rtor2.bit.ravg = Ravg;
  cnfg_rtor2.bit.rhsf = Rhsf;

  if (MAX30001_reg_write(CNFG_RTOR2, cnfg_rtor2.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001_Stop_RtoR(void) {

  max30001_cnfg_rtor1_t cnfg_rtor1;

  if (MAX30001_reg_read(CNFG_RTOR1, &cnfg_rtor1.all) == -1) {
    return -1;
  }

  cnfg_rtor1.bit.en_rtor = 0; ///< Stop RtoR

  if (MAX30001_reg_write(CNFG_RTOR1, cnfg_rtor1.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001_PLL_lock(void) {
  ///< Spin to see PLLint become zero to indicate a lock.

  max30001_status_t status;

  max30001_timeout = 0;

  do {
    if (MAX30001_reg_read(STATUS, &status.all) == -1) { ///< Wait and spin for PLL to lock...

      return -1;
    }

  } while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);

  return 0;
}


//******************************************************************************

//******************************************************************************


//******************************************************************************


//******************************************************************************


//******************************************************************************
int MAX30001_Disable_DcLeadOFF(void) {

  max30001_cnfg_gen_t cnfg_gen;

  ///< CNFG_GEN
  if (MAX30001_reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_dcloff = 0; // Turned off the dc lead off.

  if (MAX30001_reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001_BIOZ_Enable_ACLeadOFF_Init(uint8_t En_bloff, uint8_t Bloff_hi_it,
                                         uint8_t Bloff_lo_it) {

  max30001_cnfg_gen_t cnfg_gen;
  max30001_mngr_dyn_t mngr_dyn;

  ///< CNFG_GEN
  if (MAX30001_reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_bloff = En_bloff;

  if (MAX30001_reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  ///< MNGR_DYN
  if (MAX30001_reg_read(MNGR_DYN, &mngr_dyn.all) == -1) {
    return -1;
  }

  mngr_dyn.bit.bloff_hi_it = Bloff_hi_it;
  mngr_dyn.bit.bloff_lo_it = Bloff_lo_it;

  if (MAX30001_reg_write(MNGR_DYN, mngr_dyn.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001_BIOZ_Disable_ACleadOFF(void) {

  max30001_cnfg_gen_t cnfg_gen;

  ///< CNFG_GEN
  if (MAX30001_reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_bloff = 0b0; // Turns of the BIOZ AC Lead OFF feature

  if (MAX30001_reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001_BIOZ_Enable_BCGMON(void) {

  max30001_cnfg_bioz_t cnfg_bioz;

  ///< CNFG_BIOZ
  if (MAX30001_reg_read(CNFG_BIOZ, &cnfg_bioz.all) == -1) {
    return -1;
  }

  cnfg_bioz.bit.cgmon = 1;

  if (MAX30001_reg_write(CNFG_BIOZ, cnfg_bioz.all) == -1) {
    return -1;
  }

  return 0;
}


//******************************************************************************
int MAX30001_Enable_LeadON(int8_t Channel) // Channel: ECG = 0b01, BIOZ = 0b10, Disable = 0b00
{

  max30001_cnfg_gen_t cnfg_gen;

  ///< CNFG_GEN
  if (MAX30001_reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_ecg  = 0b0;
  cnfg_gen.bit.en_bioz = 0b0;
  cnfg_gen.bit.en_pace = 0b0;

  cnfg_gen.bit.en_ulp_lon = Channel; ///< BIOZ ULP lead on detection...

  if (MAX30001_reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001_Disable_LeadON(void) {

  max30001_cnfg_gen_t cnfg_gen;
  ///< CNFG_GEN
  if (MAX30001_reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_ulp_lon = 0b0;

  if (MAX30001_reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
#define LEADOFF_SERVICE_TIME 0x2000 ///< 0x1000 = 1 second
#define LEADOFF_NUMSTATES 2
uint32_t leadoffState = 0;
uint32_t max30001_LeadOffoldTime = 0;
void MAX30001_ServiceLeadoff(uint32_t currentTime) {

  uint32_t delta_Time;

  delta_Time = currentTime - max30001_LeadOffoldTime;

  if (delta_Time > LEADOFF_SERVICE_TIME) {
    switch (leadoffState) {
    case 0: ///< switch to ECG DC Lead OFF
    MAX30001_Enable_DcLeadOFF_Init(0b01, 0b0, 0b001, 0b00);
      break;

    case 1: ///< switch to BIOZ DC Lead OFF
    MAX30001_Enable_DcLeadOFF_Init(0b10, 0b0, 0b001, 0b00);
      break;
    }

    leadoffState++;
    leadoffState %= LEADOFF_NUMSTATES;

    max30001_LeadOffoldTime = currentTime;
  }
}
//******************************************************************************
#define LEADON_SERVICE_TIME 0x2000 // 0x1000 = 1 second
#define LEADON_NUMSTATES 2
uint32_t leadOnState = 0;
uint32_t max30001_LeadOnoldTime = 0;
void MAX30001_ServiceLeadON(uint32_t currentTime) {

  uint32_t delta_Time;

  delta_Time = currentTime - max30001_LeadOnoldTime;

  if (delta_Time > LEADON_SERVICE_TIME) {
    switch (leadOnState) {
    case 0: ///< switch to ECG DC Lead ON
      MAX30001_Enable_LeadON(0b01);
      break;

    case 1: ///< switch to BIOZ DC Lead ON
      MAX30001_Enable_LeadON(0b10);
      break;
    }

    leadOnState++;
    leadOnState %= LEADON_NUMSTATES;

    max30001_LeadOnoldTime = currentTime;
  }
}

//******************************************************************************


//******************************************************************************






volatile int MAX30001_xferFlag = 0;


#endif
