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
#include <stdint.h>
#include "Freertos.h"
#include <task.h>
#include <queue.h>

#include "main.h"
#include "MAX30001_dev.h"
#include "MAX30001_bus.h"
#include "bus.h"

#define MOD_NAME "[MAX30001 DEV_BIT]"

extern MAX30001_Object_t max30001_obj_0;

#if 0
void ECG_test_MAX30001(void *para)
{
	uint32_t all;
	uint8_t deb_block[512];
	uint16_t deb_block_index = 0;

	aux_dbg_printLogStr( CONCAT_MODNAME( MOD_NAME , "Init MAX30101 callback, interrupt...\r\n"));

	//MAX30001_FCLK_MaximOnly();

	/* allow interrupts for maxim 30001 device */
	MAX30001_AllowInterrupts(1);

	/*  Do a software reset of the MAX30001 */
	MAX30001_sw_rst();

	/* Configure io pins in interrupt mode */
	EXTI_IRQHandler_Config();

	/* interrupt pins assignment Max30001 */
	MAX30001_INT_assignment(MAX30001_INT_B,    MAX30001_NO_INT,   MAX30001_NO_INT,  //  en_enint_loc,      en_eovf_loc,   en_fstint_loc,
							MAX30001_INT_2B,   MAX30001_INT_2B,   MAX30001_NO_INT,  //  en_dcloffint_loc,  en_bint_loc,   en_bovf_loc,
							MAX30001_INT_2B,   MAX30001_INT_2B,   MAX30001_NO_INT,  //  en_bover_loc,      en_bundr_loc,  en_bcgmon_loc,
							MAX30001_INT_B,    MAX30001_NO_INT,   MAX30001_NO_INT,  //  en_pint_loc,       en_povf_loc,   en_pedge_loc,
							MAX30001_INT_2B,   MAX30001_INT_B,    MAX30001_NO_INT,  //  en_lonint_loc,     en_rrint_loc,  en_samp_loc,
							MAX30001_INT_ODNR, MAX30001_INT_ODNR);                  //  intb_Type,         int2b_Type)

	MAX30001_onDataAvailable(&StreamPacketUint32_ecg);

	/// Set and Start the VCAL input
	/// @brief NOTE VCAL must be set first if VCAL is to be used
	//MAX30001_CAL_InitStart(EN_VCAL , VMODE, VMAG, FCAL, THIGH, FIFTY);

	aux_dbg_printLogStr("setting up ECG streaming\n");

	/// ECG Initialization
	MAX30001_ECG_InitStart(EN_ECG, OPENP, OPENN, POL, CALP_SEL, CALN_SEL, E_FIT, RATE, GAIN, DHPF, DLPF);

	/// @brief Set Rbias & FMSTR over here
	//MAX30001_Rbias_FMSTR_Init(EN_RBIAS, RBIASV, RBIASP, RBIASN,FMSTR);

	aux_dbg_printLogStr("start streaming\n");

	/* sync with max30001 */
	MAX30001_synch();
	/* clear the status register for a clean start */
	MAX30001_reg_read(STATUS, &all);

	aux_dbg_printLogStr("Please wait for data to start streaming\n");
	while (1)//todo no polling add mailbox
	{
		int i =0;
		uint32_t gen_cnt = 0;
		uint8_t etag=0,ptag=0;
		uint32_t vals_cnt = 0;
		uint32_t emptf_cnt = 0;
		uint32_t lasts_cnt = 0;
		uint32_t badData_cnt = 0;
		uint32_t overflow_cnt = 0;

		if(data_trigger == 1)
		{
			etag=0,ptag=0;
			vals_cnt = 0;
			emptf_cnt = 0;
			lasts_cnt = 0;
			badData_cnt = 0;
			overflow_cnt = 0;

			data_trigger = 0;
			deb_block_index = 0;

			for( i = 0; i < 32; i++ )
			{
				uint8_t *ptr;
				uint8_t str[16];

				sprintf((char *)str, "%X ", (0xFFFF & (ecgBuffer[i] >> 6)));

				etag = (0b00111000 & ecgBuffer[i]) >> 3;
				ptag = 0b00000111 & ecgBuffer[i];
				ptr = str;

				if(ptag != 0b111)//unexpected
				{
					Error_Handler((uint8_t *)__FILE__, __LINE__);
				}

				if (etag == 0b111)//FIFO overflow
				{
					Error_Handler((uint8_t *)__FILE__, __LINE__);
				}

				if(etag == 0b000) //Valid sample
				{
					vals_cnt++;
				}

				if(etag == 0b110) //fifo empty
				{
					emptf_cnt++;
				}

				if(etag == 0b010) //last fifo el
				{
					lasts_cnt++;
				}

				/* bad data fast mode sample or last fast mode sample */
				if(etag == 0b001 || etag == 0b011)
				{
					Error_Handler((uint8_t *)__FILE__, __LINE__);
				}

				while (*ptr != 0)
				{
					deb_block[deb_block_index] = *ptr;
					ptr++;
					deb_block_index++;
				}
			}

			/* Print the ECG data on a serial port terminal software */
			PrintLogBuffer((uint8_t*)deb_block, deb_block_index);
		}
		else
		{
			gen_cnt++;
		}
	}

}/* End of bit_MAX30001 */





/**
 * @brief BIT of max30001
 * @brief save in testing_ecg_flags the bit results.
 * @brief testing_ecg_flags[flag] = 0 => PASS|testing_ecg_flags[flag] = -1 => FAIL
 */
int MAX30001_bit_func(void *para)
{
	uint32_t foundEcg = 0;
	uint32_t foundBioz = 0;
	uint32_t foundPace = 0;
	uint32_t foundRtoR = 0;
	uint32_t id =  0;
	int partVersion; // 0 = 30004  1 = 30001  2 = 30002  3 = 30003

	// clear testing flags
	testing_ecg_flags[TESTING_ECG_FLAG] = -1;
	testing_ecg_flags[TESTING_BIOZ_FLAG] = -1;
	testing_ecg_flags[TESTING_PACE_FLAG] = -1;
	testing_ecg_flags[TESTING_RTOR_FLAG] = -1;

	if(BUS_SPI3_Init())
	{
		aux_dbg_printLogStr("\r\n[MAX30001] BUS FAILED \r\n");
		return SYS_ERROR_DEVICE_FAILURE;
	}


	// read the id
	MAX30001_readID(&max30001_obj_0,&id);

	partVersion = id >> 12;
	partVersion = partVersion & 0x3;

	// start streams
	if (partVersion == 1)
	{
		aux_dbg_printLogStr( CONCAT_MODNAME( MOD_NAME , "\r\n [MAX30001] max30001 part detected \r\n"));
	}
	else
	{
		aux_dbg_printLogStr( CONCAT_MODNAME( MOD_NAME ,"bit fail wrong revision \r\n"));
		return -1;
	}

	//================================

	MAX30001_CAL_InitStart(0b1, 0b1, 0b1, 0b011, 0x7FF, 0b0);

	testing_ecg_flags[TESTING_ECG_FLAG]  = MAX30001_ECG_InitStart(&max30001_obj_0,0b1, 0b1, 0b1, 0b0, 0b10, 0b11, 0x1F, 0b00, 0b00, 0b0, 0b01);
	testing_ecg_flags[TESTING_RTOR_FLAG] = MAX30001_RtoR_InitStart(0b1, 0b0011, 0b1111, 0b00, 0b0011, 0b000001,0b00, 0b000, 0b01);
	if (partVersion == 1)
	{
	  testing_ecg_flags[TESTING_PACE_FLAG] =  MAX30001_PACE_InitStart(0b1, 0b0, 0b0, 0b1, 0x0, 0b0, 0b00, 0b0, 0b0);
	  testing_ecg_flags[TESTING_BIOZ_FLAG] =  MAX30001_BIOZ_InitStart(0b1, 0b1, 0b1, 0b10, 0b11, 0b00, 7, 0b0,0b010, 0b0, 0b10, 0b00, 0b00, 2, 0b0, 0b111, 0b0000);
	}

	MAX30001_Rbias_FMSTR_Init(0b01, 0b10, 0b1, 0b1, 0b00);
	MAX30001_synch(&max30001_obj_0);

	if ((foundEcg == 0) && (testing_ecg_flags[TESTING_ECG_FLAG] == 0))
	{
	  foundEcg = 1;
	  aux_dbg_printLogStr( CONCAT_MODNAME( MOD_NAME ,"ECG Stream: PASS|\r\n"));
	}
	if ((foundBioz == 0) && (testing_ecg_flags[TESTING_BIOZ_FLAG] == 0))
	{
	  foundBioz = 1;
	  aux_dbg_printLogStr( CONCAT_MODNAME( MOD_NAME ,"Bioz Stream: PASS|\r\n"));
	}
	if ((foundPace == 0) && (testing_ecg_flags[TESTING_PACE_FLAG] == 0))
	{
	  foundPace = 1;
	  aux_dbg_printLogStr( CONCAT_MODNAME( MOD_NAME ,"PACE Stream: PASS|\r\n"));
	}
	if ((foundRtoR == 0) && (testing_ecg_flags[TESTING_RTOR_FLAG] == 0))
	{
	  foundRtoR = 1;
	  aux_dbg_printLogStr( CONCAT_MODNAME( MOD_NAME ,"RtoR Stream: PASS|\r\n"));
	}


	// stop all streams
	MAX30001_Stop_ECG(&max30001_obj_0);

	if (partVersion == 1)
	{
	  MAX30001_Stop_PACE();
	}
	if (partVersion == 1)
	{
	  MAX30001_Stop_BIOZ();
	}
	MAX30001_Stop_RtoR();

	testing_max30001 = false;

	// final results
	//_printPassFail((int)((foundEcg == 1) && (foundBioz == 1) && (foundPace == 1) && (foundRtoR == 1) && (partVersion == 1)), 0, aux_dbg_printLogStr);

	return 0;
}/* End of bit_MAX30001 */

#endif

/**
 * @brief MAX30001_hw_test
 * @brief hardware validation function
 * @return system error
 */
int MAX30001_hw_test(void * arg)
{
	uint32_t id=0xFF;
	int partVersion=0xFF; // 0 = 30004  1 = 30001  2 = 30002  3 = 30003
//	MAX30001_IO_t  io_ctx_0;

//	uint32_t foundEcg = 0;
//	uint32_t foundBioz = 0;
//	uint32_t foundPace = 0;
//	uint32_t foundRtoR = 0;

	// clear testing flags
//	testing_ecg_flags[TESTING_ECG_FLAG] = -1;
//	testing_ecg_flags[TESTING_BIOZ_FLAG] = -1;
//	testing_ecg_flags[TESTING_PACE_FLAG] = -1;
//	testing_ecg_flags[TESTING_RTOR_FLAG] = -1;

	/* ========= SPI bus initialization =========== */

	// bus registration
	if (MAX30001_RegisterBusIO(&max30001_obj_0) != MAX30001_OK)
	{
		return SYS_ERROR_BUS_FAILURE;
	}

	// read the id
	MAX30001_readID(&max30001_obj_0,&id);

	partVersion = id >> 12;
	partVersion = partVersion & 0x3;

	// start streams
	if (partVersion != 1)
	{
		return SYS_ERROR_BUS_FAILURE;
	}

	return SYS_ERROR_NONE;
}/* end of MAX30001_hw_test */



