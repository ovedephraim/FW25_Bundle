/**************************************************************************//**
* @file BL653.h
* @brief BL653 module
* @author Anton Kanaev
* @version 0.0.1
* @date 29.07.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef BL653_H_
#define BL653_H_

extern const char at_empty_line[];
extern const char ar_mod_reset[];


extern const char at_get_s_reg[];
extern const char at_get_btname[];
extern const char at_get_fw_ver[];
extern const char at_get_ap_ver[];
extern const char btname_resp[];
extern const char at_warm_reset[];
extern const char at_save_regmp[];

extern const char getsup_UART_Transmit_Buffer_Size[];
extern const char getcur_UART_Transmit_Buffer_Size[];

extern const char getsup_UART_Receive_Buffer_Size[];
extern const char getcur_UART_Receive_Buffer_Size[];

extern const char getsup_VSP_Transmit_Buffer_Size[];
extern const char getcur_VSP_Transmit_Buffer_Size[];

extern const char getsup_VSP_Receive_Buffer_Size[];
extern const char getcur_VSP_Receive_Buffer_Size[];

extern const char getsup_UART_Baud_rate[];
extern const char getcur_UART_Baud_rate[];
extern const char set_UART_Baud_rate921600[];

extern const char unsolicited_vsp_connected[];
extern const char unsolicited_vsp_disconnected[];

int BL653_hw_test(void * arg);

#endif /* BL653_H_ */
