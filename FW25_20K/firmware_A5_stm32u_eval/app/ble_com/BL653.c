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



#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "BL653.h"

const char at_empty_line[]= "at\r";//resp OK
const char ar_mod_reset[]=  "atz\r";


/* In VSP mode (the default), where bit 0 of S register 100 is set */
const char at_get_s_reg[]= "ats 100?\r";//01 E002 - AT interface App is not running

/*
Reg Description
0 The value of #define AT_RESPONSE_0 in the source code. E.g: ”BL654”
3 The firmware version of the module (see n=23 for version of app)
4 The Bluetooth address of the module as a 14-digit hex string
10 The value of #define AT_RESPONSE_10 in the source code. E.g: “Laird
Connectivity, (c) 2020”
11 Will return 1 if low power UART operation variant of this application has been
loaded.
23 The version of the smartBASIC application, which is the value of #define LibVer
33 The version of the smartBASIC application, which is the value of #define AppVer and
can be changed by the customer in top level .sb file
42 The value of the current state of a state machine implemented by the app.
50 Count of NFC Coil energise/deenergise events.
Will be an odd number if the coil is still energised.
When this count is read, all bits except bit 0 is reset.
Wraps to 0 after 2^31
51 Count of the number of times the NFC Tag has been read.
When this count is read, it will be reset
Wraps to 0 after 2^31
2009 Number of devices in the trusted device bond database
2012 Maximum number of devices that can be saved in the trusted device bond database
*/

const char at_get_btname[]= "ati 0\r";
const char btname_resp[]= "BL653";

/* FW 30.2.2.0 -> AT AP 3.01
   FW 30.2.3.0 -> AT AP 4.02 */

const char at_get_fw_ver[]= "ati 3\r";
const char at_get_ap_ver[]= "ati 33\r";

const char at_warm_reset[]= "atz\r";
const char at_save_regmp[]= "at&w\r";

/* UART related at commands */

const char getsup_UART_Transmit_Buffer_Size[]= "ats 202?\r";
const char getcur_UART_Transmit_Buffer_Size[]= "ats 202=?\r";

const char getsup_UART_Receive_Buffer_Size[]= "ats 203?\r";
const char getcur_UART_Receive_Buffer_Size[]= "ats 203=?\r";

const char getsup_VSP_Transmit_Buffer_Size[]= "ats 204?\r";
const char getcur_VSP_Transmit_Buffer_Size[]= "ats 204=?\r";

const char getsup_VSP_Receive_Buffer_Size[]= "ats 205?\r";
const char getcur_VSP_Receive_Buffer_Size[]= "ats 205=?\r";

const char getsup_UART_Baud_rate[]= "ats 302?\r";
const char getcur_UART_Baud_rate[]= "ats 302=?\r";
const char set_UART_Baud_rate921600[]= "ats 302=921600\r";





const char unsolicited_vsp_connected[]= "CONNECT";
const char unsolicited_vsp_disconnected[]= "NOCARRIER";




