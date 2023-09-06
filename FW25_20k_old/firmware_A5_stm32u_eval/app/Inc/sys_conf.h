/**
  ******************************************************************************
  * @file           : ssys_conf.h
  * @brief          : Configuration file
  ******************************************************************************
  ******************************************************************************
*/

#ifndef SYS_CONF_H
#define SYS_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

 //==================================================================================|
 //===================== hw channels configuration ==================================|

#define ENABLE_ECG_H_CHAN      1U //enable horizontal ecg
#define ENABLE_PULSE_CHAN      1U //enable pulse
#define ENABLE_ACC3X_CHAN      1U //enable 3D acceleretion
#define ENABLE_ECG_V_CHAN      1U //enable vertical ecg
#define ENABLE_TEMP_SKIN_CHAN  1U //enable temperature sensor
#define ENABLE_AN_PACE_CHAN    1U
#define ENABLE_AN_EMG_CHAN     1U
#define ENABLE_AN_SCL_CHAN     1U
#define ENABLE_AN_RESP_CHAN    1U
#define ENABLE_AN_AUDIO_CHAN   1U
#define ENABLE_GYR3X_CHAN      1U
#define ENABLE_BIOZ_CHAN       1U

//===================================================================================|
//===================== uart mode/baud rate configuration ===========================|

#define DEBUG_AUX_BAUD              (921600)
#define DEF_BT_BAUD                 (115200)
#define OPR_BT_BAUD                 (230400)


#define ENABLE_DBG_COM_AUX          (1U) //- enable debug log output via serial

//#endif

#define ENABLE_BLE_COM_AUX          (1U) //- set to 0 when externally flashing BL653
#define RUN_BLE_APP_ON_BOOT         (1U) //- set value to bt autorun pin io

//===================================================================================|
//===================== i2c frequency configuration =================================|

#define BUS_I2C2_FREQUENCY          (100000U) /* TODO Frequency of I2C1 = 100 KHz*/

//===================================================================================|
//===================== serial logger/data configuration ============================|

#if (defined ENABLE_DBG_COM_AUX) && (ENABLE_DBG_COM_AUX == 1)

	#define ENABLE_PROTO_SERIAL_LOG     (0U) //enable data streaming via debug serial
	#define ENABLE_DEBUG_SERIAL_LOG     (1U) //enable debug log printouts (debug,error...)

	#if (defined ENABLE_DEBUG_SERIAL_LOG) && (ENABLE_DEBUG_SERIAL_LOG == 1)
		#define ENABLE_RECORD_DEBUG_LOG     (0U) //enable record specific debug
		#define ENABLE_THROUGHPUT_DEBUG_LOG (0U) //enable throughput  debug
		#define ENABLE_CHAN_ACLEN_DEBUG_LOG (0U)
	#endif

#else
	#define ENABLE_PROTO_SERIAL_LOG     (0U) //enable data streaming via debug serial
	#define ENABLE_DEBUG_SERIAL_LOG     (0U) //enable debug log printouts (debug,error...)

#endif



#define DEBUG_ENV_CAL_MAX1          (0U) //enable calibration signal ECG max30001
#define DEBUG_ENV_SAW_MAX1          (0U) //enable saw tooth signal ECG max30001
#define DEBUG_BIOZ_MAX1             (0U)

#define DEBUG_ENV_SAW_MAX3          (0U) //enable calibration signal ECG max30003
#define DEBUG_ENV_CAL_MAX3          (0U) //enable saw tooth signal ECG max30003

#ifdef __cplusplus
}
#endif
#endif  /* SYS_CONF_H */


