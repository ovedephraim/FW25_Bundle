/**
  ******************************************************************************
  * @file           : sys_errno.h
  * @brief          : Error Code
  ******************************************************************************
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SYS_ERRNO_H
#define SYS_ERRNO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* System Common Error codes */
#define SYS_ERROR_NONE                        0
#define SYS_ERROR_NO_INIT                    -1
#define SYS_ERROR_WRONG_PARAM                -2
#define SYS_ERROR_BUSY                       -3
#define SYS_ERROR_PERIPH_FAILURE             -4
#define SYS_ERROR_UNKNOWN_FAILURE            -6
#define SYS_ERROR_UNKNOWN_COMPONENT          -7
#define SYS_ERROR_BUS_FAILURE                -8
#define SYS_ERROR_CLOCK_FAILURE              -9
#define SYS_ERROR_SELF_TEST_FAILURE          -10
#define SYS_ERROR_FEATURE_NOT_SUPPORTED      -11
#define SYS_ERROR_COMPONENT_FAILURE          -12
#define SYS_ERROR_FLASH_FAILURE              -13
#define SYS_ERROR_TIMEOUT                    -14

/* SYS BUS error codes */

#define SYS_ERROR_BUS_TRANSACTION_FAILURE    -100
#define SYS_ERROR_BUS_ARBITRATION_LOSS       -101
#define SYS_ERROR_BUS_ACKNOWLEDGE_FAILURE    -102
#define SYS_ERROR_BUS_PROTOCOL_FAILURE       -103

#define SYS_ERROR_BUS_MODE_FAULT             -104
#define SYS_ERROR_BUS_FRAME_ERROR            -105
#define SYS_ERROR_BUS_CRC_ERROR              -106
#define SYS_ERROR_BUS_DMA_FAILURE            -107

 /* Device Common Error codes */

#define SYS_ERROR_DEVICE_FAILURE            -200

 /* COM error codes */

#define SYS_ERROR_BT_INIT_FAILURE           -300


 /* @brief get verbose representation for error codes */
const char * get_sys_error_verb(int err);


#ifdef __cplusplus
}
#endif
#endif /*SYS_ERRNO_H */

