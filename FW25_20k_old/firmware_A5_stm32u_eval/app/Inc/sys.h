/**
  ******************************************************************************
  * @file           : sys.h
  * @brief          : system header file
  ******************************************************************************
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SYS_H
#define SYS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

typedef struct	{
	uint8_t is_streaming;
}SYS_status_t;

typedef struct	{
	uint8_t is_imu_acc;
	uint8_t is_imu_gyr;
	uint8_t is_temp;
}SYS_config_t;

 typedef struct	{
 	SYS_status_t	st;
 	SYS_config_t	cf;
 }SYS_param_t;

 extern SYS_param_t sys_param;


#ifdef __cplusplus
}
#endif
#endif  /* SYS_H */


