
#ifndef LIS2DW_DEV_H_
#define LIS2DW_DEV_H_

#include "lis2dw12/lis2dw12.h"
#include <stdio.h>
#include <stdbool.h>

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} MOTION_SENSOR_Axes_t;


void LIS2DW_bit_func(void *para);

int LIS2DW_dev_init_sream(void);
int LIS2DW_dev_deinit_sream(void);

int LIS2DW_dev_PULSE_start_stream(void);
int LIS2DW_dev_PULSE_stop_stream(void);
int LIS2DW_dev_PULSE_handle_stream(void * p);

int LIS2DW_hw_test(void * arg);

#endif /* LIS2DW_DEV_H_ */
