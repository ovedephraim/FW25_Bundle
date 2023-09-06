
#ifndef LSM6DSL_DEV_H_
#define LSM6DSL_DEV_H_

#include "lsm6dsl/lsm6dsl.h"
#include "lsm6dsl/lsm6dsl_reg.h"
#include <stdio.h>
#include <stdbool.h>

#define DEF_ODR_ACC (52)
#define DEF_ACC_AXE (3)

#define DEF_ODR_GYR (52)
#define DEF_GYR_AXE (3)

int LSM6DSL_hw_test(void * arg);

int LSM6DSL_dev_ACC_init_sream(void);
int LSM6DSL_dev_ACC_deinit_sream(void);
int LSM6DSL_dev_ACC_start_stream(void);
int LSM6DSL_dev_ACC_stop_stream(void);

int LSM6DSL_dev_GYR_init_sream(void);
int LSM6DSL_dev_GYR_deinit_sream(void);
int LSM6DSL_dev_GYR_start_stream(void);
int LSM6DSL_dev_GYR_stop_stream(void);

int LSM6DSL_dev_handle_stream(void * p);

#endif /* LSM6DSL_DEV_H_ */
