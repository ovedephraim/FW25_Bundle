
#ifndef TMP117_DEV_H_
#define TMP117_DEV_H_

#include <stdio.h>
#include <stdbool.h>


#define DEF_ODR_TMP (1)
#define DEF_AXE_TMP (3)

int tmp117_hw_test(void * arg);

int TEMPX_dev_init_sream(void);
int TEMPX_dev_start_stream(void);
int TEMPX_dev_stop_stream(void);
int TEMPX_dev_handle_stream(void * p);
int TEMPX_dev_proc_stream(void);

void TEMPX_HeaterEnable(bool en);

#endif /* TMP117_DEV_H_ */
