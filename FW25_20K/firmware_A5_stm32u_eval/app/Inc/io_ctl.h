/**************************************************************************//**
* @file io_ctl.h
* @brief io control functions
* @author Anton Kanaev
* @version 0.0.1
* @date 09.10.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#ifndef IO_CTL_H_
#define IO_CTL_H_

#include <stdint.h>

//00 (5Hz), 01 (0.5Hz, def.) and 10 (0.05Hz)
typedef enum ecg_hpf_conf
{
	ecg_HPF_5HZ   = 0,
	ecg_HPF_05HZ  = 1,
	ecg_HPF_005HZ = 2
}ecg_hpf_conf_t;

void io_ctl_gpio_init(void);
void io_ctl_enable_power(void);
void io_ctl_set_hpf_freq(ecg_hpf_conf_t f);

void io_ctl_capsense(uint8_t sel);

#endif /* IO_CTL_H_ */
