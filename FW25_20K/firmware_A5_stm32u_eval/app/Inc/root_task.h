/**************************************************************************//**
 * @file root_task.h
 * @brief Root task
 * @author Anton Kanaev
 * @version 0.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2021 AtlaSence, http://www.atasesne</b>
 *******************************************************************************
 *
 *
 ******************************************************************************/
#ifndef ROOT_TASK_H_
#define ROOT_TASK_H_

#define STACK_SIZE_FOR_AUX_RX_TASK     (configMINIMAL_STACK_SIZE + 10)
#define PRI_FOR_AUX_RX_TASK            (3)

#define STACK_SIZE_FOR_AUX_TX_TASK     (configMINIMAL_STACK_SIZE + 10)
#define PRI_FOR_AUX_TX_TASK            (4)

#define STACK_SIZE_FOR_BTMN_TASK       (configMINIMAL_STACK_SIZE + 10)
#define PRI_FOR_BTMN_TASK              (4)

#define STACK_SIZE_FOR_STST_TASK       (configMINIMAL_STACK_SIZE + 10)
#define PRI_FOR_STST_TASK              (4)

#define STACK_SIZE_FOR_CMDH_TASK       (configMINIMAL_STACK_SIZE + 10)
#define PRI_FOR_CMDH_TASK              (4)

#define STACK_SIZE_FOR_FOTA_TASK       (configMINIMAL_STACK_SIZE + 10)
#define PRI_FOR_FOTA_TASK              (4)

#define STACK_SIZE_FOR_LEDS_TASK       (configMINIMAL_STACK_SIZE + 10)
#define PRI_FOR_LEDS_TASK              (4)

#define STACK_SIZE_FOR_FUELGAUGE_TASK  (configMINIMAL_STACK_SIZE + 10)
#define PRI_FOR_FUELGAUGE_TASK         (4)

#define STACK_SIZE_FOR_DISP_TASK       (configMINIMAL_STACK_SIZE + 10)
#define PRI_FOR_DISP_TASK              (4)

#define STACK_SIZE_FOR_CHAN_TASK       (configMINIMAL_STACK_SIZE <<1)
#define PRI_FOR_CHAN_TASK              (4)//test

#define STACK_SIZE_FOR_SAMPLER_TASK    (configMINIMAL_STACK_SIZE *2)

void rootTask(void *para);


#endif /* ROOT_TASK_H_ */
