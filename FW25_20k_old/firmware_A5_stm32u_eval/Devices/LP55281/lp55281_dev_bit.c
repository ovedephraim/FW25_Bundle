

#ifdef __cplusplus
extern "C" {
#endif


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <freertos.h>
#include <task.h>
#include <queue.h>
#include "main.h"
#include "LP55281/lp55281.h"


static
int LED_Production_Test_Procedure(void)
{
	int ret=0;

	/* Give reset to LP55281 */
	LP55281_reset ();

	/* production connections test */
	ret=LP55281_Test_Procedure();


return ret;
}

static
int32_t LED_Testleds(void)
{
	int32_t ret = 0;

	LP55281_reset ();

	LP55281_green(GREEN1, false,1,0x1c);
	vTaskDelay(100);
    LP55281_green(GREEN1, false,2,0x1c);
    vTaskDelay(100);
    LP55281_green(GREEN1, false,3,0x1c);
    vTaskDelay(100);
    LP55281_green(GREEN1, false,4,0x1c);
    vTaskDelay(100);

    LP55281_reset ();

    LP55281_red(RED1, false,1,0x1c);
	vTaskDelay(100);
	LP55281_red(RED1, false,2,0x1c);
	vTaskDelay(100);
	LP55281_red(RED1, false,3,0x1c);
	vTaskDelay(100);
	LP55281_red(RED1, false,4,0x1c);
	vTaskDelay(100);

	LP55281_reset ();

	LP55281_blue(BLUE1, false,1,0x1c);
	vTaskDelay(100);
	LP55281_blue(BLUE1, false,2,0x1c);
	vTaskDelay(100);
	LP55281_blue(BLUE1, false,3,0x1c);
	vTaskDelay(100);
	LP55281_blue(BLUE1, false,4,0x1c);

	LP55281_reset ();

	return ret;
}

/**
 * @brief LP55281_hw_test
 * @brief harware validation function
 * @return system error
 */
int LP55281_hw_test(void * arg)
{
	int rv=SYS_ERROR_NONE;


	/* Configure the lp55281 driver */
	LP55281_Init();

	/* blink leds */
	LED_Testleds();

    /* execute production test */
	LED_Production_Test_Procedure();

	/* disable hardware interface */
	LP55281_Denit();

	return rv;
}/* End of LP55281_hw_test */

#ifdef __cplusplus
}
#endif
