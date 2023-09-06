/**************************************************************************//**
* @file io_ctl.c
* @brief io control functions
* @author Anton Kanaev
* @version 0.0.1
* @date 09.10.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include "io_ctl.h"
#include "main.h"
#include "board.h"




/**
 * @brief io_ctl_enable_Power
 * @brief This function used for enable
 * board power supply
 * @param p[in/out] - none
 * @returns none
 */
void io_ctl_enable_power(void)
{
	HAL_GPIO_WritePin(EN_1V8_GPIO_PORT,
			EN_1V8_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(EN_2V8_GPIO_PORT,
			EN_2V8_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(EN_3V3_GPIO_PORT,
			EN_3V3_PIN, GPIO_PIN_SET);
}/* end of io_ctl_enable_Power */


#if 0
/**
 * @brief io_ctl_capsense
 * @brief This function used for enable
 * cap sense selection
 * @param sel[in] - 1-high 0 - low
 * @returns none
 */
void io_ctl_capsense(uint8_t sel)
{
    switch(sel)
    {
		case 0:
		HAL_GPIO_WritePin(CAPSENSE_EN_GPIO_PORT,
				CAPSENSE_EN_GPIO_PIN, GPIO_PIN_RESET);
		break;
		default:
		HAL_GPIO_WritePin(CAPSENSE_EN_GPIO_PORT,
				CAPSENSE_EN_GPIO_PIN, GPIO_PIN_SET);
		break;
    }
}/* end of io_ctl_capsense */
#endif

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void io_ctl_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* TOD replace GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
/*
	//debug pin
	GPIO_InitStruct.Pin                               =  DEBUG_PIN;
	GPIO_InitStruct.Mode                              =  DEBUG_GPIO_MODE;
	GPIO_InitStruct.Pull                              =  DEBUG_GPIO_PULL;
	GPIO_InitStruct.Speed                             =  GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DEBUG_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin                               = RX_BT_GPIO_PIN;
	GPIO_InitStruct.Mode                              = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull                              = RX_BT_GPIO_PULL;
	GPIO_InitStruct.Speed                             = RX_BT_GPIO_SPEED;
	HAL_GPIO_Init(RX_BT_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin                               = TX_BT_GPIO_PIN;
	GPIO_InitStruct.Mode                              = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull                              = TX_BT_GPIO_PULL;
	GPIO_InitStruct.Speed                             = TX_BT_GPIO_SPEED;
	HAL_GPIO_Init(TX_BT_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin                               = MAX30001_CS_PIN;
	GPIO_InitStruct.Mode                              = MAX30001_CS_GPIO_MODE;
	GPIO_InitStruct.Pull                              = MAX30001_CS_GPIO_PULL;
	GPIO_InitStruct.Speed                             = MAX30001_CS_GPIO_SPEED;
	HAL_GPIO_Init(MAX30001_CS_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin                               = MAX30003_CS_PIN;
	GPIO_InitStruct.Mode                              = MAX30003_CS_GPIO_MODE;
	GPIO_InitStruct.Pull                              = MAX30003_CS_GPIO_PULL;
	GPIO_InitStruct.Speed                             = MAX30003_CS_GPIO_SPEED;
	HAL_GPIO_Init(MAX30003_CS_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin                               = MAX3000N_FCLK_GPIO_PIN;
	GPIO_InitStruct.Mode                              = MAX3000N_FCLK_GPIO_MODE;
	GPIO_InitStruct.Pull                              = MAX3000N_FCLK_GPIO_PULL;
	GPIO_InitStruct.Speed                             = MAX3000N_FCLK_GPIO_SPEED;
	GPIO_InitStruct.Alternate                         = MAX3000N_FCLK_GPIO_AF;
	HAL_GPIO_Init(MAX3000N_FCLK_GPIO_PORT, &GPIO_InitStruct);
*/
    GPIO_InitStruct.Pin                               = EN_1V8_PIN;
    GPIO_InitStruct.Mode                              = EN_1V8_GPIO_MODE;
    GPIO_InitStruct.Pull                              = EN_1V8_GPIO_PULL;
    GPIO_InitStruct.Speed                             = EN_1V8_GPIO_SPEED;
    HAL_GPIO_Init(EN_1V8_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin                               = EN_3V3_PIN;
    GPIO_InitStruct.Mode                              = EN_3V3_GPIO_MODE;
    GPIO_InitStruct.Pull                              = EN_3V3_GPIO_PULL;
    GPIO_InitStruct.Speed                             = EN_3V3_GPIO_SPEED;
    HAL_GPIO_Init(EN_3V3_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin                               = EN_2V8_PIN;
    GPIO_InitStruct.Mode                              = EN_2V8_GPIO_MODE;
    GPIO_InitStruct.Pull                              = EN_2V8_GPIO_PULL;
    GPIO_InitStruct.Speed                             = EN_2V8_GPIO_SPEED;
    HAL_GPIO_Init(ECG_HPF0_GPIO_PORT, &GPIO_InitStruct);
/*
    GPIO_InitStruct.Pin                               = ECG_HPF0_PIN;
    GPIO_InitStruct.Mode                              = ECG_HPF0_GPIO_MODE;
    GPIO_InitStruct.Pull                              = ECG_HPF0_GPIO_PULL;
    GPIO_InitStruct.Speed                             = ECG_HPF0_GPIO_SPEED;
    HAL_GPIO_Init(ECG_HPF0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin                               = ECG_HPF1_PIN;
    GPIO_InitStruct.Mode                              = ECG_HPF1_GPIO_MODE;
    GPIO_InitStruct.Pull                              = ECG_HPF1_GPIO_PULL;
    GPIO_InitStruct.Speed                             = ECG_HPF1_GPIO_SPEED;
    HAL_GPIO_Init(ECG_HPF1_GPIO_PORT, &GPIO_InitStruct);


    //LED_RST
    GPIO_InitStruct.Pin                               = LED_RST_GPIO_PIN;
    GPIO_InitStruct.Mode                              = LED_RST_GPIO_MODE;
    GPIO_InitStruct.Pull                              = LED_RST_GPIO_PULL;
    GPIO_InitStruct.Speed                             = LED_RST_GPIO_SPEED;
    HAL_GPIO_Init(LED_RST_GPIO_PORT, &GPIO_InitStruct);

    //CAPSENSE_EN
    GPIO_InitStruct.Pin                               = CAPSENSE_EN_GPIO_PIN;
    GPIO_InitStruct.Mode                              = CAPSENSE_EN_GPIO_MODE;
    GPIO_InitStruct.Pull                              = CAPSENSE_EN_GPIO_PULL;
    GPIO_InitStruct.Speed                             = CAPSENSE_EN_GPIO_SPEED;
    HAL_GPIO_Init(CAPSENSE_EN_GPIO_PORT, &GPIO_InitStruct);

    //BUZZER_EN
    GPIO_InitStruct.Pin                               = BUZZER_EN_GPIO_PIN;
    GPIO_InitStruct.Mode                              = BUZZER_EN_GPIO_MODE;
    GPIO_InitStruct.Pull                              = BUZZER_EN_GPIO_PULL;
    GPIO_InitStruct.Speed                             = BUZZER_EN_GPIO_SPEED;
    HAL_GPIO_Init(BUZZER_EN_GPIO_PORT, &GPIO_InitStruct);

    //HEAT_EN
    GPIO_InitStruct.Pin                               = HEAT_EN_GPIO_PIN;
    GPIO_InitStruct.Mode                              = HEAT_EN_GPIO_MODE;
    GPIO_InitStruct.Pull                              = HEAT_EN_GPIO_PULL;
    GPIO_InitStruct.Speed                             = HEAT_EN_GPIO_SPEED;
    HAL_GPIO_Init(HEAT_EN_GPIO_PORT, &GPIO_InitStruct);

    //CHG_STAT1
    GPIO_InitStruct.Pin                               = CHG_STAT1_GPIO_PIN;
    GPIO_InitStruct.Mode                              = CHG_STAT1_GPIO_MODE;
    GPIO_InitStruct.Pull                              = CHG_STAT1_GPIO_PULL;
    GPIO_InitStruct.Speed                             = CHG_STAT1_GPIO_SPEED;
    HAL_GPIO_Init(CHG_STAT1_GPIO_PORT, &GPIO_InitStruct);

    //CHG_STAT2
    GPIO_InitStruct.Pin                               = CHG_STAT2_GPIO_PIN;
    GPIO_InitStruct.Mode                              = CHG_STAT2_GPIO_MODE;
    GPIO_InitStruct.Pull                              = CHG_STAT2_GPIO_PULL;
    GPIO_InitStruct.Speed                             = CHG_STAT2_GPIO_SPEED;
    HAL_GPIO_Init(CHG_STAT2_GPIO_PORT, &GPIO_InitStruct);

    //BAT_ALRT
    GPIO_InitStruct.Pin                               = BAT_ALRT_GPIO_PIN;
    GPIO_InitStruct.Mode                              = BAT_ALRT_GPIO_MODE;
    GPIO_InitStruct.Pull                              = BAT_ALRT_GPIO_PULL;
    GPIO_InitStruct.Speed                             = BAT_ALRT_GPIO_SPEED;
    HAL_GPIO_Init(BAT_ALRT_GPIO_PORT, &GPIO_InitStruct);

    //CAP_GND
    GPIO_InitStruct.Pin                               = CAP_GND_GPIO_PIN;
    GPIO_InitStruct.Mode                              = CAP_GND_GPIO_MODE;
    GPIO_InitStruct.Pull                              = CAP_GND_GPIO_PULL;
    GPIO_InitStruct.Speed                             = CAP_GND_GPIO_SPEED;
    HAL_GPIO_Init(CAP_GND_GPIO_PORT, &GPIO_InitStruct);

    //SHIELD_GND
    GPIO_InitStruct.Pin                               = SHIELD_GND_GPIO_PIN;
    GPIO_InitStruct.Mode                              = SHIELD_GND_GPIO_MODE;
    GPIO_InitStruct.Pull                              = SHIELD_GND_GPIO_PULL;
    GPIO_InitStruct.Speed                             = SHIELD_GND_GPIO_SPEED;
    HAL_GPIO_Init(SHIELD_GND_GPIO_PORT, &GPIO_InitStruct);
*/
#if 0
    //TP13 DEBUG
	GPIO_InitStruct.Pin                               = GPIO_PIN_11;
	GPIO_InitStruct.Mode                              = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull                              = GPIO_PULLUP;
	GPIO_InitStruct.Speed                             = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#endif

}/* End of io_ctl_gpio_Init */




