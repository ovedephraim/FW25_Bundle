/**
* @file board.h
* @brief A5 board I/O pins definitions
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 18.06.2022
*/
#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif



/* EN_1V8 ->PE12 */
#define EN_1V8_PIN                             GPIO_PIN_12
#define EN_1V8_GPIO_PORT                       GPIOE
#define EN_1V8_GPIO_MODE                       GPIO_MODE_OUTPUT_PP
#define EN_1V8_GPIO_PULL                       GPIO_PULLUP
#define EN_1V8_GPIO_SPEED                      GPIO_SPEED_FREQ_LOW;
#define EN_1V8_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOE_CLK_ENABLE()
#define EN_1V8_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOE_CLK_DISABLE()

/* EN_2V8 ->PE12 */
#define EN_2V8_PIN                             GPIO_PIN_8
#define EN_2V8_GPIO_PORT                       GPIOE
#define EN_2V8_GPIO_MODE                       GPIO_MODE_OUTPUT_PP
#define EN_2V8_GPIO_PULL                       GPIO_PULLUP
#define EN_2V8_GPIO_SPEED                      GPIO_SPEED_FREQ_LOW;
#define EN_2V8_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOE_CLK_ENABLE()
#define EN_2V8_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOE_CLK_DISABLE()

/* EN_3V3 ->PE12 */
#define EN_3V3_PIN                             GPIO_PIN_13
#define EN_3V3_GPIO_PORT                       GPIOE
#define EN_3V3_GPIO_MODE                  	   GPIO_MODE_OUTPUT_PP
#define EN_3V3_GPIO_PULL                  	   GPIO_PULLUP
#define EN_3V3_GPIO_SPEED                      GPIO_SPEED_FREQ_LOW;
#define EN_3V3_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOE_CLK_ENABLE()
#define EN_3V3_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOE_CLK_DISABLE()

//=============================== [ MAX30001 ] ===============================|

/* INT_30001 ->PE14 */
#define MAX30001_INT_PIN                       GPIO_PIN_14
#define MAX30001_INT_GPIO_PORT                 GPIOE
#define MAX30001_INT_SPEED                     GPIO_SPEED_FREQ_VERY_HIGH
#define MAX30001_INT_PULL                      GPIO_PULLUP
#define MAX30001_INT_MODE                      GPIO_MODE_IT_FALLING
#define MAX30001_INT_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOE_CLK_ENABLE()
#define MAX30001_INT_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOE_CLK_DISABLE()
#define MAX30001_INT_EXTI_IRQn                 EXTI14_IRQn
#define MAX30001_INT_EXTI_LINE                 EXTI_LINE_14

/* INT2_30001 ->PE15 */
#define MAX30001_INT2_PIN                      GPIO_PIN_15
#define MAX30001_INT2_GPIO_PORT                GPIOE
#define MAX30001_INT2_SPEED                    GPIO_SPEED_FREQ_VERY_HIGH
#define MAX30001_INT2_PULL                     GPIO_PULLUP
#define MAX30001_INT2_MODE                     GPIO_MODE_IT_FALLING
#define MAX30001_INT2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()
#define MAX30001_INT2_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOE_CLK_DISABLE()
#define MAX30001_INT2_EXTI_IRQn                EXTI15_IRQn
#define MAX30001_INT2_EXTI_LINE                EXTI_LINE_15

/* CS#_30001 ->PB9 (chip select) */
#define MAX30001_CS_PIN                        GPIO_PIN_9
#define MAX30001_CS_GPIO_PORT                  GPIOB
#define MAX30001_CS_GPIO_MODE                  GPIO_MODE_OUTPUT_PP
#define MAX30001_CS_GPIO_PULL                  GPIO_NOPULL
#define MAX30001_CS_GPIO_SPEED                 GPIO_SPEED_FREQ_VERY_HIGH;
#define MAX30001_CS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define MAX30001_CS_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOB_CLK_DISABLE()

//=============================== [ MAX30003 ] ===============================|

/* INT_30003 ->PF11 */
#define MAX30003_INT_PIN                       GPIO_PIN_11
#define MAX30003_INT_GPIO_PORT                 GPIOF
#define MAX30003_INT_SPEED                     GPIO_SPEED_FREQ_VERY_HIGH
#define MAX30003_INT_PULL                      GPIO_PULLUP
#define MAX30003_INT_MODE                      GPIO_MODE_IT_FALLING
#define MAX30003_INT_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()
#define MAX30003_INT_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOF_CLK_DISABLE()
#define MAX30003_INT_EXTI_IRQn                 EXTI11_IRQn
#define MAX30003_INT_EXTI_LINE                 EXTI_LINE_11

/* INT2_30003 ->PF12 */
#define MAX30003_INT2_PIN                      GPIO_PIN_12
#define MAX30003_INT2_GPIO_PORT                GPIOF
#define MAX30003_INT2_SPEED                    GPIO_SPEED_FREQ_VERY_HIGH
#define MAX30003_INT2_PULL                     GPIO_PULLUP
#define MAX30003_INT2_MODE                     GPIO_MODE_IT_FALLING
#define MAX30003_INT2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOF_CLK_ENABLE()
#define MAX30003_INT2_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOF_CLK_DISABLE()
#define MAX30003_INT2_EXTI_IRQn                EXTI12_IRQn
#define MAX30003_INT2_EXTI_LINE                EXTI_LINE_12


/* CS#_30003 ->PC5 (chip select) */
#define MAX30003_CS_PIN                        GPIO_PIN_5
#define MAX30003_CS_GPIO_PORT                  GPIOC
#define MAX30003_CS_GPIO_MODE                  GPIO_MODE_OUTPUT_PP
#define MAX30003_CS_GPIO_PULL                  GPIO_NOPULL
#define MAX30003_CS_GPIO_SPEED                 GPIO_SPEED_FREQ_VERY_HIGH
#define MAX30003_CS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()
#define MAX30003_CS_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOC_CLK_DISABLE()


//================================================================================|
//=========================== [ SPI BUS definitions ] ============================|
//================================================================================|

/* SPI2_MISO -> PC2 [MAX30001] */
#define BUS_SPI2_MISO_GPIO_PORT                GPIOC
#define BUS_SPI2_MISO_GPIO_PIN                 GPIO_PIN_2
#define BUS_SPI2_MISO_GPIO_MODE                GPIO_MODE_AF_PP
#define BUS_SPI2_MISO_GPIO_AF                  GPIO_AF5_SPI2
#define BUS_SPI2_MISO_PULL                     GPIO_NOPULL
#define BUS_SPI2_MISO_GPIO_SPEED               GPIO_SPEED_FREQ_VERY_HIGH
#define BUS_SPI2_MISO_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOC_CLK_DISABLE()
#define BUS_SPI2_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

/* SPI2_MOSI -> PC3 [MAX30001] */
#define BUS_SPI2_MOSI_GPIO_PIN                 GPIO_PIN_3
#define BUS_SPI2_MOSI_GPIO_PORT                GPIOC
#define BUS_SPI2_MOSI_GPIO_MODE                GPIO_MODE_AF_PP
#define BUS_SPI2_MOSI_GPIO_AF                  GPIO_AF5_SPI2
#define BUS_SPI2_MOSI_PULL                     GPIO_NOPULL
#define BUS_SPI2_MOSI_GPIO_SPEED               GPIO_SPEED_FREQ_VERY_HIGH
#define BUS_SPI2_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
#define BUS_SPI2_MOSI_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOC_CLK_DISABLE()

/* SPI2_SCLK -> PB10 [MAX300001] */
#define BUS_SPI2_SCLK_GPIO_PORT                GPIOB
#define BUS_SPI2_SCLK_GPIO_PIN                 GPIO_PIN_10
#define BUS_SPI2_SCLK_GPIO_MODE                GPIO_MODE_AF_PP
#define BUS_SPI2_SCLK_GPIO_AF                  GPIO_AF5_SPI2
#define BUS_SPI2_SCLK_PULL                     GPIO_NOPULL
#define BUS_SPI2_SCLK_GPIO_SPEED               GPIO_SPEED_FREQ_VERY_HIGH
#define BUS_SPI2_SCLK_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_SPI2_SCLK_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE()

//================================================================================|
//=========================== [ I2C BUS definitions ] ============================|
//================================================================================|

// I2C2 - Devices: LIS2DW, LSM6DSL -----------------------------------------------|
#define BUS_I2C2_SCL_GPIO_PORT                 GPIOF
#define BUS_I2C2_SCL_GPIO_AF                   GPIO_AF4_I2C2
#define BUS_I2C2_SCL_GPIO_PIN                  GPIO_PIN_1
#define BUS_I2C2_SCL_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOF_CLK_DISABLE()
#define BUS_I2C2_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()

#define BUS_I2C2_SDA_GPIO_PORT                 GPIOF
#define BUS_I2C2_SDA_GPIO_PIN                  GPIO_PIN_0
#define BUS_I2C2_SDA_GPIO_AF                   GPIO_AF4_I2C2
#define BUS_I2C2_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()
#define BUS_I2C2_SDA_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOF_CLK_DISABLE()

// I2C4 - Devices: LP55281, TMP117/AS6221, MAX17303, LED Driver, Fuel Gauge ------|
#define BUS_I2C4_SCL_GPIO_PORT                 GPIOF
#define BUS_I2C4_SCL_GPIO_AF                   GPIO_AF4_I2C4
#define BUS_I2C4_SCL_GPIO_PIN                  GPIO_PIN_14
#define BUS_I2C4_SCL_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOF_CLK_DISABLE()
#define BUS_I2C4_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()

#define BUS_I2C4_SDA_GPIO_PORT                 GPIOF
#define BUS_I2C4_SDA_GPIO_PIN                  GPIO_PIN_15
#define BUS_I2C4_SDA_GPIO_AF                   GPIO_AF4_I2C4
#define BUS_I2C4_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()
#define BUS_I2C4_SDA_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOF_CLK_DISABLE()

// I2C3 - Devices: external I2C for R&D TODO to check IOS ------------------------|
#define BUS_I2C3_SCL_GPIO_PORT                 GPIOG
#define BUS_I2C3_SCL_GPIO_AF                   GPIO_AF4_I2C3
#define BUS_I2C3_SCL_GPIO_PIN                  GPIO_PIN_7
#define BUS_I2C3_SCL_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOG_CLK_DISABLE()
#define BUS_I2C3_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOG_CLK_ENABLE()

#define BUS_I2C3_SDA_GPIO_PORT                 GPIOG
#define BUS_I2C3_SDA_GPIO_PIN                  GPIO_PIN_8
#define BUS_I2C3_SDA_GPIO_AF                   GPIO_AF4_I2C3
#define BUS_I2C3_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOG_CLK_ENABLE()
#define BUS_I2C3_SDA_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOG_CLK_DISABLE()

//================================================================================|
//=========================== [ IO pins definitions ] ============================|
//================================================================================|

// ECG_HPF0 -> PE2 GPO select ECG HPF --------------------------------------------|
#define ECG_HPF0_PIN                           GPIO_PIN_2
#define ECG_HPF0_GPIO_PORT                     GPIOE
#define ECG_HPF0_GPIO_MODE                     GPIO_MODE_OUTPUT_PP
#define ECG_HPF0_GPIO_PULL                     GPIO_PULLUP
#define ECG_HPF0_GPIO_SPEED                    GPIO_SPEED_FREQ_LOW;
#define ECG_HPF0_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define ECG_HPF0_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOE_CLK_DISABLE()

// ECG_HPF1 -> PE3 GPO select ECG HPF --------------------------------------------|
#define ECG_HPF1_PIN                           GPIO_PIN_3
#define ECG_HPF1_GPIO_PORT                     GPIOE
#define ECG_HPF1_GPIO_MODE                     GPIO_MODE_OUTPUT_PP
#define ECG_HPF1_GPIO_PULL                     GPIO_PULLUP
#define ECG_HPF1_GPIO_SPEED                    GPIO_SPEED_FREQ_LOW;
#define ECG_HPF1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define ECG_HPF1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOE_CLK_DISABLE()

// F_CLK ->PA8 (F_CLK) LSE clock out from MCO pin (div1) -------------------------|
#define MAX3000N_FCLK_GPIO_PIN                 GPIO_PIN_8
#define MAX3000N_FCLK_GPIO_PORT                GPIOA
#define MAX3000N_FCLK_GPIO_MODE                GPIO_MODE_AF_PP
#define MAX3000N_FCLK_GPIO_AF                  GPIO_AF0_MCO
#define MAX3000N_FCLK_GPIO_PULL                GPIO_NOPULL
#define MAX3000N_FCLK_GPIO_SPEED               GPIO_SPEED_FREQ_VERY_HIGH
#define MAX3000N_FCLK_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define MAX3000N_FCLK_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOA_CLK_DISABLE()

// SCL_RNG ->PE10 GPO set SCL current drive range --------------------------------|
#define SCL_RNG_PIN                            GPIO_PIN_10
#define SCL_RNG_GPIO_PORT                      GPIOE
#define SCL_RNG_GPIO_MODE                      GPIO_MODE_OUTPUT_PP
#define SCL_RNG_GPIO_PULL                      GPIO_PULLUP
#define SCL_RNG_GPIO_SPEED                     GPIO_SPEED_FREQ_LOW
#define SCL_RNG_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOE_CLK_ENABLE()
#define SCL_RNG_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOE_CLK_DISABLE()

// LED_RST ->PF13 GPO reset led driver -------------------------------------------|
#define LED_RST_GPIO_PIN                       GPIO_PIN_13
#define LED_RST_GPIO_PORT                      GPIOF
#define LED_RST_GPIO_MODE                      GPIO_MODE_OUTPUT_PP
#define LED_RST_GPIO_PULL                      GPIO_PULLUP
#define LED_RST_GPIO_SPEED                     GPIO_SPEED_FREQ_MEDIUM
#define LED_RST_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED_RST_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOE_CLK_DISABLE()

// CAPSENSE_EN ->PF2 GPO select cap sense ----------------------------------------|
#define CAPSENSE_EN_GPIO_PIN                   GPIO_PIN_2
#define CAPSENSE_EN_GPIO_PORT                  GPIOF
#define CAPSENSE_EN_GPIO_MODE                  GPIO_MODE_OUTPUT_PP
#define CAPSENSE_EN_GPIO_PULL                  GPIO_PULLUP
#define CAPSENSE_EN_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()
#define CAPSENSE_EN_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOF_CLK_DISABLE()
#define CAPSENSE_EN_GPIO_SPEED                 GPIO_SPEED_FREQ_LOW

// BUZZER_EN ->PF3 GPO buzzer to piezo & for cap sense ---------------------------|
#define BUZZER_EN_GPIO_PIN                    GPIO_PIN_3
#define BUZZER_EN_GPIO_PORT                   GPIOF
#define BUZZER_EN_GPIO_MODE                   GPIO_MODE_OUTPUT_PP
#define BUZZER_EN_GPIO_PULL                   GPIO_PULLUP
#define BUZZER_EN_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOF_CLK_ENABLE()
#define BUZZER_EN_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOF_CLK_DISABLE()
#define BUZZER_EN_GPIO_SPEED                  GPIO_SPEED_FREQ_LOW

// HEAT_EN ->PE0(TIM16) PWM heating control --------------------------------------|
#define HEAT_EN_GPIO_PIN                      GPIO_PIN_0
#define HEAT_EN_GPIO_PORT                     GPIOE
#define HEAT_EN_GPIO_MODE                     GPIO_MODE_OUTPUT_PP
#define HEAT_EN_GPIO_PULL                     GPIO_PULLUP
#define HEAT_EN_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define HEAT_EN_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOE_CLK_DISABLE()
#define HEAT_EN_GPIO_SPEED                    GPIO_SPEED_FREQ_LOW

// CHG_STAT1 ->PE7 GPI charge status ---------------------------------------------|
#define CHG_STAT1_GPIO_PIN                    GPIO_PIN_7
#define CHG_STAT1_GPIO_PORT                   GPIOE
#define CHG_STAT1_GPIO_MODE                   GPIO_MODE_INPUT
#define CHG_STAT1_GPIO_PULL                   GPIO_NOPULL
#define CHG_STAT1_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOE_CLK_ENABLE()
#define CHG_STAT1_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOE_CLK_DISABLE()
#define CHG_STAT1_GPIO_SPEED                  GPIO_SPEED_FREQ_LOW

// CHG_STAT2 ->PE9 GPI charge status ---------------------------------------------|
#define CHG_STAT2_GPIO_PIN                    GPIO_PIN_9
#define CHG_STAT2_GPIO_PORT                   GPIOE
#define CHG_STAT2_GPIO_MODE                   GPIO_MODE_INPUT
#define CHG_STAT2_GPIO_PULL                   GPIO_NOPULL
#define CHG_STAT2_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOE_CLK_ENABLE()
#define CHG_STAT2_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOE_CLK_DISABLE()
#define CHG_STAT2_GPIO_SPEED                    GPIO_SPEED_FREQ_LOW

// BAT_ALRT ->PE11 GPI battery alarm interrupt------------------------------------|
#define BAT_ALRT_GPIO_PIN                     GPIO_PIN_11
#define BAT_ALRT_GPIO_PORT                    GPIOE
#define BAT_ALRT_GPIO_MODE                    GPIO_MODE_INPUT
#define BAT_ALRT_GPIO_PULL                    GPIO_NOPULL
#define BAT_ALRT_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOE_CLK_ENABLE()
#define BAT_ALRT_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOE_CLK_DISABLE()
#define BAT_ALRT_GPIO_SPEED                   GPIO_SPEED_FREQ_LOW

// CAP_GND ->PB5  Capsense sense node --------------------------------------------|
#define CAP_GND_GPIO_PIN                      GPIO_PIN_5
#define CAP_GND_GPIO_PORT                     GPIOB
#define CAP_GND_GPIO_MODE                     GPIO_MODE_OUTPUT_PP
#define CAP_GND_GPIO_PULL                     GPIO_PULLUP
#define CAP_GND_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define CAP_GND_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOE_CLK_DISABLE()
#define CAP_GND_GPIO_SPEED                    GPIO_SPEED_FREQ_LOW

// SHIELD_GND ->PB12  Capsense shield node ---------------------------------------|
#define SHIELD_GND_GPIO_PIN                   GPIO_PIN_12
#define SHIELD_GND_GPIO_PORT                  GPIOB
#define SHIELD_GND_GPIO_MODE                  GPIO_MODE_OUTPUT_PP
#define SHIELD_GND_GPIO_PULL                  GPIO_PULLUP
#define SHIELD_GND_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOE_CLK_ENABLE()
#define SHIELD_GND_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOE_CLK_DISABLE()
#define SHIELD_GND_GPIO_SPEED                 GPIO_SPEED_FREQ_LOW


#define DEBUG_PIN                              GPIO_PIN_2
#define DEBUG_GPIO_PORT                        GPIOB
#define DEBUG_GPIO_MODE                        GPIO_MODE_OUTPUT_PP
#define DEBUG_GPIO_PULL                        GPIO_NOPULL
#define DEBUG_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOB_CLK_ENABLE()
#define DEBUG_GPIO_CLK_DISABLE()             __HAL_RCC_GPIOB_CLK_DISABLE()

#define LIS2DW_XL_SYNC_PIN                     GPIO_PIN_8
#define LIS2DW_XL_SYNC_GPIO_PORT               GPIOC
#define LIS2DW_XL_SYNC_GPIO_MODE               GPIO_MODE_AF_PP
#define LIS2DW_XL_SYNC_GPIO_PULL               GPIO_NOPULL
#define LIS2DW_XL_SYNC_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
#define LIS2DW_XL_SYNC_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOC_CLK_DISABLE()
#define LIS2DW_XL_SYNC_ALTERNATE               GPIO_AF14_LPTIM3
#define LIS2DW_XL_SYNC_SPEED                   GPIO_SPEED_FREQ_HIGH

/* TX DEBUG ->PC12 (UART5) */
#define TX_DEBUG_GPIO_PIN                      GPIO_PIN_12
#define TX_DEBUG_GPIO_PORT                     GPIOC
#define TX_DEBUG_GPIO_MODE                     GPIO_MODE_AF_PP
#define TX_DEBUG_GPIO_PULL                     GPIO_NOPULL
#define TX_DEBUG_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define TX_DEBUG_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()
#define TX_DEBUG_GPIO_ALTERNATE                GPIO_AF8_UART5
#define TX_DEBUG_GPIO_SPEED                    GPIO_SPEED_FREQ_HIGH

/* RX DEBUG ->PD2 (UART5) */
#define RX_DEBUG_GPIO_PIN                      GPIO_PIN_2
#define RX_DEBUG_GPIO_PORT                     GPIOD
#define RX_DEBUG_GPIO_MODE                     GPIO_MODE_AF_PP
#define RX_DEBUG_GPIO_PULL                     GPIO_NOPULL
#define RX_DEBUG_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define RX_DEBUG_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()
#define RX_DEBUG_GPIO_ALTERNATE                GPIO_AF8_UART5
#define RX_DEBUG_GPIO_SPEED                    GPIO_SPEED_FREQ_HIGH

/* BT TX ->PA0 (UART4) */
#define TX_BT_GPIO_PIN                         GPIO_PIN_0
#define TX_BT_GPIO_PORT                        GPIOA
#define TX_BT_GPIO_MODE                        GPIO_MODE_AF_PP
#define TX_BT_GPIO_PULL                        GPIO_NOPULL
#define TX_BT_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOA_CLK_ENABLE()
#define TX_BT_GPIO_CLK_DISABLE()             __HAL_RCC_GPIOA_CLK_DISABLE()
#define TX_BT_GPIO_ALTERNATE                   GPIO_AF8_UART4
#define TX_BT_GPIO_SPEED                       GPIO_SPEED_FREQ_HIGH

/* BT RX ->PC11 (UART4) */
#define RX_BT_GPIO_PIN                         GPIO_PIN_11
#define RX_BT_GPIO_PORT                        GPIOC
#define RX_BT_GPIO_MODE                        GPIO_MODE_AF_PP
#define RX_BT_GPIO_PULL                        GPIO_NOPULL
#define RX_BT_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOC_CLK_ENABLE()
#define RX_BT_GPIO_CLK_DISABLE()             __HAL_RCC_GPIOC_CLK_DISABLE()
#define RX_BT_GPIO_ALTERNATE                   GPIO_AF8_UART4
#define RX_BT_GPIO_SPEED                       GPIO_SPEED_FREQ_HIGH

/* BT AUTORUN ->PA11 */
#define BT_AUTORUN_GPIO_PIN                    GPIO_PIN_11
#define BT_AUTORUN_GPIO_PORT                   GPIOA
#define BT_AUTORUN_GPIO_MODE                   GPIO_MODE_OUTPUT_PP
#define BT_AUTORUN_GPIO_PULL                   GPIO_NOPULL
#define BT_AUTORUN_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define BT_AUTORUN_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()
#define BT_AUTORUN_GPIO_SPEED                  GPIO_SPEED_FREQ_HIGH

/* BT FOTA_EN ->PA9 */
#define BT_FOTA_EN_GPIO_PIN                    GPIO_PIN_9
#define BT_FOTA_EN_GPIO_PORT                   GPIOA
#define BT_FOTA_EN_GPIO_MODE                   GPIO_MODE_OUTPUT_PP
#define BT_FOTA_EN_GPIO_PULL                   GPIO_NOPULL
#define BT_FOTA_EN_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define BT_FOTA_EN_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()
#define BT_FOTA_EN_GPIO_SPEED                  GPIO_SPEED_FREQ_HIGH

/* BT RST ->PA10 */
#define BT_RST_GPIO_PIN                        GPIO_PIN_10
#define BT_RST_GPIO_PORT                       GPIOA
#define BT_RST_GPIO_MODE                       GPIO_MODE_OUTPUT_PP
#define BT_RST_GPIO_PULL                       GPIO_NOPULL
#define BT_RST_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOA_CLK_ENABLE()
#define BT_RST_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOA_CLK_DISABLE()
#define BT_RST_GPIO_SPEED                      GPIO_SPEED_FREQ_HIGH


//=========================== [ analog IO definitions ] ============================|

/* ANALOG_IN EMG -> PB0(+) IN15 */
#define ADC_EMG_POS_GPIO_PIN                  GPIO_PIN_0
#define ADC_EMG_POS_GPIO_PORT                 GPIOB
#define ADC_EMG_POS_GPIO_MODE                 GPIO_MODE_ANALOG
#define ADC_EMG_POS_GPIO_PULL                 GPIO_NOPULL
#define ADC_EMG_POS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define ADC_EMG_POS_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOB_CLK_DISABLE()
#define ADC_EMG_POS_GPIO_SPEED                GPIO_SPEED_FREQ_HIGH

/* ANALOG_IN EMG -> PB1(-) IN16 */
#define ADC_EMG_NEG_GPIO_PIN                  GPIO_PIN_1
#define ADC_EMG_NEG_GPIO_PORT                 GPIOB
#define ADC_EMG_NEG_GPIO_MODE                 GPIO_MODE_ANALOG
#define ADC_EMG_NEG_GPIO_PULL                 GPIO_NOPULL
#define ADC_EMG_NEG_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define ADC_EMG_NEG_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOB_CLK_DISABLE()
#define ADC_EMG_NEG_GPIO_SPEED                GPIO_SPEED_FREQ_HIGH


/* ANALOG_IN SCL -> PC4(+) IN13 skin conduct level */
#define ADC_SCL_POS_GPIO_PIN                  GPIO_PIN_7
#define ADC_SCL_POS_GPIO_PORT                 GPIOA
#define ADC_SCL_POS_GPIO_MODE                 GPIO_MODE_ANALOG
#define ADC_SCL_POS_GPIO_PULL                 GPIO_NOPULL
#define ADC_SCL_POS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADC_SCL_POS_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOC_CLK_DISABLE()
#define ADC_SCL_POS_GPIO_SPEED                GPIO_SPEED_FREQ_HIGH

/* ANALOG_IN SCL -> PA7(-) IN14 skin conduct level   */
#define ADC_SCL_NEG_GPIO_PIN                  GPIO_PIN_4
#define ADC_SCL_NEG_GPIO_PORT                 GPIOC
#define ADC_SCL_NEG_GPIO_MODE                 GPIO_MODE_ANALOG
#define ADC_SCL_NEG_GPIO_PULL                 GPIO_NOPULL
#define ADC_SCL_NEG_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADC_SCL_NEG_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE()
#define ADC_SCL_NEG_GPIO_SPEED                GPIO_SPEED_FREQ_HIGH


/* ANALOG_IN RESP -> PA6(+) IN11 Respiration */
#define ADC_RESP_POS_GPIO_PIN                  GPIO_PIN_5
#define ADC_RESP_POS_GPIO_PORT                 GPIOA
#define ADC_RESP_POS_GPIO_MODE                 GPIO_MODE_ANALOG
#define ADC_RESP_POS_GPIO_PULL                 GPIO_NOPULL
#define ADC_RESP_POS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADC_RESP_POS_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE()
#define ADC_RESP_POS_GPIO_SPEED                GPIO_SPEED_FREQ_HIGH

/* ANALOG_IN RESP -> PA5(-) IN12 Respiration */
#define ADC_RESP_NEG_GPIO_PIN                  GPIO_PIN_6
#define ADC_RESP_NEG_GPIO_PORT                 GPIOA
#define ADC_RESP_NEG_GPIO_MODE                 GPIO_MODE_ANALOG
#define ADC_RESP_NEG_GPIO_PULL                 GPIO_NOPULL
#define ADC_RESP_NEG_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADC_RESP_NEG_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE()
#define ADC_RESP_NEG_GPIO_SPEED                GPIO_SPEED_FREQ_HIGH


/* ANALOG_IN AUDIO -> PA2(+) IN7 audio */
#define ADC_AUDIO_POS_GPIO_PIN                  GPIO_PIN_1
#define ADC_AUDIO_POS_GPIO_PORT                 GPIOA
#define ADC_AUDIO_POS_GPIO_MODE                 GPIO_MODE_ANALOG
#define ADC_AUDIO_POS_GPIO_PULL                 GPIO_NOPULL
#define ADC_AUDIO_POS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADC_AUDIO_POS_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE()
#define ADC_AUDIO_POS_GPIO_SPEED                GPIO_SPEED_FREQ_HIGH

/* ANALOG_IN AUDIO -> PA1(-) IN8 audio */
#define ADC_AUDIO_NEG_GPIO_PIN                  GPIO_PIN_2
#define ADC_AUDIO_NEG_GPIO_PORT                 GPIOA
#define ADC_AUDIO_NEG_GPIO_MODE                 GPIO_MODE_ANALOG
#define ADC_AUDIO_NEG_GPIO_PULL                 GPIO_NOPULL
#define ADC_AUDIO_NEG_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADC_AUDIO_NEG_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE()
#define ADC_AUDIO_NEG_GPIO_SPEED                GPIO_SPEED_FREQ_HIGH


/* ANALOG_IN PACE -> PC0(+) IN1 external pace */
#define ADC_PACE_POS_GPIO_PIN                  GPIO_PIN_0
#define ADC_PACE_POS_GPIO_PORT                 GPIOC
#define ADC_PACE_POS_GPIO_MODE                 GPIO_MODE_ANALOG
#define ADC_PACE_POS_GPIO_PULL                 GPIO_NOPULL
#define ADC_PACE_POS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADC_PACE_POS_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOC_CLK_DISABLE()
#define ADC_PACE_POS_GPIO_SPEED                GPIO_SPEED_FREQ_HIGH

/* ANALOG_IN PACE -> PC1(-) IN2 external pace */
#define ADC_PACE_NEG_GPIO_PIN                  GPIO_PIN_1
#define ADC_PACE_NEG_GPIO_PORT                 GPIOC
#define ADC_PACE_NEG_GPIO_MODE                 GPIO_MODE_ANALOG
#define ADC_PACE_NEG_GPIO_PULL                 GPIO_NOPULL
#define ADC_PACE_NEG_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADC_PACE_NEG_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOC_CLK_DISABLE()
#define ADC_PACE_NEG_GPIO_SPEED                GPIO_SPEED_FREQ_HIGH


#ifdef __cplusplus
}
#endif


#endif /* _BOARD_H_ */
