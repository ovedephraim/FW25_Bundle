################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_comp.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cordic.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcache.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcmi.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma2d.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dsi.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fdcan.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fmac.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gfxmmu.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpu2d.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hcd.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_irda.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_iwdg.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_lptim.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mdf.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nand.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nor.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ospi.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_otfdec.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pka.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pssi.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ramcfg.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sram.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tsc.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart_ex.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_wwdg.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_xspi.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_exti.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_gpio.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_icache.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_lpgpio.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_pwr.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_rtc.c \
../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_utils.c 

OBJS += \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_comp.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cordic.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcache.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcmi.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma2d.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dsi.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fdcan.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fmac.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gfxmmu.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpu2d.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hcd.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_irda.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_iwdg.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_lptim.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mdf.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nand.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nor.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ospi.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_otfdec.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pka.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pssi.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ramcfg.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sram.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tsc.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart_ex.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_wwdg.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_xspi.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_exti.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_gpio.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_icache.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_lpgpio.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_pwr.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_rtc.o \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_utils.o 

C_DEPS += \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_comp.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cordic.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcache.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcmi.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma2d.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dsi.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fdcan.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fmac.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gfxmmu.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpu2d.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hcd.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_irda.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_iwdg.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_lptim.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mdf.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nand.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nor.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ospi.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_otfdec.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pka.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pssi.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ramcfg.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sram.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tsc.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart_ex.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_wwdg.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_xspi.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_exti.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_gpio.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_icache.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_lpgpio.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_pwr.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_rtc.d \
./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32U5xx_HAL_Driver/Src/%.o Drivers/STM32U5xx_HAL_Driver/Src/%.su Drivers/STM32U5xx_HAL_Driver/Src/%.cyclo: ../Drivers/STM32U5xx_HAL_Driver/Src/%.c Drivers/STM32U5xx_HAL_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32U585xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32U5xx_HAL_Driver-2f-Src

clean-Drivers-2f-STM32U5xx_HAL_Driver-2f-Src:
	-$(RM) ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_comp.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_comp.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_comp.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_comp.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cordic.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cordic.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cordic.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cordic.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcache.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcache.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcache.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcache.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcmi.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcmi.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcmi.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcmi.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma2d.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma2d.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma2d.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma2d.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dsi.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dsi.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dsi.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dsi.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fdcan.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fdcan.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fdcan.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fdcan.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fmac.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fmac.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fmac.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fmac.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gfxmmu.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gfxmmu.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gfxmmu.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gfxmmu.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpu2d.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpu2d.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpu2d.o
	-$(RM) ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpu2d.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hcd.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hcd.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hcd.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hcd.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_irda.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_irda.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_irda.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_irda.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_iwdg.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_iwdg.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_iwdg.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_iwdg.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_lptim.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_lptim.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_lptim.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_lptim.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mdf.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mdf.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mdf.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mdf.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nand.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nand.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nand.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nand.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nor.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nor.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nor.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nor.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ospi.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ospi.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ospi.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ospi.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_otfdec.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_otfdec.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_otfdec.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_otfdec.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pka.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pka.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pka.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pka.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pssi.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pssi.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pssi.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pssi.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.d
	-$(RM) ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ramcfg.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ramcfg.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ramcfg.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ramcfg.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sram.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sram.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sram.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sram.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tsc.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tsc.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tsc.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tsc.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart.su
	-$(RM) ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart_ex.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart_ex.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart_ex.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart_ex.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_wwdg.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_wwdg.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_wwdg.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_wwdg.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_xspi.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_xspi.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_xspi.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_xspi.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_exti.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_exti.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_exti.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_exti.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_gpio.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_gpio.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_gpio.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_gpio.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_icache.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_icache.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_icache.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_icache.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_lpgpio.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_lpgpio.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_lpgpio.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_lpgpio.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_pwr.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_pwr.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_pwr.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_pwr.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_rtc.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_rtc.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_rtc.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_rtc.su ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_utils.cyclo ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_utils.d ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_utils.o ./Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_utils.su

.PHONY: clean-Drivers-2f-STM32U5xx_HAL_Driver-2f-Src

