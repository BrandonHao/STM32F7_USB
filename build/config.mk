#################################
# GNU ARM Embedded Toolchain
#################################
PROJ_NAME=STM32F7_USB

PROJ_PATH=../
LIB_PATH=../lib/
TARGET_FLASH=$(PROJ_NAME).elf

ARCH=cortex-m7
PART=STM32F750xx

LINKER_SCRIPT_FLASH=STM32F750N8HX_FLASH.ld
    
INC_DIR= \
	../lib/Drivers/CMSIS/Device/ST/STM32F7xx/Include	\
	../lib/Drivers/CMSIS/Include						\
	../lib/Drivers/STM32F7xx_HAL_Driver/Inc				\
	../lib/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy		\
	../lib/Core/Inc

TINYUSB_INC= \
	../lib/tinyusb/src							\
	../lib/tinyusb/src/class/audio				\
	../lib/tinyusb/src/common					\
	../lib/tinyusb/src/device					\
	../lib/tinyusb/src/portable/synopsys/dwc2

APP_INC= \
	../src/app									\
	../src/error_handler						\
	../src/tusb_port							\
	../src/sai_controller						\
	../src/stm_cube

CORE_SRC= \
	lib/Core/Src/stm32f7xx_hal_msp.c				\
	lib/Core/Src/stm32f7xx_it.c						\
	lib/Core/Src/syscalls.c							\
	lib/Core/Src/sysmem.c							\
	lib/Core/Src/system_stm32f7xx.c					\
	lib/Core/Startup/startup_stm32f750n8hx.s

STM_SRC= \
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_crc.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_crc_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dcmi.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dcmi_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma2d.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dsi.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_ltdc.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_ltdc_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_qspi.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rtc.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rtc_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sai.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sai_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sd.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sdram.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spdifrx.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_dma.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_exti.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_fmc.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_gpio.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_rcc.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_sdmmc.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_tim.c\
	lib/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_utils.c

TINYUSB_SRC= \
	lib/tinyusb/src/tusb.c						\
	lib/tinyusb/src/class/audio/audio_device.c	\
	lib/tinyusb/src/common/tusb_fifo.c			\
	lib/tinyusb/src/device/usbd_control.c		\
	lib/tinyusb/src/device/usbd.c				\
	lib/tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c

TUSB_PORT_SRC= \
	src/tusb_port/usb_descriptors.c				\
	src/tusb_port/family.c

APP_SRC= \
	src/app/main.c								\
	src/error_handler/error_handler.c			\
	src/sai_controller/sai_controller.c			\
	src/stm_cube/stm_cube.c


OPTIMIZATION= \
    -Og

TUSB_FLAGS= \
	-DCFG_TUSB_MCU=OPT_MCU_STM32F7 \
  	-DBOARD_TUD_RHPORT=1		\
  	-DBOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED\
	-DCFG_TUSB_DEBUG=3			

CFLAGS= \
	-mfloat-abi=hard			\
	-mfpu=fpv5-sp-d16           \
	-mthumb						\
	--specs=nano.specs			\
	-fstack-usage

CXXFLAGS= \
	-mfloat-abi=hard			\
	-mfpu=fpv5-sp-d16           \
	-mthumb						\
	--specs=nano.specs			\
	-fstack-usage				\
	-fno-exceptions				\
	-fno-rtti					\
	-fno-use-cxa-atexit

CPPFLAGS= \
	-DUSE_FULL_ASSERT			\
	-DDEBUG						\
	-DUSE_HAL_DRIVER 			\
	-DUSE_USB_HS				\
	-DUSE_FULL_LL_DRIVER		\
	-DLOG=3						\
	$(TUSB_FLAGS)

