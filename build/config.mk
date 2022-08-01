#################################
# GNU ARM Embedded Toolchain
#################################
PROJ_NAME=STM32F7_USB

PROJ_PATH=../

TARGET_FLASH=$(PROJ_NAME).elf

ARCH=cortex-m7
PART=STM32F750xx

LINKER_SCRIPT_FLASH=STM32F750N8HX_FLASH.ld
    
INC_DIR= \
	../Drivers/CMSIS/Device/ST/STM32F7xx/Include	\
	../Drivers/CMSIS/Include						\
	../Drivers/STM32F7xx_HAL_Driver/Inc				\
	../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy		\
	../Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Inc\
	../Middlewares/ST/STM32_USB_Device_Library/Core/Inc\
	../USB_DEVICE/App								\
	../USB_DEVICE/Target							\
	../Core/Inc

TINYUSB_INC= \
	../lib/tinyusb/src							\
	../lib/tinyusb/src/class/audio				\
	../lib/tinyusb/src/common					\
	../lib/tinyusb/src/device					\
	../lib/tinyusb/src/portable/synopsys/dwc2

TUSB_PORT_INC= \
	../tusb_port

APP_INC= \
	../App/Inc

CORE_SRC= \
	Core/Src/stm32f7xx_hal_msp.c				\
	Core/Src/stm32f7xx_it.c						\
	Core/Src/syscalls.c							\
	Core/Src/sysmem.c							\
	Core/Src/system_stm32f7xx.c					\
	Core/Startup/startup_stm32f750n8hx.s

STM_SRC= \
	Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Src/usbd_audio.c\
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c\
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c\
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c\
	USB_DEVICE/Target/usbd_conf.c				\
	USB_DEVICE/App/usb_device.c					\
	USB_DEVICE/App/usbd_audio_if.c				\
	USB_DEVICE/App/usbd_desc.c					\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_crc.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_crc_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dcmi.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dcmi_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma2d.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dsi.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_ltdc.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_ltdc_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_qspi.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rtc.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rtc_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sai.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sai_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sd.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sdram.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spdifrx.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_dma.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_exti.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_fmc.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_gpio.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_rcc.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_sdmmc.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_tim.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c\
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_utils.c

TINYUSB_SRC= \
	lib/tinyusb/src/tusb.c						\
	lib/tinyusb/src/class/audio/audio_device.c	\
	lib/tinyusb/src/common/tusb_fifo.c			\
	lib/tinyusb/src/device/usbd_control.c		\
	lib/tinyusb/src/device/usbd.c				\
	lib/tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c

TUSB_PORT_SRC= \
	tusb_port/family.c

APP_SRC= \
	App/Src/main.c								\
	App/Src/stm_cube.c							\
	App/Src/usb_descriptors.c

OPTIMIZATION= \
    -Og

TUSB_FLAGS= \
	-DCFG_TUSB_MCU=OPT_MCU_STM32F7 \
  	-DBOARD_TUD_RHPORT=1		\
  	-DBOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED\
	-DCFG_TUSB_DEBUG=3			\
	

CFLAGS= \
	-mfloat-abi=hard			\
	-mfpu=fpv5-sp-d16           \
	-mthumb						\
	--specs=nano.specs			\
	-fstack-usage				\
	-DCFG_TUSB_DEBUG=3

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
	-DDEBUG						\
	-DUSE_HAL_DRIVER 			\
	-DUSE_USB_HS				\
	-DUSE_FULL_LL_DRIVER		\
	-DSTM32F750xx				\
	-DLOG=3						\
	$(TUSB_FLAGS)

