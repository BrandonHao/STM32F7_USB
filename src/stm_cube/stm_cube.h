#ifndef _STM_CUBE_H_
#define _STM_CUBE_H_

#include "sys_headers.h"

extern CRC_HandleTypeDef hcrc;

extern DMA2D_HandleTypeDef hdma2d;

extern LTDC_HandleTypeDef hltdc;

extern QSPI_HandleTypeDef hqspi;

extern RTC_HandleTypeDef hrtc;

extern UART_HandleTypeDef huart1;

extern DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
extern SDRAM_HandleTypeDef hsdram1;

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_GPIO_Init(void);
void MX_CRC_Init(void);
void MX_DMA2D_Init(void);
void MX_FMC_Init(void);
void MX_LTDC_Init(void);
void MX_QUADSPI_Init(void);
void MX_RTC_Init(void);
void MX_DMA_Init(void);
void MX_USART1_UART_Init(void);

#endif
