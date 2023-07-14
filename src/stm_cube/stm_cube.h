#ifndef _STM_CUBE_H_
#define _STM_CUBE_H_

#include "sys_headers.h"

extern SAI_HandleTypeDef hsai_BlockB1;
extern DMA_HandleTypeDef hdma_sai1_b;

extern UART_HandleTypeDef huart1;

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_USART1_UART_Init(void);
void MX_SAI1_Init(void);

#endif
