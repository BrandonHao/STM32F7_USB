/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "sys_headers.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OTG_HS_OverCurrent_Pin LL_GPIO_PIN_3
#define OTG_HS_OverCurrent_GPIO_Port GPIOE
#define SWO_Pin LL_GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define VCP_RX_Pin LL_GPIO_PIN_7
#define VCP_RX_GPIO_Port GPIOB
#define OTG_FS_VBUS_Pin LL_GPIO_PIN_12
#define OTG_FS_VBUS_GPIO_Port GPIOJ
#define Audio_INT_Pin LL_GPIO_PIN_6
#define Audio_INT_GPIO_Port GPIOD
#define NC1_Pin LL_GPIO_PIN_8
#define NC1_GPIO_Port GPIOI
#define OTG_FS_PowerSwitchOn_Pin LL_GPIO_PIN_5
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOD
#define ARDUINO_D7_Pin LL_GPIO_PIN_3
#define ARDUINO_D7_GPIO_Port GPIOI
#define ARDUINO_D8_Pin LL_GPIO_PIN_2
#define ARDUINO_D8_GPIO_Port GPIOI
#define uSD_Detect_Pin LL_GPIO_PIN_13
#define uSD_Detect_GPIO_Port GPIOC
#define LCD_BL_CTRL_Pin LL_GPIO_PIN_3
#define LCD_BL_CTRL_GPIO_Port GPIOK
#define OTG_FS_OverCurrent_Pin LL_GPIO_PIN_4
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define TP3_Pin LL_GPIO_PIN_15
#define TP3_GPIO_Port GPIOH
#define RCC_OSC32_IN_Pin LL_GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define LCD_DISP_Pin LL_GPIO_PIN_12
#define LCD_DISP_GPIO_Port GPIOI
#define DCMI_PWR_EN_Pin LL_GPIO_PIN_13
#define DCMI_PWR_EN_GPIO_Port GPIOH
#define VCP_TX_Pin LL_GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define RCC_OSC32_OUT_Pin LL_GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define OSC_25M_Pin LL_GPIO_PIN_0
#define OSC_25M_GPIO_Port GPIOH
#define LCD_INT_Pin LL_GPIO_PIN_13
#define LCD_INT_GPIO_Port GPIOI
#define ARDUINO_D4_Pin LL_GPIO_PIN_7
#define ARDUINO_D4_GPIO_Port GPIOG
#define ARDUINO_D2_Pin LL_GPIO_PIN_6
#define ARDUINO_D2_GPIO_Port GPIOG
#define NC2_Pin LL_GPIO_PIN_2
#define NC2_GPIO_Port GPIOH
#define EXT_RST_Pin LL_GPIO_PIN_3
#define EXT_RST_GPIO_Port GPIOG
#define RMII_RXER_Pin LL_GPIO_PIN_2
#define RMII_RXER_GPIO_Port GPIOG
#define ULPI_D3_Pin LL_GPIO_PIN_10
#define ULPI_D3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
