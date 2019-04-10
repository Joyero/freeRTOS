/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"

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
#define GPS_PPS_Pin LL_GPIO_PIN_13
#define GPS_PPS_GPIO_Port GPIOC
#define BLE_PWR_ON_Pin LL_GPIO_PIN_0
#define BLE_PWR_ON_GPIO_Port GPIOC
#define GSM_PWRKEY_Pin LL_GPIO_PIN_1
#define GSM_PWRKEY_GPIO_Port GPIOC
#define ADC_MPVOLT_Pin LL_GPIO_PIN_0
#define ADC_MPVOLT_GPIO_Port GPIOA
#define ADC_BAT_NTC_Pin LL_GPIO_PIN_1
#define ADC_BAT_NTC_GPIO_Port GPIOA
#define ADC_BCK_BAT_Pin LL_GPIO_PIN_2
#define ADC_BCK_BAT_GPIO_Port GPIOA
#define AUDIO_MCLK_Pin LL_GPIO_PIN_3
#define AUDIO_MCLK_GPIO_Port GPIOA
#define GSM_PWR_ON_Pin LL_GPIO_PIN_4
#define GSM_PWR_ON_GPIO_Port GPIOA
#define MEM_SPICLK_Pin LL_GPIO_PIN_5
#define MEM_SPICLK_GPIO_Port GPIOA
#define MEM_SPIMISO_Pin LL_GPIO_PIN_6
#define MEM_SPIMISO_GPIO_Port GPIOA
#define MEM_SPIMOSI_Pin LL_GPIO_PIN_7
#define MEM_SPIMOSI_GPIO_Port GPIOA
#define GPS_PWR_ON_Pin LL_GPIO_PIN_4
#define GPS_PWR_ON_GPIO_Port GPIOC
#define ADC_EXTIN1_Pin LL_GPIO_PIN_5
#define ADC_EXTIN1_GPIO_Port GPIOC
#define ADC_EXTIN2_Pin LL_GPIO_PIN_0
#define ADC_EXTIN2_GPIO_Port GPIOB
#define GPI_EXT2_Pin LL_GPIO_PIN_1
#define GPI_EXT2_GPIO_Port GPIOB
#define GPI_EXT1_Pin LL_GPIO_PIN_2
#define GPI_EXT1_GPIO_Port GPIOB
#define RAD_SPICLK_Pin LL_GPIO_PIN_10
#define RAD_SPICLK_GPIO_Port GPIOB
#define MEM_SPICS_Pin LL_GPIO_PIN_11
#define MEM_SPICS_GPIO_Port GPIOB
#define BAT_CHG_EN_Pin LL_GPIO_PIN_12
#define BAT_CHG_EN_GPIO_Port GPIOB
#define ONEW_DATA_Pin LL_GPIO_PIN_13
#define ONEW_DATA_GPIO_Port GPIOB
#define RAD_SPIMISO_Pin LL_GPIO_PIN_14
#define RAD_SPIMISO_GPIO_Port GPIOB
#define RAD_SPIMOSI_Pin LL_GPIO_PIN_15
#define RAD_SPIMOSI_GPIO_Port GPIOB
#define EXTUSR2_TX_Pin LL_GPIO_PIN_6
#define EXTUSR2_TX_GPIO_Port GPIOC
#define EXTUSR2_RX_Pin LL_GPIO_PIN_7
#define EXTUSR2_RX_GPIO_Port GPIOC
#define GPO_EXT5_Pin LL_GPIO_PIN_8
#define GPO_EXT5_GPIO_Port GPIOC
#define GPO_EXT4_Pin LL_GPIO_PIN_9
#define GPO_EXT4_GPIO_Port GPIOC
#define GPO_EXT3_Pin LL_GPIO_PIN_8
#define GPO_EXT3_GPIO_Port GPIOA
#define GPS_TX_Pin LL_GPIO_PIN_9
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin LL_GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOA
#define GPO_EXT2_Pin LL_GPIO_PIN_11
#define GPO_EXT2_GPIO_Port GPIOA
#define GPO_EXT1_Pin LL_GPIO_PIN_12
#define GPO_EXT1_GPIO_Port GPIOA
#define GPI_EXT3_Pin LL_GPIO_PIN_15
#define GPI_EXT3_GPIO_Port GPIOA
#define EXTUSR1_TX_Pin LL_GPIO_PIN_10
#define EXTUSR1_TX_GPIO_Port GPIOC
#define EXTUSR1_RX_Pin LL_GPIO_PIN_11
#define EXTUSR1_RX_GPIO_Port GPIOC
#define BLE_TX_Pin LL_GPIO_PIN_12
#define BLE_TX_GPIO_Port GPIOC
#define BLE_RX_Pin LL_GPIO_PIN_2
#define BLE_RX_GPIO_Port GPIOD
#define BLE_CONF_Pin LL_GPIO_PIN_3
#define BLE_CONF_GPIO_Port GPIOB
#define ACCEL_INT1_Pin LL_GPIO_PIN_4
#define ACCEL_INT1_GPIO_Port GPIOB
#define ACCEL_INT2_Pin LL_GPIO_PIN_5
#define ACCEL_INT2_GPIO_Port GPIOB
#define RAD_RESET_Pin LL_GPIO_PIN_6
#define RAD_RESET_GPIO_Port GPIOB
#define RAD_CS_Pin LL_GPIO_PIN_7
#define RAD_CS_GPIO_Port GPIOB
#define ACEL_SCL_Pin LL_GPIO_PIN_8
#define ACEL_SCL_GPIO_Port GPIOB
#define ACEL_SDA_Pin LL_GPIO_PIN_9
#define ACEL_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
