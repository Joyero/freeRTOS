/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : gps.h
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
#ifndef __GPS_H
#define __GPS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/**
@name  	    sGPSConfig
@brief	    Contain configuration of class GPS mainly NMEA type and fields of interest
*/
 typedef struct
 {
  uint8_t  aFrameName[4];
  uint8_t  ui8WeightNMEA; 
  uint32_t ui32FrameFields; 
 }sGPSConfig;
 
 //Estructura de Tiempo
typedef struct
{
    uint8_t  u8Hour;
    uint8_t  u8Minutes;
    uint8_t  u8Seconds;
//    uint32_t u32mSeconds;
} sGPSTime;

//Estructura de Fecha
typedef struct
{
    uint8_t  u8Day;
    uint8_t  u8Month;
    uint16_t u16Year;
} sGPSDate;

//Estructura de Fecha/Tiempo
typedef struct
{
    sGPSDate Date;
    //sGPSTime Time;
		uint32_t lDaySecs;
		uint32_t lEpoch;
} sGPSDateTime;

typedef struct
{
    uint8_t   cOrientation;
    uint8_t  u8Degrees;
    uint8_t  u8Minutes;
    int16_t u16MilMin;
} sGPSCoordinate;

typedef struct
{ 
    volatile uint8_t          bValidFrame;
    volatile int8_t           cStatus;
    volatile uint8_t          ui8NSats;
    volatile uint8_t          ui8FixQuality;
    volatile uint8_t          u8OrientationByte;
    volatile uint32_t         ui32Epoch;
    //volatile uint32_t         ui32MiliSecondsTime;
    volatile uint8_t					u8Hdop; /* HDOP Value x 10 */
    volatile uint32_t        	ulHeight; /* Height in decimeters */
		volatile uint32_t        	ulSpeedKmh; /* Km/h *100 */
    //volatile float        		fMagCourse;
    volatile uint32_t        	ulSpeedKnot; /* Knots x 100 */
    volatile uint32_t        	ulTrueCourse; /* Course x 100 */
    volatile sGPSDateTime   	sDateTime;        
    //volatile sGPSCoordinate 	sLatitude;
    //volatile sGPSCoordinate 	sLongitude;
		volatile int32_t lLatitude;
	  volatile int32_t lLongitude;
}sGPSData;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void gpsReceiveData(uint8_t);
void StartTaskGPS(void const * argument);
void gps_start(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
//#define GPS_ON_OFF_Pin GPIO_PIN_4
//#define GPS_ON_OFF_GPIO_Port GPIOC
//#define GPS_TX_Pin GPIO_PIN_9
//#define GPS_TX_GPIO_Port GPIOA
//#define GPS_RX_Pin GPIO_PIN_10
//#define GPS_RX_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
//#define GPS_TX_Pin GPIO_PIN_9
//#define GPS_TX_GPIO_Port GPIOA
//#define GPS_RX_Pin GPIO_PIN_10
//#define GPS_RX_GPIO_Port GPIOA


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
