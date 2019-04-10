/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : gps.c
  * @brief          : GPS related function
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

/* Includes ------------------------------------------------------------------*/
#include "gps.h"
#include "main.h"
#include "cmsis_os.h"
#include "GPS_Definitions.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
# define GPS_RX_QUEUE_SIZE 80

#define GPS_MAX_FIELD_LEN		13
#define GPS_FRAME_TOKEN			','
#define GPS_FRAME_END			'*'


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

extern UART_HandleTypeDef huart1;

osThreadId GPSTaskHandle;
QueueHandle_t gpsRxQueueHandle;
uint8_t gpsRxQueueBuffer[ GPS_RX_QUEUE_SIZE * sizeof( uint8_t ) ];
SemaphoreHandle_t gpsSemaphoreFullRx = NULL;
/* USER CODE BEGIN PV */
const uint8_t PMTK_SET_NMEA_OUTPUT_RMCGGA[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";

const uint32_t ui32MaskValidation = VALIDATION_NSATS  | VALIDATION_HDOP | VALIDATION_STATUS | VALIDATION_LATITUDE |	VALIDATION_LONGITUDE ;

///Declarate Array Structure configuration of RMC, GGA and VTG.
 sGPSConfig sConfigNMEA[] = {
        {"RMC", 0u, RMC_TIME + RMC_STATUS + RMC_LATITUDE + RMC_LAT_SIGN + RMC_LONGITUDE + RMC_LON_SIGN + RMC_SPEEDKNOT + RMC_COURSE + RMC_DATE},
        {"GGA", 1u, GGA_FIX_STATUS + GGA_NUMSATS + GGA_HDOP + GGA_HEIGHT},
        //{"VTG", 2u, VTG_SPEED_KMH},
};

volatile sGPSData GPSTempData;        //Variable de datos válidos
volatile sGPSData GPSData;            //Variable de datos temporales mientras se validan

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//static void MX_USART1_UART_Init(void);
void StartTaskGPS(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */


/**
  * @brief USART1 Interrupt Settings.
  * @retval None
  */
static void MX_USART1_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
	/* Enable Receive Interrupt */
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
}



/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_USART1_UART_Init(void)
//{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 9600;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

//}




/* USER CODE BEGIN 4 */





/*******************************************************************************
* Name: This_HexaToASCII
* Developer(s): Cesar Sarmiento
* References:
* -----------------------------------------------------------------------------
* Notes: Esta función retorna el caracter equivalente a un dígito ASCII
*******************************************************************************/
uint8_t This_HexaToASCII(uint8_t ucHexa)
{
  /* Variable para entregar el resultado, si no es ninguna de las opciones 
     válidas, se retorna 0xFF */
  uint8_t ucResult = 0xFF;
  /* Se filtra que ingresen datos válidos a la función */
  if( ucHexa < 0x10 )
  {
    /* Si es un dígito, la operación es sumarle 0x30, por ejemplo si entra un
       0x05, sale un 0x35, que en la tabla ASCII corresponde a un '5' */
    if( ucHexa < 0x0A )
    {
      ucResult = ucHexa + 0x30;
    }
    else
    {
      /* Si es una letra, el proceso es sumarle 0x37, por ejemplo, si entra un
         0x0A, sale un 0x41, que en la table ASCII corresponde a un 'A' */
      ucResult = ucHexa + 0x37;
    }
  }
  return(ucResult);
}
/*******************************************************************************/
void gpsSendFrame(uint8_t *pFrame, uint8_t ucSize)
{
	HAL_UART_Transmit_IT(&huart1,pFrame,ucSize);
}

/*******************************************************************************
* Name: gpsShutDownFrame
* Developer(s): Cesar Sarmiento
* References:
* -----------------------------------------------------------------------------
* Notes: Esta función apaga las tramas de GPS no deseadas.
*******************************************************************************/
void gpsShutDownFrame(uint8_t *pFrame)
{
	switch(*(pFrame+2))
	{
		case 'V':
		{
			if( (*(pFrame+3) == 'T') && (*(pFrame+4) == 'G') )
			{
				gpsSendFrame((uint8_t *)&PMTK_SET_NMEA_OUTPUT_RMCGGA,sizeof(PMTK_SET_NMEA_OUTPUT_RMCGGA));
			}
		}break;
	}
}
/********************************************************************************/
/*******************************************************************************
* Name: gpsFrameOfInterest
* Developer(s): Cesar Sarmiento
* References:
* -----------------------------------------------------------------------------
* Notes: Esta función retorna el índice de la trama dependiendo si la trama ingresada 
*        como argumento de la función es deseada correcto(0,1,2) o falso(0xFF).
*******************************************************************************/
uint8_t gpsFrameOfInterest(uint8_t *pFrame)
{
	uint8_t ucReturn = 0xFF;
	switch(*(pFrame+2))
	{
		case 'R':
		{
			if( (*(pFrame+3) == 'M') && (*(pFrame+4) == 'C') )
			{
				ucReturn = 0x00;
			}
		}break;
		case 'G':
		{
			if( (*(pFrame+3) == 'G') && (*(pFrame+4) == 'A') )
			{
				ucReturn = 0x01;
			}
		}break;
//		case 'V':
//		{
//			if( (*(pFrame+3) == 'T') && (*(pFrame+4) == 'G') )
//			{
//				ucReturn = 0x02;
//			}
//		}break;
	}
	return(ucReturn);
}
/********************************************************************************/


/*******************************************************************************
* Name: This_NmeaChecksumVerify
* Developer(s): Cesar Sarmiento
* References:
* -----------------------------------------------------------------------------
* Notes: Esta función retorna un 0 o un 1 dependiendo si la trama ingresada 
*        como argumento de la función tiene un checksum correcto(1) o falso(0).
*******************************************************************************/
uint8_t This_NmeaChecksumVerify(unsigned char * ucpNmeaFrame)
{
  /* El estándar de tipos de variables no es soportado por el proyecto, para
     conservar al mínimo las modificaciones se continuará con las definiciones 
     de variable susadas por el proyecto */
  /* variable utilizada para recorrer la trama, empieza en 1 porque para el 
     cálculo del checksum se hace a partir de la 'G' excluyendo el '$' */
  static uint8_t ucIndex;
  /* variable para calcular el resultado parcial del checksum, se forza a 
     estática para que el compilador la situe en la RAM disponible y no en 
     stack lo que le da mas agilidad al disassembly (menos ciclos de reloj) */
  static uint8_t ucCalculatedChecksum;
  /* variable para guardar el resultado de la verificación del checksum */
  static uint8_t ucVerificationResult;
  /* variable para almacenar temporalmente el dato que está siendo analizado */
  static uint8_t ucCurrentValue;
  /* variable para almacenar resultados parciales */
  static uint8_t ucPartialResult;
	/* las dos condiciones para romper el while son, que se encuentre con el 
     final de los datos en la trama y el comienzo del checksum, que esta 
     marcado por un caracter '*' o que llegue al límite de longitud de la trama
     NMEA sin encontrar el *, en cuyo caso la trama recibida no es correcta y 
     se debe abortar el cálculo del checksum */
  ucIndex = 0;
  ucCalculatedChecksum = 0;
  ucVerificationResult = 0;
  ucPartialResult = 0;
  do
  {
    /* se actualiza la variable local, con el caracter de la trama */
    ucCurrentValue = *(ucpNmeaFrame + ucIndex);
    /* se actualiza el apuntador para recorrer la trama */
    ucIndex++;
    if( ucCurrentValue == GPS_FRAME_END )
    {
      /* Se marca el hallazgo del '*' */
      ucVerificationResult = 1;
    }
    else
    {
      /* cálculo parcial del checksum */
      ucCalculatedChecksum ^= ucCurrentValue;
    }
  }
  while( ucCurrentValue != GPS_FRAME_END &&  ucIndex < GPS_RX_QUEUE_SIZE );
  /* Se condiciona la comparación del checksum calculado con el que viene en la
     trama, a que se haya detectado el caracter '*' */
  if( ucVerificationResult == 1 )
  {
    /* Se reinicia la variable porque la verificación no ha concluido */
    ucVerificationResult = 0;
    /* El checksum que se encuentra al final de la trama NMEA, justo después
       del caracter '*' viene como una representación ASCII de un hexadecimal
       puesto en dos bytes, por ejemplo, si el checksum es un 0x2A, a 
       continuación del '*' encontraremos un caracter ASCII '2' (0x32) y otro
       caracter ASCII 'A' (0x41) */
     /* Se asigna a uCurrentValue la parte alta del checksum calculado, para 
        compararlo con el siguiente byte despues del '*' que en este momento
        esta apuntado por ucIndex */
     ucCurrentValue = ( ucCalculatedChecksum >> 4 );
     /* Se transforma la parte alta del checksum en el correspondiente caracter
        ASCII */
     ucPartialResult = This_HexaToASCII(ucCurrentValue);
     /* Se recupera el primer dígito del checksum de la trama */
     ucCurrentValue = *(ucpNmeaFrame + ucIndex);
     if( ucCurrentValue == ucPartialResult )
     {
       /* Si pasa por esta sección de código, el primer dígito del checksum 
          corresponde, se procede a comparar el segundo */
       /* en ucCurrentValue queda la parte baja del checksum */
       ucCurrentValue = ( ucCalculatedChecksum & 0x0F );
       /* Se transforma la parte baja del checksum en el correspondiente 
          caracter ASCII */
       ucPartialResult = This_HexaToASCII(ucCurrentValue);
       /* Se actualiza el índice ucIndex para apuntar al siguiente byte del
          checksum */
       ucIndex++;
       /* Se recupera del frame, el segundo byte del checksum */
       ucCurrentValue = *(ucpNmeaFrame + ucIndex);
       if( ucCurrentValue == ucPartialResult )
       {
         /* el checksum de la trama corresponde */
         ucVerificationResult = 1;
       }
     }
  }
  return(ucVerificationResult);
}

uint32_t gpsGetTime(uint8_t *pData)
{
	uint8_t ucHour = 0;
	uint8_t ucMinutes = 0;
	uint8_t ucSeconds = 0;
	uint32_t ulSecondsOfDay = 0;
	
	/* Hora */
	ucHour = (*pData - 0x30) * 10;
	ucHour += (*(pData+1) - 0x30);
	/* Minutos */
	ucMinutes = (*(pData + 2) - 0x30) * 10;
	ucMinutes += (*(pData + 3) - 0x30);
	/* Segundos */
	ucSeconds = (*(pData + 4) - 0x30) * 10;
	ucSeconds += (*(pData + 5) - 0x30);
	
	ulSecondsOfDay = (ucHour * 3600) + (ucMinutes * 60) + ucSeconds;
	
	return(ulSecondsOfDay);
	
}


uint8_t * gpsGetNextToken(uint8_t * pData)
{
	uint8_t ucFieldLenCtrl = 0;
	
	for(;( (ucFieldLenCtrl < GPS_MAX_FIELD_LEN) && ( *(pData + ucFieldLenCtrl) != GPS_FRAME_TOKEN)&& ( *(pData + ucFieldLenCtrl) != GPS_FRAME_END) ) ; ucFieldLenCtrl++ );
	ucFieldLenCtrl++;
	return(pData + ucFieldLenCtrl);
}

int32_t gpsGetCoordinate(uint8_t *pData)
{
	uint8_t ucDataLen = 0;
	uint8_t *pNextData;
	int32_t lReturn = 0;
	uint8_t u8Grados = 0;
	uint8_t u8Minutos = 0;
	int32_t i32MiliMinutos = 0;
	
	/* Obtain Field Len */
	pNextData = gpsGetNextToken(pData);
	ucDataLen = (uint8_t)(pNextData - pData);
	
	switch(ucDataLen)
	{
		case 11:
		{
			u8Grados = (*pData - 0x30) * 100;
			u8Grados += (*(pData + 1) - 0x30) * 10;
			u8Grados += (*(pData + 2) - 0x30);

			u8Minutos = (*(pData+3) - 0x30) * 10;
			u8Minutos += (*(pData+4) - 0x30);
			
			i32MiliMinutos = (*(pData+6) - 0x30) * 1000;
			i32MiliMinutos += (*(pData+7) - 0x30) * 100;
			i32MiliMinutos += (*(pData+8) - 0x30) * 10;
			i32MiliMinutos += (*(pData+9) - 0x30);

		}break;
		case 10:
		{
			u8Grados = 0;
			u8Grados += (*pData - 0x30) * 10;
			u8Grados += (*(pData+1) - 0x30);
			
			u8Minutos = (*(pData+2) - 0x30) * 10;
			u8Minutos += (*(pData+3) - 0x30);
			
			i32MiliMinutos = (*(pData+5) - 0x30) * 1000;
			i32MiliMinutos += (*(pData+6) - 0x30) * 100;
			i32MiliMinutos += (*(pData+7) - 0x30) * 10;
			i32MiliMinutos += (*(pData+8) - 0x30);
		}break;
		default:
		{
			u8Grados = 0;
			u8Minutos = 0;
			i32MiliMinutos = 0;
		}break;
	}
	lReturn = (u8Grados * 60000) + ( u8Minutos * 1000 ) + i32MiliMinutos;
	
	return(lReturn);
}

uint32_t gpsGetSpeed(uint8_t *pData)
{
	uint8_t ucDataLen = 0;
	uint8_t *pNextData;
	uint32_t ulReturn = 0;

	/* Obtain Field Len */
	pNextData = gpsGetNextToken(pData);
	ucDataLen = (uint8_t)(pNextData - pData);
	
	switch(ucDataLen)
	{
		case 5:
		{
			ulReturn = ( *pData -0x30 ) * 100;
			ulReturn += ( *(pData+2) -0x30 ) * 10;
			ulReturn += ( *(pData+3) -0x30 );
		}break;
		case 6:
		{
			ulReturn = ( *pData -0x30 ) * 1000;
			ulReturn += ( *(pData+1) -0x30 ) * 100;
			ulReturn += ( *(pData+3) -0x30 ) * 10;
			ulReturn += ( *(pData+4) -0x30 );
		}break;
		case 7:
		{
			ulReturn = ( *pData -0x30 ) * 10000;
			ulReturn += ( *(pData+1) -0x30 ) * 1000;
			ulReturn += ( *(pData+2) -0x30 ) * 100;
			ulReturn += ( *(pData+4) -0x30 ) * 10;
			ulReturn += ( *(pData+5) -0x30 );
		}break;
	}
	return(ulReturn);
}

void gpsGetDate(uint8_t *pData)
{
	
	uint8_t ucYear = 0;
	
	ucYear = (*(pData+4) -0x30) *10;
	ucYear += (*(pData+5) -0x30);
	GPSTempData.sDateTime.Date.u16Year = 2000 + ucYear;
	
	GPSTempData.sDateTime.Date.u8Month = (*(pData+2) -0x30) *10;
	GPSTempData.sDateTime.Date.u8Month += (*(pData+3) -0x30);

	GPSTempData.sDateTime.Date.u8Day = (*(pData) -0x30) *10;
	GPSTempData.sDateTime.Date.u8Day += (*(pData+1) -0x30);
	
}

void gpsGetDataRMC(uint8_t *pframe)
{
	uint8_t *pData = pframe;
	uint32_t u32DesiredFields = sConfigNMEA[0].ui32FrameFields;
	uint32_t u32FieldIndex = 1;
	/* Is there at least one field desired from RMC frame? */
	if(u32DesiredFields != 0)
	{
		do{
			pData = gpsGetNextToken(pData);
			/* Does this particular field requires decodification? */
			if(u32DesiredFields & 0x0001)
			{
				switch(u32FieldIndex)
				{
					case RMC_TIME:
					{
						GPSTempData.sDateTime.lDaySecs = gpsGetTime(pData);
					}break;
					case RMC_STATUS:
					{
						GPSTempData.cStatus = *pData;
						if(GPSTempData.cStatus != 'A')
						{
							/* Trama inválida, no se continúa analizando */
							u32DesiredFields = 0;
						}
					}break;
					case RMC_LATITUDE:
					{
						/* Grados */
						GPSTempData.lLatitude = gpsGetCoordinate(pData);
					}break;
					case RMC_LAT_SIGN:
					{
						if(*pData == 'S')
						{
							GPSTempData.lLatitude *= -1;
						}
					}break;
					case RMC_LONGITUDE:
					{
						GPSTempData.lLongitude = gpsGetCoordinate(pData);
					}break;
					case RMC_LON_SIGN:
					{
						if(*pData == 'W')
						{
							GPSTempData.lLongitude *= -1;
						}
					}break;
					case RMC_SPEEDKNOT:
					{
						GPSTempData.ulSpeedKnot = gpsGetSpeed(pData);
					}break;
					case RMC_COURSE:
					{
						GPSTempData.ulTrueCourse = gpsGetSpeed(pData);
					}break;
					case RMC_DATE:
					{
						gpsGetDate(pData);
					}break;
				}
			}
			/* Rotate the field mask to rigth */
			u32DesiredFields >>= 1;
			u32FieldIndex<<= 1;
		}while( u32DesiredFields != 0);
	}
}

uint8_t gpsGetHdop(uint8_t *pData)
{
	uint8_t ucDataLen = 0;
	uint8_t *pNextData;
	uint8_t cReturn = 0;
	
	/* Obtain Field Len */
	pNextData = gpsGetNextToken(pData);
	ucDataLen = (uint8_t)(pNextData - pData);

	switch(ucDataLen)
	{
		case 4:
		{
			cReturn = (*(pData) - 0x30) * 10;
			cReturn += (*(pData+2) - 0x30);
		}break;
		case 5:
		{
			cReturn = (*(pData) - 0x30) * 100;
			cReturn += (*(pData+1) - 0x30) * 10;
			cReturn += (*(pData+3) - 0x30);
		}break;
		default:
		{
			cReturn = 255;
		}break;
	}
	return(cReturn);
}

int32_t gpsGetHeight(uint8_t *pData)
{
	uint8_t ucNegFlag = 0;
	uint8_t ucDataLen = 0;
	uint8_t *pNextData;
	int32_t lReturn = 0;

	if(*pData == '-')
	{
		/* La altura es negativa? */
		ucNegFlag = 1;
		pData++;
	}
	/* Obtain Field Len */
	pNextData = gpsGetNextToken(pData);
	ucDataLen = (uint8_t)(pNextData - pData);
	
	switch(ucDataLen)
	{
		case 7:
		{
			lReturn = (*pData - 0x30) * 10000;
			lReturn += (*(pData + 1) - 0x30) * 1000;
			lReturn += (*(pData + 2) - 0x30) * 100;
			lReturn += (*(pData + 3) - 0x30) * 10;
			lReturn += (*(pData + 5) - 0x30);
		}break;
		case 6:
		{
			lReturn += (*(pData + 0) - 0x30) * 1000;
			lReturn += (*(pData + 1) - 0x30) * 100;
			lReturn += (*(pData + 2) - 0x30) * 10;
			lReturn += (*(pData + 4) - 0x30);
		}break;
		case 5:
		{
			lReturn += (*(pData + 0) - 0x30) * 100;
			lReturn += (*(pData + 1) - 0x30) * 10;
			lReturn += (*(pData + 3) - 0x30);
		}break;
		case 4:
		{
			lReturn += (*(pData + 0) - 0x30) * 10;
			lReturn += (*(pData + 2) - 0x30);
		}break;
	}
	if(ucNegFlag == 1)
	{
		lReturn *= -1;
	}
	return(lReturn);
}

void gpsGetDataGGA(uint8_t *pframe)
{
	/* Is there a field desired from GGA frame? */
	uint8_t *pData = pframe;
	uint32_t u32DesiredFields = sConfigNMEA[1].ui32FrameFields;
	uint32_t u32FieldIndex = 1;
	/* Is there at least one field desired from RMC frame? */
	if(u32DesiredFields != 0)
	{
		do{
			pData = gpsGetNextToken(pData);
			/* Does this particular field requires decodification? */
			if(u32DesiredFields & 0x0001)
			{
				switch(u32FieldIndex)
				{
					case GGA_FIX_STATUS:
					{
						GPSTempData.ui8FixQuality = *pData;
						if(GPSTempData.ui8FixQuality != '1')
						{
							/* Trama inválida, no se continúa analizando */
							u32DesiredFields = 0;
						}
					}break;
					case GGA_NUMSATS:
					{
						GPSTempData.ui8NSats = (*(pData) -0x30) *10;
						GPSTempData.ui8NSats += (*(pData+1) -0x30);
					}break;
					case GGA_HDOP:
					{
						GPSTempData.u8Hdop = gpsGetHdop(pData);
					}break;
					case GGA_HEIGHT:
					{
						GPSTempData.ulHeight = gpsGetHeight(pData);
					}break;
				}
			}
			/* Rotate the field mask to rigth */
			u32DesiredFields >>= 1;
			u32FieldIndex<<= 1;
		}while( u32DesiredFields != 0);
	}
}

void gpsGetData(uint8_t *pdata, uint8_t gpsDataIndex)
{
	switch(gpsDataIndex)
	{
		/* RMC */
		case 0:
		{
			gpsGetDataRMC(pdata);
		}break;
		/* GGA */
		case 1:
		{
			gpsGetDataGGA(pdata);
		}break;
		/* VTG */
//		case 2:
//		{
//			gpsGetDataVTG(pdata);
//		}break;
	}
}
/*******************************************************************************
* Name: gpsFrameDecoder
* Developer(s): Cesar Sarmiento
* References:
* -----------------------------------------------------------------------------
* Notes: Copia desde la cola a un buffer local, envía a decodificación.
*******************************************************************************/
void gpsFrameDecoder(void)
{
	static uint8_t gpsRxTempBuff[GPS_RX_QUEUE_SIZE];
	uint8_t gpsRxSize = 0;
	uint8_t gpsRxIndex = 0;
	uint8_t gps_currRx = 0;

	gpsRxSize = uxQueueMessagesWaiting(gpsRxQueueHandle);
	if( gpsRxSize >= 3)
	{
		/* Copy from queue to local buffer */
		do
		{
			if( xQueueReceive( gpsRxQueueHandle, &gps_currRx, portMAX_DELAY == pdTRUE ) )
			{
				gpsRxTempBuff[gpsRxIndex] = gps_currRx;
			}
			gpsRxIndex++;
		}while(gpsRxIndex <= (gpsRxSize-2) );
		if( This_NmeaChecksumVerify(gpsRxTempBuff) == 1 )
		{
			/* The GPS CheckSum was verified */
			gps_currRx = gpsFrameOfInterest(gpsRxTempBuff);
			if(gps_currRx != 0xFF)
			{
				gpsGetData(gpsRxTempBuff, gps_currRx);
			}
			else
			{
				gpsShutDownFrame(gpsRxTempBuff);
			}
		}
	}
}

/* USER CODE END 4 */

static void MX_GPIO_GPS_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPS_ON_OFF_GPIO_Port, GPS_ON_OFF_Pin, GPIO_PIN_RESET);
	LL_GPIO_ResetOutputPin(GPS_PWR_ON_GPIO_Port,GPS_PWR_ON_Pin);
	
  /*Configure GPIO pin : GPS_ON_OFF_Pin */
  GPIO_InitStruct.Pin = GPS_PWR_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	LL_GPIO_Init(GPS_PWR_ON_GPIO_Port,&GPIO_InitStruct);
  //HAL_GPIO_Init(GPS_ON_OFF_GPIO_Port, &GPIO_InitStruct);

}

void gps_start(void)
{
	osThreadDef(GPSTask, StartTaskGPS, osPriorityNormal, 0, 512);
  GPSTaskHandle = osThreadCreate(osThread(GPSTask), NULL);

}

void gps_stop(void)
{
	
}


/* USART1 Rx Interrupt Private Function (GPS) */
/* USER CODE BEGIN gpsReceiveData */
/**
  * @brief  Function callback for receive byte from UART1.
  * @param  argument: Not used 
  * @retval None
  */
void gpsReceiveData(uint8_t pRxData)
{
static portBASE_TYPE xHigherPriorityTaskWoken;
	
	xHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(gpsRxQueueHandle, &pRxData, NULL);
	if( (pRxData == '\n') || (xQueueIsQueueFullFromISR(gpsRxQueueHandle) != pdFALSE))
	{
		xSemaphoreGiveFromISR( gpsSemaphoreFullRx, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );		
	}
}


/* USER CODE BEGIN Header_StartTaskGPS */
/**
  * @brief  Function implementing the GPSTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTaskGPS */
void StartTaskGPS(void const * argument)
{

  /* USER CODE BEGIN 5 */
	uint8_t gpsFirstByte = 0;
	
	MX_GPIO_GPS_Init();
  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPS_ON_OFF_GPIO_Port, GPS_ON_OFF_Pin, GPIO_PIN_SET);
	LL_GPIO_SetOutputPin(GPS_PWR_ON_GPIO_Port,GPS_PWR_ON_Pin);
  MX_USART1_UART_Init();
	/* Set the Interrupt Settings */
	MX_USART1_NVIC_Init();
  /* Create the semaphores(s) */
  /* definition and creation of gpsFullFrameSemaphore */
  gpsSemaphoreFullRx = xSemaphoreCreateBinary();
  /* Create the queue(s) */
  /* definition and creation of gpsRxQueue */
  gpsRxQueueHandle = xQueueCreate(GPS_RX_QUEUE_SIZE,sizeof(uint8_t));

	
	for (;;) 
	{
		/* Block the task until \n arrives from UART */
		if( xSemaphoreTake( gpsSemaphoreFullRx, portMAX_DELAY ) == pdTRUE)
		{
			/* Receive the first byte from the queue in gpsFirstByte */
			if( xQueueReceive( gpsRxQueueHandle, &gpsFirstByte, portMAX_DELAY ) == pdTRUE )
			{
				/* Verifies the head of Frame */
				if(gpsFirstByte == '$')
				{
					gpsFrameDecoder();
				}
			}
			xQueueReset(gpsRxQueueHandle);
		}
	}
  /* USER CODE END 5 */ 
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
