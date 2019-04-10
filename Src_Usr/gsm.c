/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : gsm.c
  * @brief          : Main program body
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
#include "gsm.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GSM_BEGIN_GAP_MAX 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart8;
QueueHandle_t gsmRxQueueHandle;
//uint8_t gsmRxQueueBuffer[ GSM_RX_QUEUE_SIZE * sizeof( uint8_t ) ];
//osStaticMessageQDef_t gsmRxQueueControlBlock;
//osSemaphoreId gsmFullFrameSemaphoreHandle;
SemaphoreHandle_t gsmFullFrameSemaphoreHandle;
/* Control variable to communicate to modem */
uint8_t gsmWaitForResponse = 0;
uint8_t gsmResponseReceived = 0;
uint8_t gsmCmdTimeout = 0;
/* Variable to control GSM Commands routine */
enum gsm_states gsm_state;
/* Task Handle */
osThreadId GSMTaskHandle;
/* USER CODE BEGIN PV */
/* Timer Handler */
TimerHandle_t gsm_cmd_timeout_timer;
TimerHandle_t gsm_uart_timeout_timer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//void MX_GPIO_GSM_Init(void);
//static void MX_USART8_UART_Init(void);
void gsmTurnOn(void);


static void MX_NVIC_USART8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USART1 Rx Interrupt Private Function (GPS) */
/* USER CODE END 0 */
void gsmSendFrame(uint8_t *pFrame, uint8_t ucSize)
{
	HAL_UART_Transmit_IT(&huart8,pFrame,ucSize);
}

uint8_t gsmTxData(uint8_t *pFrame, uint8_t ucSize)
{
	uint8_t u8Error = 0;
	if(gsmWaitForResponse == 0)
	{
		gsmSendFrame(pFrame, ucSize);
		gsmWaitForResponse = 1;
	}
	else
	{
		u8Error = 1;
	}
	xTimerStart(gsm_cmd_timeout_timer,2000);
	return(u8Error);
}


/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_USART8_Init(void)
{
  /* USART3_8_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_8_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART3_8_IRQn);
	/* Enable Receive Interrupt */
	__HAL_UART_ENABLE_IT(&huart8,UART_IT_RXNE);
}

/**
  * @brief USART8 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_USART8_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART8_Init 0 */
//
//  /* USER CODE END USART8_Init 0 */
//
//  /* USER CODE BEGIN USART8_Init 1 */
//
//  /* USER CODE END USART8_Init 1 */
//  huart8.Instance = USART8;
//  huart8.Init.BaudRate = 115200;
//  huart8.Init.WordLength = UART_WORDLENGTH_8B;
//  huart8.Init.StopBits = UART_STOPBITS_1;
//  huart8.Init.Parity = UART_PARITY_NONE;
//  huart8.Init.Mode = UART_MODE_TX_RX;
//  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart8) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN USART8_Init 2 */

  /* USER CODE END USART8_Init 2 */

//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_GSM_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin, GPIO_PIN_RESET);
	LL_GPIO_ResetOutputPin(GSM_PWRKEY_GPIO_Port,GSM_PWRKEY_Pin);

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GSM_PWR_ON_GPIO_Port, GSM_PWR_ON_Pin, GPIO_PIN_SET);
	LL_GPIO_ResetOutputPin(GSM_PWR_ON_GPIO_Port,GSM_PWR_ON_Pin);
	

  /*Configure GPIO pins : GSM_PWR_KEY_Pin  */
  GPIO_InitStruct.Pin = GSM_PWRKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_PWR_ON_Pin */
  GPIO_InitStruct.Pin = GSM_PWR_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  LL_GPIO_Init(GSM_PWR_ON_GPIO_Port, &GPIO_InitStruct);


}

/* USER CODE BEGIN 4 */

uint8_t *gsmResponseBegin(uint8_t *pRxData, uint16_t uiSize)
{
	uint8_t *pData1 = pRxData;
	uint8_t ucGap = 0;
	do{
		ucGap++;
	}while((ucGap <GSM_BEGIN_GAP_MAX)&&( *(pData1 + ucGap) < ' ' )&&(ucGap<uiSize));
	if((ucGap<GSM_BEGIN_GAP_MAX)&&(ucGap<uiSize))
	{
		pData1 += ucGap;
	}
	return(pData1);
	
}
void gsmResponseAnalize(uint8_t *pData, uint16_t uiSize)
{
	switch(gsm_state)
	{
		case GSM_ST_AT:
		{
			pData = gsmResponseBegin(pData,uiSize);
			if( (*pData == 'A')&&(*(pData+1) == 'T') )
			{
				gsmWaitForResponse = 0;
				gsm_state = GSM_ST_ATE0;
			}
		}break;
		case GSM_ST_ATE0:
		{
			pData = gsmResponseBegin(pData,uiSize);
			if( (*pData == 'A')&&(*(pData+1) == 'T')&&(*(pData+2) == 'E') )
			{
				gsmWaitForResponse = 0;
				gsm_state++;
			}
			else
			{
				if( (*pData == 'O')&&(*(pData+1) == 'K') )
				{
					gsmWaitForResponse = 0;
					gsm_state++;
				}
			}
		}break;
		case GSM_ST_CPIN:
		{
			pData = gsmResponseBegin(pData,uiSize);
			if( (*pData == '+')&&(*(pData+1) == 'C') )
			{
				gsmWaitForResponse = 0;
				gsm_state++;
			}
		}break;
		
		case GSM_ST_CREG:
		{
			/* +CREG: 0,1 +CREG:0,5 */
			pData = gsmResponseBegin(pData,uiSize);
			if( (*pData == '+')&&((*(pData+9) == '1')||(*(pData+9) == '5') ))
			{
				gsmWaitForResponse = 0;
				gsm_state = GSM_ST_CSQ;
			}
		}
		case GSM_ST_CSQ:
		{
			pData = gsmResponseBegin(pData,uiSize);
			
			if( *pData == '+')
			{
				gsmWaitForResponse = 0;
				gsm_state = GSM_ST_CPIN;
			}
		}
		default:
		{
			
		}break;
	}
}

void gsmSendCommandProcess(void)
{
	if(gsmWaitForResponse == 0)
	{
		switch(gsm_state)
			{
				case GSM_ST_PWR_ON:
				{
					gsmTurnOn();
					MX_USART8_UART_Init();
					MX_NVIC_USART8_Init();
					gsm_state = GSM_ST_AT;
					gsmWaitForResponse = 0;
					gsmCmdTimeout = 0;
					(void)xTimerStart(gsm_cmd_timeout_timer,2000);
				}break;
				case GSM_ST_AT:
				{
					(void)gsmTxData((uint8_t*)GSM_CMD_AT,sizeof(GSM_CMD_AT));
				}break;
				case GSM_ST_ATE0:
				{
					(void)gsmTxData((uint8_t*)GSM_CMD_ATE0,sizeof(GSM_CMD_ATE0));
				}break;
				case GSM_ST_CPIN:
				{
					(void)gsmTxData((uint8_t*)GSM_CMD_CPIN,sizeof(GSM_CMD_CPIN));
				}break;
				case GSM_ST_CREG:
				{
					(void)gsmTxData((uint8_t*)GSM_CMD_CREG,sizeof(GSM_CMD_CREG));
				}break;
				case GSM_ST_CSQ:
				{
					(void)gsmTxData((uint8_t*)GSM_CMD_CSQ,sizeof(GSM_CMD_CSQ));
				}break;
				default:
				{
					(void)gsmTxData((uint8_t*)GSM_CMD_AT,sizeof(GSM_CMD_AT));
				}break;
		}
	}	
}

void gsmReveivedMsgProcess(void)
{
	uint8_t gsmRxBuffer[GSM_RX_QUEUE_SIZE];
	uint16_t gsmRxSize;
	uint8_t gsmRxData;
	uint16_t gsmRxIndex = 0;
	
	if(gsmResponseReceived == 1)
	{
		gsmRxSize = uxQueueMessagesWaiting(gsmRxQueueHandle);
		do
		{
			
			if( xQueueReceive( gsmRxQueueHandle, &gsmRxData, portMAX_DELAY) == pdTRUE )
			{
				gsmRxBuffer[gsmRxIndex] = gsmRxData;
			}
			gsmRxIndex++;
		}while(gsmRxIndex < gsmRxSize );
		gsmResponseAnalize(gsmRxBuffer,gsmRxSize);
		xQueueReset(gsmRxQueueHandle);
		gsmResponseReceived = 0;
	}
}

void gsmTimeoutCommandProcess(void)
{
	if(gsmCmdTimeout == 1)
	{
		switch(gsm_state)
		{
			case GSM_ST_PWR_ON:
			{
				gsmTurnOn();
				gsmWaitForResponse = 0;
				MX_USART8_UART_Init();
				MX_NVIC_USART8_Init();
				gsm_state = GSM_ST_AT;
			}break;
			case GSM_ST_AT:
			{
				gsmWaitForResponse = 0;
			}break;
			default:
			{
				gsm_state--;
				gsmWaitForResponse = 0;
			}
		}
		gsmCmdTimeout = 0;
	}
}


void gsmTurnOn(void)
{
  
	/*Configure GPIO pin Output Level */
	LL_GPIO_SetOutputPin(GSM_PWR_ON_GPIO_Port,GSM_PWR_ON_Pin);
	osDelay(500);
  LL_GPIO_SetOutputPin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin);
	osDelay(1000);
  LL_GPIO_ResetOutputPin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin);
	//osDelay(2000);
}

void gsm_init(void)
{
	osThreadDef(GSMTask, StartTaskGSM, osPriorityNormal, 0, 512);
  GSMTaskHandle = osThreadCreate(osThread(GSMTask), NULL);
	gsm_state = GSM_ST_PWR_ON;
	gsmWaitForResponse = 0;
	gsmResponseReceived = 0;
	gsmCmdTimeout = 0;
}

/* USER CODE END 4 */

/* USART1 Rx Interrupt Private Function (GSM) */
/* USER CODE BEGIN gpsReceiveData */
/**
  * @brief  Function callback for receive byte from UART1.
  * @param  argument: Not used 
  * @retval None
  */
void gsmReceiveData(uint8_t pRxData)
{
static portBASE_TYPE xHigherPriorityTaskWoken;
	
	xHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(gsmRxQueueHandle, &pRxData, NULL);
	xTimerStartFromISR(gsm_uart_timeout_timer,&xHigherPriorityTaskWoken);
	if( /*(pRxData == '\n') || */(xQueueIsQueueFullFromISR(gsmRxQueueHandle) != pdFALSE))
	{
		xSemaphoreGiveFromISR( gsmFullFrameSemaphoreHandle, &xHigherPriorityTaskWoken );
		gsmResponseReceived = 1;
		xTimerStopFromISR(gsm_cmd_timeout_timer,&xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

/* GSM Command Timeout Callback Function */
void gsmCmdTimeoutCallback( TimerHandle_t xTimer )
{
	gsmCmdTimeout = 1;
	xSemaphoreGive(gsmFullFrameSemaphoreHandle);
}
void gsmUartTimeoutCallback( TimerHandle_t xTimer )
{
	gsmResponseReceived = 1;
	xSemaphoreGive(gsmFullFrameSemaphoreHandle);
}


/* USER CODE BEGIN Header_StartTaskGSM */
/**
* @brief Function implementing the GSMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGSM */
void StartTaskGSM(void const * argument)
{
  /* USER CODE BEGIN StartTaskGSM */
	MX_GPIO_GSM_Init();
	/* Send the pulse in pwr key pin */

  /* Create the semaphores(s) */
  /* definition and creation of gpsFullFrameSemaphore */
  //osSemaphoreDef(gsmFullFrameSemaphore);
  //gsmFullFrameSemaphoreHandle = osSemaphoreCreate(osSemaphore(gsmFullFrameSemaphore), 1);
	gsmFullFrameSemaphoreHandle = xSemaphoreCreateBinary();
	
  /* Create the queue(s) */
  /* definition and creation of gpsRxQueue */
  gsmRxQueueHandle = xQueueCreate(GSM_RX_QUEUE_SIZE, sizeof(uint8_t));
	
	/* Create the Timeout Timer for GSM Commands */
	gsm_cmd_timeout_timer = xTimerCreate("TmrGSMTimOut",2000,pdFALSE,(void *)0,gsmCmdTimeoutCallback);
	//(void)xTimerStart(gsm_cmd_timeout_timer,2000);
	gsm_uart_timeout_timer = xTimerCreate("TmrGSMUart",200,pdFALSE,(void *)1,gsmUartTimeoutCallback);
	xSemaphoreGive(gsmFullFrameSemaphoreHandle);
	
  /* Infinite loop */
  for(;;)
  {
		if( xSemaphoreTake( gsmFullFrameSemaphoreHandle, portMAX_DELAY ) == pdTRUE )
		{
			gsmReveivedMsgProcess();
			gsmSendCommandProcess();
			gsmTimeoutCommandProcess();
			
		}
  }
  /* USER CODE END StartTaskGSM */
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
