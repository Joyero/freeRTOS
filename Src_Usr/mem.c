#include "mem.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

uint8_t uBTxArray[10];
uint8_t uBRxArray[10];

uint8_t memReadStatus(void)
{
	/*Read Status Procedure */
	/* Drive Chip Select to Low */
	LL_GPIO_ResetOutputPin(MEM_SPICS_GPIO_Port,MEM_SPICS_Pin);
	/* Charge the first position of TX Buffer with Status Address */
	uBTxArray[0]= MEM_READ_STATUS_REG1;
	HAL_SPI_Transmit(&hspi1,uBTxArray,1,100);
	HAL_SPI_Receive(&hspi1,uBRxArray,1,100);
	/* Drive Chip Select to High */
	LL_GPIO_SetOutputPin(MEM_SPICS_GPIO_Port,MEM_SPICS_Pin);
	return(uBRxArray[0]);
}


void memWriteEnable(void)
{
	/* Drive Chip Select to Low */
	LL_GPIO_ResetOutputPin(MEM_SPICS_GPIO_Port,MEM_SPICS_Pin);
	uBTxArray[0]= MEM_WRITE_ENABLE_CMD;
	HAL_SPI_Transmit(&hspi1,uBTxArray,1,100);
	/* Drive Chip Select to High */
	LL_GPIO_SetOutputPin(MEM_SPICS_GPIO_Port,MEM_SPICS_Pin);
}

void memWriteDisable(void)
{
	/* Drive Chip Select to Low */
	LL_GPIO_ResetOutputPin(MEM_SPICS_GPIO_Port,MEM_SPICS_Pin);
	uBTxArray[0]= MEM_WRITE_ENABLE_CMD;
	HAL_SPI_Transmit(&hspi1,uBTxArray,1,100);
	/* Drive Chip Select to High */
	LL_GPIO_SetOutputPin(MEM_SPICS_GPIO_Port,MEM_SPICS_Pin);
}

uint16_t memReadManufDeviceID(void)
{
	/* Drive Chip Select to Low */
	LL_GPIO_ResetOutputPin(MEM_SPICS_GPIO_Port,MEM_SPICS_Pin);
	/* Charge the first position of TX Buffer with Status Address */
	uBTxArray[0]= MEM_MANUF_DEVICE_ID_ADD;
	uBTxArray[1]= 0x33; //Dummy
	uBTxArray[2]= 0x33; //Dummy
	HAL_SPI_Transmit(&hspi1,uBTxArray,3,100);
	HAL_SPI_Receive(&hspi1,uBRxArray,3,100);
	/* Drive Chip Select to High */
	LL_GPIO_SetOutputPin(MEM_SPICS_GPIO_Port,MEM_SPICS_Pin);
	return(uBRxArray[1]<<8 | uBRxArray[2]);
}



void mem_init(void)
{
	MX_SPI1_Init();
	
	
	if(memReadStatus()==0x00)
	{
		if(memReadManufDeviceID()== 0xEF14)
		{
			uBRxArray[1] = 0xFF;
		}
	}
}
