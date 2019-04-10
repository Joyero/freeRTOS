#include "i2c.h"
#include "accel.h"


uint8_t accBuff[7];
uint16_t xAxis;
uint16_t yAxis;
uint16_t zAxis;
uint16_t iTemp;
uint8_t accelStatus;
uint8_t reference;



void accel_Init(void)
{
	MX_I2C1_Init();
	/* Who I am Register Address 0x0F it's to verify two way communication with Accel */
	accBuff[0] = ACC_REG_WHO_I_AM;
	if(HAL_I2C_Master_Transmit(&hi2c1, (ACCEL_I2C_ADDRESS<<1),accBuff,1,100) == HAL_OK)
	{
		/* Retreive the Who I am Register response */
		HAL_I2C_Master_Receive(&hi2c1,ACCEL_I2C_ADDRESS<<1,accBuff,1,100);
		/* The response should be in accBuff[0] */
		if(accBuff[0] == 0x33)
		{
			/* 0x1F it's the temperature conversion register */
			accBuff[0] = 0x1F;
			/* 0x40 means that temperature conversion it's enabled */
			accBuff[1] = 0x40;
			HAL_I2C_Master_Transmit(&hi2c1, ACCEL_I2C_ADDRESS<<1,accBuff,2,100);
			/* 0x1E it's the ConfigReg0 register  Pull Up Enable*/
			accBuff[0] = 0x1E;
			/* 0x40 means that temperature conversion it's enabled */
			accBuff[1] = 0x90;
			HAL_I2C_Master_Transmit(&hi2c1, ACCEL_I2C_ADDRESS<<1,accBuff,2,100);
			/* Config Ctrl1 Reg 0x20 to set conversion rate to 200 Hz */
			accBuff[0]=0x20;
			//accBuff[1]=0x67;
			accBuff[1]=0x17;
			HAL_I2C_Master_Transmit(&hi2c1, ACCEL_I2C_ADDRESS<<1,accBuff,2,100);
			/* Config Ctrl1 Reg 0x23 to set resolution and g Range */
			accBuff[0]=0x23;
			accBuff[1]=0x2A;
			HAL_I2C_Master_Transmit(&hi2c1, ACCEL_I2C_ADDRESS<<1,accBuff,2,100);
			/* Config Ctrl1 Reg 0x24 to enable FIFO */
			accBuff[0]=0x24;
			accBuff[1]=0x40;
			HAL_I2C_Master_Transmit(&hi2c1, ACCEL_I2C_ADDRESS<<1,accBuff,2,100);
			/* Config Ctrl1 Reg 0x2E to config FIFO in stream mode */
			accBuff[0]=0x2E;
			accBuff[1]=0x80;
			HAL_I2C_Master_Transmit(&hi2c1, ACCEL_I2C_ADDRESS<<1,accBuff,2,100);
			/*Temperature Read*/
			accBuff[0] = 0x26;
			HAL_I2C_Master_Receive(&hi2c1,ACCEL_I2C_ADDRESS<<1,accBuff,1,100);
			reference = accBuff[0];
			/*Temperature Read*/
			accBuff[0] = 0x0C;
			HAL_I2C_Master_Receive(&hi2c1,ACCEL_I2C_ADDRESS<<1,accBuff,1,100);
			iTemp = accBuff[1]<<8 | accBuff[0];
			
			for(;;)
			{
				/* Read Status */
				accBuff[0]= 0x27;
				HAL_I2C_Master_Transmit(&hi2c1, ACCEL_I2C_ADDRESS<<1,accBuff,1,100);
				HAL_I2C_Master_Receive(&hi2c1,ACCEL_I2C_ADDRESS<<1,accBuff,1,100);
				accelStatus = accBuff[0];
				/* Read Accel Sample*/
				accBuff[0] = 0x28;
				HAL_I2C_Master_Transmit(&hi2c1, ACCEL_I2C_ADDRESS<<1,accBuff,1,100);
				HAL_I2C_Master_Receive(&hi2c1,ACCEL_I2C_ADDRESS<<1,accBuff,6,100);
				xAxis = accBuff[1]<<8 | accBuff[0];
				yAxis = accBuff[3]<<8 | accBuff[2];
				zAxis = accBuff[5]<<8 | accBuff[4];
				HAL_Delay(200);
			}
		}
	}
}
