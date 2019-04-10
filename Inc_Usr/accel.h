
#define ACCEL_I2C_ADDRESS 0x19

#define ACC_REG_CTRL_REG_AUX	0x07
#define ACC_REG_ADC1_L				0x08
#define ACC_REG_ADC1_H				0x09
#define ACC_REG_ADC2_L				0x0A
#define ACC_REG_ADC2_H				0x0B
#define ACC_REG_ADC3_L				0x0C
#define ACC_REG_ADC3_H				0x0D
#define ACC_REG_WHO_I_AM			0x0F

#define ACC_REG_CTRL0					0x1E
#define ACC_REG_TEMP_CFG_REG	0x1F
#define ACC_REG_CTRL1					0x20
#define ACC_REG_CTRL2					0x21
#define ACC_REG_CTRL3					0x22
#define ACC_REG_CTRL4					0x23
#define ACC_REG_CTRL5					0x24
#define ACC_REG_CTRL6					0x25
#define ACC_REG_REFERENCE			0x26
#define ACC_REG_STATUS_REG		0x27

#define ACC_REG_OUT_X_L				0x28
#define ACC_REG_OUT_X_H				0x29
#define ACC_REG_OUT_Y_L				0x2A
#define ACC_REG_OUT_Y_H				0x2B
#define ACC_REG_OUT_Z_L				0x2C
#define ACC_REG_OUT_Z_H				0x2D

#define ACC_FIFO_CTRL_REG			0x2E
#define ACC_FIFO_SRC_REG			0c2F

#define ACC_INT1_CFG					0x30
#define ACC_INT1_SRC					0x31
#define ACC_INT1_THS					0x32
#define ACC_INT1_DURATION			0x33

#define ACC_INT2_CFG					0x34
#define ACC_INT2_SRC					0x35
#define ACC_INT2_THS					0x36
#define ACC_INT2_DURATION			0x37

#define ACC_CLICK_CFG					0x38
#define ACC_CLICK_SRC					0x39
#define ACC_CLICK_THS					0x3A

#define ACC_TIME_LIMIT				0x3B
#define ACC_TIME_LATENCY			0x3C
#define ACC_TIME_WINDOW				0x3D

#define ACC_ACT_THS						0x3E
#define ACC_ACT_DUR						0x3F


/*
START UP SEQUENCE

Write CTRL_REG1
Write CTRL_REG2
Write CTRL_REG3
Write CTRL_REG4
Write CTRL_REG5
Write CTRL_REG6
Write REFERENCE
Write INTx_THS
Write INTx_DUR 
Write INTx_CFG
Write CTRL_REG5

*/





void accel_Init(void);
