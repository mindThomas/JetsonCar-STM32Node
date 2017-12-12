#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "MPU6050_LowLevel.h"

I2C_HandleTypeDef hi2c1;

#define MPU6050_ADDR	0x68
#define TIMEOUT	10 // milliseconds

void MPU6050_I2C_Init(void)
{
	__I2C1_CLK_ENABLE();

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x20303E5D; //0x2000090E;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	HAL_I2C_Init(&hi2c1);

	//Configure Analogue filter 
	HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);
}

void MPU6050_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__GPIOB_CLK_ENABLE();	
	
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

uint8_t MPU_Write(uint8_t addr, uint8_t data)
{
    uint8_t ret = 0; // sucess
    uint8_t buf[] = {addr, data};

    //__disable_irq();
    if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, buf, 2, TIMEOUT) != HAL_OK) {
        ret = 0xFF;
    }
    //__enable_irq();
    return ret;
}

uint8_t MPU_WriteMultiple(uint8_t addr, uint8_t * data, uint8_t length)
{
    uint8_t ret = 0; // sucess
    uint8_t buf[20];
    uint8_t i;

    buf[0] = addr;
    for (i = 0; i < length; i++) {
    	buf[i+1] = data[i];
    }

    //__disable_irq();
    if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, buf, (length + 1), TIMEOUT) != HAL_OK) {
        ret = 0xFF;
    }
    //__enable_irq();
    return ret;
}

uint8_t MPU_Read(uint8_t addr, uint8_t * buffer, uint8_t length)
{
    uint8_t ret = 0; // sucess
    HAL_StatusTypeDef errCode;

    //__disable_irq();

    //while(HAL_I2C_IsDeviceReady(mpu_i2c_handle, MPU_Addr << 1, trials, timeout) != HAL_OK) ;
    errCode = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, &addr, 1, TIMEOUT);
    if (errCode != HAL_OK) {
    	ret = 0xFF;
        goto error_w;
    }
    if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR << 1, buffer, length, TIMEOUT) != HAL_OK) {
    	ret = 0xFF;
    }

error_w:
    //__enable_irq();
    return ret;
}
