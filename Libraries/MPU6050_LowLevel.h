#ifndef _MPU6050_LOWLEVEL_h
#define _MPU6050_LOWLEVEL_h

void MPU6050_I2C_Init(void);
void MPU6050_GPIO_Init(void);

uint8_t MPU_Write(uint8_t addr, uint8_t data);
uint8_t MPU_WriteMultiple(uint8_t addr, uint8_t * data, uint8_t length);
uint8_t MPU_Read(uint8_t addr, uint8_t * buffer, uint8_t length);

uint8_t AK8963_Write(uint8_t addr, uint8_t data);
uint8_t AK8963_WriteMultiple(uint8_t addr, uint8_t * data, uint8_t length);
uint8_t AK8963_Read(uint8_t addr, uint8_t * buffer, uint8_t length);




#define MPU6500_DEG_PER_LSB_250  (float)((2 * 250.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_500  (float)((2 * 500.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_1000 (float)((2 * 1000.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)

#define MPU6500_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
#define MPU6500_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
#define MPU6500_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6500_G_PER_LSB_16     (float)((2 * 16) / 65536.0)

// Test limits
#define MPU6500_ST_GYRO_LOW      (-14.0)  // %
#define MPU6500_ST_GYRO_HIGH     14.0  // %
#define MPU6500_ST_ACCEL_LOW     (-14.0)  // %
#define MPU6500_ST_ACCEL_HIGH    14.0  // %



#endif	// _MPU6050_LOWLEVEL_h
