#ifndef _XV11_LIDAR_h
#define _XV11_LIDAR_h

#include "cmsis_os.h"

#define LIDAR_UART				USART1
#define LIDAR_UART_IRQn         USART1_IRQn
#define LIDAR_DMA_ENABLE		__HAL_RCC_DMA2_CLK_ENABLE
#define LIDAR_UART_ENABLE		__HAL_RCC_USART1_CLK_ENABLE
#define LIDAR_UART_GPIO_ENABLE	__GPIOB_CLK_ENABLE
#define LIDAR_UART_GPIO_PORT	GPIOB
#define LIDAR_UART_TX_PIN		GPIO_PIN_6
#define LIDAR_UART_RX_PIN		GPIO_PIN_7

#define LIDAR_DMA_RX_STREAM              DMA2_Stream5
#define LIDAR_DMA_RX_CHANNEL             DMA_CHANNEL_4
#define LIDAR_DMA_RX_IRQn                DMA2_Stream5_IRQn
//#define LIDAR_DMA_RX_IRQHandler          DMA2_Stream5_IRQHandler

#define LIDAR_UART_BAUD			115200
#define LIDAR_PACKAGE			22
#define LIDAR_DMA_BUFFER		(50*LIDAR_PACKAGE)
#define LIDAR_BUFFER_SIZE		(200*LIDAR_PACKAGE)

#define LIDAR_TIMEOUT			500		// if no packages received for 500 ms then restart DMA (this usually happens when debugging and stopping on a breakpoint)

extern SemaphoreHandle_t XV11_Semaphore;
extern uint32_t LiDAR_LastPackageTimestamp;

void LiDAR_Init(void);
void LiDAR_GPIO_Init(void);
void LiDAR_UART_Init(void);
void LiDAR_DMA_Init(void);

void LiDAR_Restart(void);
void LiDAR_Transmit(uint8_t * buffer, uint16_t len);
void Debug(uint8_t * text);
void LiDAR_Parser(void const * argument);
void ParsePackage(uint8_t * packagePointer);
uint16_t PackageChecksum(uint8_t * packagePointer);
void LiDAR_SpeedController(void const * argument);

#endif	// _XV11_LIDAR_h
