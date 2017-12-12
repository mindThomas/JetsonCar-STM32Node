#include "stm32f4xx_hal.h"
#include "XV11_LiDAR.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"
#include "cmsis_os.h"

// DMA inspired by https://stm32f4-discovery.net/2017/07/stm32-tutorial-efficiently-receive-uart-data-using-dma/

UART_HandleTypeDef UartHandle;
DMA_HandleTypeDef hdma_rx;
extern TIM_HandleTypeDef htim12;

uint8_t LiDAR_Buffer[LIDAR_BUFFER_SIZE];

osThreadId LiDAR_ParserTaskHandle;
osThreadId LiDAR_SpeedControllerTaskHandle;
SemaphoreHandle_t XV11_Semaphore = NULL;

static uint16_t readOffset;
static uint16_t writeOffset;
static uint16_t writeRolloverPos = LIDAR_BUFFER_SIZE;
static uint16_t DMAReadSize = 0;

static float LiDAR_Speed = 0;

extern xQueueHandle xExternal;

void LiDAR_Init(void)
{
	readOffset = 0;
	writeOffset = 0;
	XV11_Semaphore = xSemaphoreCreateBinary();

	LiDAR_GPIO_Init();
	LiDAR_UART_Init();
	LiDAR_DMA_Init();

	osThreadDef(LiDAR_ParserTask, LiDAR_Parser, osPriorityNormal, 0, 512);
	LiDAR_ParserTaskHandle = osThreadCreate(osThread(LiDAR_ParserTask), NULL);

	osThreadDef(LiDAR_SpeedControllerTask, LiDAR_SpeedController, osPriorityNormal, 0, 128);
	LiDAR_SpeedControllerTaskHandle = osThreadCreate(osThread(LiDAR_SpeedControllerTask), NULL);
}

void LiDAR_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	LIDAR_UART_GPIO_ENABLE();

	GPIO_InitStruct.Pin = LIDAR_UART_TX_PIN  | LIDAR_UART_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(LIDAR_UART_GPIO_PORT, &GPIO_InitStruct);
}

void LiDAR_UART_Init(void)
{
	LIDAR_UART_ENABLE();
	  /*##-1- Configure the UART peripheral ######################################*/
	  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	  /* UART configured as follows:
	      - Word Length = 8 Bits
	      - Stop Bit    = One Stop bit
	      - Parity      = No parity
	      - BaudRate    = 9600 baud
	      - Hardware flow control disabled (RTS and CTS signals) */
	  UartHandle.Instance          = LIDAR_UART;

	  UartHandle.Init.BaudRate     = LIDAR_UART_BAUD;
	  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	  UartHandle.Init.StopBits     = UART_STOPBITS_1;
	  UartHandle.Init.Parity       = UART_PARITY_NONE;
	  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	  UartHandle.Init.Mode         = UART_MODE_TX_RX;
	  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	  if (HAL_UART_Init(&UartHandle) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }

	// Enable UART interrupt
	HAL_NVIC_SetPriority(LIDAR_UART_IRQn, 6, 1);
	HAL_NVIC_EnableIRQ(LIDAR_UART_IRQn);

	//SET_BIT(LIDAR_UART->CR1, USART_CR1_IDLEIE);  // Enable IDLE line detection for DMA processing
}

void LiDAR_DMA_Init(void)
{
	uint8_t * ptr;
	uint32_t tmp;

	LIDAR_DMA_ENABLE();

	  /* Configure the DMA handler for reception process */
	  hdma_rx.Instance                 = LIDAR_DMA_RX_STREAM;
	  hdma_rx.Init.Channel             = LIDAR_DMA_RX_CHANNEL;
	  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
	  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
	  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	  hdma_rx.Init.Mode                = DMA_NORMAL;
	  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
	  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	  hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
	  hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

	  HAL_DMA_Init(&hdma_rx);

	  /* Associate the initialized DMA handle to the the UART handle */
	  __HAL_LINKDMA(&UartHandle, hdmarx, hdma_rx);

	  /* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
	  HAL_NVIC_SetPriority(LIDAR_DMA_RX_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(LIDAR_DMA_RX_IRQn);

	  HAL_DMA_Start_IT(&hdma_rx, (uint32_t)&UartHandle.Instance->DR, (uint32_t)LiDAR_Buffer, LIDAR_DMA_BUFFER);
	  __HAL_UART_CLEAR_OREFLAG(&UartHandle);
	  SET_BIT(UartHandle.Instance->CR3, USART_CR3_DMAR);
}


/**
 * \brief       Global interrupt handler for USART2
 */
void USART1_IRQHandler(void) {
    /* Check for IDLE flag */
    if (USART1->SR & USART_FLAG_IDLE) {         /* We want IDLE flag only */
        /* This part is important */
        /* Clear IDLE flag by reading status register first */
        /* And follow by reading data register */
        volatile uint32_t tmp;                  /* Must be volatile to prevent optimizations */
        tmp = USART1->SR;                       /* Read status register */
        tmp = USART1->DR;                       /* Read data register */
        (void)tmp;                              /* Prevent compiler warnings */
        __HAL_DMA_DISABLE(&hdma_rx);            /* Disabling DMA will force transfer complete interrupt if enabled */
    }
}

/**
 * \brief       Global interrupt handler for DMA1 stream5
 * \note        Except memcpy, there is no functions used to
 */
void DMA2_Stream5_IRQHandler(void) {
	uint16_t i;
    size_t len;
    portBASE_TYPE xHigherPriorityTaskWoken;
    portBASE_TYPE status = pdFALSE;

    /* Check transfer complete flag */
    if (DMA2->HISR & DMA_HISR_TCIF5) {
        DMA2->HIFCR = DMA_HISR_TCIF5;           /* Clear transfer complete flag */

        /* Calculate number of bytes actually transfered by DMA so far */
        /**
         * Transfer could be completed by 2 events:
         *  - All data actually transfered (NDTR = 0)
         *  - Stream disabled inside USART IDLE line detected interrupt (NDTR != 0)
         */
        len = DMAReadSize - DMA2_Stream5->NDTR;

        writeOffset += len;

        DMAReadSize = LIDAR_DMA_BUFFER;
        /*if (writeOffset >= LIDAR_BUFFER_SIZE) {
        	writeOffset = 0;
        }
        else if ((LIDAR_BUFFER_SIZE-writeOffset) < LIDAR_DMA_BUFFER) { // fill up buffer completely
        	DMAReadSize = (LIDAR_BUFFER_SIZE-writeOffset)-1;
        }*/

        //Debug("DATA\n");

        if ((writeOffset+LIDAR_DMA_BUFFER) > LIDAR_BUFFER_SIZE) {
        	writeRolloverPos = writeOffset;
        	writeOffset = 0;
        	//Debug("Rollover\n");
        }

        /* Prepare DMA for next transfer */
        /* Important! DMA stream won't start if all flags are not cleared first */
        DMA2->HIFCR =  DMA_HISR_DMEIF5 | DMA_HISR_FEIF5 | DMA_HISR_HTIF5 | DMA_HISR_TCIF5 | DMA_HISR_TEIF5;
        DMA2_Stream5->M0AR = (uint32_t)&LiDAR_Buffer[writeOffset];   /* Set memory address for DMA again */
        DMA2_Stream5->NDTR = LIDAR_DMA_BUFFER;    /* Set number of bytes to receive */
        __HAL_DMA_ENABLE(&hdma_rx);				   /* Re-enable the DMA transfer */



        //status = xQueueSendToBackFromISR( xExternal, &i, &xHigherPriorityTaskWoken );
        xSemaphoreGiveFromISR( XV11_Semaphore, &xHigherPriorityTaskWoken );

        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
}

void LiDAR_Parser(void const * argument)
{
	uint16_t len;
	//uint8_t text_buffer[50];
	//sprintf(text_buffer, "New package\n");

	while (1) {
		/*if (pdTRUE == xQueueReceive( xExternal, &len, portMAX_DELAY)) {
        	sprintf(text_buffer, "Length: %d\n", len);
        	LiDAR_Transmit(text_buffer, strlen(text_buffer));
		}*/

		if( xSemaphoreTake( XV11_Semaphore, portMAX_DELAY ) == pdTRUE ) {
			//LiDAR_Transmit(text_buffer, strlen(text_buffer));
			ParseIncomingData();
		}
	}
}

uint16_t StartBytePosition = 0;
void ParseIncomingData() {
	uint8_t text_buffer[50];
	uint8_t Package[22];
	uint16_t writeOffset_, writeRolloverPos_;
	uint16_t availableDataLength;
	uint8_t i;

	//Debug("Starting parsing\n");

	if (writeOffset > writeRolloverPos) {
		Debug("------------Resetting RolloverPos-------------\n");
		writeRolloverPos = writeOffset;
	}

	writeRolloverPos_ = writeRolloverPos;
	writeOffset_ = writeOffset;

	do {
		while (LiDAR_Buffer[readOffset] != 0xFA && readOffset != writeOffset_) {
			readOffset = (readOffset+1) % writeRolloverPos_;
		}

		if (readOffset > writeOffset_) {
			availableDataLength = (writeRolloverPos_ - readOffset) + writeOffset_;
			//Debug("Taking rollover into consideration\n");
		} else {
			availableDataLength = writeOffset_ - readOffset;
		}

		if (availableDataLength > 0) {
			if (LiDAR_Buffer[readOffset] == 0xFA && LiDAR_Buffer[(readOffset+1) % writeRolloverPos_] >= 0xA0) {
				if (availableDataLength >= 22) {
					//sprintf(text_buffer, "Found package with ID %d\n", LiDAR_Buffer[(readOffset+1) % writeRolloverPos]);
					//LiDAR_Transmit(text_buffer, strlen(text_buffer));

					for (i = 0; i < 22; i++) {
						Package[i] = LiDAR_Buffer[readOffset];
						readOffset = (readOffset+1) % writeRolloverPos_;
					}
					ParsePackage(Package);

					//readOffset = (readOffset+22) % writeRolloverPos_;
				} else {
					//sprintf(text_buffer, "Found start byte belonging to: %d but without enough data\n", LiDAR_Buffer[(readOffset+1) % writeRolloverPos]);
					break;
				}
			} else { // incorrect package, so continue parsing
				readOffset = (readOffset+1) % writeRolloverPos_;
			}
		}
	} while (availableDataLength > 0);

	//Debug("Finished parsing\n");
}

uint16_t Distance[360];
uint16_t GoodReadings = 0, BadReadings = 0;
uint16_t AnglesCovered = 0;
void ParsePackage(uint8_t * packagePointer)
{
	uint16_t i;
	uint16_t Index;
	uint16_t Speed16;
	float Speed;
	uint8_t InvalidFlag[4];
	uint8_t WarningFlag[4];
	uint16_t Checksum, ChecksumCalculated;

	if (packagePointer[0] != 0xFA) {
		Debug("Something BAD happened as the package for parsing is not correct!\n");
		return;
	}

	Checksum = ((uint16_t)packagePointer[21] << 8) | packagePointer[20];
	ChecksumCalculated = PackageChecksum(packagePointer);
	if (Checksum != ChecksumCalculated) {
		BadReadings += 4;
		return;
	}
	if (packagePointer[1] < 0xA0) {
		BadReadings += 4;
		return;
	}

	Index = (packagePointer[1] - 0xA0) * 4;
	Speed16 = ((uint16_t)packagePointer[3] << 8) | packagePointer[2];
	Speed = (float)Speed16 / 64;
	InvalidFlag[0] = (packagePointer[5] & 0x80) >> 7;
	InvalidFlag[1] = (packagePointer[9] & 0x80) >> 7;
	InvalidFlag[2] = (packagePointer[13] & 0x80) >> 7;
	InvalidFlag[3] = (packagePointer[17] & 0x80) >> 7;
	WarningFlag[0] = (packagePointer[5] & 0x40) >> 6;
	WarningFlag[1] = (packagePointer[9] & 0x40) >> 6;
	WarningFlag[2] = (packagePointer[13] & 0x40) >> 6;
	WarningFlag[3] = (packagePointer[17] & 0x40) >> 6;

	if (Index > 356) {
		BadReadings += 4;
		return;
	}

	if (Index == 0) {
		AnglesCovered = 0;
		for (i = 0; i < 360; i++) {
			if (Distance[i] > 0) AnglesCovered++;
		}

		GoodReadings = 0;
		BadReadings = 0;
	}

	LiDAR_Speed = 0.8*LiDAR_Speed + 0.2*Speed;

	for (i = 0; i < 4; i++) {
		if (!InvalidFlag[i])
		{
			Distance[Index+i] = packagePointer[4+(i*4)] | ((uint16_t)(packagePointer[5+(i*4)] & 0x3F) << 8);
			GoodReadings++;
		} else {
			Distance[Index+i] = 0;
			BadReadings++;
		}
	}

}

uint16_t PackageChecksum(uint8_t * packagePointer)
{
	uint8_t i;
	uint16_t data[10];
	uint16_t checksum;
	uint32_t chk32;

	// group the data by word, little-endian
	for (i = 0; i < 10; i++) {
		data[i] = packagePointer[2*i] | (((uint16_t)packagePointer[2*i+1]) << 8);
	}

	// compute the checksum on 32 bits
	chk32 = 0;
	for (i = 0; i < 10; i++) {
    	chk32 = (chk32 << 1) + data[i];
	}

   // return a value wrapped around on 15bits, and truncated to still fit into 15 bits
   checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ); // wrap around to fit into 15 bits
   checksum = checksum & 0x7FFF; // truncate to 15 bits
}

void LiDAR_Transmit(uint8_t * buffer, uint16_t len)
{
	do {
		//if(UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, Timeout) != HAL_OK)
		while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TXE) == RESET);
		UartHandle.Instance->DR = *buffer++;
	} while (--len > 0);
}

void Debug(uint8_t * text)
{
	uint16_t len = strlen(text);
	do {
		//if(UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, Timeout) != HAL_OK)
		while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TXE) == RESET);
		UartHandle.Instance->DR = *text++;
	} while (--len > 0);
}

void LiDAR_SpeedController(void const * argument)
{
	uint8_t text_buffer[50];
	uint16_t PWM;
	float LiDAR_Speed_Setpoint = 300;
	float err;
	float P = 0;
	float I = 895;

	while (1) {
		err = LiDAR_Speed_Setpoint - LiDAR_Speed;
		if (err > 50) err = 50;
		if (err < -50) err = -50;
		P = 1 * err;
		I = I + 0.02 * err;
		if (I > 1000) I = 1000;
		if (I < -1000) I = -1000;

		PWM = P + I;
		if (PWM > 999) PWM = 999;

		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, PWM);

		/*sprintf(text_buffer, "Speed: %d\n", (uint16_t)LiDAR_Speed);
		LiDAR_Transmit(text_buffer, strlen(text_buffer));*/
		osDelay(10);
	}
}
