#include "communication.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "crc.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

osThreadId CommunicationTransmitterTaskHandle;
osThreadId CommunicationReceiverTaskHandle;

SemaphoreHandle_t CommunicationReceiveSemaphore = NULL;
QueueHandle_t LiDAR_Message_Queue;

void Communication_Init(void)
{
	CommunicationReceiveSemaphore = xSemaphoreCreateBinary();
	LiDAR_Message_Queue = xQueueCreate( 100, sizeof(LiDAR_Message) );

	osThreadDef(CommunicationTransmitterTask, Communication_Transmitter, osPriorityNormal, 0, 256);
	CommunicationTransmitterTaskHandle = osThreadCreate(osThread(CommunicationTransmitterTask), NULL);

	osThreadDef(CommunicationReceiverTask, Communication_Receiver, osPriorityNormal, 0, 256);
	CommunicationReceiverTaskHandle = osThreadCreate(osThread(CommunicationReceiverTask), NULL);
}


void Communication_Transmitter(void const * argument)
{
	int i;
	uint16_t packCRC;
	uint8_t package[17];
	uint16_t counter = 0;
	LiDAR_Message meas[4];

	for (i = 0; i < 16; i++) {
		package[i] = 0;
	}

	while (CDC_Transmit_FS(package, sizeof(package)) != USBD_OK) { // send initial zero package - wait for communication channel to be opened
		osDelay(1);
	}

	package[0] = 0xFE;
	package[1] = 0xFE;
	package[2] = 0x10;

	while (1) {
		counter++;
		package[3] = counter >> 8;
		package[4] = counter & 0xFF;

		if (uxQueueMessagesWaiting(LiDAR_Message_Queue) >= 4) {
			for (i = 0; i < 4; i++) {
				if( xQueueReceive( LiDAR_Message_Queue, &(meas[i]), ( TickType_t ) 10 ) )
				{
					package[3+3*i] = (meas[i].ID & 0x1FF) >> 1;
					package[3+3*i+1] = (meas[i].ID & 0x01) << 7 | ((meas[i].Distance & 0x7FFF) >> 9);
					package[3+3*i+2] = (meas[i].Distance & 0xFF);
				} else {
					i = -1;
					break;
				}
			}

			if (i == 4) {
				packCRC = crcFast(package, sizeof(package));
				package[15] = packCRC >> 8;
				package[16] = packCRC & 0xFF;
				CDC_Transmit_FS_ThreadBlocking(package, sizeof(package));
			}
		}

		osDelay(10);
	}
}

void Communication_Receiver(void const * argument)
{
	uint8_t buffer[20];
	while (1) {
		if( xSemaphoreTake( CommunicationReceiveSemaphore, portMAX_DELAY ) == pdTRUE ) {
			buffer[0] = 0;
			//sprintf(buffer, "I received something\n");
			//CDC_Transmit_FS(buffer, strlen(buffer));
		}
	}
}
