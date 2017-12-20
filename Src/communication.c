#include "communication.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "crc.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "XV11_LiDAR.h"
#include "Encoders.h"
#include "RCReceiver.h"

#define DISABLE_TRANSMITTER

osThreadId CommunicationTransmitterTaskHandle;
osThreadId CommunicationReceiverTaskHandle;

uint16_t TransmitInterval_Encoder = 10; // by default configure to 100 Hz = 10 ms
uint16_t TransmitInterval_RC = 20; // by default configure to 50 Hz = 20 ms

QueueHandle_t CommunicationReceiveQueue;
SemaphoreHandle_t CommunicationReceiveSemaphore = NULL;
QueueHandle_t LiDAR_Message_Queue;

void Communication_Init(void)
{
	CommunicationReceiveSemaphore = xSemaphoreCreateBinary();
	CommunicationReceiveQueue = xQueueCreate( COMM_RX_QUEUE_LENGTH, sizeof(Comm_Package) );
	LiDAR_Message_Queue = xQueueCreate( LIDAR_QUEUE_LENGTH, sizeof(LiDAR_Message) );

	osThreadDef(CommunicationTransmitterTask, Communication_Transmitter, osPriorityNormal, 0, 256);
	CommunicationTransmitterTaskHandle = osThreadCreate(osThread(CommunicationTransmitterTask), NULL);

	osThreadDef(CommunicationReceiverTask, Communication_Receiver, osPriorityNormal, 0, 256);
	CommunicationReceiverTaskHandle = osThreadCreate(osThread(CommunicationReceiverTask), NULL);
}


void Communication_Transmitter(void const * argument)
{
	int i;
	uint16_t packCRC;
	int32_t Encoder_Front, Encoder_Rear, Encoder_Future;
	uint16_t RC_Throttle, RC_Steering;
	uint32_t Timestamp;
	uint8_t * u8ptr;
	uint8_t package[COMM_PACKAGE_LENGTH];
	LiDAR_Message meas[4];
	uint16_t nextEncoderTransmitTimestamp = 0;
	uint16_t nextRCTransmitTimestamp = 0;

	for (i = 0; i < 16; i++) {
		package[i] = 0;
	}

	while (CDC_Transmit_FS(package, COMM_PACKAGE_LENGTH) != USBD_OK) { // send initial zero package - wait for communication channel to be opened
		osDelay(1);
	}

#ifdef DISABLE_TRANSMITTER
	while (1) {
		osDelay(1000);
	}
#endif

	while (1) {
		if (uxQueueMessagesWaiting(LiDAR_Message_Queue) >= 4) {
			// Prepare LiDAR package
			package[0] = 0xFE;
			package[1] = 0xFE;
			package[2] = 0x10;
			for (i = 0; i < 4; i++) {
				if( xQueueReceive( LiDAR_Message_Queue, &(meas[i]), ( TickType_t ) 10 ) == pdPASS )
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
				packCRC = crcFast(package, COMM_PACKAGE_LENGTH-2); // calculate CRC based on package content excluding CRC bytes
				package[15] = packCRC >> 8;
				package[16] = packCRC & 0xFF;
				CDC_Transmit_FS_ThreadBlocking(package, COMM_PACKAGE_LENGTH);
			}
		}

		if (xTaskGetTickCount() > nextEncoderTransmitTimestamp) {
			nextEncoderTransmitTimestamp = xTaskGetTickCount() + TransmitInterval_Encoder;

			Timestamp = xTaskGetTickCount();
			Encoders_GetCount(&Encoder_Front, &Encoder_Rear);

			// Prepare Encoder package     -   consider to include Encoder direction: __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)
			package[0] = 0xFE;
			package[1] = 0xFE;
			package[2] = 0x11;

			u8ptr = (uint8_t *)&Timestamp;
			package[3] = u8ptr[0]; // little-endian packing of integer, with least significant byte first
			package[4] = u8ptr[1];
			package[5] = u8ptr[2];
			package[6] = u8ptr[3];

			u8ptr = (uint8_t *)&Encoder_Front;
			package[7] = u8ptr[0]; // little-endian packing of integer, with least significant byte first
			package[8] = u8ptr[1];
			package[9] = u8ptr[2];
			package[10] = u8ptr[3];

			u8ptr = (uint8_t *)&Encoder_Rear;
			package[11] = u8ptr[0]; // little-endian packing of integer, with least significant byte first
			package[12] = u8ptr[1];
			package[13] = u8ptr[2];
			package[14] = u8ptr[3];

			packCRC = crcFast(package, COMM_PACKAGE_LENGTH-2); // calculate CRC based on package content excluding CRC bytes
			package[15] = packCRC >> 8;
			package[16] = packCRC & 0xFF;
			CDC_Transmit_FS_ThreadBlocking(package, COMM_PACKAGE_LENGTH);
		}

		if (xTaskGetTickCount() > nextRCTransmitTimestamp) {
			nextRCTransmitTimestamp = xTaskGetTickCount() + TransmitInterval_RC;

			if (RC_Receiver_Connected()) {
				RC_Receiver_GetValues(&RC_Throttle, &RC_Steering);

				// Prepare Encoder package
				package[0] = 0xFE;
				package[1] = 0xFE;
				package[2] = 0x12;

				u8ptr = (uint8_t *)&Timestamp;
				package[3] = u8ptr[0]; // little-endian packing of integer, with least significant byte first
				package[4] = u8ptr[1];
				package[5] = u8ptr[2];
				package[6] = u8ptr[3];

				u8ptr = (uint8_t *)&RC_Throttle;
				package[7] = u8ptr[0]; // little-endian packing of integer, with least significant byte first
				package[8] = u8ptr[1];

				u8ptr = (uint8_t *)&RC_Steering;
				package[9] = u8ptr[0]; // little-endian packing of integer, with least significant byte first
				package[10] = u8ptr[1];

				package[11] = 0;
				package[12] = 0;
				package[13] = 0;
				package[14] = 0;

				packCRC = crcFast(package, COMM_PACKAGE_LENGTH-2); // calculate CRC based on package content excluding CRC bytes
				package[15] = packCRC >> 8;
				package[16] = packCRC & 0xFF;
				CDC_Transmit_FS_ThreadBlocking(package, COMM_PACKAGE_LENGTH);
			}
		}

		if (xTaskGetTickCount() > (LiDAR_LastPackageTimestamp+LIDAR_TIMEOUT)) {
			LiDAR_Restart();
		}

		osDelay(1);
	}
}

void Communication_Receiver(void const * argument)
{
	Comm_Package package;
	uint8_t buffer[50];

	while (1) {
		//if( xSemaphoreTake( CommunicationReceiveSemaphore, portMAX_DELAY ) == pdPASS ) {
		if( xQueueReceive( CommunicationReceiveQueue, &package, ( TickType_t ) portMAX_DELAY ) == pdPASS ) {
			sprintf(buffer, "I received %d bytes: ", package.DataLength);
			CDC_Transmit_FS_ThreadBlocking(buffer, strlen(buffer));
			package.Data[package.DataLength] = '\n';
			CDC_Transmit_FS_ThreadBlocking(package.Data, package.DataLength+1);


			if (package.DataLength == COMM_PACKAGE_LENGTH) {
				if (package.Data[0] == COMM_HEADER_BYTE && package.Data[1] == COMM_HEADER_BYTE)
					Communication_ParsePackage(package.Data);
			}

			/*if (UserRxBufferFS[0] > 0) {
				sprintf(buffer, "Hello World\n", range);
				CDC_Transmit_FS(buffer, strlen(buffer));
				CDC_EmptyReceiveBuffer();
				Enter_DFU_Bootloader();
			}*/
		}
	}
}

void Communication_ParsePackage(uint8_t * package)
{
	uint16_t onTime;
	uint16_t Throttle, Steering;
	uint8_t Red, Green, Blue;

	uint16_t packCRC, calcCRC;
	packCRC = ((uint16_t)package[15] << 8) | package[16];
	calcCRC = crcFast(package, COMM_PACKAGE_LENGTH-2); // calculate CRC based on package content excluding CRC bytes
	if (packCRC != calcCRC) return; // incorrect checksum

	switch (package[2]) {
		case COMM_PACKID_ACTUATOR:
			Throttle = *((uint16_t*)&package[3]);
			Steering = *((uint16_t*)&package[5]);
			if (Throttle <= 1000 && Steering <= 1000)
				Motors_Set(Throttle, Steering);
			break;
		case COMM_PACKID_BUZZER_BEEP:
			onTime = *((uint16_t*)&package[3]);
			if (onTime > 1000) onTime = 1000; // Beeps can not be longer than 1 second!
			// Consider to make this beep event-based so the beep and the time it consumes is in a different thread
			if (onTime != 0)
				Buzzer_Beep(onTime);
			break;
		case COMM_PACKID_LED:
			Red = package[3];
			Green = package[4];
			Blue = package[5];
			RGB_SetColor(Red, Green, Blue);
			break;
		case COMM_PACKID_BOOTLOADER:
			Enter_DFU_Bootloader();
			break;
		default:
			return; // unknown package?!
			break;
	}
}
