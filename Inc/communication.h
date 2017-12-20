#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define COMM_PACKAGE_LENGTH				17
#define COMM_HEADER_BYTE				0xFE

#define COMM_PACKID_ACTUATOR			0x01
#define COMM_PACKID_BUZZER_BEEP			0x0A
#define COMM_PACKID_LED					0x0B
#define COMM_PACKID_BOOTLOADER			0x0F

#define LIDAR_QUEUE_LENGTH				100
#define COMM_RX_QUEUE_LENGTH			50
#define COMM_ALLOCATED_BUFFER_SPACE		30

extern SemaphoreHandle_t CommunicationReceiveSemaphore;
extern QueueHandle_t CommunicationReceiveQueue;
extern QueueHandle_t LiDAR_Message_Queue;

void Communication_Init(void);
void Communication_Transmitter(void const * argument);
void Communication_Receiver(void const * argument);

typedef struct
{
	uint32_t Timestamp;
    uint16_t ID;
    uint16_t Distance;
} LiDAR_Message;

typedef struct
{
	uint8_t DataLength;
	uint8_t Data[COMM_ALLOCATED_BUFFER_SPACE];
} Comm_Package;


#endif /* __COMMUNICATION_H */
