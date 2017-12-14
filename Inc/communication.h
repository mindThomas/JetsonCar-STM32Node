#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

SemaphoreHandle_t CommunicationReceiveSemaphore;
QueueHandle_t LiDAR_Message_Queue;

void Communication_Init(void);
void Communication_Transmitter(void const * argument);
void Communication_Receiver(void const * argument);

typedef struct
{
    uint16_t ID;
    uint16_t Distance;
} LiDAR_Message;

#endif /* __COMMUNICATION_H */
