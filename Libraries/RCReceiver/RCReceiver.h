#ifndef _RCRECEIVER_h
#define _RCRECEIVER_h

#include "stm32f4xx_hal.h"

#define RC_RECEIVER_TIMEOUT			500		// if nothing is received for 500 ms then return 0
#define RC_RECEIVER_OUTPUT_RANGE	1000	// output of RC receiver library should map from 0 - 1000

void RC_Receiver_Init(void);
void RC_Receiver_CalculateCalValues(void);
void RC_Receiver_Calibrate(void);
void RC_Receiver_GetValues(uint16_t * Throttle, uint16_t * Steering);
uint8_t RC_Receiver_Connected(void);

void TIM4_IRQHandler(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif	// _RCRECEIVER_h
