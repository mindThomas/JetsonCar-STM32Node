#include "RCReceiver.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim4;

uint8_t  RCIn_Throttle_State = 0;
uint32_t RCIn_Throttle_IC1 = 0;
uint32_t RCIn_Throttle_IC2 = 0;
uint32_t RCIn_Throttle_Micros = 0;
uint32_t RCIn_Throttle_LastUpdate = 0;

uint32_t RCIn_Throttle_CalMicrosMin = 858;
uint32_t RCIn_Throttle_CalMicrosMax = 2062;
float    RCIn_Throttle_CalScaling = 0.0f;

uint8_t  RCIn_Steering_State = 0;
uint32_t RCIn_Steering_IC1 = 0;
uint32_t RCIn_Steering_IC2 = 0;
uint32_t RCIn_Steering_Micros = 0;
uint32_t RCIn_Steering_LastUpdate = 0;

uint32_t RCIn_Steering_CalMicrosMin = 948;
uint32_t RCIn_Steering_CalMicrosMax = 2152;
float    RCIn_Steering_CalScaling = 0.0f;


void RC_Receiver_Init(void)
{
	RC_Receiver_CalculateCalValues();
	
	// Enable input capture for RC receiver
	HAL_NVIC_SetPriority(TIM4_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);	
}

void TIM4_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim4);
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  GPIO_PinState pinState;
  uint32_t diff;

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
	pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);

    if (pinState == GPIO_PIN_SET && RCIn_Throttle_State == 0) // registered a transition from low-to-high
    {
      /* Get the 1st Input Capture value */
      RCIn_Throttle_IC1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
      RCIn_Throttle_State = 1; // now we are waiting for a low-to-high transition
    }
    else if (pinState == GPIO_PIN_RESET && RCIn_Throttle_State == 1) // registered a transition from high-to-low
    {
      /* Get the 2nd Input Capture value */
      RCIn_Throttle_IC2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

      /* Capture computation */
      if (RCIn_Throttle_IC2 > RCIn_Throttle_IC1)
      {
        diff = (RCIn_Throttle_IC2 - RCIn_Throttle_IC1);
      }
      else  /* (RCIn_Throttle_IC2 <= RCIn_Throttle_IC1) */
      {
        diff = ((0xFFFF - RCIn_Throttle_IC1) + RCIn_Throttle_IC2);
      }

      RCIn_Throttle_Micros = 2*diff; // due to timer clock being 2*APB1 = 84 MHz and a prescaler of 168
	  RCIn_Throttle_LastUpdate = xTaskGetTickCountFromISR();
	  
      /* Frequency computation: for this example TIMx (TIM1) is clocked by
         2xAPB1Clk */
      //uwFrequency = (2*HAL_RCC_GetPCLK1Freq()/168) / uwDiffCapture;
      RCIn_Throttle_State = 0; // now we are waiting for a high-to-low transition
    }
    else { // an error occured (out of sync)
      RCIn_Throttle_State = 0;
    }
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

    if (pinState == GPIO_PIN_SET && RCIn_Steering_State == 0) // registered a transition from low-to-high
    {
      /* Get the 1st Input Capture value */
      RCIn_Steering_IC1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
      RCIn_Steering_State = 1; // now we are waiting for a low-to-high transition
    }
    else if (pinState == GPIO_PIN_RESET && RCIn_Steering_State == 1) // registered a transition from high-to-low
    {
      /* Get the 2nd Input Capture value */
      RCIn_Steering_IC2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

      /* Capture computation */
      if (RCIn_Steering_IC2 > RCIn_Steering_IC1)
      {
	    diff = (RCIn_Steering_IC2 - RCIn_Steering_IC1);
      }
      else  /* (RCIn_Steering_IC2 <= RCIn_Steering_IC1) */
      {
	    diff = ((0xFFFF - RCIn_Steering_IC1) + RCIn_Steering_IC2);
      }

      RCIn_Steering_Micros = 2*diff; // due to timer clock being 2*APB1 = 84 MHz and a prescaler of 168
	  RCIn_Steering_LastUpdate = xTaskGetTickCountFromISR();
	  
      /* Frequency computation: for this example TIMx (TIM1) is clocked by
	     2xAPB1Clk */
      //uwFrequency = (2*HAL_RCC_GetPCLK1Freq()/168) / uwDiffCapture;
      RCIn_Steering_State = 0; // now we are waiting for a high-to-low transition
    }
    else { // an error occured (out of sync)
      RCIn_Steering_State = 0;
    }
  }
}

void RC_Receiver_Calibrate(void)
{
	uint32_t tEnd = xTaskGetTickCount() + 10000; // calibration for 10 seconds

	RCIn_Throttle_CalMicrosMin = 9999;
	RCIn_Throttle_CalMicrosMax = 0;
	RCIn_Steering_CalMicrosMin = 9999;
	RCIn_Steering_CalMicrosMax = 0;

	while (xTaskGetTickCount() < tEnd) {
		if (RCIn_Throttle_Micros > 0) {
			if (RCIn_Throttle_Micros > RCIn_Throttle_CalMicrosMax) RCIn_Throttle_CalMicrosMax = RCIn_Throttle_Micros;
			if (RCIn_Throttle_Micros < RCIn_Throttle_CalMicrosMin) RCIn_Throttle_CalMicrosMin = RCIn_Throttle_Micros;
		}
		if (RCIn_Steering_Micros > 0) {
			if (RCIn_Steering_Micros > RCIn_Steering_CalMicrosMax) RCIn_Steering_CalMicrosMax = RCIn_Steering_Micros;
			if (RCIn_Steering_Micros < RCIn_Steering_CalMicrosMin) RCIn_Steering_CalMicrosMin = RCIn_Steering_Micros;
		}
		osDelay(10);
	}

	RC_Receiver_CalculateCalValues();
}

void RC_Receiver_CalculateCalValues(void)
{
	if (RCIn_Throttle_CalMicrosMax != RCIn_Throttle_CalMicrosMin) {
		RCIn_Throttle_CalScaling = (float)RC_RECEIVER_OUTPUT_RANGE / (float)(RCIn_Throttle_CalMicrosMax - RCIn_Throttle_CalMicrosMin);
	} else {
		RCIn_Throttle_CalScaling = 0;
	}

	if (RCIn_Steering_CalMicrosMax != RCIn_Steering_CalMicrosMin) {
		RCIn_Steering_CalScaling = (float)RC_RECEIVER_OUTPUT_RANGE / (float)(RCIn_Steering_CalMicrosMax - RCIn_Steering_CalMicrosMin);
	} else {
		RCIn_Steering_CalScaling = 0;
	}
}

void RC_Receiver_RunningCalibration(void)
{
	// Consider to implement as filter
	if (RCIn_Throttle_Micros > 0) {
		if (RCIn_Throttle_Micros > RCIn_Throttle_CalMicrosMax) RCIn_Throttle_CalMicrosMax = RCIn_Throttle_Micros;
		if (RCIn_Throttle_Micros < RCIn_Throttle_CalMicrosMin) RCIn_Throttle_CalMicrosMin = RCIn_Throttle_Micros;
	}
	if (RCIn_Steering_Micros > 0) {
		if (RCIn_Steering_Micros > RCIn_Steering_CalMicrosMax) RCIn_Steering_CalMicrosMax = RCIn_Steering_Micros;
		if (RCIn_Steering_Micros < RCIn_Steering_CalMicrosMin) RCIn_Steering_CalMicrosMin = RCIn_Steering_Micros;
	}

	RC_Receiver_CalculateCalValues();
}

void RC_Receiver_GetValues(uint16_t * Throttle, uint16_t * Steering)
{
	//RC_Receiver_RunningCalibration();

	if (RCIn_Throttle_Micros == 0 || (xTaskGetTickCount() - RCIn_Throttle_LastUpdate) > RC_RECEIVER_TIMEOUT) {
		*Throttle = 0;
	} else {
		*Throttle = (RCIn_Throttle_Micros - RCIn_Throttle_CalMicrosMin) * RCIn_Throttle_CalScaling;
	}
	
	if (RCIn_Steering_Micros == 0 || (xTaskGetTickCount() - RCIn_Steering_LastUpdate) > RC_RECEIVER_TIMEOUT) {
		*Steering = 0;
	} else {
		*Steering = (RCIn_Steering_Micros - RCIn_Steering_CalMicrosMin) * RCIn_Steering_CalScaling;
	}
}

uint8_t RC_Receiver_Connected(void)
{
	uint16_t Throttle, Steering;
	RC_Receiver_GetValues(&Throttle, &Steering);

	if (Steering == 0 && Throttle > 495 && Throttle < 505) { // RC receiver off
		return 0;
	} else {
		return 1;
	}
}

