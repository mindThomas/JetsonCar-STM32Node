#include "stm32f4xx_hal.h"
#include "Motors.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim9;

uint32_t Throttle_Min = 429;
uint32_t Throttle_Max = 1031;
uint32_t Steering_Min = 560;
uint32_t Steering_Max = 960;
float Throttle_Scaling;
float Steering_Scaling;

void Motors_Init(void)
{
	// Enable PWM output for Throttle and Steering servo
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

	Throttle_Scaling = (float)(Throttle_Max - Throttle_Min) / (float)MOTORS_INPUT_RANGE;
	Steering_Scaling = (float)(Steering_Max - Steering_Min) / (float)MOTORS_INPUT_RANGE;

	Motors_Set(500,500); // initially turn off throttle and center steering motor
}

void Motors_Set(uint16_t Throttle, uint16_t Steering)
{
	Motors_SetThrottle(Throttle);
	Motors_SetFrontSteering(Steering);
}

void Motors_SetThrottle(uint16_t Throttle)
{
	uint16_t Period;
	Period = (Throttle * Throttle_Scaling) + Throttle_Min; // Map from input (0-1000) to output defined by Min and Max values
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Period); // TIM1, ch1 = throttle output
}

void Motors_SetFrontSteering(uint16_t Steering)
{
	uint16_t Period;
	Period = (Steering * Steering_Scaling) + Steering_Min; // Map from input (0-1000) to output defined by Min and Max values
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, Period); // TIM9, ch1 = front steering output
}

void Motors_SetRearSteering(uint16_t Steering)
{
	uint16_t Period;
	Period = (Steering * Steering_Scaling) + Steering_Min; // Map from input (0-1000) to output defined by Min and Max values
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, Period); // TIM9, ch2 = rear steering output
}

// Consider to calibrate ESC!?
void SetServoPWM(TIM_HandleTypeDef * timer, uint8_t channel, uint16_t PWM)
{
	uint16_t Period;

	if (PWM > 1000) PWM = 1000;
	Period = 250 + PWM; // PWM should be between 0 to 1000
	                     // This will result in an output from 0.5 ms to 2.5 ms

	if (channel == 1) {
		__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, Period);
	}
	else if (channel == 2) {
		__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2, Period);
	}
}
