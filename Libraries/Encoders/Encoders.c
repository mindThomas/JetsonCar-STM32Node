#include "stm32f4xx_hal.h"
#include "Encoders.h"

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim2;

void Encoders_Init(void)
{
	// Enable Encoder interface
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // Front encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Back encoder
	//HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Future use encoder
}

void Encoders_GetCount(int32_t * front, int32_t * rear)
{
	*front = __HAL_TIM_GET_COUNTER(&htim5);
	*rear = -__HAL_TIM_GET_COUNTER(&htim2);
}

void Encoders_GetAngle(float * angleFront, float * angleRear)
{
	int32_t front, rear;

	Encoders_GetCount(&front, &rear);

	*angleFront = (((float)front / (COUNTS_PR_REVOLUTION*GEARING_RATIO))*360);
	*angleRear = (((float)rear / (COUNTS_PR_REVOLUTION*GEARING_RATIO))*360);
}
