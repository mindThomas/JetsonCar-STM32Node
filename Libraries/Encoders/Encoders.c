#include "stm32f4xx_hal.h"
#include "Encoders.h"

int32_t Encoder_Left = 0;
int32_t Encoder_Right = 0;

// http://en.wikipedia.org/wiki/Rotary_encoder#Incremental_rotary_encoder
const int8_t EncoderStates[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 }; // Encoder lookup table if it interrupts on every edge
//const int8_t EncoderStates[16] = { 0, 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0 };
//const int8_t EncoderStates[16] = { 0, -1, 1, 0, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, 0};
static uint8_t oldLeftAB = 0;
static uint8_t oldRightAB = 0;

void Encoders_Init(void)
{
	Encoders_GPIO_Init();

	Encoder_Left = 0;
	Encoder_Right = 0;
}

void Encoders_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	ENCODERS_GPIO_ENABLE();

	GPIO_InitStruct.Pin = ENCODERS_LEFT_A_PIN  | ENCODERS_RIGHT_A_PIN | ENCODERS_LEFT_B_PIN | ENCODERS_RIGHT_B_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ENCODERS_GPIO_PORT, &GPIO_InitStruct);

	// Enable encoder interrupt
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}


/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  /* EXTI line interrupt detected */
  if (__HAL_GPIO_EXTI_GET_IT(ENCODERS_LEFT_A_PIN) != RESET) {
	  __HAL_GPIO_EXTI_CLEAR_IT(ENCODERS_LEFT_A_PIN);
      Encoders_LeftHandler();
  }
  if (__HAL_GPIO_EXTI_GET_IT(ENCODERS_LEFT_B_PIN) != RESET) {
	  __HAL_GPIO_EXTI_CLEAR_IT(ENCODERS_LEFT_B_PIN);
	  Encoders_LeftHandler();
  }
  if (__HAL_GPIO_EXTI_GET_IT(ENCODERS_RIGHT_A_PIN) != RESET) {
	  __HAL_GPIO_EXTI_CLEAR_IT(ENCODERS_RIGHT_A_PIN);
	  Encoders_RightHandler();
  }
  if (__HAL_GPIO_EXTI_GET_IT(ENCODERS_RIGHT_B_PIN) != RESET) {
	  __HAL_GPIO_EXTI_CLEAR_IT(ENCODERS_RIGHT_B_PIN);
	  Encoders_RightHandler();
  }
}

inline void Encoders_LeftHandler(void)
{
  oldLeftAB <<= 2; // Remember previous state
  oldLeftAB |= (ENCODERS_GPIO_PORT->IDR & (ENCODERS_LEFT_A_PIN | ENCODERS_LEFT_B_PIN)) >> ENCODERS_LEFT_SHIFT;
  Encoder_Left -= EncoderStates[ oldLeftAB & 0x0F ];
}

inline void Encoders_RightHandler(void)
{
  oldRightAB <<= 2; // Remember previous state
  oldRightAB |= (ENCODERS_GPIO_PORT->IDR & (ENCODERS_RIGHT_A_PIN | ENCODERS_RIGHT_B_PIN)) >> ENCODERS_RIGHT_SHIFT;
  Encoder_Right -= EncoderStates[ oldRightAB & 0x0F ];
}
