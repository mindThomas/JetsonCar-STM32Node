#include "stm32f4xx_hal.h"
#include "Misc.h"

extern TIM_HandleTypeDef htim8;

void Misc_Init(void)
{
	// Enable PWM output for RGB LED
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}

void Enter_DFU_Bootloader(){
//call this at any time to initiate a reboot into bootloader
	//HAL_PWR_EnableBkUpAccess();
    *BOOTLOADER_MAGIC_ADDR = BOOTLOADER_MAGIC_TOKEN;
	//(*(__IO uint32_t *) (BKPSRAM_BASE + 0)) = A_VALUE;
    NVIC_SystemReset();
}

void OnBoot_Check_DFU_Reqeust(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();

	void (*SysMemBootJump)(void);
	volatile uint32_t addr = 0x1FFF0000; // For STM32F4 Discovery

	//if((*(__IO uint32_t *) (BKPSRAM_BASE + 0)) == A_VALUE)
	if (*BOOTLOADER_MAGIC_ADDR == BOOTLOADER_MAGIC_TOKEN)
	{
	  //(*(__IO uint32_t *) (BKPSRAM_BASE + 0)) = 0; // Reset memory, if desired.

	  SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4))); // Set Bootloader address

	  __set_MSP(*(uint32_t *)addr); // Move Stack Pointer

	  SysMemBootJump(); // Execute Bootloader
	  while (1);
	}
}

void RGB_SetColor(uint8_t Red, uint8_t Green, uint8_t Blue)
{
	SetLEDPWM(&htim8, 1, 256-Red); 		// Red
	SetLEDPWM(&htim8, 2, 256-Green); 	// Green
	SetLEDPWM(&htim8, 3, 256-Blue); 	// Blue
}

void SetLEDPWM(TIM_HandleTypeDef * timer, uint8_t channel, uint16_t PWM)
{
	if (PWM > 256) PWM = 256; // PWM should be between 1 to 256 where 256 gives turned off LED and 1 gives full brightness

	if (channel == 1) {
		__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, PWM);
	}
	else if (channel == 2) {
		__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2, PWM);
	}
	else if (channel == 3) {
		__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, PWM);
	}
}

void Buzzer_Beep(uint16_t onTime)
{
	Buzzer_On();
	osDelay(onTime);
	Buzzer_Off();
}

void Buzzer_On(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void Buzzer_Off(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
