/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "communication.h"
#include "crc.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "VL53L0X.h"
#include "XV11_LiDAR.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim12;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osThreadId I2C_TestTaskHandle;
osThreadId ServoTestTaskHandle;
xQueueHandle xExternal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void I2C_Test(void const * argument);
void ServoTest(void const * argument);
void SetPWM(TIM_HandleTypeDef * timer, uint8_t channel, uint16_t PWM);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t checksum;
/* USER CODE END 0 */

int main(void)
 {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  crcInit();
  checksum = crcFast("123456789", 9);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM10_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  LiDAR_Init();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Start the encoder interface
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

  xExternal = xQueueCreate( 100, sizeof(uint16_t) );
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(I2C_TestTask, I2C_Test, osPriorityNormal, 0, 256);
  I2C_TestTaskHandle = osThreadCreate(osThread(I2C_TestTask), NULL);

  osThreadDef(ServoTestTask, ServoTest, osPriorityNormal, 0, 128);
  ServoTestTaskHandle = osThreadCreate(osThread(ServoTestTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 335;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 9999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim9);

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 335;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim10);

}

/* TIM12 init function */
static void MX_TIM12_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 7;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim12);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
#define LONG_RANGE
#define HIGH_ACCURACY
#include <string.h>
uint16_t range;
void I2C_Test(void const * argument)
{
	uint8_t buffer[50];
	while (1) {
		osDelay(100);
	}

	VL53L0X_init(true);
	VL53L0X_setTimeout(500);
	/*VL53L0X_startContinuous(0);

	while (1) {
		range = VL53L0X_readRangeContinuousMillimeters();
		//osDelay(100);
	}*/

#ifdef LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
	VL53L0X_setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
	VL53L0X_setVcselPulsePeriod(VL53L0X_VcselPeriodPreRange, 18);
	VL53L0X_setVcselPulsePeriod(VL53L0X_VcselPeriodFinalRange, 14);
#endif

#ifdef HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
	VL53L0X_setMeasurementTimingBudget(20000);
#elif defined(HIGH_ACCURACY)
  // increase timing budget to 200 ms
	VL53L0X_setMeasurementTimingBudget(200000);
#endif

	while (1) {
		range = VL53L0X_readRangeSingleMillimeters();
		sprintf(buffer, "Distance: %d\n", range);
		//CDC_Transmit_FS(buffer, strlen(buffer));
		osDelay(100);
	}
}

void ServoTest(void const * argument)
{
	uint16_t PWM;
	int32_t encoder;
	float angle;

	while (1) {
		encoder = __HAL_TIM_GET_COUNTER(&htim2);
		angle = (((float)encoder / 600)*360);
		if (angle > 90) angle = 90;
		if (angle < -90) angle = -90;
		PWM = (uint16_t)((float)((angle + 90.0) * 1000.0) / 180.0);
		SetPWM(&htim9, 1, PWM);
		osDelay(20);
	}

	/*while (1) {
		for (PWM = 0; PWM < 1000; PWM += 10) {
			SetPWM(&htim10, 1, PWM);
			osDelay(50);
		}
		for (PWM = 1000; PWM > 10; PWM -= 10) {
			SetPWM(&htim10, 1, PWM);
			osDelay(50);
		}
	}*/
}

/*
void Enter_DFU_Bootloader(void)
{
	// Activate the bootloader without BOOT* pins.
	    //pyb_usb_dev_deinit();
	    //storage_flush();

	    //HAL_RCC_DeInit();
	    //HAL_DeInit();
		__disable_irq();

	#if defined(MCU_SERIES_F7)
	    // arm-none-eabi-gcc 4.9.0 does not correctly inline this
	    // MSP function, so we write it out explicitly here.
	    //__set_MSP(*((uint32_t*) 0x1FF00000));
	    __ASM volatile ("movw r3, #0x0000\nmovt r3, #0x1FF0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");

	    ((void (*)(void)) *((uint32_t*) 0x1FF00004))();
	#else
	    __HAL_REMAPMEMORY_SYSTEMFLASH();

	    // arm-none-eabi-gcc 4.9.0 does not correctly inline this
	    // MSP function, so we write it out explicitly here.
	    //__set_MSP(*((uint32_t*) 0x00000000));
	    __ASM volatile ("movs r3, #0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");

	    ((void (*)(void)) *((uint32_t*) 0x00000004))();
	#endif

	    while (1);
}*/

#define BOOTLOADER_MAGIC_ADDR ((uint32_t*) ((uint32_t) 0x20001000)) //4k into SRAM (out of 6k)
#define BOOTLOADER_MAGIC_TOKEN 0xDEADBEEF  // :D

//Value taken from CD00167594.pdf page 35, system memory start.
#define BOOTLOADER_START_ADDR 0x1fffc400 //for ST32F042

#define A_VALUE 0x12345678

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

/*
//This is the first thing the micro runs after startup_stm32f0xx.s
//Only the SRAM is initialized at this point
void Enter_DFU_Bootloader(){
  uint32_t jumpaddr,tmp;

  tmp=*BOOTLOADER_MAGIC_ADDR;
  //tmp=BOOTLOADER_MAGIC_TOKEN; //DEBUG
  if (tmp == BOOTLOADER_MAGIC_TOKEN){
    *BOOTLOADER_MAGIC_ADDR=0; //Zero it so we don't loop by accident

    // For the STM32F405, the BOOT0 pin is at PF11
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
    GPIO_InitTypeDef gp;
    gp.GPIO_Pin = GPIO_Pin_11; //BOOT0 pin
    gp.GPIO_Mode = GPIO_Mode_OUT;
    gp.GPIO_Speed = GPIO_Speed_2MHz;
    gp.GPIO_OType = GPIO_OType_PP;
    gp.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &gp);
    GPIO_WriteBit(GPIOF, GPIO_Pin_11, 1);

    void (*bootloader)(void) = 0; //Zero the function pointer.
    jumpaddr = *(__IO uint32_t*)(BOOTLOADER_START_ADDR + 4);
    bootloader = (void (*)(void)) jumpaddr; //Set the function pointer to bootaddr +4
    __set_MSP(*(__IO uint32_t*) BOOTLOADER_START_ADDR); //load the stackpointer - bye bye program
    bootloader(); //GO TO DFU MODE **** :D

    //this should never be hit, trap for debugging
    while(1){}
  }
}*/

void SetPWM(TIM_HandleTypeDef * timer, uint8_t channel, uint16_t PWM)
{
	uint16_t Period;

	if (PWM > 1000) PWM = 1000;
	Period = 250 + PWM; // PWM should be between 0 to 999
	                     // This will result in an output from 0.5 ms to 2.5 ms

	if (channel == 1) {
		__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, Period);
	}
	else if (channel == 2) {
		__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2, Period);
	}
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  Communication_Init();

  /* USER CODE BEGIN 5 */
  int counter = 0;
  uint8_t buffer[20];
  /* Infinite loop */
  for(;;)
  {
	/*if (UserRxBufferFS[0] > 0) {
		sprintf(buffer, "Hello World\n", range);
		CDC_Transmit_FS(buffer, strlen(buffer));
		CDC_EmptyReceiveBuffer();
		Enter_DFU_Bootloader();
	}*/

	/*if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
		sprintf(buffer, "Motor BCK: %d\n", __HAL_TIM_GET_COUNTER(&htim2));
	} else {
		sprintf(buffer, "Motor FWD: %d\n", __HAL_TIM_GET_COUNTER(&htim2));
	}
	CDC_Transmit_FS(buffer, strlen(buffer));*/

	sprintf(buffer, "%d\n", counter);
	LiDAR_Transmit(buffer, strlen(buffer));
	counter++;

    osDelay(200);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
