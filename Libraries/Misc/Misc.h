#ifndef _MISC_h
#define _MISC_h

#define BOOTLOADER_MAGIC_ADDR ((uint32_t*) ((uint32_t) 0x20001000)) //4k into SRAM (out of 6k)
#define BOOTLOADER_MAGIC_TOKEN 0xDEADBEEF  // :D

//Value taken from CD00167594.pdf page 35, system memory start.
#define BOOTLOADER_START_ADDR 0x1fffc400 //for ST32F042

#define A_VALUE 0x12345678

void Misc_Init(void);
void RGB_SetColor(uint8_t Red, uint8_t Green, uint8_t Blue);
void SetLEDPWM(TIM_HandleTypeDef * timer, uint8_t channel, uint16_t PWM);

void Buzzer_Beep(void);
void Buzzer_On(void);
void Buzzer_Off(void);

#endif	// _MISC_h
