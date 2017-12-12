#ifndef _ENCODERS_h
#define _ENCODERS_h

#define ENCODERS_GPIO_ENABLE	__GPIOB_CLK_ENABLE
#define ENCODERS_GPIO_PORT		GPIOB
#define ENCODERS_LEFT_A_PIN		GPIO_PIN_4
#define ENCODERS_LEFT_B_PIN		GPIO_PIN_5
#define ENCODERS_LEFT_SHIFT		4
#define ENCODERS_RIGHT_A_PIN	GPIO_PIN_14
#define ENCODERS_RIGHT_B_PIN	GPIO_PIN_15
#define ENCODERS_RIGHT_SHIFT	14

extern int32_t Encoder_Left;
extern int32_t Encoder_Right;


void Encoders_Init(void);
void Encoders_GPIO_Init(void);

void Encoders_LeftHandler(void);
void Encoders_RightHandler(void);


#endif	// _ENCODERS_h
