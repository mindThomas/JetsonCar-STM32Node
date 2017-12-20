#ifndef _MOTORS_h
#define _MOTORS_h

#define MOTORS_INPUT_RANGE	1000	// input into motor functions supports values in the range 0 - 1000

void Motors_Init(void);
void Motors_Set(uint16_t Throttle, uint16_t Steering);
void Motors_SetThrottle(uint16_t Throttle);
void Motors_SetFrontSteering(uint16_t Steering);
void Motors_SetRearSteering(uint16_t Steering);
void SetServoPWM(TIM_HandleTypeDef * timer, uint8_t channel, uint16_t PWM);

#endif	// _MOTORS_h
