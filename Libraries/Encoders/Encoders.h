#ifndef _ENCODERS_h
#define _ENCODERS_h

#define COUNTS_PR_REVOLUTION	12
#define GEARING_RATIO			40

void Encoders_Init(void);
void Encoders_GetCount(int32_t * front, int32_t * rear);
void Encoders_GetAngle(float * angleFront, float * angleRear);

#endif	// _ENCODERS_h
