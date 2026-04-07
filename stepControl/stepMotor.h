#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H
#define minStep (360.0f/4096.0f)

extern float accumulatedError; //误差累积
void stepMotor_Init(void);//初始化
int allModeRotate(float speed,float angle,uint8_t Mode);//8拍
int halfModeRotate(float speed,float angle,uint8_t Mode);//4拍

void GenericEightstep(uint32_t delyTime,uint8_t Mode,uint8_t StepNum);//通用低精度
void stepOnce(uint8_t stepCurrent, uint32_t delyTime);//单步驱动

#endif