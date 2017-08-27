#ifndef __TIM3_ENCODER_H
#define	__TIM3_ENCODER_H

#include "includes.h"


/****************************定时器定时中断优先级*************************/
#define TIM3_IRQPreemptionPrio   1                                         //TIM4的抢占式优先级      
#define TIM3_IRQSubPrio          3                                         //TIM4的次优先级

/*************************TIM3 编码器用户函数**************************/
extern void TIM3_Encoder_Init(void);                                       //TIME4配置为编码器模式                                      
extern void TIM3_Encoder_SetData(int32_t EncoderValue);                    //设置编码器数据 （-2^31-1  ~  2^31 - 1）
extern int32_t TIM3_Encoder_GetData(void);                                 //读取编码器数据 （-2^31-1  ~  2^31 - 1）



#endif /* __PWM_OUTPUT_H */

