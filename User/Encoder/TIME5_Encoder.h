#ifndef __TIM5_ENCODER_H
#define	__TIM5_ENCODER_H

#include "includes.h"


/****************************定时器定时中断优先级*************************/
#define TIM5_IRQPreemptionPrio   2                                        //TIM5的抢占式优先级      
#define TIM5_IRQSubPrio          2                                         //TIM5的次优先级

/*************************TIME5 编码器用户函数**************************/
extern void TIM5_Encoder_Init(void);                                       //TIME5配置为编码器模式                                      
extern void TIM5_Encoder_SetData(int32_t EncoderValue);                    //设置编码器数据 （-2^31-1  ~  2^31 - 1）
extern int32_t TIM5_Encoder_GetData(void);                                 //读取编码器数据 （-2^31-1  ~  2^31 - 1）



#endif /* __PWM_OUTPUT_H */

