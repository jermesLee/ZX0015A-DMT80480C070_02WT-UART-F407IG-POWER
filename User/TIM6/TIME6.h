#ifndef __TIME6_H_
#define	__TIME6_H_

#include "includes.h"

/****************************定时器定时中断优先级*************************/
#define TIM6_TimeTick            5                                       //TIM6定时时间 ,用于定时的基准

/****************************定时器定时中断优先级*************************/
#define TIM6_IRQPreemptionPrio   1                                        //TIM6的抢占式优先级      
#define TIM6_IRQSubPrio          3                                        //TIM6的次优先级


/*****************************定时器用户函数******************************/
extern void TIM6_Init(void);                                              //TIME6初始化 ,单位 ： ms ，最小1ms ，最大1200ms
extern FlagStatus TIM6_50MsFinish(void);                                  //定时50ms ， 定时时间到 返回SET ,否则返回RESET
extern FlagStatus TIM6_100MsFinish(void);                                 //定时100ms , 定时时间到 返回SET ,否则返回RESET


#endif 



