#ifndef __LED_H_
#define	__LED_H_

#include "includes.h"

/* 定义控制IO的宏 */
#define LED1_TOGGLE()		GPIO_ToggleBits(GPIOH,GPIO_Pin_5)                 // 取反
#define LED1_OFF()			GPIO_SetBits(GPIOH,GPIO_Pin_5)                    // 置位
#define LED1_ON()		   	GPIO_ResetBits(GPIOH,GPIO_Pin_5)                  // 清零

#define LED2_TOGGLE()		GPIO_ToggleBits(GPIOA,GPIO_Pin_4)                 // 取反
#define LED2_OFF()			GPIO_SetBits(GPIOA,GPIO_Pin_4)                    // 置位
#define LED2_ON()		   	GPIO_ResetBits(GPIOA,GPIO_Pin_4)                  // 清零

#define LED3_TOGGLE()		GPIO_ToggleBits(GPIOA,GPIO_Pin_6)                 // 取反
#define LED3_OFF()			GPIO_SetBits(GPIOA,GPIO_Pin_6)                    // 置位
#define LED3_ON()		   	GPIO_ResetBits(GPIOA,GPIO_Pin_6)                  // 清零

extern void LED_Init(void);


#endif /* LED.H*/


