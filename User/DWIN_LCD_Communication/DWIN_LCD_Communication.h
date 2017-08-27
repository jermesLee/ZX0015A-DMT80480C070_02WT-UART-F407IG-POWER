#ifndef _DWIN_LCD_COMMUNICATION_H_
#define	_DWIN_LCD_COMMUNICATION_H_

#include "includes.h"
#include "DWIN_LCD.h"
#include "TIME3_Encoder.h" 
#include "TIME5_Encoder.h" 


/************************DWIN_LCD显示地址宏定义**************************/ 
#define DWIN_LCD_InputLineVoltageAddress         (uint16_t)0X000E         //触摸输入电压地址
#define DWIN_LCD_DisplaySetLineVoltageAddress    (uint16_t)0X000A         //实际显示设置线电压的地址
 
#define DWIN_LCD_InputFrequencyAddress           (uint16_t)0X000C         //触摸输入频率的地址
#define DWIN_LCD_DisplaySetFrequencyAddress      (uint16_t)0X0008         //实际显示设置频率的地址


/*********************设置线电压 频率 最值宏定义*************************/ 
#define DWIN_LCD_DisplaySetLineVoltageMin        (uint16_t)1             //实际显示设置的线电压最小值
#define DWIN_LCD_DisplaySetLineVoltageMax        (uint16_t)2000          //实际显示设置的线电压最大值 ，单位 ：0.01 LSB (V)

#define DWIN_LCD_DisplaySetFrequencyMin          (uint16_t)1             //实际显示设置的信号频率的最小值
#define DWIN_LCD_DisplaySetFrequencyMax          (uint16_t)2000          //实际显示设置的信号频率的最大值 ，单位 ：1 LSB(Hz)


/************************** DWIN_LCD功能函数声明*************************/ 
extern void DWIN_LCD_DisplayInit(void);                                   //上电初始化DWIN_LCD显示数据
extern void DWIN_LCD_AndMCUCommunication(void);                           //需要从缓冲区写入	DWIN_LCD的数据 ， 和 将接收到的迪文屏数据处理写入DWIN_LCD缓冲区 
extern uint16_t DWIN_LCD_DisplaySetLineVoltage(void);                     //计算DWIN_LCD屏显示设置线电压 ，和将设置的线电压写入DWIN_LCD缓冲区
extern uint16_t DWIN_LCD_DisplaySetFrequency(void);                       //计算DWIN_LCD屏显示设置信号频率，和将设置的频率写入DWIN_LCD缓冲区


#endif /* __DWIN_LCD_H */


