#ifndef _FSMC_H_
#define _FSMC_H_

#include "includes.h"

/******************************数据访问宽度宏定义************************/       //这个宏定义决定了不同访问宽度下的地址总线的映射关系 
#define FSMC_BANK1_DataWidth                         FSMC_BANK1_DataWidth16Bit   //数据访问宽度宏定义  ，FSMC_BANK1_DataWidth16Bit  表示16位的访问 ，
                                                                                 //FSMC_BANK1_DataWidth8Bit  表示8位的访问 ，


/*********************数据访问宽度条件选择（不允许修改）*****************/
#define FSMC_BANK1_DataWidth16Bit                     1                   // 1 表示FSMC传送数据为16位 , 作用 ：用于决定地址线的映射关系
#define FSMC_BANK1_DataWidth8Bit                      0                   // 0 表示FSMC传送数据为8位 ， 作用 ：用于决定地址线的映射关系

#ifdef  FSMC_BANK1_DataWidth                                              // BANK1 地址生成器  ,访问方式 ：16位     
#define FSMC_BANK1_AddressGenerate(Addr)             ((uint32_t)( ( ( Addr ) <<1 ) & 0X03FFFFFF | 0X60000000 ))                             
#else                                                                     // BANK1 地址生成器  ，访问方式：8位     
#define FSMC_BANK1_AddressGenerate(Addr)             ((uint32_t)( ( Addr ) & 0X03FFFFFF | 0X60000000 ))                                   
#endif


/********************************数据访问读写*****************************/
#define FSMC_BANK1_FPGAWriteHalfWord(Addr , Value)   (*(volatile uint16_t *)FSMC_BANK1_AddressGenerate(Addr))=((uint16_t)Value)          // 往BANK1里写数据
#define FSMC_BANK1_FPGAReadHalfWord(Addr)            (*(volatile uint16_t *)FSMC_BANK1_AddressGenerate(Addr))                            //读取BANK1区域数据 

extern void FSMC_Init(void);                                              //FSMC 总线初始化总函数

#endif


