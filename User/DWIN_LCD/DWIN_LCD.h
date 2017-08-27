#ifndef _DWIN_LCD_H_
#define	_DWIN_LCD_H_

#include "includes.h"                                                                              // 所有的头文件
#include "USART1.h"                                                                                // 串口 ，用于与DWIN_LCD迪文屏的通信
#include "CRC16.h"                                                                                 // CRC16校验 ，确保通信数据正常


/******************************DWIN_LCD迪文屏 指令集枚举定义*************************************/
typedef enum DWIN_LCD_CommandType 
{
	DWIN_LCD_WriteControlRegiter = 0X80 ,                                                            //写控制寄存器指令
	DWIN_LCD_ReadControlRegiter  = 0X81 ,                                                            //读控制寄存器指令
	DWIN_LCD_WriteDataMemory     = 0X82 ,                                                            //写数据内存指令
	DWIN_LCD_ReadDataMemory      = 0X83                                                              //读数据内存指令
}DWIN_LCD_CommandType ;                                                                            //DWIN_LCD迪文屏 指令集类型


/**************************DWIN_LCD迪文屏通信内存分配大小宏定义 *********************************/
#define DWIN_LCD_FrameSizeMax                          74                                         // 与DWIN_LCD 通信允许的最大的一帧数据的大小 ，单位：字节 ，可以使用默认值
                                                                                                   // 当没有使用这个宏定义时 ，就使用默认值 ，对于默认值为  DWIN_LCD_NumSize * 2 + 10 
#define DWIN_LCD_FrameHeadH                          0X5A                                          // 通信帧头高字节             
#define DWIN_LCD_FrameHeadL                          0XA5                                          // 通信帧头低字节
#define DWIN_LCD_StartAddr                         0X0000                                          // DWIN_LCD迪文屏允许使用的地址首地址
#define DWIN_LCD_NumSize                               32                                          // DWIN_LCD迪文屏允许使用的地址数量


/**************************DWIN_LCD迪文屏通信用到的外部函数***************************************/
#define DWIN_LCD_Init(Baudrate)                          USART1_Init(Baudrate)                     // DWIN_LCD迪文屏初始化 ，实际就是串口初始化
#define DWIN_LCD_WriteSend(Data   ,Num)                  USART1_WriteDMASendMulData(Data   ,Num)   // 发送数据于迪文屏  ，data为需要发送的数据，num为需要发送的字节数
#define DWIN_LCD_ReadReceive(Data ,Num)                  USART1_ReadDMAReceiveMulData(Data ,Num)   // 读取接收到的迪文屏数据与data中 ，接收到的字节数保存在Num
#define DWIN_LCD_CRC16_CheckCompute(DatHead,DatLen,ReturnDat)  CRC16_CheckCompute(DatHead,DatLen,ReturnDat) // crc16计算,计算结果保存在ReturnDat里
#define DWIN_LCD_CRC16_Check(DataHeader1,DataHeader2)    CRC16_Check(DataHeader1,DataHeader2)      // 比较两个crc值是否相等 ，相等则返回SUCCESS ,否则为 ERROR 


/******************************DWIN_LCD迪文屏用户函数*********************************************/ 
// DWIN_LCD_Init(Baudrate)                                                                         //宏定义表示 ，实际调用的是串口初始化函数
extern void DWIN_LCD_ReceiveHandleFunction(void);                                                  //mcu接收到迪文屏的响应数据 ，根据接收数据的情况将相应的数据写入相应位置
extern void DWIN_LCD_SendHandleFunction(DWIN_LCD_CommandType FunSel ,uint16_t StartAddr , uint16_t Num);    //发送数据给迪文屏 ，更新屏数据，或读取屏数据
extern ErrorStatus DWIN_LCD_AddrMapDataRead(uint16_t *Data , uint16_t Num , uint16_t StartAddr );  //读取内存维护区的数据保存到变量中，用于实际应用       
extern ErrorStatus DWIN_LCD_AddrMapDataWrite(uint16_t *Data , uint16_t Num , uint16_t StartAddr);  //将相应变量写入 mcu 维护的内存区 ，更新内存维护区数据     


#endif /* __DWIN_LCD_H */


