#ifndef _FPGA_COMMUNICATION_H_
#define	_FPGA_COMMUNICATION_H_

#include "includes.h"                                                     // 所有的头文件
#include "FSMC.h"


/***********************FPGA地址数据说明********************************/ 
#define FPGA_StandardInputFrequency             ((uint32_t)90000000)      //FPGA 基准输入频率 90M

/*************写数据地址************/
#define FPGA_SetDataLatchAddress                (1 << 24)                 //将设置数据所存到寄存器的信号线地址 ，实际是利用了一个地址线的边沿做锁存

#define FPGA_SetFrequencyDivsionAddressH        (1 << 20)                 //设置 FPGA 波形分频系数的高16位数据地址 
#define FPGA_SetFrequencyDivsionAddressL        (2 << 20)                 //设置 FPGA 波形分频系数的低16位数据地址     
#define FPGA_SetDAC16BitValueAddress            (3 << 20)                 //设置 FPGA 输出波形的幅值 （16位DAC对应的数值 ）地址      


/*************读数据地址************/




/************************** FPGA功能函数声明*************************/ 
extern void FPGA_SetOutputFrequency (uint32_t OutputFrequency);           //设置FPGA 输出频率值 ,单位 ：   1 LSB = 1 Hz 
extern void FPGA_SetOutputVoltageAmplitude (uint32_t OutputAmplitude );   //设置FPGA 输出电压的幅值 ,单位：1 LSB = 0.01v 




#endif /* __DWIN_LCD_H */


