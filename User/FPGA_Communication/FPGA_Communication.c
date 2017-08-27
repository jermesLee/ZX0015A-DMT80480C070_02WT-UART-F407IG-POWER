#include "FPGA_Communication.h"

/************************** FPGA功能函数声明*************************/ 
void FPGA_SetOutputFrequency (uint32_t OutputFrequency);                  //设置FPGA 输出频率值 ,单位 ：   1 LSB = 1 Hz 
void FPGA_SetOutputVoltageAmplitude (uint32_t OutputAmplitude );          //设置FPGA 输出电压的幅值 ,单位：1 LSB = 0.01v 


/************************************************************************
  * @brief  设置FPGA 输出频率值
  * @param  输出信号的频率值 ,单位 ：1 LSB = 1 Hz 
  * @retval 无 
	* @notice 实际是计算和设置了分频系数 
*************************************************************************/
void FPGA_SetOutputFrequency (uint32_t OutputFrequency)
{
	uint32_t FrequencyDivsion;                                              // 分频系数（ 32 位 ）
	uint16_t FrequencyDivsionH;                                             // 分频系数的高16位 
	uint16_t FrequencyDivsionL;                                             // 分频系数的低16位
	
	/********************设置的分频系数和输出频率的关系********************/
	OutputFrequency   = OutputFrequency * 6 ;                               // 分频系数和输出频率的关系由 FPGA 的程序设计决定 
	FrequencyDivsion  = (FPGA_StandardInputFrequency + OutputFrequency )/( OutputFrequency << 1 )-1; //加OutputFrequency 起到 四舍五入 的效果
	FrequencyDivsionH = (uint16_t)(FrequencyDivsion >> 16 );                // 分离 高16位        
	FrequencyDivsionL = (uint16_t)(FrequencyDivsion & 0X0000FFFF ) ;        // 分离 低16位
	
	/**************************数据FSMC发送给FPGA**************************/
	FSMC_BANK1_FPGAWriteHalfWord(FPGA_SetFrequencyDivsionAddressH , FrequencyDivsionH);  // 发送分频系数的高 8 位 
	FSMC_BANK1_FPGAWriteHalfWord(FPGA_SetFrequencyDivsionAddressL , FrequencyDivsionL);  // 发送分频系数的低 8 位 
	FSMC_BANK1_FPGAWriteHalfWord(FPGA_SetDataLatchAddress , 1);             //发送数据锁存信号 ，实际是利用一个地址线的上升沿 ，数据没有实际意义                     
}






/************************************************************************
  * @brief  设置FPGA 输出电压的幅值
  * @param  设置输出电压的幅值 ，单位：1LSB = 0.01v 
  * @retval 无 
	* @notice 实际项目是通过16位的DAC输出 ，所以实际设置的是DAC值
*************************************************************************/
void FPGA_SetOutputVoltageAmplitude (uint32_t OutputAmplitude )
{
	uint16_t DAC16BitValue;                                                 //16位DAC值
	
	/******************设置的输出电压幅值和DAC16值的关系 ******************/
	DAC16BitValue = OutputAmplitude * 65536 / 250 ;                         //DAC16BitValue = (OutputAmplitude / 100 ) * 65536 / 2.5 ;100来至于 1LSB = 0.01v 
	                                                                        // 2.5 为 参考电压 2.5V ,65536 为  16位 DAC的满量程
	/**************************数据FSMC发送给FPGA**************************/
	FSMC_BANK1_FPGAWriteHalfWord(FPGA_SetDAC16BitValueAddress , DAC16BitValue);  // 发送DAC16值 
	FSMC_BANK1_FPGAWriteHalfWord(FPGA_SetDataLatchAddress , 1);             //发送数据锁存信号 ，实际是利用一个地址线的上升沿 ，数据没有实际意义                     
}



