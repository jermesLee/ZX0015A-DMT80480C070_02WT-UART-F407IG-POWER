#include "includes.h"

typedef struct DisplayParametersType
{
	uint16_t SetFrequency ;                                                // 设置输出信号频率
	uint32_t SetVoltageAmplitude;                                          // 设置输出电压幅值
	uint16_t ActualFrequency ;                                             // 实际输出电压频率
	uint32_t ActualVoltageAmplitude;                                       // 实际输出电压幅值

}DisplayParametersType ;

DisplayParametersType DWIN_LCD_DisplayParameters ;

/************************************************************************
  * @brief  主函数
  * @param  无  
  * @retval 无 
	* @notice 
*************************************************************************/
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                         //设置系统中断优先级分组2
  LED_Init();                                                             //LED调试引脚初始化函数 ( 调试时使用 )
	TIM6_Init();                                                            //定时器定时初始化
	DWIN_LCD_Init(115200);                                                  //迪文屏通信硬件初始化
	DWIN_LCD_DisplayInit();                                                 //刚上电时屏幕的显示信息初始化
	TIM3_Encoder_Init();                                                    //频率  增量调节时，定时器TIM3编码器模式初始化
	TIM5_Encoder_Init();                                                    //线电压增量调节时，定时器TIM5编码器模式初始化
	FSMC_Init();                                                            //FSMC 总线初始化
	while(1)                                                                                                                                                                              
	{
		/***每隔100ms把需要需改的数据写入LCD缓冲区**/ 	
		if( TIM6_100MsFinish() == SET )                                       //100MS定时时间到 
		{	
			LED1_TOGGLE();                                                      //调试使用
			DWIN_LCD_DisplayParameters.SetVoltageAmplitude = DWIN_LCD_DisplaySetLineVoltage(); //显示设置线电压值 ，返回  设置线电压值
			DWIN_LCD_DisplayParameters.SetFrequency        = DWIN_LCD_DisplaySetFrequency();	 //显示设置频率	    ，返回  设置频率

			FPGA_SetOutputFrequency (DWIN_LCD_DisplayParameters.SetFrequency);                 // 将设置的输出电压频率发送给FPGA
			FPGA_SetOutputVoltageAmplitude (DWIN_LCD_DisplayParameters.SetVoltageAmplitude);   //	将设置的输出电压幅值发送给FPGA
		}
		
   			
		/*********每隔50ms与迪文屏通信一次**********/ 
		if( TIM6_50MsFinish() == SET )                                        //50MS定时时间到 
		{	
			LED2_TOGGLE();                                                      //调试使用
			DWIN_LCD_AndMCUCommunication();                                     //将内存区数据发送给迪文屏,处理接收到的数据
		}
	}
}















