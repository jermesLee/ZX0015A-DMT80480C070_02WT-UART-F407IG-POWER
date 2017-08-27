#include "DWIN_LCD_Communication.h"


/************************** DWIN_LCD功能函数声明*************************/ 
void DWIN_LCD_DisplayInit(void);                                          //上电初始化DWIN_LCD显示数据
void DWIN_LCD_AndMCUCommunication(void);                                  //需要从缓冲区写入	DWIN_LCD的数据 ， 和 将接收到的迪文屏数据处理写入DWIN_LCD缓冲区 
uint16_t DWIN_LCD_DisplaySetLineVoltage(void);                            //计算DWIN_LCD屏显示设置线电压 ，和将设置的线电压写入DWIN_LCD缓冲区
uint16_t DWIN_LCD_DisplaySetFrequency(void);                              //计算DWIN_LCD屏显示设置信号频率，和将设置的频率写入DWIN_LCD缓冲区


/************************************************************************
  * @brief  上电初始化DWIN_LCD显示数据
  * @param  无  
  * @retval 无 
	* @notice Dat里的数据为上电初始化时DWIN_LCD屏显示的数据 
*************************************************************************/
void DWIN_LCD_DisplayInit(void)
{
	uint16_t Dat[]={11,22,33,44,55,66,1007,0,0,10,11,12,13,14,15,16};       //需要在DWIN_LCD显示屏上显示的数据 
  DWIN_LCD_AddrMapDataWrite(Dat , 16 , 0X0000);                           //将需要显示的数据写入DWIN_LCD缓冲区
}





/************************************************************************
  * @brief  需要从缓冲区写入	DWIN_LCD的数据 ， 和 将接收到的迪文屏数据处理写入DWIN_LCD缓冲区 
  * @param  无  
  * @retval 无 
	* @notice 根据实际应用进行修改
*************************************************************************/
void  DWIN_LCD_AndMCUCommunication(void) 
{		
	DWIN_LCD_ReceiveHandleFunction();                                       //将串口接收到的所有数据解析保存到迪文屏数据缓冲区
  DWIN_LCD_SendHandleFunction(DWIN_LCD_WriteDataMemory,0X0000,11);	      //把缓冲区数据写入迪文屏
}



/************************************************************************
  * @brief  计算DWIN_LCD屏显示设置线电压 ，和将设置的线电压写入DWIN_LCD缓冲区
  * @param  无  
  * @retval 无
	* @notice 有无数据输入的标志数 0X7FFF 根据情况进行修改 ，这个数字必须是实际应用不到的数据 
*************************************************************************/
uint16_t  DWIN_LCD_DisplaySetLineVoltage(void) 
{ 
	static uint16_t DWIN_LCD_InputLineVoltage ;                             //实际触摸输入线电压 
	uint16_t InputLineVoltage;                                              //读取触摸输入线电压临时保存位置 ，当为0表示没有触摸数据输入 ，使用上次的触摸输入 
  int32_t  DisplaySetLineVoltageValue ;                                   //显示线电压的计算
  
	/************读取触摸输入线电压***************/
	DWIN_LCD_AddrMapDataRead(&InputLineVoltage , 1 ,DWIN_LCD_InputLineVoltageAddress  ) ;      //读取迪文屏触摸输入的线电压值 
	
	/*************有触摸线电压*输入***************/
	if(InputLineVoltage != 0X7FFF )                                         //有触摸数据输入 （ 如果数据不是O0X7FFF,则屏幕实际有输入） 
	{	
     TIM5_Encoder_SetData(0);                                             //清除编码器值 
		 DWIN_LCD_InputLineVoltage = InputLineVoltage ;                       //保存迪文屏现在数据输入线电压数据
		
		 InputLineVoltage= 0X7FFF;                                            //数据读取后清零 ( 0x7fff是一个实际用不到的数据 ，没有实际意义 ，就是一个标志数字)
		 DWIN_LCD_AddrMapDataWrite(&InputLineVoltage , 1 ,DWIN_LCD_InputLineVoltageAddress  ) ;  //将标志数字（0x7fff）写入缓冲区 ，数据在 DWIN_LCD_AndMCUCommunication() 里发给迪文屏
	}
	
	/**显示线电压 = 迪文屏输入线电压 + 编码器增量**/
	DisplaySetLineVoltageValue = TIM5_Encoder_GetData() + DWIN_LCD_InputLineVoltage ;          //显示线电压 = 迪文屏输入线电压 + 编码器增量
	
	/**************限制最值设置线电压**************/	
	if(DisplaySetLineVoltageValue < DWIN_LCD_DisplaySetLineVoltageMin  )    //线电压小于DWIN_LCD_DisplaySetFrequencyMin
	{
		DisplaySetLineVoltageValue  = DWIN_LCD_DisplaySetLineVoltageMin ;     //设置的线电压过小 ，强制转换为最小电压值 ： DWIN_LCD_DisplaySetFrequencyMin                         
    TIM5_Encoder_SetData( DWIN_LCD_DisplaySetLineVoltageMin - DWIN_LCD_InputLineVoltage);  //设置编码器值 ，使下次   (int16_t)TIM4_Encoder_GetData() + DWIN_LCD_InputFrequency 的显示值为DWIN_LCD_DisplaySetFrequencyMin
  }
  else if(DisplaySetLineVoltageValue > DWIN_LCD_DisplaySetLineVoltageMax )//线电压大于DWIN_LCD_DisplaySetFrequencyMax
	{
		DisplaySetLineVoltageValue = DWIN_LCD_DisplaySetLineVoltageMax ;      //设置的线电压过大 ，强制转换为最大电压值 ： DWIN_LCD_DisplaySetFrequencyMax                         
    TIM5_Encoder_SetData( DWIN_LCD_DisplaySetLineVoltageMax - DWIN_LCD_InputLineVoltage);  //设置编码器值 ，使下次   (int16_t)TIM4_Encoder_GetData() + DWIN_LCD_InputFrequency 的显示值为DWIN_LCD_DisplaySetFrequencyMax	
	}
	
	/************将设置线电压写入缓冲区**********/	
	DWIN_LCD_AddrMapDataWrite( (uint16_t*)&DisplaySetLineVoltageValue , 1 ,DWIN_LCD_DisplaySetLineVoltageAddress ) ; //将显示线电压保存在缓冲区 
  return   (uint16_t)DisplaySetLineVoltageValue ;                        //返回现在实际设置的线电压值
}






/************************************************************************
  * @brief  计算DWIN_LCD屏显示设置信号频率，和将设置的频率写入DWIN_LCD缓冲区
  * @param  无  
  * @retval 无
	* @notice 有无数据输入的标志数 0X7FFF 根据情况进行修改 ，这个数字必须是实际应用不到的数据 
*************************************************************************/
uint16_t  DWIN_LCD_DisplaySetFrequency(void) 
{ 
	static uint16_t DWIN_LCD_InputFrequency ;                               //实际触摸输入频率
	uint16_t InputFrequency;                                                //读取触摸输入频率临时保存位置 ，当为0X7FFF表示没有触摸数据输入 ，使用上次的触摸输入 
  int32_t  DisplaySetFrequencyValue ;                                     //显示频率的计算
  
	/************读取触摸频率输入*****************/
	DWIN_LCD_AddrMapDataRead(&InputFrequency , 1 ,DWIN_LCD_InputFrequencyAddress  ) ;      //读取迪文屏触摸输入的线电压值 
	
	/*************有触摸频率输入******************/
	if(InputFrequency != 0X7FFF )                                           //有触摸数据输入 （ 如果数据不是0X7FFF,则屏幕实际有输入 ,就是一个标志数字） 
	{	
     TIM3_Encoder_SetData(0);                                             //清除编码器值 
		 DWIN_LCD_InputFrequency= InputFrequency ;                            //保存迪文屏现在数据输入频率数据
		
		 InputFrequency= 0X7FFF ;                                             //数据读取后清零 ( 0x7fff是一个实际用不到的数据 ，没有实际意义 ，就是一个标志数字)
		 DWIN_LCD_AddrMapDataWrite(&InputFrequency , 1 ,DWIN_LCD_InputFrequencyAddress  ) ;  //将标志数字（0x7fff）写入缓冲区 ，数据在 DWIN_LCD_AndMCUCommunication() 里发给迪文屏
	}
	
	/**显示设置频率 = 迪文屏输入频率 + 编码器增量**/
	DisplaySetFrequencyValue = TIM3_Encoder_GetData() + DWIN_LCD_InputFrequency ; //显示设置频率= 迪文屏输入频率+ 编码器增量
	
	/**************限制最值设置信号频率************/	
	if(DisplaySetFrequencyValue < DWIN_LCD_DisplaySetFrequencyMin  )        //线电压小于DWIN_LCD_DisplaySetFrequencyMin
	{
		DisplaySetFrequencyValue = DWIN_LCD_DisplaySetFrequencyMin ;          //设置的线电压过小 ，强制转换为最小电压值 ： DWIN_LCD_DisplaySetFrequencyMin                         
    TIM3_Encoder_SetData( DWIN_LCD_DisplaySetFrequencyMin - DWIN_LCD_InputFrequency);  //设置编码器值 ，使下次   (int16_t)TIM4_Encoder_GetData() + DWIN_LCD_InputFrequency 的显示值为DWIN_LCD_DisplaySetFrequencyMin
  }
  else if(DisplaySetFrequencyValue > DWIN_LCD_DisplaySetFrequencyMax )    //线电压大于DWIN_LCD_DisplaySetFrequencyMax
	{
		DisplaySetFrequencyValue = DWIN_LCD_DisplaySetFrequencyMax ;          //设置的线电压过大 ，强制转换为最大电压值 ： DWIN_LCD_DisplaySetFrequencyMax                         
    TIM3_Encoder_SetData( DWIN_LCD_DisplaySetFrequencyMax - DWIN_LCD_InputFrequency);  //设置编码器值 ，使下次   (int16_t)TIM4_Encoder_GetData() + DWIN_LCD_InputFrequency 的显示值为DWIN_LCD_DisplaySetFrequencyMax	
	}
	
	/************将设置信号频率写入缓冲区**********/	
	DWIN_LCD_AddrMapDataWrite( (uint16_t*)&DisplaySetFrequencyValue , 1 ,DWIN_LCD_DisplaySetFrequencyAddress ) ; //将显示频率保存在缓冲区 
  return   (uint16_t)DisplaySetFrequencyValue ;                          //返回现在实际设置的信号频率值
}
