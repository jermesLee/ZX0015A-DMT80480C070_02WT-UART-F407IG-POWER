#include "TIME5_Encoder.h" 

#define TIM5_Encoder_InitialValue       (uint16_t)32000                  // 在不溢出的条件下一次向上或向上能够记的数据   TIM5_Encoder_InitialValue 最大为 32767                
#define TIM5_Encoder_ARRValue           (uint16_t)(TIM5_Encoder_InitialValue * 2)   // 此处必须为 * 2                 

/*************************TIM5 编码器内存分配**************************/
int16_t TIM5_Encoder_Overflown = 0 ;                                      //内存溢出计数


/*************************TIM5 编码器内部函数**************************/
static void TIM5_GPIO_Config(void);                                       //配置TIM5 编码器模式的引脚 PH10和PH11
static void TIM5_Mode_Config(void);                                       //配置TIM5工作在 编码器模式
static void TIM5_NVIC_Configuration(void);                                //配置TIM5中断及其优先级


/*************************TIM5 编码器用户函数**************************/
void TIM5_Encoder_Init(void);                                             //TIM5配置为编码器模式                                      
void TIM5_Encoder_SetData(int32_t EncoderValue);                          //设置编码器数据 （-2^31-1  ~  2^31 - 1）
int32_t TIM5_Encoder_GetData(void);                                       //读取编码器数据 （-2^31-1  ~  2^31 - 1）





/*************************************************************************
  * @brief  配置TIM5 编码器模式的引脚 PH10和PH11
  * @param  None
  * @retval None
  * @notice 
*************************************************************************/
static void TIM5_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5);                //引脚复用功能映射
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource11, GPIO_AF_TIM5);
	
	/* TIM5 channel 1 and 2 pin (PH10 and PH11 ) configuration */
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_10 | GPIO_Pin_11 ;	          //选择要控制的GPIOH引脚			
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;                          //复用方式
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;                         //推挽输出 
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_100MHz;                     //设置引脚速率为100MHz 
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_UP ;                         //上拉
	GPIO_Init(GPIOH, &GPIO_InitStructure);                                  //调用库函数，初始化GPIOH10,11

} 





/*************************************************************************
  * @brief  配置TIM5工作在 编码器模式
  * @param  无
  * @retval 无
  * @notice 
*************************************************************************/
static void TIM5_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure; 
	 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);                         //默认缺省值
  TIM_TimeBaseStructure.TIM_Prescaler     = 0x0;                          //预分频器 
  TIM_TimeBaseStructure.TIM_Period        = TIM5_Encoder_ARRValue ;       //设定计数器自动重装值 ARR
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                 //选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_CenterAligned3;//TIM向上/向下计数都会产生中断（条件 ：中断使能）  
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);                         //配置
	 
  TIM_ICInitStructure.TIM_Channel         = TIM_Channel_1;                //IC1
  TIM_ICInitStructure.TIM_ICPolarity      = TIM_ICPolarity_Rising;        //不反相
  TIM_ICInitStructure.TIM_ICSelection     = TIM_ICSelection_DirectTI;     //TI1FP1 -> IC1 , TI2FP2 -> IC2
  TIM_ICInitStructure.TIM_ICPrescaler     = TIM_ICPSC_DIV1;               //外部输入信号不分频
  TIM_ICInitStructure.TIM_ICFilter        = 15 ;                          //选择输入比较滤波器
  TIM_ICInit(TIM5, &TIM_ICInitStructure);                                 //配置 
	
	TIM_ICInitStructure.TIM_Channel         = TIM_Channel_2;                //IC2
  TIM_ICInit(TIM5, &TIM_ICInitStructure);                                 //配置 

  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI2, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式2 ,根据IC2电平对 IC1边沿进行计数
	
  TIM_ARRPreloadConfig(TIM5,DISABLE);                                     //预装载不使能
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);                                   //清除TIM的更新标志位
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);                              //使能更新中断

  TIM_SetCounter(TIM5,TIM5_Encoder_InitialValue);                         //初始化计数器初值
  TIM_Cmd(TIM5, ENABLE);                                                  //使能计数器计数

}







/*************************************************************************
  * @brief  配置TIM5中断及其优先级
  * @param  无
  * @retval 无
  * @notice 
*************************************************************************/
static void TIM5_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;	    // 需打开的中断源  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM5_IRQPreemptionPrio; //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = TIM5_IRQSubPrio ;       //次级优先级	
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;        //TIM2_IRQn中断使能
    NVIC_Init(&NVIC_InitStructure);                                       //配置
}






/*************************************************************************
  * @brief  TIM5初始化总函数 （编码器模式  ）
  * @param  无
  * @retval 无
  * @notice 
*************************************************************************/
void TIM5_Encoder_Init(void)                                        
{
	/**************时钟配置*********************/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);                    //使能定时器5的时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);                   //使能PH端口时钟

	/**************硬件配置*********************/
	TIM5_NVIC_Configuration();                                              // 配置TIM5中断及其优先级
	TIM5_Mode_Config();	                                                    // 定时器TIM5工作模式配置   
	TIM5_GPIO_Config();                                                     // I/0输入引脚配置
}




/*************************************************************************
  * @brief  初始化TIM5 编码器值
  * @param  无
  * @retval 无
  * @notice 在设置前必须关闭定时器TIM5
*************************************************************************/
void TIM5_Encoder_SetData(int32_t EncoderValue)                           //设置编码器数据 （-2^31-1  ~  2^31 - 1）
{	
	 int32_t Encoder_InitialValue ;

	 TIM_Cmd(TIM5, DISABLE);                                                //关定时器  
	
   if( ( EncoderValue < TIM5_Encoder_InitialValue)&&( EncoderValue > -TIM5_Encoder_InitialValue) )	  //EncoderValue没有溢出 ，减小计算量
	 {	
     /*********没有溢出*********/		 
		 TIM5_Encoder_Overflown = 0 ;                                         //溢出出为0
	   Encoder_InitialValue   = EncoderValue + TIM5_Encoder_InitialValue ;  //计算写进定时器ARR的值
	 }
	 else                                                                   //编码器的正确计算 ，把没有溢出的情况分离开 ，减小计算                                                   
	 {
     /***********有溢出*********/		 
		 TIM5_Encoder_Overflown = EncoderValue / TIM5_Encoder_InitialValue ;  //向上向下溢出的次数 （ -13 /5 = -2 ....-3）
		 Encoder_InitialValue   = EncoderValue % TIM5_Encoder_InitialValue ;  //单片机里负数的除法和取余运算：（ -13 /5 = -2 ....-3） 
		 Encoder_InitialValue   = Encoder_InitialValue + TIM5_Encoder_InitialValue ;  //计算写进定时器ARR的值
	 }
	//内存溢出计数	
	 TIM_SetCounter(TIM5,Encoder_InitialValue);                             //初始化计数器初值 ，将Encoder_InitialValue写入 ARR寄存器
	 TIM_Cmd(TIM5, ENABLE);                                                 //使能定时器
}







/*************************************************************************
  * @brief  读取编码器数据 （ -2^31 -1 ~ 2^31 -1）
  * @param  无
  * @retval 编码器数据 （ -2^31 -1 ~ 2^31 -1）
  * @notice 
*************************************************************************/
int32_t TIM5_Encoder_GetData(void)
{
	int32_t Encoder_Data;   
	int32_t Time_CounterData;   
		
	Time_CounterData = TIM_GetCounter(TIM5) - TIM5_Encoder_InitialValue ;   //读取当前编码器值 ：定时器值 - 初始值  	
	
	/*正常情况TIM5_Encoder_Overflown为0*/
	if( TIM5_Encoder_Overflown == 0)                                        // 正常使用 ，不存在向上和向下溢出                                   
	{                              
		Encoder_Data = Time_CounterData ;                                     // 当前编码器值就是旋转的整个编码器的值
		return Encoder_Data ;                                                 // 返回编码值
	}
	
	/*往一个方向旋转 TIM5_Encoder_InitialValue （32000）步以上的情况*/
	Encoder_Data =  TIM5_Encoder_InitialValue * TIM5_Encoder_Overflown  + Time_CounterData ;  //溢出编码值计算
	return Encoder_Data ;                                                   // 返回编码值                           
}







/*************************************************************************
  * @brief  TIM5配置为编码器模式   
  * @param  无
  * @retval 无
  * @notice 
*************************************************************************/
void  TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)                       //更新中断
	{
		if((TIM5->CR1)&0X0010)                                                //向下减1溢出
		{				  
       TIM5_Encoder_Overflown--;                                          //向下减1溢出，溢出次数减1
		} 
		else                                                                  //向上加1溢出
		{				 
			 TIM5_Encoder_Overflown++;                                          //向上加1溢出，溢出次数加1
		}
		TIM_SetCounter(TIM5,TIM5_Encoder_InitialValue);                       //初始化计数器初值
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);                           //清中断标志位
	}
}






