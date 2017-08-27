#include "LED.h" 

/****************************用户函数************************************/
void LED_Init(void);                                                      //LED初始化函数 ，配置MCU IO口 和初始化IO口初始化电平

/****************************内部函数************************************/
static void LED_GPIO_Config(void);


/*************************************************************************
  * @brief  LED IO 口配置 
  * @param  无
  * @retval 无
  * @notice 无
*************************************************************************/
void LED_GPIO_Config(void)
{		
		GPIO_InitTypeDef GPIO_InitStructure;		                              //定义一个GPIO_InitTypeDef类型的结构体
		                                                   
	  /* Configure PG6 and PG8 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_5;	                        //选择要控制的GPIOH引脚			
		GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_OUT;                       //输出方式
		GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;                       //推挽输出 
		GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_100MHz;                   //设置引脚速率为100MHz 
		GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_UP;                        //上拉
		GPIO_Init(GPIOH, &GPIO_InitStructure);                                //调用库函数，初始化GPIOH5
     	
	 	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_4;			                    //选择要控制的GPIOA引脚														   	
		GPIO_Init(GPIOA, &GPIO_InitStructure);                                //调用库函数，初始化GPIOA4

		GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_6;			                    //选择要控制的GPIOA引脚														   	
		GPIO_Init(GPIOA, &GPIO_InitStructure);          	                    //调用库函数，初始化GPIOA6

		GPIO_SetBits(GPIOH, GPIO_Pin_5);	                                    //初始化引脚
		GPIO_SetBits(GPIOA, GPIO_Pin_4);	                                    //初始化引脚
		GPIO_SetBits(GPIOA, GPIO_Pin_6);	                                    //初始化引脚
}




/*************************************************************************
  * @brief  LED 初始化函数 
  * @param  无
  * @retval 无
  * @notice 开启时钟 ，和Cortex-M3不同 ，IO口外设挂在AHB1 总线上
*************************************************************************/
void LED_Init(void)
{
	 /**********开启GPIOH和GPIOA的外设时钟******/  
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH |RCC_AHB1Periph_GPIOA, ENABLE);	 
  
	 /**********LED用到的io口配置***************/
 	 LED_GPIO_Config(); 
}



/*********************************************END OF FILE**********************/
