#include "FSMC.h"

static void FSMC_GPIO_Config(void);                                       //初始化FSMC I/O 控制引脚
static void FSMC_MODE_Config(void);                                       //FSMC工作模式和时序时间配置
void FSMC_Init(void);                                                     //fsmc 总线初始化总函数




/*************************************************************************
  * @brief  初始化FSMC I/O 控制引脚
  * @param  无
  * @retval 无
  * @notice 根据实际用到的引脚可以进行裁剪 ，对于不用的FSMC引脚可以当普通的I/O口使用
*************************************************************************/
static void FSMC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;  		   

	
/******************************数据线 FSMC_D[15..0] ********************** 
	     FSMC_D0   --->  PD14     // PORT D
	     FSMC_D1   --->  PD15     //
	     FSMC_D2   --->  PD0      //
	     FSMC_D3   --->  PD1      // 
	     FSMC_D4   --->  PE7      // PORT E
	     FSMC_D5   --->  PE8      //
	     FSMC_D6   --->  PE9      //
	     FSMC_D7   --->  PE10     //
	     FSMC_D8   --->  PE11     //
	     FSMC_D9   --->  PE12     //
	     FSMC_D10  --->  PE13     //
	     FSMC_D11  --->  PE14     //
	     FSMC_D12  --->  PE15     //
	     FSMC_D13  --->  PD8      // PORT D
	     FSMC_D14  --->  PD9      //
	     FSMC_D15  --->  PD10     //
***************************************************************************/
    /* 配置FSMC相对应的数据线,FSMC D0~D15*/	
		GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;                        //复用方式
		GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;                       //推挽输出 
		GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_100MHz;                   //设置引脚速率为100MHz 
		GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_UP ;                       //上拉
 		
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FSMC);                //FSMC_D0   --->  PD14
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_FSMC);                //FSMC_D1   --->  PD15
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource0 ,GPIO_AF_FSMC);                //FSMC_D2   --->  PD0
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource1 ,GPIO_AF_FSMC);                //FSMC_D3   --->  PD1
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource7 ,GPIO_AF_FSMC);                //FSMC_D4   --->  PE7
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource8 ,GPIO_AF_FSMC);                //FSMC_D5   --->  PE8
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource9 ,GPIO_AF_FSMC);                //FSMC_D6   --->  PE9
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource10,GPIO_AF_FSMC);                //FSMC_D7   --->  PE10
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_FSMC);                //FSMC_D8   --->  PE11
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_FSMC);                //FSMC_D9   --->  PE12
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_FSMC);                //FSMC_D10  --->  PE13
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_FSMC);                //FSMC_D11  --->  PE14
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource15,GPIO_AF_FSMC);                //FSMC_D12  --->  PE15
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource8 ,GPIO_AF_FSMC);                //FSMC_D13  --->  PD8
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource9 ,GPIO_AF_FSMC);                //FSMC_D14  --->  PD9
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource10,GPIO_AF_FSMC);                //FSMC_D15  --->  PD10


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 |              
                                  GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;                                           
    GPIO_Init(GPIOD, &GPIO_InitStructure);                                // PORT D  上FSMC 引脚的配置                                                    
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7  | GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | 
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                  GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure);                                // PORT E  上FSMC 引脚的配置     
 
 
 
 
 
/******************************地址线 FSMC_A[25..0] ********************** 
	     FSMC_A0   --->  PF0     // PORT F 
	     FSMC_A1   --->  PF1     // 
	     FSMC_A2   --->  PF2     // 
	     FSMC_A3   --->  PF3     // 
	     FSMC_A4   --->  PF4     // 
	     FSMC_A5   --->  PF5     // 
	     FSMC_A6   --->  PF12    // 
	     FSMC_A7   --->  PF13    // 
	     FSMC_A8   --->  PF14    // 
	     FSMC_A9   --->  PF15    // 
	     FSMC_A10  --->  PG0     // PORT G
	     FSMC_A11  --->  PG1     // 
	     FSMC_A12  --->  PG2     // 
	     FSMC_A13  --->  PG3     // 
	     FSMC_A14  --->  PG4     // 
	     FSMC_A15  --->  PG5     // 
	     FSMC_A16  --->  PD11    // PORT D 
	     FSMC_A17  --->  PD12    // 
	     FSMC_A18  --->  PD13    // 
	     FSMC_A19  --->  PE3     // PORT E
	     FSMC_A20  --->  PE4     // 
	     FSMC_A21  --->  PE5     // 
	     FSMC_A22  --->  PE6     // 
	     FSMC_A23  --->  PE2     // 
	     FSMC_A24  --->  PG13    // PORT G
	     FSMC_A25  --->  PG14    // 
*************************************************************************/

  /* 配置FSMC相对应的数据线,FSMC A0~A25 */
		
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource0 ,GPIO_AF_FSMC);              //FSMC_A0   --->  PF0
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource1 ,GPIO_AF_FSMC);              //FSMC_A1   --->  PF1
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource2 ,GPIO_AF_FSMC);              //FSMC_A2   --->  PF2
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource3 ,GPIO_AF_FSMC);              //FSMC_A3   --->  PF3
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource4 ,GPIO_AF_FSMC);              //FSMC_A4   --->  PF4
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource5 ,GPIO_AF_FSMC);              //FSMC_A5   --->  PF5
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource12,GPIO_AF_FSMC);              //FSMC_A6   --->  PF6
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource13,GPIO_AF_FSMC);              //FSMC_A7   --->  PF7
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource14,GPIO_AF_FSMC);              //FSMC_A8   --->  PF8
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource15,GPIO_AF_FSMC);              //FSMC_A9   --->  PF9
//		GPIO_PinAFConfig(GPIOF,GPIO_PinSource0 ,GPIO_AF_FSMC);              //FSMC_A10  --->  PG0
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource1 ,GPIO_AF_FSMC);              //FSMC_A11  --->  PG1
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource2 ,GPIO_AF_FSMC);              //FSMC_A12  --->  PG2
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource3 ,GPIO_AF_FSMC);              //FSMC_A13  --->  PG3
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource4 ,GPIO_AF_FSMC);              //FSMC_A14  --->  PG4
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource5 ,GPIO_AF_FSMC);              //FSMC_A15  --->  PG5
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource11,GPIO_AF_FSMC);                //FSMC_A16  --->  PD11
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_FSMC);                //FSMC_A17  --->  PD12
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_FSMC);                //FSMC_A18  --->  PD13
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource3 ,GPIO_AF_FSMC);                //FSMC_A19  --->  PE3
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource4 ,GPIO_AF_FSMC);                //FSMC_A20  --->  PE4
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource5 ,GPIO_AF_FSMC);                //FSMC_A21  --->  PE5	
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource6 ,GPIO_AF_FSMC);                //FSMC_A22  --->  PE6
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource2 ,GPIO_AF_FSMC);                //FSMC_A23  --->  PE2
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource13,GPIO_AF_FSMC);                //FSMC_A24  --->  PG13
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_FSMC);                //FSMC_A25  --->  PG14
		

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2  | GPIO_Pin_3  | 
//                                GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | 
//                                GPIO_Pin_14 | GPIO_Pin_15;
//  GPIO_Init(GPIOF, &GPIO_InitStructure);                                // PORT F  上FSMC 引脚的配置   
  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
//                                GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_13| GPIO_Pin_14;                               
//  GPIO_Init(GPIOG, &GPIO_InitStructure);                                // PORT G  上FSMC 引脚的配置   
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13| GPIO_Pin_14;                 // PORT G  上FSMC 引脚的配置                             
  GPIO_Init(GPIOG, &GPIO_InitStructure); 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_6 ; 
  GPIO_Init(GPIOE, &GPIO_InitStructure);                                  // PORT E  上FSMC 引脚的配置   
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);                                  // PORT D  上FSMC 引脚的配置   
   
	 	



/********************控制线 FSMC_NOE , FSMC_NWE ,FSMC_NWAIT ***************** 
	     FSMC_NOE   --->  PD4     // PORT D 
	     FSMC_NWE   --->  PD5     //  
	     FSMC_NWAIT --->  PD6     //  
*****************************************************************************/	  

		GPIO_PinAFConfig(GPIOD,GPIO_PinSource4 ,GPIO_AF_FSMC);                //FSMC_NOE   --->  PD4 
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource5 ,GPIO_AF_FSMC);                //FSMC_NWE   --->  PD5
//		GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_FSMC);              //FSMC_NWAIT --->  PD6 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5;                 // NOE and NWE configuration 
		GPIO_Init(GPIOD, &GPIO_InitStructure);
  
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;                             // NWAIT configuration 
//  GPIO_Init(GPIOD, &GPIO_InitStructure);





/****************************************************************************
2^26 =0X0400 0000 = 64MB,每个 BANK 有4*64MB = 256MB
64MB:FSMC_Bank1_NORSRAM1:0X6000 0000 ~ 0X63FF FFFF
64MB:FSMC_Bank1_NORSRAM2:0X6400 0000 ~ 0X67FF FFFF
64MB:FSMC_Bank1_NORSRAM3:0X6800 0000 ~ 0X6BFF FFFF
64MB:FSMC_Bank1_NORSRAM4:0X6C00 0000 ~ 0X6FFF FFFF
*****************************************************************************/
/*****************************NOR/PSRAM 存储控制器****************************
	     FSMC_NE1 / FSMC_NCE2     --->  PD7     // PORT D 
	     FSMC_NE2 / FSMC_NCE3     --->  PG9     // PORT G
	     FSMC_NE3 / FSMC_NCE4_1   --->  PG10    // 
	     FSMC_NE4                 --->  PG12    // 
	     FSMC_CLK                 --->  PD3     // PORT D
	     FSMC_NBL1                --->  PE1     // PORT E 
	     FSMC_NBL0                --->  PE0     // 
	     FSMC_NCE4_2              --->  PG11    // PORT G 
	     FSMC_NL(NADV)            --->  PB7     // PORT B 
******************************************************************************/   
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource7 ,GPIO_AF_FSMC);                  //FSMC_NE1 / FSMC_NCE2     --->  PD7 
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource9 ,GPIO_AF_FSMC);                //FSMC_NE2 / FSMC_NCE3     --->  PG9
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource10,GPIO_AF_FSMC);                //FSMC_NE3 / FSMC_NCE4_1   --->  PG10 
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource12,GPIO_AF_FSMC);                //FSMC_NE4                 --->  PG12
//		GPIO_PinAFConfig(GPIOD,GPIO_PinSource3 ,GPIO_AF_FSMC);                //FSMC_CLK                 --->  PD3
//    GPIO_PinAFConfig(GPIOE,GPIO_PinSource1 ,GPIO_AF_FSMC);                //FSMC_NBL1                --->  PE1
//    GPIO_PinAFConfig(GPIOE,GPIO_PinSource0 ,GPIO_AF_FSMC);                //FSMC_NBL0                --->  PE0  
//    GPIO_PinAFConfig(GPIOG,GPIO_PinSource11,GPIO_AF_FSMC);                //FSMC_NCE4_2              --->  PG11 
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7 ,GPIO_AF_FSMC);                  //FSMC_NL(NADV)            --->  PB7  

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
		GPIO_Init(GPIOD, &GPIO_InitStructure);                                  // NE1 configuration 
    
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
//  GPIO_Init(GPIOG, &GPIO_InitStructure);                                  // NE2 configuration 
//    
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
//  GPIO_Init(GPIOG, &GPIO_InitStructure);                                  // NE3 configuration 
//    
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
//  GPIO_Init(GPIOG, &GPIO_InitStructure);                                  // NE4 configuration 
// 
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
//  GPIO_Init(GPIOD, &GPIO_InitStructure);                                  // CLK configuration 
//  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
//  GPIO_Init(GPIOE, &GPIO_InitStructure);                                  // NBL0, NBL1 configuration 
//  
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
//  GPIO_Init(GPIOG, &GPIO_InitStructure);                                  //  FSMC_NCE4_2 configuration 
//  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);                                  //  FSMC_NL     configuration 
  
	
}





/*************************************************************************
  * @brief  FSMC工作模式及工作时序时间的配置
  * @param  无
  * @retval 无
  * @notice 工作时序时间由被控存储器时间计算得出
*************************************************************************/
static void FSMC_MODE_Config(void)
{
    FSMC_NORSRAMInitTypeDef        FSMC_NORSRAMInitStructure;                                      // FSMC工作模式变量
    FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMReadTimingInitStructure;                            // FSMC工作时序时间变量 
    FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMWriteTimingInitStructure;                           // FSMC工作时序时间变量 
    
	  /*******************写时序*****************/
  	FSMC_NORSRAMWriteTimingInitStructure.FSMC_AddressSetupTime      = 0x01;	                       //SetTimeValue(0-15)  + 1 :地址建立时间 ( 异步 )   
    FSMC_NORSRAMWriteTimingInitStructure.FSMC_AddressHoldTime       = 0x01;	                       //SetTimeValue(0-15)  + 1 :地址保持时间 ( 异步 复用IO )
    FSMC_NORSRAMWriteTimingInitStructure.FSMC_DataSetupTime         = 0x02;		                     //SetTimeValue(1-255) + 1 :数据建立时间 ( 异步 )
    FSMC_NORSRAMWriteTimingInitStructure.FSMC_BusTurnAroundDuration = 0x0F;                        //SetTimeValue(0-15)  + 1 :总线恢复时间 ( 异步或同步读 ），一次读操作之后控制器需要在数据总线上为下次操作送出地址 ，这个延时就是为了防止总线冲突
    FSMC_NORSRAMWriteTimingInitStructure.FSMC_CLKDivision           = 0x00;                        //SetTimeValue(1-15)  + 1 :时钟分频输出（ 同步通信使用 ） ，以 HCLK 周期为单位
    FSMC_NORSRAMWriteTimingInitStructure.FSMC_DataLatency           = 0x00;                        //SetTimeValue(0-15)  + 2 :数据延迟时间（ 同步通信使用 ），处于同步成组模式的NOR闪存 ，需要定义在读取第一个数据之前等待的存储器周期数目 ，单位：CLK ,不是HCLK ，且在异步访问中不起作用
  	FSMC_NORSRAMWriteTimingInitStructure.FSMC_AccessMode            = FSMC_AccessMode_D;	         //访问模式 D
   
	 /********************读时序*****************/
  	FSMC_NORSRAMReadTimingInitStructure.FSMC_AddressSetupTime       = 0x01;	                       //SetTimeValue(0-15)  + 1 :地址建立时间 ( 异步 )   
    FSMC_NORSRAMReadTimingInitStructure.FSMC_AddressHoldTime        = 0x01;	                       //SetTimeValue(0-15)  + 1 :地址保持时间 ( 异步 复用IO )
    FSMC_NORSRAMReadTimingInitStructure.FSMC_DataSetupTime          = 0x03;		                     //SetTimeValue(1-255) + 1 :数据建立时间 ( 异步 )
    FSMC_NORSRAMReadTimingInitStructure.FSMC_BusTurnAroundDuration  = 0x00;                        //SetTimeValue(0-15)  + 1 :总线恢复时间 ( 异步或同步读 ），一次读操作之后控制器需要在数据总线上为下次操作送出地址 ，这个延时就是为了防止总线冲突
    FSMC_NORSRAMReadTimingInitStructure.FSMC_CLKDivision            = 0x00;                        //SetTimeValue(1-15)  + 1 :时钟分频输出（ 同步通信使用 ） ，以 HCLK 周期为单位
    FSMC_NORSRAMReadTimingInitStructure.FSMC_DataLatency            = 0x00;                        //SetTimeValue(0-15)  + 2 :数据延迟时间（ 同步通信使用 ），处于同步成组模式的NOR闪存 ，需要定义在读取第一个数据之前等待的存储器周期数目 ，单位：CLK ,不是HCLK ，且在异步访问中不起作用
  	FSMC_NORSRAMReadTimingInitStructure.FSMC_AccessMode             = FSMC_AccessMode_A;	         //访问模式 A
 
    /***************寄存器配置****************/ 
    FSMC_NORSRAMInitStructure.FSMC_Bank                  = FSMC_Bank1_NORSRAM1;                    //使用FSMC_BANK1 的子板块 1                  
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux        = FSMC_DataAddressMux_Disable;            //禁用地址数据复用
    FSMC_NORSRAMInitStructure.FSMC_MemoryType            = FSMC_MemoryType_SRAM;                   //存储器类型为 ： SRAM
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;               //存储器数据宽度为 ： 16位
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;             //是否允许对存储器进行写操作 
	  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode          = FSMC_ExtendedMode_Enable;               //是否允许使用扩展功能 （ 读写使用不同的时序 ，即 ：允许使用FSMC_SWTR寄存器）
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait      = FSMC_AsynchronousWait_Disable ;         //异步通信等待 不使能  
		FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;           //关闭FSMC突发成组访问模式 （ 在成组模式下使用 ）
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;            //等待信号有效极性 （ 在成组模式下使用 ）
    FSMC_NORSRAMInitStructure.FSMC_WrapMode              = FSMC_WrapMode_Disable;                  //决定控制器是否支持吧非对齐的AHB成组操作分割为2次线性操作（ 在成组模式下使用 ）
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;  //决定存储器在等待状态前的一个时钟周期产生NWAIT信号 ，还是等待状态期间产生NWAIT信号 （ 在成组模式下使用 ）
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;                //是否允许通过NWAIT信号插入等待状态 （ 成组传输模式下使用 ）
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst            = FSMC_WriteBurst_Disable;                //是否允许通过NWAIT信号插入等待状态 ， 读操作的同步成组传输协议使能位是 ：FSMC_BCRx寄存器的BURETEN位 （ 成组传输模式下使用 ）
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMReadTimingInitStructure;   //读时序参数
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &FSMC_NORSRAMWriteTimingInitStructure;  //写时序参数  
    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);                                                  //配置信息写入相应寄存器
    
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);                                                  //使能 FSMC_Bank1 子板块 1 
                                        
}





/*************************************************************************
  * @brief  FSMC初始化总函数
  * @param  无
  * @retval 无
  * @notice 时钟使能 和 FSMC_GPIO_Config()中的引脚需要根据实际情况进行留舍
*************************************************************************/
	
void FSMC_Init(void)
{
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);                                           // 使能FSMC时钟
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE |                          // 使能FSMC对应相应管脚时钟
	                          RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG |
	                          RCC_AHB1Periph_GPIOB , ENABLE);
    FSMC_GPIO_Config();                                                                            //初始化FSMC I/O 控制引脚
    FSMC_MODE_Config();                                                                            //FSMC工作模式和时序时间配置
}




