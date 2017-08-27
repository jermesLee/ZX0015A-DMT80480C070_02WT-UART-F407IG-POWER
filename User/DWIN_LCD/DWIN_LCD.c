#include "DWIN_LCD.h"

/***************************************DWIN_LCD可能的最大一帧数据计算*****************************/ 
#ifndef DWIN_LCD_FrameSizeMax                                                                       // 如果没有宏定义DWIN_LCD_FrameSizeMax   ，则使用默认的DWIN_LCD_FrameSizeMax 
#define DWIN_LCD_FrameSizeMax           (DWIN_LCD_NumSize * 2 + 10 )                                // 加10是DWIN_LCD 别的数据流出空间 ：
#endif                                                                                         


/******************************迪文屏（DWIN_LCD）内存分配*****************************************/ 
uint8_t DWIN_LCD_Data[DWIN_LCD_NumSize * 2];                                                       //为DWIN_LCD迪文屏开辟的缓冲区                                                     
uint8_t DWIN_LCD_SendFrameData[DWIN_LCD_FrameSizeMax];                                             //发送的临时保存位置 ，一帧数据准备好再发送                      
uint8_t DWIN_LCD_ReceiveFrameData[DWIN_LCD_FrameSizeMax];                                          //接收的临时保存位置 ，读取一帧数据保存到 DWIN_LCD_ReceiveFrameData ，然后进行解析                  
uint16_t  DWIN_LCD_SendFrameLength ;                                                               //接收或需发送的字节长度
uint16_t  DWIN_LCD_ReceiveFrameLength ;                                                            //接收或需发送的字节长度



/******************************迪文屏（DWIN_LCD）用户函数*****************************************/ 
void DWIN_LCD_ReceiveHandleFunction(void);                                                         //mcu接收到迪文屏的响应数据 ，根据接收数据的情况将相应的数据写入相应位置
void DWIN_LCD_SendHandleFunction(DWIN_LCD_CommandType FunSel ,uint16_t StartAddr , uint16_t Num);  //发送数据给迪文屏 ，更新屏数据，或读取屏数据
ErrorStatus DWIN_LCD_AddrMapDataRead(uint16_t *Data , uint16_t Num , uint16_t StartAddr );         //读取内存维护区的数据保存到变量中，用于实际应用       
ErrorStatus DWIN_LCD_AddrMapDataWrite(uint16_t *Data , uint16_t Num , uint16_t StartAddr);         //将相应变量写入 mcu 维护的内存区 ，更新内存维护区数据     


/*****************************迪文屏（DWIN_LCD）基本功能子函数************************************/
static ErrorStatus DWIN_LCD_ReadReceiveFrame(void);                                                //从接收缓冲区读取一帧数据（判断原则：帧头和数据量）
static void DWIN_LCD_ReceiveFunction_0X83(void);                                                   // mcu发送给迪文屏0x83指令后，迪文屏响应mcu后 ，mcu对这帧数据进行解析保存到相应内存区
static void DWIN_LCD_SendFunction_0X82(uint16_t StartAddr , uint16_t Num);                         // 发送数据给迪文屏 ，更新迪文屏数据显示
static void DWIN_LCD_SendFunction_0X83(uint16_t StartAddr , uint16_t Num);                         // 读取迪文屏数据 ，将读取的数据保存到mcu为迪文屏开辟的地址里
static void HalfWordArrayCopyByteArray( uint16_t *IN , uint8_t *OUT ,uint16_t IN_Num );            // 将半字数组按一等的方式转换成字节数组
static void ByteArrayCopyHalfWordArray( uint8_t *IN , uint16_t *OUT ,uint16_t OUT_Num );           // 将字节数组按一定的组合方式组合成半字数组
static void ByteArrayCopyByteArray( uint8_t *IN , uint8_t *OUT ,uint8_t Num);                      // 将字节数组按顺序复制到字节数组





/**************************************************************************************************
  * @brief  接收到迪文屏的响应后的总的处理函数，根据接收到的数据情况对缓冲区
  * @param  无
  * @retval 无
  * @notice 本程序只使用了两个指令 ： 0x82 和 0x83 ，对于0x82 写指令没有响应返回
**************************************************************************************************/
void DWIN_LCD_ReceiveHandleFunction(void)
{	
	uint8_t   crc16[2] ;                                                                             // 计算接收到的一帧数据的crc值 的临时保存区 ，便于与接收到的crc进行对比
	
	/****一旦执行一次DWIN_LCD_ReceiveHandleFunction函数就一直读取接收缓冲区数据 ，直到缓冲为空退出函数*****/
  /**即 ：退出while的方式 只有一种: 接收缓冲区为空****/	
	while(1)                                                                                         // 一直读取接收缓冲区里的数据 ，直到缓冲区为空 ，退出函数             
	{	
ReadReceiveFrame :		                                                                             // crc校验失败时，跳到此处，读取下一帧数据
		/**读取一帧数据保存到 DWIN_LCD_ReceiveFrameData***/
		if(DWIN_LCD_ReadReceiveFrame() != SUCCESS ) return ;                                           // 接收迪文屏的一帧数据保存到 DWIN_LCD_FrameData 中 ，接收到的字节数保存到  DWIN_LCD_FrameLength   ，已经包含了帧头的比对                                                                         //
   
		/*********读取一帧数据的CRC校验*******************/
		DWIN_LCD_CRC16_CheckCompute(&DWIN_LCD_ReceiveFrameData[3],DWIN_LCD_ReceiveFrameData[2] - 2 ,crc16);        // 计算接收的数据CRC值 ，注意 ：crc16校验从功能码开始计算 ,减2是因为数据字节数包括CRC校验
		if(DWIN_LCD_CRC16_Check(&DWIN_LCD_ReceiveFrameData[DWIN_LCD_ReceiveFrameData[2] + 1 ],crc16) != SUCCESS )  // 比较计算的CRC16和接收到的crc16是否相等 ，1由 3 - 2 得 ：3：2枕头+1字节 ，2：2CRC校验
			goto ReadReceiveFrame;                                                                       // crc校验失败 ，读取下一帧数据                                   
				 
		/*********在帧数据正确的情况下处理帧数据**********/	
		switch( DWIN_LCD_ReceiveFrameData[3] )                                                         // 不同功能码实现不同的功能 ，本机作为主机 （ 现在只用了0x82 和 0x83 功能码），只有功能码0x83有响应（ 读内存区 ）
		{
			case 0X83 : DWIN_LCD_ReceiveFunction_0X83(); break;                                          // 读数据存储区指令 0x83 ，将读取的数据保存到数据区
			default   : break;                                                                           // 其他功能码暂不支持，这帧数据不要 ，读取下一帧数据
		}                                                                                                    	                                                                                     
	}
}



/**************************************************************************************************
  * @brief  发送给迪文屏显示数据总函数  ，更新迪文屏数据显示 或 读取迪文屏数据
  * @param  FunSel： 功能码选择 ，本程序只使用了两个功能码 ：0x82 和 0x83  
  *         StartAddr：需要操作的首地址 （ 16位 ）
  *         Num      ：需要操作的数据量 （ 单位 ：16位 ，对应一个地址的数据 ）
  * @retval 无
  * @notice DWIN_LCD_SendFunction_0X??函数只是准备好了该发送的数据 ，DWIN_LCD_WriteSend()才是真正的数据发送
**************************************************************************************************/
void DWIN_LCD_SendHandleFunction(DWIN_LCD_CommandType FunSel ,uint16_t StartAddr , uint16_t Num)
{
	/******检测输入参数是否正确，错误则退出函数******/
	if(( Num == 0 ) || ( Num > ( DWIN_LCD_FrameSizeMax >> 1 ) - 4 ) )return ;                        // 数据量错误 ，退出函数 ，一帧数据有 8个字节必须有 ：2帧头 + 1字节数 + 1功能码 + 2起始地址  +  2CRC

#if DWIN_LCD_StartAddr                                                                             //允许的最小地址为非0X000数据            
	if( ( StartAddr < DWIN_LCD_StartAddr ) || ( StartAddr + Num > DWIN_LCD_StartAddr + DWIN_LCD_NumSize ) )// 输入地址是否正确
		return ;                                                                                       // 不做任何处理 ，退出函数
#else                                                                                              //当开始地址为0X0000时  StartAddr < DWIN_LCD_StartAddr  不可能存在  ，没有这个条件编译将有警告                                         
	if( StartAddr + Num > DWIN_LCD_StartAddr + DWIN_LCD_NumSize )                                    // 输入地址是否正确
		return ;                                                                                       // 不做任何处理 ，退出函数
#endif

	/**参数输入正确，不同功能码FunSel执行不同的操作*/
	switch( FunSel )                                                                                 // 根据功能码发送相应的数据
	{                                                                  
		case 0X82 : DWIN_LCD_SendFunction_0X82(StartAddr , Num);break ;                                // 0X82 :DWIN_LCD_WriteDataMemory (写数据存储区) ：2帧头 + 1字节数 + 1功能码 + 2起始地址 +  2N数据       +  2CRC
		case 0X83 : DWIN_LCD_SendFunction_0X83(StartAddr , Num); break ;                               // 0X83 :DWIN_LCD_ReadDataMemory  (读数据存储区) ：2帧头 + 1字节数 + 1功能码 + 2起始地址 +  1需读数据量  +  2CRC
		default   : return ;                                                                           // 其他功能暂不支持 ，退出函数 
	}
  
	/**上面的不同功能码只是准备好数据 ，这才是触发数据发送*/
	DWIN_LCD_WriteSend(DWIN_LCD_SendFrameData ,DWIN_LCD_SendFrameLength);                            // 将准备好的数据发送出去	
}




/**************************************************************************************************
  * @brief  发送给迪文屏显示数据，写屏数据存储区数据 。 功能码 ：0x82 ：写数据存储区指令
  * @param  StartAddr： 需要写数据的起始地值
  *         Num ：      需要写的数据量 （ 单位 ：半字 ）
  * @retval 无
  * @notice 地址和数据量输入是否在地址范围内的判断在DWIN_LCD_SendHandleFunction()处理总函数里有判断
**************************************************************************************************/
static void DWIN_LCD_SendFunction_0X82(uint16_t StartAddr , uint16_t Num)
{
  uint16_t  addr_index ;                                                                           // 地址索引

	/****输入虚拟地址与内存实际地址对应****/
  addr_index = StartAddr - DWIN_LCD_StartAddr ;                                                    //地址索引计算
  addr_index = addr_index << 1 ;                                                                   //访问方式 ：16位
	Num = Num << 1 ;                                                                                 //访问方式 ：16位

	/********准备需要发送的数据************/
	DWIN_LCD_SendFrameLength  = Num  + 8 ;                                                           //2帧头＋１字节数＋１功能码＋2起始地址+2CRC = 8 字节
	DWIN_LCD_SendFrameData[0] = DWIN_LCD_FrameHeadH ;                                                //帧头
	DWIN_LCD_SendFrameData[1] = DWIN_LCD_FrameHeadL ;                                                    
	DWIN_LCD_SendFrameData[2] = Num  + 5 ;                                                           //帧需要的字节数（ 除帧头 和它本身）
	DWIN_LCD_SendFrameData[3] = 0X82 ;                                                               //功能码 , 0x82 ：写数据存储区
	DWIN_LCD_SendFrameData[4] = StartAddr >> 8   ;                                                   //起始地址高字节
	DWIN_LCD_SendFrameData[5] = StartAddr & 0XFF ;                                                   //起始地址低字节
	ByteArrayCopyByteArray(&DWIN_LCD_Data[addr_index], &DWIN_LCD_SendFrameData[6] ,Num);             //复制数据到带发送的数组里
	
	/*************crc16计算***************/
	DWIN_LCD_CRC16_CheckCompute(&DWIN_LCD_SendFrameData[3],Num  + 3 ,&DWIN_LCD_SendFrameData[Num  + 6]);//CRC16计算
	
}





/**************************************************************************************************
  * @brief  读取迪文屏显示的数据 ，读取屏数据存储区数据 。 功能码 ：0x83 ：读数据存储区指令
  * @param  StartAddr： 需要读取数据的起始地值
  *         Num ：      需要读取的数据量 （ 单位 ：半字 ）
  * @retval 无
  * @notice 地址和数据量输入是否在地址范围内的判断在DWIN_LCD_SendHandleFunction(）处理总函数里有判断
**************************************************************************************************/
static void DWIN_LCD_SendFunction_0X83(uint16_t StartAddr , uint16_t Num)
{
	
	/********准备需要发送的数据************/
	DWIN_LCD_SendFrameLength  = 9 ;                                                                  //2帧头＋１字节数＋１功能码＋2起始地址+1需读数据量+2CRC = 9字节
	DWIN_LCD_SendFrameData[0] = DWIN_LCD_FrameHeadH ;                                                //帧头
	DWIN_LCD_SendFrameData[1] = DWIN_LCD_FrameHeadL ;                                                    
	DWIN_LCD_SendFrameData[2] = 0X06 ;                                                               //帧需要的字节数（ 除帧头 和 它本身）
	DWIN_LCD_SendFrameData[3] = 0X83 ;                                                               //功能码 , 0x83 ：读取数据存储区
	DWIN_LCD_SendFrameData[4] = StartAddr >> 8   ;                                                   //起始地址高字节
	DWIN_LCD_SendFrameData[5] = StartAddr & 0XFF ;                                                   //起始地址低字节
	DWIN_LCD_SendFrameData[6] = Num ;                                                                //需要读取的数据量 ，单位：半字
	
	/*************crc16计算***************/
	DWIN_LCD_CRC16_CheckCompute(&DWIN_LCD_SendFrameData[3],4 ,&DWIN_LCD_SendFrameData[7]);           //CRC16计算
	
}





/**************************************************************************************************
  * @brief  维护DWIN_LCD的一段数据区 ，将接收到的迪文屏数据保存到相应的内存区
  * @param  无
  * @retval 无 
  * @notice 当接收到的响应的地址不正确时，退出函数，不做任何处理
**************************************************************************************************/
static void DWIN_LCD_ReceiveFunction_0X83(void)
{
  uint16_t  start_address ;                                                                        //需要操作的首地址                                                                                 // 	
  uint16_t  addr_index ;                                                                           //地址索引 
	uint16_t  num ;                                                                                  //接收到的数据量 （ 单位 ：16位 ）
	
	/******输入虚拟地址与内存实际地址映射*******/
	start_address  =  DWIN_LCD_ReceiveFrameData[4] << 8 | DWIN_LCD_ReceiveFrameData[5] ;             //读取数据的起始地址
	num            =  DWIN_LCD_ReceiveFrameData[6] ;                                                 //需要读取的数据量 。单位 ：半字

#if DWIN_LCD_StartAddr                                                                             //允许的最小地址为非0X000数据            
	if( ( start_address < DWIN_LCD_StartAddr ) || ( start_address + num > DWIN_LCD_StartAddr + DWIN_LCD_NumSize ) ) //地址检测是否正确
		return ;	                                                                                     //地址错误 ，退出函数
#else                                                                                              //当开始地址为0X0000时  StartAddr < DWIN_LCD_StartAddr  不可能存在  ，没有这个条件编译将有警告  
	if( start_address + num > DWIN_LCD_StartAddr + DWIN_LCD_NumSize  )                               //地址检测是否正确
		return ;	                                                                                     //地址错误 ，退出函数
#endif	

	addr_index =  start_address - DWIN_LCD_StartAddr  ;                                              //地址索引
	addr_index =  addr_index << 1 ;                                                                  //访问为16位 ，所有需要左移1位
	num        =   num       << 1 ;                                                                  //num 的单位为 ：16位（ 半字 ）
	
	/******将接收到的数据写入内存实际地址*******/
	ByteArrayCopyByteArray( &DWIN_LCD_ReceiveFrameData[7] , &DWIN_LCD_Data[addr_index] , num );      //保存数据于DWIN_LCD数据区
}




/**************************************************************************************************
  * @brief  维护DWIN_LCD的一段数据区，从DWIN_LCD 的内存区读取数据到实际应用场合
  * @param  Data： 需读取数据的保存位置
  *         Num ： 需要读取的数据量
  *         StartAddr ：读取数据的首地址
  * @retval ERROR :   输入数据有误,不做任何操作，退出函数 ;  
  *         SUCCESS : 输入数据正确 ，读取数据成功
  * @notice 在DWIN_LCD维护的一段数据区里，以字节的方式进行访问 ，且先读高字节数据 ，
  *         再读低字节数据 ， 最后组成一个16位的数据
**************************************************************************************************/
ErrorStatus DWIN_LCD_AddrMapDataRead(uint16_t *Data , uint16_t Num , uint16_t StartAddr )         
{
	uint16_t Index ;                                                                                 // 地址索引
	
	/********检测输入虚拟地址是否正确***********/
#if DWIN_LCD_StartAddr                                                                             //允许的最小地址为非0X000数据            
	if( ( StartAddr < DWIN_LCD_StartAddr ) || ( StartAddr + Num > DWIN_LCD_StartAddr + DWIN_LCD_NumSize ) )  //判断地址输入是否正确
		return ERROR;                                                                                  // 地址不正确 ，退出函数 
#else                                                                                              //当开始地址为0X0000时  StartAddr < DWIN_LCD_StartAddr  不可能存在  ，没有这个条件编译将有警告                                         
	if( StartAddr + Num > DWIN_LCD_StartAddr + DWIN_LCD_NumSize  )                                   //判断地址输入是否正确
		return ERROR;                                                                                  // 地址不正确 ，退出函数 
#endif
		
	/*在地址数据正确下，读取迪文屏数据于实际应用变量里*/
	Index = StartAddr - DWIN_LCD_StartAddr ;                                                         // 地址索引计算
  ByteArrayCopyHalfWordArray( &DWIN_LCD_Data[Index << 1] , Data , Num );                           // 从缓冲区读取数据到实际应用场合
	return SUCCESS ;                                                                                 // 数据读取成功 ，返回 SUCCESS
}






/**************************************************************************************************
  * @brief  维护DWIN_LCD的一段数据区，往DWIN_LCD 的内存区写数据
  * @param  Data： 需写的数据
  *         Num ： 需写数据量
  *         StartAddr ：需写数据在DWIN_LCD的首地址
  * @retval ERROR :   输入数据有误,不做任何操作，退出函数 ;  
  *         SUCCESS : 输入数据正确 ，写数据成功
  * @notice 在DWIN_LCD维护的一段数据区里，以字节的方式进行访问 ，且先读高字节数据 ，
  *         再读低字节数据 ， 最后组成一个16位的数据
**************************************************************************************************/
ErrorStatus DWIN_LCD_AddrMapDataWrite(uint16_t *Data , uint16_t Num , uint16_t StartAddr)       
{
	uint16_t Index ;                                                                                 // 地址索引

	/********检测输入虚拟地址是否正确***********/
#if DWIN_LCD_StartAddr                                                                             //允许的最小地址为非0X000数据            
	if( ( StartAddr < DWIN_LCD_StartAddr ) || ( StartAddr + Num > DWIN_LCD_StartAddr + DWIN_LCD_NumSize ) )  //判断地址输入是否正确
		return ERROR;                                                                                  // 地址不正确 ，退出函数 
#else                                                                                              //当开始地址为0X0000时  StartAddr < DWIN_LCD_StartAddr  不可能存在  ，没有这个条件编译将有警告                                         
	if( StartAddr + Num > DWIN_LCD_StartAddr + DWIN_LCD_NumSize  )                                   //判断地址输入是否正确
		return ERROR;                                                                                  // 地址不正确 ，退出函数 
#endif
	
	/*在地址数据正确下，将数据写入DWIN迪文屏缓冲*/
	Index = StartAddr - DWIN_LCD_StartAddr ;                                                         // 地址索i引计算
  HalfWordArrayCopyByteArray( Data , &DWIN_LCD_Data[Index << 1] , Num );                           // 将数据写进缓冲区中
	return SUCCESS ;                                                                                 // 数据操作成功 ，返回 SUCCESS
}






/**************************************************************************************************
  * @brief  从串口接收缓冲区读取一帧数据（判断原则 ：帧头和字节数 ）
  * @param  无
  * @retval ERR0R :接收缓冲区空 ，SUCCESS :读取到一帧数据
  * @notice 本函数使用了状态机的框架 
**************************************************************************************************/
typedef enum {FrameHeaderH = 1 , FrameHeaderL ,FrameNum ,FrameData   }FrameStatusType ;            // 一帧数据里每个字节的意思 的枚举
FrameStatusType ReceiveFrameStatus = FrameHeaderH ;                                                // 默认为帧头高字节
static ErrorStatus DWIN_LCD_ReadReceiveFrame(void)
{
  /***两种退出while的方式 ： 1. 读取到一阵数据  ；  2. 接收缓冲区为空***/	
	while(1)                                                                                         //一直循环 ，直到读到一帧数据 或 接收缓冲区为空
	{
		switch( ReceiveFrameStatus)                                                                    //状态之间的切换（ 表示现在将要读取字节在帧里表示的意思）
		{
			
			/**************接收帧头高字节状态***********/	
			case FrameHeaderH :                                                                          //状态1 ： 帧头高字节
                				if(DWIN_LCD_ReadReceive(&DWIN_LCD_ReceiveFrameData[0],1) != SUCCESS ) return ERROR ;   // 读取失败 ，缓冲区空 ，退出函数
												if(DWIN_LCD_ReceiveFrameData[0] == DWIN_LCD_FrameHeadH )                   //判断这个字节是帧头高字节吗 ？ 不是不做处理，下次读的还是帧头高字节
                        { 
													ReceiveFrameStatus = FrameHeaderL ;                                      //是帧头 ，切换到状态2 ：读取帧头低字节
												} 
												break ;  
		 
			/**************接收帧头低字节状态***********/	
			case FrameHeaderL :                                                                          //状态2 ： 帧头低字节
												if(DWIN_LCD_ReadReceive(&DWIN_LCD_ReceiveFrameData[1],1) != SUCCESS ) return ERROR ;	 // 读取失败 ，缓冲区空 ，退出函数			
												if(DWIN_LCD_ReceiveFrameData[1] == DWIN_LCD_FrameHeadL )                   // 判断这个字节是帧头低字节吗 ？ 不是退回到状态1 ，是切换到妆台3
													ReceiveFrameStatus = FrameNum ;                                          // 是帧头低字节 ，切换到状态3 ： 读取数据字节数 
											  else
													ReceiveFrameStatus = FrameHeaderH ;	                                     //不是帧头 ，切换到状态1 ：读取帧数据帧头高字节
												break ;  				 
			
			/**************接收帧字节数状态************/
			case FrameNum     :                                                                          //状态3 ：帧数据字节数 （ 不包括 2帧头 + 1字节数（自身）+ 2 CRC校验）
												if(DWIN_LCD_ReadReceive(&DWIN_LCD_ReceiveFrameData[2],1) != SUCCESS ) return ERROR ;	 //读取失败 ，缓冲区空 ，退出函数							 
			                  ReceiveFrameStatus = FrameData ;                                           //读取字节数（数据量）成功 ，切换到 状态4 ： 帧数据和CRC校验
			                  break ;  
			
			/************接收帧数据和CRC校验**********/
			case FrameData    :                                                                          //状态4 ： 帧数据和CRC校验
												if(DWIN_LCD_ReadReceive(&DWIN_LCD_ReceiveFrameData[3],DWIN_LCD_ReceiveFrameData[2] ) != SUCCESS ) //读取数据和CRC
													return ERROR ;                                                           //	读取失败 ，缓冲区空 ，退出函数
			                  else                                                                       //数据读取成狗的处理方法
												{
													ReceiveFrameStatus = FrameHeaderH ;                                      //切换到状态1 ： 帧头高字节 ，下次数据读取为 帧头高字节
													                                                                         //接收到的整帧字节数
													return SUCCESS ;                                                         //退出函数 ，返回数据读取成功
												}		                                                                       //这儿没有break ，if。。else。。两种情况下都会退出函数 ，break不可能执行											 
			                                                                                              
		   /****************不存在的状态*************/
			 default           : ReceiveFrameStatus = FrameHeaderH ;                                     //不存在的状态 ，强制下一个字节为帧头高字节
		}
	}

}










/*****************************************基本功能子函数******************************************/

/**************************************************************************************************
  * @brief  16位数组复制到字符数组
  * @param  IN ： 输入数组头指针 ；  OUT ： 输出数组头指针 ；   Num ：需要复制的数据量
  * @retval 无
  * @notice 先保存16位数据的高字节，再保存低字节 （ 对于16为数据在小端存储模式下，小地址保存低字节数据 ，大地址保存高字节数据）
**************************************************************************************************/
static void HalfWordArrayCopyByteArray( uint16_t *IN , uint8_t *OUT ,uint16_t IN_Num )
{
	uint8_t *In ;                                                           
	uint16_t i ;                                                              
	In  = (uint8_t *)IN ;                                                                            // 将输入的16位数据指针转换为8位数据指针进行访问
	for( i = 0 ; i < 2 * IN_Num ; i = i + 2 )                                                        // 数据填充计数 ，下面为交叉填充
	{
		/********16位数据高低字节交叉复制***********/
		OUT[i+1] = In[i] ;                                                                             // [1] <-- [0] ,  [3] <-- [2] , [5] <-- [4]  ......  
		OUT[i]   = In[i+1] ;                                                                           // [0] <-- [1] ,  [2] <-- [3] , [4] <-- [5] ......  
	}
}





/**************************************************************************************************
  * @brief  8位数组复制到16位数组
  * @param  IN ： 输入数组头指针 ；  OUT ： 输出数组头指针 ；   Num ：需要复制的数据量
  * @retval 无
  * @notice 先保存16位数据的高字节，再保存低字节 （ 对于16为数据在小端存储模式下，小地址保存低字节数据 ，大地址保存高字节数据）
**************************************************************************************************/
static void ByteArrayCopyHalfWordArray( uint8_t *IN , uint16_t *OUT ,uint16_t OUT_Num )
{                                                        
	uint16_t i,j ;                                                           
	for( i = 0 , j = 0 ; i < 2 * OUT_Num ; i = i + 2 ,j++)                                           // 数据填充计数 ，两个8位数据组成一个16位数据
	{
		/***先读高字节在读低字节组成一个16位的数据***/
		OUT[j] = IN[i] << 8 | IN[i+1];                                                                 // [0] <-- [0]<<8|[1] , [1] <-- [2]<<8|[3]  ......  
	}
}



/*************************************************************************
  * @brief  数组复制
  * @param  IN ： 输入数组头指针 ；  OUT ： 输出数组头指针 ；   Num ：需要复制的数据量
  * @retval 无
  * @notice 无
*************************************************************************/
static void ByteArrayCopyByteArray( uint8_t *IN , uint8_t *OUT ,uint8_t Num)
{
  while( Num-- )                                                                                   //复制数据计数
	{
		/****************按顺序复制****************/
	  *OUT++ = *IN++ ;                                                                               //复制数据
	}
}





