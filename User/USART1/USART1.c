#include "USART1.h"

/****************************USART1  �ڴ����******************************/
uint8_t USART1_SendData[USART1_SendSizeMax];                               //�������ݷ��ͻ�����
uint8_t USART1_ReceiveData[USART1_ReceiveSizeMax];                         //�������ݽ��ջ�����
uint16_t USART1_SendFront ,USART1_SendRear ,USART1_SendNextStart;          //���ڷ��ͻ��������ݹ���
uint16_t USART1_ReceiveFront ,USART1_ReceiveRear ;                         //���ڽ��ջ��������ݹ���
uint8_t USART1_SendBusyFlag  ;                                             //����DMA��������æµ��־λ ��1�����ڷ������� ��0��DMA���������ݷ��� ����ʱ���Դ���������һ������


/****************************USART1  �ڲ�����******************************/ 
static void USART1_GPIO_Config(void);                                      //���ô���USART1���ݴ�������I/O��                 
static void USART1_Mode_Config(uint32_t Baudrate);                         //���ô���USART1����ģʽ 
static void USART1_NVIC_Config(void);                                      //�ж����ȼ�����
static void USART1_DMA_Config(void);                                       //USART1 DMA ����
static void USART1_DataSendTrigger(void);                                  //����DMA�Ƿ����ڷ��ͣ��� �ڴ����Ƿ������ݴ�����һ�����ݵķ���
static void USART1_MemoryInit(void);                                       //��ʼ�����ڻ��������ݹ���



/****************************USART1  �û�����******************************/ 
void USART1_Init(uint32_t Baudrate);                                       //USART1�������ó�ʼ�� ,Baudrate :������
ErrorStatus USART1_WriteDMASendMulData(uint8_t *Data , uint16_t Num);      //���ڷ�������     
ErrorStatus USART1_ReadDMAReceiveMulData(uint8_t *Data , uint16_t Num);    //���ڶ�ȡ����

//void USART1_Debug(void);                                                  //����ʹ�� �����ڽ���ʲô���ݾ͸����ڻ���Ŀ���ݣ�DMA��ʽ��





/***************************Ӳ����ʼ������*******************************/


/*************************************************************************
  * @brief  USART1��������ȫ����
  * @param  Baudrate �� ������
  * @retval ��
  * @notice �����ô��ڹ���ģʽ������I/O ����Ϊ����I/O�������ô���ʱ��
  *         ������I/O�����ô���֮��ʱI/O���Ż����һ��Ĭ�ϵĵ�ƽ���͵�ƽ���������˴��ڶ��һ������
**************************************************************************/ 
void USART1_Init(uint32_t Baudrate)
{
	/***************ʱ������*********************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2   ,ENABLE);                  //Enable DMA clock 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA  ,ENABLE);	                //���ô���GPIO����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 ,ENABLE);                  //���ô���USART1��ʱ��
	
	/***************�ڴ�����*********************/
	USART1_MemoryInit();                                                    //�ڴ滺������ʼ��
	
	/***************Ӳ������*********************/
  USART1_NVIC_Config();                                                   //����USART1�ж����ȼ� 
	USART1_Mode_Config(Baudrate);                                           //���ô���USART1����ģ ,����������
	USART1_DMA_Config();                                                    //���ô���USART1 DMA ģʽ
  USART1_GPIO_Config();                                                   //���ô���USART1���ݴ�������I/O��
}



/*************************************************************************                
  * @brief  ���ô���USART1���ݴ�������I/O��
  * @param  ��
  * @retval �� 
  * @notice TXD������� ��RXD�������� 
*************************************************************************/
static void USART1_GPIO_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		
		//����1��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource9 ,GPIO_AF_USART1);              //GPIOA9����ΪUSART1
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);              //GPIOA10����ΪUSART1
	
	  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
	 	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_9 | GPIO_Pin_10;	          //TXD :PA9 ;  //RXD :PA10
		GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;                        //���÷�ʽ
		GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;                       //������� 
		GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_2MHz;                     //������������Ϊ2MHz 
		GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_UP;                       //����
		GPIO_Init(GPIOA, &GPIO_InitStructure);                                //���ÿ⺯������ʼ��GPIOH5
}




/*************************************************************************
  * @brief  ���ô���USART1����ģʽ
  * @param  Baudrate �� ������ 
  * @retval �� 
  * @notice ��
*************************************************************************/
static void USART1_Mode_Config(uint32_t Baudrate)
{
	USART_InitTypeDef USART_InitStructure;
  
	/* USART1 mode config */	
	USART_InitStructure.USART_BaudRate   = Baudrate ;                       //���ڲ����ʣ�USART1_BAUDRATE
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;             //֡���ݳ��ȣ�8λ
	USART_InitStructure.USART_StopBits   = USART_StopBits_1;                //ֹͣλλ����1λ
	USART_InitStructure.USART_Parity     = USART_Parity_No ;                //��żУ�� ����У��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ����
	USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;   //���գ�����ʹ��
	USART_Init(USART1, &USART_InitStructure);                               //����
	USART_ITConfig(USART1, USART_IT_IDLE , ENABLE );                        //�������߿����ж�ʹ��
	USART_ITConfig(USART1, USART_IT_TXE  , DISABLE);                        //���ͻ��������жϹر�
	USART_ITConfig(USART1, USART_IT_TC   , DISABLE);                        //��������жϹر�
	USART_Cmd(USART1, ENABLE);                                              //����USART1ʹ�� 
}




/*************************************************************************
  * @brief  ���ô���USART1�ж����ȼ�
  * @param  ��
  * @retval �� 
  * @notice ���ȼ� �� #include "USART1.h" ���� define �궨��
*************************************************************************/
static void USART1_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	/* USART1_RX :USART1 RX ILDE configuration */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 	                    // ָ�� USART1 ���տ����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_IRQPreemptionPrio ;//��ռʽ���ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART1_IRQSubPrio;      //�����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                         //�ж�ʹ��
	NVIC_Init(&NVIC_InitStructure);                                         //���üĴ���
	
	/* USART1_RX :DMA1 channel5 configuration */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;	 	              // ָ�� USART1 ����DMA����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMA2_Stream5_IRQPreemptionPrio  ;//��ռʽ���ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = DMA2_Stream5_IRQSubPrio ;               //�����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                         //�ж�ʹ��
	NVIC_Init(&NVIC_InitStructure);                                         //���üĴ���

	/* USART1_TX :DMA1 channel4 configuration */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;                 // ָ������USART1 dma��������ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMA2_Stream7_IRQPreemptionPrio ;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = DMA2_Stream7_IRQSubPrio;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                         //�ж�ʹ��
  NVIC_Init(&NVIC_InitStructure);                                         //���üĴ���
	

} 



/*************************************************************************
  * @brief  ���ô���USART1��DMAģʽ 
  * @param  ��
  * @retval �� 
  * @notice ��ʹ�ܷ���DMA,��������Ҫ����ʱʹ��DMA
*************************************************************************/
static void USART1_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	  
  /* USART1_TX :DMA2 Stream7 channel4 configuration */
  DMA_InitStructure.DMA_Channel            = DMA_Channel_4 ;              //ͨ�� 
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral ; //���򣺴��ڴ浽����
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)USART1_SendData;	  //DMAԴ��ַ ���ڴ��ַ
  DMA_InitStructure.DMA_BufferSize         = USART1_SendSizeMax;          //DMA�����������
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable ;       //ֱ�Ӵ��� ����ʹ��FIFO
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO ��բ��
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single ;     //����
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;     //�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        //�ڴ��ַ��1
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;				  	  //���δ���
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR ;      //DMAĿ�ĵ�ַ   : ����1�����ݷ��͵�ַ
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;  //����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݳ���:8λ
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   //�����ַ�̶�
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;           //DMA���ȼ�
  DMA_Init(DMA2_Stream7,&DMA_InitStructure);                              //����
	
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);                            //DMA2 Stream4 ���ݴ�������ж�ʹ��
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);                          //USART1 ����DMAʹ��                
	DMA_Cmd(DMA2_Stream7, DISABLE);                                         //��ʹ��DMA,��������Ҫ����ʱʹ��DMA



	/* USART1_RX :DMA2 Stream5 channel4 configuration */
  DMA_InitStructure.DMA_Channel         = DMA_Channel_4 ;                //ͨ�� 
  DMA_InitStructure.DMA_DIR             = DMA_DIR_PeripheralToMemory ;   //���򣺴����赽�ڴ�
  DMA_InitStructure.DMA_Mode            = DMA_Mode_Circular;					   //ѭ������
	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)USART1_ReceiveData;   //DMAĿ�ĵ�ַ ���ڴ��ַ
  DMA_Init(DMA2_Stream5,&DMA_InitStructure);                             //����
 
	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC, ENABLE);                          //DMA2 Stream5 ���ݴ�������жϴ�
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);                         //ʹ�� USART1 ���ݽ���DMAʹ��  
	DMA_Cmd(DMA2_Stream5, ENABLE);                                         //ʹ�ܽ���DMA,�ȴ����ݵĵ���

}





/***************************�жϷ�����*********************************/


/*************************************************************************
  * @brief  USART1 �жϷ�����(����DMA���ݽ���ʱ�����ڿ��м��)
  * @param  ��
  * @retval ��
  * @notice �����õ������߿����ж�  �� һ���ֽ�ʱ��Ŀ���ʱ������ж� ��
  *         ������߿����жϱ�־λ  ֻ��ͨ���ȶ�USART_SR ,�ٶ�USART_DR�������������
*************************************************************************/
void USART1_IRQHandler(void) 
{
	/******�����ж� ����¼�������ݽ��յ���λ��*****/
	if(USART_GetITStatus(USART1,USART_IT_IDLE)!= RESET)                     //�ж��Ƿ�Ϊ���߿����ж� ����ʾ���յ�һ������
	{	
		USART_ReceiveData(USART1);                                            //������߿����жϱ�־λ ( ֻ��ͨ���ȶ�USART_SR ,�ٶ�USART_DR�������������) 
		USART1_ReceiveFront = USART1_ReceiveSizeMax - DMA_GetCurrDataCounter(DMA2_Stream5);  //�������ڽ��յ����ݵ�ַ
	}
}






/*************************************************************************
  * @brief  DMA1_Channel4�жϷ�����(����DMA���ݷ��� ������������ʱ�����ж�)
  * @param  ��
  * @retval ��
  * @notice USART1_SendRear ��������ݿ�ʼ����ʱ���´η������ݵĵ�ַ
            USART1_SendNextStart��������ݷ�����ɽ�USART1_SendRear����´����ݷ��͵�ַ������
*************************************************************************/
void DMA2_Stream7_IRQHandler(void)
{
	/******һ�����ݷ����� �������´����ݵķ���****/
	if(DMA_GetITStatus(DMA2_Stream7 , DMA_IT_TCIF7)!= RESET)                //�ж��Ƿ�Ϊ��������ж�
  { 
    DMA_ClearITPendingBit(DMA2_Stream7 , DMA_IT_TCIF7);                   //���DMA�жϱ�־λ 
		DMA_Cmd(DMA2_Stream7,DISABLE);                                        //һ֡���ݷ������ ���ر�DMA���͵����ݴ���
	  USART1_SendNextStart = USART1_SendRear ;                              //���ڴ��͵�һ֡���ݷ�����ɣ�����һ�����ݷ��͵ĵ�ַ �� USART1_SendNextStart
	  USART1_SendBusyFlag  = 0 ;                                            //USART1 DMA ���ڲ�����æµ״̬ �����Է�����һ֡����
	  USART1_DataSendTrigger() ;                                            //����Ƿ�����Ҫ���͵����ݣ������������ٴη��� 
	}
}




/*************************************************************************
  * @brief  DMA1_Channel5�жϷ�����(����DMA���ݽ��� �����ظ�ģʽ�£�����ж�λ���ٴο�ʼ��������
  * @param  ��
  * @retval ��
  * @notice ��Ҫ��Ŀ��Ϊ ������жϱ�־λ ��ȷ�����յ�����һ���ֽڽ�������USART1_ReceiveData[0]��
*************************************************************************/
void DMA2_Stream5_IRQHandler(void)
{
		/******���жϱ�־���´����ݴ�ͷ����****/
	if(DMA_GetITStatus(DMA2_Stream5 , DMA_IT_TCIF5)!= RESET)                //�ж��Ƿ�Ϊ����DMA�Ƿ����
  { 
    DMA_ClearITPendingBit(DMA2_Stream5 , DMA_IT_TCIF5);                   //���DMA�жϱ�־λ �� ���ظ�ģʽ�� ����һ���ֽڽ��ٴδ�USART1_ReceiveData[0]��ʼ
	}
}









/***************************�Ի����ڴ�Ĳ���*******************************/

/*************************************************************************
  * @brief  USART1�����ڴ��ʼ��
  * @param  ��
  * @retval ��
  * @notice USART1_SendBusyFlag��ʼ����Ϊ 0 ����ʾ��ʼ�ܹ���������
*************************************************************************/
static void USART1_MemoryInit( void )
{
	USART1_SendBusyFlag = 0 ;                                               // һ��ʼ���ڿ��� �����Է�������
	USART1_SendFront    = 0 ;                                               // �������ݻ��������ͷ��ַ��ʼ��
	USART1_SendRear     = 0 ;                                               // �������ݻ��������β��ַ��ʼ��
	USART1_SendNextStart= 0 ;                                               // ��һ֡���ݷ��͵Ŀ�ʼ��ַ��ʼ��
	USART1_ReceiveFront = 0 ;                                               // �������ݻ��������ͷ��ַ��ʼ��
	USART1_ReceiveRear  = 0 ;                                               // �������ݻ��������β��ַ��ʼ��
}



/*************************************************************************
  * @brief  �������ݷ��ʹ��� ������DMA�Ƿ����ڷ��ͣ��� �ڴ����Ƿ�������������һ�����ݵķ���
  * @param  ��
  * @retval ��
  * @notice ���жϺ�����������ʹ��
*************************************************************************/
static void USART1_DataSendTrigger( void )
{
	uint16_t SendNum ;                                                      //��Ҫ���͵������� 
	uint16_t SendNextRear;                                                  //��һ�����ݴ���û�г���dmaһ������ܹ�������ֽ���ʱ����һ֡���ݵĿ�ʼλ��
	
	/******�ж��Ƿ���Ҫ����DMA�����ݷ��� **********/
	if(USART1_SendBusyFlag  != 0 ) return ;                                 //�����ϴ����ݻ�δ������� ��������æµ״̬ ��
	if( USART1_SendFront == USART1_SendRear ) return ;                      //����������Ϊ�� ��û��������Ҫ����
	
	/***����Ҫ���͵�����£�������Ҫ���͵�������***/
	if(USART1_SendFront > USART1_SendRear)                                  //��������ͷ������β��λ��ȷ�����ݷ��͵���
	{
		SendNum      =  USART1_SendFront -  USART1_SendRear ;                 //֡ͷ��֡β���һ���Է������ 	
		SendNextRear =  USART1_SendFront ;                                    //��һ�����ݵķ��ʹ�USART1_SendFront ��ʼ
	}
	else                                                                    //֡ͷ��֡βС�ķ��������ݷ��� ���ȷ���  USART1_SendRear ��  USART1_SendSizeMax ������
	{                                                                       //�ٷ��ʹ�0��ʼ��USART1_SendFront ������
		SendNum      = USART1_SendSizeMax -  USART1_SendRear ;                //�����һ����Ҫ���͵�������
	  SendNextRear = 0 ;                                                    //��һ�����ݵķ��ʹ� 0 ��ʼ
	}
	

	/**����һ��DMA�������������������ڴ��ͷ�**/
	if( SendNum > USART1_DMASendOneSizeMax )	                              //�Ƚ�������Ҫ������ֽ���������ֽ����Ĵ�С                           
	{
		SendNum = USART1_DMASendOneSizeMax ;                                  //��������һ��DMA���ݴ��� ��ʹ�����һ�����ݴ���
		USART1_SendRear = USART1_SendRear + USART1_DMASendOneSizeMax ;        //��һ�����ݵķ��ʹ�USART1_SendRear + USART1_DMASendOneSizeMax  ��ʼ
	}
	else
	{
		USART1_SendRear = SendNextRear ;                                      //��һ�����ݵķ��ʹ�SendNextRear��ʼ
	} 

	
	/******��λ����æµ��־λ ���������ݷ���******/
	USART1_SendBusyFlag  = 1 ;                                              //æµ״̬
	DMA_Cmd(DMA2_Stream7,DISABLE);                                          //DMA_CNDTR��ҪDMA_CCRx�е�EN=0ʱ����д�� 
  DMA2_Stream7->M0AR = (uint32_t)&USART1_SendData[USART1_SendNextStart];  //�������ݷ��͵���ʼ��ַ
  DMA2_Stream7->NDTR = SendNum;                                           //�������ݷ��͵������� ����λ ���ֽ�
	DMA_Cmd(DMA2_Stream7,ENABLE);                                           //ʹ��DMA���ݷ���

}




/*************************************************************************
  * @brief  �������ݷ��� �����ݴ����Ƿ�æµ�ٴ���DMA���ݵķ���
  * @param  Data  �� �跢���������׵�ַ
					  Num   �� �跢�͵�������
  * @retval ��
  * @notice if( Num >= IdleNum ) return ERROR ;���е��ںţ�����һ���ֽڲ��� ��
  *         �������������� �� ���ݿ�
*************************************************************************/
ErrorStatus USART1_WriteDMASendMulData(uint8_t *Data , uint16_t Num)
{
	uint16_t  i ,IdleNum ;                                                  //���������е��ڴ���                                               
	
	/******��������ڴ� ���ж��Ƿ񻹹��� ***********/
	if(USART1_SendFront >= USART1_SendNextStart )                           //����������㻹�ж����ڴ湻�� ��ע����ںš� = �� 
		IdleNum = USART1_SendSizeMax-USART1_SendFront + USART1_SendNextStart ;                   
	else
		IdleNum = USART1_SendNextStart - USART1_SendFront ;
	if( Num >= IdleNum ) return ERROR ;                                     //�������ڴ治�� ������д���������һ���ֽڲ��ã�ע����ںš�=��
	
	/*****���ڴ湻�õ����������д�뻺���ڴ���******/
	for( i = 0 ; i < Num ; i++ )                                            //д���ڴ����
	{
		USART1_SendData[USART1_SendFront] = *Data++ ;                         //������д�뻺���ڴ���
		if(USART1_SendFront == USART1_SendSizeMax - 1)                        //�Ƿ�д���ڴ���β��
			USART1_SendFront  = 0 ;                                             //д���ڴ������һ���ֽ� ����һ���ֽڴ�0���¿�ʼ
		else
			USART1_SendFront++ ;                                                //�������������һ���ֽڵı���λ��
	}
	
	/******************�������ݵķ���***************/
	USART1_DataSendTrigger();                                               //�������ݵķ��� 
	return SUCCESS;                                                         //���ݴ���ɹ� 
}






/*************************************************************************
  * @brief  ���ڶ�ȡһ֡���� 
  * @param  Data  ����ȡ���ݱ���λ�� 
  *         Num   ����ȡ����֡���ݵĳ���
  * @retval ��
  * @notice USART1_ReceiveFinishFlag ��ʾ һ֡���ݶ�ȡ��� ��û�д���������� 
*************************************************************************/
ErrorStatus USART1_ReadDMAReceiveMulData(uint8_t *Data , uint16_t Num)
{
	uint16_t  i , num ;                                                     //���ջ���������Ч�ֽ���
	
	/******�жϻ������Ƿ���Num������******/
	if( USART1_ReceiveFront >= USART1_ReceiveRear)                          //�������������յ����ֽ���
		num = USART1_ReceiveFront - USART1_ReceiveRear ;                      
	else                                                                    
		num = USART1_ReceiveSizeMax - USART1_ReceiveRear + USART1_ReceiveFront ; 
	if( num < Num ) return ERROR ;                                          //���ݲ��� �����ض�ȡʧ�� ��ERROR
	
	/******�����ݹ�������¶�ȡ����******/
	for( i = 0 ; i < Num ; i++ )                                            //�������ݼ���
	{
			*Data++ = USART1_ReceiveData[USART1_ReceiveRear];                   //�ӽ��ջ����ڴ��ȡ���ݱ��浽Data��
	    if(USART1_ReceiveRear == USART1_ReceiveSizeMax - 1 )                //�Ƿ��ȡ���ڴ���β��
				USART1_ReceiveRear = 0 ;                                          //��ȡ���ڴ������һ���ֽ� ����һ���ֽڴ�0���¿�ʼ
			else                          
				USART1_ReceiveRear++ ;                                            //��������¼����ȡ����һ���ֽ�λ��
	}
	return SUCCESS;                                                         //���ݶ�ȡ�ɹ�
}







///************************************************************************
//  * @brief  ����ʹ�� �����ڽ���ʲô���ݾ͸����ڻ�ʲô���ݣ�DMA��ʽ��
//  * @param  ��
//  * @retval ��
//************************************************************************/
//void USART1_Debug(void)
//{
//	uint8_t USART1_Dat[16];
//	uint8_t USART1_Num = 16;

//	if(USART1_ReadDMAReceiveMulData(USART1_Dat , USART1_Num ) == SUCCESS)            //��ȡ���յ�һ֡����
//		USART1_WriteDMASendMulData(USART1_Dat , USART1_Num ) ;                         //���ͽ��ܵ���һ֡����
//}


/*********************************************END OF FILE**********************/
