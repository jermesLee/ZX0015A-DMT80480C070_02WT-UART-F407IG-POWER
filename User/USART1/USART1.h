#ifndef _USART1_H_
#define	_USART1_H_

#include "includes.h"

/************************USART1  �ڴ�����С�궨��**********************/
#define USART1_SendSizeMax                 256                            //���ڷ���һ�η��͵�����ֽ���(DMA��ʽ)
#define USART1_ReceiveSizeMax              256                            //���ڽ���һ���Խ��յ�����ֽ�����DMA��ʽ��
#define USART1_DMASendOneSizeMax            32                            //DMAһ���������͵�����ֽ��� �����ڷ��͵�һ֡�������ڴ������ܱ����ģ�������д���� �� �������Ѿ������˵������ڴ��޷��õ��ͷ� ��
                                                                          //Ϊ��������⣬����һ�η��͵�����ֽ�������һ֡���ݱȽϳ�ʱ��Ӱ���ر����������һ��һ��DMA�ܹ���������һ�����ݣ�������ͨ��ʱ�ڴ���ͷ�
/************************USART1  DMA���ͽ����ж����ȼ�*******************/
#define USART1_IRQPreemptionPrio            2                             //USART1��ռʽ���ȼ� ( ���ڽ��� ) ,������ռʽ���ȼ���DMA1_Channel5_IRQPreemptionPrio�� 
#define USART1_IRQSubPrio                   2                             //USART1�����ȼ�

#define DMA2_Stream7_IRQPreemptionPrio      1                             //USART1 DMA2_Stream7 ������ռʽ���ȼ� �� ���ڷ��� ��
#define DMA2_Stream7_IRQSubPrio             2                             //USART1 DMA2_Stream7 ��������ȼ�                   

#define DMA2_Stream5_IRQPreemptionPrio      1                             //USART1 DMA2_Stream5 ������ռʽ���ȼ� �� ���ڽ��� ����������ռʽ���ȼ���USART1_IRQPreemptionPrio ��
#define DMA2_Stream5_IRQSubPrio             1                             //USART1 DMA2_Stream5 ��������ȼ�                   




/**************************USART1  �û�����*******************************/
extern void USART1_Init(uint32_t Baudrate);                               //USART1���� �� Baudrate ��������
extern ErrorStatus USART1_WriteDMASendMulData(uint8_t *Data   , uint16_t  Num);  //���ڷ�������  ��Num ����Ҫ���͵������� 
extern ErrorStatus USART1_ReadDMAReceiveMulData(uint8_t *Data , uint16_t  Num);  //���ڶ�ȡ����  ��Num �����ؽ��յ���������

//extern void USART1_Debug(void) ;


#endif /* __USART1_H */
