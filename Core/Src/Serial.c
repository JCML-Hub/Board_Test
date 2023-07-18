/**
  ************************************* Copyright ****************************** 
  *

  *                 (C) Copyright 2023,--,China, CUIT.
  *                            All Rights Reserved
  *                              
  *                     By(JCML)
    *
  *    
  * FileName   : Serial.c   
  * Version    : v1.0		
  * Author     : JCML			
  * Date       : 2023-01-17         
  * Description: 基于Hal库的串口收发定义函数  
					PS在使用该函数前请到stm32f1xx_it.h中将
									void USART1_IRQHandler(void)
									void DMA1_Channel5_IRQHandler(void)
							两个函数注释掉
******************************************************************************
 */
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "Serial.h"
#include "stm32f1xx_it.h"


uint8_t USART1_TX_BUF[MAX_TX_LEN];   // my_printf的发送缓冲，下文详述其作用。
volatile uint8_t USART1_TX_FLAG = 0; // USART发送标志，启动发送时置1，加volatile防编译器优化
uint8_t u1rxbuf[MAX_RX_LEN];         // 数据接收缓冲1
uint8_t u2rxbuf[MAX_RX_LEN];         // 数据接收缓冲2
uint8_t WhichBufIsReady = 0;         // 双缓存指示器。
// 0:u1rxbuf 被DMA占用接收,  u2rxbuf 可以读取.
// 0:u2rxbuf 被DMA占用接收,  u1rxbuf 可以读取.
uint8_t *RxData = u2rxbuf;        // 指针——指向可以读取的那个缓冲
uint8_t *p_IsToReceive = u1rxbuf; // 指针——指向被占用的那个缓冲
char RxDataStr[MAX_RX_LEN];
//注意定义的时候要先让这两个指针按照WhichBufIsReady的初始状态先初始化一下。
uint8_t Rxflag;
uint32_t data_length;


/***********************************************************
*@fuction	:USART1_Init
*@brief		:串口初始化
*@param		:void
*@return	:void
*@author	:JCML
*@date		:2023-01-17
***********************************************************/

void USART1_Init(void)
{
  // 开启串口1空闲中断
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  // 开启DMA发送通道的发送完成中断，才能实现封装发送函数里面的等待功能
  __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);

  // 清除空闲标志位，防止中断误入
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);

  // 立即就要打开DMA接收
  // 不然DMA没有提前准备，第一次接收的数据是读取不出来的
  HAL_UART_Receive_DMA(&huart1, p_IsToReceive, MAX_RX_LEN);
}

/***********************************************************
*@fuction	:StringTrans
*@brief		:将字符数组转化为字符串也可在该函数中添加if条件构成特定的通信方式
*@param		:void
*@return	:void
*@author	:JCML
*@date		:2023-01-17
***********************************************************/

void StringTrans(void)
{
	uint32_t i=0;
	while (i < data_length)
	{
		if (RxData[i] == '#' ||RxData[i] == 0)
		{
			break;
		}
		else {RxDataStr[i]=RxData[i];}
		i++;
	}
	//不建议采用以下方式
//	for (i=0; i < data_length; i++)//
//	{
//		if (u1rxbuf[i] == '\r' ||u1rxbuf[i] == '\n')
//		RxDataStr[i]=u1rxbuf[i];
//	}
	RxDataStr[data_length]='\0';
}



/***********************************************************
*@名称 	:GetRxValue
*@描述	:转换获取从串口中接收数字 输出为float型
*@参数	:Str指向目标字符串的指针， lenth字符串的长度，大于字符串长度即可
*@返回值	:提取出的值
*@作者	:JCML
*@日期	:2023-04-20
***********************************************************/

float GetRxValue(char *Str, uint8_t lenth)
{
	uint8_t i, j;
	for (i = 0; i<lenth; i++)
	{//判读当出现负号或者数字时将RxDataStr中的值前移
		if ((Str[i]=='-'&&Str[i+1]<='9'&&Str[i+1]>='0')||
															(Str[i]<='9'&&Str[i]>='0'))
		{
			for(j=0;Str[j]!='\0';j++)
			{
				Str[j] = Str[j+i];
			}
			Str[j]='\0';
			break;
		}
	}
	return atof(Str);//
}


/***********************************************************
*@fuction	:DMA_USART1_Tx_Data
*@brief		:
*@param		:void
*@return	:void
*@author	:JCML
*@date		:2023-01-17
***********************************************************/

void DMA_USART1_Tx_Data(uint8_t *buffer, uint16_t size)
{
  USART1_TX_Wait();                             // 等待上一次发送完成（USART1_TX_FLAG为1即还在发送数据）
  USART1_TX_FLAG = 1;                           // USART1发送标志（启动发送）
  HAL_UART_Transmit_DMA(&huart1, buffer, size); // 发送指定长度的数据
}
/***********************************************************
*@fuction	:Myprintf
*@brief		:printf重定义
*@param		:void
*@return	:void
*@author	:JCML
*@date		:2023-01-17
***********************************************************/

void Myprintf(char *format, ...)
{
  //VA_LIST 是在C语言中解决变参问题的一组宏，
  //所在头文件：#include <stdarg.h>，用于获取不确定个数的参数。
  va_list arg_ptr;//实例化可变长参数列表

  USART1_TX_Wait(); //等待上一次发送完成（USART1_TX_FLAG为1即还在发送数据）

  va_start(arg_ptr, format);//初始化可变参数列表，设置format为可变长列表的起始点（第一个元素）

  // MAX_TX_LEN+1可接受的最大字符数(非字节数，UNICODE一个字符两个字节), 防止产生数组越界
  vsnprintf((char *)USART1_TX_BUF, MAX_TX_LEN + 1, format, arg_ptr);
  //从USART1_TX_BUF的首地址开始拼合，拼合format内容；MAX_TX_LEN+1限制长度，防止产生数组越界

  va_end(arg_ptr); //注意必须关闭

  DMA_USART1_Tx_Data(USART1_TX_BUF, strlen((const char *)USART1_TX_BUF)); 
  // 记得把buf里面的东西用HAL发出去 
}

void USART1_TX_Wait(void)
{
  uint16_t delay = 20000;
  while (USART1_TX_FLAG)
  {
    delay--;
    if (delay == 0)
      return;
  }
}

/***********************************************************
*@fuction	:GetRxFlag
*@brief		:返回接收数据的标志同时将其转换为字符串
*@param		:void
*@return	:void
*@author	:JCML
*@date		:2023-01-17
***********************************************************/

uint8_t GetRxFlag(void)
{
	if (Rxflag==1){StringTrans();Rxflag=0;return 1;}
	else {return 0;}
}


/***********************************************************
*@fuction	:DMA1_Channel5_IRQHandler
*@brief		:DMA中断处理
*@param		:void
*@return	:void
*@author	:JCML
*@date		:2023-01-17
***********************************************************/

void USART1_TX_DMACallBack(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */
  if (__HAL_DMA_GET_FLAG(&hdma_usart1_tx, DMA_FLAG_TC4) != RESET) //数据发送完成中断
  {
    // __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_FLAG_TC4);
    // 这一部分其实在 HAL_DMA_IRQHandler(&hdma_usart1_tx) 也完成了。

    __HAL_UART_CLEAR_IDLEFLAG(&huart1); //清除串口空闲中断标志位，发送完成那么串口也是空闲态

    USART1_TX_FLAG = 0; // 重置发送标志位

    huart1.gState = HAL_UART_STATE_READY;
    hdma_usart1_tx.State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(&hdma_usart1_tx);
    // 这里疑似是HAL库函数的bug，具体可以参考我给的链接
    // huart1,hdma_usart1_tx 的状态要手动复位成READY状态
    // 不然发送函数会一直以为通道忙，就不再发送数据了！
  }
}


/***********************************************************
*@fuction	:USART1_IRQHandler
*@brief		:串口接收数据中断处理
*@param		:void
*@return	:void
*@author	:JCML
*@date		:2023-01-17
***********************************************************/

void USART1_RX_CallBack(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
  { 
	  Rxflag = 1;	
    // __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    // 这一部分其实在 HAL_UART_IRQHandler(&huart1) 也完成了。

    HAL_UART_DMAStop(&huart1); // 把DMA接收停掉，防止速度过快导致中断重入，数据被覆写。
	
    data_length = MAX_RX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    // 数据总长度=极限接收长度-DMA剩余的接收长度
    if (WhichBufIsReady)	//WhichBufIsReady=1
    {
      RxData = u2rxbuf;        // u2rxbuf 可以读取，就绪指针指向它。
      p_IsToReceive = u1rxbuf; // u1rxbuf 作为下一次DMA存储的缓冲，占用指针指向它。
      WhichBufIsReady = 0;		//切换一下指示器状态
    }
    else				//WhichBufIsReady=0
    {
      RxData = u1rxbuf;        // u1rxbuf 可以读取，就绪指针指向它。
      p_IsToReceive = u2rxbuf; // u2rxbuf 作为下一次DMA存储的缓冲，占用指针指向它。
      WhichBufIsReady = 1;		//切换一下指示器状态
    }
    //从下面开始可以处理你接收到的数据啦！举个栗子，把你收到的数据原原本本的还回去
      
    //DMA_USART1_Tx_Data(RxData,data_length);//数据打回去，长度就是数据长度
      
    memset((uint8_t *)p_IsToReceive, 0, MAX_RX_LEN);	// 把接收数据的指针指向的缓冲区清空
  }
  HAL_UART_Receive_DMA(&huart1, p_IsToReceive, MAX_RX_LEN); //数据处理完毕，重新启动接收
}





