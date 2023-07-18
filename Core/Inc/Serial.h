#ifndef __SERIAL_H__
#define __SERIAL_H__

#define MAX_RX_LEN (256U) // 一次性可以接受的数据字节长度，你可以自己定义。U是Unsigned的意思。
#define MAX_TX_LEN (512U) // 一次性可以发送的数据字节长度，你可以自己定义。
#include "stdio.h"
#include "string.h"
#include <stdarg.h>

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern uint8_t *RxData;
extern uint8_t *p_IsToReceive;
extern uint8_t WhichBufIsReady;
extern char RxDataStr[MAX_RX_LEN];


void USART1_Init(void);
void DMA_USART1_Tx_Data(uint8_t *buffer, uint16_t size);//数组发送串口数据
void Myprintf(char *format, ...);//仿制printf发送串口数据
void USART1_TX_Wait(void);//发送等待函数
void USART1_RX_CallBack(void);
void USART1_TX_DMACallBack(void);
uint8_t GetRxFlag(void);
void StringTrans(void);
float GetRxValue(char *Str, uint8_t lenth);
#endif