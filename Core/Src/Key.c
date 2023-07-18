/**
  ************************************* Copyright ******************************
  *                 (C) Copyright 2023,--,China, CUIT.
  *                            All Rights Reserved
  *                     By(JCML)

  * FileName   : Key.c
  * Version    : v1.0
  * Author     : JCML
  * Date       : 2023-07-01
  * Description: ��PB14��PB15�ֱ�Ϊ����1�Ͱ���2�İ�����ȡ����
******************************************************************************
 */

#include "main.h"
#include "Key.h"

/***********************************************************
*@fuction	:Key_GetNum();
*@brief		:��ȡ�����µļ���
*@param		:void
*@return	:KeyNum
*@author	:JCML
*@date		:2023-07-01
***********************************************************/

uint8_t GetKeyNum(void)
{
  uint8_t KeyNum = 0;
  if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)==0)
    KeyNum=1;
  if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)==0)
    KeyNum=2;
  return KeyNum;
}