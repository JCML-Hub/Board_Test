#ifndef __ST_I2C_H
#define __ST_I2C_H
#include "stm32f1xx_hal.h"
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#ifdef __cplusplus
 extern "C" {
#endif
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

/**
            * @name 需要自行更改的IIC_GPIO定义
            * @{
            */
#define SCL_H         HAL_GPIO_WritePin(IIC_SCL_GPIO_Port,IIC_SCL_Pin,GPIO_PIN_SET)
#define SCL_L         HAL_GPIO_WritePin(IIC_SCL_GPIO_Port,IIC_SCL_Pin,GPIO_PIN_RESET)

#define SDA_H         HAL_GPIO_WritePin(IIC_SDA_GPIO_Port,IIC_SDA_Pin,GPIO_PIN_SET)
#define SDA_L         HAL_GPIO_WritePin(IIC_SDA_GPIO_Port,IIC_SDA_Pin,GPIO_PIN_RESET)
#define I2C_X_GPIO       	GPIOB
#define	I2C_PIN_X0       	IIC1_Pin
#define I2C_X0_HIGH()       HAL_GPIO_WritePin(I2C_X_GPIO,I2C_PIN_X0,GPIO_PIN_SET) 
#define I2C_X0_LOW()  	    HAL_GPIO_WritePin(I2C_X_GPIO,I2C_PIN_X0,GPIO_PIN_RESET)

#define	I2C_PIN_X1       	IIC2_Pin
#define I2C_X1_HIGH()       HAL_GPIO_WritePin(I2C_X_GPIO,I2C_PIN_X1,GPIO_PIN_SET) 
#define I2C_X1_LOW()  	    HAL_GPIO_WritePin(I2C_X_GPIO,I2C_PIN_X1,GPIO_PIN_RESET)

#define	I2C_PIN_X2       	IIC3_Pin
#define I2C_X2_HIGH()       HAL_GPIO_WritePin(I2C_X_GPIO,I2C_PIN_X2,GPIO_PIN_SET) 
#define I2C_X2_LOW()  	    HAL_GPIO_WritePin(I2C_X_GPIO,I2C_PIN_X2,GPIO_PIN_RESET)

#define	I2C_PIN_X3       	IIC4_Pin
#define I2C_X3_HIGH()       HAL_GPIO_WritePin(I2C_X_GPIO,I2C_PIN_X3,GPIO_PIN_SET) 
#define I2C_X3_LOW()  	    HAL_GPIO_WritePin(I2C_X_GPIO,I2C_PIN_X3,GPIO_PIN_RESET)

#define SCL_read      HAL_GPIO_ReadPin(IIC_SCL_GPIO_Port,IIC_SCL_Pin)
#define SDA_read      HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port,IIC_SDA_Pin)
            /** @} */
extern void ST_IIC_Init(void);                //<初始化IIC的IO口


extern unsigned char ST_IIC_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
extern unsigned char ST_IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
extern uint8_t ST_IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
extern uint8_t ST_IICwriteBit(uint8_t dev,uint8_t reg,uint8_t bitNum,uint8_t data);
extern uint8_t ST_IICreadBytes(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t len,uint8_t *data);
#ifdef __cplusplus
}
#endif
#endif

//------------------End of File----------------------------
