/** 
* @brief        移植用的适配STM32的HAL库的驱动文件 非官方 自己适配的
* @author      WMD
* @date     2018年4月22日21:20:32
* @version  
* @par Copyright (c):  
*       WMD 
* @par 日志
*/  
//#include "delay.h"
#include "ST_I2C.h"
#include "vl53l0x_api.h"
//#include "cmsis_os.h"

/**
            * @name 自定义移植接口函数
            * @{
            */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	return ST_IICwriteBytes(Dev->I2cDevAddr,index,count,pdata);
}
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	return ST_IICreadBytes(Dev->I2cDevAddr,index,count,pdata);
}
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
	return ST_IICwriteByte(Dev->I2cDevAddr,index,data);
}
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
	return VL53L0X_ReadMulti(Dev,index,data,1);
}
///注意以下这些函数有可能会导致大小端不对
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
	uint8_t tmp[2]={	((uint16_t)(data&0xff00)>>8) ,	((uint16_t)data&0x00ff)	};
	return ST_IICwriteBytes(Dev->I2cDevAddr,index,2,tmp);
}
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
	uint8_t tmp[4]={	((data&0xff000000)>>24),((data&0x00ff0000)>>16),((data&0x0000ff00)>>8),((data&0x000000ff))};
	return ST_IICwriteBytes(Dev->I2cDevAddr,index,4,tmp);
}
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
	uint8_t tmp[2];
	uint8_t* p=(uint8_t*)data;
	ST_IICreadBytes(Dev->I2cDevAddr,index,2,tmp);
	p[0]=tmp[1];
	p[1]=tmp[0];
	return 0;
}
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
	uint8_t tmp[4];
	uint8_t* p=(uint8_t*)data;
	ST_IICreadBytes(Dev->I2cDevAddr,index,4,tmp);
	p[0]=tmp[3];
	p[1]=tmp[2];
	p[2]=tmp[1];
	p[3]=tmp[0];
	return 0;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
	uint8_t tmp;
	ST_IICreadBytes(Dev->I2cDevAddr,index,1,&tmp);
	tmp=(tmp & AndData) | OrData;
	return ST_IICwriteByte(Dev->I2cDevAddr,index,tmp);
}
//!该函数是等待用函数 要改成对应环境的Delay
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    HAL_Delay(5);
	return 0;
}
            /** @} */
//接下来是要引出的简化API函数
//!激光传感器的结构体
static VL53L0X_Dev_t TestDev_s0;
static VL53L0X_DEV TestDev0=&TestDev_s0;
static VL53L0X_RangingMeasurementData_t TestData_s0;
static VL53L0X_RangingMeasurementData_t* TestData0=&TestData_s0;

static VL53L0X_Dev_t TestDev_s1;
static VL53L0X_DEV TestDev1=&TestDev_s1;
static VL53L0X_RangingMeasurementData_t TestData_s1;
static VL53L0X_RangingMeasurementData_t* TestData1=&TestData_s1;

static VL53L0X_Dev_t TestDev_s2;
static VL53L0X_DEV TestDev2=&TestDev_s2;
static VL53L0X_RangingMeasurementData_t TestData_s2;
static VL53L0X_RangingMeasurementData_t* TestData2=&TestData_s2;

static VL53L0X_Dev_t TestDev_s3;
static VL53L0X_DEV TestDev3=&TestDev_s3;
static VL53L0X_RangingMeasurementData_t TestData_s3;
static VL53L0X_RangingMeasurementData_t* TestData3=&TestData_s3;

//static VL53L0X_Dev_t TestDev_s4;
//static VL53L0X_DEV TestDev4=&TestDev_s4;
//static VL53L0X_RangingMeasurementData_t TestData_s4;
//static VL53L0X_RangingMeasurementData_t* TestData4=&TestData_s4;
/** 
* @brief  激光传感器初始化 
* @param void
* @retval  void
* @par 日志 
*
*/
void VL53L0X_Init(VL53L0X_Dev_t *pMyDevice ,unsigned char vl53l0_x_id)
{
	
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;   //初始值赋值为0
        pMyDevice->I2cDevAddr      = 0x52;        //iic地址  0x52是默认地址,要初始化必须先写0x52,才能初始化，之后再通过软件修改
//    pMyDevice->comms_type      =  1;              //选择IIC还是SPI    iic=1；SPI=0
//    pMyDevice->comms_speed_khz =  400;            //iic速率   

    switch(vl53l0_x_id)
      {
        case 0:  
           I2C_X0_HIGH();  
           VL53L0X_SetDeviceAddress(pMyDevice,0x52);//设置第一个VL53L0X传感器I2C地址
		   pMyDevice->I2cDevAddr      = 0x52;

           break;
        case 1:               
           I2C_X1_HIGH();
           VL53L0X_SetDeviceAddress(pMyDevice,0x54);//设置第二个VL53L0X传感器I2C地址
		   pMyDevice->I2cDevAddr      = 0x54;
           break;
        case 2:               
           I2C_X2_HIGH();
           VL53L0X_SetDeviceAddress(pMyDevice,0x56);//设置第一个VL53L0X传感器I2C地址
		   pMyDevice->I2cDevAddr      = 0x56;
           break;	
        case 3:               
           I2C_X3_HIGH();
           VL53L0X_SetDeviceAddress(pMyDevice,0x58);//设置第一个VL53L0X传感器I2C地址
		   pMyDevice->I2cDevAddr      = 0x58;
           break;					
	}
//	pMyDevice->I2cDevAddr=0x52;
//	delay_ms(20);
//	I2C_X0_HIGH();
	
//	VL53L0X_SetDeviceAddress(pMyDevice,0x54);//设置地址
	VL53L0X_SetPowerMode(pMyDevice,VL53L0X_POWERMODE_STANDBY_LEVEL2);//设定最不省电模式
	VL53L0X_SetDeviceMode(pMyDevice,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);//设定连续读取模式
	VL53L0X_SetInterMeasurementPeriodMilliSeconds(pMyDevice,10);//设定采样时间
	VL53L0X_DataInit(pMyDevice);
	
}

VL53L0X_Error vl53l0x_init(void)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	I2C_X0_LOW();
	I2C_X1_LOW();
	I2C_X2_LOW();
//	I2C_X3_LOW();
	
//	I2C_X0_HIGH();
//	TestDev0->I2cDevAddr      = 0x52; 	
//	VL53L0X_SetDeviceAddress(TestDev0,0x54);
//	TestDev0->I2cDevAddr      = 0x54;
//	VL53L0X_SetPowerMode(TestDev0,VL53L0X_POWERMODE_STANDBY_LEVEL2);//设定最不省电模式
//	VL53L0X_SetDeviceMode(TestDev0,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);//设定连续读取模式
//	VL53L0X_SetInterMeasurementPeriodMilliSeconds(TestDev0,10);//设定采样时间
//	VL53L0X_DataInit(TestDev0);
//	
	
//	I2C_X1_HIGH();
//	TestDev1->I2cDevAddr      = 0x52; 
//	VL53L0X_SetDeviceAddress(TestDev1,0x58);
//	TestDev1->I2cDevAddr      = 0x58;
//	VL53L0X_SetPowerMode(TestDev1,VL53L0X_POWERMODE_STANDBY_LEVEL2);//设定最不省电模式
//	VL53L0X_SetDeviceMode(TestDev1,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);//设定连续读取模式
//	VL53L0X_SetInterMeasurementPeriodMilliSeconds(TestDev1,10);//设定采样时间
//	VL53L0X_DataInit(TestDev1);
//    VL53L0X_Init(TestDev2,2); 
	VL53L0X_Init(TestDev0,0); 
	VL53L0X_Init(TestDev1,1);
	VL53L0X_Init(TestDev2,2);
//	VL53L0X_Init(TestDev3,3);

   	
    return Status;           //返回0
}
/** 
* @brief  获得激光传感器的距离
* @param void
* @retval  uint16_t 距离(单位mm) 
* @note 如果不在任务中使用的话记得修改延时函数
* @par 日志 
*
*/
uint16_t VL53L0X_GetValue(int ch)
{
	int value = 0;
	
	if (ch == 0)
	{
		VL53L0X_PerformSingleMeasurement(TestDev0);//简单测量
		VL53L0X_GetRangingMeasurementData(TestDev0,TestData0);
		value = TestData0->RangeMilliMeter;
	}
	else if (ch == 1)
	{
		VL53L0X_PerformSingleMeasurement(TestDev1);//简单测量
		VL53L0X_GetRangingMeasurementData(TestDev1,TestData1);
		value = TestData1->RangeMilliMeter;
	}
	else if (ch == 2)
	{
		VL53L0X_PerformSingleMeasurement(TestDev2);//简单测量
		VL53L0X_GetRangingMeasurementData(TestDev2,TestData2);
		value = TestData2->RangeMilliMeter;
	}
	else if (ch == 3)
	{
		VL53L0X_PerformSingleMeasurement(TestDev3);//简单测量
		VL53L0X_GetRangingMeasurementData(TestDev3,TestData3);
		value = TestData3->RangeMilliMeter;
	}
//	else if (ch == 4)
//	{
//		VL53L0X_PerformSingleMeasurement(TestDev4);//简单测量
//		VL53L0X_GetRangingMeasurementData(TestDev4,TestData4);
//		value = TestData4->RangeMilliMeter;
//	}
	
	return value;
}

