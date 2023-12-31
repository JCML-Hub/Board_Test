/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Serial.h"
#include "Key.h"
#include "OLED.h"
#include "Motor.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "vl53l0x_api.h"
#include "PID.h"
#include "Control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int speedLeft,speedRight;
PID pid;
uint64_t status;
int distanceLeft, distanceRight, distanceFront;
VL53L0X_Dev_t dev1,dev2,dev3;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

 VL53L0X_Error vl53l0x_init(void);
 uint16_t VL53L0X_GetValue(int ch);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
short gyrox, gyroy, gyroz;//陀螺仪原始数据
float pitch,roll,yaw;
void Updata(void)
{
  mpu_dmp_get_data(&pitch, &roll, &yaw);
//	MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
  MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
//	BalanG = KalmanFilter(&kfp,-gyroy);
  Myprintf("%f,%f,%f\r\n", pitch,roll, yaw);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE BEGIN 1 */
  uint16_t pwmVal=0;   //PWM占空比
  uint8_t dir=1;
  _Motor motor0;
  short cnt=0;
  int temp,temp2,temp3;
  uint32_t count1=0,count2=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  USART1_Init();
  Motor_Init();
  setup();

  vl53l0x_init();
//  MPU_Init();
//  mpu_dmp_init();		//dmp初始化
  OLED_ShowString(0,0,"Test!",16);
//  Myprintf("Hello!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    Updata();
    temp=VL53L0X_GetValue(0);
    temp2=VL53L0X_GetValue(1);
    temp3=VL53L0X_GetValue(2);
    HAL_Delay(50);
    OLED_ShowNum(0,2,temp,6,16);
    OLED_ShowNum(0,4,temp2,6,16);
    OLED_ShowNum(0,6,temp3,6,16);
//    if (GetRxFlag()==1)
//    {
//      sscanf(RxDataStr,"L:%d,R:%d",&temp,&temp2);
//      Control_A(temp);
//      Control_B(temp2);
//      OLED_ShowSignedNum(0,6,temp,4,16);
//      OLED_ShowSignedNum(64,6,temp2,4,16);
//    }
//    Encode_CallBack();
//    GetSpeed(&motor0);
//    OLED_ShowSignedNum(0,2,(int )motor0.M1_ActualSpeed,6,16);
//    OLED_ShowSignedNum(0,4,(int )motor0.M2_ActualSpeed,6,16);
//    HAL_Delay(200);
//    if (motor0.M1_ActualSpeed>=10000)
//    {
//      __HAL_TIM_SET_COUNTER(&Encoder_Timer1, 0);
//      count1++;
//    }
//    if (motor0.M1_ActualSpeed>=10000)
//    {
//      __HAL_TIM_SET_COUNTER(&Encoder_Timer2, 0);
//      count2++;
//    }
//    Myprintf("%f,%f,%d,%d\n",motor0.M1_ActualSpeed,motor0.M2_ActualSpeed,count1,count2);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    update();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//		Updata();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
