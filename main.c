/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>	//sprintf --print บนตัวแปร
#include <string.h>	//strlen --วัความยาว str
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2; // ใช้ได้เลยโดยไม่ต้องสั่ง start stop

/* USER CODE BEGIN PV */

typedef enum
{
	state_start0 = 1 ,
	state_start1,
	state_LEDControl0,
	state_LEDControl1,
	state_ButtonStatus0,
	state_Buttonstatus1
} state;//0 = Print, 1 = Select

uint32_t TimeStamp = 0;
uint32_t frequency = 10;
uint8_t LED = 1;
uint8_t Button[2];

char TxDataBuffer[32] = {0}; //Global -- every time
char RxDataBuffer[32] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTRecieveAndResponsePolling();
int16_t UARTRecieveIT();
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  {
	  char temp[] = "HELP ME PLZ\r\n";
	  HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp), 100); // ส่ง str -- ถ้าประโยคยาวกว่า 10 ms ประโยคอาจจะไม่ขึ้น สามารถใส่มากๆไว้ก่อนก็ได้ หรือจะทำ interrupt ก็ได้
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*Method 1 Polling Mode*/ //--ครบตาม timeout
//	  UARTRecieveAndResponsePolling(); //Call Function For Recieve Data

	  /*Method 2 Interrupt Mode*/ //--ครบตามจำนวนตัวอักษร
	  HAL_UART_Receive_IT(&huart2, (uint8_t*)RxDataBuffer, 32);
	  	  	  //ฟังก์ชันนี้สามารถใส่นอกลูปได้ แต่เมื่อใส่ข้อมูลครบ 32 ตัว จะจบฟังก์ชันนี้ด้วย
	  	  	  //เมื่อเข้าสู่การเตรียมรับแล้ว โปรแกรมจะข้างฟังก์ชันนี้ไปเอง จึงสามารถใส่ฟังก์ชันนี้ในลูปได้

	  if (LED == 1) //on
	  {
		  if(HAL_GetTick() - TimeStamp >= 100000) //กระพริบ LED
		  {
			  TimeStamp = HAL_GetTick();
			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  }
	  }

	  else
	  {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	  }

	  Button[1] = Button[0];
	  Button[0] = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	  int16_t inputchar = UARTRecieveIT();

	  static state STATE = 1;

	  switch(STATE)
	  {
	  case state_start0:
		  sprintf(TxDataBuffer, " Main Menu\n\r 0: LED Control \n\r 1: Button status\n\r");
		  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		  STATE = state_start1;
		  break;

	  case state_start1:
		  switch (inputchar)
		  {
		  case '0':
			  STATE = state_LEDControl0;
			  break;
		  case '1':
			  STATE = state_ButtonStatus0;
			  break;
		  case -1:
			  break;
		  default:
				  sprintf(TxDataBuffer, "Sorry, Key is wrong\n\r");
				  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
				  STATE = state_start0;
				  break;
		  }
		  break;
/*------------------------------------------------------------------------------------------------------------------*/
	  case state_LEDControl0:
		  sprintf(TxDataBuffer,"a: Speed Up +1Hz\n\r s: Speed Down -1Hz\n\r d: On/Off\n\r x: back\n\r");
		  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
		  STATE = state_LEDControl1;
		  break;

	  case state_LEDControl1:
		  switch(inputchar)
		  {
		  case 'a':
			  frequency += 1;
			  sprintf(TxDataBuffer, " [%d]\r\n" , frequency);
			  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			  break;

		  case's':
			  if(frequency != 0)
			  {
				  frequency -= 1;
				  sprintf(TxDataBuffer, " [%d]\r\n" , frequency);
				  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			  }
			  else
			  {
				  sprintf(TxDataBuffer, "Current Frequency is low \r\n");
				  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			  }
			  break;

		  case'd':
			  if(LED == 1)
			  {
				  LED == 0;
				  sprintf(TxDataBuffer,
						  "Turn Off LED\n\r");
				  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			  }
			  else
			  {
				  LED == 1;
				  sprintf(TxDataBuffer,"Turn On LED\n\r");
				  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			  }
			  LED = !LED ;
			  break;

		  case 'x':
			  STATE = state_start0;
			  break;

		  default:
			  STATE = state_LEDControl1;
			  break;
		  }
		  break;
/*------------------------------------------------------------------------------------------------------------------*/
	  case state_ButtonStatus0:
		  sprintf(TxDataBuffer,"Button Status\n\r x: Back \n\r ");
		  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
		  STATE = state_Buttonstatus1;
		  break;

	  case state_Buttonstatus1:
		  if(Button[0] == 0 && Button[1] == 1)
		  {
			  sprintf(TxDataBuffer, "PRESSED\n\r");
			  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			  STATE = state_ButtonStatus0;
		  }
		  else if (Button[0] == 1 && Button[1] == 0)
		  {
			  sprintf(TxDataBuffer, "UNPRESSED\n\r");
			  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			  STATE = state_ButtonStatus0;
		  }
	  	  switch(inputchar)
	  	  {
			  case 'x':
			  STATE = state_start0;
			  break;

			  default:
			  STATE = state_Buttonstatus1;
			  break;
	  	  }
	  	  break;
	  }



/*----------------------------------------------------------------------------*/
	  /*Method 2 W/ 1 Char Received*/ //--เอาท์ออกมาทีละตัวอักษร
//	  int16_t inputchar = UARTRecieveIT();
//	  if(inputchar != -1) /* mean have new data */
//	  {
//		  sprintf(TxDataBuffer, "ReceivedChar:[%c]\r\n", inputchar);
//		  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
//	  }
//
//	  /*This section just simmulate Work Load*/
	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
/*----------------------------------------------------------------------------*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UARTRecieveAndResponsePolling()
{
	//create buffer
	char Recieve[32]={0};
	//start receive
	HAL_UART_Receive(&huart2, (uint8_t*)Recieve, 4, 1000);
	//Feedback
	sprintf(TxDataBuffer, "Received:[%s]\r\n", Recieve); //ข้อความอะไรก็ได้ ไม่ฟิคจำนวนเหมือนแบบแรกที่ทำ
	//send
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000); // Polling

}

int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;   //สำหรับตาม buffer -- ข้อมูลล่าสุดที่คอนโทลเลอร์ต้องอ่านแล้วนำไปใช้ ไม่ใช่ข้อมูลล่าสุดที่เข้ามาในคอนโทลเลอร์
		//static เก็บค่าไว้ใช้ต่อ ไม่ใช่เริ่มใหม่ที่ 0 ทุกครั้ง
	int16_t data=-1; //ถ้าอ่านไปจนสุดแล้ว จะรีเทิร์น -1 หมายความว่า ไม่มีข้อมูลหลังจากนั้นแล้ว
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos) // เช็คว่าที่ต้องการนำไปใช้ ตรงกับข้อมูลปัจจุบันแล้วยัง
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
						// Data in RxBuffer  ไปแปะใน TxBuffer by format Received:[%s]\r\n
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer)); // Interrupt
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
