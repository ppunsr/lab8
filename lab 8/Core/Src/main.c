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
#include <stdio.h>
#include <string.h>
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char TxDataBuffer[32] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };
char LEDfrequency[25]={0};
uint16_t STATE_Display = 0;
int frequency=1;
float period;
float LED_DELAY=500;
uint32_t TimeStamp=0;
uint8_t Switch[2]={0};
int count=0;
int LED=0;
int a;
enum _StateDisplay
		{
			StateDisplay_Start = 0,
			StateDisplay_Menu_Print =10,
			StateDisplay_Menu_WaitInput,
			StateDisplay_Menu1_Print =20,
			StateDisplay_Menu1_WaitInput,
			StateDisplay_Menu2_Print =30,
			StateDisplay_Menu2_WaitInput,
			StateDisplay_LED_Control=40,
			StateDisplay_LED_Increase_Frequency=50,
			StateDisplay_LED_Decrease_Frequency=60,
			StateDisplay_LED_ONOFF=70,
			StateDisplay_Show_Status_Button=80,
			StateDisplay_ERROR=90,

		};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	  char temp[]="HELLO WORLD\r\n please type something to test UART\r\n";
	  char Menu[]= "Menu1 please press 0\r\n Menu2 please press 1\r\n";
	  char Menu1[]="a = frequency +1hz\r\n s= frequency - 1hz\r\n d= LED On/OFF\r\n x= Back\r\n";
	  char Menu2[]="Show buttonStatus\r\n x= Back\r\n ";
	  char Buttonpress[] = "Button Press\r\n";
	  char Buttonunpress[] = "Button UnPress\r\n";
	  char Error[] = "Error Please try again\r\n";
	  char LEDON[]="LED ON\r\n";
	  char LEDOFF[]="LED OFF\r\n";


  //HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp),10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/*Method 1 Polling Mode*/

//		UARTRecieveAndResponsePolling();

		/*Method 2 Interrupt Mode*/
		HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);

		/*Method 2 W/ 1 Char Received*/
		int16_t inputchar = UARTRecieveIT();
		if(inputchar!=-1)
		{

			sprintf(TxDataBuffer, "ReceivedChar:[%c]\r\n", inputchar);
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
		}
		switch(STATE_Display)
		{
		case StateDisplay_Start:
			STATE_Display = StateDisplay_Menu_Print;
			break;
		case StateDisplay_Menu_Print:
			HAL_UART_Transmit(&huart2, (u_int8_t*)Menu, strlen(Menu), 10);
			 STATE_Display = StateDisplay_Menu_WaitInput;
			 break;
		case StateDisplay_Menu_WaitInput:
		switch(inputchar)
		{
		case'0':
			STATE_Display = StateDisplay_Menu1_Print;
			break;
		case'1':
			STATE_Display = StateDisplay_Menu2_Print;
			break;
//		default:
//			HAL_UART_Transmit(&huart2, (u_int8_t*)Error, strlen(Error), 10);
//			break;
		}
		break;
		case StateDisplay_Menu1_Print:
			HAL_UART_Transmit(&huart2, (u_int8_t*)Menu1, strlen(Menu1), 10);
			 STATE_Display = StateDisplay_LED_Control;
			 break;
		case StateDisplay_Menu2_Print:
			HAL_UART_Transmit(&huart2, (u_int8_t*)Menu2, strlen(Menu2), 10);
			STATE_Display = StateDisplay_Show_Status_Button ;
			break;
		case StateDisplay_Show_Status_Button:
			Switch[0]=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
			if (Switch[0]==0 && Switch[1]==1)
			{
				HAL_UART_Transmit(&huart2, (u_int8_t*)Buttonpress, strlen(Buttonpress), 10);
			}
			else if (Switch[0]==1 && Switch[1]==0)
			{
				HAL_UART_Transmit(&huart2, (u_int8_t*)Buttonunpress, strlen(Buttonunpress), 10);

			}
			Switch[1]=Switch[0];
		case StateDisplay_LED_Control:
				switch(inputchar)
				{
				case'0':
					break;
				case 'a':
					STATE_Display= StateDisplay_LED_Increase_Frequency;
					break;
				case 's':
					STATE_Display= StateDisplay_LED_Decrease_Frequency;
					break;
				case 'd':
					STATE_Display= StateDisplay_LED_ONOFF;
					break;
				case 'x':
					STATE_Display= StateDisplay_Menu_Print;
					break;
				}

				break;
//		case StateDisplay_ERROR:
//			HAL_UART_Transmit(&huart2, (u_int8_t*)Error, strlen(Error), 10);
//			STATE_Display=StateDisplay_LED_Control;
//			break;


		case StateDisplay_LED_Increase_Frequency:
				frequency+=1;
				period= 1.0/frequency;
				LED_DELAY=period *500;
				sprintf(LEDfrequency,"Frequency: %d Hz\r\n",frequency);
				HAL_UART_Transmit(&huart2, (uint8_t*)LEDfrequency, strlen(LEDfrequency), 10);
//				LED=1;
//				if(HAL_GetTick()-TimeStamp>= LED_DELAY)
//				{
//					TimeStamp=HAL_GetTick();
//					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//a
//				}
				STATE_Display= StateDisplay_LED_Control;
		break;
		case StateDisplay_LED_Decrease_Frequency:
			frequency-=1;
			period= 1.0/frequency;
			LED_DELAY=period *500;
			sprintf(LEDfrequency,"Frequency: %d Hz\r\n",frequency);
			HAL_UART_Transmit(&huart2, (uint8_t*)LEDfrequency, strlen(LEDfrequency), 10);
//			if(HAL_GetTick()-TimeStamp>=LED_DELAY)
//			{
//				TimeStamp=HAL_GetTick();
//				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			}

			STATE_Display= StateDisplay_LED_Control;
			break;
		case StateDisplay_LED_ONOFF:
			LED+=1;
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)==GPIO_PIN_RESET && count%2==0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_UART_Transmit(&huart2, (u_int8_t*)LEDON, strlen(LEDON), 10);
				count+=1;

			}

//			if(HAL_GetTick()-TimeStamp>=LED_DELAY)
//			{
//				count%2==0;
//				TimeStamp=HAL_GetTick();
//				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//				count+=1;
//			}
			else if(count%2==1)
			{
				HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)==GPIO_PIN_SET;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart2, (u_int8_t*)LEDOFF, strlen(LEDOFF), 10);
				count+=1;
				LED=0;
			}
			STATE_Display= StateDisplay_LED_Control;
			break;
		}
					if(HAL_GetTick()-TimeStamp>=LED_DELAY && LED == 1)
								{
									TimeStamp=HAL_GetTick();
									HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
								}
					if(inputchar>0 && inputchar!= 'a' && inputchar!= 's' && inputchar != 'd' && inputchar!= 'x' && inputchar!= '1'&& inputchar!='0')
					{
						HAL_UART_Transmit(&huart2, (u_int8_t*)Error, strlen(Error), 10);
					}
		/*This section just simmulate Work Load*/
//		HAL_Delay(100);
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
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
