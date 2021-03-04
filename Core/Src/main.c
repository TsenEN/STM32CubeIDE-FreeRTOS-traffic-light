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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
osThreadId LEDTaskHandle;
osThreadId LBtnTaskHandle;
osMessageQId myQueue01Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void LEDTask_App(void const * argument);
void LBtnTask_App(void const * argument);

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 16, uint16_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, LEDTask_App, osPriorityNormal, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of LBtnTask */
  osThreadDef(LBtnTask, LBtnTask_App, osPriorityNormal, 0, 128);
  LBtnTaskHandle = osThreadCreate(osThread(LBtnTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Green_LED_Pin|Red_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Blue_Button_Pin */
  GPIO_InitStruct.Pin = Blue_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Blue_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_LED_Pin Red_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin|Red_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_LEDTask_App */
/**
  * @brief  Function implementing the LEDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LEDTask_App */
void LEDTask_App(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  osEvent osevent;
  int Busy_waiting_count;
  int Change_state_flag = 0; // It's for change state with "busy waiting".
  int BRW;  // It's for break while with "busy waiting" .

  for(;;){
	  Busy_waiting_count = 0;
	  BRW = 1;
	  if(Change_state_flag){
		  while(BRW){
			  HAL_GPIO_WritePin(Green_LED_GPIO_Port,GPIO_PIN_12,GPIO_PIN_RESET); //Close the green LED
			  Busy_waiting_count = 0;
			  HAL_GPIO_WritePin(Red_LED_GPIO_Port,GPIO_PIN_14,GPIO_PIN_SET);
			  while((Busy_waiting_count < 20000)){
				  Busy_waiting_count++;
				  osevent= osMessageGet( myQueue01Handle,0);
				  if(osevent.status==osEventMessage){ // When a message is received from the queue
					  BRW = 0; // break while
					  Change_state_flag = 0; // Set change status flag
					  break;
				  }
			  }
			  HAL_GPIO_WritePin(Red_LED_GPIO_Port,GPIO_PIN_14,GPIO_PIN_RESET);
			  Busy_waiting_count = 0;
			  while((Busy_waiting_count < 20000) &  BRW){
				  Busy_waiting_count++;
				  osevent= osMessageGet( myQueue01Handle,0);
				  if(osevent.status==osEventMessage){
					  BRW = 0;
					  Change_state_flag = 0;
					  break;
				  }
			  }
		  }
	  }else{
		  while(BRW){
			  HAL_GPIO_WritePin(Red_LED_GPIO_Port,GPIO_PIN_14,GPIO_PIN_RESET);
			  Busy_waiting_count = 0;
			  HAL_GPIO_WritePin(Green_LED_GPIO_Port,GPIO_PIN_12,GPIO_PIN_SET);
			  while((Busy_waiting_count < 120000)){
				  Busy_waiting_count++;
				  osevent= osMessageGet( myQueue01Handle,0);
				  if(osevent.status==osEventMessage){
					  BRW = 0;
					  Change_state_flag = 1;
					  break;
				  }
			  }
			  HAL_GPIO_WritePin(Green_LED_GPIO_Port,GPIO_PIN_12,GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(Red_LED_GPIO_Port,GPIO_PIN_14,GPIO_PIN_SET);
			  Busy_waiting_count = 0;
			  while((Busy_waiting_count < 120000) &  BRW){
				  Busy_waiting_count++;
				  osevent= osMessageGet( myQueue01Handle,0);
				  if(osevent.status==osEventMessage){
					  BRW = 0;
					  Change_state_flag = 1;
					  break;
				  }
			  }
		  }
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LBtnTask_App */
/**
* @brief Function implementing the LBtnTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LBtnTask_App */
void LBtnTask_App(void const * argument)
{
  /* USER CODE BEGIN LBtnTask_App */
  /* Infinite loop */
  uint8_t curr_state,prev_state = 0; //State for the button，Used to solve edge detection.
  uint8_t timer = 0;
  for(;;)
  {
	  timer = 0;
	  while(timer <1){
		  timer ++;
	  }
	  curr_state = HAL_GPIO_ReadPin(Blue_Button_GPIO_Port,GPIO_PIN_0);
	  if((curr_state == 1) && (prev_state == 0)){
		  osMessagePut(myQueue01Handle,1, osWaitForever);
	 }
	  prev_state = curr_state;
  }
  /* USER CODE END LBtnTask_App */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
