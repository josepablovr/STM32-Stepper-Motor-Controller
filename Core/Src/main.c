/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>  // for memset
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t tx_buffer[16];
//uint8_t rx_data[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DIR_PIN DIR_Pin
#define DIR_PORT GPIOA
#define STEP_PIN STEP_Pin
#define STEP_PORT GPIOA
const int MICROSTEPS = 25600;
const int Velocity = 120; // (rpm)
const uint32_t temp = (uint32_t)1000000UL * 60UL / (2UL * Velocity * MICROSTEPS);
const int stepDelay = (uint16_t)temp; // 1000us more delay means less speed
const float pitch = 4.0f;  //mm / rev

const uint32_t comm_freq = 10; // Hz
const uint32_t comm_delay = (uint32_t)1000000UL / comm_freq;
float current_distance = 0.0f;

//Every Step
float dx = pitch / (float)MICROSTEPS;

#define RX_BUFFER_SIZE 16
uint8_t rx_data[1];              // Single char buffer for interrupt
char rx_buffer[RX_BUFFER_SIZE];  // Larger buffer to accumulate received text
uint8_t rx_index = 0;            // Index into rx_buffer
uint8_t transfer_cplt = 0;       // Flag: full command received

bool paired_flag = 0;
bool distance_feedback_flag = 0;
bool move_flag = 0;
bool calibration_flag = 0;
bool gpio_flag = 0;
bool error_flag = 0;
float desired_distance = 0.0f;

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

void move_forward(){
	HAL_GPIO_WritePin(DIR_PORT, ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
	microDelay(stepDelay);
	HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
	microDelay(stepDelay);
	current_distance += dx;}

void move_backwards(){
	HAL_GPIO_WritePin(DIR_PORT, ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
	microDelay(stepDelay);
	HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
	microDelay(stepDelay);
	current_distance -= dx;}

void stop_motor(){
	HAL_GPIO_WritePin(DIR_PORT, ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
}

bool read_BT1(void) {
    return (HAL_GPIO_ReadPin(GPIOB, BT1_Pin) == GPIO_PIN_SET);
}

bool read_BT2(void) {
    return (HAL_GPIO_ReadPin(GPIOB, BT2_Pin) == GPIO_PIN_SET);
}

bool read_LMT1(void) {
    return (HAL_GPIO_ReadPin(GPIOB, LMT1_Pin) == GPIO_PIN_SET);
}

bool read_LMT2(void) {
    return (HAL_GPIO_ReadPin(GPIOB, LMT2_Pin) == GPIO_PIN_SET);
}

bool read_SW1(void) {
    return (HAL_GPIO_ReadPin(GPIOA, SW1_Pin) == GPIO_PIN_SET);
}

bool read_SW2(void) {
	return (HAL_GPIO_ReadPin(GPIOA, SW2_Pin) == GPIO_PIN_SET);
}

bool read_SW3(void) {
	return (HAL_GPIO_ReadPin(GPIOA, SW3_Pin) == GPIO_PIN_SET);
}

void turn_on_led(void){
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
}

void turn_off_led(void){
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}

typedef enum {
    STATE_START = 0,
    STATE_FORWARD,
    STATE_BACKWARDS,
    STATE_STOP_MAX,
    STATE_STOP_MIN,
	STATE_CALIBRATE,
	STATE_MOVE
} LogicStates;


typedef enum {
	FLAG_NULL = 0,
    FLAG_PAIRED,
    FLAG_FEEDBACK,
    FLAG_MOVE,
	FLAG_MOVE_FINISHED,
    FLAG_CALIBRATION,
    FLAG_CALIBRATION_FINISHED,
    FLAG_GPIO,
	FLAG_ERROR
} Flags;



static Flags raised_flag = FLAG_NULL;
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_UART_Receive_IT(&huart2, rx_data, 1);
  static LogicStates currentState = STATE_START;
  bool com_cycle_begun = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //COMUNICATION PROTOCOL
	  if (com_cycle_begun == 0){
	  	  __HAL_TIM_SET_COUNTER(&htim2, 0);
	  	  com_cycle_begun = 1;}

	  else if (__HAL_TIM_GET_COUNTER(&htim2) >= comm_delay){
		  // 1) Scale by 10 for one decimal place, then round
		  int scaled_distance = (int)(current_distance*10 + 0.5f);

		  // Construct the output string (integer placeholders)
		  snprintf((char*)tx_buffer, sizeof(tx_buffer),
					  "DX: %03d mm\n\r",
					  scaled_distance);
		  //HAL_UART_Transmit_IT(&huart2, tx_buffer,sizeof(tx_buffer));
		  com_cycle_begun = 0;
		  uint8_t replyMsg[20];
		  memset(replyMsg, ' ', sizeof(replyMsg));
		  switch (raised_flag)  {

		  	  case FLAG_NULL:
		  		  break;

		      case FLAG_PAIRED:
		    	  strcpy((char*)replyMsg, "PAIRED\n\r");
		    	  HAL_UART_Transmit_IT(&huart2, replyMsg, strlen((char*)replyMsg));
		    	  raised_flag = FLAG_NULL;
		  		  break;

		      case FLAG_FEEDBACK:
		      {
		          // First ensure the distance is in [0..300]
		          int distance = (int)current_distance;
		          if (distance < 0)     distance = 0;
		          if (distance > 300)   distance = 300;

		          // Build the reply: "DXXXU#L#1#2#3#"
		          //  - %03d formats the distance with 3 digits (e.g. 007, 042, 300)
		          //  - read_LMT1(), read_LMT2(), read_SW1(), read_SW2(), read_SW3() each
		          //    return 0 or 1, so those get inserted as single digits.
		          sprintf((char*)replyMsg,
		                  "D%03dU%dL%d1%d2%d3%d\n\r",
		                  distance,
		                  (int)read_LMT1(),
		                  (int)read_LMT2(),
		                  (int)read_SW1(),
		                  (int)read_SW2(),
		                  (int)read_SW3());

		          // Transmit the message
		          HAL_UART_Transmit_IT(&huart2, replyMsg, strlen((char*)replyMsg));
		          raised_flag = FLAG_NULL;
		      }
		      break;

		      case FLAG_MOVE:
		    	  // First ensure the distance is in [0..300]
				  int distance = (int)current_distance;
				  if (distance < 0)     distance = 0;
				  if (distance > 300)   distance = 300;

				  // Build the reply: "DXXXU#L#1#2#3#"
				  //  - %03d formats the distance with 3 digits (e.g. 007, 042, 300)
				  //  - read_LMT1(), read_LMT2(), read_SW1(), read_SW2(), read_SW3() each
				  //    return 0 or 1, so those get inserted as single digits.
				  sprintf((char*)replyMsg,
						  "D%03dU%dL%d1%d2%d3%d\n\r",
						  distance,
						  (int)read_LMT1(),
						  (int)read_LMT2(),
						  (int)read_SW1(),
						  (int)read_SW2(),
						  (int)read_SW3());

				  // Transmit the message
				  HAL_UART_Transmit_IT(&huart2, replyMsg, strlen((char*)replyMsg));

		    	  break;
		      case FLAG_MOVE_FINISHED:
		    	  strcpy((char*)replyMsg, "GOAL REACHED\n\r");
				  HAL_UART_Transmit_IT(&huart2, replyMsg, strlen((char*)replyMsg));
				  raised_flag = FLAG_NULL;
		    	  break;

		      case FLAG_CALIBRATION:
		    	  strcpy((char*)replyMsg, "CALIBRATING\n\r");
		    	  HAL_UART_Transmit_IT(&huart2, replyMsg, strlen((char*)replyMsg));
		    	  break;
		      case FLAG_CALIBRATION_FINISHED:
				  strcpy((char*)replyMsg, "CALIBRATION DONE\n\r");
				  HAL_UART_Transmit_IT(&huart2, replyMsg, strlen((char*)replyMsg));
				  raised_flag = FLAG_NULL;
				  break;


		      case FLAG_GPIO:
		    	  if (read_LMT1()){
		    		  strcpy((char*)replyMsg, "GPIO 1 0\n\r");}
		    	  else if (read_LMT2()){
		    		  strcpy((char*)replyMsg, "GPIO 0 1\n\r");}
		  	  	  else{
		  	  		  strcpy((char*)replyMsg, "GPIO 0 0\n\r"); }

		    	  HAL_UART_Transmit_IT(&huart2, replyMsg, strlen((char*)replyMsg));
		    	  raised_flag = FLAG_NULL;
		    	  break;

		      case FLAG_ERROR:
		    	  strcpy((char*)replyMsg, "ERROR\n\r");
		    	  HAL_UART_Transmit_IT(&huart2, replyMsg, strlen((char*)replyMsg));
		    	  raised_flag = FLAG_NULL;
		    	  break;

		      default:
		    	  break;}

	  }


	  switch (currentState){
		  case STATE_START:
			  turn_off_led();
			  stop_motor();

			  if (read_BT1()) {
				  currentState = STATE_FORWARD;}
			  else if (read_BT2()) {
				  currentState = STATE_BACKWARDS;}
			  else if (raised_flag == FLAG_CALIBRATION){
				  currentState = STATE_CALIBRATE;}
			  else if (raised_flag == FLAG_MOVE){
				  currentState = STATE_MOVE;}

			  break;

		  case STATE_FORWARD:
			  move_forward();
			  turn_on_led();

			  if (read_BT1() == false) {
				  currentState = STATE_START;}

			  else if (read_LMT2()) {
				  currentState = STATE_STOP_MAX;}

			  break;

		  case STATE_BACKWARDS:
			  move_backwards();
			  turn_on_led();

			  if (read_BT2() == false) {
				  currentState = STATE_START;}

			  else if (read_LMT1()) {
				  currentState = STATE_STOP_MIN;}

			  break;


		  case STATE_STOP_MAX:
			  turn_off_led();
			  stop_motor();
			  if (read_BT2()) {
				currentState = STATE_BACKWARDS;}

			  break;

		  case STATE_STOP_MIN:
			  turn_off_led();
			  stop_motor();
			  current_distance = 0.0f;
			  if (read_BT1()) {
				  currentState = STATE_FORWARD;}

			  break;


		  case STATE_CALIBRATE:
			  turn_on_led();
			  move_backwards();
			  if (read_LMT1()) {
				  current_distance = 0.0f;
				  raised_flag = FLAG_CALIBRATION_FINISHED;
			  	  currentState = STATE_START;}


			  else if (read_BT1() || read_BT2()) {
				  raised_flag = FLAG_CALIBRATION_FINISHED;
				  currentState = STATE_START;}

			  break;

		  case STATE_MOVE:
			  turn_on_led();

			  if (abs(desired_distance - current_distance) <= 0.5f){
				  raised_flag = FLAG_MOVE_FINISHED;
			  	  currentState = STATE_START;}

			  else if (desired_distance > current_distance){
			  	  move_forward();}
			  else if (desired_distance < current_distance){
				  move_backwards();}

			  else if (read_BT1() || read_BT2()) {
				  raised_flag = FLAG_NULL;
				  currentState = STATE_START;}

			  break;


		  default:

			  printf("Unknown state! Forcing STOP.\n");
			  currentState = STATE_START;
			  break;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

//	  if (read_BT1()) {
//		  move_forward();
//	  	  turn_on_led();}
//
//	  else if (read_BT2()) {
//		  move_backwards();
//		  turn_on_led();}
//	  else{
//		  stop_motor();
//		  turn_off_led();}
//
//	  if (read_LMT1() ||read_LMT2()){
//		  turn_on_led();}
//	  else{
//		  turn_off_led();}




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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 47;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENABLE_Pin|DIR_Pin|STEP_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENABLE_Pin DIR_Pin STEP_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin|DIR_Pin|STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW3_Pin SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW3_Pin|SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BT1_Pin LMT1_Pin LMT2_Pin BT2_Pin */
  GPIO_InitStruct.Pin = BT1_Pin|LMT1_Pin|LMT2_Pin|BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
//	memset(rx_data, 0, sizeof(rx_data));
//	snprintf((char*)tx_buffer, sizeof(tx_buffer),
//						  "RECEIVED");
//	HAL_UART_Transmit(&huart2, tx_buffer, 16,10);

	if (huart->Instance == USART2) // Check we're handling USART2
	{
		// If this is the first character of a new message, clear the buffer
		if (rx_index == 0)
		{
			// Clear the buffer
			for (uint8_t i = 0; i < RX_BUFFER_SIZE; i++)
			{
				rx_buffer[i] = 0;			}


		}

		// Check if we received a carriage return (ASCII 13, '\r')
		if (rx_data[0] != 13)
		{
			// Store the received character and increment index
			rx_buffer[rx_index++] = rx_data[0];

			// Prevent buffer overflow
			if (rx_index >= RX_BUFFER_SIZE - 1)
			{
				rx_index = 0;
			}
		}
		else
		{
			// We got a '\r' → treat it as end of command
			rx_index = 0;
			transfer_cplt = 1; // Mark command as complete

			// Optionally send back a newline/return to the sender




			// Example: if the buffer contains "LED ON", turn LED on


			if (!strcmp(rx_buffer, "PAIRING")){
				raised_flag = FLAG_PAIRED;}

			else if (strcmp(rx_buffer, "STATUS") == 0){
				raised_flag = FLAG_FEEDBACK;}

			else if ((strncmp(rx_buffer, "MV ", 3) == 0)){
				int moveValue  = atoi(rx_buffer + 3);
				if (moveValue >= 0 && moveValue <= 250){
					raised_flag = FLAG_MOVE;
					desired_distance = moveValue;}
				else{
					raised_flag = FLAG_ERROR;}}

			else if (strcmp(rx_buffer, "CALIBRATE") == 0){
				raised_flag = FLAG_CALIBRATION;}

			else if (strcmp(rx_buffer, "GPIO") == 0){
				raised_flag = FLAG_GPIO;}
			else{
				raised_flag = FLAG_ERROR;}



			// You could handle other commands here (e.g. "LED OFF", "CALIB", etc.)
		}

		// Echo the received character (optional)
		//HAL_UART_Transmit(&huart2, rx_data, 1, 100);

		// Re-enable interrupt reception of the next character
		HAL_UART_Receive_IT(&huart2, rx_data, 1);
	}
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
