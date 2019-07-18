/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DELAY 1000 //oczekiwanie do komunikatu o braku odpowiedzi w ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t timer_ms=0, timer_us=0;
uint16_t engine_timer=7000;
uint8_t RH_b1=0, RH_b2=0, T_b1=0, T_b2=0, sum=0;
uint8_t timer_ms_flag=0, timer_us_flag=0;
uint8_t error_flag=1;//flaga pozwalajaca na wylaczenie komunikatow o bledach sensora 1 - komunikaty wlaczone
uint8_t sensor_flag=0;//flaga pozwalajaca na wylaczenie odczytu z sensora 1 - odczyt wlaczony
uint8_t pullup_flag=1;//1 - z rezystorem pullup
uint8_t stage_ext=1;//etap obslugi czujnika
uint8_t pwm_vf=75, pwm_vrl=75, pwm_vrr=75;//silniki pionowe PC0 PC1 PC3
uint8_t pwm_hl=75, pwm_hr=75;//silniki poziome PA4 PA6

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void sensor_procedure(void);
void control_procedure(void);
void init_engines(void);
void set_engine_duty(uint8_t engine, uint8_t duty);
void manual(void);
void engine_manual(void);
void commands_manual(void);
void toggle_pullup(void);
void GPIO_set_out(void);
void GPIO_set_in(void);
void DHT11_Start(void);
void check_response(void);
void read_data(void);
void send_data(void);
void send_char(char c);
int __io_putchar(int ch);
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
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	printf("Witaj w programie\n");
	manual();
	init_engines();
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1439;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1439;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 71;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RH_T_Sensor_GPIO_Port, RH_T_Sensor_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : RH_T_Sensor_Pin */
  GPIO_InitStruct.Pin = RH_T_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RH_T_Sensor_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * funkcja do obslugi przerwan od timerow
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM2){
		if(timer_ms_flag==1){
			timer_ms++;
		}
		if(engine_timer>0){
			engine_timer--;
			if(engine_timer==0){
				printf("Silniki gotowe\n");
			}
		}
	} else if(htim->Instance==TIM4){
		if(timer_us_flag==1){
			timer_us++;
		}
		control_procedure();
		if(sensor_flag==1){
			sensor_procedure();
		}
	}
}
/**
 * procedura obslugi czujnika
 * 1. inicjalizacja czujnika
 * 2. sprawdzanie odpowiedzi
 * 3. odczyt danych
 * 4. wysylanie danych
 */
void sensor_procedure(void){
	switch(stage_ext){
	case 1:
		DHT11_Start();
		break;
	case 2:
		check_response();
		break;
	case 3:
		read_data();
		break;
	case 4:
		send_data();
		break;
	}
}
/**
 * procedura do obslugi komend sterujacych
 */
void control_procedure(void){
	static uint8_t index=0, command=0;
	static uint8_t duty=0;
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == SET){
		uint8_t value;
		HAL_UART_Receive(&huart2, &value, 1, 100);
		switch(index){
		case 0:
			if(value=='s'){
				printf("%c\n",value);
				duty=75;
				set_engine_duty(1,duty);
				set_engine_duty(2, duty);
				set_engine_duty(3, duty);
				set_engine_duty(4, duty);
				set_engine_duty(5, duty);
			} else if(value=='?'){
				manual();
			} else if(value=='c'){
				if(sensor_flag==1){
					sensor_flag=0;
					printf("Odczyt z czujnika wylaczony.\n");
				} else if(sensor_flag==0){
					sensor_flag=1;
					printf("Odczyt z czujnika wlaczony.\n");
				}
			} else if(value=='e'){
				if(error_flag==1){
					error_flag=0;
					printf("Komunikaty o bledzie czujnika wylaczone.\n");
				} else if(error_flag==0){
					error_flag=1;
					printf("Komunikaty o bledzie czujnika wlaczone.\n");
				}
			} else if(value=='p'){
				toggle_pullup();
			} else if(value=='v'||value=='h'||(value>='1'&&value<='5')){
				if(engine_timer>0){
					printf("Silniki jeszcze nie sa gotowe.\n");
					printf("Odczekaj %d ms\n",engine_timer);
				} else{
					command=value;
					printf("%c\n",value);
					printf("Podaj wypelnienie\n");
					index=1;
				}
			} else{
				printf("Niewlasciwa komenda.\n");
				commands_manual();
			}
			break;
		case 1:
			if(value<'5'||value>'9'){
				printf("Niewlasciwy znak.\n");
				printf("Obslugiwane wypelnienie 50-99%%.\n");
			} else{
				duty=(value-'0')*10;
				printf("%c", value);
				index=2;
			}
			break;
		case 2:
			if(value<'0'||value>'9'){
				printf("Niewlasciwy znak.\n");
				printf("Obslugiwane wypelnienie 50-99%%.\n");
			} else{
				duty+=(value-'0');
				printf("%c\n", value);
				switch(command){
				case 'v':
					set_engine_duty(1,duty);
					set_engine_duty(2, duty);
					set_engine_duty(3, duty);
					break;
				case 'h':
					set_engine_duty(4, duty);
					set_engine_duty(5, duty);
					break;
				case '1':
					set_engine_duty(1,duty);
					break;
				case '2':
					set_engine_duty(2, duty);
					break;
				case '3':
					set_engine_duty(3, duty);
					break;
				case '4':
					set_engine_duty(4, duty);
					break;
				case '5':
					set_engine_duty(5, duty);
					break;
				}
				index=0;
			}
			break;
		}
	}
}
/**
 * inicjalizacja silnikow
 */
void init_engines(void){
	uint8_t duty=75;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	pwm_vf=duty;
	TIM1->CCR1=pwm_vf;
	pwm_vrl=duty;
	TIM1->CCR2=pwm_vrl;
	pwm_vrr=duty;
	TIM1->CCR4=pwm_vrr;
	pwm_hl=duty;
	TIM3->CCR1=pwm_hl;
	pwm_hr=duty;
	TIM3->CCR2=pwm_hr;
}
/**
 * ustawianie wypelnienia silnikow
 */
void set_engine_duty(uint8_t engine, uint8_t duty){
	switch(engine){
	case 1:
		pwm_vf=duty;
		TIM1->CCR1=pwm_vf;
		printf("engine vf PWM=%2d%%\n",pwm_vf);
		break;
	case 2:
		pwm_vrl=duty;
		TIM1->CCR2=pwm_vrl;
		printf("engine vrl PWM=%2d%%\n",pwm_vrl);
		break;
	case 3:
		pwm_vrr=duty;
		TIM1->CCR4=pwm_vrr;
		printf("engine vrr PWM=%2d%%\n",pwm_vrr);
		break;
	case 4:
		pwm_hl=duty;
		TIM3->CCR1=pwm_hl;
		printf("engine hl PWM=%2d%%\n",pwm_hl);
		break;
	case 5:
		pwm_hr=duty;
		TIM3->CCR2=pwm_hr;
		printf("engine hr PWM=%2d%%\n",pwm_hl);
		break;
	}
}
/**
 * funkcja wyswietlajaca instrukcje
 */
void manual(void){
	engine_manual();
	commands_manual();
}
/**
 * funkcja wyswietlajaca oznaczenia silnikow wg pinow
 */
void engine_manual(void){
	printf("Oznaczenia silnikow:\n");
	printf("vf - PC0\t");
	printf("vrl - PC1\t");
	printf("vrr - PC3\n");
	printf("hl - PA4\t");
	printf("hr - PA6\n\n");
}
/**
 * funkcja wyswietlajaca obslugiwane komendy
 */
void commands_manual(void){
	printf("Obslugiwane komendy to:\n");
	printf("? - instrukcja\n");
	printf("c - wlaczenie lub wylaczenie odczytow z czujnika\n");
	printf("e - wlaczenie lub wylaczenie komunikatow o bledzie czujnika\n");
	printf("p - wlaczenie lub wylaczenie rezystora pullup.\n");
	printf("s - zatrzymanie wszystkich silnikow\n");
	printf("v - sterowanie silnikami pionowymi\n");
	printf("h - sterowanie silnikami poziomymi\n");
	printf("1 - sterowanie silnikiem vf\n");
	printf("2 - sterowanie silnikiem vrl\n");
	printf("3 - sterowanie silnikiem vrr\n");
	printf("4 - sterowanie silnikiem hl\n");
	printf("5 - sterowanie silnikiem hr\n\n\n");
}
/**
 * funkcja pozwalajaca na wlaczenie lub wylaczenie rezystora pullup
 */
void toggle_pullup(void){
	if(pullup_flag==1){
		pullup_flag=0;
		printf("Rezystor pullup zostanie wylaczony przy najblizszej zmianie konfiguracji pinu.\n");
	} else if(pullup_flag==0){
		pullup_flag=1;
		printf("Rezystor pullup zostanie wlaczony przy najblizszej zmianie konfiguracji pinu.\n");
	}
}
/**
 * funkcja ustawiajaca pin czujnika jako wyjscie
 */
void GPIO_set_out(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = RH_T_Sensor_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RH_T_Sensor_GPIO_Port, &GPIO_InitStruct);
}
/**
 * funkcja ustawiajaca pin czujnika jako wejscie
 */
void GPIO_set_in(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = RH_T_Sensor_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	if(pullup_flag==1){
		GPIO_InitStruct.Pull = GPIO_PULLUP;
	} else if(pullup_flag==0){
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	HAL_GPIO_Init(RH_T_Sensor_GPIO_Port, &GPIO_InitStruct);
}
/**
 * inicjalizacja czujnika:
 * 1. ustawianie pinu czujnika jako wyjscie
 * 2. wysylanie stanu niskiego na czujnik
 * 3. odczekiwanie 18 ms
 * 4. ustawianie pinu czujnika jako wejscie
 */
void DHT11_Start(void){
	static uint8_t stage_int=1;
	switch(stage_int){
	case 1:
		GPIO_set_out();
		HAL_GPIO_WritePin(RH_T_Sensor_GPIO_Port, RH_T_Sensor_Pin, GPIO_PIN_RESET);
		timer_ms_flag=1;
		stage_int=2;
		break;
	case 2:
		if(timer_ms>=18){
			GPIO_set_in();
			timer_ms_flag=0;
			timer_ms=0;
			stage_int=1;
			stage_ext=2;
		}
		break;
	}
}
/**
 * sprawdzanie odpowiedzi czujnika
 * 1. czekiwanie 40 us
 * 2. oczekiwanie na stan niski
 * 3. czekanie 80 us
 * 4. oczekiwanie na stan wysoki
 */
void check_response(void){
	static uint8_t stage_int=1;
	switch(stage_int){
	case 1:
		timer_us_flag=1;
		stage_int=2;
		break;
	case 2:
		if(timer_us>=40){
			timer_us_flag=0;
			timer_ms_flag=1;
			timer_us=0;
			stage_int=3;
		}
		break;
	case 3:
		if(HAL_GPIO_ReadPin(RH_T_Sensor_GPIO_Port, RH_T_Sensor_Pin)==GPIO_PIN_RESET){
			timer_us_flag=1;
			timer_ms_flag=0;
			timer_ms=0;
			stage_int=4;
		} else if(timer_ms>DELAY){
			/*
			 * jesli brak zmiany przez DELAY ms wysylany jest komunikat
			 * procedura obslugi czujnika zaczyna sie od poczatku
			 */
			timer_ms_flag=0;
			timer_ms=0;
			stage_int=1;
			if(error_flag==1){
				printf("Czujnik nie odpowiada\n");
				printf("Sprawdzanie odpowiedzi, stan wysoki\n");
			}
			stage_ext=1;
		}
		break;
	case 4:
		if(timer_us>=80){
			timer_us_flag=0;
			timer_ms_flag=1;
			timer_us=0;
			stage_int=5;
		}
		break;
	case 5:
		if(HAL_GPIO_ReadPin(RH_T_Sensor_GPIO_Port, RH_T_Sensor_Pin)==GPIO_PIN_SET){
			timer_ms_flag=0;
			timer_ms=0;
			stage_int=1;
			stage_ext=3;
		} else if(timer_ms>DELAY){
			/*
			 * jesli brak zmiany przez DELAY ms wysylany jest komunikat
			 * procedura obslugi czujnika zaczyna sie od poczatku
			 */
			timer_ms_flag=0;
			timer_ms=0;
			stage_int=1;
			if(error_flag==1){
				printf("Czujnik nie odpowiada\n");
				printf("Sprawdzanie odpowiedzi, stan niski\n");
			}
			stage_ext=1;
		}
		break;
	}
}
/**
 * funkcja do odczytu danych
 * odczytywane sa wartosci:
 * bajtu zawierajacego calkowita czesc wzglednej wilgotnosci
 * bajtu zawierajacego ulamkowa czesc wzglednej wilgotnosci (powinien byc rowny 0)
 * bajtu zawierajacego calkowita czesc temperatury
 * bajtu zawierajacego ulamkowa czesc temperatury (powinien byc rowny 0)
 * bajtu zawierajacego sume kontrolna rowna sumie pozostalych bajtow
 *
 * dane skladaja sie z 8 bitow
 * procedura odczytu dla kazdego bitu
 * 1. oczekiwanie na stan wysoki
 * 2. czekanie 40 us
 * 3. sprawdzanie stanu pinu
 * a) stan wysoki - zapis jedynki na odpowiedniej pozycji
 * b) stan niski - zapis zera na odpowiedniej pozycji
 * 4. oczekiwanie na stan niski
 * po odczycie 8 bitow wartosci zapisywane sa do odpowiednich zmiennych
 *
 */
void read_data(void){
	static uint8_t stage_int=1;
	static uint8_t index=0;
	static uint8_t byte=0;
	static uint8_t byte_index=0;
	if(index<8){
		switch(stage_int){
		case 1:
			timer_ms_flag=1;
			stage_int=2;
			break;
		case 2:
			if(HAL_GPIO_ReadPin(RH_T_Sensor_GPIO_Port, RH_T_Sensor_Pin)==GPIO_PIN_SET){
				timer_ms_flag=0;
				timer_ms=0;
				timer_us_flag=1;
				stage_int=3;
			} else if(timer_ms>DELAY){
				/*
				 * jesli brak zmiany przez DELAY ms wysylany jest komunikat
				 * procedura obslugi czujnika zaczyna sie od poczatku
				 */
				timer_ms_flag=0;
				timer_ms=0;
				stage_int=1;
				if(error_flag==1){
					printf("Czujnik nie odpowiada\n");
					printf("Odczyt danych, stan niski\n");
				}
				stage_ext=1;
			}
			break;
		case 3:
			if(timer_us>=40){
				timer_us_flag=0;
				timer_us=0;
				if(HAL_GPIO_ReadPin(RH_T_Sensor_GPIO_Port, RH_T_Sensor_Pin)==GPIO_PIN_RESET){
					byte &= ~(1<<(7-index));
				} else if(HAL_GPIO_ReadPin(RH_T_Sensor_GPIO_Port, RH_T_Sensor_Pin)==GPIO_PIN_SET){
					byte |= (1<<(7-index));
				}
				timer_ms_flag=1;
				stage_int=4;
			}
			break;
		case 4:
			if(HAL_GPIO_ReadPin(RH_T_Sensor_GPIO_Port, RH_T_Sensor_Pin)==GPIO_PIN_RESET){
				index++;
				stage_int=1;
				timer_ms_flag=0;
				timer_ms=0;
			} else if(timer_ms>DELAY){
				/*
				 * jesli brak zmiany przez DELAY ms wysylany jest komunikat
				 * procedura obslugi czujnika zaczyna sie od poczatku
				 */
				timer_ms_flag=0;
				timer_ms=0;
				stage_int=1;
				if(error_flag==1){
					printf("Czujnik nie odpowiada\n");
					printf("Odczyt danych, stan wysoki\n");
				}
				stage_ext=1;
			}
			break;
		}
	} else{
		printf("%d\n", byte);//wypisanie odczytanej wartosci bajta (do testow)
		index=0;
		switch(byte_index){
		case 0:
			RH_b1=byte;
			byte_index=1;
			break;
		case 1:
			RH_b2=byte;
			byte_index=2;
			break;
		case 2:
			T_b1=byte;
			byte_index=3;
			break;
		case 3:
			T_b2=byte;
			byte_index=4;
			break;
		case 4:
			sum=byte;
			byte_index=0;
			stage_ext=4;
			break;
		}
	}
}
/**
 * funkcja do wysylania odczytow z czujnika
 * 1. jesli suma kontrolna jest rowna sumie pozostalych bitow wysylany jest komunikat
 * 2. oczekiwanie 1s
 */
void send_data(void){
	static uint8_t stage_int=1;
	switch(stage_int){
	case 1:
		if(sum==RH_b1+RH_b2+T_b1+T_b2){
			printf("RH: %2d %% \t T: %2d *C\n", RH_b1, T_b1);

		}
		timer_ms_flag=1;
		stage_int=2;
		break;
	case 2:
		if(timer_ms>=1000){
			timer_ms_flag=0;
			timer_ms=0;
			stage_int=1;
			stage_ext=1;
		}
	}
}
/**
 * funkcja do wysylania pojedynczego znaku
 */
void send_char(char c){
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, 1);
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_RESET);
}
/**
 * funkcja zapisu znaku wykorzystywana przez printf
 */
int __io_putchar(int ch){
	if (ch == '\n'){
		send_char('\r');
	}
	send_char(ch);
	return ch;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
