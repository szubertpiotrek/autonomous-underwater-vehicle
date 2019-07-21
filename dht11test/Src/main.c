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
#include "stm32f3xx_hal.h"
#include "dwt_stm32_delay.h"
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t received;
uint8_t sensor_flag=1, engines_flag=0;//flaga pozwalajaca na wylaczenie odczytu z sensora 1 - odczyt wlaczony
uint8_t engines_delay=0;
uint8_t pwm_vf=75, pwm_vrl=75, pwm_vrr=75;//silniki pionowe PC0 PC1 PC3
uint8_t pwm_hl=75, pwm_hr=75;//silniki poziome PA4 PA6
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t sum, RH, TEMP;
uint8_t check = 0;

GPIO_InitTypeDef GPIO_InitStruct;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void set_gpio_output (void);
void set_gpio_input (void);
void DHT11_start (void);
void check_response (void);
uint8_t read_data (void);
void send_char(char c);
int __io_putchar(int ch);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void control_procedure(void);
void init_engines(void);
void set_engine_duty(uint8_t engine, uint8_t duty);
void manual(void);
void engine_manual(void);
void commands_manual(void);
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
	MX_TIM1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	DWT_Delay_Init ();
	printf("Witaj w programie\n");
	manual();
	init_engines();
	HAL_UART_Receive_IT(&huart2, &received, 1);
	HAL_Delay (1000);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if(sensor_flag){
			DHT11_start ();
			check_response ();
			Rh_byte1 = read_data ();
			Rh_byte2 = read_data ();
			Temp_byte1 = read_data ();
			Temp_byte2 = read_data ();
			sum = read_data();
			if (sum == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2)){    // if the data is correct
				printf("RH: %2d %% \t T: %2d *C \n", Rh_byte1, Temp_byte1);
				HAL_Delay (1200);
				if((engines_delay>=6)&&(engines_flag==0)){
					engines_flag=1;
					printf("Silniki gotowe\n");
				} else{
					engines_delay++;
				}
			}
		}
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
			|RCC_PERIPHCLK_TIM34;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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

	/*Configure GPIO pin : PB0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void set_gpio_output (void)
{
	/*Configure GPIO pin output: PA2 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void set_gpio_input (void)
{
	/*Configure GPIO pin input: PA2 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void DHT11_start (void)
{
	set_gpio_output ();  // set the pin as output
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_0, 0);   // pull the pin low
	DWT_Delay_us (18000);   // wait for 18ms
	set_gpio_input ();   // set as input
}

void check_response (void)
{
	DWT_Delay_us (40);
	if (!(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_0)))  // if the pin is low
	{
		DWT_Delay_us (80);  // wait for 80us
		if ((HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_0))) check = 1;  // now if the pin is high response = ok i.e. check =1
	}
	while ((HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_0)));   // wait for the pin to go low
}

uint8_t read_data (void)
{
	uint8_t i=0,j=0;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_0)));   // wait for the pin to go high
		DWT_Delay_us (40);   // wait for 40 us
		if ((HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_0)) == 0)   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_0)));  // wait for the pin to go low
	}
	return i;
}
void send_char(char c){
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, 1);
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_RESET);
}
int __io_putchar(int ch){
	if (ch == '\n'){
		send_char('\r');
	}
	send_char(ch);
	return ch;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	control_procedure();
	HAL_UART_Receive_IT(&huart2, &received, 1);
}
void control_procedure(void){
	static uint8_t index=0, command=0;
	static uint8_t duty=0;
	uint8_t value=received;
	switch(index){
	case 0:
		if(value=='s'){
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
		} else if(value=='v'||value=='h'||(value>='1'&&value<='5')){
			if(engines_flag==0){
				printf("Silniki jeszcze nie sa gotowe.\n");
			} else{
				command=value;
				printf("Podaj wypelnienie\n");
				index=1;
			}
		} else{
			printf("Niewlasciwa komenda.\n");
			commands_manual();
		}
		break;
	case 1:
		if(value=='s'){
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
		} else if(value<'5'||value>'9'){
			printf("Niewlasciwy znak.\n");
			printf("Obslugiwane wypelnienie 50-99%%.\n");
		} else{
			duty=(value-'0')*10;
			index=2;
		}
		break;
	case 2:
		if(value=='s'){
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
		} else if(value<'0'||value>'9'){
			printf("Niewlasciwy znak.\n");
			printf("Obslugiwane wypelnienie 50-99%%.\n");
		} else{
			duty+=(value-'0');
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
	printf("s - zatrzymanie wszystkich silnikow\n");
	printf("v - sterowanie silnikami pionowymi\n");
	printf("h - sterowanie silnikami poziomymi\n");
	printf("1 - sterowanie silnikiem vf\n");
	printf("2 - sterowanie silnikiem vrl\n");
	printf("3 - sterowanie silnikiem vrr\n");
	printf("4 - sterowanie silnikiem hl\n");
	printf("5 - sterowanie silnikiem hr\n\n\n");
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
