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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
 * gnd
 * clk
 * data
 * 3.3v
 * */
#define LED0 0x0002
#define LED1 0x0001
#define LED2 0x0080
#define LED3 0x0040
#define LED4 0x0020
#define OFF 1
#define ON 0
#define OFFSET_LED 500
#define OFFSET_STAGE 1800

#define DEFAULT_SPEED 35

#define WHITE 0 //PC13 LED ON
#define BLACK 1 //PC13 LED OFF

#define ADC_MIN 210
#define ADC_MAX 2900

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ADC_ChannelConfTypeDef sConfig = {0};
uint16_t adc_sensors[5] = {0}; // where adc's results are stored
float normalized_values[5];
float weight[5] = {-2, -1, 0, 1, 2};
uint16_t LED_state[5] = {0};
uint16_t reference_value= 0;

uint16_t state_direction;
/* ------------------------- */
float sum = 0;
float error;
float previousError;
float p, i, d;
float kp = 10;
float ki = 0;
float kd = 0;
int pidOutput = 0;
int leftSpeed = 0;
int rightSpeed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void normalize_sensors() {
    for (int i = 0; i < 5; i++) {
        if (adc_sensors[i] < ADC_MIN)
            normalized_values[i] = 0.0;
        else if (adc_sensors[i] > ADC_MAX)
            normalized_values[i] = 1.0;
        else
            normalized_values[i] = (float)(adc_sensors[i] - ADC_MIN) / (ADC_MAX - ADC_MIN);

        normalized_values[i] = normalized_values[i] * weight[i];
    }
}

void read_ADC() {
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	HAL_ADC_Start(&hadc1);
	for (int i = 0; i < 5; i++) {
		sConfig.Channel = ADC_CHANNEL_0 + i;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		adc_sensors[i] = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
	normalize_sensors();
}
/*
 * sum > 0 : turn right
 * sum < 0 : turn left
 *
 * pidOutput > 0 : turn right (left up ,right down)
 * pidOutput < 0 : turn left (left down, right up)
 * */
void pid() {
	float temp;
	for (int i = 0; i < 5; i++) {
		temp += normalized_values[i];
	}
	sum = temp;

	error = sum; // sum - 0
	p = kp * sum;
	i = i + (ki * error);
	d = kd *(error - previousError);
	previousError = error;

	pidOutput = p + i + d;
	temp = 0;

	if (pidOutput > 100 -DEFAULT_SPEED)
		pidOutput = 100;
	else if (pidOutput < -(100-DEFAULT_SPEED))
		pidOutput = -(100-DEFAULT_SPEED);

	leftSpeed = DEFAULT_SPEED + pidOutput;
	rightSpeed = DEFAULT_SPEED - pidOutput;

	if (leftSpeed < 0)
		leftSpeed = 0;
	if (rightSpeed <0)
		rightSpeed = 0;
	htim3.Instance->CCR2 = leftSpeed; //left
	htim1.Instance->CCR4 = rightSpeed; // right
}

/*
 * brief
 * check if the sensor is on black
 * 1 will be returned if it is on black
 * 0 will be returned if it is not
 */
uint16_t isOnBlack(uint16_t check) {
	if (check > reference_value)
		return 1;
	else
		return 0;
}

/*
 * determine LED's state
 * LED ON when it's on black
 * LED OFF when it's on white
 */
void determine_LED() {
	for (int i = 0; i < 5; i++) {
		// LED ON when it's on black
		if (isOnBlack(adc_sensors[i])) {

			switch(i) {
			case 0 :
				HAL_GPIO_WritePin(GPIOB, LED0, ON);
				continue;
			case 1 :
				HAL_GPIO_WritePin(GPIOB, LED1, ON);
				continue;
			case 2 :
				HAL_GPIO_WritePin(GPIOA, LED2, ON);
				continue;
			case 3 :
				HAL_GPIO_WritePin(GPIOA, LED3, ON);
				continue;
			case 4:
				HAL_GPIO_WritePin(GPIOA, LED4, ON);
				continue;
			}
		}
		else {
			switch(i) {
			case 0 :
				HAL_GPIO_WritePin(GPIOB, LED0, OFF);
				continue;
			case 1 :
				HAL_GPIO_WritePin(GPIOB, LED1, OFF);
				continue;
			case 2 :
				HAL_GPIO_WritePin(GPIOA, LED2, OFF);
				continue;
			case 3 :
				HAL_GPIO_WritePin(GPIOA, LED3, OFF);
				continue;
			case 4:
				HAL_GPIO_WritePin(GPIOA, LED4, OFF);
				continue;
			}
		}
	}
}

void go() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); //in1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1); //in2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0); //in3
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1); //in4
}

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  for (int i = 0; i < 100; i ++) {
	  read_ADC();
	  reference_value += ((adc_sensors[0] + adc_sensors[1] + adc_sensors[2] + adc_sensors[3] + adc_sensors[4]) / 5);
  }
  reference_value = (reference_value / 100) + OFFSET_LED;
  normalize_sensors();
  for (int i = 0; i <4; i ++) {
  	  HAL_GPIO_TogglePin(GPIOB, LED0 | LED1);
  	  HAL_GPIO_TogglePin(GPIOA, LED2 | LED3 | LED4);
  	  HAL_Delay(300);
  }
  go();
//  read_ADC();
//  reference_value = ((adc_sensors[0] + adc_sensors[1] + adc_sensors[2] + adc_sensors[3] + adc_sensors[4]) / 5) + OFFSET;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // blink led
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1);
//	  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);

	  read_ADC();
	  determine_LED();
	  pid();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
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
