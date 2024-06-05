/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"
#include "AS5048A.h"
#include <math.h>
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

/* USER CODE BEGIN PV */
uint8_t TxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};

uint16_t input16[16] = {1800, 1811, 1023, 900, 1900, 400, 1500, 1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100};
uint8_t output8[32];
uint16_t value = 170;
uint8_t direction = 1;

uint32_t ICValue =0;
uint32_t Frequency = 0;
float Duty =0;

volatile float position = 0.0f;
volatile float last_position = 0.0f;
volatile float speed= 1023.0f;
const float max_position = 2000.0f;
const float max_rpm = 1000.0f;
const float max_speed = (max_rpm / 60.0f) * max_position;
const float dead_band = 2.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void convert16to8(const uint16_t *input16, uint8_t *output8);
void convert8to16(const uint8_t *input8, uint16_t *output16);
uint16_t map_position_to_sbus(int32_t position);
uint16_t map_pwm_to_sbus(int32_t position);
float CalculateAngularSpeed(float current_position, float last_position, float delta_time);
float map(float x, float in_min, float in_max, float out_min, float out_max);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Odczyt pozycji enkodera PWM
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		ICValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	}
	if(ICValue != 0)
	{
		Duty = (HAL_TIM_ReadCapturedValue(htim,  TIM_CHANNEL_2)*2000)/ICValue;
		Frequency = 72000000/ICValue;
	}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  NRF24_Init();
  NRF24_TxMode(TxAddress, 10);
  AS5048A_Init(&hspi3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
  uint32_t last_tick = HAL_GetTick();
  uint32_t last_tick1 = HAL_GetTick();
  uint32_t last_tick2 = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Symulacja 1 kanału SBUS
	  if (HAL_GetTick() - last_tick >= 5)
	  {
		  last_tick = HAL_GetTick();
		  if (direction == 1) {
					value++;
					if (value >= 1800) {
						direction = -1;
					}
				}
				else {
					value--;
					if (value <= 170) {
						direction = 1;
					}
				}
				input16[0] = value;
	  }
	  	//uint16_t position = AS5048A_GetRaw();
	  	//input16[1] = map_position_to_sbus(position);
	  	//input16[1] = map_pwm_to_sbus(Duty);
	  	//input16[1] = angular_velocity;

	  //Praca korby w trybie SPEED
	  if (HAL_GetTick() - last_tick1 >= 10)
		{
			last_tick1 = HAL_GetTick();
			position = Duty;
			if (fabs(position - last_position) > dead_band)
			{
				speed = CalculateAngularSpeed(position, last_position, 0.01f); // 10 ms
				last_position = position;
				input16[1] = (uint16_t)speed;
			}
			else
			{
				speed = 1023;
				input16[1] = (uint16_t)speed;
			}
		}

		// Transmisja nRF24
		if (HAL_GetTick() - last_tick2 >= 10)
		{
			last_tick2 = HAL_GetTick();
			convert16to8(input16, output8);
			NRF24_Transmit(output8);
		}
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void convert16to8(const uint16_t *input16, uint8_t *output8) {
    for (int i = 0; i < 16; ++i) {
        output8[2 * i] = (uint8_t)(input16[i] & 0x00FF);      // Dolny bajt
        output8[2 * i + 1] = (uint8_t)((input16[i] >> 8) & 0x00FF);  // Górny bajt
    }
}

void convert8to16(const uint8_t *input8, uint16_t *output16) {
    for (int i = 0; i < 16; ++i) {
        output16[i] = (uint16_t)input8[2 * i] | ((uint16_t)input8[2 * i + 1] << 8);
    }
}

uint16_t map_position_to_sbus(int32_t position) {
    // Zakresy wejściowy i wyjściowy
    int32_t input_min = 0;
    int32_t input_max = 16384;
    int32_t output_min = 192;
    int32_t output_max = 1792;

    // Mapowanie z użyciem proporcji
    int32_t sbus = output_min + ((position - input_min) * (output_max - output_min)) / (input_max - input_min);

    // Zwrócenie zmiennej sbus jako uint16_t
    return (uint16_t)sbus;
}

uint16_t map_pwm_to_sbus(int32_t position) {
    // Zakresy wejściowy i wyjściowy
    int32_t input_min = 0;
    int32_t input_max = 2000;
    int32_t output_min = 192;
    int32_t output_max = 1792;

    // Mapowanie z użyciem proporcji
    int32_t sbus = output_min + ((position - input_min) * (output_max - output_min)) / (input_max - input_min);

    // Zwrócenie zmiennej sbus jako uint16_t
    return (uint16_t)sbus;
}

float CalculateAngularSpeed(float current_position, float last_position, float delta_time) {
    float delta_position = current_position - last_position;

    if (delta_position > (max_position / 2)) {
        delta_position -= max_position;
    } else if (delta_position < -(max_position / 2)) {
        delta_position += max_position;
    }

    // Obliczenie prędkości kątowej
    float speed = delta_position / delta_time;

    // Ustaw prędkość zgodnie z kierunkiem zmiany position
    if (speed > 0) {
    	return map(speed, 0, max_speed, 1024, 1792);
    } else {
    	return map(speed, -max_speed, 0, 192, 1022);
    }
}


float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
