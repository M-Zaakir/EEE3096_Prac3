/* USER CODE BEGIN Header F0*/
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

#include <stdint.h>

#define CPU_FREQ_HZ 48000000ULL   /* STM32F0 clock frequency for cycles calculation */
/* USER CODE BEGIN PV */
uint64_t checksum = 0;
uint32_t start_time = 0;
uint32_t end_time = 0;

int image_sizes_w[] = {320, 640, 1280, 1920};
int image_sizes_h[] = {240, 480, 720, 1080};
int num_sizes = 4;


/* Results arrays (one entry per image size) - visible in Live Expressions */
volatile uint32_t exec_time[5]    = {0};   /* wall-clock time in ms */
volatile uint64_t cycles_arr[5]    = {0};   /* estimated CPU cycles */
volatile double throughput_pps_arr[5]    = {0.0}; /* pixels per second (double for readability) */

#define MAX_ITER 100  /* Constraint for Task 3 */
/* ---------------- USER CODE END PV ---------------- */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//TODO: Define any function prototypes you might need such as the calculate Mandelbrot function among others
uint64_t calculate_mandelbrot_fixed_point(int width, int height, int max_iterations);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t calculate_mandelbrot_float_scalable(int width, int height, int max_iterations) {
    uint64_t local_checksum = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float x0 = ((float)x / (float)width) * 3.5f - 2.5f;
            float y0 = ((float)y / (float)height) * 2.0f - 1.0f;
            float xi = 0.0f, yi = 0.0f;
            int iteration = 0;

            while (iteration < max_iterations && (xi*xi + yi*yi) <= 4.0f) {
                float temp = xi*xi - yi*yi + x0;
                yi = 2.0f*xi*yi + y0;
                xi = temp;
                iteration++;
            }
            local_checksum += iteration;
        }
    }
    return local_checksum;
}

uint64_t calculate_mandelbrot_fixed_point(int width, int height,
		int max_iterations) {
	int SCALE = 1000000;  // Fixed-point scaling factor (10^6)
	uint64_t checksum = 0;

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int32_t x0 = ((int64_t) x * 3500000 / width) - 2500000;
			int32_t y0 = ((int64_t) y * 2000000 / height) - 1000000;

			int32_t xi = 0, yi = 0;
			int iteration = 0;

			while (iteration < max_iterations) {
				int64_t x2 = ((int64_t) xi * xi) / SCALE;
				int64_t y2 = ((int64_t) yi * yi) / SCALE;
				if (x2 + y2 > 4000000)
					break;

				int32_t temp = (x2 - y2) + x0;
				yi = (2LL * xi * yi) / SCALE + y0;
				xi = temp;

				iteration++;
			}
			checksum += iteration;
		}
	}
	return checksum;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Visual indicator: Turn on LED0
	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	      for (int i = 0; i < num_sizes; i++) {
	              int width = image_sizes_w[i];
	              int height = image_sizes_h[i];

	              start_time = HAL_GetTick();
	              checksum = calculate_mandelbrot_float_scalable(width, height, 100);
	              end_time = HAL_GetTick();

	              uint32_t elapsed_ms = (end_time >= start_time) ? (end_time - start_time)
	                                                            : (0xFFFFFFFFUL - start_time + end_time + 1);

	              exec_time[i] = elapsed_ms;
	              cycles_arr[i] = (48000000ULL * elapsed_ms) / 1000ULL; // 48 MHz F0
	              throughput_arr[i] = ((double)width * (double)height) / ((double)elapsed_ms / 1000.0);

	              // Observe checksum, exec_time_ms_arr, cycles_arr, throughput_arr in Live Expressions
	          }

	          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // LED1 ON
	          HAL_Delay(2000);
	          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

	      // Optional: stop after one full run
	      // while (1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Function signatures you defined previously , implement them here

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
#ifdef USE_FULL_ASSERT
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
