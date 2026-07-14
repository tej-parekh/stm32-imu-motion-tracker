/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MPU6050_I2C_ADDR       (0x68 << 1)
#define MPU6050_WHO_AM_I_REG   0x75
#define MPU6050_WHO_AM_I_VALUE 0x68

#define MPU6050_PWR_MGMT_1_REG  0x6B
#define MPU6050_WAKE_VALUE      0x00

#define MPU6050_ACCEL_XOUT_H_REG   0x3B
#define MPU6050_ACCEL_SENSITIVITY 16384.0f

#define MPU6050_GYRO_XOUT_H_REG     0x43
#define MPU6050_GYRO_SENSITIVITY    131.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  uint8_t who_am_i = 0;

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
      &hi2c1,
      MPU6050_I2C_ADDR,
      MPU6050_WHO_AM_I_REG,
      I2C_MEMADD_SIZE_8BIT,
      &who_am_i,
      1,
      100
  );


  if (status == HAL_OK)
  {
	  char uart_buffer[50];
      int message_length = snprintf(
          uart_buffer,
          sizeof(uart_buffer),
		  "WHO_AM_I = 0x%02X\r\n",
          who_am_i
      );

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)uart_buffer,
          message_length,
          HAL_MAX_DELAY
      );
  }

  else
  {
      char error_message[] = "MPU-6050 read failed\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)error_message,
          sizeof(error_message) - 1,
          HAL_MAX_DELAY
      );
  }

  uint8_t wake_value = MPU6050_WAKE_VALUE;

  HAL_StatusTypeDef wake_status = HAL_I2C_Mem_Write(
      &hi2c1,
      MPU6050_I2C_ADDR,
      MPU6050_PWR_MGMT_1_REG,
      I2C_MEMADD_SIZE_8BIT,
      &wake_value,
      1,
      100
  );

  if (wake_status == HAL_OK)
  {
      char wake_message[] = "MPU-6050 awake\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)wake_message,
          sizeof(wake_message) - 1,
          HAL_MAX_DELAY
      );
  }
  else
  {
      char wake_error[] = "MPU-6050 wake failed\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)wake_error,
          sizeof(wake_error) - 1,
          HAL_MAX_DELAY
      );
  }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  uint8_t accel_data[6];

	  HAL_StatusTypeDef accel_status = HAL_I2C_Mem_Read(
	      &hi2c1,
	      MPU6050_I2C_ADDR,
	      MPU6050_ACCEL_XOUT_H_REG,
	      I2C_MEMADD_SIZE_8BIT,
	      accel_data,
	      6,
	      100
	  );

	  if (accel_status == HAL_OK)
	  {
		  int16_t accel_x = (int16_t)((accel_data[0] << 8) | accel_data[1]);
		  int16_t accel_y = (int16_t)((accel_data[2] << 8) | accel_data[3]);
		  int16_t accel_z = (int16_t)((accel_data[4] << 8) | accel_data[5]);

		  float accel_x_g = accel_x / MPU6050_ACCEL_SENSITIVITY;
		  float accel_y_g = accel_y / MPU6050_ACCEL_SENSITIVITY;
		  float accel_z_g = accel_z / MPU6050_ACCEL_SENSITIVITY;

		  char accel_buffer[100];

		  int message_length = snprintf(
		      accel_buffer,
		      sizeof(accel_buffer),
		      "Accel (g): X=%.3f Y=%.3f Z=%.3f\r\n",
		      accel_x_g,
		      accel_y_g,
		      accel_z_g
		  );

		  HAL_UART_Transmit(
		      &huart2,
		      (uint8_t *)accel_buffer,
		      message_length,
		      HAL_MAX_DELAY
		  );
	  }
	  else
	  {
	      char accel_error[] = "Accel reading failed\r\n";

	      HAL_UART_Transmit(
	          &huart2,
	          (uint8_t *)accel_error,
	          sizeof(accel_error) - 1,
	          HAL_MAX_DELAY
	      );
	  }

	  uint8_t gyro_data[6];

	  HAL_StatusTypeDef gyro_status = HAL_I2C_Mem_Read(
	      &hi2c1,
	      MPU6050_I2C_ADDR,
	      MPU6050_GYRO_XOUT_H_REG,
	      I2C_MEMADD_SIZE_8BIT,
	      gyro_data,
	      6,
	      100
	  );

	  if (gyro_status == HAL_OK) {
		  int16_t gyro_x_raw =
		  	      (int16_t)((gyro_data[0] << 8) | gyro_data[1]);

		  int16_t gyro_y_raw =
		  	      (int16_t)((gyro_data[2] << 8) | gyro_data[3]);

		  int16_t gyro_z_raw =
		  	      (int16_t)((gyro_data[4] << 8) | gyro_data[5]);

		  float gyro_x_dps = gyro_x_raw / MPU6050_GYRO_SENSITIVITY;
		  float gyro_y_dps = gyro_y_raw / MPU6050_GYRO_SENSITIVITY;
		  float gyro_z_dps = gyro_z_raw / MPU6050_GYRO_SENSITIVITY;

		  char gyro_buffer[100];

		  int gyro_message_length = snprintf(gyro_buffer,
				  sizeof(gyro_buffer),
				  "Gyro (deg/s): X=%.2f Y=%.2f Z=%.2f\r\n",
				  gyro_x_dps,
				  gyro_y_dps,
				  gyro_z_dps
		      	  );

		  HAL_UART_Transmit(
		          &huart2,
		          (uint8_t *)gyro_buffer,
		          gyro_message_length,
		          HAL_MAX_DELAY
		      	  );
	  }

	  else
	  {
	      char gyro_error[] = "Gyro reading failed\r\n";

	      HAL_UART_Transmit(
	          &huart2,
	          (uint8_t *)gyro_error,
	          sizeof(gyro_error) - 1,
	          HAL_MAX_DELAY
	      );
	  }


	  HAL_Delay(1000);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
