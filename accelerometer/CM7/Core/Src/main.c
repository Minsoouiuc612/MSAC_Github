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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
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
	/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;
	/* USER CODE END Boot_Mode_Sequence_0 */

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* Wait until CPU2 boots and enters in stop mode or timeout*/
	timeout = 0xFFFF;
	while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
	if ( timeout < 0 )
	{
		Error_Handler();
	}
	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	int _write(int le, char *ptr, int len)
	{
		int DataIdx;
		for(DataIdx = 0; DataIdx < len; DataIdx++)
		{
			ITM_SendChar(*ptr++);
		}

		return len;

	}

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0,0);
	/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
	if ( timeout < 0 )
	{
		Error_Handler();
	}
	/* USER CODE END Boot_Mode_Sequence_2 */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, (0b00011111 << 1) + 0, 1, 100);

	if(ret == HAL_OK)
		printf("Device is recognized \n");
	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 1); // yellow
	else
		printf("Device is not recognized \n");

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1); //green

	//	HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c,
	//					 uint16_t DevAddress,
	//					 uint16_t MemAddress,
	//					 uint16_t MemAddSize,
	//					 uint8_t *pData,
	//					 uint16_t Size,
	//					 uint32_t Timeout)

	// First, read what's in register already then print it
	uint8_t cntl1 = 0;
	HAL_I2C_Mem_Read(&hi2c1, (0b00011111 << 1) + 1, 0x1B, 1, &cntl1, 1, 100);
	printf("cntl1 initial: %d \n", cntl1);

	// Reset register
	cntl1 = 0;
	HAL_I2C_Mem_Write(&hi2c1, (0b00011111 << 1) + 0, 0x1B, 1, &cntl1, 1, 100);

	// %%%%%%%%%% Setting up ODCNTL register %%%%%%%%%%
	uint8_t odcntl = 0b00000110;
	HAL_I2C_Mem_Write(&hi2c1, (0b00011111 << 1) + 0, 0x21, 1, &odcntl, 1, 100);

	odcntl = odcntl | 0b00001101;
	HAL_I2C_Mem_Write(&hi2c1, (0b00011111 << 1) + 0, 0x21, 1, &odcntl, 1, 100);

	HAL_I2C_Mem_Read(&hi2c1, (0b00011111 << 1) + 1, 0x21, 1, &odcntl, 1, 100);
	printf("odcntl after settings: %d \n", odcntl);
	// %%%%%%%%%% Setting up ODCNTL register %%%%%%%%%%

	// Read for debugging purposes
	HAL_I2C_Mem_Read(&hi2c1, (0b00011111 << 1) + 1, 0x1B, 1, &cntl1, 1, 100);
	printf("cntl1 after reset: %d \n", cntl1);

	// Then, set everything we want besides most significant bit
	uint8_t new_cntl1 = 0b01010000 | cntl1; // +- 32 g range
	//	uint8_t new_cntl1 = 0b01001000 | cntl1; // +- 16 g range
	HAL_I2C_Mem_Write(&hi2c1, (0b00011111 << 1) + 0, 0x1B, 1, &new_cntl1, 1, 100);

	// Read for debugging purposes
	HAL_I2C_Mem_Read(&hi2c1, (0b00011111 << 1) + 1, 0x1B, 1, &cntl1, 1, 100);
	printf("cntl1 after settings before enable: %d \n", cntl1);

	// Set the most significant bit to 1 to enable accelerometer
	new_cntl1 = 0b10000000 | cntl1;
	HAL_I2C_Mem_Write(&hi2c1, (0b00011111 << 1) + 0, 0x1B, 1, &new_cntl1, 1, 100);

	// Read for debugging purposes
	HAL_I2C_Mem_Read(&hi2c1, (0b00011111 << 1) + 1, 0x1B, 1, &cntl1, 1, 100);
	printf("cntl1 after enable: %d \n", cntl1);

	// vars for while loop
	uint8_t data[6];
	int16_t x_acc = 0;
	int16_t y_acc = 0;
	int16_t z_acc = 0;
	float x_accel_converted = 0.0;
	float y_accel_converted = 0.0;
	float z_accel_converted = 0.0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_I2C_Mem_Read(&hi2c1, (0b00011111 << 1) + 1, 0x08, 1, data, 6, 100);

		x_acc = (data[1] << 8) + data[0];
		x_accel_converted = x_acc * ((float)(31.99902 / 32767.0));
		//		x_accel_converted = x_acc * ((float)(15.99951 / 32767.0));

		y_acc = (data[3] << 8) + data[2];
		y_accel_converted = y_acc * ((float)(31.99902 / 32767.0));
		//		y_accel_converted = y_acc * ((float)(15.99951 / 32767.0));

		z_acc = (data[5] << 8) + data[4];
		z_accel_converted = z_acc * ((float)(31.99902 / 32767.0));
		//		z_accel_converted = z_acc * ((float)(15.99951 / 32767.0));

		printf("X Accel: %f, Y Accel: %f, Z Accel: %f \n", x_accel_converted, y_accel_converted, z_accel_converted);
		//		printf("Z Accel float: %0.6f, Z Accel int: %d, MSB: %d, LSM: %d \n", z_accel_converted, z_acc, data[1], data[0]);
		HAL_Delay(1);
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

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
	hi2c1.Init.Timing = 0x00707CBB;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PE1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	(void)file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
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
