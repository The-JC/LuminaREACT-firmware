/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "arm_math.h"
#include "arm_const_structs.h"
#include "cat5171.h"
#include "test_signal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define TRANSMIT_UART
#define TRANSMIT_FFT

#define FLOAT_FFT

//#define TEST_SIGNAL

#define FFT_SIZE 4096
#define ADC_BUF_LEN FFT_SIZE*2

#define START_BYTE 0xFEFEFEFE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
int16_t adc_buf[ADC_BUF_LEN];
#ifndef FLOAT_FFT
q15_t fft_work_buf[FFT_SIZE];
q15_t fft_buf[FFT_SIZE];
arm_rfft_instance_q15 fft;
uint16_t tx_data[FFT_SIZE*2+1];
#else
float32_t fft_buf_in[FFT_SIZE];
float32_t fft_buf_out[FFT_SIZE];
arm_rfft_fast_instance_f32 fft;
uint32_t tx_data[FFT_SIZE+1];
#endif


arm_status fft_status;
uint8_t transmitting = 0;
uint8_t state = 0; //0 -no fft ir, 1- first half, 2- second half
uint32_t counter_fft = 0;

uint8_t UART2_rxBuffer[1] = {0};

uint8_t setWiper = 50;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void performFFT(void);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Init real FFT
#ifndef FLOAT_FFT
  arm_status status = ARM_MATH_SUCCESS;
  status = arm_rfft_init_q15(&fft, FFT_SIZE, 0, 0);
#else
  arm_rfft_fast_init_f32 (&fft, FFT_SIZE);
#endif

  // Init programmable Potentiometer
  CAT5171_Init();
  CAT5171_SetWiper(setWiper);

  // Start ADC to Memory DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

  HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(LD_User_GPIO_Port, LD_User_Pin);
	  CAT5171_SetWiper(setWiper);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.BaudRate = 921600 ;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD_User_Pin|LD_Sig_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : LD_User_Pin LD_Sig_Pin */
  GPIO_InitStruct.Pin = LD_User_Pin|LD_Sig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Called when first half of buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	//HAL_DMA_Start(&hdma_usart2_tx, (uint32_t*)&adc_buf[0], (uint32_t)&huart2.Instance->DR, ADC_BUF_LEN/2);

#ifdef TRANSMIT_UART
	memcpy(tx_data, adc_buf, sizeof(adc_buf)/2);
	tx_data[FFT_SIZE] = 0x0800;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&tx_data, sizeof(tx_data));
	return;
#endif /* TRANSMIT_UART */
	if(state == 0) {
		state = 1;
		counter_fft++;

#ifdef TEST_SIGNAL
		memcpy(adc_buf, test_signal, sizeof(test_signal));
#endif /* TEST_SIGNAL */

#ifndef FLOAT_FFT
		memcpy(fft_work_buf, adc_buf, FFT_SIZE);
		arm_shift_q15(fft_work_buf, 3, fft_work_buf, FFT_SIZE);

		fft_status = ARM_MATH_SUCCESS;
		fft_status = arm_rfft_init_q15(&fft, FFT_SIZE, 0, 0);
		// perform FFT on first half of the input buffer
		arm_rfft_q15(&fft, fft_work_buf, fft_buf);
#else
		for(uint16_t i=0; i< FFT_SIZE; i++) {
			fft_buf_in[i] = ((float) adc_buf[i])-2048;
		}

		arm_rfft_fast_f32(&fft, fft_buf_in, fft_buf_out, 0);
#endif /* FLOAT FFT */

#ifdef TRANSMIT_FFT
		if(!transmitting) {
			transmitting=1;
#ifndef FLOAT_FFT
			memcpy(tx_data+1, fft_buf, sizeof(fft_buf));
//			arm_shift_q15(fft_buf, 11, fft_buf, FFT_SIZE);
			arm_cmplx_mag_q15(fft_buf, tx_data+1, FFT_SIZE);
#else
			arm_cmplx_mag_f32(fft_buf_out, (float32_t*) tx_data+1, FFT_SIZE);

#endif /* FLOAT_FFT */
			tx_data[0] = START_BYTE;
//			tx_data[1] = 0x40b00000;
//			tx_data[2] = 0x41280000;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&tx_data, sizeof(float32_t)*(FFT_SIZE/2+1));
		}
#endif /* TRANSMIT_FFT */
		state = 0;
	}
}

// Called when buffer is completly filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	//HAL_DMA_Start(&hdma_usart2_tx, (uint32_t*)&adc_buf[ADC_BUF_LEN/2], (uint32_t)&huart2.Instance->DR, ADC_BUF_LEN/2);

#ifdef TRANSMIT_UART
	memcpy(tx_data, adc_buf + ADC_BUF_LEN/2, sizeof(adc_buf)/2);
	tx_data[FFT_SIZE] = 0xFFFF;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&tx_data, sizeof(tx_data));
	return;
#endif /* TRANSMIT_UART */

	if(state == 0) {
		state = 2;
		counter_fft++;

#ifdef TEST_SIGNAL
		memcpy(adc_buf +ADC_BUF_LEN/2, test_signal, sizeof(test_signal));
#endif /* TEST_SIGNAL */

#ifndef FLOAT_FFT
		arm_shift_q15(adc_buf +ADC_BUF_LEN/2, 3, fft_work_buf, FFT_SIZE);

		fft_status = ARM_MATH_SUCCESS;
		fft_status = arm_rfft_init_q15(&fft, FFT_SIZE, 0, 0);
		// perform FFT on second half of the input buffer
		arm_rfft_q15(&fft, fft_work_buf +ADC_BUF_LEN/2, fft_buf);
#else
		for(uint16_t i=0; i< FFT_SIZE; i++) {
			fft_buf_in[i] = ((float) adc_buf[i+FFT_SIZE])-2048;
		}

		arm_rfft_fast_f32 (&fft, fft_buf_in, fft_buf_out, 0);
#endif /* FLOAT FFT */

#ifdef TRANSMIT_FFT
		if(!transmitting) {
			transmitting=1;
#ifndef FLOAT_FFT
			memcpy(tx_data+1, fft_buf, sizeof(fft_buf));
//			arm_shift_q15(fft_buf, 11, fft_buf, FFT_SIZE);
			arm_cmplx_mag_q15(fft_buf, tx_data+1, FFT_SIZE);
#else
			arm_cmplx_mag_f32(fft_buf_out, (float32_t*) tx_data+1, FFT_SIZE);

#endif /* FLOAT_FFT */
			tx_data[0] = START_BYTE;
//			tx_data[1] = 0x40b00000;
//			tx_data[2] = 0x41280000;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&tx_data, sizeof(float32_t)*(FFT_SIZE/2+1));
		}
#endif /* TRANSMIT_FFT */
		state = 0;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	transmitting=0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	CAT5171_SetWiper(UART2_rxBuffer[0]);
//	HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 1);
}

void performFFT(void) {

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
