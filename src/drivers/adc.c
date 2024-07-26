#include "adc.h"

#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_dma.h"

void ADC_Init(void) {
	DMA_HandleTypeDef* hdma_adc1 = DMA2_Stream0;
	ADC_HandleTypeDef*  hadc1 = ADC1;
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1->Instance = ADC1;
	hadc1->Init.ClockPrescaler = ADC_CLOCK_PRESCALER;	/// <- Set clock Prescaler to 1/6
	hadc1->Init.Resolution = ADC_RESOULTION_8B;				/// <- Set ADC to 8bit resolution
	hadc1->Init.ScanConvMode = DISABLE;						/// Scan mode isn't needed as we only use one channel for conversion
	hadc1->Init.ContinuousConvMode = ENABLE;				/// <- Activate continous conversion
	hadc1->Init.DiscontinuousConvMode = DISABLE;
	hadc1->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1->Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1->Init.NbrOfConversion = 1;
	hadc1->Init.DMAContinuousRequests = ENABLE;
	hadc1->Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	if(HAL_ADC_Init(hadc1) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLING_TIME; /// Conversion freqeuncy is APB2/

	if(HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// ADC DMA initialization
	hdma_adc1->Instance = DMA2_Stream0;
	hdma_adc1->Init.Channel = DMA_PERIPH_TO_MEMORY;
	hdma_adc1->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc1->Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc1->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_adc1->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_adc1->Init.Mode = DMA_CIRCULAR;
	hdma_adc1->Init.Priority = DMA_PRIORITY_MEDIUM;
	hdma_adc1->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_adc1->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_adc1->Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_adc1->Init.PeriphBurst = DMA_PBURST_SINGLE;
	
	if(HAL_DMA_Init(hdma_adc1) != HAL_OK) {
		Error_Handler();
	}

	__HAL_LINKDMA(hadc1, DMA_Handle, hdma_adc1);

	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
}