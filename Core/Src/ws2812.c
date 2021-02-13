/*******************************************************************************
 * Copyright (C) 2021 Julian Hellner - All Rights Reserved
 * 
 * The file ws2812.c is part of LuminaREACT.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Julian Hellner <hellnerjulian@gmail.com>, Feb 11, 2021
 *
 ******************************************************************************/

#include "ws2812.h"

#include "string.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"

#define CMPH 73-1
#define CMPL 36-1

#define DMA_HT 0
#define DMA_TC 1

static uint16_t tmp_buffer[BUFFER_LENGTH_PER_LED*2];

#define WS2812_RESET_NONE 0
#define WS2812_RESET_START 1
#define WS2812_RESET_END 2

static uint8_t is_updating;
static uint8_t is_reseting;
static uint32_t index_led;

void ws2812_transfer(uint8_t flag);
void ws2812_to_buffer(uint32_t idx, uint16_t *ptr);
static void ws2812_start_reset(uint8_t state);
static void ws2812_clear_flags(void);
static void ws2812_disable_output(void);

static LL_TIM_InitTypeDef TIM_init = {
		.Prescaler = 0,
		.CounterMode = LL_TIM_COUNTERMODE_UP,
//		.Autoreload = 111,
		.Autoreload = 110,
		.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
};
static LL_TIM_OC_InitTypeDef TIM_OC_init = {
		.OCMode = LL_TIM_OCMODE_PWM1,
		.OCState = LL_TIM_OCSTATE_DISABLE,
		.OCNState = LL_TIM_OCSTATE_DISABLE,
		.OCPolarity = LL_TIM_OCPOLARITY_HIGH,
		.CompareValue = 36,
};

static LL_GPIO_InitTypeDef GPIO_init = {
		.Pin = LL_GPIO_PIN_6,
		.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_2
};

void ws2812_init() {
	is_updating = 0;
	is_reseting = WS2812_RESET_NONE;

	// Enable peripheral clock
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	// Init DMA
	LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_4, LL_DMA_CHANNEL_5);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_4, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MODE_CIRCULAR);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_HALFWORD);
	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_4);

	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)&TIM3->CCR1);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)buffer);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, BUFFER_SIZE);

	NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(TIM3_IRQn);

	// Init Timer 3
	LL_TIM_Init(TIM3, &TIM_init);
	LL_TIM_EnableARRPreload(TIM3);
	LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);

	// Init output compare in PWM1 mode on channel 1
	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_init);
	LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM3);

	// Enable GPIO peripheral clock
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	// Init GPIO pin PA6
	LL_GPIO_Init(GPIOA, &GPIO_init);

	LL_TIM_OC_SetCompareCH1(TIM3, 56);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableDMAReq_CC1(TIM3);

	 NVIC_SetPriority(DMA1_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
	 NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

void ws2812_set_rgb(uint32_t i, rgb_t color) {
	leds[i*BYTES_PER_LED+0] = color.R;
	leds[i*BYTES_PER_LED+1] = color.G;
	leds[i*BYTES_PER_LED+2] = color.B;
}
void ws2812_set_hsv(uint32_t i, hsv_t color) {
	ws2812_set_rgb(i, HSV_to_RGB(color));
}

void ws2812_clear() {
	memset(&leds, 0, sizeof(leds));
}

uint8_t ws2812_update() {
	if(is_updating) return 0;
	is_updating = 1;

	ws2812_start_reset(WS2812_RESET_START);
	return 1;
}

void ws2812_to_buffer(uint32_t idx, uint16_t *ptr) {
	uint32_t i;

	assert_param(idx < NUMBER_OF_LEDS);

	for(i=0; i<8; i++) {
		ptr[i + 0*8] = (leds[idx * BYTES_PER_LED + 1] & (1 << (7-i))) ? CMPH : CMPL; // GREEN
		ptr[i + 1*8] = (leds[idx * BYTES_PER_LED + 0] & (1 << (7-i))) ? CMPH : CMPL; // RED
		ptr[i + 2*8] = (leds[idx * BYTES_PER_LED + 2] & (1 << (7-i))) ? CMPH : CMPL; // BLUE

	}
}

void ws2812_transfer(uint8_t flag) {
	if(is_reseting == WS2812_RESET_END) {
		// Disable timer output and DMA stream
		ws2812_disable_output();
		is_updating = 0;
		return;
	}

	if(is_reseting == WS2812_RESET_START) {
		if(!flag) return;

		// Disable timer output and DMA stream
//		ws2812_disable_output();
		is_reseting = 0;
		index_led = 0;
	} else {
		// Increment led index when not in reset
		++index_led;
	}

	if(index_led < NUMBER_OF_LEDS) {
		if(index_led == 0 || !flag) {
			ws2812_to_buffer(index_led, &tmp_buffer[0]);
		} else {
			ws2812_to_buffer(index_led, &tmp_buffer[BUFFER_LENGTH_PER_LED]);
		}

		if(index_led == 0) {
			index_led++;

			// Set DMA to circular mode and length to 48 bytes for 2 leds
			LL_DMA_SetMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MODE_CIRCULAR);
			LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)tmp_buffer);
			LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, BUFFER_LENGTH_PER_LED * 2);

			// Clear flags, enable interupt, stream and timer output
			ws2812_clear_flags();
			LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_4);
			LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
			LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);

			ws2812_to_buffer(index_led, &tmp_buffer[BUFFER_LENGTH_PER_LED]);
		}
	} else if((!flag && (NUMBER_OF_LEDS & 0x01)) || (flag && !(NUMBER_OF_LEDS & 0x01))) {
		ws2812_disable_output();

		ws2812_start_reset(WS2812_RESET_END);
	}
}

static void ws2812_start_reset(uint8_t state) {
	is_reseting = state;

	memset(tmp_buffer, 0, sizeof(tmp_buffer));

	if(state == WS2812_RESET_START) {
		tmp_buffer[0] = TIM3->ARR / 2;
	}

	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)tmp_buffer);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, RESET_LEN);

	ws2812_clear_flags();
	LL_DMA_DisableIT_HT(DMA1, LL_DMA_STREAM_4);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_4);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);

	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM3);
}

void ws2812_shift(uint32_t shift) {
	uint32_t i;
	for(i=NUMBER_OF_LEDS; i > 0; i--) {
		if(i>shift*BYTES_PER_LED) {
			leds[i*BYTES_PER_LED + 0 - 1] = leds[i*BYTES_PER_LED + 0 - 1 - shift*BYTES_PER_LED];
			leds[i*BYTES_PER_LED + 1 - 1] = leds[i*BYTES_PER_LED + 1 - 1 - shift*BYTES_PER_LED];
			leds[i*BYTES_PER_LED + 2 - 1] = leds[i*BYTES_PER_LED + 2 - 1 - shift*BYTES_PER_LED];
		} else {
			leds[i*BYTES_PER_LED + 0 - 1] = 0;
			leds[i*BYTES_PER_LED + 1 - 1] = 0;
			leds[i*BYTES_PER_LED + 2 - 1] = 0;
		}
	}
}

void DMA1_Stream4_IRQHandler(void) {
	if(LL_DMA_IsActiveFlag_HT4(DMA1)) {			// Check HT(Halft Transfer) interupt
		LL_DMA_ClearFlag_HT4(DMA1);
		ws2812_transfer(DMA_HT);
	} else if(LL_DMA_IsActiveFlag_TC4(DMA1)) {	// Check TC(Transfer complete) interupt
		LL_DMA_ClearFlag_TC4(DMA1);
		ws2812_transfer(DMA_TC);
	}
}

static void ws2812_clear_flags(void) {
	LL_DMA_ClearFlag_HT4(DMA1);
	LL_DMA_ClearFlag_TC4(DMA1);
}

static void ws2812_disable_output(void) {
	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);
}

