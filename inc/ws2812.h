/*******************************************************************************
 * Copyright (C) 2021 Julian Hellner - All Rights Reserved
 * 
 * The file ws2812.h is part of LuminaREACT.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Julian Hellner <hellnerjulian@gmail.com>, Feb 11, 2021
 *
 ******************************************************************************/

#ifndef _WS2812_H_
#define _WS2812_H_

#include "color.h"
#include "stm32f4xx.h"

#define NUMBER_OF_LEDS 152
#define BYTES_PER_LED 3

#define RESET_LEN 40 //20*1.25µs = 50µs

#define BUFFER_LENGTH_PER_LED (BYTES_PER_LED*8)

uint8_t leds[BYTES_PER_LED*NUMBER_OF_LEDS];

void ws2812_init();
void ws2812_set_rgb(uint32_t i, rgb_t color);
void ws2812_set_hsv(uint32_t i, hsv_t color);
void ws2812_clear(void);
uint8_t ws2812_update(void);
void ws2812_shift(uint32_t shift);

void DMA1_Stream4_IRQHandler(void);

#endif /* INC_WS2812_H_ */
