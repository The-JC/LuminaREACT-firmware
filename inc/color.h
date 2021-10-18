/*******************************************************************************
 * Copyright (C) 2021 Julian Hellner - All Rights Reserved
 * 
 * The file color.h is part of LuminaREACT.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Julian Hellner <hellnerjulian@gmail.com>, 13.02.2021
 *
 ******************************************************************************/

#ifndef _COLOR_H_
#define _COLOR_H_

#include "stdint.h"

typedef struct{
	uint8_t R;
	uint8_t G;
	uint8_t B;
} rgb_t;

typedef struct{
	uint8_t h;
	uint8_t s;
	uint8_t v;
} hsv_t;

rgb_t HSV_to_RGB(hsv_t hsv);
hsv_t RGB_to_HSV(rgb_t rgb);

#endif /* INC_COLOR_H_ */
