/*******************************************************************************
 * Copyright (C) 2021 Julian Hellner - All Rights Reserved
 * 
 * The file color.c is part of LuminaREACT.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Julian Hellner <hellnerjulian@gmail.com>, 13.02.2021
 *
 ******************************************************************************/

#include "color.h"

rgb_t HSV_to_RGB(hsv_t hsv) {
	rgb_t rgb;
	unsigned char region, remainder, p, q, t;

	if (hsv.s == 0) {
		rgb.R = hsv.v;
		rgb.G = hsv.v;
		rgb.B = hsv.v;
		return rgb;
	}

	region = hsv.h / 43;
	remainder = (hsv.h - (region * 43)) * 6;

	p = (hsv.v * (255 - hsv.s)) >> 8;
	q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
	t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

	switch (region) {
		case 0:
			rgb.R = hsv.v; rgb.G = t; rgb.B = p;
			break;
		case 1:
			rgb.R = q; rgb.G = hsv.v; rgb.B = p;
			break;
		case 2:
			rgb.R = p; rgb.G = hsv.v; rgb.B = t;
			break;
		case 3:
			rgb.R = p; rgb.G = q; rgb.B = hsv.v;
			break;
		case 4:
			rgb.R = t; rgb.G = p; rgb.B = hsv.v;
			break;
		default:
			rgb.R = hsv.v; rgb.G = p; rgb.B = q;
			break;
	}

	return rgb;
}

hsv_t RGB_to_HSV(rgb_t rgb){
	hsv_t hsv;
	uint8_t rgbMin, rgbMax;

	rgbMin = rgb.R < rgb.G ? (rgb.R < rgb.B ? rgb.R : rgb.B) : (rgb.G < rgb.B ? rgb.G : rgb.B);
	rgbMax = rgb.R > rgb.G ? (rgb.R > rgb.B ? rgb.R : rgb.B) : (rgb.G > rgb.B ? rgb.G : rgb.B);

	hsv.v = rgbMax;
	if (hsv.v == 0) {
		hsv.h = 0;
		hsv.s = 0;
		return hsv;
	}

	hsv.s = 255 * ((uint32_t)rgbMax - (uint32_t)rgbMin) / hsv.v;
	if (hsv.s == 0) {
		hsv.h = 0;
		return hsv;
	}

	if (rgbMax == rgb.R)
		hsv.h = 0 + 43 * (rgb.G - rgb.B) / (rgbMax - rgbMin);
	else if (rgbMax == rgb.G)
		hsv.h = 85 + 43 * (rgb.B - rgb.R) / (rgbMax - rgbMin);
	else
		hsv.h = 171 + 43 * (rgb.R - rgb.G) / (rgbMax - rgbMin);

	return hsv;
}
