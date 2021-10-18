/*******************************************************************************
 * Copyright (C) 2019 Julian Hellner - All Rights Reserved
 * 
 * The file cat5171.c is part of GainAmpliefier.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Julian Hellner <hellnerjulian@gmail.com>, Sep 24, 2021
 *
 ******************************************************************************/
#include "cat5171.h"


/**
 * @brief builds the instruction byte for writting to the CAT5171
 *
 * @param RS: Set the reset bit
 * @param SD: Set the shutdown bit
 * @return Instruction byte
 */
uint8_t CAT5171_Instruction(uint8_t RS, uint8_t SD);

CAT5171_RESULT_t CAT5171_Init(void) {
	if(I2C_Init(CAT5171, CAT5171_PINGROUP, CAT5171_CLOCK) != I2C_Result_OK) {
		return CAT5171_RESULT_ERROR;
	}

	return CAT5171_RESULT_OK;
}

CAT5171_RESULT_t CAT5171_SetWiper(uint8_t tap) {
	uint8_t data[2];

	data[0] = CAT5171_Instruction(0, 0);
	data[1] = tap;

	if(I2C_WriteMultiNoRegister(CAT5171, CAT5171_ADDRESS, (uint8_t*) data, 2) != I2C_Result_OK) {
		return CAT5171_RESULT_ERROR;
	}

	return CAT5171_RESULT_OK;
}

CAT5171_RESULT_t CAT5171_GetWiper(uint8_t* tap) {
	uint8_t data;

	if(I2C_ReadNoRegister(CAT5171, CAT5171_ADDRESS, &data) != I2C_Result_OK) {
		return CAT5171_RESULT_ERROR;
	}

	return CAT5171_RESULT_OK;
}

uint8_t CAT5171_Instruction(uint8_t RS, uint8_t SD) {
	uint8_t instruction = 0x0;
	instruction |= RS << 7;
	instruction |= SD << 6;
	return instruction;
}
