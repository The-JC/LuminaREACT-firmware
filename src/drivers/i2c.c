/*******************************************************************************
 * Copyright (C) 2019 Julian Hellner - All Rights Reserved
 * 
 * The file i2c.c is part of GainAmpliefier.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Julian Hellner <hellnerjulian@gmail.com>, Sep 24, 2021
 *
 ******************************************************************************/

#include "i2c.h"

#define I2C_TIMEOUT 1000

// Handle values for I2C
#ifdef I2C1
	static I2C_HandleTypeDef I2C1Handle = {I2C1};
#endif
#ifdef I2C2
	static I2C_HandleTypeDef I2C2Handle = {I2C2};
#endif
#ifdef I2C3
	static I2C_HandleTypeDef I2C3Handle = {I2C3};
#endif
#ifdef I2C4
	static I2C_HandleTypeDef I2C4Handle = {I2C4};
#endif

// Private functions
#ifdef I2C1
	static void I2C_I2C1_InitGPIO(I2C_PinGroup_t pinGroup);
#endif
#ifdef I2C2
	static void I2C_I2C2_InitGPIO(I2C_PinGroup_t pinGroup);
#endif
#ifdef I2C3
	static void I2C_I2C3_InitGPIO(I2C_PinGroup_t pinGroup);
#endif
#ifdef I2C4
	static void I2C_I2C4_InitGPIO(I2C_PinGroup_t pinGroup);
#endif

I2C_HandleTypeDef* I2C_GetHandle(I2C_TypeDef* I2Cx) {
	#ifdef I2C1
		if (I2Cx == I2C1) {
			return &I2C1Handle;
		}
	#endif
	#ifdef I2C2
		if (I2Cx == I2C2) {
			return &I2C2Handle;
		}
	#endif
	#ifdef I2C3
		if (I2Cx == I2C3) {
			return &I2C3Handle;
		}
	#endif
	#ifdef I2C4
		if (I2Cx == I2C4) {
			return &I2C4Handle;
		}
	#endif

		// Return invalid
		return 0;
}

static void I2C_Configure(I2C_HandleTypeDef* h, uint32_t clockSpeed) {

	h->Init.OwnAddress1 = 0x0;
	h->Init.OwnAddress2 = 0x0;
	h->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	h->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	h->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	h->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	h->Init.ClockSpeed = clockSpeed;
	h->Init.DutyCycle = I2C_DUTYCYCLE_2;

}

I2C_RESULT_t I2C_Init(I2C_TypeDef* I2Cx, I2C_PinGroup_t pinGroup, uint32_t clockSpeed) {
	I2C_HandleTypeDef* handle = I2C_GetHandle(I2Cx);

	handle->Instance = I2Cx;

#ifdef I2C1
	if(I2Cx == I2C1) {
		// Enable clock
		__HAL_RCC_I2C1_CLK_ENABLE();

		// Enable pins
		I2C_I2C1_InitGPIO(pinGroup);
	}
#endif /* I2C1 */
#ifdef I2C2
	if(I2Cx == I2C2) {
		// Enable clock
		__HAL_RCC_I2C2_CLK_ENABLE();

		// Enable pins
		I2C_I2C2_InitGPIO(pinGroup);
	}
#endif /* I2C2 */
#ifdef I2C3
	if(I2Cx == I2C3) {
		// Enable clock
		__HAL_RCC_I2C3_CLK_ENABLE();

		// Enable pins
		I2C_I2C3_InitGPIO(pinGroup);
	}
#endif /* I2C3 */
#ifdef I2C4
	if(I2Cx == I2C4) {
		// Enable clock
		__HAL_RCC_I2C4_CLK_ENABLE();

		// Enable pins
		I2C_I2C4_InitGPIO(pinGroup);
	}
#endif /* I2C4 */

	I2C_Configure(handle, clockSpeed);

	if(HAL_I2C_Init(handle) != HAL_OK)
		return I2C_Result_ERROR;

	return I2C_Result_OK;
}

I2C_RESULT_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data) {
	I2C_HandleTypeDef* handle = I2C_GetHandle(I2Cx);

	// Send address to peripheral
	if(HAL_I2C_Master_Transmit(handle, (uint16_t) deviceAddress, &registerAddress, 1, I2C_TIMEOUT) != HAL_OK) {

		return I2C_Result_ERROR;
	}

	// Receive data
	if(HAL_I2C_Master_Receive(handle, (uint16_t)deviceAddress, data, 1, I2C_TIMEOUT) != HAL_OK) {
		return I2C_Result_ERROR;
	}

	return I2C_Result_OK;
}

I2C_RESULT_t I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data, uint16_t size) {
	I2C_HandleTypeDef* handle = I2C_GetHandle(I2Cx);

		// Send address to peripheral
		if(HAL_I2C_Master_Transmit(handle, (uint16_t) deviceAddress, &registerAddress, 1, I2C_TIMEOUT) != HAL_OK) {

			return I2C_Result_ERROR;
		}

		// Receive data
		if(HAL_I2C_Master_Receive(handle, (uint16_t)deviceAddress, data, size, I2C_TIMEOUT) != HAL_OK) {
			return I2C_Result_ERROR;
		}

		return I2C_Result_OK;
}

I2C_RESULT_t I2C_ReadNoRegister(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t *data) {
	I2C_HandleTypeDef* handle = I2C_GetHandle(I2Cx);

		// Receive data
		if(HAL_I2C_Master_Receive(handle, (uint16_t)deviceAddress, data, 1, I2C_TIMEOUT) != HAL_OK) {
			return I2C_Result_ERROR;
		}

		return I2C_Result_OK;
}

I2C_RESULT_t I2C_ReadMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t *data, uint16_t size) {
	I2C_HandleTypeDef* handle = I2C_GetHandle(I2Cx);

		// Receive data
		if(HAL_I2C_Master_Receive(handle, (uint16_t)deviceAddress, data, size, I2C_TIMEOUT) != HAL_OK) {
			return I2C_Result_ERROR;
		}

		return I2C_Result_OK;
}

I2C_RESULT_t I2C_Write(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) {
	I2C_HandleTypeDef* handle = I2C_GetHandle(I2Cx);
	uint8_t tmp_data[2] = {registerAddress, data};

	// Send address and data to peripheral
	if(HAL_I2C_Master_Transmit(handle, (uint16_t) deviceAddress, (uint8_t *)tmp_data, 2, I2C_TIMEOUT) != HAL_OK) {

		return I2C_Result_ERROR;
	}

	return I2C_Result_OK;
}

I2C_RESULT_t I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint16_t size) {
	I2C_HandleTypeDef* handle = I2C_GetHandle(I2Cx);

	// Send address and data to peripheral
	if(HAL_I2C_Mem_Write(handle, deviceAddress, registerAddress, registerAddress > 0xFF ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, data, size, I2C_TIMEOUT) != HAL_OK) {

		return I2C_Result_ERROR;
	}

	return I2C_Result_OK;
}

I2C_RESULT_t I2C_WriteNoRegister(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t data) {
	I2C_HandleTypeDef* handle = I2C_GetHandle(I2Cx);

	// Send address and data to peripheral
	if(HAL_I2C_Master_Transmit(handle, (uint16_t) deviceAddress, &data, 1, I2C_TIMEOUT) != HAL_OK) {

		return I2C_Result_ERROR;
	}

	return I2C_Result_OK;
}

I2C_RESULT_t I2C_WriteMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t* data, uint16_t size) {
	I2C_HandleTypeDef* handle = I2C_GetHandle(I2Cx);

	// Send address and data to peripheral
	if(HAL_I2C_Master_Transmit(handle, (uint16_t) deviceAddress, data, size, I2C_TIMEOUT) != HAL_OK) {

		return I2C_Result_ERROR;
	}

	return I2C_Result_OK;
}

I2C_RESULT_t I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t deviceAddress) {
	I2C_HandleTypeDef* handle = I2C_GetHandle(I2Cx);

	if(HAL_I2C_IsDeviceReady(handle, deviceAddress, 2, 5) != HAL_OK) {
		return I2C_Result_ERROR;
	}

	return I2C_Result_OK;
}

// Private functions
#ifdef I2C1
	static void I2C_I2C1_InitGPIO(I2C_PinGroup_t pinGroup) {
		/* Init pins */
	#if defined(GPIOB)
		if (pinGroup == I2C_PinGroup_1) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

			__HAL_RCC_GPIOB_CLK_ENABLE();
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		}
	#endif /* defined(GPIOB) */
	#if defined(GPIOB)
		if (pinGroup == I2C_PinGroup_2) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

			__HAL_RCC_GPIOB_CLK_ENABLE();
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		}
	#endif /* defined(GPIOB) */
	#if defined(GPIOB)
		if (pinGroup == I2C_PinGroup_3) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_9;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

			__HAL_RCC_GPIOB_CLK_ENABLE();
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		}
	#endif /* defined(GPIOB) */
	}
#endif /* I2C1 */

#ifdef I2C2
	static void I2C_I2C2_InitGPIO(I2C_PinGroup_t pinGroup) {
		/* Init pins */
	#if defined(GPIOB)
		if (pinGroup == I2C_PinGroup_1) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;

			__HAL_RCC_GPIOB_CLK_ENABLE();
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		}
	#endif /* defined(GPIOB) */
	#if defined(GPIOF)
		if (pinGroup == I2C_PinGroup_2) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;

			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
	#endif /* defined(GPIOF) */
	#if defined(GPIOH)
		if (pinGroup == I2C_PinGroup_4) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;

			__HAL_RCC_GPIOH_CLK_ENABLE();
			HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
		}
	#endif /* defined(GPIOH) */
	}
#endif /* I2C2 */

#ifdef I2C3
	static void I2C_I2C3_InitGPIO(I2C_PinGroup_t pinGroup) {
		/* Init pins */
	#if defined(GPIOA) && defined(GPIOC)
		if (pinGroup == I2C_PinGroup_1) {
			GPIO_InitTypeDef GPIO_InitStructA = {0};
			GPIO_InitStructA.Pin = GPIO_PIN_8;
			GPIO_InitStructA.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStructA.Pull = I2C_GPIO_PULL;
			GPIO_InitStructA.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStructA.Alternate = GPIO_AF4_I2C3;

			__HAL_RCC_GPIOA_CLK_ENABLE();
			HAL_GPIO_Init(GPIOA, &GPIO_InitStructA);

			GPIO_InitTypeDef GPIO_InitStructB = {0};
			GPIO_InitStructB.Pin = GPIO_PIN_9;
			GPIO_InitStructB.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStructB.Pull = I2C_GPIO_PULL;
			GPIO_InitStructB.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStructB.Alternate = GPIO_AF4_I2C3;

			__HAL_RCC_GPIOC_CLK_ENABLE();
			HAL_GPIO_Init(GPIOC, &GPIO_InitStructB);
		}
	#endif /* defined(GPIOA) && defined(GPIOC) */
	#if defined(GPIOH)
		if (pinGroup == I2C_PinGroup_2) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;

			__HAL_RCC_GPIOH_CLK_ENABLE();
			HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
		}
	#endif /* defined(GPIOB) */
	}
#endif /* I2C3 */

#ifdef I2C4
	static void I2C_I2C4_InitGPIO(I2C_PinGroup_t pinGroup) {
		/* Init pins */
	#if defined(GPIOD)
		if (pinGroup == I2C_PinGroup_1) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;

			__HAL_RCC_GPIOD_CLK_ENABLE();
			HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		}
	#endif /* defined(GPIOD) */
	#if defined(GPIOF)
		if (pinGroup == I2C_PinGroup_2) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;

			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		if (pinGroup == I2C_PinGroup_3) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;

			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
	#endif /* defined(GPIOF) */
	#if defined(GPIOH)
		if (pinGroup == I2C_PinGroup_4) {
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = I2C_GPIO_PULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;

			__HAL_RCC_GPIOH_CLK_ENABLE();
			HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
		}
	#endif /* defined(GPIOH) */
	}
#endif


