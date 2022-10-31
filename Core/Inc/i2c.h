/*******************************************************************************
 * Copyright (C) 2021 Julian Hellner - All Rights Reserved
 * 
 * The file i2c.h is part of GainAmpliefier.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Julian Hellner <hellnerjulian@gmail.com>, Sep 24, 2021
 *
 ******************************************************************************/

#ifndef INC_I2C_H_
#define INC_I2C_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup I2C
 * @brief I2C libary for STM32F4
 * @{
 *
 * \par Pinout
 *
\verbatim
       |PINGROUP 1   |PINGROUP 2   |PINGROUP 3   |PINSPACK 4
I2CX   |SCL   SDA    |SCL   SDA    |SCL   SDA    |SCL   SDA
       |             |             |             |
I2C1   |PB6   PB7    |PB8   PB9    |PB6   PB9    |
I2C2   |PB10  PB11   |PF1   PF0    |PH4   PH5    |
I2C3   |PA8   PC9    |PH7   PH8    |-     -      |
I2C4   |PD12  PD13   |PF1   PF0    |PF14  PF15   |PH11  PH12
\endverbatim
 *
 * I2C4 is not available on all devices. Please check if it is available for your device before using it!
 */

#include "stm32f4xx_hal.h"

/**
 * @defgroup CAT5171_Macros
 * @brief Library defines
 * @{
 */

#ifndef I2C_GPIO_PULL
#define I2C_GPIO_PULL GPIO_NOPULL
#endif /* I2C_GPIO_PULL */

/**
 * @\}
 */

/**
 * @defgroup I2C_Typedefs
 * @brief Library Typedefs
 * @{
 */

/**
 * @brief I2C pin group
 */
typedef enum {
	I2C_PinGroup_1 = 0x00,
	I2C_PinGroup_2,
	I2C_PinGroup_3,
	I2C_PinGroup_4,
} I2C_PinGroup_t;

/**
 * @brief I2C return values
 */
typedef enum {
	I2C_Result_OK = 0x00, 	/// Everything OK
	I2C_Result_ERROR		/// An error occurred
} I2C_RESULT_t;

/**
 * @\}
 */

/**
 * @defgroup I2C_Functions
 * @brief Library functions
 * @{
 */

/**
 * @brief Initializes the CAT5171 peripheral
 *
 * @param *I2Cx: I2C peripheral
 * @param pinGroup: Group of pins to use for I2C
 */
I2C_RESULT_t I2C_Init(I2C_TypeDef* I2Cx, I2C_PinGroup_t pinGroup, uint32_t clockSpeed);

/**
 * @brief Read a single byte from a peripheral
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be read from
 * @param registerAddress:	the registers address to be read
 * @param *data:			pointer to the read back data
 * @return @ref I2C_RESULT_t
 */
I2C_RESULT_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data);

/**
 * @brief Read a multiple bytes from a peripheral
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be read from
 * @param registerAddress:	the registers address to be read
 * @param *data:			pointer to the read back data
 * @param size:				Amount of data to be sent
 * @return @ref I2C_RESULT_t
 */
I2C_RESULT_t I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data, uint16_t size);

/**
 * @brief Read a single byte from a peripheral without specifying a register
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be read from
 * @param *data:			pointer to the read back data
 * @return @ref I2C_RESULT_t
 */
I2C_RESULT_t I2C_ReadNoRegister(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t *data);

/**
 * @brief Read a multiple bytes from a peripheral without specifying a register
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be read from
 * @param *data:			pointer to the read back data
 * @param size:				Amount of data to be sent
 * @return @ref I2C_RESULT_t
 */
I2C_RESULT_t I2C_ReadMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t *data, uint16_t size);

/**
 * @brief Read a single byte from a 16-bit register peripheral
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be read from
 * @param registerAddress:	the registers address to be read
 * @param *data:			pointer to the read back data
 * @return @ref I2C_RESULT_t
 */
I2C_RESULT_t I2C_Read16(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint16_t registerAddress, uint8_t *data);

/**
 * @brief Writes a single byte to a peripheral
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be written to
 * @param registerAddress:	the registers address to be written to
 * @param *data:			pointer to the data to be written
 * @return @ref I2C_RESULT_t
 */
I2C_RESULT_t I2C_Write(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t registerAddress, uint8_t data);

/**
 * @brief Writes multiple bytes to a peripheral
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be written to
 * @param registerAddress:	the registers address to be written to
 * @param *data:			pointer to the data to be written
 * @param size:				Amount of data to be sent
 * @return @ref I2C_RESULT_t
 */
I2C_RESULT_t I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint16_t size);

/**
 * @brief Writes a single byte to a peripheral without specifying a register
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be written to
 * @param *data:			pointer to the data to be written
 * @return @ref I2C_RESULT_t
 */
I2C_RESULT_t I2C_WriteNoRegister(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t data);

/**
 * @brief Writes multiple bytes to a peripheral without specifying a register
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be written to
 * @param *data:			pointer to the data to be written
 * @param size:				Amount of data to be sent
 * @return @ref I2C_RESULT_t
 */
I2C_RESULT_t I2C_WriteMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t* data, uint16_t size);

/**
 * @brief Writes a single byte to a peripheral with 16-bit peripheral
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be written to
 * @param registerAddress:	the registers address to be written to
 * @param *data:			pointer to the data to be written
 * @return @ref I2C_RESULT_t
 */
I2C_RESULT_t I2C_Write16(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint16_t registerAddress, uint8_t data);

/**
 * @brief Checks if a peripheral is connected to I2C
 *
 * @param *I2Cx:			I2C peripheral
 * @param deviceAddress:	the 7-bit device address to be checked
 */
I2C_RESULT_t I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t deviceAddress);


/**
 * @}
 */

/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif


#endif /* INC_I2C_H_ */
