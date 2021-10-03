/*******************************************************************************
 * Copyright (C) 2021 Julian Hellner - All Rights Reserved
 * 
 * The file cat5171.h is part of GainAmpliefier.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Julian Hellner <julian@hellner.cc>, Sep 24, 2021
 *
 ******************************************************************************/

#ifndef INC_CAT5171_H_
#define INC_CAT5171_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

/**
 * @defgroup CAT5171_Macros
 * @brief Library defines
 * @{
 */

#define CAT5171				I2C1
#define CAT5171_PINGROUP 	I2C_PinGroup_1
#define CAT5171_CLOCK		250000

#define CAT5171_ADDRESS 	0x2C << 1


/**
 * @}
 */

/**
 * @defgroup CAT5171_Typedefs
 * @brief Library Typedefs
 * @{
 */

/**
 * @brief CAT return values
 */
typedef enum {
	CAT5171_RESULT_OK = 0x0,				/// Everything OK
	CAT5171_RESULT_PeripheralNotConnected,	/// Peripheral is not connected
	CAT5171_RESULT_ERROR					/// An error occured
} CAT5171_RESULT_t;

/**
 * @}
 */

/**
 * @defgroup CAT5171_Functions
 * @brief Library functions
 * @{
 */

/**
 *	@brief Initializes the CAT5171 peripheral
 *
 *	@return @ref CAT5171_RESULT_t
 */
CAT5171_RESULT_t CAT5171_Init(void);

/**
 *	@brief Sets the wiper of the CAT5171 to a set value
 *
 *	@param tap: tap to set wiper to
 *	@return @ref CAT5171_RESULT_t
 */
CAT5171_RESULT_t CAT5171_SetWiper(uint8_t tap);

/**
 *	@brief Reads back the wiper position of the CAT5171
 *
 *	@param *tap: pointer to the read back data
 *	@return @ref CAT5171_RESULT_t
 */
CAT5171_RESULT_t CAT5171_GetWiper(uint8_t* tap);


/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /* INC_CAT5171_H_ */
