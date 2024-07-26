/**
 * @file adc.h
 * @author Julian Hellner (julian@hellner.cc)
 * @brief ADC conversion and DMA control
 * @version 0.1
 * @date 2021-10-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef _ADC_H_
#define _ADC_H_

/**
 * @defgroup ADC_Macros
 * @brief Library defines
 * @{
 */

/**
 * @name ADC_Settings
 * 
 * Conversion cycles is \f$ n_{Conversion} = n_{ADC\_RESOLUTION} + n_{ADC\_SAMPLETIME} \f$ ADCCLK cycles
 * 
 * Conversion frequency is \f$ f_{Conversion} = \frac{f_{ADCCLK}}{n_{Conversion}} \f$
 * 
 * @{
 */

/// ADC Prescaler 
#define ADC_CLOCK_PRESCALER ADC_CLOCK_SYNC_PCLK_DIV6

/// ADC resolution
#define ADC_RESOLUTION ADC_RESOULTION_8B

/// ADC sampling tie in clock cycles
#define ADC_SAMPLING_TIME ADC_SAMPLETIME_144CYCLES

/**
 * @}
 */

/**
 * @\}
 */

/**
 * @defgroup ADC_Functions
 * @brief Library functions
 * @{
 */

/**
 * @brief Initializes the ADC periheral
 * 
 */
void ADC_Init(void);

/**
 * @\}
 */

#endif /* _ADC_H_ */