/*
 * luminareact.h
 *
 *  Created on: Jul 24, 2024
 *      Author: julia
 */

#ifndef INC_LUMINAREACT_H_
#define INC_LUMINAREACT_H_

// settings

//#define TRANSMIT_UART
#define TRANSMIT_FFT

#define FLOAT_FFT

/* Private defines -----------------------------------------------------------*/

#define FFT_SIZE 4096
#define ADC_BUF_LEN FFT_SIZE
#define ADC_RESOLUTION 12

#define ADC_MAX_VALUE (2^ADC_RESOLUTION)

//#define START_BYTE 0xFEFEFEFE

#define INIT_FFT_RETRY_COUNT 3

// function prototypes
void controlTask(void *argument);
void fftTask(void *argument);
void ledTask(void *argument);
void runADCCallback(void *argument);



#endif /* INC_LUMINAREACT_H_ */
