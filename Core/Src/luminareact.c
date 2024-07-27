/* Includes ------------------------------------------------------------------*/
#include "luminareact.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "ussp.h"

/* Private variables ---------------------------------------------------------*/
extern osSemaphoreId_t xAdcNewDataSemaphoreHandle;
extern osSemaphoreId_t xMaxAmplitudeSemaphoreHandle;
extern osSemaphoreId_t xFftBufferSemaphoreHandle;
extern osEventFlagsId_t fftNewDataHandle;
extern ADC_HandleTypeDef hadc1;

/* Private variables ---------------------------------------------------------*/
int16_t adc_buf[ADC_BUF_LEN];
float32_t fft_buf_in[FFT_SIZE];
float32_t fft_buf_out[FFT_SIZE];

volatile int16_t inputMaxAmplitude;
volatile int16_t inputRmsAmplitude;

/* Private function prototypes -----------------------------------------------*/
void initializeFFT(arm_rfft_fast_instance_f32 *fft_instance);

void controlTask(void *argument) {
	uint8_t i = 0;
	while(1){
		const char *str = "Hello world %d\n";
		usspSendSMessage(0, str,i++);
		osDelay(500);
	}
}

void runADCCallback(void *argument) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
}

void fftTask(void *argument) {
	arm_rfft_fast_instance_f32 fft;
	initializeFFT(&fft);

    while (1) {
        if (osSemaphoreAcquire(xAdcNewDataSemaphoreHandle, osWaitForever) == osOK) {
            // Process ADC buffer data

        	// Convert integer input data to floating-point format
        	arm_q15_to_float(adc_buf, fft_buf_in, FFT_SIZE);

        	// Determine Max value for
        	if(osSemaphoreAcquire(xMaxAmplitudeSemaphoreHandle, 0) == osOK) {
        		int16_t value;
        		uint32_t index;
        		arm_max_q15(adc_buf, ADC_BUF_LEN, &value, &index);
        		inputMaxAmplitude = value;

        		arm_rms_q15(adc_buf, ADC_BUF_LEN, &value);
        		inputRmsAmplitude = value;

        		osSemaphoreRelease(xMaxAmplitudeSemaphoreHandle);
        	}

        	// release adcNewData semaphore
        	osSemaphoreRelease(xAdcNewDataSemaphoreHandle);

        	// Normalize the floating-point data to [0, 1]
        	arm_scale_f32(fft_buf_in, 1.0f / ADC_MAX_VALUE, fft_buf_in, FFT_SIZE);

        	if(osSemaphoreAcquire(xFftBufferSemaphoreHandle, 0) == osOK) {

        		// Perform Real forward FFT
        		// fft_buf_in is modified during fft
        		arm_rfft_fast_f32(&fft, fft_buf_in, fft_buf_out, 0);

        		osSemaphoreRelease(xFftBufferSemaphoreHandle);
        	}

        	osDelay(1);
        }
    }
}

void ledTask(void *argument) {
  for(;;)
  {
    osDelay(10);
  }
}

void initializeFFT(arm_rfft_fast_instance_f32 *fft_instance) {
	uint8_t retryCount = 0;
	arm_status status;

	while(retryCount < INIT_FFT_RETRY_COUNT) {
		status = arm_rfft_fast_init_f32(fft_instance, FFT_SIZE);
		if(status == ARM_MATH_SUCCESS){
			// Initialization succeeded
			return;
		} else {
			// Log error message
//			printf("FFT Initialization failed, attempt %d\n", retryCount + 1);
			retryCount++;
			vTaskDelay(pdMS_TO_TICKS(500));  // Wait before retrying
		}
	}

	// Check if initialization eventually succeeded
	if (status != ARM_MATH_SUCCESS) {
		// Handle persistent initialization failure
//		printf("FFT Initialization failed after %d attempts. Entering fail-safe mode.\n", INIT_FFT_RETRY_COUNT);

		// Take appropriate action, e.g., notify the system or enter a fail-safe mode
		osThreadExit();  // Terminate the task if FFT is essential
		return;
	}
}
