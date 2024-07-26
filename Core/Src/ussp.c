/* Includes ------------------------------------------------------------------*/
#include "ussp.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdarg.h"

/* Private variables ---------------------------------------------------------*/
extern osMessageQueueId_t xUartSendQueueHandle;
extern osSemaphoreId_t xUARTHandle;
extern CRC_HandleTypeDef hcrc;
extern osThreadId_t xSendTaskHandle;
extern UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
uint8_t usspCalculateCRC(ussp_payload *data, uint8_t length);
ussp_status_t usspEncodePayload(const ussp_packet *pkt, uint8_t *encodedData, uint16_t *encodedLength);
ussp_status_t usspDecodePayload(const ussp_packet *pkt, uint8_t *encodedData, uint16_t *encodedLength);

ussp_status_t usspSendSMessage(uint32_t timeout, const char *format, ...) {
	va_list args;
	ussp_packet pkt;
	va_start(args, format);
	uint16_t length;

	// Format the string
	length = vsnprintf((char *)&pkt.payload.payloadData, MAX_PAYLOAD_SIZE, format, args);

	va_end(args);

	// Check if the formatted string length exceeds the buffer
	if (length < 0 || length >= MAX_PAYLOAD_SIZE) {
		// *TODO* Implement error
//		printf("Error: Formatted string is too long.\n");
		return usspErrorSizeExceeded;
	}

	 // Send the formatted string to the queue
	if (osMessageQueuePut(xUartSendQueueHandle, &pkt, 0, timeout) != osOK) {
		// Handle error if the queue is full
//		printf("Queue is full! Failed to send message.\n");
		return usspErrorQueueFull;
	}

	return usspOK;
}

void sendTask(void *argument) {
	ussp_packet pkt;
	uint32_t

	while(1) {
		// Wait for a packet to be available in the queue
		if(osMessageQueueGet(xUartSendQueueHandle, &pkt, NULL, osWaitForever) == osOK) {

			if(osSemaphoreAcquire(xUARTHandle, osWaitForever)) {
				// Calculate checksum
				pkt.checksum = usspCalculateCRC(&pkt.payload, pkt.length);

				// Start the DMA transfer
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&pkt, sizeof(ussp_packet));

				// Wait for the notification that the transmission is complete
				osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

			}
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Notify the task that transmission is complete
        osThreadFlagsSet(xSendTaskHandle, 0x01);
    }
}

void HAL_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma) {
    if (hdma->Instance == DMA1_Stream4) {
        // Notify the task that DMA transmission is complete
        osThreadFlagsSet(xSendTaskHandle, 0x01);
    }
}

ussp_status_t usspEncodePayload(const ussp_packet *pkt, uint8_t *encodedData, uint16_t *encodedLength) {
	if (pkt == NULL || encodedData == NULL || encodedLength == NULL) {
		return usspErrorInvalidArg;  // Invalid arguments
	}

    // Ensure buffer is large enough
    if (*encodedLength < (pkt->length + 4)) {  // Minimum size: start byte, length byte, checksum byte, end byte
        return usspErrorBufferToSmall;  // Buffer too small
    }

	uint16_t length = 0;
	uint8_t byte;

	// Start byte
	encodedData[length++] = START_BYTE;
    // Length byte (unchanged)
    encodedData[length++] = pkt->length;

    // Encode payload
    for (uint8_t i = 0; i < pkt->length+1; i++) {
        uint8_t byte = pkt->payload.data[i];
        if (byte == START_BYTE || byte == END_BYTE || byte == ESCAPE_BYTE) {
        	if (length + 2 > *encodedLength)
        		return usspErrorBufferToSmall; // Buffer overflow

            encodedData[length++] = ESCAPE_BYTE;
            encodedData[length++] = byte ^ 0x20; // XOR to create unique substitution
        } else {
        	if (length++ > *encodedLength)
        		return usspErrorBufferToSmall;  // Buffer overflow

            encodedData[length] = byte;
        }
    }

    // Checksum byte (unchanged, added as is)
	if (length++ > *encodedLength)
		return usspErrorBufferToSmall;  // Buffer overflow
    encodedData[length] = pkt->checksum;

    // End byte
	if (length++ > *encodedLength)
		return usspErrorBufferToSmall;  // Buffer overflow
    encodedData[length] = END_BYTE;

    // Set the encoded length
    *encodedLength = length;
}

ussp_status_t usspDecodePayload(const ussp_packet *pkt, uint8_t *encodedData, uint16_t *encodedLength) {
	if (encodedLength < 4) {  // Minimum length to include start, length, checksum, and end byte
		return usspError;
	}

	uint16_t i = 0;
	uint16_t j = 0;
	bool escape = false;

	// Verify start byte
	if (encodedData[i++] != START_BYTE) {
		return false; // Invalid packet
	}

	// Extract length
	pkt->length = encodedData[i++];

	    // Ensure length is within bounds
	    if (pkt->length > (encodedLength - 4)) {
	        return false; // Length is too large
	    }

	    // Decode payload
	    while (i < encodedLength - 2) {  // Exclude checksum and end byte
	        if (escape) {
	            if (encodedData[i] == 0x5E) {
	                pkt->payloadUnion.data[j++] = START_BYTE;
	            } else if (encodedData[i] == 0x5F) {
	                pkt->payloadUnion.data[j++] = END_BYTE;
	            } else if (encodedData[i] == 0x5D) {
	                pkt->payloadUnion.data[j++] = ESCAPE_BYTE;
	            } else {
	                // Invalid escape sequence
	                return false;
	            }
	            escape = false;
	        } else if (encodedData[i] == ESCAPE_BYTE) {
	            escape = true;
	        } else {
	            pkt->payloadUnion.data[j++] = encodedData[i];
	        }
	        i++;
	    }

	    // Check if we have decoded enough payload data
	    if (j != pkt->length) {
	        return false; // Payload length mismatch
	    }

	    // Extract checksum
	    pkt->checksum = encodedData[i++];

	    // Verify end byte
	    if (i >= encodedLength || encodedData[i] != END_BYTE) {
	        return false; // Invalid packet
	    }

	    return true; // Successfully decoded
}

uint8_t usspCalculateCRC(ussp_payload *data, uint8_t length) {
	__HAL_CRC_DR_RESET(&hcrc); // Reset CRC hardware to default state
	return (HAL_CRC_Accumulate(&hcrc, (uint32_t*)data, length+1) & 0xFF);
}
