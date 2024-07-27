/* Includes ------------------------------------------------------------------*/
#include "ussp.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"

/* Private variables ---------------------------------------------------------*/
extern osMessageQueueId_t xUartSendQueueHandle;
extern osSemaphoreId_t xUARTHandle;
extern CRC_HandleTypeDef hcrc;
extern osThreadId_t xTxTaskHandle;
extern UART_HandleTypeDef huart2;

ussp_packet send_packet;
uint8_t send_buffer[MAX_PACKAGE_SIZE];

/* Private function prototypes -----------------------------------------------*/
uint8_t usspCalculateCRC(ussp_payload *data, uint8_t length);
ussp_status_t usspEncodePayload(const ussp_packet *pkt, uint8_t *encodedData, uint16_t *encodedLength);
ussp_status_t usspDecodePayload(const uint8_t *encodedData, uint16_t encodedLength, ussp_packet *pkt);

ussp_status_t usspSendSMessage(uint32_t timeout, const char *format, ...) {
	va_list args;
	ussp_packet pkt = {0};
	va_start(args, format);
	uint16_t length;

	pkt.payload.payloadType = ussp_type_message;

	// Format the string
	length = vsnprintf((char *)&pkt.payload.payloadData, MAX_PAYLOAD_SIZE, format, args);

	va_end(args);

	// Check if the formatted string length exceeds the buffer
	if (length < 0 || length >= MAX_PAYLOAD_SIZE) {
		// *TODO* Implement error
//		printf("Error: Formatted string is too long.\n");
		return usspErrorSizeExceeded;
	}

	pkt.length = length;

	 // Send the formatted string to the queue
	if (osMessageQueuePut(xUartSendQueueHandle, &pkt, 0, timeout) != osOK) {
		// Handle error if the queue is full
//		printf("Queue is full! Failed to send message.\n");
		return usspErrorQueueFull;
	}

	return usspOK;
}

void sendTask(void *argument) {
	uint16_t encodedLength;

	while(1) {

		// Send a packet if one is available
		if(osMessageQueueGet(xUartSendQueueHandle, &send_packet, NULL, 0) == osOK) {

			if(osSemaphoreAcquire(xUARTHandle, osWaitForever) == osOK) {
				// Calculate checksum
				send_packet.checksum = usspCalculateCRC(&send_packet.payload, send_packet.length);

				encodedLength = MAX_PACKAGE_SIZE;
				if(usspEncodePayload(&send_packet, send_buffer, &encodedLength) != usspOK)
					// *TODO* implement error handling
					break;

				// Start the DMA transfer
//				HAL_UART_Transmit(&huart2, buffer, encodedLength, osWaitForever);
				HAL_UART_Transmit_DMA(&huart2, send_buffer, encodedLength);

				// Wait for the notification that the transmission is complete
				osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

				osSemaphoreRelease(xUARTHandle);
			}
		}

		osDelay(10);


	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//	osSemaphoreRelease(xUARTHandle);
    if (huart->Instance == USART2) {
        // Notify the task that transmission is complete
        osThreadFlagsSet(xTxTaskHandle, 0x01);
    }
}

void HAL_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma) {
//	osSemaphoreRelease(xUARTHandle);
    if (hdma->Instance == DMA1_Stream6) {
        // Notify the task that DMA transmission is complete
        osThreadFlagsSet(xTxTaskHandle, 0x01);
    }
}



ussp_status_t usspEncodePayload(const ussp_packet *pkt, uint8_t *encodedData, uint16_t *encodedLength) {
	if (pkt == NULL || encodedData == NULL || encodedLength == NULL) {
		return usspErrorInvalidArg;  // Invalid arguments
	}

    // Ensure buffer is large enough
    if (*encodedLength < ((uint16_t)pkt->length + USSP_PACKAGE_OVERHEAD)) {  // Minimum size: start byte, length byte, checksum byte, end byte
        return usspErrorBufferToSmall;  // Buffer too small
    }

	uint16_t length = 0;
	uint8_t byte;
	memset(encodedData, 0, *encodedLength);
	// Start byte
	encodedData[length++] = START_BYTE;
    // Length byte (unchanged)
	byte = pkt->length;
	if (byte == START_BYTE || byte == END_BYTE || byte == ESCAPE_BYTE) {
        encodedData[length++] = ESCAPE_BYTE;
        encodedData[length++] = byte ^ 0x20; // XOR to create unique substitution
	} else {
		encodedData[length++] = pkt->length;
	}

    // Encode payload
    for (uint8_t i = 0; i < pkt->length+1; i++) {
        byte = pkt->payload.data[i];
        if (byte == START_BYTE || byte == END_BYTE || byte == ESCAPE_BYTE) {
        	if (length + 2 > *encodedLength)
        		return usspErrorBufferToSmall; // Buffer overflow

            encodedData[length++] = ESCAPE_BYTE;
            encodedData[length++] = byte ^ 0x20; // XOR to create unique substitution
        } else {
        	if (length+1 > *encodedLength)
        		return usspErrorBufferToSmall;  // Buffer overflow

            encodedData[length++] = byte;
        }
    }

    // Checksum byte (unchanged, added as is)
    byte = pkt->checksum;
    if (byte == START_BYTE || byte == END_BYTE || byte == ESCAPE_BYTE) {
    	if (length+2 > *encodedLength)
    		return usspErrorBufferToSmall;  // Buffer overflow

        encodedData[length++] = ESCAPE_BYTE;
        encodedData[length++] = byte ^ 0x20; // XOR to create unique substitution
    } else {
		if (length+1 > *encodedLength)
			return usspErrorBufferToSmall;  // Buffer overflow
		encodedData[length++] = byte;
    }

    // End byte
	if (length+1 > *encodedLength)
		return usspErrorBufferToSmall;  // Buffer overflow
    encodedData[length++] = END_BYTE;

    // Set the encoded length
    *encodedLength = length;

    return usspOK;
}

ussp_status_t usspDecodePayload(const uint8_t *encodedData, uint16_t encodedLength, ussp_packet *pkt) {
	if (encodedLength < 4) {  // Minimum length to include start, length, checksum, and end byte
		return usspError;
	}

	uint16_t i = 0;
	uint16_t j = 0;
	uint8_t escapeByte = 0;

	// Verify start byte
	if (encodedData[i++] != START_BYTE) {
		return usspErrorInvalidPacket; // Invalid packet
	}

	// Extract length
	pkt->length = encodedData[i++];
	if(pkt->length == ESCAPE_BYTE) {
		pkt->length = encodedData[i++] ^ 0x20;
	}

	// Ensure length is within bounds
	if (pkt->length > (encodedLength - 4)) {
		return usspErrorSizeExceeded; // Length is too large
	}

	// Decode payload
	while (i < encodedLength - 2) {  // Exclude checksum and end byte
		if (escapeByte) {
//			if (encodedData[i] == 0x5E) {
//				pkt->payload.data[j++] = START_BYTE;
//			} else if (encodedData[i] == 0x5F) {
//				pkt->payload.data[j++] = END_BYTE;
//			} else if (encodedData[i] == 0x5D) {
//				pkt->payload.data[j++] = ESCAPE_BYTE;
//			} else {
//				// Invalid escape sequence
//				return usspErrorInvalidPacket;
//			}
			pkt->payload.data[j++] = encodedData[i] ^ 0x20;
			escapeByte = 0;
		} else if (encodedData[i] == ESCAPE_BYTE) {
			escapeByte = 1;
		} else {
			pkt->payload.data[j++] = encodedData[i];
		}

		i++;
	}

	// Check if we have decoded enough payload data
	if (j != pkt->length) {
		return usspError; // Payload length mismatch
	}

	// Extract checksum
	pkt->checksum = encodedData[i++];
	if(pkt->checksum == ESCAPE_BYTE) {
		pkt->checksum = encodedData[i++] ^ 0x20;
	}

	// Verify end byte
	if (i >= encodedLength || encodedData[i] != END_BYTE) {
		return usspErrorInvalidPacket; // Invalid packet
	}

	return usspOK; // Successfully decoded
}

uint8_t usspCalculateCRC(ussp_payload *data, uint8_t length) {
	__HAL_CRC_DR_RESET(&hcrc); // Reset CRC hardware to default state
	uint32_t crc32 = HAL_CRC_Accumulate(&hcrc, (uint32_t*)data, (length+1 + 4-1)/4);
	return (crc32 & 0xFF);
}
