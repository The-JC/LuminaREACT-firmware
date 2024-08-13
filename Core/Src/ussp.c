/* Includes ------------------------------------------------------------------*/
#include "ussp.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "lwrb/lwrb.h"

/* Private variables ---------------------------------------------------------*/
extern osMessageQueueId_t xUartSendQueueHandle;
extern osMessageQueueId_t xUartRxDMAQueueHandle;

extern osSemaphoreId_t xUARTHandle;
extern CRC_HandleTypeDef hcrc;
extern osThreadId_t xTxTaskHandle;
//extern UART_HandleTypeDef huart2;

//ussp_packet tmp_packet;
ussp_packet send_packet;
uint8_t send_buffer[MAX_PACKAGE_SIZE];


osMemoryPoolId_t packet_MemPool;
lwrb_t rx_buffer;
uint8_t rx_buffer_data[USSP_RX_BUFFER_SIZE];

/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/**
 * \brief           USART RX buffer for DMA to transfer every received byte
 * \note            Contains raw data that are about to be processed by different events
 */
uint8_t usart_rx_dma_buffer[64];

/* Private function prototypes -----------------------------------------------*/
ussp_status_t usspEncodePayload(const ussp_packet *pkt, uint8_t *encodedData, uint16_t *encodedLength);
ussp_status_t usspDecodePayload(const uint8_t *encodedData, uint16_t encodedLength, ussp_packet *pkt);
ussp_status_t usspDecodePayloadLWRB(lwrb_t *buffer, uint16_t encodedLength, ussp_packet *pkt);
uint8_t usspCalculateCRC(ussp_payload *data, uint8_t length);
uint8_t usart_start_tx_dma_transfer(const uint8_t *data, uint16_t len);
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usspProccessRxPackages(void);
void usspPackageHandler(const ussp_packet *pkt);

ussp_status_t usspSendSMessage(uint32_t timeout, const char *format, ...) {
	va_list args;
	ussp_packet *pkt;
	va_start(args, format);
	uint16_t length;

	pkt = osMemoryPoolAlloc(packet_MemPool, 0);
	if(pkt == 0) {
		return usspError;
	}

	pkt->payload.payloadType = ussp_type_message;

	// Format the string
	length = vsnprintf((char *)pkt->payload.payloadData, MAX_PAYLOAD_SIZE, format, args);

	va_end(args);

	// Check if the formatted string length exceeds the buffer
	if (length < 0 || length >= MAX_PAYLOAD_SIZE) {
		// *TODO* Implement error
//		printf("Error: Formatted string is too long.\n");
		return usspErrorSizeExceeded;
	}

	pkt->length = length;

	 // Send the formatted string to the queue
	if (osMessageQueuePut(xUartSendQueueHandle, pkt, 0, timeout) != osOK) {
		// Handle error if the queue is full
//		printf("Queue is full! Failed to send message.\n");

		// Release memory before returning
		if(osMemoryPoolFree(packet_MemPool, pkt) != osOK) {
				return usspError;
		}
		return usspErrorQueueFull;
	}

	// Release memory before returning
	if(osMemoryPoolFree(packet_MemPool, pkt) != osOK) {
		return usspError;
	}

	return usspOK;
}

ussp_status_t usspSendFFT(uint32_t timeout, const uint32_t *fftData) {
	return osOK;
}

void sendTask(void *argument) {
	uint16_t encodedLength;
    void* d;

//	HAL_UART_Receive_DMA(&huart2, receive_buffer, USSP_RX_BUFFER_SIZE);

	packet_MemPool = osMemoryPoolNew(USSP_TX_MEMPOOL, sizeof(ussp_packet), NULL);

	lwrb_init(&rx_buffer, rx_buffer_data, sizeof(rx_buffer_data)); /* Initialize buffer */

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
				usart_start_tx_dma_transfer(send_buffer, encodedLength);

				// Wait for the notification that the transmission is complete
//				osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
//
				// Semaphore is released from Interrupt to make thread non blocking
//				osSemaphoreRelease(xUARTHandle);
			}
		}

		// Handle reception of packets
		if(osMessageQueueGet(xUartRxDMAQueueHandle, &d, NULL, 0) == osOK) {
			 usart_rx_check();

			 usspProccessRxPackages();
		}

		// Handle received packets
//		if(rxBufferSection = osThreadFlagsGet() > 0) {
//			if(rxBufferSection == 1) {
//				// The first half of the RX has been read so we can read it now
//				rxBufferSection = receive_buffer;
//			} else if(rxBufferSection == 2) {
//				rxBufferSection = receive_buffer + USSP_RX_BUFFER_SIZE/2;
//			}
//
//			// Decode
//		}

		osDelay(10);


	}
}

void usspPackageHandler(const ussp_packet *pkt) {

	switch (pkt->payload.payloadType) {
		case ussp_type_message:

			break;
		default:
			break;
	}
}

void usspProccessRxPackages(void) {
//	static uint8_t data_buffer[MAX_PACKAGE_SIZE];
	static ussp_packet packet;
	lwrb_sz_t index = 0;
	uint8_t tmp;

	// Find start byte and jump to beginning of the packet
	tmp = START_BYTE;
	if(lwrb_find(&rx_buffer, &tmp, 1, 0, &index) == 0 ) {
		// No start byte was found in buffer
		return;
	}
	lwrb_skip(&rx_buffer, index);

	tmp = END_BYTE;
	if(lwrb_find(&rx_buffer, &tmp, 1, USSP_PACKAGE_OVERHEAD-1, &index) == 0) {
		// No end byte in buffer yet
		return;
	}

	if(index > MAX_PACKAGE_SIZE) {
		// Package size is to big, skip package
		lwrb_skip(&rx_buffer, index);
		return;
	}

	// A full package is in buffer to be processed
	// Read into local buffer to be decoded
//	if(lwrb_read(&rx_buffer, data_buffer, index) == 0) {
//		// Buffer couldn't be read for some reason
//		return;
//	}
//	if(usspDecodePayload(data_buffer, index, &packet) != usspOK) {
//		// An error occured during package decoding
//		return;
//	}
	if(usspDecodePayloadLWRB(&rx_buffer, index, &packet) != usspOK) {
		// An error occured during package decoding
		return;
	}

	usspPackageHandler(&packet);
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


ussp_status_t usspDecodePayloadLWRB(lwrb_t *buffer, uint16_t encodedLength, ussp_packet *pkt) {
	if (encodedLength < 4) {  // Minimum length to include start, length, checksum, and end byte
		return usspError;
	}

	uint16_t i = 0;
	uint16_t j = 0;
	uint8_t escapeByte = 0;
	uint8_t tmp;

	// Verify start byte
	lwrb_read(buffer, &tmp, 1);
	if (tmp != START_BYTE) {
		return usspErrorInvalidPacket; // Invalid packet
	}
	i++;

	// Extract length
	lwrb_read(buffer, &pkt->length, 1);
	i++;
	if(pkt->length == ESCAPE_BYTE) {
		lwrb_read(buffer, &tmp, 1);
		pkt->length = tmp ^ 0x20;
		i++;
	}

	// Ensure length is within bounds
	if (pkt->length > (encodedLength - 4)) {
		return usspErrorSizeExceeded; // Length is too large
	}

	// Decode payload
	while (i < encodedLength - 2) {  // Exclude checksum and end byte
		lwrb_read(buffer, &tmp, 1);
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
			pkt->payload.data[j++] = tmp ^ 0x20;
			escapeByte = 0;
		} else if (tmp == ESCAPE_BYTE) {
			escapeByte = 1;
		} else {
			pkt->payload.data[j++] = tmp;
		}

		i++;
	}

	// Check if we have decoded enough payload data
	if (j != pkt->length) {
		return usspError; // Payload length mismatch
	}

	// Extract checksum
	lwrb_read(buffer, &pkt->checksum, 1);
	i++;
	if(pkt->checksum == ESCAPE_BYTE) {
		lwrb_read(buffer, &tmp, 1);
		pkt->checksum = tmp ^ 0x20;
		i++;
	}

	// *TODO* Implement checksum check

	// Verify end byte
	lwrb_read(buffer, &tmp, 1);
	if (i >= encodedLength || tmp != END_BYTE) {
		return usspErrorInvalidPacket; // Invalid packet
	}

	return usspOK; // Successfully decoded
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

/**
 * \brief           Check for new data received with DMA
 *
 * User must select context to call this function from:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both context-es, exclusive access protection must be implemented
 * This mode is not advised as it usually means architecture design problems
 *
 * When IDLE interrupt is not present, application must rely only on thread context,
 * by manually calling function as quickly as possible, to make sure
 * data are read from raw buffer and processed.
 *
 * Not doing reads fast enough may cause DMA to overflow unread received bytes,
 * hence application will lost useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
void usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
        old_pos = pos;                          /* Save current position as old for next transfers */
    }
}

/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void usart_process_data(const void* data, size_t len) {

    /*
     * This function is called on DMA TC or HT events, and on UART IDLE (if enabled) event.
     *
     */
    lwrb_write(&rx_buffer, data, len);

}

/**
 * \brief           Checks for data in buffer and starts transfer if not in progress
 */
uint8_t usart_start_tx_dma_transfer(const uint8_t *data, uint16_t len) {
    uint32_t primask;
//    uint8_t started = 0;

    /*
     * First check if transfer is currently in-active,
     * by examining the value of usart_tx_dma_current_len variable.
     *
     * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
     *
     * It is not necessary to disable the interrupts before checking the variable:
     *
     * When usart_tx_dma_current_len == 0
     *    - This function is called by either application or TX DMA interrupt
     *    - When called from interrupt, it was just reset before the call,
     *         indicating transfer just completed and ready for more
     *    - When called from an application, transfer was previously already in-active
     *         and immediate call from interrupt cannot happen at this moment
     *
     * When usart_tx_dma_current_len != 0
     *    - This function is called only by an application.
     *    - It will never be called from interrupt with usart_tx_dma_current_len != 0 condition
     *
     * Disabling interrupts before checking for next transfer is advised
     * only if multiple operating system threads can access to this function w/o
     * exclusive access protection (mutex) configured,
     * or if application calls this function from multiple interrupts.
     *
     * This example assumes worst use case scenario,
     * hence interrupts are disabled prior every check
     */
    primask = __get_PRIMASK();
    __disable_irq();

    /* Configure DMA */
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, len);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)data);

	/* Clear all flags */
	LL_DMA_ClearFlag_TC3(DMA1);
	LL_DMA_ClearFlag_HT3(DMA1);
	LL_DMA_ClearFlag_DME3(DMA1);
	LL_DMA_ClearFlag_FE3(DMA1);
	LL_DMA_ClearFlag_TE3(DMA1);

	/* Start transfer */
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);

    __set_PRIMASK(primask);
//    __enable_irq();
    return usspOK;
}

/**
 * \brief           DMA1 stream5 interrupt handler for USART2 RX
 */
void DMA1_Stream5_IRQHandler(void) {
    void* d = (void *)1;

    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_5) && LL_DMA_IsActiveFlag_HT5(DMA1)) {
        LL_DMA_ClearFlag_HT5(DMA1);             /* Clear half-transfer complete flag */
        osMessageQueuePut(xUartRxDMAQueueHandle, &d, 0, 0); /* Write data to queue. Do not use wait function! */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_5) && LL_DMA_IsActiveFlag_TC5(DMA1)) {
        LL_DMA_ClearFlag_TC5(DMA1);             /* Clear transfer complete flag */
        osMessageQueuePut(xUartRxDMAQueueHandle, &d, 0, 0); /* Write data to queue. Do not use wait function! */
    }

    /* Implement other events when needed */
}

/**
 * \brief           DMA1 stream6 interrupt handler for USART2 TX
 */
void DMA1_Stream6_IRQHandler(void) {
    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_6) && LL_DMA_IsActiveFlag_TC6(DMA1)) {
        LL_DMA_ClearFlag_TC6(DMA1);             /* Clear transfer complete flag */
        osSemaphoreRelease(xUARTHandle);
    }

    /* Implement other events when needed */
}

/**
 * \brief           USART2 global interrupt handler
 */
void USART2_IRQHandler(void) {
    void* d = (void *)1;

    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(USART2) && LL_USART_IsActiveFlag_IDLE(USART2)) {
        LL_USART_ClearFlag_IDLE(USART2);        /* Clear IDLE line flag */
        osMessageQueuePut(xUartRxDMAQueueHandle, &d, 0, 0); /* Write data to queue. Do not use wait function! */
    }

    /* Implement other events when needed */
}



