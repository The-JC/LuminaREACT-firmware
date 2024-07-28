/*
 * ussp.h
 *
 *  Created on: Jul 25, 2024
 *      Author: julia
 */

#ifndef INC_USSP_H_
#define INC_USSP_H_

#include "main.h"
#include "cmsis_os.h"

#define START_BYTE 		0x7E
#define END_BYTE   		0x7F
#define ESCAPE_BYTE		0x7D

#define MAX_PAYLOAD_SIZE 255
// START_BYTE LENGTH PAYLOAD_TYPE CHECKSUM END_BYTE
#define USSP_PACKAGE_OVERHEAD 5
#define MAX_PACKAGE_SIZE USSP_PACKAGE_OVERHEAD + MAX_PAYLOAD_SIZE

#define USSP_RX_BUFFER_SIZE MAX_PACKAGE_SIZE*4
#define USSP_TX_MEMPOOL 2

// Define a union to simplify access to payload data
typedef union {
	struct {
		uint8_t payloadType;
		uint8_t payloadData[MAX_PAYLOAD_SIZE];
	};
	uint8_t data[1 + MAX_PAYLOAD_SIZE];  // One byte for payload type + payload data
} ussp_payload;

typedef struct {
    uint8_t length;
    ussp_payload payload;
    uint8_t checksum;
} ussp_packet;

/// Status code values returned by USSP functions.
typedef enum {
	usspOK					= 0,	///< Operation completed successfully.
	usspError				= 1,	///< Unspecified USSP error: run-time error but no other error message fits.
	usspErrorSizeExceeded	= 2,	///<
	usspErrorQueueFull		= 3, 	///<
	usspErrorInvalidArg		= 4,
	usspErrorBufferToSmall	= 5,
	usspErrorInvalidPacket	= 6,
	usspErroInvalidCRC		= 7,

} ussp_status_t;

typedef enum {
	ussp_type_invalid		= 0,
	ussp_type_message		= 0x80,
} ussp_payload_type_t;

void sendTask(void *argument);

ussp_status_t usspSendSMessage(uint32_t timeout, const char *format, ...);

//void usspProcessReceivedPacket(uint8_t *data, uint16_t len);
//osStatus_t usspSendPacket(ussp_packet *pkt);

#endif /* INC_USSP_H_ */
