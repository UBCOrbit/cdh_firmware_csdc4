/*
 * payloadsync.h
 *	Description: Functions for sending, receiving, and parsing commands between CDH and Payload.
 *  Created on: Jun 16, 2018
 *      Author: Carter
 */

#ifndef PAYLOAD_H_
#define PAYLOAD_H_

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>

#define TRUE 1
#define FALSE 0
#define WAIT_TIME 250
#define TX_DELAY 50
#define RX_DELAY 100
#define COMMAND_DELAY 250
#define TX2_BOOT_DELAY 30000

// Command Types
#define POWEROFF        0
#define START_DOWNLOAD  1
#define START_UPLOAD    2
#define REQUEST_PACKET  3
#define SEND_PACKET     4
#define CANCEL_UPLOAD   5
#define CANCEL_DOWNLOAD 6
#define FINALIZE_UPLOAD 7
#define TAKE_PHOTO      8
#define EXECUTE_COMMAND 9

/* Structures -----------------------------------------------*/

// Description: Payload message format. Consists of the command code, data length, and actual data.
typedef struct Message {
	uint8_t code;
	uint16_t payloadLen;
	uint8_t *payload;
	uint8_t err;
	struct Message *next;
} Message;

// Description: Container for messages received from ground-station. Messages are handled in a FIFO
//				manner when forwarded to the TX2
typedef struct Queue {
	Message *front;
	Message *back;
	uint8_t numMessages;
} Queue;

extern uint32_t upload_index; // upload progress tracker
extern Queue *commandQue;
extern Queue *errors;
extern Message *command;
extern uint8_t shasum[];
extern uint8_t packetLenArr[];
extern uint16_t packetLen;
extern uint8_t *data;

/* Function Prototypes -----------------------------------------------*/
uint8_t saveData(uint8_t *data, uint8_t dataLen); // Dummy Memory
void payloadHeartbeatListen();

//Queue Functions
Queue* initQueue();
Message *peekQueue(Queue *que);
void enqueue(Message *newMessage, Queue *que);
void dequeue(Queue *que);
int queueIsEmpty(Queue *que);

//Message Functions
Message *createMessage(uint8_t command_code, uint16_t data_len, uint8_t *data);
void sendData(uint8_t *data, int dataLen);
void receiveData(uint8_t *reply, int numBytes);
void sendmHeader(Message *msg);
uint8_t handleError(Queue *errQue, Message *command, uint8_t *reply);

#endif
