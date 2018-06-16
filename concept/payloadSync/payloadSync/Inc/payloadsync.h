/*
 * payloadsync.h
 *
 *  Created on: Jun 16, 2018
 *      Author: Carter
 */

#ifndef PAYLOADSYNC_H_
#define PAYLOADSYNC_H_

#define TRUE 1
#define FALSE 0
#define WAIT_TIME 250
#define TX_DELAY 50
#define RX_DELAY 100
#define COMMAND_DELAY 250
#define TX2_BOOT_DELAY 10000
// To-do: Different time-out durations for different commands

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


/* Function Prototypes -----------------------------------------------*/

/* Description: Appends a fixed number of bytes to the global memory array.
 * Inputs: Pointer to the data to be appended, amount of data to be appended
 * Output: 8-bit value either True or False depending on success of the memory transfer
 */
uint8_t saveData(uint8_t *data, uint8_t dataLen); // Dummy Memory


/* Description: TX2 has a heartbeat that inverts its signal every 1second.
 * 				This function checks for two consecutive beats over PCA8 and resets the TX2
 * 				over PCA6 if it's not observed.
 */
void heartbeatListen();


/* Description: Populate a message struct with given command code, data length and data pointer
 * Inputs: 8-bit command code, 16-bit data length, pointer to byte-formatted data array
 * Output: Pointer to newly created message
 * 		   Needs to be freed after transmitted to TX2.
 */
Message *createMessage(uint8_t command_code, uint16_t data_len, uint8_t *data); // Queue operations


/* Description: Send a debug message via serial on UART2
 * Input: Pointer to character array to be transmitted, 16-bit length of message
 * Output: Pointer to newly created message.
 */
void debugWrite(char *debug_msg);


/* Description: Send data over UART6
 * Input: Pointer to the 8-bit formatted data array, 64-bit data length
 * Output: Pointer to newly created message.
 */
void sendData(uint8_t *data, int dataLen);


/* Description: Send data-header over UART6 by extracting
 * 				the header from the message. Header contains (1) command code
 * 				and (2) payload length
 * Input: Pointer to the message to be transmitted.
 */
void sendmHeader(Message *msg);


/* Description: Receive data over UART6 and store it in the passed container.
 * Input: Pointer to the container for the response, and 8-bit length of
 * 		  the data to be received
 */
void receiveData(uint8_t *reply, int numBytes);


/* Description: Reads the reply and documents the error in the error queue if needed.
 * 				If error, reads the command, creates a similar error object appending the reply code,
 * 				then adds it to the error queue
 * Input: Pointer to the error queue, pointer to the reply, and pointer to the command
 * Output: Returns 8-bit value of True or False depending on whether error was observed
 */
uint8_t handleError(Queue *errQue, Message *command, uint8_t *reply);


/* Description: Initialize and return a new queue by nullifying front and end pointers. Zero-ing number of queue items.
 * Output: Returns pointer to newly allocated queue. Needs to be freed when no longer in use.
 */
Queue* initQueue();


/* Description: For a given queue, returns pointer to front message.
 * Output: Pointer to the front message. When freeing messages peeked from the queue, use the dequeue(queue) function.
 * 		   This ensures that the queue maintains front pointer and number of items properly.
 */
Message *peekQueue(Queue *que);


/* Description: Given a queue and a message, appends the message, updating queue pointers and params (num messages).
 */
void enqueue(Message *newMessage, Queue *que);


/* Description: Free the first message from the queue and update queue pointers and params (num messages)
 */
void dequeue(Queue *que);

#endif /* PAYLOADSYNC_H_ */
