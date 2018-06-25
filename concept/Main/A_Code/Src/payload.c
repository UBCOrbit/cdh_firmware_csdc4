/*
 * payloadsync.c
 *	Description: Functions for sending, receiving, and parsing commands between CDH and Payload.
 *  Created on: Jun 16, 2018
 *      Author: Carter
 */
#include "payload.h"

UART_HandleTypeDef huart6;

/* Operation Variables*/
uint32_t upload_index = 0; // upload progress tracker
Queue *commandQue;
Queue *errors;
Message *command;

/* File transfer variables */
uint16_t packetLen;
uint8_t *data;

uint8_t memory[64];
uint32_t memIndex = 0;

/* Description: Appends a fixed number of bytes to the global memory array.
 * Inputs: Pointer to the data to be appended, amount of data to be appended
 * Output: 8-bit value either True or False depending on success of the memory transfer
 */
uint8_t saveData(uint8_t *data, uint8_t dataLen){
	for(int i = 0; i < dataLen; i++){
		memory[memIndex + i] = data[i];
		memIndex++;

		if(memIndex + i > 255)
			return FALSE;
	}
	return TRUE;
}

/* Description: TX2 has a heartbeat that inverts its signal every 1second.
 * 				This function checks for two consecutive beats over PC13 and resets the TX2
 * 				over PC9 if it's not observed.
 */
void payloadHeartbeatListen(){
	GPIO_PinState currState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	HAL_Delay(1000);

	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == currState){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_Delay(TX2_BOOT_DELAY); // To-do: adjust boot-time for TX2
	}
	else
		return;
}

/*QUEUE FUNCTIONS --------------------------------------------------------------*/

/* Description: Initialize and return a new queue by nullifying front and end pointers. Zero-ing number of queue items.
 * Output: Returns pointer to newly allocated queue. Needs to be freed when no longer in use.
 */
Queue *initQueue(){
  Queue *que = malloc(sizeof(Queue));
  que->numMessages = 0;
  que->front = NULL;
  que->back = NULL;
  return que;
}

/* Description: For a given queue, returns pointer to front message.
 * Output: Pointer to the front message. When freeing messages peeked from the queue, use the dequeue(queue) function.
 * 		   This ensures that the queue maintains front pointer and number of items properly.
 */
Message *peekQueue(Queue *que){
	return que->front;
}

/* Description: Given a queue and a message, appends the message, updating queue pointers and params (num messages).
 */
void enqueue(Message *newMessage, Queue *que){
	if(que->numMessages == 0){
		que->front = newMessage;
		que->back = newMessage;
	}
	else{
		(que->back)->next = newMessage;
		que->back = newMessage;
	}

	que->numMessages++;
}

/* Description: Free the first message from the queue and update queue pointers and params (num messages)
 */
void dequeue(Queue *que){
	Message *temp = que->front;
	que->front = temp->next;
	free(temp);
	que->numMessages--;
}

/* Description: Returns whether or not the queue is empty
 */
int queueIsEmpty(Queue *que) {
	return (que->numMessages == 0);
}

/* MESSAGE FUNCTIONS -----------------------------------------------------------*/
/* Description: Populate a message struct with given command code, data length and data pointer
 * Inputs: 8-bit command code, 16-bit data length, pointer to byte-formatted data array
 * Output: Pointer to newly created message
 * 		   Needs to be freed after transmitted to TX2.
 */
Message *createMessage(uint8_t command_code, uint16_t data_len, uint8_t *data){
	Message *newMessage = (Message*)malloc(sizeof(Message)); // Command type and parameter

	newMessage->code = command_code;
	newMessage->payloadLen = data_len;
	newMessage->payload = data;
	newMessage->err = 0;
	newMessage->next = NULL;

	return newMessage;
}

/* Description: Send data over UART6
 * Input: Pointer to the 8-bit formatted data array, 64-bit data length
 * Output: Pointer to newly created message.
 */
void sendData(uint8_t *data, int dataLen){
	  while(HAL_UART_Transmit(&huart6, data, dataLen, TX_DELAY) != HAL_OK){
		  HAL_Delay(TX_DELAY); // Wait 10ms before retry
		  payloadHeartbeatListen(); // Check if still alive
	  }
}

/* Description: Receive data over UART6 and store it in the passed container.
 * Input: Pointer to the container for the response, and 8-bit length of
 * 		  the data to be received
 */
void receiveData(uint8_t *reply, int numBytes){
/* Wait for a response -----------*/
	while(HAL_UART_Receive(&huart6, reply, numBytes, RX_DELAY) != HAL_OK){
		HAL_Delay(RX_DELAY);
	}
	printStringToConsole("Response Received\n");

}

/* Description: Send data-header over UART6 by extracting
 * 				the header from the message. Header contains (1) command code
 * 				and (2) payload length
 * Input: Pointer to the message to be transmitted.
 */
void sendmHeader(Message *msg){
	uint8_t mHeader[3];
	mHeader[0] = msg->code; // message code
	mHeader[1] = (msg->payloadLen >> 0) & 0xFF;
	mHeader[2] = (msg->payloadLen >> 8) & 0xFF;
	sendData(mHeader, sizeof(mHeader));
}

/* Description: Reads the reply and documents the error in the error queue if needed.
 * 				If error, reads the command, creates a similar error object appending the reply code,
 * 				then adds it to the error queue
 * Input: Pointer to the error queue, pointer to the reply, and pointer to the command
 * Output: Returns 8-bit value of True or False depending on whether error was observed
 */
uint8_t handleError(Queue *errQue, Message *command, uint8_t *reply){
	if(*reply == 0)
		return FALSE; // No error
	else{
		command->err = *reply;
		enqueue(command, errQue);
		return TRUE; // Notify of error
	}
}
