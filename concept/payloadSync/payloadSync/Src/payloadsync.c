/*
 * payloadsync.c
 *	Description: Functions for sending, receiving, and parsing commands between CDH and Payload.
 *  Created on: Jun 16, 2018
 *      Author: Carter
 */
#include "payloadsync.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Operation Variables*/
uint32_t upload_index = 0; // upload progress tracker
Queue *commandQue;
Queue *errors;
Message *command;
uint8_t shasum[32];

/* File transfer variables */
uint8_t packetLenArr[2];
uint16_t packetLen;
uint8_t *data;

/* Function Definitions ---------------------------------------------------------------- */

void heartbeatListen(){
	return; // To-do: remove this when pins are set
	GPIO_PinState currState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
	HAL_Delay(1000);

	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == currState){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(TX2_BOOT_DELAY); // To-do: adjust boot-time for TX2
	}
	else
		return;
}

/*Queue Functions*/
Queue *initQueue(){
  Queue *que = malloc(sizeof(Queue));
  que->numMessages = 0;
  que->front = NULL;
  que->back = NULL;
  return que;
}

Message *peekQueue(Queue *que){
	return que->front;
}

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

void dequeue(Queue *que){
	Message *temp = que->front;
	que->front = temp->next;
	free(temp);
	que->numMessages--;
}

/* Message Functions*/
Message *createMessage(uint8_t command_code, uint16_t data_len, uint8_t *data){
	Message *newMessage = (Message*)malloc(sizeof(Message)); // Command type and parameter

	newMessage->code = command_code;
	newMessage->payloadLen = data_len;
	newMessage->payload = data;
	newMessage->err = 0;
	newMessage->next = NULL;

	return newMessage;
}

void debugWrite(char *debug_msg){
	while(HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), TX_DELAY) != HAL_OK){
		HAL_Delay(TX_DELAY);
	}
	HAL_Delay(100);
}

void sendData(uint8_t *data, int dataLen){
	  while(HAL_UART_Transmit(&huart6, data, dataLen, TX_DELAY) != HAL_OK){
		  HAL_Delay(TX_DELAY); // Wait 10ms before retry
		  heartbeatListen(); // Check if still alive
	  }
}

void sendmHeader(Message *msg){
	uint8_t mHeader[3];
	mHeader[0] = msg->code; // message code
	mHeader[1] = (msg->payloadLen >> 0) & 0xFF;
	mHeader[2] = (msg->payloadLen >> 8) & 0xFF;
	sendData(mHeader, sizeof(mHeader));
}

uint8_t handleError(Queue *errQue, Message *command, uint8_t *reply){
	if(*reply == 0)
		return FALSE; // No error
	else{
		command->err = *reply;
		enqueue(command, errQue);
		return TRUE; // Notify of error
	}

}

void receiveData(uint8_t *reply, int numBytes){
/* Wait for a response -----------*/
	while(HAL_UART_Receive(&huart6, reply, numBytes, RX_DELAY) != HAL_OK){
		HAL_Delay(RX_DELAY);
		heartbeatListen(); // Check if still alive
	}
}

uint8_t memory[64];
uint32_t memIndex = 0;

uint8_t saveData(uint8_t *data, uint8_t dataLen){
	for(int i = 0; i < dataLen; i++){
		memory[memIndex + i] = data[i];
		memIndex++;

		if(memIndex + i > 255)
			return FALSE;
	}
	return TRUE;
}
