#include "B_Comparison.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* Private Functions-------------------------------------------------------*/
void clearArray(uint8_t *buffer);

void STM_BOARD_Init(void) {
	//Declare letter to identify boards for output messages.
	STM_A.letter = 'A';
	STM_B.letter = 'B';
	STM_C.letter = 'C';
	//Clear data buffers.
	clearArray(STM_A.data);
	clearArray(STM_B.data);
	clearArray(STM_C.data);
}

// Description: Query STM_B for a portion of data. Compare received STM_B data with STM_A's own data.
//				Send result of comparison over to C for reset if needed.
// Input: base index (starting index) of the data in memory, number of bytes to be compared
// Side-Effects: Request data from B. Receive and store B's data. Send result to C. Get power cycled.
// Assumptions: A, B, and C have the exact same data[] array
//Description: Wait for STM_A to send query string, process the query string and send the data back.
void compareData() {
	int reqBytes = 2;
	int baseIndex, numBytes;

	char baseIndex_s[2];
	char numBytes_s[2];

	uint8_t tempBuffer[MAX_BUFFER_SIZE];
	clearArray(tempBuffer);

	// Wait until STM_A sends the request string
	int received = 0;
	while (received == 0) {
		if (HAL_SPI_Receive(&hspi1, tempBuffer, reqBytes, 0x0FFF) == HAL_OK)
			received = 1;
	}

	// Parse data request string
	baseIndex_s[0] = tempBuffer[0];
	baseIndex_s[1] = '\0';

	// Store numBytes
	numBytes_s[0] = tempBuffer[1];
	numBytes_s[1] = '\0';

	// Convert from bytes to int
	baseIndex = atoi(baseIndex_s);
	numBytes = atoi(numBytes_s);

	// Generate data array
	uint8_t reqData[numBytes];
	clearArray(reqData);

	// Transfer data from internal storage to msg buffer
	for (int count = 0; count < numBytes; count++) {
		reqData[count] = STM_B.data[baseIndex + count];
	}

	// Send data to STM_A
	HAL_Delay(500);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)reqData, strlen(reqData), 0x0FFF);
}

//Description: This function writes null bytes to the buffer array passed to it
//Input: pointer to buffer that needs to be cleared
void clearArray(uint8_t *buffer) {
	for (int i = 0; i < MAX_BUFFER_SIZE; i++) {
		buffer[i] = '\0';
	}
}
