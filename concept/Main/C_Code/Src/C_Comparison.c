#include "c_comparison.h"

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
// Description: Receive result of comparison from A. Initiate power reset of A and B depending on result.
// Assumptions: A, B, and C have the exact same data[] array
int compareData() {
	uint8_t tempBuffer[MAX_BUFFER_SIZE];
	int result;
	char result_s[2];
	result_s[1] = '\0';

	// Wait to receive result of comparison from A
	int counter = 0;
	while (1) {
		if (HAL_SPI_Receive(&hspi1, tempBuffer, 1, 0x0FFF) == HAL_OK)
			break;
		
		HAL_Delay(100);
		counter++;
		if (counter > 50)
			return 0;
	}

	// Parse result message
	result_s[0] = tempBuffer[0];
	result = atoi(result_s);	// Convert from bytes to int

	// Execute action based on result received
	return result;
}

// Description: Take received STM_B data from temporary buffer and store in the data array for STM_B on this board.
// Input: temporary buffer, base index (starting index) of the data in memory, number of bytes to be compared
void processData(uint8_t tempBuffer[], int baseIndex, int numBytes) {
	int stmCount = 0;

	// Copy temporary buffer to STM_B.data
	while (stmCount <= numBytes + 1) {
		STM_B.data[baseIndex + stmCount] = tempBuffer[stmCount];
		stmCount++;
	}
}

//Description: This function writes null bytes to the buffer array passed to it
//Input: pointer to buffer that needs to be cleared
void clearArray(uint8_t *buffer) {
	for (int i = 0; i < MAX_BUFFER_SIZE; i++) {
		buffer[i] = '\0';
	}
}
