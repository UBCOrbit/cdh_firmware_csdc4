#include "a_comparison.h"

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

/* Description: Query STM_B for a portion of data. Compare received STM_B data with STM_A's own data.
 *				Send result of comparison over to C for reset if needed.
 * Input: base index (starting index) of the data in memory, number of bytes to be compared
 * Side-Effects: Request data from B. Receive and store B's data. Send result to C. Get power cycled.
 * Assumptions: A, B, and C have the exact same data[] array
 */
void compareData(int baseIndex, int numBytes) {
	// Generate query request for STMB data
	char addressString[8] = "";
	itoa(baseIndex, addressString, 10);

	char sizeString[24] = "";
	itoa(numBytes, sizeString, 10);

	char fullString[32] = "";
	strcpy(fullString, addressString);
	strcat(fullString, sizeString);

	// Send query to B
	HAL_SPI_Transmit(&hspi2, (uint8_t*)fullString, strlen(fullString), 0x0FFF);

	// Wait for data from B and store data in temporary buffer when it comes in
	int counter = 0;
	uint8_t tempBuffer[MAX_BUFFER_SIZE];
	clearArray(tempBuffer);

	while (1) {
		if (HAL_SPI_Receive(&hspi1, tempBuffer, numBytes, 0x0FFF) == HAL_OK)
			break;

		HAL_Delay(100);
		counter++;
		if (counter > 50)
			break;
	}

	processData(tempBuffer, baseIndex, numBytes);

	// Compare received B data with A data 
	int i = 0;
	int comp_result = 1;
	for (i = 0; i<numBytes; i++) {
		if (STM_A.data[baseIndex + i] != STM_B.data[baseIndex + i])
			comp_result = 0;
	}

	// General result message for C 
	char result[2];
	itoa(comp_result, result, 10);
	result[1] = '\0';

	// Send result string to C 
	HAL_SPI_Transmit(&hspi2, (uint8_t*)result, strlen(result), 0x0FFF);
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

//Description: This function resends all current variable values to STM's that were reset
void reinitialize(){
	int reqBytes = 2;
	int baseIndex, numBytes;

	char baseIndex_s[2];
	char numBytes_s[2];

	uint8_t tempBuffer[MAX_BUFFER_SIZE];
	clearArray(tempBuffer);

	// Parse data request string --------------------------------------
	baseIndex_s[0] = tempBuffer[0];
	baseIndex_s[1] = '\0';

	// Store numBytes -------------------------------------------------
	numBytes_s[0] = tempBuffer[1];
	numBytes_s[1] = '\0';

	// Convert from bytes to int --------------------------------------
	baseIndex = atoi(baseIndex_s);
	numBytes = atoi(numBytes_s);

	// Generate data array --------------------------------------------
	uint8_t reqData[numBytes];
	clearArray(reqData);

	// Transfer data from internal storage to msg buffer --------------
	for(int count = 0; count < numBytes; count++){
	  reqData[count] = STM_A.data[baseIndex+count];
	}

	// Send data to STM_A and STM_B -------------------------------------------------
	HAL_Delay(500);

	HAL_SPI_Transmit(&hspi2, (uint8_t*)reqData, strlen(reqData), 0x0FFF);
}

//Description: This function writes null bytes to the buffer array passed to it
//Input: pointer to buffer that needs to be cleared
void clearArray(uint8_t *buffer) {
	for (int i = 0; i < MAX_BUFFER_SIZE; i++) {
		buffer[i] = '\0';
	}
}
