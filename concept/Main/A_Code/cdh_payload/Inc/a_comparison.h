#ifndef A_COMPARISON_H_
#define A_COMPARISON_H_

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>

/*GLOBAL VARIABLES---------------------------------------*/
#define MAX_BUFFER_SIZE 64

//Structure declaring board settings, allows each board to keep track of other boards.
struct board {
	uint8_t data[MAX_BUFFER_SIZE];
	char letter;
}STM_A, STM_B, STM_C;

/*FUNCTION PROTYPES---------------------------------------*/
void STM_BOARD_Init(void);
void processData(uint8_t tempBuffer[], int baseIndex, int numBytes);
void compareData(int baseIndex, int numBytes);

#endif
