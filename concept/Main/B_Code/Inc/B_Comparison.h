#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>

/*GLOBAL VARIABLES---------------------------------------*/
#define MAX_BUFFER_SIZE 64

//Structure declaring board settings, allows each board to keep track of other boards.
struct board {
	uint8_t data[MAX_BUFFER_SIZE];
	char letter;
}STM_A, STM_B, STM_C;

/*FUNCTION PROTYPES---------------------------------------*/
void STM_BOARD_Init(void);
void compareData();
