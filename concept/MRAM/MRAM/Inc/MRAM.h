#include "stm32f4xx_hal.h"

SPI_HandleTypeDef hspi3;

//Memory Protect Commands
#define PROTECT_ALL 0x8E
#define PROTECT_UPPER_HALF 0x8A
#define PROTECT_UPPER_QUARTER 0x86
#define PROTECT_NONE 0x82

//Function prototypes
void init_mem();
void write_enable(int enable);
void write_status(uint8_t data);
void read_status(uint8_t *status);
void read_mem(uint16_t address, int size, uint8_t *buffer);
void write_mem(uint16_t address, int size, uint8_t *buffer);
void sleep(int sleep);
