#ifndef COMMS_H_
#define COMMS_H_

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>

/*GLOBAL VARIABLES----------------------------------------------------------------------------*/
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

//Standard definition for data buffer
#define PACKET_SIZE 256 // bytes
#define BYTE_SIZE 8 // number of bits in a byte

/*FUNCTION PROTOTYPES-----------------------------------------------------------------------------*/
void clearArray(uint8_t *buf);
void print_buffer(uint8_t *buf, int size);
uint8_t receive_packet(uint8_t *buf);
//uint8_t mram_packet(uint8_t *pointer, uint8_t *mram_pointer);
uint8_t check_start_protocol(uint8_t *buf);
void get_address(uint8_t *buf, uint8_t *adr);
void check_flag(uint8_t *buf, uint8_t *flg);
void get_lengthCommand(uint8_t *buf, uint8_t *len_command);
void save_data(uint8_t *buf, uint8_t *data, uint8_t *length_command);
void save_command(uint8_t * buf, uint8_t *data, uint8_t *len_command);
uint8_t parse_packet(uint8_t *buf, uint8_t *adr, uint8_t *flg, uint8_t *len_command, uint8_t *data);
void printStringToConsole(char message[]);

#endif
