#ifndef __PYMEM_H
#define __PYMEM_H

#include <stdint.h>

void write_pkt(uint8_t *data,uint16_t address, int size);
void ser_print(uint8_t *data, int size);
void request_pkt(uint8_t *data,uint16_t address,int size);
void py_cmd(char cmd, uint8_t *data, int size);
void wcsdc(uint8_t *head, uint8_t *data, uint8_t size);
uint8_t rcsdc(uint8_t *data);
void sertest();
void csdcdemo();


#endif