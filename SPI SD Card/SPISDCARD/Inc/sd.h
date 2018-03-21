/*
 * sd.h
 *
 *  Created on: Feb 3, 2018
 *      Author: Andrada Zoltan
 */

#ifndef SD_H_
#define SD_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
// Based on the document:
//
// SD Specifications
// Part 1
// Physical Layer
// Simplified Specification
// Version 2.00
// September 25, 2006
//
// www.sdcard.org/developers/tech/sdcard/pls/Simplified_Physical_Layer_Spec.pdf
//------------------------------------------------------------------------------
// SD card commands
/** GO_IDLE_STATE - init card in spi mode if CS low */
uint8_t const CMD0 = 0X00;
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
uint8_t const CMD8 = 0X08;
/** SEND_CSD - read the Card Specific Data (CSD register) */
uint8_t const CMD9 = 0X09;
/** SEND_CID - read the card identification information (CID register) */
uint8_t const CMD10 = 0X0A;
/** SEND_STATUS - read the card status register */
uint8_t const CMD13 = 0X0D;
/** READ_BLOCK - read a single data block from the card */
uint8_t const CMD17 = 0X11;
/** WRITE_BLOCK - write a single data block to the card */
uint8_t const CMD24 = 0X18;
/** WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION */
uint8_t const CMD25 = 0X19;
/** ERASE_WR_BLK_START - sets the address of the first block to be erased */
uint8_t const CMD32 = 0X20;
/** ERASE_WR_BLK_END - sets the address of the last block of the continuous
 range to be erased*/
uint8_t const CMD33 = 0X21;
/** ERASE - erase all previously selected blocks */
uint8_t const CMD38 = 0X26;
/** APP_CMD - escape for application specific command */
uint8_t const CMD55 = 0X37;
/** READ_OCR - read the OCR register of a card */
uint8_t const CMD58 = 0X3A;
/** SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be
 pre-erased before writing */
uint8_t const ACMD23 = 0X17;
/** SD_SEND_OP_COMD - Sends host capacity support information and
 activates the card's initialization process */
uint8_t const ACMD41 = 0X29;
//------------------------------------------------------------------------------
/** status for card in the ready state */
uint8_t const R1_READY_STATE = 0X00;
/** status for card in the idle state */
uint8_t const R1_IDLE_STATE = 0X01;
/** status bit for illegal command */
uint8_t const R1_ILLEGAL_COMMAND = 0X04;
/** start data token for read or write single block*/
uint8_t const DATA_START_BLOCK = 0XFE;
/** stop token for write multiple blocks*/
uint8_t const STOP_TRAN_TOKEN = 0XFD;
/** start data token for write multiple blocks*/
uint8_t const WRITE_MULTIPLE_TOKEN = 0XFC;
/** mask for data response tokens after a write block operation */
uint8_t const DATA_RES_MASK = 0X1F;
/** write data accepted token */
uint8_t const DATA_RES_ACCEPTED = 0X05;
//------------------------------------------------------------------------------
typedef struct CID {
	// byte 0
	uint8_t mid;  // Manufacturer ID
	// byte 1-2
	char oid[2];  // OEM/Application ID
	// byte 3-7
	char pnm[5];  // Product name
	// byte 8
	unsigned prv_m :4;  // Product revision n.m
	unsigned prv_n :4;
	// byte 9-12
	uint32_t psn;  // Product serial number
	// byte 13
	unsigned mdt_year_high :4;  // Manufacturing date
	unsigned reserved :4;
	// byte 14
	unsigned mdt_month :4;
	unsigned mdt_year_low :4;
	// byte 15
	unsigned always1 :1;
	unsigned crc :7;
} cid_t;
//------------------------------------------------------------------------------
// CSD for version 1.00 cards
typedef struct CSDV1 {
	// byte 0
	unsigned reserved1 :6;
	unsigned csd_ver :2;
	// byte 1
	uint8_t taac;
	// byte 2
	uint8_t nsac;
	// byte 3
	uint8_t tran_speed;
	// byte 4
	uint8_t ccc_high;
	// byte 5
	unsigned read_bl_len :4;
	unsigned ccc_low :4;
	// byte 6
	unsigned c_size_high :2;
	unsigned reserved2 :2;
	unsigned dsr_imp :1;
	unsigned read_blk_misalign :1;
	unsigned write_blk_misalign :1;
	unsigned read_bl_partial :1;
	// byte 7
	uint8_t c_size_mid;
	// byte 8
	unsigned vdd_r_curr_max :3;
	unsigned vdd_r_curr_min :3;
	unsigned c_size_low :2;
	// byte 9
	unsigned c_size_mult_high :2;
	unsigned vdd_w_cur_max :3;
	unsigned vdd_w_curr_min :3;
	// byte 10
	unsigned sector_size_high :6;
	unsigned erase_blk_en :1;
	unsigned c_size_mult_low :1;
	// byte 11
	unsigned wp_grp_size :7;
	unsigned sector_size_low :1;
	// byte 12
	unsigned write_bl_len_high :2;
	unsigned r2w_factor :3;
	unsigned reserved3 :2;
	unsigned wp_grp_enable :1;
	// byte 13
	unsigned reserved4 :5;
	unsigned write_partial :1;
	unsigned write_bl_len_low :2;
	// byte 14
	unsigned reserved5 :2;
	unsigned file_format :2;
	unsigned tmp_write_protect :1;
	unsigned perm_write_protect :1;
	unsigned copy :1;
	unsigned file_format_grp :1;
	// byte 15
	unsigned always1 :1;
	unsigned crc :7;
} csd1_t;
//------------------------------------------------------------------------------
//Public functions
extern void SDCard_Init(SPI_HandleTypeDef* hspi, uint16_t cs1Pin, GPIO_TypeDef* cs1Port);
extern int initalize(); //startup the cards
extern uint32_t getSize(); //get the total storage space size
extern int readBlock(uint32_t blockaddr, uint8_t* buffer); //reads a single 512 byte block
extern int writeBlock(uint32_t blockaddr, uint8_t* buffer); //writes a single 512 byte block

//Private functions
void waitUntilReady(); //waits until the card is ready
uint8_t cardCommand(uint8_t command, uint32_t arg);
uint8_t cardAcmd(uint8_t cmd, uint32_t arg) {
	cardCommand(CMD55, 0);
	return cardCommand(cmd, arg);
}
HAL_StatusTypeDef SPI_Recieve(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef lSPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi,
			uint32_t Flag, uint32_t State, uint32_t Timeout,
			uint32_t Tickstart);
uint8_t readRegister(uint8_t cmd, void* buf);

/**
* Read a cards CSD register. The CSD contains Card-Specific Data that
* provides information regarding access to the card's contents. */
uint8_t readCSD(csd1_t* csd) {
	return readRegister(CMD9, csd);
}
void selectCard();
void deselectCard();
SPI_HandleTypeDef* _spi;
GPIO_TypeDef *_cs1Port;
uint16_t _cs1Pin;
uint64_t _sdSize;


#endif /* SD_H_ */