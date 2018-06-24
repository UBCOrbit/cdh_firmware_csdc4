#include "comms.h"

UART_HandleTypeDef huart1;

// Description: This function writes null bytes to the buffer array passed to it
// Input: pointer to buffer that needs to be cleared
void clear_array(uint8_t *buf) {
	for (int i = 0; i < PACKET_SIZE; i++) {
		buf[i] = '\0';
	}
}

// Description: Uses the print_string_to_console function to print the contents
//        of the buffer to the serial monitor.
// Input: Pointer the to buffer to be printed and the length of the buffer
//      (number of char elements in the buffer)
/*void print_buffer(uint8_t *buf, int size) {
	char temp;
	for (uint8_t i = 0; i < size; i++) {
		temp = (char)*(buf + i);
		print_string_to_console(temp);
	}
}*/

// Description: Tries to receive a packet. If a packet is received, the function
//				saves the packet at the input pointer. Additionally, it returns 1 once
//        the packet has been saved. The function returns 0 if it is unable to
//				receive.
// Input: pointer to where the packet is to be saved
// Returns: Confirmation on successful reception of packet.
uint8_t receive_packet(uint8_t *buf) {
	// Creates temporary buffer for receiving the packet.
	uint8_t tempBuffer[PACKET_SIZE];

	// Saves the packet in the temporary buffer.
	if (HAL_UART_Receive(&huart1, tempBuffer, PACKET_SIZE, 0x0FFF) == HAL_OK) {
		for (int i = 0; i < PACKET_SIZE; i += 1) {
			*(buf + i) = tempBuffer[i];
		}
		return 1;
	}
	return 0;
}

//// Pulls a packet from the
//uint8_t mram_packet(uint8_t *pointer, uint8_t *mram_pointer) {
//
//}

// Description: Checks whether the packet follows the correct start bit
//        protocol.
// Input: Pointer to where the packet is stored.
uint8_t check_start_protocol(uint8_t *buf) {

	uint8_t holder[BYTE_SIZE];
	char buffer = *buf;
	for (int i = 7; 0 <= i; i--) {
		holder[7 - i] = ((buffer >> i) & 0x01);
	}
	if ((holder[0] == 0) && (holder[1] == 1) && (holder[2] == 1) &&
		(holder[3] == 0)) {
		return 1;
	}
	else {
		return 0;
	}
}

// Description: updates the address pointer with the value specified
// by the address field of the packet
//
// Input: pointer to where the packet is stored
void get_address(uint8_t *buf, uint8_t *adr) {
	uint8_t holder[BYTE_SIZE];
	uint8_t temp1 = *buf;
	for (int i = 7; 0 <= i; i--) {
		holder[7 - i] = ((temp1 >> i) & 0x01);
	}
	*adr = ((holder[4] * (4)) +
		(holder[5] * (2)) +
		(holder[6] * (1)));
}

// Description: updates the flag pointer with the value specified
// by the command/data flag field of the packet
//
// Input: pointer to where the packet is stored
void check_flag(uint8_t *buf, uint8_t *flg) {
	uint8_t holder[BYTE_SIZE];
	uint8_t temp1 = *buf;
	for (int i = 7; 0 <= i; i--) {
		holder[7 - i] = ((temp1 >> i) & 0x01);
	}
	*flg = holder[7];
}


// Description: updates the length_command pointer with the value specified
// by the length/command field of the packet
//
// Input: pointer to where the packet is stored
void get_lengthCommand(uint8_t *buf, uint8_t *len_command) {
	uint8_t holder[BYTE_SIZE];
	uint8_t temp = *(buf + 1);
	for (int i = 7; 0 <= i; i--) {
		holder[7 - i] = ((temp >> i) & 0x01);
	}
	*len_command = ((holder[0] * (128)) +
		(holder[1] * (64)) +
		(holder[2] * (32)) +
		(holder[3] * (16)) +
		(holder[4] * (8)) +
		(holder[5] * (4)) +
		(holder[6] * (2)) +
		(holder[7] * (1)));
}

// Description: Saves what is in the payload field of the saved packet into the data
// pointer.
void save_data(uint8_t *buf, uint8_t *data, uint8_t *len_command) {
	for (int x = 0; x < *len_command; x++) {
		*(data + x) = *(buf + 2 + x);
	}
}

void save_command(uint8_t * buf, uint8_t *data, uint8_t *len_command) {

}


// Description: This function parses packets received from COMMs. Checks each of the overhead
// fields and updates each of the relevant pointers. Additionally, it updates the payload
// field.
//
// Input: pointer to where the packet is saved, and the length of the packet
// Output: returns 1 if the packets was properly parsed and returns 0 if there was an issue
uint8_t parse_packet(uint8_t *buf, uint8_t *adr, uint8_t *flg, uint8_t *len_command, uint8_t *data) {
	if (check_start_protocol(buf) == 0) {
		return 0;
	}
	get_address(buf, adr);
	check_flag(buf, flg);
	get_lengthCommand(buf, len_command);
	if (*flg == 0) {
		save_command(buf, data, len_command);
	}
	else {
		save_data(buf, data, len_command);
	}
	return 1;
}