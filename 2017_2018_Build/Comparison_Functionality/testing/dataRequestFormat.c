// Warnings are for VS2017 to ignore safety errors.
#define _CRT_NONSTDC_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

char *formatRequest(long, long, int);
void parseRequest(long*, long*, char*, int);

void main() {
	long *size = (long*)malloc(sizeof(long));
	long *address = (long*)malloc(sizeof(long));
	char *fullString = formatRequest(5555, 999999, 16);

	printf("%s\n", fullString);
	parseRequest(size, address, fullString, 16);
}

// Description: Generate STM A's request for data from STM B
// Input: Integer address and size of message. Base to-be used for conversion.
// Side-Effects: Assuming a message format ########X########. Convert
//				 integral values to representative strings of specified base.
//				 Pads the address and size integral values with zeros. 
//				 Two messages are divided by an 'X'.
// Return: Pointer to the generated request message
char *formatRequest(long address, long size, int base) {
	// Convert integral address and size to strings
	char addressString[8] = "";
	itoa(address, addressString, base);

	char sizeString[8] = "";
	itoa(size, sizeString, base);

	char *fullString = (char*)malloc(sizeof(char) * 18);
	int i, count=0, nonZeroInd = 8-strlen(addressString);

	// Zero padding
	for (i = 0; i< nonZeroInd; i++) {
		fullString[i] = '0';
	}
	// Write address string
	for (i = 0; i<strlen(addressString); i++) {
		fullString[nonZeroInd +i] = addressString[i];
	}

	// Separation character
	fullString[8] = 'X';

	// Zero padding
	nonZeroInd = 17 - strlen(sizeString);
	for (i = 9; i < nonZeroInd; i++) {
		fullString[i] = '0';
	}
	for (i = 0; i < strlen(sizeString); i++) {
		fullString[nonZeroInd + i] = sizeString[i];
	}

	fullString[17] = '\0';
	return fullString;
}

// Description: Given a formatted request, parse it and extract integer address and size
// Side-Effects: Loop through padded zeros in address and size strings to extract address
//				 and size. Convert to decimal integers based on specified base.
//				 Store converted values in designated pointers.
// Input: Pointer to request string. Pointers to address and size integers.
void parseRequest(long *size, long *address, char *reqString, int base) {
	int count = 0, i;
	char *addressString = (char*)malloc(sizeof(char) * 8);
	char *sizeString = (char*)malloc(sizeof(char) * 8);

	// Skip padding
	while (*(reqString+count) == '0') {
		count++;
	}
	
	// Parse address
	i = 0;
	while (reqString[count] != 'X') {
		*(addressString+i) = reqString[count];
		count++;
		i++;
	}
	addressString[i] = '\0';
	// Skip the X
	count++;

	// Skip padding
	while (reqString[count] == '0') {
		count++;
	}

	// Parse size
	i = 0;
	while (reqString[count] != '\0') {
		*(sizeString+i) = reqString[count];
		count++;
		i++;
	}
	sizeString[i] = '\0';

	// Store converted size and address integers
	*size = strtol(sizeString, NULL, base);
	*address = strtol(addressString, NULL, base);
}