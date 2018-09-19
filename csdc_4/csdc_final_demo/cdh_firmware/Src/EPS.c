#include "eps.h"

I2C_HandleTypeDef hi2c1;

/*
* 	Purpose:
* 	This function will get the current status of either the EPS or Battery module
*
*	Reference:
*	Section 12.4.1 for Battery, 11.3 for EPS
*
* 	Parameters:
* 		address - target address (either EPS or Battery)
* 		data_received - array to hold received bytes
*/
void getStatus(uint8_t address, uint8_t data_received[]) {
	uint8_t command = GET_STATUS;
	uint8_t data1 = DEFAULT_DATA;

	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&command, 1, 50) != HAL_OK);
	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&data1, 1, 50) != HAL_OK);
	HAL_Delay(1);

	while (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(address << 1) | 0x01, data_received, 2, 50) != HAL_OK);
}


/*
* 	Purpose:
*	This function will send a command to either the EPS or Battery module to request data
*
*	Reference:
* 	Section 12 for Battery, 11 for EPS
*
*	Parameters:
*		address : target address (either EPS or Battery)
*		command : request command to send
*		data_sent : additional data byte to be sent along with the command
*		data_received : array to hold received bytes
*		bytes_received: number of bytes to request from the module
*		delay: delay required before a response can be requested, in milliseconds
*/
void sendCommand(uint8_t address, uint8_t command, uint8_t data_sent, uint8_t data_received[], uint8_t bytes_received, int delay) {
	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&command, 1, 50) != HAL_OK);
	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&data_sent, 1, 50) != HAL_OK);

	HAL_Delay(delay);

	if (bytes_received != 0) {
		while (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(address << 1) | 0x01, data_received, (uint16_t)bytes_received, 50) != HAL_OK);
	}
}


/*
* 	Purpose:
* 	This function will get the last error message of either the EPS or Batt module
*
* 	Reference:
* 	Section 12.4.2 for Battery, 11.3 for EPS.
*
* 	Parameters:
* 		address - target address (either EPS or Battery)
* 		data_received - array to hold received bytes
*/
void getError(uint8_t address, uint8_t data_received[]) {
	uint8_t command = GET_ERROR;
	uint8_t data1 = DEFAULT_DATA;

	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&command, 2, 50) != HAL_OK);
	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&data1, 2, 50) != HAL_OK);

	HAL_Delay(1);

	while (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(address << 1) | 0x01, data_received, (uint16_t)2, (uint32_t)50) != HAL_OK);
}


/* 	Purpose:
* 	This function will retrieve a specific piece of telemetry information for either the battery or EPS module
*
* 	Reference:
* 	Section 12.6 for Battery, 11.4 for EPS
*
* 	Parameters:
* 		address: target address (either EPS or Battery)
* 		command: telemetry request command (0x10)
* 		data1: first telemetry code to send
* 		data0: second telemetry code to send
* 		data_received: array to hold received bytes
* 		bytes_received: number of bytes to request from the module
*/
void getTelemetry(uint8_t address, uint8_t data1, uint8_t data0, uint8_t data_received[], uint8_t bytes_received) {
	uint8_t command = GET_TELEMETRY;

	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&command, 1, 50) != HAL_OK);
	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&data1, 1, 50) != HAL_OK);
	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&data0, 1, 50) != HAL_OK);
	HAL_Delay(15);



	if (bytes_received != 0) {
		while (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(address << 1) | 0x01, data_received, (uint16_t)bytes_received, 50) != HAL_OK);
	}
}


/*
* 	Purpose:
* 	This function will manually reset either the EPS or Batt module
*
* 	Reference:
* 	Section 12.4.5 for Battery, 11.3 for EPS
*
* 	Parameters:
* 		address - the target I2C Address. Either EPS or Batt in this case
*/
void manualReset(uint8_t address) {
	uint8_t command = MANUAL_RESET;
	uint8_t data0 = DEFAULT_DATA;

	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&command, 2, 50) != HAL_OK);
	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, (uint8_t*)&data0, 2, 50) != HAL_OK);
}


/*
* 	Purpose:
* 	Converts raw ADC values into proper values through equations found on battery datasheet
*
* 	Parameters:
* 		ADCdata: - raw ADC value given from the EPS or BAT module
* 		telem_code - two byte integer representing which telemetry was gathered
*
* 	Return:
* 		converted: The converted ADC value
*/
double convertBATADC(uint8_t ADCdata[], uint16_t telem_code) {
	uint16_t adcResponse = (uint16_t)ADCdata[0] + ((uint16_t)ADCdata[1] * 256);
	double converted = 0.0;

	switch (telem_code) {
	case 0xE280: //battery output voltage
		converted = adcResponse * 0.008993;
		break;
	case 0xE284: //battery current magnitude
		converted = adcResponse * 14.662757;
		break;
	case 0xE28E: //battery current direction
		if (adcResponse < 512)
			converted = 1.0;
		else
			converted = 0.0;
		break;
	case 0xE308: //motherboard temperature
		converted = 0.372434 * adcResponse - 273.15;
		break;
	case 0xE214: //current draw of 5V bus
		converted = 1.327547 * adcResponse;
		break;
	case 0xE210: //output voltage of 5V bus
		converted = 0.005965 * adcResponse;
		break;
	case 0xE204: //current draw of 3.3V bus
		converted = 1.327547 * adcResponse;
		break;
	case 0xE200: //output voltage of 3.3V bus
		converted = 0.004311 * adcResponse;
		break;
	case 0xE398: //daughterboard 1 temperature
		converted = 0.397600 * adcResponse - 238.57;
		break;
	case 0xE39F: //daughterboard 1 heater status
		if (adcResponse < 512)
			converted = 1.0;
		else
			converted = 0.0;
		break;
	case 0xE3A8: //daughterboard 2 temperature
		converted = 0.397600 * adcResponse - 238.57;
		break;
	case 0xE3AF: //daughterboard 2 heater status
		if (adcResponse < 512)
			converted = 1.0;
		else
			converted = 0.0;
		break;
	case 0xE3B8: //daughterboard 3 temperature
		converted = 0.397600 * adcResponse - 238.57;
		break;
	case 0xE3BF: //daughterboard 3 heater status
		if (adcResponse < 512)
			converted = 1.0;
		else
			converted = 0.0;
		break;
	default:
		converted = 0.0;
		break;
	}

	return converted;
}

/*
* 	Purpose:
* 	Converts raw ADC values into proper values through equations found on EPS datasheet
*
* 	Parameters:
* 		ADCdata: - raw ADC value given from the EPS or BAT module
* 		telem_code - two byte integer representing which telemetry was gathered
*
* 	Return:
* 		converted: The converted ADC value
*/
double convertEPSADC(uint8_t ADCdata[], uint16_t telem_code) {
	uint16_t adcResponse = (uint16_t)ADCdata[0] + ((uint16_t)ADCdata[1] * 256);
	double converted = 0;

	switch (telem_code) {
	case 0xE284: //BCR output current
		converted = 14.662757 * adcResponse;
		break;
	case 0xE280: //BCR output voltage
		converted = 0.008993157 * adcResponse;
		break;
	case 0xE205: //3.3V current draw
		converted = 0.001327547 * adcResponse;
		break;
	case 0xE215: //5V current draw
		converted = 0.001327547 * adcResponse;
		break;
	case 0xE234: //output current of 12V bus
		converted = 0.00207 * adcResponse;
		break;
	case 0xE230: //output voltage of 12V bus
		converted = 0.01349 * adcResponse;
		break;
	case 0xE224: //output current of battery bus
		converted = 0.005237 * adcResponse;
		break;
	case 0xE220: //output voltage of battery bus
		converted = 0.008978 * adcResponse;
		break;
	case 0xE214: //output current of 5V bus
		converted = 0.005237 * adcResponse;
		break;
	case 0xE210: //output voltage of 5V bus
		converted = 0.005865 * adcResponse;
		break;
	case 0xE204: //output current of 3.3V bus
		converted = 0.005237 * adcResponse;
		break;
	case 0xE200: //output voltage of 3.3V bus
		converted = 0.004311 * adcResponse;
		break;
	case 0xE410: //output voltage switch 1
		converted = 0.01349 * adcResponse;
		break;
	case 0xE414: //output current switch 1
		converted = 0.001328 * adcResponse;
		break;
	case 0xE420: //output voltage switch 2
		converted = 0.01349 * adcResponse;
		break;
	case 0xE424: //output current switch 2
		converted = 0.001328 * adcResponse;
		break;
	case 0xE430: //output voltage switch 3
		converted = 0.008993 * adcResponse;
		break;
	case 0xE434: //output current switch 3
		converted = 0.001328 * adcResponse;
		break;
	case 0xE440: //output voltage switch 4
		converted = 0.008993 * adcResponse;
		break;
	case 0xE444: //output current switch 4
		converted = 0.001328 * adcResponse;
		break;
	case 0xE450: //output voltage switch 5
		converted = 0.005865 * adcResponse;
		break;
	case 0xE454: //output current switch 5
		converted = 0.001328 * adcResponse;
		break;
	case 0xE460: //output voltage switch 6
		converted = 0.005865 * adcResponse;
		break;
	case 0xE464: //output current switch 6
		converted = 0.001328 * adcResponse;
		break;
	case 0xE470: //output voltage switch 7
		converted = 0.005865 * adcResponse;
		break;
	case 0xE474: //output current switch 7
		converted = 0.001328 * adcResponse;
		break;
	case 0xE480: //output voltage switch 8
		converted = 0.004311 * adcResponse;
		break;
	case 0xE484: //output current switch 8
		converted = 0.001328 * adcResponse;
		break;
	case 0xE490: //output voltage switch 9
		converted = 0.004311 * adcResponse;
		break;
	case 0xE494: //output current switch 9
		converted = 0.001328 * adcResponse;
		break;
	case 0xE4A0: //output voltage switch 10
		converted = 0.004311 * adcResponse;
		break;
	case 0xE4A4: //output current switch 10
		converted = 0.001328 * adcResponse;
		break;
	case 0xE308: //motherboard temperature
		converted = 0.372434 * adcResponse - 273.15;
		break;
	case 0xE110: //voltage feeding BCR1
		converted = 0.0249 * adcResponse;
		break;
	case 0xE114: //current BCR1, connector SA1A
		converted = 0.0009775 * adcResponse;
		break;
	case 0xE115: //current BCR1, connector SA1B
		converted = 0.0009775 * adcResponse;
		break;
	case 0xE120: //voltage feeding BCR2
		converted = 0.0249 * adcResponse;
		break;
	case 0xE124: //current BCR2, connector SA2A
		converted = 0.0009775 * adcResponse;
		break;
	case 0xE125: //current BCR2, connector SA2B
		converted = 0.0009775 * adcResponse;
		break;
	case 0xE130: //voltage feeding BCR3
		converted = 0.0099706 * adcResponse;
		break;
	case 0xE134: //current BCR3, connector SA3A
		converted = 0.0009775 * adcResponse;
		break;
	case 0xE135: //current BCR3, connector SA3B
		converted = 0.0009775 * adcResponse;
		break;
	default:
		converted = 0.0;
		break;
	}
	return converted;
}

/*
* 	Purpose:
* 	Take in a two byte integer and convert it to two one byte integers.
*
* 	Parameters:
* 		to_convert - two byte integer to convert
* 		converted - points to the now split two byte integer,
* 					the first integer is data0, the second is data1
*/
void convertCommand(uint16_t to_convert, uint8_t converted[]) {
	converted[0] = to_convert & 0xFF;
	converted[1] = (to_convert >> 8) & 0xFF;
}
