#ifndef EPS_H_
#define EPS_H_

#include "stm32f4xx_hal.h"

/*GLOBAL VARIABLES-------------------------------------*/
//I2C Addresses for EPS and Battery
#define EPS_ADDRESS 0x2B
#define BAT_ADDRESS 0x54

//Common Commands
#define DEFAULT_DATA 0x00
#define GET_STATUS 0x01
#define GET_ERROR 0x03
#define GET_VERSION 0x04
#define GET_CHECKSUM 0x05
#define GET_TELEMETRY 0x10
#define GET_BROWNOUT_RESETS 0x31
#define GET_AUTO_SOFTWARE_RESETS 0x32
#define GET_MANUAL_RESETS 0x33
#define MANUAL_RESET 0x80

//EPS Commands
#define EPS_GET_WATCHDOG_PERIOD 0x20
#define EPS_SET_WATCHDOG_PERIOD 0x21
#define EPS_RESET_WATCHDOG 0x22
#define EPS_GET_WATCHDOG_RESETS 0x34
#define EPS_ALLPDM_ON 0x40
#define EPS_ALLPDM_OFF 0x41
#define EPS_GET_ALLPDM_STATE 0x42
#define EPS_GET_ALLPDM_INITIAL_STATE 0x44
#define EPS_SET_ALLPDM_TO_INITIAL_STATE 0x45
#define EPS_SET_PDM_ON 0x50
#define EPS_SET_PDM_OFF 0x51
#define EPS_SET_PDM_INITIAL_STATE_ON 0x52
#define EPS_SET_PDM_INITIAL_STATE_OFF 0x53
#define EPS_GET_PDM_STATUS 0x54
#define EPS_PCM_RESET 0x70

//EPS Get Telemetry Commands
#define I_IDIODE 0xE284
#define V_IDIODE 0xE280
#define I_3V3_DRW 0xE205
#define I_5V_DRW 0xE215
#define I_PCM12V 0xE234
#define V_PCM12V 0xE230
#define I_PCMBATV 0xE224
#define V_PCMBATV 0xE220
#define I_PCM5V 0xE214
#define V_PCM5V 0xE210
#define I_PCM3V3 0xE204
#define V_PCM3V3 0xE200

#define V_SW1 0xE410
#define I_SW1 0xE414
#define V_SW2 0xE420
#define I_SW2 0xE424
#define V_SW3 0xE430
#define I_SW3 0xE434
#define V_SW4 0xE440
#define I_SW4 0xE444
#define V_SW5 0xE440
#define I_SW5 0xE454
#define V_SW6 0xE460
#define I_SW6 0xE464
#define V_SW7 0xE470
#define I_SW7 0xE474
#define V_SW8 0xE480
#define I_SW8 0xE484
#define V_SW9 0xE490
#define I_SW9 0xE494
#define V_SW10 0xE4A0
#define I_SW10 0xE4A4
#define MBRD_TEMP 0xE308

#define V_BCR1 0xE110
#define I_BCR1A 0xE114
#define I_BCR1B 0xE115
#define BCR1A_TEMP 0xE118
#define BCR1B_TEMP 0xE119
#define BCR1A_SUN_DETECT 0xE11C
#define BCR1B_SUN_DETECT 0xE11D

#define V_BCR2 0xE120
#define I_BCR2A 0xE124
#define I_BCR2B 0xE125
#define BCR2A_TEMP 0xE128
#define BCR2B_TEMP 0xE129
#define BCR2A_SUN_DETECT 0xE12C
#define BCR2B_SUN_DETECT 0xE12D

#define V_BCR3 0xE130
#define I_BCR3A 0xE134
#define I_BCR3B 0xE135
#define BCR3A_TEMP 0xE138
#define BCR3B_TEMP 0xE139
#define BCR3A_SUN_DETECT 0xE13C
#define BCR3B_SUN_DETECT 0xE13D

//BAT Commands
#define BAT_GET_HEATER_CONTROLLER_STATUS 0x90
#define BAT_SET_HEATER_CONTROLLER_STATUS 0x91
#define HEATER_OFF 0x00
#define HEATER_ON 0x01

//BAT Telemetry Commands
#define V_BAT 0xE280
#define I_BAT 0xE284
#define I_DIR_BAT 0xE28E
#define MBRD_TMP 0xE308
#define I_PCM5V 0xE214
#define V_PCM5V 0xE210
#define I_PCM3V3 0xE204
#define V_PCM3V3 0xE200
#define DBAT1_TEMP 0xE398
#define DBAT1_HEATER_STATUS 0xE239F
#define DBAT2_TEMP 0xE3A8
#define DBAT2_HEATER_STATUS 0xE3AF
#define DBAT3_TEMP 0xE3B8
#define DBAT3_HEATER_STATUS 0xE3CF

/*FUNCTION PROTOTYPES ---------------------------------------------*/
void getStatus(uint8_t address, uint8_t data_received[]);
void sendCommand(uint8_t address, uint8_t command, uint8_t data_sent, uint8_t data_received[], uint8_t bytes_received, int delay);
void getError(uint8_t address, uint8_t data_received[]);
void getTelemetry(uint8_t address, uint8_t data1, uint8_t data0, uint8_t data_received[], uint8_t bytes_received);
void manualReset(uint8_t address);
void convertCommand(uint16_t to_convert, uint8_t converted[]);
double convertEPSADC(uint8_t ADCdata[], uint16_t telem_code);
double convertBATADC(uint8_t ADCdata[], uint16_t telem_code);

#endif
