/*
 * PN532.c
 *
 *  Created on: Apr 18, 2023
 *      Author: segundo
 */

#include "PN532.h"
#define LENGTH_OF_ACK_FRAME 7
#define LENGTH_OF_PREAMBLE 4
#define LENGTH_OF_FIRMWARE_RESPONSE_CODE 2
#define LENGTH_OF_FIRMWARE_RESPONSE 4
#define LENGTH_OF_SAM_CONFIGURE_RESPONSE_CODE 2
#define LENGTH_OF_INLISTPASSIVETARGET_RESPONSE_CODE 2
#define LENGTH_LEN_LCS 2
#define LENGTH_OF_POSTAMBLE 1

static uint8_t pn532_address = 0x24<<1;
static bool pn532Driver_initialized=false;
static bool searchingTarget=false;
static const uint8_t commandGetFirmware[] = {0x00, 0x00, 0xFF, //Preamble
		0x02, //(LEN) Length of msg (D4+cmd)
		0xFE, //(LCS) Checksum: LEN+LCS = X00
		0xD4, //(TFI) From Host to PN532
		0x02, //(CMD) Command Get Firmware
		0x2A, //(DCS) checksum: TFI+CMD = X00
		0x00  // Postamble
};
static uint8_t command_configure_SAM [] = {0x00,0x00,0xFF, //Preamble
		0x05, //LEN
		0xFB, //LCS (LEN+LCS=X00)
		0xD4, //TFI
		0x14, //CMD
		0x01, //
		0x03, //
		0x00, //
		0x14, //DCS TFI+CMDN=X00
		0x00
};
static uint8_t command_inListPassiveTarget [] = {0x00,0x00,0xFF, //Preamble
		0x04, //LEN
		0xFC, //LCS (LEN+LCS=X00)
		0xD4, //TFI
		0x4A, //CMD
		0x01, //
		0x00, //
		0xE1, //DCS TFI+CMDN=X00
		0x00
};
static uint8_t command_inDataExchange [] = {0x00,0x00,0xFF, //Preamble
		0x05, //LEN
		0xFB, //LCS (LEN+LCS=X00)
		0xD4, //TFI
		0x40, //CMD
		0x01,
		0x30,
		0x00,
		0xBA, //DCS TFI+CMDN=X00
		0x00
};

static bool arrays_equal(uint8_t* arr1, uint8_t* arr2, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (arr1[i] != arr2[i]) {
            return false;
        }
    }
    return true;
}

static const uint8_t pn532AckFrame[7]={0x01,0x00,0x00,0xFF,0x00,0xFF,0x00};
static uint8_t preamble[4] = {0x01, 0x00, 0x00, 0xFF};
static uint8_t firmware_response_code[2] = {0xD5, 0x03};
static uint8_t SAM_configure_response_code[2] = {0xD5, 0x15};
static uint8_t inListPassiveTarget_response_code[2] = {0xD5, 0x4B};
static uint8_t inDataExchange_response_code[2] = {0xD5, 0x41};

bool get_pn532Driver_initialized(){
	return pn532Driver_initialized;
}

uint8_t get_pn532_address(){
	return pn532_address;
}

void set_pn532_address(uint8_t address){
	pn532_address = address;
}

bool pn532Driver_I2C_init(){
	pn532Driver_I2C_portNucleo_init();
	pn532Driver_initialized=true;
}

bool pn532Driver_I2C_deinit(){
	pn532Driver_I2C_portNucleo_deinit();
	pn532Driver_initialized=false;
}

static bool receive_ACK(){
	uint8_t ackBuffer[7]={0};

	if(!pn532Driver_I2C_portNucleo_receiveToBuffer(ackBuffer, LENGTH_OF_ACK_FRAME, pn532_address)){
		return false;
	}
	if(!arrays_equal(pn532AckFrame,ackBuffer, LENGTH_OF_ACK_FRAME )){
		return false;
	}
	return true;
}

PN532_response_t pn532Driver_I2C_getFirmware(PN532_firmware_t* firmware){

	uint8_t* responseBuffer[LENGTH_OF_FIRMWARE_RESPONSE+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE+LENGTH_OF_POSTAMBLE];

	if(!pn532Driver_I2C_portNucleo_sendCommand(commandGetFirmware, sizeof(commandGetFirmware), pn532_address)){
		return PN532_CMD_ERROR;
	}

	portNucleo_Delay(1);

	if(!receive_ACK()){
		return PN532_ACK_NOT_RECEIVED;
	}

	portNucleo_Delay(1);

	if(!pn532Driver_I2C_portNucleo_receiveToBuffer(responseBuffer, sizeof(responseBuffer), pn532_address)){
		return PN532_RESPONSE_ERROR;
	}
	if(!arrays_equal(preamble, responseBuffer, LENGTH_OF_PREAMBLE)
			|| !arrays_equal(firmware_response_code, responseBuffer+(LENGTH_LEN_LCS+LENGTH_OF_PREAMBLE)*sizeof(uint8_t), LENGTH_OF_FIRMWARE_RESPONSE_CODE)) {
		return PN532_RESPONSE_ERROR;
	}

	firmware->IC=responseBuffer[LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	firmware->version=responseBuffer[LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE+1];
	firmware->revision=responseBuffer[LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE+2];
	firmware->support=responseBuffer[LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE+3];

	return PN532_OK;
}

PN532_response_t pn532Driver_I2C_configureSAM(){

	uint8_t responseBuffer[9];

	if(!pn532Driver_I2C_portNucleo_sendCommand(command_configure_SAM, sizeof(command_configure_SAM), pn532_address)){
		return PN532_CMD_ERROR;
	}
	portNucleo_Delay(1);

	if(!receive_ACK()){
		return PN532_ACK_NOT_RECEIVED;
	}

	portNucleo_Delay(1);

	if(!pn532Driver_I2C_portNucleo_receiveToBuffer(responseBuffer, sizeof(responseBuffer), pn532_address)){
		return PN532_RESPONSE_ERROR;
	}
	if(!arrays_equal(preamble, responseBuffer, LENGTH_OF_PREAMBLE)
			|| !arrays_equal(SAM_configure_response_code, responseBuffer+(LENGTH_LEN_LCS+LENGTH_OF_PREAMBLE)*sizeof(uint8_t), LENGTH_OF_SAM_CONFIGURE_RESPONSE_CODE)) {
		return PN532_RESPONSE_ERROR;
	}
	return PN532_OK;
}

PN532_response_t pn532Driver_I2C_listPassiveTarget(PN532_target_t * target){

	uint8_t responseBuffer[20];

	if(!searchingTarget){
		if(!pn532Driver_I2C_portNucleo_sendCommand(command_inListPassiveTarget, sizeof(command_inListPassiveTarget), pn532_address)){
			return PN532_CMD_ERROR;
		}

		portNucleo_Delay(1);

		if(!receive_ACK()){
			return PN532_ACK_NOT_RECEIVED;
		}
		searchingTarget=true;
	}


	portNucleo_Delay(1);
	if(!pn532Driver_I2C_portNucleo_receiveToBuffer(responseBuffer, sizeof(responseBuffer), pn532_address)){
		return PN532_RESPONSE_ERROR;
	}
	if(responseBuffer[0]==0x00){
		return PN532_EMPTY;
	}

	target->logical_number = responseBuffer[1+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	target->SENS_RES[0] = responseBuffer[2+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	target->SENS_RES[1] = responseBuffer[3+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	target->SEL_RES = responseBuffer[4+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	target->NFCID_length = responseBuffer[5+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	target->NFCID[0] = responseBuffer[6+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	target->NFCID[1] = responseBuffer[7+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	target->NFCID[2] = responseBuffer[8+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	target->NFCID[3] = responseBuffer[9+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	searchingTarget=false;
	return PN532_OK;
}

PN532_response_t pn532Driver_I2C_readMifareData(uint8_t* buffer, uint8_t len, PN532_target_t target){

	uint8_t* responseBuffer[100];

	if(!pn532Driver_I2C_portNucleo_sendCommand(command_inDataExchange, sizeof(command_inDataExchange), pn532_address)){
		return PN532_CMD_ERROR;
	}

	portNucleo_Delay(1);

	if(!receive_ACK()){
		return PN532_ACK_NOT_RECEIVED;
	}

	portNucleo_Delay(1);

	if(!pn532Driver_I2C_portNucleo_receiveToBuffer(responseBuffer, sizeof(responseBuffer), pn532_address)){
		return PN532_RESPONSE_ERROR;
	}
	if(!arrays_equal(preamble, responseBuffer, LENGTH_OF_PREAMBLE)
			|| !arrays_equal(inDataExchange_response_code, responseBuffer+(LENGTH_LEN_LCS+LENGTH_OF_PREAMBLE)*sizeof(uint8_t), LENGTH_OF_FIRMWARE_RESPONSE_CODE)) {
		return PN532_RESPONSE_ERROR;
	}

	for(int i = 0; i<len-LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE; i++){
		buffer[i] = responseBuffer[LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE+i];
	}

	return PN532_OK;

}

