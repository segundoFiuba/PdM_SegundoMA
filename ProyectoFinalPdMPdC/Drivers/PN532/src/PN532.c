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
#define LENGTH_OF_DATA_RESPONSE_CODE 3
#define LENGTH_OF_FIRMWARE_RESPONSE 4
#define LENGTH_OF_SAM_CONFIGURE_RESPONSE_CODE 2
#define LENGTH_OF_INLISTPASSIVETARGET_RESPONSE_CODE 2
#define LENGTH_LEN_LCS 2
#define LENGTH_OF_POSTAMBLE 1

static uint8_t pn532_address = 0x24<<1;
static bool pn532Driver_initialized=false;
static bool searchingTarget=false;
static bool card_found=false;
static PN532_target_t found_target;

typedef struct {
	uint8_t command[60];
	uint8_t len;
} command_t;
static const uint8_t command_get_firmware[] = {0x00, 0x00, 0xFF, //Preamble
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
		0x05,
		0xB6, //DCS TFI+CMDN=X00
		0x00
};

static uint8_t command_auth1 [] = {0x00,0x00,0xFF, //Preamble
		0x0F, //LEN
		0xF1, //LCS (LEN+LCS=X00)
		0xD4, //TFI
		0x40, //CMD
		0x01,
		0x61,
		0x05,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xB3,
		0x2B,
		0x2B,
		0x94,
		0xEE, //DCS TFI+CMDN=X00
		0x00
};

static uint8_t command_configureTiming [] = {0x00,0x00,0xFF, //Preamble
		0x06, //LEN
		0xFA, //LCS (LEN+LCS=X00)
		0xD4, //TFI
		0x32, //CMD
		0x02,
		0x00,
		0x0B,
		0x0D,
		0xE0, //DCS TFI+CMDN=X00
		0x00
};

/**
 * @fn bool arrays_equal(uint8_t*, uint8_t*, uint8_t)
 * @brief checks if two arrays are equal. Usefull for validating incoming responses
 * @param arr1 first array
 * @param arr2 second array
 * @param len length to compare
 * @return true if are equal for that length, false otherwise
 */
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
static uint8_t configTiming_response_code[2] = {0xD5, 0x33};
static command_t pn532Driver_I2C_createCommand(uint8_t * input_command, uint8_t len);
static uint8_t calculate_lcs(uint8_t length);
static uint8_t calculate_DCS(uint8_t *data, uint8_t len);

/**
 * @fn bool pn532_get_pn532Driver_initialized()
 * @brief getter for pn532Driver_initialized
 * @return pn532Driver_initialized
 */
bool pn532_get_pn532Driver_initialized(){
	return pn532Driver_initialized;
}
/**
 * @fn uint8_t pn532_get_pn532_address()
 * @brief getter for pn532_address
 * @return pn532_address
 */
uint8_t pn532_get_pn532_address(){
	return pn532_address;
}
/**
 * @fn void pn532_set_pn532_address(uint8_t)
 * @brief setter for pn532_address
 * @param address address to set
 */
void pn532_set_pn532_address(uint8_t address){
	pn532_address = address;
}

/**
 * @fn bool pn532_get_card_found()
 * @brief getter for card_found
 * @return card_found
 */
bool pn532_get_card_found(){
	return card_found;
}

/**
 * @fn bool pn532Driver_I2C_init()
 * @brief i2c driver initializer
 * @return true if initialized correctly
 */
bool pn532Driver_I2C_init(){
	if(pn532Driver_I2C_portNucleo_init()){
		pn532Driver_initialized=true;
		return true;
	}
	return false;
}

/**
 * @fn bool pn532Driver_I2C_deinit()
 * @brief i2c driver deinitializer
 * @return true if deinit correctly
 */
bool pn532Driver_I2C_deinit(){
	if(pn532Driver_I2C_portNucleo_deinit()){
		pn532Driver_initialized=false;
		return true;
	}
	return false;
}

/**
 * @fn bool receive_ACK()
 * @brief private function to receive an ack from the pn532 and validate it
 * @return true if ack received and validated
 */
static bool receive_ACK(){
	uint8_t ackBuffer[7]={0};

	if(!pn532Driver_I2C_portNucleo_receiveToBuffer(ackBuffer, LENGTH_OF_ACK_FRAME, pn532_address)){
		return false;
	}
	return arrays_equal(pn532AckFrame,ackBuffer, LENGTH_OF_ACK_FRAME );
}

/**
 * @fn PN532_response_t pn532Driver_I2C_getFirmware(PN532_firmware_t*)
 * @brief function to get Firmware
 * @param firmware pointer to PN532_firmware_t where firmware info will be stored
 * @return PN532_response_t with result of firmware read action
 */
PN532_response_t pn532Driver_I2C_getFirmware(PN532_firmware_t* firmware){
	if(firmware ==  NULL) return PN532_PARAM_ERROR;

	uint8_t responseBuffer[LENGTH_OF_FIRMWARE_RESPONSE+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE+LENGTH_OF_POSTAMBLE];

	uint8_t command1[2] = {0xD4, 0x02};
	command_t test_command_get_firmware = pn532Driver_I2C_createCommand(command1, 2);

	if(!pn532Driver_I2C_portNucleo_sendCommand(command_get_firmware, sizeof(command_get_firmware) , pn532_address)){
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

/**
 * @fn PN532_response_t pn532Driver_I2C_configureSAM()
 * @brief function to configure SAM
 * @return response with status of action
 */
PN532_response_t pn532Driver_I2C_configureSAM(){

	uint8_t responseBuffer[9];

	uint8_t command1[5] = {0xD4, //TFI
			0x14, //CMD
			0x01, //
			0x03, //
			0x00,};
	command_t test_command_configure = pn532Driver_I2C_createCommand(command1, 5);

	if(!pn532Driver_I2C_portNucleo_sendCommand(command_configure_SAM, command_configure_SAM, pn532_address)){
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

/**
 * @fn PN532_response_t pn532Driver_I2C_listPassiveTarget(PN532_target_t*)
 * @brief command to begin listening for devices
 * @param target pointer to target that will be filled with target data once detected
 * @return response with result of action
 */
PN532_response_t pn532Driver_I2C_listPassiveTarget(PN532_target_t * target){
	if(target==NULL) return PN532_PARAM_ERROR;

	uint8_t responseBuffer[20];

	uint8_t command1[4] = {0xD4, //TFI
			0x4A, //CMD
			0x01, //
			0x00,};
	command_t test_command_list = pn532Driver_I2C_createCommand(command1, 4);

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

	found_target.logical_number = responseBuffer[1+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	found_target.SENS_RES[0] = responseBuffer[2+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	found_target.SENS_RES[1] = responseBuffer[3+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	found_target.SEL_RES = responseBuffer[4+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	found_target.NFCID_length = responseBuffer[5+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	found_target.NFCID[0] = responseBuffer[6+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	found_target.NFCID[1] = responseBuffer[7+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	found_target.NFCID[2] = responseBuffer[8+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	found_target.NFCID[3] = responseBuffer[9+LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_FIRMWARE_RESPONSE_CODE];
	target->logical_number = found_target.logical_number;
	target->SENS_RES[0] = found_target.SENS_RES[0];
	target->SENS_RES[1] = found_target.SENS_RES[1];
	target->SEL_RES = found_target.SEL_RES;
	target->NFCID_length = found_target.NFCID_length;
	target->NFCID[0] = found_target.NFCID[0];
	target->NFCID[1] = found_target.NFCID[1];
	target->NFCID[2] = found_target.NFCID[2];
	target->NFCID[3] = found_target.NFCID[3];
	card_found=true;
	searchingTarget=false;
	return PN532_OK;
}

/**
 * @fn PN532_response_t pn532Driver_I2C_readMifareData_sans_target(uint8_t*, uint8_t)
 * @brief read data from currently initialized target
 * @param buffer to store the read data
 * @param len length of the buffer
 * @return response with result of action
 */
PN532_response_t pn532Driver_I2C_readMifareData_sans_target(uint8_t* buffer, uint8_t len){
	card_found=false;
	return pn532Driver_I2C_readMifareData(buffer, len, found_target);
}

/**
 * @fn PN532_response_t pn532Driver_I2C_readMifareData(uint8_t*, uint8_t, PN532_target_t)
 * @brief read data from currently initialized target
 * @param buffer to store the read data
 * @param len length of the buffer
 * @return response with result of action
 */
PN532_response_t pn532Driver_I2C_readMifareData(uint8_t* buffer, uint8_t len, PN532_target_t target){

	uint8_t responseBuffer[100]={0x00};

	if(!pn532Driver_I2C_portNucleo_sendCommand(command_auth1, sizeof(command_auth1), pn532_address)){
		return PN532_CMD_ERROR;
	}

	portNucleo_Delay(1);

	if(!receive_ACK()){
		return PN532_ACK_NOT_RECEIVED;
	}

	portNucleo_Delay(10);

	if(!pn532Driver_I2C_portNucleo_receiveToBuffer(responseBuffer, sizeof(responseBuffer), pn532_address)){
		return PN532_RESPONSE_ERROR;
	}

	if(!pn532Driver_I2C_portNucleo_sendCommand(command_inDataExchange, sizeof(command_inDataExchange), pn532_address)){
		return PN532_CMD_ERROR;
	}

	portNucleo_Delay(1);

	if(!receive_ACK()){
		return PN532_ACK_NOT_RECEIVED;
	}

	portNucleo_Delay(10);

	if(!pn532Driver_I2C_portNucleo_receiveToBuffer(responseBuffer, sizeof(responseBuffer), pn532_address)){
		return PN532_RESPONSE_ERROR;
	}
	if(!arrays_equal(preamble, responseBuffer, LENGTH_OF_PREAMBLE)
			|| !arrays_equal(inDataExchange_response_code, responseBuffer+(LENGTH_LEN_LCS+LENGTH_OF_PREAMBLE)*sizeof(uint8_t), LENGTH_OF_FIRMWARE_RESPONSE_CODE)) {
		return PN532_RESPONSE_ERROR;
	}

	for(int i = 0; i<len; i++){
		buffer[i] = responseBuffer[LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_DATA_RESPONSE_CODE+i];
	}

	return PN532_OK;

}

/**
 * @fn PN532_response_t pn532Driver_I2C_readMifareData_full(uint8_t*, uint8_t)
 * @brief function to read the full card
 * @param buffer
 * @param len
 * @return response according to result
 */
PN532_response_t pn532Driver_I2C_readMifareData_full(uint8_t* buffer, uint8_t len){

	uint8_t responseBuffer[100]={0x00};

	for(int i =0x00; i<0xBB; i++){
		command_auth1[0x09]=i;
		command_auth1[0x14]=0xF3-i;

		if(!pn532Driver_I2C_portNucleo_sendCommand(command_auth1, sizeof(command_auth1), pn532_address)){
				return PN532_OK;
			}

			portNucleo_Delay(10);

			if(!receive_ACK()){
				return PN532_OK;
			}

			portNucleo_Delay(10);

			if(!pn532Driver_I2C_portNucleo_receiveToBuffer(responseBuffer, sizeof(responseBuffer), pn532_address)){
				return PN532_OK;
			}

			command_inDataExchange[9]=i;
			command_inDataExchange[10]=0xBB-i;
			if(!pn532Driver_I2C_portNucleo_sendCommand(command_inDataExchange, sizeof(command_inDataExchange), pn532_address)){
				return PN532_OK;
			}

			portNucleo_Delay(10);

			if(!receive_ACK()){
				return PN532_OK;
			}

			portNucleo_Delay(10);

			if(!pn532Driver_I2C_portNucleo_receiveToBuffer(responseBuffer, sizeof(responseBuffer), pn532_address)){
				return PN532_OK;
			}
			if(!arrays_equal(preamble, responseBuffer, LENGTH_OF_PREAMBLE)
					|| !arrays_equal(inDataExchange_response_code, responseBuffer+(LENGTH_LEN_LCS+LENGTH_OF_PREAMBLE)*sizeof(uint8_t), LENGTH_OF_FIRMWARE_RESPONSE_CODE)) {
				return PN532_OK;
			}

			for(int j = 0; j<16; j++){
				buffer[i*16+j] = responseBuffer[LENGTH_OF_PREAMBLE+LENGTH_LEN_LCS+LENGTH_OF_DATA_RESPONSE_CODE+j];
			}
	}
	return PN532_OK;

}

/**
 * @fn PN532_response_t pn532Driver_I2C_configureTiming()
 * @brief function to configure timing
 * @return status according to result of action
 */
PN532_response_t pn532Driver_I2C_configureTiming(){

	uint8_t responseBuffer[100]={0x00};

	if(!pn532Driver_I2C_portNucleo_sendCommand(command_configureTiming, sizeof(command_configureTiming), pn532_address)){
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
			|| !arrays_equal(configTiming_response_code, responseBuffer+(LENGTH_LEN_LCS+LENGTH_OF_PREAMBLE)*sizeof(uint8_t), LENGTH_OF_FIRMWARE_RESPONSE_CODE)) {
		return PN532_RESPONSE_ERROR;
	}


	return PN532_OK;

}

/**
 * @fn command_t pn532Driver_I2C_createCommand(uint8_t*, uint8_t)
 * @brief function to create the full command from the short command
 * @param input_command short command including 0xD4
 * @param len length of command including D4
 * @return
 */
static command_t pn532Driver_I2C_createCommand(uint8_t * input_command, uint8_t len){
	command_t output_command;
	output_command.command[0] = 0x00;
	output_command.command[1] = 0x00;
	output_command.command[2] = 0xFF;
	output_command.command[3] = len;
	output_command.command[4] = calculate_lcs(len);
	for(int i = 0; i<len;i++){
		output_command.command[5+i] = input_command[i];
	}
	output_command.command[5+len] = calculate_DCS(input_command, len);
	output_command.command[6+len] = 0x00;

	output_command.len = len+7;

	return output_command;
}

/**
 * @fn uint8_t calculate_lcs(uint8_t)
 * @brief function to calculate LCS checksum
 * @param length length to calculate the checksum
 * @return
 */
static uint8_t calculate_lcs(uint8_t length) {
    return ~(length-1) & 0xFF;
}

/**
 * @fn uint8_t calculate_DCS(uint8_t*, uint8_t)
 * @brief function to calculate DCS checksum
 * @param data full command to calculate the checksum
 * @param len length of the command
 * @return
 */
static uint8_t calculate_DCS(uint8_t *data, uint8_t len) {
    uint8_t dcs = 0xFF;
    for (int i = 0; i < len; i++) {
        uint16_t sum = dcs + data[i];
        if (sum > 0xFF) {
            sum -= 0xFF;
            sum--;
        }
        dcs = sum;
    }
    return ~(dcs-1);
}

