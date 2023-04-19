/*
 * PN532.h
 *
 *  Created on: Apr 18, 2023
 *      Author: segundo
 */

#ifndef PN532_INC_PN532_H_
#define PN532_INC_PN532_H_

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
typedef bool bool_t;
typedef enum{
	PN532_OK,
	PN532_CMD_ERROR,
	PN532_ACK_NOT_RECEIVED,
	PN532_BUSY,
	PN532_RESPONSE_ERROR,
	PN532_PARAM_ERROR,
	PN532_EMPTY
} PN532_response_t;

typedef struct{
	uint8_t IC;
	uint8_t version;
	uint8_t revision;
	uint8_t support;
} PN532_firmware_t;

typedef struct{
	uint8_t logical_number;
	uint8_t SENS_RES[2];
	uint8_t SEL_RES;
	uint8_t NFCID_length;
	uint8_t NFCID[4];
} PN532_target_t;

bool pn532Driver_I2C_init();
PN532_response_t pn532Driver_I2C_getFirmware(PN532_firmware_t* firmware);
PN532_response_t pn532Driver_I2C_configureSAM();
PN532_response_t pn532Driver_I2C_listPassiveTarget(PN532_target_t* target);
PN532_response_t pn532Driver_I2C_readMifareData(uint8_t* buffer, uint8_t len, PN532_target_t target);





#endif /* PN532_INC_PN532_H_ */
