/*
 * API_uart.h
 *
 *  Created on: Apr 1, 2023
 *      Author: segundo
 */

#ifndef API_INC_API_UART_H_
#define API_INC_API_UART_H_

#include "stdint.h"
#include "stdbool.h"
#include "assert.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"



typedef uint32_t tick_t;
typedef bool bool_t;

bool_t uartInit();
void uartSendString(uint8_t * pstring);
void uartSendStringSize(uint8_t * pstring, uint16_t size);
void uartReceiveStringSize(uint8_t * pstring, uint16_t size);


#endif /* API_INC_API_UART_H_ */
