/*
 * API_debounce.h
 *
 *  Created on: Mar 25, 2023
 *      Author: leonardo
 */

#ifndef API_INC_API_DEBOUNCE_H_
#define API_INC_API_DEBOUNCE_H_

#include "stdbool.h"
#include "stm32f4xx_hal.h"  		/* <- HAL include */
#include "stm32f4xx_nucleo_144.h" /* <- BSP include */
#include "API_uart.h"

void debounceFSM_init();
void debounceFSM_update();

bool readKey();

typedef enum{
BUTTON_UP,
BUTTON_FALLING,
BUTTON_DOWN,
BUTTON_RAISING,
} debounceState_t;

typedef bool bool_t;

#endif /* API_INC_API_DEBOUNCE_H_ */
