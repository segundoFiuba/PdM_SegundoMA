/*
 * API_debounce.c
 *
 *  Created on: Mar 25, 2023
 *      Author: leonardo
 */

#include "API_debounce.h"

static debounceState_t debounce;	// Variable interna de maquina de estados
static bool_t pressed = false;		//Variable si el boton se presiono

//Inicializa la maquina de estados con boton en alto, inicializaliza el pulsador
void debounceFSM_init() {
	debounce = BUTTON_UP;
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
}

//Implementacion de maquina de estados, se debe checkear periodicamente
void debounceFSM_update() {
	switch (debounce) {
	case BUTTON_UP:
		if (BSP_PB_GetState(BUTTON_USER)) {
			debounce = BUTTON_FALLING;
		}
		break;
	case BUTTON_FALLING:
		if (BSP_PB_GetState(BUTTON_USER)) {
			debounce = BUTTON_DOWN;
			pressed = true;		//Variable que usuario lee a traves de readKey
			uartSendString("FLANCO DESCENDENTE \n");
		} else {
			debounce = BUTTON_UP;
		}
		break;
	case BUTTON_DOWN:
		if (!BSP_PB_GetState(BUTTON_USER)) {
			debounce = BUTTON_RAISING;
		}
		break;
	case BUTTON_RAISING:
		if (!BSP_PB_GetState(BUTTON_USER)) {
			debounce = BUTTON_UP;
			uartSendString("FLANCO ASCENDENTE \n");
		} else {
			debounce = BUTTON_DOWN;
		}
		break;
	default:
		break;
	}
}

//Funcion intermeria para acceder interna pressed
bool_t readKey() {
	if (pressed){
		pressed=false;
		return true;
	}
	else
		return false;
}
