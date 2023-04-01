/*
 * API_debounce.c
 *
 *  Created on: Mar 25, 2023
 *      Author: leonardo
 */

#include "API_debounce.h"

static debounceState_t debounce;	// Variable interna de maquina de estados
static bool_t pressed = false;		//Variable si el boton se presiono
static delay_t delayFSM;


/**
 *Inicializa la MEF del debounce
 *
 *Inicializa la maquina de estados con boton en alto, inicializaliza el pulsador
 *
 * @returns
 */
void debounceFSM_init() {
	debounce = BUTTON_UP;
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
	delayInit(&delayFSM, FSM_UPDATE_PERIOD);
}


/**
 * Implementación de la MEF
 *
 * Chequea los cuatro estados, y si se encuentra en estado de transición
 * y se cumplió el tiempo de delay cambia al próixmo estado.
 *
 * @returns
 */
void debounceFSM_update() {
	switch (debounce) {
	case BUTTON_UP:
		if (BSP_PB_GetState(BUTTON_USER)) {
			debounce = BUTTON_FALLING;
			delayRead(&delayFSM);
		}
		break;
	case BUTTON_FALLING:
		if (delayRead(&delayFSM) && BSP_PB_GetState(BUTTON_USER)) {
			debounce = BUTTON_DOWN;
			pressed = true;		//Variable que usuario lee a traves de readKey
		} else {
			debounce = BUTTON_UP;
		}
		break;
	case BUTTON_DOWN:
		if (!BSP_PB_GetState(BUTTON_USER)) {
			debounce = BUTTON_RAISING;
			delayRead(&delayFSM);
		}
		break;
	case BUTTON_RAISING:
		if (delayRead(&delayFSM) && !BSP_PB_GetState(BUTTON_USER)) {
			debounce = BUTTON_UP;
		} else {
			debounce = BUTTON_DOWN;
		}
		break;
	default:
		break;
	}
}

//Funcion intermeria para acceder interna pressed
/**
 * @fn bool_t readKey()
 * @return true si se encuentra presionado
 */
bool_t readKey() {
	if (pressed){
		pressed=false;
		return true;
	}
	else
		return false;
}
