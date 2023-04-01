/*
 * API_delay.c
 *
 *  Created on: Mar 18, 2023
 *      Author: segundo
 */
#include "API_delay.h"

/**
 * Initializes the non-blocking delay
 *
 * La función recibe un puntero a una estructura de delay
 * y una duración en millisegundos, e inicializa esa estructura
 * con el tiempo indicado.
 *
 * @param delay apunta a la estructura de delay que hay que inicializar
 * @param duration es la duración en milisegundoas del delay
 * */
void delayInit(delay_t *delay, tick_t duration) {
	assert(duration >0);
	assert(delay != NULL);
	delay->duration = duration;
	delay->running = false;
}

/**
 * Reads the time passed since the delay began or starts counting time
 *
 * La función recibe un puntero a una estructura de delay. Si la estructura
 * no se encuentra corriendo, inicializa el tiempo con el contador de tick, si
 * se encuentra corriendo, verifica que no se haya pasado el tiempo.
 *
 * @param delay apunta a la estructura de delay que se lee
 * @returns true si se cumplió el tiempo o false si no
 * */
bool_t delayRead(delay_t *delay) {
	assert(delay!=NULL);
	if (!delay->running) {
		delay->running = true;
		delay->startTime = HAL_GetTick();
		return false;
	} else {
		if ((HAL_GetTick() - delay->startTime) >= delay->duration) {
			delay->running = false;
			return true;
		} else
			return false;
	}
}

/**
 * Writes a new duration to the delay structure
 *
 * La función recibe un puntero a una estructura de delay y una duración, y
 * sobreescribe la duración recibida en la estructura de delay.
 *
 * @param delay apunta a la estructura de delay cuya duración se sobreescribe
 * @param duration es la nueva duración que se impone en la estructura de delay
 * @returns void
 * */
void delayWrite(delay_t *delay, tick_t duration) {
	assert(duration >0);
	assert(delay != NULL);
	delay->duration = duration;
}
