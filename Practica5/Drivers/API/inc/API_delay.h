#ifndef __API_DELAY_H
#define __API_DELAY_H

#include "stdint.h"
#include "stdbool.h"
#include "assert.h"
#include "stm32f4xx_hal.h"  		/* <- HAL include */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
typedef uint32_t tick_t;
typedef bool bool_t;

typedef struct{
   tick_t startTime;
   tick_t duration;
   bool_t running;
} delay_t;

void delayInit( delay_t * delay, tick_t duration );
bool_t delayRead( delay_t * delay );
void delayWrite( delay_t * delay, tick_t duration );


#endif
