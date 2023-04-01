/*
 * API_uart.c
 *
 *  Created on: Apr 1, 2023
 *      Author: segundo
 */

#include "API_uart.h"

static UART_HandleTypeDef UartHandle;
#define USARTx                           USART3


/**
 * Inicializar la UART
 *
 * Inicializa la UART con los parámetros por defecto y envía por serial la configuración inicial.
 *
 * @fn bool_t uartInit()
 * @return true si se inicializó bien
 */
bool_t uartInit(){
	/*##-1- Configure the UART peripheral ######################################*/
	  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	  /* UART configured as follows:
	      - Word Length = 8 Bits (7 data bit + 1 parity bit) :
		                  BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
	      - Stop Bit    = One Stop bit
	      - Parity      = ODD parity
	      - BaudRate    = 9600 baud
	      - Hardware flow control disabled (RTS and CTS signals) */
	  UartHandle.Instance        = USARTx;

	  UartHandle.Init.BaudRate   = 9600;
	  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	  UartHandle.Init.StopBits   = UART_STOPBITS_1;
	  UartHandle.Init.Parity     = UART_PARITY_ODD;
	  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	  UartHandle.Init.Mode       = UART_MODE_TX_RX;
	  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	  if (HAL_UART_Init(&UartHandle) != HAL_OK)
	  {
	    return false;
	  }
	  uartSendString("{UartHandle : "
			  	  	  "\n{\t Instance : USARTx, "
			  	  	  "\n\t Init : "
			  	  	  	  "\n\t{\t  BaudRate: 9600, "
			  	  	  	  "\n\t\t WordLength: UART_WORDLENGTH_8B, "
			  	  	  	  "\n\t\t StopBits: UART_STOPBITS_1, "
			  	  	  	  "\n\t\t Parity: UART_PARITY_ODD, "
			  	  	  	  "\n\t\t HwFlowCtl: UART_HWCONTROL_NONE, "
			  	  	  	  "\n\t\t Mode: UART_MODE_TX_RX, "
			  	  	  	  "\n\t\t OverSampling : UART_OVERSAMPLING_16 "
			  	  	  	  "\n\t\t}"
			  	  	  	  "\n\t}"
			  	  	  	  "\n}\n");
	  return true;

}

/**
 * @fn void uartSendString(uint8_t*)
 * Envía string por uart
 *
 * La función chequea si el puntero al string no es nulo,
 * y luego lo envía usando la función de la HAL, calculando el
 * tamaño del string con strlen()
 *
 * @param pstring puntero al string, que debe terminar con \0
 */
void uartSendString(uint8_t * pstring){

	assert(pstring != NULL);
	HAL_UART_Transmit(&UartHandle, pstring, strlen(pstring), 0xFFFF);

}

/**
 * @fn void uartSendStringSize(uint8_t*, uint16_t)
 * Envía string por uart con el tamaño especificado
 *
 * La función chequea si el puntero al string no es nulo y si
 * el tamaño no es cero, luego usa la función de la HAL para enviarlo.
 *
 *
 * @param pstring puntero al string que se desea enviar
 * @param size tamaño del string que se desa enviar
 */
void uartSendStringSize(uint8_t * pstring, uint16_t size){
	assert(pstring != NULL);
	assert(size!=0);
	HAL_UART_Transmit(&UartHandle, pstring, size, 0xFFFF);

}

/**
 * @fn void uartReceiveStringSize(uint8_t*, uint16_t)
 * Recibe el string por UART
 *
 * La función chequea si el puntero del buffer no es nulo y si el tamaño no es cero.
 * Si se cumple, llama a la función de la HAL para almacenar los datos recibidos
 * en el buffer.
 *
 * @param pstring puntero al buffer de datos donde se almacena el string
 * @param size tamaño del string que se recibe
 */
void uartReceiveStringSize(uint8_t * pstring, uint16_t size){
	assert(pstring != NULL);
	assert(size!=0);
	HAL_UART_Receive(&UartHandle, pstring, size, 0xFFFF);
}
