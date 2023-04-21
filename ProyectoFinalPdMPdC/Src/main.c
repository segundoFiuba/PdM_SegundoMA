/**
 ******************************************************************************
 * @file    UART/UART_Printf/Src/main.c
 * @author  MCD Application Team
 * @brief   This example shows how to retarget the C library printf function
 *          to the UART.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "API_delay.h"		//Typedef y funciones de delay no bloqueante
#include "API_debounce.h"
#include "API_uart.h"
#include "PN532.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#define FSM_UPDATE_PERIOD 10
#define DEBUG
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config(void);
static void Error_Handler(void);
static char* hex_string(uint8_t *array, size_t length);
static void fsm_update();
static void fsm_init();
static void LOG(char*);
static void configure_pn532();
static void read_firmware_pn532();
static void init_read_pn532();
static void read_full_card();
static void read_specific_block();
fsm_state_t fsm_state;


/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
	/* STM32F4xx HAL library initialization:
	 - Configure the Flash prefetch
	 - Systick timer is configured by default as source of time base, but user
	 can eventually implement his proper time base source (a general purpose
	 timer for example or other time source), keeping in mind that Time base
	 duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
	 handled in milliseconds basis.
	 - Set NVIC Group Priority to 4
	 - Low Level Initialization
	 */
	HAL_Init();
	SystemClock_Config();

	delay_t delayFSM;

	delayInit(&delayFSM, FSM_UPDATE_PERIOD);
	fsm_init();

	while(true){
		if (delayRead(&delayFSM)) {	//Acciones al cumplir el periodo de interrupción
			fsm_update();
		}
	}
}

static void fsm_update(){
	switch(fsm_state) {
	case FSM_INIT: {
		LOG("[FSM_INIT]\n");
		fsm_init();
		fsm_state = FSM_READ_FIRMWARE_PN532;
		break;
	}
	case FSM_READ_FIRMWARE_PN532: {
		LOG("[FSM_READ_FIRMWARE_PN532]\n");
		read_firmware_pn532();
		fsm_state=FSM_CONFIGURE_PN532;
		break;
	}
	case FSM_CONFIGURE_PN532: {
		LOG("[FSM_CONFIGURE_PN532]\n");
		configure_pn532();
		fsm_state = FSM_READY_TO_READ_FULL;
	}
	case FSM_READY_TO_READ_FULL: {
		//LOG("[FSM_READY_TO_READ_FULL]\n");
		if(read_button()){
			fsm_state = FSM_READY_TO_READ_SPECIFIC;
			break;
		}
		if (pn532_get_card_found()){
			fsm_state = FSM_READ_FULL_CARD;
			break;
		}
		init_read_pn532();
		break;
	}
	case FSM_READ_FULL_CARD: {
		LOG("[FSM_READ_FULL_CARD]\n");
		read_full_card();
		fsm_state=FSM_READY_TO_READ_FULL;
		break;
	}
	case FSM_READY_TO_READ_SPECIFIC: {
		LOG("[FSM_READY_TO_READ_SPECIFIC]\n");
		if(read_button()){
			fsm_state = FSM_READY_TO_READ_FULL;
			break;
		} else if (pn532_get_card_found()){
			fsm_state = FSM_READ_SPECIFIC;
			break;
		}
		init_read_pn532();
		break;
	}
	case FSM_READ_SPECIFIC: {
		LOG("[FSM_READ_SPECIFIC]\n");
		read_specific_block();
		fsm_state=FSM_READY_TO_READ_SPECIFIC;
		break;
	}
	case FSM_DEINIT: LOG("[FSM_DEINIT]\n");break;
	case FSM_ERROR:
		LOG("[FSM_ERROR]\n");
		fsm_state=FSM_INIT;
		break;
	default: {
		LOG("[default]\n");
		fsm_state= FSM_ERROR;
		break;
	}
	}
}


static void fsm_init(){
	if(!uartInit()){
		LOG("[UART_INIT_ERROR]\n");
	}
	if(!pn532_get_pn532Driver_initialized()){
		if(!pn532Driver_I2C_init()){
			LOG("[I2C_INIT_ERROR]\n");
		}
	}
	fsm_state=FSM_READ_FIRMWARE_PN532;
}

static void read_firmware_pn532(){
	PN532_response_t res;
	PN532_firmware_t firmwareBuffer;
	res = pn532Driver_I2C_getFirmware(&firmwareBuffer);
	switch(res) {
		case PN532_CMD_ERROR: LOG("FW CMD ERROR \n"); break;
		case PN532_ACK_NOT_RECEIVED: LOG("FW ACK ERROR \n"); break;
		case PN532_RESPONSE_ERROR: LOG("FW RESPONSE ERROR \n"); break;
		case PN532_OK: {
			char firmware_string[50];
			sprintf(firmware_string, "Firmware- IC: %02X , ver: %02X, rev: %02X, supp: %02X \n", firmwareBuffer.IC,firmwareBuffer.version,firmwareBuffer.revision,firmwareBuffer.support);
			LOG(firmware_string);
			break;
		}
		default: LOG("UNKNOWN ERROR");
	}
}

static void configure_pn532(){
	PN532_response_t res;
	res = pn532Driver_I2C_configureSAM();
	switch(res) {
		case PN532_CMD_ERROR: LOG("SAM CMD ERROR \n"); break;
		case PN532_ACK_NOT_RECEIVED: LOG("SAM ACK ERROR \n"); break;
		case PN532_RESPONSE_ERROR: LOG("SAM RESPONSE ERROR \n"); break;
		case PN532_OK: LOG("SAM SUCCESFULLY CONFIGURED \n"); break;
		default: LOG("SAM UNKNOWN ERROR"); break;
	}
	HAL_Delay(10);
	res = pn532Driver_I2C_configureTiming();
	switch(res) {
		case PN532_CMD_ERROR: LOG("TIME CMD ERROR \n"); break;
		case PN532_ACK_NOT_RECEIVED: LOG("TIME ACK ERROR \n"); break;
		case PN532_RESPONSE_ERROR: LOG("TIME RESPONSE ERROR \n"); break;
		case PN532_OK: LOG("TIME SUCCESFULLY CONFIGURED \n"); break;
		default: LOG("TIME UNKNOWN ERROR"); break;
	}
}

static void init_read_pn532(){
	PN532_response_t res;
	PN532_target_t targetBuffer;
	res = pn532Driver_I2C_listPassiveTarget(&targetBuffer);
	switch(res) {
		case PN532_CMD_ERROR: LOG("LIST CMD ERROR \n"); break;
		case PN532_ACK_NOT_RECEIVED: LOG("LIST ACK ERROR \n"); break;
		case PN532_RESPONSE_ERROR: LOG("LIST RESPONSE ERROR \n"); break;
		case PN532_EMPTY: LOG("."); break;
		case PN532_OK: {
			LOG("\nCARD FOUND\n");
			char target_string[100];
			sprintf(target_string, "Target- ln: %02X , SENS_RES: %02X %02X, SEL_RES: %02X, NFCID_Length: %02X, NFCID: %02X %02X %02X %02X \n",
					targetBuffer.logical_number,
					targetBuffer.SENS_RES[0],
					targetBuffer.SENS_RES[1],
					targetBuffer.SEL_RES,
					targetBuffer.NFCID_length,
					targetBuffer.NFCID[0],
					targetBuffer.NFCID[1],
					targetBuffer.NFCID[2],
					targetBuffer.NFCID[3]
					);
			LOG(target_string);
			fsm_state=FSM_READ_FULL_CARD;
			break;
		}
		default: LOG("LIST UNKNOWN ERROR"); break;
	}
}
static void read_full_card(){
	PN532_response_t res;
	uint8_t dataBuffer[1024];
	res = pn532Driver_I2C_readMifareData_full(dataBuffer, sizeof(dataBuffer));
	switch(res){
		case PN532_OK: LOG(hex_string(dataBuffer, sizeof(dataBuffer))); break;
		case PN532_ACK_NOT_RECEIVED: LOG("READ ACK ERROR \n"); break;
		case PN532_RESPONSE_ERROR: LOG("READ RESPONSE ERROR \n"); break;
		case PN532_EMPTY: LOG("."); break;
		default: LOG("READ UNKNOWN ERROR"); break;
	}
}

static void read_specific_block(){
	PN532_response_t res;
	uint8_t dataBuffer[100];
	res = pn532Driver_I2C_readMifareData_sans_target(dataBuffer, sizeof(dataBuffer));
	switch(res){
		case PN532_OK: LOG(hex_string(dataBuffer, sizeof(dataBuffer))); break;
		case PN532_ACK_NOT_RECEIVED: LOG("READ ACK ERROR \n"); break;
		case PN532_RESPONSE_ERROR: LOG("READ RESPONSE ERROR \n"); break;
		case PN532_EMPTY: LOG("."); break;
		default: LOG("READ UNKNOWN ERROR"); break;
	}
}
char* hex_string(uint8_t *array, size_t length) {
    char *output = malloc(length * 3 + (length / 16) + 1); // allocate space for hex string and newlines
    size_t i, j = 0;
    for (i = 0; i < length; i++) {
        if (i % 16 == 0 && i != 0) { // add newline every 16th item
            output[j++] = '\n';
        }
        sprintf(&output[j], "%02x ", array[i]); // convert byte to hex string
        j += 3; // move index to next hex string position
    }
    output[j] = '\0'; // null-terminate the string
    return output;
}

static void LOG(char* string){
#ifdef DEBUG
	uartSendString(string);
#endif
}
/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 180000000
 *            HCLK(Hz)                       = 180000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 360
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            PLL_R                          = 2
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is
	 clocked below the maximum system frequency, to update the voltage scaling value
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void) {
	/* Turn LED2 on */
	BSP_LED_On(LED2);
	while (1) {
	}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
