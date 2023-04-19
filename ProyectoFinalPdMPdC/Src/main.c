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
#define LED_PERIOD_1 100
#define LED_PERIOD_2 500

#define FSM_UPDATE_PERIOD 40

#define LED_TOGGLE LED_BLUE
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config(void);
static void Error_Handler(void);
static char* hex_string(uint8_t *array, size_t length);

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
	PN532_response_t res;
	HAL_Init();

	/* Configure the system clock to 180 MHz */
	SystemClock_Config();

	uartInit();

	uartSendString(pn532Driver_I2C_init()? "Init Success \n" : "Init Failure \n");

	PN532_firmware_t firmwareBuffer;
	res = pn532Driver_I2C_getFirmware(&firmwareBuffer);
	switch(res) {
		case PN532_CMD_ERROR: uartSendString("CMD ERROR \n"); break;
		case PN532_ACK_NOT_RECEIVED: uartSendString("ACK ERROR \n"); break;
		case PN532_RESPONSE_ERROR: uartSendString("RESPONSE ERROR \n"); break;
		case PN532_OK: {
			char firmware_string[50];
			sprintf(firmware_string, "Firmware- IC: %02X , ver: %02X, rev: %02X, supp: %02X \n", firmwareBuffer.IC,firmwareBuffer.version,firmwareBuffer.revision,firmwareBuffer.support);
			uartSendString(firmware_string);
			break;
		}
		default: uartSendString("UNKNOWN ERROR");
	}
	HAL_Delay(10);
	res = pn532Driver_I2C_configureSAM();
	switch(res) {
		case PN532_CMD_ERROR: uartSendString("CMD ERROR \n"); break;
		case PN532_ACK_NOT_RECEIVED: uartSendString("ACK ERROR \n"); break;
		case PN532_RESPONSE_ERROR: uartSendString("RESPONSE ERROR \n"); break;
		case PN532_OK: uartSendString("SAM SUCCESFULLY CONFIGURED \n"); break;
		default: uartSendString("UNKNOWN ERROR"); break;
	}
	HAL_Delay(10);
	/* Infinite loop */
	PN532_target_t targetBuffer;
	while (1) {
		res = pn532Driver_I2C_listPassiveTarget(&targetBuffer);
		switch(res) {
			case PN532_CMD_ERROR: uartSendString("CMD ERROR \n"); break;
			case PN532_ACK_NOT_RECEIVED: uartSendString("ACK ERROR \n"); break;
			case PN532_RESPONSE_ERROR: uartSendString("RESPONSE ERROR \n"); break;
			case PN532_EMPTY: uartSendString("."); break;
			case PN532_OK: {
				uartSendString("\nCARD FOUND\n");
				char target_string[50];
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
				uartSendString(target_string);
				uint8_t dataBuffer[100];
				HAL_Delay(1);
				res = pn532Driver_I2C_readMifareData(dataBuffer, sizeof(dataBuffer), targetBuffer);
				switch(res){
				case PN532_OK: uartSendString(hex_string(dataBuffer, sizeof(dataBuffer))); break;
				default: uartSendString("ERROR"); break;
				}

				break;
			}
			default: uartSendString("UNKNOWN ERROR"); break;
		}
		HAL_Delay(1000);

	}

}

static char* hex_string(uint8_t *array, size_t length) {
    char *result = (char*)malloc(length*3+1); // Allocate space for the string
    for (size_t i = 0; i < length; i++) {
        sprintf(result+i*3, "%02X ", array[i]); // Format the hex value and store it in the string
    }
    result[length*3] = '\0'; // Add null-terminator to the end of the string
    return result;
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
