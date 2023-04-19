/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

typedef struct
 {
    I2C_HandleTypeDef* instance;
    uint16_t sdaPin;
    GPIO_TypeDef* sdaPort;
    uint16_t sclPin;
    GPIO_TypeDef* sclPort;
} I2C_Module_t;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void I2C_ClearBusyFlagErratum(I2C_Module_t* i2c, uint32_t timeout);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const uint8_t PN532_ADDRESS = 0x24<<1;
static const uint8_t PN532_REG = 0x00;
#define PN532_BUFFER_SIZE   8
uint8_t pn532_buffer[PN532_BUFFER_SIZE];


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	HAL_StatusTypeDef ret;
	int16_t val;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  I2C_Module_t i2cModule = {&hi2c1, GPIO_PIN_9, GPIOB, GPIO_PIN_8, GPIOB};
  uint8_t buff[12];
  uint8_t pn532_buffer[PN532_BUFFER_SIZE];

  while (1)
  {
	// Send the command to configure SAM without IRQ
	pn532_read_firmware_version();

	HAL_Delay(1000);
	configure_reader();
	HAL_Delay(1000);
	wait_for_input();


  }
  /* USER CODE END 3 */
}

int arrays_equal(uint8_t* arr1, uint8_t* arr2, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (arr1[i] != arr2[i]) {
            return 0;
        }
    }
    return 1;
}

void wait_for_input(){
	uint8_t command_inListPassiveTarget [] = {0x00,0x00,0xFF, //Preamble
			0x04, //LEN
			0xFC, //LCS (LEN+LCS=X00)
			0xD4, //TFI
			0x4A, //CMD
			0x01, //
			0x00, //
			0xE1, //DCS TFI+CMDN=X00
			0x00
	};
	HAL_I2C_Master_Transmit(&hi2c1, PN532_ADDRESS, command_inListPassiveTarget, sizeof(command_inListPassiveTarget), HAL_MAX_DELAY);
	HAL_Delay(1);
	uint8_t pn532AckFrame[7]={0x01,0x00,0x00,0xFF,0x00,0xFF,0x00};
	uint8_t pn532ReceivedFrame[7];
	HAL_I2C_Master_Receive(&hi2c1, PN532_ADDRESS, pn532ReceivedFrame, sizeof(pn532ReceivedFrame), HAL_MAX_DELAY);

	char * string_ack_received = "InListPassiveTarget ACK Received \n";
	char * string_ack__not_received = "InListPassiveTarget NOT Received \n";

	if(arrays_equal(pn532ReceivedFrame,pn532AckFrame, 7 )){
		HAL_UART_Transmit(&huart3, string_ack_received, strlen(string_ack_received), HAL_MAX_DELAY);
	}
	else {
		HAL_UART_Transmit(&huart3, string_ack__not_received, strlen(string_ack__not_received), HAL_MAX_DELAY);
	}
	HAL_Delay(10);

	uint8_t detected = 0;

	while (!detected){
		uint8_t response_Read[50] = {0x00}; // Response buffer
		HAL_I2C_Master_Receive(&hi2c1, PN532_ADDRESS, response_Read, sizeof(response_Read), HAL_MAX_DELAY);
		char * string_firmware_received = "InListPassiveTarget Received \n";
		char * string_firmware__not_received = "InListPassiveTarget NOT Received \n";
		uint8_t preamble[4] = {0x01, 0x00, 0x00, 0xFF};
		uint8_t firmware_response_code[2] = {0xD5, 0x4B};
		if(arrays_equal(preamble,response_Read, 4 ) && arrays_equal(firmware_response_code, response_Read+6*sizeof(uint8_t), 2)){
			HAL_UART_Transmit(&huart3, string_firmware_received, strlen(string_firmware_received), HAL_MAX_DELAY);
			detected = response_Read[8];
			char * string_card_received = "DETECTED CARD!!! \n";
			if (detected) {
				HAL_UART_Transmit(&huart3, string_card_received, strlen(string_card_received), HAL_MAX_DELAY);
			}
		}
		else {
			HAL_UART_Transmit(&huart3, string_firmware__not_received, strlen(string_firmware__not_received), HAL_MAX_DELAY);
		}

		HAL_Delay(1000);
	}
}

void configure_reader(void) {
	uint8_t command_configure_SAM [] = {0x00,0x00,0xFF, //Preamble
			0x05, //LEN
			0xFB, //LCS (LEN+LCS=X00)
			0xD4, //TFI
			0x14, //CMD
			0x01, //
			0x03, //
			0x00, //
			0x14, //DCS TFI+CMDN=X00
			0x00
	};

	HAL_I2C_Master_Transmit(&hi2c1, PN532_ADDRESS, command_configure_SAM, sizeof(command_configure_SAM), HAL_MAX_DELAY);
	HAL_Delay(1);
	uint8_t pn532AckFrame[7]={0x01,0x00,0x00,0xFF,0x00,0xFF,0x00};
	uint8_t pn532ReceivedFrame[7];
	HAL_I2C_Master_Receive(&hi2c1, PN532_ADDRESS, pn532ReceivedFrame, sizeof(pn532ReceivedFrame), HAL_MAX_DELAY);

	char * string_ack_received = "SAM CONFIG ACK Received \n";
	char * string_ack__not_received = "SAM CONFIG ACK NOT Received \n";

	if(arrays_equal(pn532ReceivedFrame,pn532AckFrame, 7 )){
		HAL_UART_Transmit(&huart3, string_ack_received, strlen(string_ack_received), HAL_MAX_DELAY);
	}
	else {
		HAL_UART_Transmit(&huart3, string_ack__not_received, strlen(string_ack__not_received), HAL_MAX_DELAY);
	}

	HAL_Delay(1);

	uint8_t response_SAM[20] = {0x00}; // Response buffer
	HAL_I2C_Master_Receive(&hi2c1, PN532_ADDRESS, response_SAM, sizeof(response_SAM), HAL_MAX_DELAY);
	char * string_firmware_received = "SAM Received \n";
	char * string_firmware__not_received = "SAM NOT Received \n";
	uint8_t preamble[4] = {0x01, 0x00, 0x00, 0xFF};
	uint8_t firmware_response_code[2] = {0xD5, 0x15};
	if(arrays_equal(preamble,response_SAM, 4 ) && arrays_equal(firmware_response_code, response_SAM+6*sizeof(uint8_t), 2)){
		HAL_UART_Transmit(&huart3, string_firmware_received, strlen(string_firmware_received), HAL_MAX_DELAY);
	}
	else {
		HAL_UART_Transmit(&huart3, string_firmware__not_received, strlen(string_firmware__not_received), HAL_MAX_DELAY);

	}

}

void pn532_read_firmware_version(void)
{
	uint8_t commandGetFirmware[] = {0x00, 0x00, 0xFF, //Preamble
			0x02, //(LEN) Length of msg (D4+cmd)
			0xFE, //(LCS) Checksum: LEN+LCS = X00
			0xD4, //(TFI) From Host to PN532
			0x02, //(CMD) Command Get Firmware
			0x2A, //(DCS) checksum: TFI+CMD = X00
			0x00  // Postamble
	};


	HAL_I2C_Master_Transmit(&hi2c1, PN532_ADDRESS, commandGetFirmware, sizeof(commandGetFirmware), HAL_MAX_DELAY);
	HAL_Delay(1);

	uint8_t pn532AckFrame[7]={0x01,0x00,0x00,0xFF,0x00,0xFF,0x00};
	uint8_t pn532ReceivedFrame[7];
	HAL_I2C_Master_Receive(&hi2c1, PN532_ADDRESS, pn532ReceivedFrame, sizeof(pn532ReceivedFrame), HAL_MAX_DELAY);

	char * string_ack_received = "ACK Received \n";
	char * string_ack__not_received = "ACK NOT Received \n";

	if(arrays_equal(pn532ReceivedFrame,pn532AckFrame, 7 )){
		HAL_UART_Transmit(&huart3, string_ack_received, strlen(string_ack_received), HAL_MAX_DELAY);
	}
	else {
		HAL_UART_Transmit(&huart3, string_ack__not_received, strlen(string_ack__not_received), HAL_MAX_DELAY);

	}
	HAL_Delay(1);

	uint8_t response_firmware[20] = {0x00}; // Response buffer
	HAL_I2C_Master_Receive(&hi2c1, PN532_ADDRESS, response_firmware, sizeof(response_firmware), HAL_MAX_DELAY);
	char * string_firmware_received = "Firmware Received \n";
	char * string_firmware__not_received = "Firmware NOT Received \n";
	uint8_t preamble[4] = {0x01, 0x00, 0x00, 0xFF};
	uint8_t firmware_response_code[2] = {0xD5, 0x03};
	if(arrays_equal(preamble,response_firmware, 4 ) && arrays_equal(firmware_response_code, response_firmware+6*sizeof(uint8_t), 2)){
		HAL_UART_Transmit(&huart3, string_firmware_received, strlen(string_firmware_received), HAL_MAX_DELAY);
		char firmware_string[50];
		sprintf(firmware_string, "Firmware- IC: %02X , ver: %02X, rev: %02X, supp: %02X \n", response_firmware[8],response_firmware[9],response_firmware[10],response_firmware[11] );
		HAL_UART_Transmit(&huart3, firmware_string, strlen(firmware_string), HAL_MAX_DELAY);

	}
	else {
		HAL_UART_Transmit(&huart3, string_firmware__not_received, strlen(string_firmware__not_received), HAL_MAX_DELAY);

	}
    //HAL_UART_Transmit(&huart3, (uint8_t*)version_string, strlen(version_string), HAL_MAX_DELAY);
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}


static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout)
 {
    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = true;
    /* Wait until flag is set */
    for(;(state != HAL_GPIO_ReadPin(port, pin)) && (true == ret);)
    {
        /* Check for the timeout */
        if (timeout != HAL_MAX_DELAY)
        {
            if ((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout))
            {
                ret = false;
            }
            else
            {
            }
        }
        asm("nop");
    }
    return ret;
}

static void I2C_ClearBusyFlagErratum(I2C_Module_t* i2c, uint32_t timeout)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    I2C_HandleTypeDef* handler = NULL;

    handler = i2c->instance;

    // 1. Clear PE bit.
    CLEAR_BIT(handler->Instance->CR1, I2C_CR1_PE);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_I2C_DeInit(handler);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = i2c->sclPin;
    HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = i2c->sdaPin;
    HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET, timeout);

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);

    // 5. Check SDA Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET, timeout);

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);

    // 7. Check SCL Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET, timeout);

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET, timeout);

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET, timeout);

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C2;

    GPIO_InitStructure.Pin = i2c->sclPin;
    HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = i2c->sdaPin;
    HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(handler->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(handler->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(handler->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    // Call initialization function.
    HAL_I2C_Init(handler);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
