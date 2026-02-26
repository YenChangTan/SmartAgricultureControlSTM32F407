/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SystemContext systemContext = {0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t modbusSwitchCase = 0;
ModbusMaster mb2;

TCPContext tcpContext = {0};

static Output outputs[OutputCount] = {
    [PumpA] = {GPIOA, GPIO_PIN_12, GPIO_PIN_RESET, 0},
    [ValveA] = {GPIOA, GPIO_PIN_11, GPIO_PIN_RESET, 0},
    [PumpB] = {GPIOA, GPIO_PIN_8, GPIO_PIN_RESET, 0},
    [ValveB] = {GPIOC, GPIO_PIN_9, GPIO_PIN_RESET, 0},
    [FertigationPump] = {GPIOB, GPIO_PIN_3, GPIO_PIN_RESET, 0},
    [FertigationValve1] = {GPIOB, GPIO_PIN_4, GPIO_PIN_RESET, 0},
    [FertigationValve2] = {GPIOB, GPIO_PIN_5, GPIO_PIN_RESET, 0},
};

static Input inputs[InputCount] = {
    [MixedWaterLow] = {GPIOD, GPIO_PIN_0, GPIO_PIN_RESET},
    [MixedWaterHigh] = {GPIOD, GPIO_PIN_1, GPIO_PIN_RESET},
    [MixedWaterLimit] = {GPIOD, GPIO_PIN_2, GPIO_PIN_RESET},
	[CleanWaterLow] = {GPIOD, GPIO_PIN_3, GPIO_PIN_RESET},
	[ABLow] = {GPIOD, GPIO_PIN_4, GPIO_PIN_RESET}
};

static Output TCPDirectionalPin = {GPIOC, GPIO_PIN_4, GPIO_PIN_RESET,0};
uint8_t test;

static uint32_t CleanWaterWateringTimeOut = 3600000;
static uint32_t MixedWaterWateringTimeOut = 3600000;
static uint32_t CleanWaterFillingStopbyTimingTimeOut = 3600000;
static uint32_t CleanWaterFillingStopbyHighSensorTimeOut = 3600000;
static uint32_t ABFertilizerAddingbyECTimeOut = 3600000;
static uint32_t* timeoutList[5] = {
    &CleanWaterWateringTimeOut,
    &MixedWaterWateringTimeOut,
    &CleanWaterFillingStopbyTimingTimeOut,
    &CleanWaterFillingStopbyHighSensorTimeOut,
    &ABFertilizerAddingbyECTimeOut
};


static uint32_t TargetEC = 0;

static uint32_t mb2slave1PPT32 = 0;
static float mb2slave1PPT = 0;

static uint32_t mb2slave1Temp32 = 0;
static float mb2slave1Temp = 0;

static uint32_t mb2slave1EC32 = 0;
static float mb2slave1EC = 0;

static uint32_t mb2slave2AccumulateVolumeA = 0;
static uint32_t mb2slave3AccumulateVolumeB = 0;
static uint32_t StampVolumeA = 0;
static uint32_t StampVolumeB = 0;
static uint32_t currentVolumeA = 0;
static uint32_t currentVolumeB = 0;
static uint32_t targetVolumeA = 0;
static uint32_t targetVolumeB = 0;
static uint32_t targetVolumeIncrement = 500;

static uint32_t mb2slave2CurrentFlowRateA = 0;
static uint32_t mb2slave3CurrentFlowRateB = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static void SystemConfigInit(void);
static void TCPContextInit(void);
static void IOListInit(void);
static void StateMachineRunning(void);
static void StateUpdate(void);
static void IOListUpdate(void);
static void ResetAllIO(void);

static void WriteOutput(Output* output, GPIO_PinState pinIsSet);
static void ToggleOutput(Output* output);
static void ModbusTransmitSwitchDevice(void);

static uint16_t CRC16privateFunction (uint8_t *nData, uint8_t lenOfData);
static void UpdateTCP(void);
static void WriteTCPDirectionalPin(GPIO_PinState pinIsSet);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		//ModbusMaster_UART_TxCpltCallback(&huart);
		ModbusMaster_UART_TxCpltCallback(&mb2);
	}
	else if (huart->Instance == USART6){
		if(tcpContext.tcpState == TCP_STATE_TX_COMPLETE){
			//WriteTCPDirectionalPin(GPIO_PIN_RESET);
			tcpContext.tcpState = TCP_STATE_WAITING_RESPONSE;
			tcpContext.timeStamp = HAL_GetTick();
			//tcpContext.txBufferCount = 0;
			tcpContext.rxBufferCount = 0;
			HAL_UART_Receive_IT(&huart6, &tcpContext.rxSingleByte, 1);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART6){
		if(tcpContext.tcpState == TCP_STATE_WAITING_RESPONSE){
			tcpContext.rxBuffer[tcpContext.rxBufferCount++] = tcpContext.rxSingleByte;
			tcpContext.timeStamp = HAL_GetTick();
			HAL_UART_Receive_IT(&huart6, &tcpContext.rxSingleByte, 1);
			tcpContext.tcpState = TCP_STATE_RECEIVING;
		}
		else if (tcpContext.tcpState == TCP_STATE_RECEIVING){
			tcpContext.rxBuffer[tcpContext.rxBufferCount++] = tcpContext.rxSingleByte;
			tcpContext.timeStamp = HAL_GetTick();
			HAL_UART_Receive_IT(&huart6, &tcpContext.rxSingleByte, 1);
		}

	}
	else if (huart->Instance == USART2){
		ModbusMaster_UART_RxCpltCallback(&mb2);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemConfigInit();
  TCPContextInit();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  IOListInit();
  ModbusMaster_Init(&mb2,&huart2, GPIOC, GPIO_PIN_4);
  ModbusMaster_AddReadQueue(&mb2, 1, 0, 6, 500, 1000, 1000);
    //ModbusMaster_AddReadQueue(&mb2, 2, 0, 8, 10000, 1000, 1000);
  ModbusMaster_UpdateReadTransaction(&mb2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  StateMachineRunning();
	  ModbusTransmitSwitchDevice();
	  IOListUpdate();
	  UpdateTCP();
	  StateUpdate();
	  //HAL_UART_Transmit(&huart6, toSend,2, 3000);
	  //HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC4
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4
                           PB5 PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void SystemConfigInit(void){
	systemContext.currentState = STATE_IDLE;
	systemContext.ManualOutputAutoResetTimeOut = 5000;
	systemContext.ABDelay = 5000;
}

static void TCPContextInit(void){
	tcpContext.delayTime = 100;
	tcpContext.errorCount = 0;
	tcpContext.rxBufferCount = 0;
	tcpContext.rxTimeOut = 1000;
	tcpContext.txBufferCount = 0;
	tcpContext.txTimeOut = 1000;
	tcpContext.intercharTimeOut = 10;
}

static uint16_t CRC16privateFunction (uint8_t *nData, uint8_t lenOfData)
{
	uint16_t wCRCTable[] = { 0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

	uint16_t nTemp;
	uint16_t wCRCWord = 0xFFFF;
   while (lenOfData--)
   {
      nTemp = *nData++ ^ (wCRCWord & 0xFF);
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;
}

static void UpdateTCP(){
	switch(tcpContext.tcpState){
	case TCP_STATE_IDLE:
		if(HAL_GetTick()-tcpContext.timeStamp> tcpContext.delayTime){
			tcpContext.txBufferCount = 0;
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)systemContext.currentState;
			uint16_t outputMask = 0;
			for (int i=0; i < OutputCount;i++){
				if (outputs[i].IsSet){
					outputMask |=(uint8_t)(1<<i);
				}
			}
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)outputMask;
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(outputMask>>8);
			uint16_t inputMask = 0;
			for (int i=0; i < InputCount;i++){
				if (inputs[i].IsSet){
					inputMask |=(uint8_t)(1<<i);
				}
			}
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)inputMask;
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(inputMask>>8);
			for (int i = 0; i<sizeof(timeoutList) / sizeof(timeoutList[0]);i++){
				tcpContext.txBufferCount++;
				tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(*timeoutList[i]);
				tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)((*timeoutList[i])>>8);
				tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)((*timeoutList[i])>>16);
				tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)((*timeoutList[i])>>24);

			}
			tcpContext.txBufferCount++;
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(TargetEC);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(TargetEC>>8);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(TargetEC>>16);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(TargetEC>>24);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave1Temp32);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave1Temp32>>8);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave1Temp32>>16);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave1Temp32>>24);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave2CurrentFlowRateA);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave2CurrentFlowRateA>>8);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave2CurrentFlowRateA>>16);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave2CurrentFlowRateA>>24);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(currentVolumeA);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(currentVolumeA>>8);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(currentVolumeA>>16);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(currentVolumeA>>24);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave3CurrentFlowRateB);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave3CurrentFlowRateB>>8);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave3CurrentFlowRateB>>16);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(mb2slave3CurrentFlowRateB>>24);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(currentVolumeB);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(currentVolumeB>>8);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(currentVolumeB>>16);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(currentVolumeB>>24);
			uint16_t calculatedCRC = CRC16privateFunction(tcpContext.txBuffer,tcpContext.txBufferCount);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(calculatedCRC);
			tcpContext.txBuffer[tcpContext.txBufferCount++] = (uint8_t)(calculatedCRC>>8);
			//WriteTCPDirectionalPin(GPIO_PIN_SET);
			HAL_UART_Transmit_IT(&huart6, tcpContext.txBuffer, tcpContext.txBufferCount);
			tcpContext.tcpState = TCP_STATE_TX_COMPLETE;
			tcpContext.timeStamp = HAL_GetTick();
		}
		break;
	case TCP_STATE_TX_COMPLETE:
		if(HAL_GetTick()-tcpContext.timeStamp> tcpContext.txTimeOut){
			tcpContext.tcpState = TCP_STATE_IDLE;
			tcpContext.txBufferCount = 0;
			tcpContext.errorCount +=1;
			HAL_UART_AbortTransmit(&huart6);
			//WriteTCPDirectionalPin(GPIO_PIN_RESET);
		}
		break;
	case TCP_STATE_WAITING_RESPONSE:
		if(HAL_GetTick()-tcpContext.timeStamp> tcpContext.rxTimeOut){
			tcpContext.tcpState = TCP_STATE_IDLE;
			tcpContext.errorCount +=1;
			HAL_UART_AbortReceive(&huart6);
		}
		break;
	case TCP_STATE_RECEIVING:
		if (tcpContext.rxBufferCount > 0 && (HAL_GetTick()-tcpContext.timeStamp) > tcpContext.intercharTimeOut){
			uint16_t calculatedCRC = CRC16privateFunction(tcpContext.rxBuffer,tcpContext.rxBufferCount);
			if(calculatedCRC !=0 || tcpContext.rxBufferCount != tcpContext.txBufferCount){
				break;
			}
			HAL_UART_AbortReceive(&huart6);
			uint8_t byteCount = 0;
			uint8_t commandReceived = 0;
			commandReceived = tcpContext.rxBuffer[byteCount++];
			if(commandReceived<STATE_COUNT){

				systemContext.upcomingState = commandReceived;
			}
			uint16_t outputMask = 0;
			if(systemContext.currentState == STATE_IDLE){
				outputMask = tcpContext.rxBuffer[byteCount++];
				outputMask |= tcpContext.rxBuffer[byteCount++]<<8;
			}
			else{
				byteCount+=2;
			}
			for (int i=0; i < OutputCount;i++){
				if (outputMask&(1 <<i)){
					ToggleOutput(&outputs[i]);
				}
			}
			byteCount+=2;
			for (int i = 0; i<sizeof(timeoutList) / sizeof(timeoutList[0]);i++){
				if(systemContext.currentState == STATE_IDLE && tcpContext.rxBuffer[byteCount++] !=0){
					uint32_t timeoutValue = tcpContext.rxBuffer[byteCount++];
					timeoutValue |= (tcpContext.rxBuffer[byteCount++]<<8);
					timeoutValue |= (tcpContext.rxBuffer[byteCount++]<<16);
					timeoutValue |= (tcpContext.rxBuffer[byteCount++]<<24);
					*timeoutList[i] = timeoutValue;
				}
				else{
					byteCount+=4;
				}
			}
			if(systemContext.currentState == STATE_IDLE && tcpContext.rxBuffer[byteCount++] !=0){
				uint32_t ecValue = tcpContext.rxBuffer[byteCount++];
				ecValue |= tcpContext.rxBuffer[byteCount++]<<8;
				ecValue |= tcpContext.rxBuffer[byteCount++]<<16;
				ecValue |= tcpContext.rxBuffer[byteCount++]<<24;
				TargetEC= ecValue;
			}
			else{
				byteCount+=4;
			}
			byteCount+=20;
			tcpContext.tcpState = TCP_STATE_IDLE;
			tcpContext.errorCount = 0;
		}
	}
}

static void WriteTCPDirectionalPin(GPIO_PinState pinIsSet){
	HAL_GPIO_WritePin(TCPDirectionalPin.Port, TCPDirectionalPin.Pin, pinIsSet? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void StateMachineRunning(void){
	switch(systemContext.currentState){
	case STATE_IDLE:
		for (int i = 0;i< OutputCount;i++){
			if(outputs[i].IsSet == GPIO_PIN_SET && (HAL_GetTick()-outputs[i].UpdateTimeStamp> systemContext.ManualOutputAutoResetTimeOut)){
				outputs[i].IsSet = GPIO_PIN_RESET;
			}
		}
		break;
	case STATE_CLEAN_WATER_WATERING:
		WriteOutput(OutFertigationPump, GPIO_PIN_SET);
		WriteOutput(OutFertigationValve1, GPIO_PIN_SET);
		WriteOutput(OutFertigationValve2, GPIO_PIN_SET);
		if(HAL_GetTick()-systemContext.TimeStamp > CleanWaterWateringTimeOut){
			systemContext.currentState = STATE_ABORT;
		}
		else if(InCleanWaterLow.IsSet){
			systemContext.currentState = STATE_ERROR;
		}
		break;
	case STATE_MIXED_WATER_WATERING:
		WriteOutput(OutFertigationPump, GPIO_PIN_SET);
		WriteOutput(OutFertigationValve1, GPIO_PIN_RESET);
		WriteOutput(OutFertigationValve2, GPIO_PIN_SET);
		if(HAL_GetTick()-systemContext.TimeStamp > MixedWaterWateringTimeOut){
			systemContext.currentState = STATE_ABORT;
		}
		else if(InCleanWaterLow.IsSet){
			systemContext.currentState = STATE_ERROR;
		}
		break;

	case STATE_CLEAN_WATER_FILLING_TIMING:
		WriteOutput(OutFertigationPump, GPIO_PIN_SET);
		WriteOutput(OutFertigationValve1, GPIO_PIN_SET);
		WriteOutput(OutFertigationValve2, GPIO_PIN_RESET);
		if(HAL_GetTick()-systemContext.TimeStamp > CleanWaterFillingStopbyTimingTimeOut){
			systemContext.currentState = STATE_ABORT;
		}
		else if(InCleanWaterLow.IsSet){
			systemContext.currentState = STATE_ERROR;
		}
		else if(InMixedWaterLimit.IsSet){
			systemContext.currentState = STATE_ERROR;
		}
		break;
	case STATE_CLEAN_WATER_FILLING_EC:
		WriteOutput(OutFertigationPump, GPIO_PIN_SET);
		WriteOutput(OutFertigationValve1, GPIO_PIN_SET);
		WriteOutput(OutFertigationValve2, GPIO_PIN_RESET);
		if(HAL_GetTick()-systemContext.TimeStamp > CleanWaterFillingStopbyHighSensorTimeOut){
			systemContext.currentState = STATE_ABORT;
		}
		else if(InCleanWaterLow.IsSet){
			systemContext.currentState = STATE_ERROR;
		}
		else if(InMixedWaterHigh.IsSet){
			systemContext.currentState = STATE_ABORT;
		}
		else if(InMixedWaterLimit.IsSet){
			systemContext.currentState = STATE_ERROR;
		}
		break;
	case STATE_A_B_FERTILIZER_ADDING:
		if (OutPumpA.IsSet && OutPumpB.IsSet) {
			if (mb2slave2AccumulateVolumeA-StampVolumeA>= targetVolumeA) {
				WriteOutput(OutPumpA, GPIO_PIN_RESET);  // Turn OFF A
				WriteOutput(OutValveA, GPIO_PIN_RESET);
			}
			if (mb2slave3AccumulateVolumeB-StampVolumeB>= targetVolumeB) {
				WriteOutput(OutPumpB, GPIO_PIN_RESET);
				WriteOutput(OutValveB, GPIO_PIN_RESET);
			}
		}
		else if ((OutPumpA.IsSet || OutPumpB.IsSet)&& (OutPumpA.IsSet != OutPumpB.IsSet)) {
			if ((mb2slave2AccumulateVolumeA-StampVolumeA>= targetVolumeA) && (mb2slave3AccumulateVolumeB-StampVolumeB>= targetVolumeB)) {
				WriteOutput(OutPumpA, GPIO_PIN_RESET);  // Turn OFF A
				WriteOutput(OutValveA, GPIO_PIN_RESET);
				WriteOutput(OutPumpB, GPIO_PIN_RESET);
				WriteOutput(OutValveB, GPIO_PIN_RESET);
				systemContext.ABTimeStamp = HAL_GetTick();
			}
		}
		else {
			if(HAL_GetTick()-systemContext.ABTimeStamp>systemContext.ABDelay){
				targetVolumeA+= targetVolumeIncrement;
				targetVolumeB+= targetVolumeIncrement;
				WriteOutput(OutPumpA, GPIO_PIN_SET);  // Turn OFF A
				WriteOutput(OutValveA, GPIO_PIN_SET);
				WriteOutput(OutPumpB, GPIO_PIN_SET);
				WriteOutput(OutValveB, GPIO_PIN_SET);
			}
		}
		if(HAL_GetTick()-systemContext.TimeStamp > ABFertilizerAddingbyECTimeOut){
			systemContext.currentState = STATE_ERROR;
		}
		if(InMixedWaterLimit.IsSet){
			systemContext.currentState = STATE_ERROR;
		}
		if(mb2slave2AccumulateVolumeA == 0xFFFFFFFF){
			systemContext.currentState = STATE_ERROR;
		}
		if (mb2slave1EC == -1){
			systemContext.currentState = STATE_ERROR;
		}
		if(mb2slave1EC > TargetEC){
			systemContext.currentState = STATE_ABORT;
		}
		currentVolumeA = mb2slave2AccumulateVolumeA-StampVolumeA;
		currentVolumeB = mb2slave3AccumulateVolumeB-StampVolumeB;
		break;
	case STATE_ABORT:
		ResetAllIO();
		systemContext.currentState = STATE_IDLE;
		break;
	case STATE_ERROR:
		ResetAllIO();
		break;
	}
}

static void IOListInit(void){
	for (int i = 0; i< OutputCount; i++){
		(&outputs[i])->IsSet = GPIO_PIN_RESET;
	}

	for (int i = 0;i<InputCount;i++){
		(&inputs[i])->IsSet = GPIO_PIN_RESET;
	}

	//Write specifically here if th output initial status is not reset.
}

static void ResetAllIO(void){
	for (int i = 0; i< OutputCount; i++){
		(&outputs[i])->IsSet = GPIO_PIN_RESET;
	}

	for (int i = 0;i<InputCount;i++){
		(&inputs[i])->IsSet = GPIO_PIN_RESET;
	}
}

static void ModbusTransmitSwitchDevice(void){
	switch(modbusSwitchCase){
		case 0:
			if (mb2.modbusWriteQueue.count != 0){
				ModbusMaster_UpdateWriteTransaction(&mb2);
				ModbusMaster_WriteMultipleRegisters(&mb2);
				switch(mb2.modbusTransaction.slaveAddress){
					case 1:
						modbusSwitchCase = 20;
						break;
					case 2:
						modbusSwitchCase = 12;
						break;
				}
			}
			else{
				uint32_t currentTime = HAL_GetTick();
				if (currentTime-mb2.lastOperationTimeStamp>=mb2.modbusTransaction.delayTime){
					ModbusMaster_ReadHoldingRegister(&mb2);
					switch(mb2.modbusReadQueue.head){
						case 0:
							modbusSwitchCase = 10;
							break;
						case 1:
							modbusSwitchCase = 11;
							break;
						case 2:
							modbusSwitchCase = 12;
							break;
					}
				}
			}
			break;
		case 10:
			if (mb2.modbusState == MB_STATE_TIMEOUT || mb2.modbusState == MB_STATE_ERROR){
				if (mb2.modbusReadQueue.requestQueue[mb2.modbusReadQueue.head].errorCount>20){
					mb2slave1EC = -1;
					memcpy(&mb2slave1EC32, &mb2slave1EC, sizeof(uint32_t));
					mb2slave1Temp = -1;
					memcpy(&mb2slave1Temp32, &mb2slave1Temp, sizeof(uint32_t));
					mb2slave1PPT = -1;
					memcpy(&mb2slave1PPT32, &mb2slave1PPT, sizeof(uint32_t));
				}
				mb2.modbusReadQueue.head = (mb2.modbusReadQueue.head +1)%(mb2.modbusReadQueue.count);
				modbusSwitchCase = 100;
			}
			if(mb2.modbusState == MB_STATE_RX_COMPLETE){
				mb2slave1EC32 = mb2.modbusTransaction.data[0]<<16|mb2.modbusTransaction.data[1];
				memcpy(&mb2slave1EC, &mb2slave1EC32, sizeof(uint32_t));
				mb2slave1Temp32 = mb2.modbusTransaction.data[2]<<16|mb2.modbusTransaction.data[3];
				memcpy(&mb2slave1Temp, &mb2slave1Temp32, sizeof(uint32_t));
				mb2slave1PPT32 = mb2.modbusTransaction.data[4]<<16|mb2.modbusTransaction.data[5];
				memcpy(&mb2slave1PPT, &mb2slave1PPT32, sizeof(uint32_t));
				mb2.modbusReadQueue.head = (mb2.modbusReadQueue.head +1)%(mb2.modbusReadQueue.count);
				modbusSwitchCase = 100;
			}
			break;
		case 11:
			break;
		case 12:
		  break;
		case 20:
			if (mb2.modbusState == MB_STATE_TIMEOUT || mb2.modbusState == MB_STATE_ERROR){
				ModbusMaster_DeleteWriteQueue(&mb2);
				modbusSwitchCase = 100;
			}
			if(mb2.modbusState == MB_STATE_RX_COMPLETE){
				ModbusMaster_DeleteWriteQueue(&mb2);
				modbusSwitchCase = 100;
			}
			break;
		case 21:
		  break;
		case 22:
		  break;
		case 100:
			ModbusMaster_UpdateReadTransaction(&mb2);
			modbusSwitchCase = 0;
			break;
		}
		ModbusMaster_MonitorTransceive(&mb2);
}

static void WriteOutput(Output* output, GPIO_PinState pinIsSet){
	output->IsSet = pinIsSet;
	output->UpdateTimeStamp = HAL_GetTick();
}

static void ToggleOutput(Output* output){
	output->IsSet = !output->IsSet;
	output->UpdateTimeStamp = HAL_GetTick();
}

static void IOListUpdate(void){
	for (int i = 0; i<OutputCount; i++){
		HAL_GPIO_WritePin(outputs[i].Port, outputs[i].Pin,!outputs[i].IsSet);
	}

	for (int i= 0; i<InputCount;i++){
		inputs[i].IsSet = !HAL_GPIO_ReadPin(inputs[i].Port, inputs[i].Pin);
	}
}

static void StateUpdate(void) {
    if (systemContext.upcomingState == STATE_IDLE) return;

    switch(systemContext.currentState) {
        case STATE_IDLE:
            if (systemContext.upcomingState != STATE_ABORT && systemContext.upcomingState != STATE_ERROR && systemContext.upcomingState != STATE_IDLE) {
                if(systemContext.upcomingState == STATE_A_B_FERTILIZER_ADDING){
                	StampVolumeA = mb2slave2AccumulateVolumeA;
                	StampVolumeB = mb2slave3AccumulateVolumeB;
                	currentVolumeA = 0;
                	currentVolumeB = 0;
                	targetVolumeA = mb2slave2AccumulateVolumeA;
                	targetVolumeB = mb2slave3AccumulateVolumeB;
                	systemContext.ABTimeStamp = HAL_GetTick();
                }
            	systemContext.currentState = systemContext.upcomingState;

                ResetAllIO();
                systemContext.TimeStamp = HAL_GetTick();
            }
            break;
        default:  // Active states
            if (systemContext.upcomingState == STATE_ABORT) {
                systemContext.currentState = STATE_ABORT;
            }
            // Other commands ignored while active
            break;
    }
    systemContext.upcomingState = STATE_IDLE;
}

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
#ifdef USE_FULL_ASSERT
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
