/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_gpio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum{
	STATE_IDLE = 0,
	STATE_CLEAN_WATER_WATERING,
	STATE_MIXED_WATER_WATERING,
	STATE_CLEAN_WATER_FILLING_TIMING,
	STATE_CLEAN_WATER_FILLING_EC,
	STATE_A_B_FERTILIZER_ADDING,
	STATE_ABORT,
	STATE_ERROR,
	STATE_COUNT
} SystemState;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

typedef struct{
	GPIO_TypeDef* Port;
	uint16_t Pin;
	GPIO_PinState IsSet;
	uint32_t UpdateTimeStamp;
	uint32_t TimeOut;
	uint32_t TimeElapsed;
}Output;

typedef enum{
	ValveA = 0,
	ValveB,
	FertigationPump,
	FertigationValve2,
	PumpAB,
	FertigationValve1,
	OutputCount
}OutputEnum;

#define OutValveA (&outputs[ValveA])
#define OutValveB (&outputs[ValveB])
#define OutFertigationPump (&outputs[FertigationPump])
#define OutPumpAB (&outputs[PumpAB])
#define OutFertigationValve1 (&outputs[FertigationValve1])
#define OutFertigationValve2 (&outputs[FertigationValve2])

typedef struct{
	GPIO_TypeDef* Port;
	uint16_t Pin;
	GPIO_PinState IsSet;
}Input;

typedef enum{
	MixedWaterLow = 0,
	MixedWaterHigh,
	MixedWaterLimit,
	CleanWaterLow,
	ABLow,
	InputCount
}InputEnum;

#define InMixedWaterLow (&inputs[MixedWaterLow])
#define InMixedWaterHigh (&inputs[MixedWaterHigh])
#define InMixedWaterLimit (&inputs[MixedWaterLimit])
#define InCleanWaterLow (&inputs[CleanWaterLow])
#define InABLow (&inputs[ABLow])

typedef struct{
	uint32_t ManualOutputAutoResetTimeOut;
	uint32_t currentTick;
	uint32_t TimeStamp;
	uint32_t TimeElapsed;
	uint32_t ABDelay;
	uint32_t ABTimeStamp;
	SystemState currentState;
	SystemState upcomingState;
	uint32_t currentEC;
	uint32_t targetEC;
}SystemContext;

typedef enum{
	TCP_STATE_IDLE = 0,
	TCP_STATE_TX_COMPLETE,
	TCP_STATE_WAITING_RESPONSE,
	TCP_STATE_RECEIVING,
	TCP_STATE_RX_COMPLETE,
	TCP_STATE_ERROR
}TCPState;

typedef struct{
	uint8_t txBuffer[128];
	uint8_t txBufferCount;
	uint32_t txTimeOut;
	uint8_t rxSingleByte;
	uint8_t rxBuffer[128];
	uint8_t rxBufferCount;
	uint32_t rxTimeStamp;
	uint32_t rxTimeOut;
	uint32_t intercharTimeOut;
	uint16_t errorCount;
	uint32_t timeStamp;
	uint32_t delayTime;
	TCPState tcpState;
	uint8_t maxErrorCount;
}TCPContext;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
