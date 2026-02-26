/*
 * modbus.h
 *
 *  Created on: Nov 19, 2025
 *      Author: Tan Yen Chang
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_
#include "stm32f4xx_hal.h"\

#define maxModbusQueue 10;
#define defaultModbusTxTimeOut 80;
#define defaultModbusRxTimeOut 4000;

typedef enum{
	MB_FC_READ_COILS = 0x01,
	MB_FC_READ_DISCRETE_INPUTS = 0x02,
	MB_FC_READ_HOLDING_REGISTERS = 0x03,
	MB_FC_READ_INPUT_REGISTERS = 0x04,
	MB_FC_WRITE_SINGLE_COIL = 0x05,
	MB_FC_WRITE_SINGLE_REGISTER = 0x06,
	MB_FC_WRITE_MULTIPLE_COILS = 0x0F,
	MB_FC_WRITE_MULTIPLE_REGISTERS = 0x10
}ModbusFunctionCode;

typedef enum{
	MB_SUCCESS = 0,
	MB_ERROR_TIMEOUT,
	MB_ERROR_CRC,
	MB_ERROR_SLAVE_NO_RESPONSE,
	MB_ERROR_SLAVE_EXCEPTION,
	MB_ERROR_INVALID_PARAM,
	MB_ERROR_BUS_BUSY
}ModbusError;

typedef enum{
	MB_STATE_IDLE = 0,
	MB_STATE_TX_COMPLETE,
	MB_STATE_WAITING_RESPONSE,
	MB_STATE_RECEIVING,
	MB_STATE_RX_COMPLETE,
	MB_STATE_TIMEOUT,
	MB_STATE_ERROR
}ModbusState;

typedef struct{
	uint8_t slaveAddress;
	uint32_t delayTime;
	uint8_t isEnable;
	uint8_t functionCode;
	uint16_t startRegister;
	uint8_t exceptionCode;
	uint16_t data[256];
	uint16_t dataLength;
	uint32_t txTimeOut;
	uint32_t rxTimeOut;
	uint32_t errorCount;
}ModbusMaster_Transaction;


typedef struct {
    ModbusMaster_Transaction requestQueue[10];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} ModbusMaster_Queue;

typedef struct{
	UART_HandleTypeDef *huart;
	GPIO_TypeDef *directionPort;
	uint16_t directionPinNum;
	uint8_t txBuffer[256];
	uint8_t txBufferCount;
	uint32_t txTimeStamp;

	uint8_t rxSingleByte;
	uint8_t rxBuffer[256];
	uint8_t rxBufferCount;
	uint8_t rxExpectedBufferCount;
	uint32_t rxTimeStamp;

	ModbusMaster_Transaction modbusTransaction;
	ModbusMaster_Queue modbusReadQueue;
	ModbusMaster_Queue modbusWriteQueue;
	uint32_t intercharTimeOut;
	uint8_t retryCount;

	ModbusState modbusState;

	uint8_t errorCount;
	uint32_t lastOperationTimeStamp;
}ModbusMaster;

void ModbusMaster_Init(ModbusMaster *mb, UART_HandleTypeDef *huart, GPIO_TypeDef * directionPort, uint16_t directionPin);

void ModbusMaster_SetTimeOut(ModbusMaster *mb, uint32_t timeOut_ms);

void ModbusMaster_SetRetryCount(ModbusMaster *mb, uint8_t retries);
uint8_t ModbusMaster_AddReadQueue(ModbusMaster *mb, uint8_t slaveAddress, uint16_t startRegister, uint16_t registerCount, uint32_t delayTime,uint32_t txTimeOut, uint32_t rxTimeOut);
uint8_t ModbusMaster_AddWriteQueue(ModbusMaster *mb, uint8_t slaveAddress, uint16_t startRegister, uint16_t registerCount, uint16_t* valueToWrite, uint32_t txTimeOut, uint32_t rxTimeOut);
uint8_t ModbusMaster_DeleteWriteQueue(ModbusMaster *mb);
uint8_t ModbusMaster_ReadHoldingRegister(ModbusMaster *mb);

void ModbusMaster_MonitorTransceive(ModbusMaster *mb);

void ModbusMaster_UpdateReadTransaction(ModbusMaster *mb);

void ModbusMaster_UpdateWriteTransaction(ModbusMaster *mb);

uint8_t ModbusMaster_WriteMultipleRegisters(ModbusMaster *mb);

void ModbusMaster_UART_TxCpltCallback(ModbusMaster *mb);

void ModbusMaster_UART_RxCpltCallback(ModbusMaster *mb);

void ModbusMaster_TimeoutCallback(ModbusMaster *mb);

uint8_t ModbusMaster_IsBusy(ModbusMaster *mb);



uint8_t generateRequestHoldingRegister(uint8_t slaveAddress,uint16_t startAddress, uint16_t quantity,uint8_t *arrData);
void decodeRequestedHoldingRegister(uint8_t slaveAddress, uint16_t quantity,uint8_t *arrRawData, uint8_t lenOfArrRawData, uint16_t *arrProcessedData, uint8_t lenOfArrProcessedData);
uint8_t generateWritingMultipleHoldingRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t writeValue, uint8_t *arrData);
uint16_t CRC16 (uint8_t *nData, uint8_t lenOfData, uint8_t startingAddress);
#endif /* INC_MODBUS_H_ */
