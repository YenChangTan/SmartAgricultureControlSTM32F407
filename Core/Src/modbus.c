/*
 * modbus.c
 *
 *  Created on: Nov 19, 2025
 *      Author: Tan Yen Chang
 */

#include "modbus.h"
#include <string.h>

static uint32_t SafeTimeDiff(uint32_t newer, uint32_t older);
static void _enable_tx_mode(ModbusMaster *mb, uint8_t enable);
static void _start_transmission(ModbusMaster *mb);
static void _abort_transmission(ModbusMaster *mb);
static void _abort_reception(ModbusMaster *mb);
static void _validate_and_process_received_frame(ModbusMaster *mb);
static void _process_response(ModbusMaster *mb);
static uint8_t _is_queue_full(ModbusMaster_Queue *mbQueue);
static uint8_t _add_write_queue(ModbusMaster_Queue *mbQueue, ModbusMaster_Transaction trans);
static uint8_t _add_read_queue(ModbusMaster_Queue *mbQueue, ModbusMaster_Transaction trans);


uint16_t CRC16 (uint8_t *nData, uint8_t lenOfData, uint8_t startingAddress)
{
uint16_t wCRCTable[] = { 0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

uint16_t nTemp;
uint16_t wCRCWord = 0xFFFF;
lenOfData -= startingAddress;
nData+= startingAddress;
   while (lenOfData--)
   {
      nTemp = *nData++ ^ (wCRCWord & 0xFF);
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;
}

void ModbusMaster_Init(ModbusMaster *mb, UART_HandleTypeDef *huart, GPIO_TypeDef * directionPort, uint16_t directionPinNum){
	memset(mb,0,sizeof(ModbusMaster));
	mb->huart = huart;
	mb->directionPort = directionPort;
	mb->directionPinNum = directionPinNum;
	mb->modbusState = MB_STATE_IDLE;
	mb->intercharTimeOut = 10;

}

uint8_t ModbusMaster_ReadHoldingRegister(ModbusMaster *mb){
	if (mb-> modbusState != MB_STATE_IDLE) return MB_ERROR_BUS_BUSY;


	uint8_t *tx = mb->txBuffer;
	uint8_t* txBufferCount = &mb->txBufferCount;
	*txBufferCount = 0;

	tx[(*txBufferCount)++] = mb -> modbusTransaction.slaveAddress;
	tx[(*txBufferCount)++] = mb -> modbusTransaction.functionCode;
	tx[(*txBufferCount)++] = mb -> modbusTransaction.startRegister >>8;
	tx[(*txBufferCount)++] = mb -> modbusTransaction.startRegister & 0xFF;
	tx[(*txBufferCount)++] = mb -> modbusTransaction.dataLength >>8;
	tx[(*txBufferCount)++] = mb -> modbusTransaction.dataLength & 0xFF;
	uint16_t calCRC = CRC16(tx, *txBufferCount, 0);
	tx[(*txBufferCount)++] = calCRC & 0xFF;
	tx[(*txBufferCount)++] = calCRC >> 8;
	mb -> rxExpectedBufferCount = 5 + mb -> modbusTransaction.dataLength*2;
	_start_transmission(mb);
	return MB_SUCCESS;
}

uint8_t ModbusMaster_WriteMultipleRegisters(ModbusMaster *mb){
	if (mb-> modbusState != MB_STATE_IDLE) return MB_ERROR_BUS_BUSY;

	uint8_t *tx = mb->txBuffer;
	uint8_t* txBufferCount = &mb->txBufferCount;
	*txBufferCount = 0;
	tx[(*txBufferCount)++] = mb -> modbusTransaction.slaveAddress;
	tx[(*txBufferCount)++] = MB_FC_WRITE_MULTIPLE_REGISTERS;
	tx[(*txBufferCount)++] = (mb->modbusTransaction.startRegister)>>8;
	tx[(*txBufferCount)++] = (mb->modbusTransaction.startRegister)& 0xFF;
	tx[(*txBufferCount)++] = (mb->modbusTransaction.dataLength) >>8;
	tx[(*txBufferCount)++] = (mb->modbusTransaction.dataLength) & 0xFF;
	tx[(*txBufferCount)++] = (mb->modbusTransaction.dataLength)*2;
	for (int i =0; i< (mb->modbusTransaction.dataLength);i++){
		tx[(*txBufferCount)++] = mb->modbusTransaction.data[i]>>8;
		tx[(*txBufferCount)++] = mb->modbusTransaction.data[i] & 0xFF;
	}
	uint16_t calCRC = CRC16(tx, *txBufferCount, 0);
	tx[(*txBufferCount)++] = calCRC & 0xFF;
	tx[(*txBufferCount)++] = calCRC >> 8;
	mb -> rxExpectedBufferCount = mb->txBufferCount;
	_start_transmission(mb);
	return MB_SUCCESS;
	//mb -> rxTimeOut = rxTimeOut;
}

uint8_t ModbusMaster_AddReadQueue(ModbusMaster *mb, uint8_t slaveAddress, uint16_t startRegister, uint16_t registerCount, uint32_t delayTime, uint32_t txTimeOut, uint32_t rxTimeOut){
	ModbusMaster_Transaction modbusTransaction = {0};
	modbusTransaction.dataLength = registerCount;
	modbusTransaction.slaveAddress = slaveAddress;
	modbusTransaction.functionCode = MB_FC_READ_HOLDING_REGISTERS;
	modbusTransaction.startRegister = startRegister;
	modbusTransaction.isEnable = 1;
	modbusTransaction.delayTime = delayTime;
	modbusTransaction.txTimeOut = txTimeOut;
	modbusTransaction.rxTimeOut = rxTimeOut;
	return _add_read_queue(&mb->modbusReadQueue, modbusTransaction);
}

uint8_t ModbusMaster_AddWriteQueue(ModbusMaster *mb, uint8_t slaveAddress, uint16_t startRegister, uint16_t registerCount, uint16_t* valueToWrite, uint32_t txTimeOut, uint32_t rxTimeOut){
	if(registerCount>123) return MB_ERROR_INVALID_PARAM;

	ModbusMaster_Transaction modbusTransaction = {0};
	modbusTransaction.slaveAddress = slaveAddress;
	modbusTransaction.functionCode = MB_FC_WRITE_MULTIPLE_REGISTERS;
	modbusTransaction.startRegister = startRegister;
	modbusTransaction.dataLength = registerCount;
	modbusTransaction.txTimeOut = txTimeOut;
	modbusTransaction.rxTimeOut = rxTimeOut;
	for (int i = 0;i < registerCount;i++){
		modbusTransaction.data[i] = valueToWrite[i];
	}
	return _add_write_queue(&mb->modbusWriteQueue, modbusTransaction);
}

uint8_t ModbusMaster_DeleteWriteQueue(ModbusMaster *mb){
	if(mb->modbusWriteQueue.count == 0){
		return 0;
	}
	mb->modbusWriteQueue.head = (mb->modbusWriteQueue.head +1)% maxModbusQueue;
	mb->modbusWriteQueue.count--;
	return 1;
}

void ModbusMaster_MonitorTransceive(ModbusMaster *mb){
	switch(mb->modbusState){
	case MB_STATE_IDLE:
		break;
	case MB_STATE_TX_COMPLETE:
		if(SafeTimeDiff(HAL_GetTick(), mb->txTimeStamp)> mb->modbusTransaction.txTimeOut){
			mb-> modbusState = MB_STATE_TIMEOUT;
			_abort_transmission(mb);
		}
		break;
	case MB_STATE_WAITING_RESPONSE:
		if(SafeTimeDiff(HAL_GetTick(), mb->rxTimeStamp)> mb->modbusTransaction.rxTimeOut){

			mb->modbusState = MB_STATE_TIMEOUT;
			_abort_reception(mb);

		}
		break;
	case MB_STATE_RECEIVING:
		if (mb->rxBufferCount >0 && (SafeTimeDiff(HAL_GetTick(), mb->rxTimeStamp) > mb->intercharTimeOut)){
			HAL_UART_AbortReceive(mb->huart);
			_validate_and_process_received_frame(mb);
		}
		break;
	case MB_STATE_RX_COMPLETE:
		//mb->errorCount = 0;
		mb->modbusReadQueue.requestQueue[mb->modbusReadQueue.head].errorCount =0;
		break;
	case MB_STATE_TIMEOUT:
		mb->modbusReadQueue.requestQueue[mb->modbusReadQueue.head].errorCount +=1;
		mb->modbusState = MB_STATE_ERROR;
		break;
	case MB_STATE_ERROR:
		break;
	}
}

void ModbusMaster_UpdateReadTransaction(ModbusMaster *mb){
	memcpy(&mb->modbusTransaction, &mb->modbusReadQueue.requestQueue[mb->modbusReadQueue.head], sizeof(ModbusMaster_Transaction));
	mb->lastOperationTimeStamp = HAL_GetTick();
	mb->modbusState = MB_STATE_IDLE;
}

void ModbusMaster_UpdateWriteTransaction(ModbusMaster *mb){
	memcpy(&mb->modbusTransaction, &mb->modbusWriteQueue.requestQueue[mb->modbusWriteQueue.head], sizeof(ModbusMaster_Transaction));
	mb->modbusState = MB_STATE_IDLE;
}

void ModbusMaster_UART_TxCpltCallback(ModbusMaster *mb){
	if(mb->modbusState == MB_STATE_TX_COMPLETE){
		_enable_tx_mode(mb,0);
		mb->modbusState = MB_STATE_WAITING_RESPONSE;
		mb -> rxTimeStamp = HAL_GetTick();
		mb -> rxBufferCount = 0;
		HAL_UART_Receive_IT(mb->huart, &(mb->rxSingleByte), 1);
	}
}

void ModbusMaster_UART_RxCpltCallback(ModbusMaster *mb){
	mb->rxBuffer[mb->rxBufferCount++] = mb->rxSingleByte;
	mb->rxTimeStamp = HAL_GetTick();
	if (mb->modbusState == MB_STATE_WAITING_RESPONSE){
		mb->modbusState = MB_STATE_RECEIVING;
	}
	HAL_UART_Receive_IT(mb->huart, &(mb->rxSingleByte),1);
}


uint8_t generateRequestHoldingRegister(uint8_t slaveAddress,uint16_t startAddress, uint16_t quantity,uint8_t *arrData){
	uint8_t lenOfArrData = 0;
	uint16_t calCRC = 0;
	//arrData[lenOfArrData++] = loraAddress>>8;
	//arrData[lenOfArrData++] = loraAddress;
	arrData[lenOfArrData++] = slaveAddress;
	arrData[lenOfArrData++] = 3;//Function Code 03 is read holding register
	arrData[lenOfArrData++] = startAddress>>8;//Address High
	arrData[lenOfArrData++] = startAddress;//Address Low
	arrData[lenOfArrData++] = quantity>>8;
	arrData[lenOfArrData++] = quantity;
	calCRC = CRC16(arrData,lenOfArrData,0);
	arrData[lenOfArrData++] = calCRC;
	arrData[lenOfArrData++] = calCRC>>8;
	return lenOfArrData;
}

uint8_t generateWritingMultipleHoldingRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t writeValue, uint8_t *arrData){
	uint8_t lenOfArrData = 0;
	uint16_t calCRC = 0;
	arrData[lenOfArrData++] = slaveAddress;
	arrData[lenOfArrData++] = 6;
	arrData[lenOfArrData++] = startAddress>>8;
	arrData[lenOfArrData++] = startAddress;
	calCRC = CRC16(arrData, lenOfArrData,0);
	arrData[lenOfArrData++] = calCRC;
	arrData[lenOfArrData++] = calCRC>>8;
	return lenOfArrData;
}

static void _start_transmission(ModbusMaster *mb){
	_enable_tx_mode(mb, 1);
	HAL_UART_Transmit_IT(mb->huart, mb->txBuffer,mb->txBufferCount);
	mb->txTimeStamp = HAL_GetTick();
	mb-> modbusState = MB_STATE_TX_COMPLETE;
}

static void _abort_transmission(ModbusMaster *mb){
	HAL_UART_AbortTransmit(mb->huart);
	_enable_tx_mode(mb,0);
}

static void _abort_reception(ModbusMaster *mb){
	HAL_UART_AbortReceive(mb->huart);
	mb->rxBufferCount = 0;
}

static void _validate_and_process_received_frame(ModbusMaster *mb){
	if ((mb ->rxBufferCount == mb -> rxExpectedBufferCount) && (CRC16(mb->rxBuffer,mb->rxBufferCount,0) == 0) && (mb->rxBuffer[0] == mb->modbusTransaction.slaveAddress) && (mb-> rxBuffer[1] == mb->modbusTransaction.functionCode)&& !(mb->rxBuffer[1]&0x80)){
		for(int i = 0; i<mb->modbusTransaction.dataLength;i++){
			mb->modbusTransaction.data[i] = mb->rxBuffer[3+i*2]<<8|mb->rxBuffer[3+i*2+1];
		}
		mb->modbusState = MB_STATE_RX_COMPLETE;
	}
	else{
		mb->modbusState = MB_STATE_ERROR;
	}
}


static uint8_t _is_queue_full(ModbusMaster_Queue *mbQueue){
	return mbQueue->count >= maxModbusQueue;
}

static uint8_t _add_write_queue(ModbusMaster_Queue *mbQueue, ModbusMaster_Transaction trans){
	if (_is_queue_full(mbQueue)){
		return 0;
	}
	memcpy(&mbQueue->requestQueue[mbQueue->tail], &trans, sizeof(ModbusMaster_Transaction));
	mbQueue-> tail = (mbQueue->tail+1)% maxModbusQueue;
	mbQueue-> count++;
	return 1;
}
static uint8_t _add_read_queue(ModbusMaster_Queue *mbQueue, ModbusMaster_Transaction trans){
	if (_is_queue_full(mbQueue)){
		return 0;
	}

	memcpy(&mbQueue->requestQueue[mbQueue->tail], &trans, sizeof(ModbusMaster_Transaction));
	mbQueue-> tail = (mbQueue->tail+1)% maxModbusQueue;
	mbQueue-> count++;
	return 1;
}

static void _process_response(ModbusMaster *mb){
	return;
}

static void _enable_tx_mode(ModbusMaster *mb, uint8_t enable){
	if(mb->directionPort){
		HAL_GPIO_WritePin(mb->directionPort, mb-> directionPinNum, enable? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

uint8_t ModbusMaster_IsBusy(ModbusMaster *mb){
	return (mb->modbusState != MB_STATE_IDLE);
}

static uint32_t SafeTimeDiff(uint32_t newer, uint32_t older){
	return(newer>= older) ? (newer-older) : (0xFFFFFFFF-older+newer +1);
}

