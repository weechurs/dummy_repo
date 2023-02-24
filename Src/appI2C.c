#include "cmsis_os2.h"
#include "main.h"
#include "i2c.h"
#include "appI2C.h"
#include <string.h>
 
static uint8_t I2C1TxBuf[I2C1_TX_BUF_SIZE];

extern osMessageQueueId_t I2C1TXEventQueueHandle;
extern osMessageQueueId_t I2C1RXEventQueueHandle;
extern osMutexId_t I2C1MutexHandle;

#define I2C1RXEventQueue I2C1RXEventQueueHandle
#define I2C1TXEventQueue I2C1TXEventQueueHandle
#define I2C1Mutex        I2C1MutexHandle

void I2C1Init(void)
{
}


uint8_t I2C1RxDataWaitTimed(uint32_t timeout) 
{ 
  osStatus_t status;
  uint8_t msg;

  status = osMessageQueueGet(I2C1RXEventQueue, &msg, NULL, timeout);
  return (status == osOK);
}

uint8_t I2C1TxDataWaitTimed(uint32_t timeout) 
{
  osStatus_t status;
  uint8_t msg;

  status = osMessageQueueGet(I2C1TXEventQueue, &msg, NULL, timeout);
  return (status == osOK);
}

void I2C1TxDataWait(void)            
{
	uint8_t msg;

	osMessageQueueGet(I2C1TXEventQueue, &msg, NULL, osWaitForever);
}  

void I2C1RxDataWait(void)            
{         
  uint8_t msg;

  osMessageQueueGet(I2C1RXEventQueue, &msg, NULL, osWaitForever);
}

void I2C1CompleteCallback(uint8_t RX)
{
  uint8_t msg=0;

  if(RX == 1)
    osMessageQueuePut(I2C1RXEventQueue, &msg, 0, 0);
  else
    osMessageQueuePut(I2C1TXEventQueue, &msg, 0, 0);
}

void I2C1Tx(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t size)
{
  osMutexAcquire(I2C1Mutex, osWaitForever);
  I2C1TxBuf[0]=reg;
  memcpy(&I2C1TxBuf[1],data,size);
  HAL_I2C_Master_Transmit_IT(&hi2c1, addr, I2C1TxBuf, size+1);     
  I2C1TxDataWait();  
  osMutexRelease(I2C1Mutex);
}

void I2C1Rx(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t size)
{
  osMutexAcquire(I2C1Mutex, osWaitForever);
  HAL_I2C_Master_Transmit_IT(&hi2c1, addr, &reg, 1);
  I2C1TxDataWait();
  HAL_I2C_Master_Receive_IT(&hi2c1, addr,  data, size);
  I2C1RxDataWait();
  osMutexRelease(I2C1Mutex);
}  

