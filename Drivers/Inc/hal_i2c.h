/*
 * hal_i2c.h
 *
 *  Created on: 2 de abr de 2022
 *      Author: matheus
 */

#ifndef HAL_I2C_H_
#define HAL_I2C_H_

#include <stdint.h>
#include "stm32f103xx.h"


/* Expressions */
#define IS_I2C_REG_VALID(I2Cx) 	((I2Cx == I2C1) || \
								 (I2Cx == I2C2) ? 1U : 0U)


typedef enum
{
	I2C_CLKSTRETCH_EN = 0,
	I2C_CLKSTRETCH_DIS
} I2C_ClkStretch_enum;

typedef enum
{
	I2C_STANDARD_MODE = 0,
	I2C_FAST_MODE
} I2C_MasterMode_enum;

typedef enum
{
	I2C_DUTY_2 = 0,
	I2C_DUTY_16_9
} I2C_DutyFM_enum;

typedef enum
{
	I2C_ADDR_7BITS = 0,
	I2C_ADDR_10BITS
} I2C_AddrMode_enum;

typedef enum
{
	I2C_STATE_FREE = 0,
	I2C_STATE_TXBUSY,
	I2C_STATE_RXBUSY
} I2C_PerifState_enum;

typedef enum
{
	I2C_EV_RX_COMPLETE = 1U,
	I2C_EV_TX_COMPLETE,
	I2C_EV_DATA_REQ,
	I2C_EV_DATA_RCV	,
	I2C_EV_STOPF,
	I2C_ERR_BERR,
	I2C_ERR_ARLO,
	I2C_ERR_AF,
	I2C_ERR_OVR,
	I2C_ERR_PECERR,
	I2C_ERR_TIMEOUT
} I2C_ITEvents_enum;

typedef struct
{
	uint32_t MasterClock;
	uint8_t DutyFastMode;
	uint8_t ClkStretch;
	uint16_t SlaveAddr;
	uint8_t AddrMode;
} I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t Config;
	uint8_t TxRxState;
	uint8_t *TxBuffer;
	uint16_t TxCounter;
	uint8_t *RxBuffer;
	uint16_t RxDataLen;
	uint16_t RxCounter;
	uint8_t DevAddr;
} I2C_Handle_t;


void I2C_Init(I2C_Handle_t *const pI2CHandle);
void I2C_Reset(I2C_RegDef_t *const pI2Cx);

void I2C_EnableClock(I2C_RegDef_t *const pI2Cx);
void I2C_DisableClock(I2C_RegDef_t *const pI2Cx);

void I2C_StartCommunication(I2C_RegDef_t *const pI2Cx);
void I2C_StopCommunication(I2C_RegDef_t *const pI2Cx);

void I2C_MasterWrite(I2C_Handle_t *const pI2CHandle, uint8_t SlaveAddr, uint8_t *pData, uint16_t Size);
void I2C_MasterRead(I2C_Handle_t *const pI2CHandle, uint8_t SlaveAddr, uint8_t *pData, uint16_t Size);
uint8_t I2C_MasterWriteIT(I2C_Handle_t *const pI2CHandle, uint8_t SlaveAddr, uint8_t *pData, uint16_t Size);
uint8_t I2C_MasterReadIT(I2C_Handle_t *const pI2CHandle, uint8_t SlaveAddr, uint8_t *pData, uint16_t Size);

void I2C_SlaveWrite(I2C_RegDef_t *const pI2Cx, uint8_t Data);
uint8_t I2C_SlaveRead(I2C_RegDef_t *const pI2Cx);
void I2C_ControlSlaveIT(I2C_Handle_t *const pI2CHandle, uint8_t ITControl);

void I2C_EventsHandler(I2C_Handle_t *const pI2CHandle);
void I2C_ErrorsHandler(I2C_Handle_t *const pI2CHandle);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

#endif /* HAL_I2C_H_ */
