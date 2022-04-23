/*
 * hal_spi.h
 *
 *  Created on: 2 de fev de 2022
 *      Author: matheus
 */

#ifndef HAL_SPI_H_
#define HAL_SPI_H_

#include <stdint.h>
#include "stm32f103xx.h"

/* Expressions */
#define IS_SPI_REG_VALID(SPIx) 	((SPIx == SPI1) || \
								 (SPIx == SPI2) || \
								 (SPIx == SPI3) ? 1 : 0)


typedef enum
{
	SPI_MODE_SLAVE = 0,
	SPI_MODE_MASTER
} SPI_Mode_enum;

typedef enum
{
	SPI_BUS_FULL_DUPLEX = 0,
	SPI_BUS_HALF_DUPLEX,
	SPI_BUS_RX_ONLY
} SPI_BusConfig_enum;

typedef enum
{
	SPI_CLK_PHASE_1EDGE = 0,
	SPI_CLK_PHASE_2EDGE
} SPI_ClkPhase_enum;

typedef enum
{
	SPI_CLK_POL_LOW = 0,
	SPI_CLK_POL_HIGH
} SPI_ClkPol_enum;

typedef enum
{
	SPI_CLK_DIV_2 = 0,
	SPI_CLK_DIV_4,
	SPI_CLK_DIV_8,
	SPI_CLK_DIV_16,
	SPI_CLK_DIV_32,
	SPI_CLK_DIV_64,
	SPI_CLK_DIV_128,
	SPI_CLK_DIV_256,
} SPI_ClkDivider_enum;

typedef enum
{
	SPI_DATA_SIZE_8BITS = 0,
	SPI_DATA_SIZE_16BITS
} SPI_DataSize_enum;

typedef enum
{
	SPI_FIRST_BIT_MSB = 0,
	SPI_FIRST_BIT_LSB
} SPI_FirstBit_enum;

typedef enum
{
	SPI_SSM_HW_OUTPUT = 0,
	SPI_SSM_HW_INPUT,
	SPI_SSM_SW
} SPI_SSM_enum;

typedef enum
{
	SPI_COMM_READY = 0,
	SPI_TX_COMM_BUSY,
	SPI_RX_COMM_BUSY
} SPI_ITRState_enum;


typedef struct
{
	uint8_t Mode;
	uint8_t BusType;
	uint8_t SSM;
	uint8_t ClkPhase;
	uint8_t ClkPol;
	uint8_t ClkDivider;
	uint8_t DataSize;
	uint8_t FirstBit;
} SPI_DevConfig_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_DevConfig_t Config;
	uint8_t *TxBuffer;
	uint8_t TxCounter;
	uint8_t TxState;
	uint8_t *RxBuffer;
	uint8_t RxCounter;
	uint8_t RxState;
} SPI_Handle_t;


void SPI_Init(SPI_Handle_t *const pSPIHandle);

void SPI_Reset(SPI_RegDef_t *const pSPIx);

void SPI_EnableClock(SPI_RegDef_t *const pSPIx);
void SPI_DisableClock(SPI_RegDef_t *const pSPIx);

void SPI_StartCommunication(SPI_RegDef_t *const pSPIx);
void SPI_StopCommunication(SPI_RegDef_t *const pSPIx);

void SPI_Write(SPI_Handle_t *const pSPIHandle, uint8_t *pData, uint16_t Size);
void SPI_Transfer(SPI_Handle_t *const pSPIHandle, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
void SPI_Read(SPI_Handle_t *const pSPIHandle, uint8_t *pData, uint16_t Size);
uint8_t SPI_WriteITR(SPI_Handle_t *const pSPIHandle, uint8_t *pData, uint16_t Size);
uint8_t SPI_ReadITR(SPI_Handle_t *const pSPIHandle, uint8_t *pData, uint16_t Size);

void SPI_ITRHandler(SPI_Handle_t *const pSPIHandle);

#endif /* HAL_SPI_H_ */
