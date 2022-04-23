/*
 * hal_spi.c
 *
 *  Created on: 2 de fev de 2022
 *      Author: matheus
 */
#include <assert.h>
#include "hal_spi.h"
#include "utils_defs.h"

/* Local Functions Prototypes */
static uint8_t SPI_GetFlagStatus(SPI_RegDef_t *const pSPIx, uint8_t Flag);
static void SPI_WriteITRHandler(SPI_Handle_t *const pSPIHandle);
static void SPI_ReadITRHandler(SPI_Handle_t *const pSPIHandle);


void SPI_Init(SPI_Handle_t *const pSPIHandle)
{
	uint32_t RegVal = 0;

	/* Check the parameters */
	assert(IS_NOT_NULL(pSPIHandle));
	assert(IS_SPI_REG_VALID(pSPIHandle->pSPIx));

	/* Configure Bus Config Mode */
	if(pSPIHandle->Config.BusType == SPI_BUS_FULL_DUPLEX)
	{
		/* Configure 2-line unidirectional data mode */
		BIT_CLR(RegVal, SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->Config.BusType == SPI_BUS_HALF_DUPLEX)
	{
		/* Configure 1-line bidirectional data mode */
		BIT_SET(RegVal, SPI_CR1_BIDI_MODE);
	}
	else /* SPI_BUS_RX_ONLY */
	{
		/* Configure 2-line unidirectional data mode */
		BIT_CLR(RegVal, SPI_CR1_BIDI_MODE);
		/* Enable ONLY receiving */
		BIT_SET(RegVal, SPI_CR1_RX_ONLY);
	}

	/* Configure Device Mode (MSTR or Slave) */
	RegVal |= (uint32_t)(pSPIHandle->Config.Mode << SPI_CR1_MSTR);

	/* Configure Software Selection Management (SSM) */
	if(pSPIHandle->Config.SSM == SPI_SSM_SW)
	{
		BIT_SET(RegVal, SPI_CR1_SSM);
	}

	/* Configure Internal Slave Select (SSI) */
	if((pSPIHandle->Config.Mode == SPI_MODE_MASTER) &&
	   (pSPIHandle->Config.SSM == SPI_SSM_SW))
	{
		BIT_SET(RegVal, SPI_CR1_SSI);
	}

	/* Configure Slave Select Output Enable (SSOE) */
	if((pSPIHandle->Config.Mode == SPI_MODE_MASTER) &&
	   (pSPIHandle->Config.SSM == SPI_SSM_HW_OUTPUT))
	{
		BIT_SET(pSPIHandle->pSPIx->CR2, SPI_CR2_SSOE);
	}

	/* Configure Clock Phase */
	RegVal |= (uint32_t)(pSPIHandle->Config.ClkPhase << SPI_CR1_CPHA);

	/* Configure Clock Polarity */
	RegVal |= (uint32_t)(pSPIHandle->Config.ClkPol << SPI_CR1_CPOL);

	/* Configure Clock Divider */
	RegVal |= (uint32_t)(pSPIHandle->Config.ClkDivider << SPI_CR1_BR);

	/* Configure Data Size */
	RegVal |= (uint32_t)(pSPIHandle->Config.DataSize << SPI_CR1_DFF);

	/* Configure First Bit Mode */
	RegVal |= (uint32_t)(pSPIHandle->Config.FirstBit << SPI_CR1_LSB_FIRST);

	/* Put Configuration Data into SPI_CR1 */
	pSPIHandle->pSPIx->CR1 = RegVal;
}


void SPI_Reset(SPI_RegDef_t *const pSPIx)
{
	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIx));

	if(pSPIx == SPI1)
	{
		BIT_SET(RCC->APB2RSTR, RCC_APB2RSTR_SPI1_RST);
		BIT_CLR(RCC->APB2RSTR, RCC_APB2RSTR_SPI1_RST);
	}
	else if(pSPIx == SPI2)
	{
		BIT_SET(RCC->APB1RSTR, RCC_APB1RSTR_SPI2_RST);
		BIT_CLR(RCC->APB1RSTR, RCC_APB1RSTR_SPI2_RST);
	}
	else
	{
		BIT_SET(RCC->APB1RSTR, RCC_APB1RSTR_SPI3_RST);
		BIT_CLR(RCC->APB1RSTR, RCC_APB1RSTR_SPI3_RST);
	}
}


void SPI_EnableClock(SPI_RegDef_t *const pSPIx)
{
	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIx));

	if(pSPIx == SPI1)
	{
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_SPI1_EN);
	}
	else if(pSPIx == SPI2)
	{
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_SPI2_EN);
	}
	else
	{
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_SPI3_EN);
	}
}


void SPI_DisableClock(SPI_RegDef_t *const pSPIx)
{
	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIx));

	if(pSPIx == SPI1)
	{
		BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_SPI1_EN);
	}
	else if(pSPIx == SPI2)
	{
		BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_SPI2_EN);
	}
	else
	{
		BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_SPI3_EN);
	}
}


void SPI_StartCommunication(SPI_RegDef_t *const pSPIx)
{
	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIx));

	/* Enable SPI Peripheral */
	BIT_SET(pSPIx->CR1, SPI_CR1_SPE);
}


void SPI_StopCommunication(SPI_RegDef_t *const pSPIx)
{
	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIx));

	/* Wait until the peripheral is not busy. */
	while(!SPI_GetFlagStatus(pSPIx, SPI_SR_BSY));

	/* Disable SPI Peripheral */
	BIT_CLR(pSPIx->CR1, SPI_CR1_SPE);
}


void SPI_Write(SPI_Handle_t *const pSPIHandle, uint8_t *pData, uint16_t Size)
{
	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIHandle->pSPIx));
	assert(IS_NOT_NULL(pData));

	while(Size > 0)
	{
		/* Wait until TX Buffer to be empty */
		while(!SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_TXE));

		if(BIT_TST(pSPIHandle->pSPIx->CR1, SPI_CR1_DFF))
		{
			/* 16-bits data */
			pSPIHandle->pSPIx->DR = *((uint16_t *)pData);
			pData += sizeof(uint16_t);
			Size -= sizeof(uint16_t);
		}
		else
		{
			/* 8-bits data */
			pSPIHandle->pSPIx->DR = (uint16_t) *pData;
			pData++;
			Size--;
		}
	}
}


void SPI_Read(SPI_Handle_t *const pSPIHandle, uint8_t *pData, uint16_t Size)
{
	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIHandle->pSPIx));
	assert(IS_NOT_NULL(pData));

	if(pSPIHandle->Config.BusType == SPI_BUS_FULL_DUPLEX)
	{
		SPI_Transfer(pSPIHandle, pData, pData, Size);
	}
	else
	{
		while(Size > 0)
		{
			/* Wait until RX Buffer to be filled */
			while(!SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_RXNE));

			if(BIT_TST(pSPIHandle->pSPIx->CR1, SPI_CR1_DFF))
			{
				/* 16-bits data */
				*((uint16_t *)pData) = pSPIHandle->pSPIx->DR;
				pData += sizeof(uint16_t);
				Size -= sizeof(uint16_t);
			}
			else
			{
				/* 8-bits data */
				*pData = (uint8_t) pSPIHandle->pSPIx->DR;
				pData++;
				Size--;
			}
		}
	}
}

void SPI_Transfer(SPI_Handle_t *const pSPIHandle, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIHandle->pSPIx));
	assert(IS_NOT_NULL(pTxData));
	assert(IS_NOT_NULL(pRxData));

	while(Size > 0)
	{
		if(BIT_TST(pSPIHandle->pSPIx->CR1, SPI_CR1_DFF))
		{
			/* 16-bits data */
			/* Wait until TX Buffer to be empty */
			while(!SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_TXE));
			/* Sending Data */
			*pTxData = (uint16_t) pSPIHandle->pSPIx->DR;
			pTxData += sizeof(uint16_t);

			/* Wait until RX Buffer to be filled */
			while(!SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_RXNE));
			/* Receiving Data */
			*pRxData = (uint8_t) pSPIHandle->pSPIx->DR;
			pRxData += sizeof(uint16_t);

			Size -= sizeof(uint16_t);
		}
		else
		{
			/* 8-bits data */
			/* Wait until TX Buffer to be empty */
			while(!SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_TXE));
			/* Sending Data */
			*pTxData = (uint8_t) pSPIHandle->pSPIx->DR;
			pTxData++;

			/* Wait until RX Buffer to be filled */
			while(!SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_RXNE));
			/* Receiving Data */
			*pRxData = (uint8_t) pSPIHandle->pSPIx->DR;
			pRxData++;

			Size--;
		}
	}
}

uint8_t SPI_WriteITR(SPI_Handle_t *const pSPIHandle, uint8_t *pData, uint16_t Size)
{
	uint8_t CommState;

	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIHandle->pSPIx));
	assert(IS_NOT_NULL(pData));

	CommState = pSPIHandle->TxState;

	if(pSPIHandle->TxState == SPI_COMM_READY)
	{
		/* Initialize Tx context */
		pSPIHandle->TxBuffer = pData;
		pSPIHandle->TxCounter = Size;
		pSPIHandle->TxState = SPI_TX_COMM_BUSY;

		/* Enable SPI Tx Interruption */
		BIT_SET(pSPIHandle->pSPIx->CR2, SPI_CR2_TXEIE);
	}

	return CommState;
}

uint8_t SPI_ReadITR(SPI_Handle_t *const pSPIHandle, uint8_t *pData, uint16_t Size)
{
	uint8_t CommState;

	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIHandle->pSPIx));
	assert(IS_NOT_NULL(pData));

	CommState = pSPIHandle->RxState;

	if(pSPIHandle->RxState == SPI_COMM_READY)
	{
		/* Initialize Rx context */
		pSPIHandle->RxBuffer = pData;
		pSPIHandle->RxCounter = Size;
		pSPIHandle->RxState = SPI_RX_COMM_BUSY;

		/* Enable SPI Rx Interruption */
		BIT_SET(pSPIHandle->pSPIx->CR2, SPI_CR2_RXNEIE);
	}

	return CommState;
}

void SPI_ITRHandler(SPI_Handle_t *const pSPIHandle)
{
	/* Check the parameters */
	assert(IS_SPI_REG_VALID(pSPIHandle->pSPIx));

	/* Check if TX was the interruption source */
	if(BIT_TST(pSPIHandle->pSPIx->CR2, SPI_CR2_TXEIE) &&
	   SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_TXE))
	{
		SPI_WriteITRHandler(pSPIHandle);
	}

	/* Check if RX was the interruption source */
	if(BIT_TST(pSPIHandle->pSPIx->CR2, SPI_CR2_RXNEIE) &&
			SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_RXNE))
	{
		SPI_ReadITRHandler(pSPIHandle);
	}

	/* Check if an error was the interruption source */
	if(BIT_TST(pSPIHandle->pSPIx->CR2, SPI_CR2_ERRIE) &&
			SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_OVR))
	{
		/* TODO: Implement a function to handle this */
	}
}

/************************************
 *
 * Local Functions
 *
 ************************************/
static uint8_t SPI_GetFlagStatus(SPI_RegDef_t *const pSPIx, uint8_t Flag)
{
	return BIT_TST(pSPIx->SR, Flag);
}

static void SPI_WriteITRHandler(SPI_Handle_t *const pSPIHandle)
{
	/* Write 8-bits or 16-bits data */
	if(BIT_TST(pSPIHandle->pSPIx->CR1, SPI_CR1_DFF))
	{
		/* 16-bits data */
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->TxBuffer);
		pSPIHandle->TxBuffer += sizeof(uint16_t);
		pSPIHandle->TxCounter -= sizeof(uint16_t);
	}
	else
	{
		/* 8-bits data */
		pSPIHandle->pSPIx->DR = (uint16_t) *pSPIHandle->TxBuffer;
		pSPIHandle->TxBuffer++;
		pSPIHandle->TxBuffer--;
	}

	/* Check if communication was concluded */
	if(pSPIHandle->TxCounter == 0)
	{
		pSPIHandle->TxBuffer = NULL;
		pSPIHandle->TxState = SPI_COMM_READY;

		/* Disable TX interruption */
		BIT_CLR(pSPIHandle->pSPIx->CR2, SPI_CR2_TXEIE);
	}
}


static void SPI_ReadITRHandler(SPI_Handle_t *const pSPIHandle)
{
	/* Read 8-bits or 16-bits data */
	if(BIT_TST(pSPIHandle->pSPIx->CR1, SPI_CR1_DFF))
	{
		/* 16-bits data */
		*((uint16_t *)pSPIHandle->RxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxBuffer += sizeof(uint16_t);
		pSPIHandle->RxCounter -= sizeof(uint16_t);
	}
	else
	{
		/* 8-bits data */
		*pSPIHandle->RxBuffer = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxBuffer++;
		pSPIHandle->RxBuffer--;
	}

	/* Check if communication was concluded */
	if(pSPIHandle->RxCounter == 0)
	{
		pSPIHandle->RxBuffer = NULL;
		pSPIHandle->RxState = SPI_COMM_READY;

		/* Disable RX interruption */
		BIT_CLR(pSPIHandle->pSPIx->CR2, SPI_CR2_RXNEIE);
	}
}
